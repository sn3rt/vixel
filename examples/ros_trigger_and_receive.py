#!/usr/bin/env python3
"""Trigger a Vixel group, receive its image_raw frames, and save PNGs."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys
import time
from collections import defaultdict
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from vixel_interfaces.action import TriggerGroup
from vixel_interfaces.msg import SyncGroupArray


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def stamp_key(stamp: Any) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]", "_", value)
    return cleaned or "capture"


class GroupImageClient(Node):
    def __init__(self, group_id: str) -> None:
        super().__init__("vixel_trigger_example")
        self.group_id = group_id
        self.group = None
        self.frames: dict[str, dict[tuple[int, int], Image]] = defaultdict(dict)
        self.image_subscriptions = {}
        self.bridge = CvBridge()
        self.group_subscription = self.create_subscription(
            SyncGroupArray, "/vixel/sync_groups", self._groups, STATE_QOS
        )
        self.trigger_client = ActionClient(self, TriggerGroup, "/vixel/trigger_group")

    def _groups(self, message: SyncGroupArray) -> None:
        self.group = next(
            (group for group in message.groups if group.group_id == self.group_id),
            None,
        )

    def subscribe_to_members(self) -> None:
        for sensor_id in self.group.member_ids:
            topic = f"/vixel/sensors/{sensor_id}/image_raw"
            self.image_subscriptions[sensor_id] = self.create_subscription(
                Image,
                topic,
                lambda message, member=sensor_id: self._image(member, message),
                qos_profile_sensor_data,
            )

    def _image(self, sensor_id: str, message: Image) -> None:
        frames = self.frames[sensor_id]
        frames[stamp_key(message.header.stamp)] = message
        while len(frames) > 8:
            del frames[next(iter(frames))]


def spin_until(node: Node, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not predicate():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f"timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))


def wait_future(node: Node, future, timeout: float, description: str):
    spin_until(node, future.done, timeout, description)
    exception = future.exception()
    if exception:
        raise RuntimeError(f"{description} failed: {exception}")
    return future.result()


def save_frames(
    node: GroupImageClient,
    output_root: pathlib.Path,
    capture_id: str,
    scheduled_time: Any,
    sensor_ids: list[str],
) -> pathlib.Path:
    destination = output_root / safe_name(capture_id)
    destination.mkdir(parents=True, exist_ok=False)
    key = stamp_key(scheduled_time)
    for sensor_id in sensor_ids:
        message = node.frames[sensor_id][key]
        image = node.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        path = destination / f"{safe_name(sensor_id)}.png"
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"OpenCV could not write {path}")
    return destination


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Trigger a Vixel group and save matching image_raw frames as PNG."
    )
    value.add_argument("group_id", help="Synchronization group ID, for example front")
    value.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("vixel-triggered-images"),
        help="Parent directory for capture output",
    )
    value.add_argument("--request-id", default="", help="Optional correlation ID")
    value.add_argument("--timeout", type=float, default=35.0, help="Timeout per stage")
    return value


def run(args: argparse.Namespace) -> pathlib.Path:
    node = GroupImageClient(args.group_id)
    try:
        spin_until(node, lambda: node.group is not None, args.timeout, "group state")
        if node.group.operating_mode != "capture":
            raise RuntimeError("group must be in capture mode")
        if node.group.trigger_source != "Action0":
            raise RuntimeError("group has not migrated to automatic Action0 capture")
        node.subscribe_to_members()
        spin_until(
            node,
            lambda: all(
                subscription.get_publisher_count() > 0
                for subscription in node.image_subscriptions.values()
            ),
            args.timeout,
            "image_raw publishers",
        )
        if not node.trigger_client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError("/vixel/trigger_group action is unavailable")

        goal = TriggerGroup.Goal()
        goal.group_id = args.group_id
        goal.request_id = args.request_id
        handle = wait_future(
            node,
            node.trigger_client.send_goal_async(goal),
            args.timeout,
            "trigger goal acceptance",
        )
        if not handle.accepted:
            raise RuntimeError("trigger goal was rejected")
        wrapped = wait_future(
            node, handle.get_result_async(), args.timeout, "trigger result"
        )
        result = wrapped.result
        if not result.success:
            raise RuntimeError(result.message or "group trigger failed")

        expected = list(result.participating_sensor_ids)
        key = stamp_key(result.scheduled_time)
        spin_until(
            node,
            lambda: all(key in node.frames[sensor_id] for sensor_id in expected),
            args.timeout,
            "triggered image_raw frames",
        )
        destination = save_frames(
            node, args.output_dir, result.capture_id, result.scheduled_time, expected
        )
        metadata = {
            "capture_id": result.capture_id,
            "scheduled_time": {"sec": key[0], "nanosec": key[1]},
            "participating_sensor_ids": expected,
            "missing_sensor_ids": list(result.missing_sensor_ids),
            "trigger_span_ns": int(result.trigger_span_ns),
            "exposure_skew_ns": int(result.exposure_skew_ns),
            "within_tolerance": bool(result.within_tolerance),
            "camera_timings": [
                {
                    "sensor_id": timing.sensor_id,
                    "device_timestamp_ns": int(timing.device_timestamp_ns),
                    "ptp_offset_ns": int(timing.ptp_offset_ns),
                    "synchronized": bool(timing.synchronized),
                }
                for timing in result.camera_timings
            ],
        }
        (destination / "trigger_result.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Saved {len(expected)} image(s) to {destination}")
        print(f"Exposure skew: {result.exposure_skew_ns} ns")
        if result.missing_sensor_ids:
            print(f"Missing: {', '.join(result.missing_sensor_ids)}")
        return destination
    finally:
        node.destroy_node()


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    rclpy.init()
    try:
        run(args)
        return 0
    except (RuntimeError, KeyError, FileExistsError, OSError) as error:
        print(f"Trigger failed: {error}", file=sys.stderr)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
