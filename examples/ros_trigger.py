#!/usr/bin/env python3
"""Ask Vixel through ROS to trigger-and-publish or trigger-and-save a group."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vixel_interfaces.action import RecordCapture, TriggerGroup


class GroupTriggerClient(Node):
    def __init__(self) -> None:
        super().__init__("vixel_group_trigger_client")
        self.publish_client = ActionClient(self, TriggerGroup, "/vixel/trigger_group")
        self.save_client = ActionClient(self, RecordCapture, "/vixel/record_capture")


def wait_future(node: Node, future: Any, timeout: float, description: str) -> Any:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f"timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if not future.done():
        raise RuntimeError(f"ROS stopped while waiting for {description}")
    exception = future.exception()
    if exception:
        raise RuntimeError(f"{description} failed: {exception}")
    return future.result()


def send_goal(
    node: GroupTriggerClient,
    group_id: str,
    mode: str,
    request_id: str,
    timeout: float,
) -> Any:
    if mode == "save":
        client = node.save_client
        goal = RecordCapture.Goal()
        action_name = "/vixel/record_capture"
    elif mode == "publish":
        client = node.publish_client
        goal = TriggerGroup.Goal()
        action_name = "/vixel/trigger_group"
    else:
        raise ValueError(f"unsupported mode: {mode}")

    if not client.wait_for_server(timeout_sec=timeout):
        raise RuntimeError(f"{action_name} action is unavailable")
    goal.group_id = group_id
    goal.request_id = request_id
    handle = wait_future(
        node,
        client.send_goal_async(goal),
        timeout,
        f"{action_name} goal acceptance",
    )
    if not handle.accepted:
        raise RuntimeError(f"{action_name} goal was rejected")
    wrapped = wait_future(
        node, handle.get_result_async(), timeout, f"{action_name} result"
    )
    if not wrapped.result.success:
        raise RuntimeError(wrapped.result.message or f"trigger-and-{mode} failed")
    return wrapped.result


def print_result(result: Any, mode: str) -> None:
    missing = ", ".join(result.missing_sensor_ids) or "none"
    if mode == "save":
        saved = ", ".join(result.saved_sensor_ids) or "none"
        print(f"Saved capture ID: {result.capture_id}")
        print(f"Directory:        {result.directory}")
        print(f"Saved:            {saved}")
        print(f"Missing:          {missing}")
        return

    participating = ", ".join(result.participating_sensor_ids) or "none"
    print(f"Trigger ID:        {result.capture_id}")
    print(
        f"Scheduled time:    {result.scheduled_time.sec}."
        f"{result.scheduled_time.nanosec:09d}"
    )
    print(f"Published:         {participating}")
    print(f"Missing:           {missing}")
    print(f"Host trigger span: {result.trigger_span_ns} ns")
    print(f"Exposure skew:     {result.exposure_skew_ns} ns")
    print("Images were published on ROS image_raw topics and were not saved by Vixel.")


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Ask Vixel through ROS to publish or persist a triggered group."
    )
    value.add_argument("group_id", help="Synchronization group ID, for example front")
    value.add_argument(
        "--mode",
        choices=("publish", "save"),
        default="publish",
        help="Publish images on ROS topics or save them in Vixel's capture directory",
    )
    value.add_argument("--request-id", default="", help="Optional correlation ID")
    value.add_argument("--timeout", type=float, default=35.0, help="Timeout per action stage")
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    rclpy.init()
    node = GroupTriggerClient()
    try:
        result = send_goal(
            node, args.group_id, args.mode, args.request_id, args.timeout
        )
        print_result(result, args.mode)
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"Trigger-and-{args.mode} failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
