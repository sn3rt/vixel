#!/usr/bin/env python3
"""Trigger multiple Vixel groups concurrently at a fixed interval."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable


def trigger_group(
    base_url: str, group_id: str, request_id: str, timeout: float
) -> dict[str, Any]:
    group = urllib.parse.quote(group_id, safe="")
    url = f"{base_url.rstrip('/')}/api/v1/groups/{group}/trigger"
    request = urllib.request.Request(
        url,
        data=json.dumps({"request_id": request_id}).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            result = json.load(response)
    except urllib.error.HTTPError as error:
        try:
            detail = json.loads(error.read().decode("utf-8")).get("message", "")
        except (UnicodeDecodeError, json.JSONDecodeError):
            detail = ""
        raise RuntimeError(detail or f"HTTP {error.code}: {error.reason}") from error
    except urllib.error.URLError as error:
        raise RuntimeError(f"cannot reach Vixel API: {error.reason}") from error
    if not isinstance(result, dict) or result.get("success") is False:
        message = (
            result.get("message", "group trigger failed")
            if isinstance(result, dict)
            else "invalid API response"
        )
        raise RuntimeError(str(message))
    return result


def scheduled_ns(result: dict[str, Any]) -> int:
    stamp = result.get("scheduled_time", {})
    return int(stamp.get("sec", 0)) * 1_000_000_000 + int(stamp.get("nanosec", 0))


def trigger_cycle(
    groups: list[str], base_url: str, timeout: float, sequence: int,
    trigger: Callable[[str, str, str, float], dict[str, Any]] = trigger_group,
) -> dict[str, dict[str, Any]]:
    barrier = threading.Barrier(len(groups))

    def invoke(group_id: str) -> dict[str, Any]:
        barrier.wait()
        return trigger(base_url, group_id, f"periodic_{sequence}_{group_id}", timeout)

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(groups)) as executor:
        futures = {group: executor.submit(invoke, group) for group in groups}
        return {group: future.result() for group, future in futures.items()}


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Trigger multiple disjoint Vixel groups concurrently and periodically."
    )
    value.add_argument("group_ids", nargs="+", help="Groups, for example: front back")
    value.add_argument("--interval", type=float, default=2.0, help="Seconds between cycles")
    value.add_argument("--count", type=int, default=0, help="Cycles; 0 runs until Ctrl-C")
    value.add_argument("--base-url", default="http://127.0.0.1:8080")
    value.add_argument("--timeout", type=float, default=35.0)
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.interval <= 0 or args.count < 0 or len(set(args.group_ids)) != len(args.group_ids):
        print("interval must be positive, count non-negative, and groups unique", file=sys.stderr)
        return 2
    sequence = 0
    next_deadline = time.monotonic()
    try:
        while args.count == 0 or sequence < args.count:
            sequence += 1
            results = trigger_cycle(
                args.group_ids, args.base_url, args.timeout, sequence
            )
            times = [scheduled_ns(result) for result in results.values()]
            group_delta = max(times) - min(times) if times else 0
            print(f"cycle {sequence}: group scheduled-time delta={group_delta} ns")
            for group, result in results.items():
                timings = result.get("camera_timings", [])
                synchronized = sum(bool(item.get("synchronized")) for item in timings)
                print(
                    f"  {group}: capture={result.get('capture_id', '')} "
                    f"members={len(result.get('participating_sensor_ids', []))} "
                    f"ptp={synchronized} exposure_skew={result.get('exposure_skew_ns', 0)} ns"
                )
            next_deadline += args.interval
            time.sleep(max(0.0, next_deadline - time.monotonic()))
    except KeyboardInterrupt:
        return 0
    except RuntimeError as error:
        print(f"Periodic trigger failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
