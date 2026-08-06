#!/usr/bin/env python3
"""Trigger multiple Vixel groups concurrently and publish or save each cycle."""

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


def request_group(
    base_url: str, group_id: str, request_id: str, timeout: float, mode: str
) -> dict[str, Any]:
    if mode not in {"publish", "save"}:
        raise ValueError(f"unsupported mode: {mode}")
    group = urllib.parse.quote(group_id, safe="")
    operation = "trigger" if mode == "publish" else "capture"
    url = f"{base_url.rstrip('/')}/api/v1/groups/{group}/{operation}"
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
    except json.JSONDecodeError as error:
        raise RuntimeError("Vixel API returned invalid JSON") from error
    if not isinstance(result, dict) or result.get("success") is False:
        message = (
            result.get("message", f"group {mode} failed")
            if isinstance(result, dict)
            else "invalid API response"
        )
        raise RuntimeError(str(message))
    return result


def trigger_group(
    base_url: str, group_id: str, request_id: str, timeout: float
) -> dict[str, Any]:
    """Backward-compatible trigger-and-publish helper."""
    return request_group(base_url, group_id, request_id, timeout, "publish")


def scheduled_ns(result: dict[str, Any]) -> int:
    stamp = result.get("scheduled_time", {})
    return int(stamp.get("sec", 0)) * 1_000_000_000 + int(stamp.get("nanosec", 0))


def trigger_cycle(
    groups: list[str], base_url: str, timeout: float, sequence: int,
    mode: str = "publish",
    request: Callable[[str, str, str, float, str], dict[str, Any]] = request_group,
    request_prefix: str = "periodic",
) -> dict[str, dict[str, Any]]:
    barrier = threading.Barrier(len(groups))

    def invoke(group_id: str) -> dict[str, Any]:
        barrier.wait()
        return request(
            base_url, group_id, f"{request_prefix}_{sequence}_{group_id}", timeout, mode
        )

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(groups)) as executor:
        futures = {group: executor.submit(invoke, group) for group in groups}
        return {group: future.result() for group, future in futures.items()}


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Trigger multiple disjoint Vixel groups concurrently and periodically."
    )
    value.add_argument("group_ids", nargs="+", help="Groups, for example: front back")
    value.add_argument(
        "--mode",
        choices=("publish", "save"),
        default="publish",
        help="Publish each triggered set or save separate group capture directories",
    )
    value.add_argument("--interval", type=float, default=2.0, help="Seconds between cycles")
    value.add_argument("--count", type=int, default=0, help="Cycles; 0 runs until Ctrl-C")
    value.add_argument("--base-url", default="http://127.0.0.1:8080")
    value.add_argument("--timeout", type=float, default=35.0)
    value.add_argument(
        "--request-prefix",
        default="",
        help="Capture ID prefix; default includes the current UTC time",
    )
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.interval <= 0 or args.count < 0 or len(set(args.group_ids)) != len(args.group_ids):
        print("interval must be positive, count non-negative, and groups unique", file=sys.stderr)
        return 2
    sequence = 0
    next_deadline = time.monotonic()
    request_prefix = args.request_prefix or time.strftime(
        "periodic_%Y%m%dT%H%M%SZ", time.gmtime()
    )
    try:
        while args.count == 0 or sequence < args.count:
            sequence += 1
            results = trigger_cycle(
                args.group_ids, args.base_url, args.timeout, sequence, args.mode,
                request_group, request_prefix,
            )
            if args.mode == "save":
                print(f"cycle {sequence}: saved {len(results)} group(s)")
                for group, result in results.items():
                    print(
                        f"  {group}: capture={result.get('capture_id', '')} "
                        f"saved={len(result.get('saved_sensor_ids', []))} "
                        f"directory={result.get('directory', '')}"
                    )
            else:
                times = [scheduled_ns(result) for result in results.values()]
                group_delta = max(times) - min(times) if times else 0
                print(f"cycle {sequence}: group scheduled-time delta={group_delta} ns")
                for group, result in results.items():
                    timings = result.get("camera_timings", [])
                    synchronized = sum(bool(item.get("synchronized")) for item in timings)
                    print(
                        f"  {group}: trigger={result.get('capture_id', '')} "
                        f"members={len(result.get('participating_sensor_ids', []))} "
                        f"ptp={synchronized} "
                        f"exposure_skew={result.get('exposure_skew_ns', 0)} ns"
                    )
            next_deadline += args.interval
            remaining = next_deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                print(
                    f"cycle {sequence}: interval overrun={-remaining:.3f} s",
                    file=sys.stderr,
                )
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"Periodic trigger-and-{args.mode} failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
