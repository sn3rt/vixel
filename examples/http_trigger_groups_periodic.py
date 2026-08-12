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


def api_request(
    base_url: str, path: str, timeout: float, body: dict[str, Any] | None = None
) -> dict[str, Any]:
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        data=None if body is None else json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="GET" if body is None else "POST",
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
        raise RuntimeError(str(result.get("message", "invalid API response")))
    return result


def set_group_capture_mode(
    base_url: str, group_id: str, timeout: float,
) -> dict[str, Any]:
    return api_request(
        base_url, "/api/v1/mode", timeout,
        {"target_kind": "group", "target_id": group_id, "mode": "capture"},
    )


def group_readiness(group: dict[str, Any] | None) -> str:
    if group is None:
        return "status unavailable"
    mode = str(group.get("operating_mode", "unknown"))
    if mode == "capture" and bool(group.get("ready", False)):
        return "ready"
    details = [f"mode={mode}"]
    members = list(group.get("member_ids", []))
    online = list(group.get("online_member_ids", []))
    if members:
        details.append(f"online={len(online)}/{len(members)}")
    missing = list(group.get("missing_member_ids", []))
    if missing:
        details.append(f"missing={','.join(map(str, missing))}")
    locking = list(group.get("locking_member_ids", []))
    if locking:
        details.append(f"PTP-locking={','.join(map(str, locking))}")
    if group.get("last_error"):
        details.append(f"notice={group['last_error']}")
    return " ".join(details)


def prepare_capture_groups(
    base_url: str, groups: list[str], timeout: float, ready_timeout: float,
) -> None:
    for group_id in groups:
        set_group_capture_mode(base_url, group_id, timeout)
    print(f"capture mode requested for: {','.join(groups)}")

    deadline = time.monotonic() + ready_timeout
    last_status = ""
    consecutive_ready_polls = 0
    while True:
        result = api_request(base_url, "/api/v1/groups", timeout)
        records = {
            str(group.get("group_id", "")): group
            for group in result.get("groups", [])
            if isinstance(group, dict)
        }
        states = {group_id: group_readiness(records.get(group_id)) for group_id in groups}
        all_ready = all(state == "ready" for state in states.values())
        consecutive_ready_polls = consecutive_ready_polls + 1 if all_ready else 0
        status = "; ".join(f"{group_id}={states[group_id]}" for group_id in groups)
        if status != last_status:
            print(f"waiting for capture readiness: {status}")
            last_status = status
        # Require two observations so the recorder has time to receive the same
        # transient-local group state before the first operation is submitted.
        if consecutive_ready_polls >= 2:
            print(f"capture groups ready: {','.join(groups)}")
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"groups did not become capture-ready within {ready_timeout:.1f} s: {status}"
            )
        time.sleep(0.25)


def start_save_sequence(
    base_url: str, groups: list[str], interval: float, count: int,
    request_prefix: str, timeout: float,
) -> dict[str, Any]:
    return api_request(
        base_url, "/api/v1/capture-operations/sequence", timeout,
        {
            "group_ids": groups,
            "request_id": request_prefix,
            "interval_ms": round(interval * 1000),
            "count": count,
            "synchronize_groups": True,
            "metadata": {"source": "http_trigger_groups_periodic.py"},
        },
    )


def monitor_save_sequence(
    base_url: str, operation_id: str, timeout: float,
) -> dict[str, Any]:
    last_progress = None
    encoded_id = urllib.parse.quote(operation_id, safe="")
    while True:
        result = api_request(
            base_url, f"/api/v1/capture-operations/{encoded_id}", timeout
        )
        operation = result["operation"]
        progress = (
            operation.get("scheduled_cycles", 0), operation.get("completed_cycles", 0),
            operation.get("failed_cycles", 0), operation.get("pending_saves", 0),
            operation.get("status", ""), operation.get("message", ""),
        )
        if progress != last_progress:
            detail = f" message={progress[5]}" if progress[5] else ""
            print(
                f"sequence {operation_id}: status={progress[4]} scheduled={progress[0]} "
                f"saved={progress[1]} failed={progress[2]} pending={progress[3]}{detail}"
            )
            last_progress = progress
        if operation.get("status") in {"complete", "failed", "cancelled"}:
            return operation
        time.sleep(0.25)


def cancel_save_sequence(base_url: str, operation_id: str, timeout: float) -> None:
    encoded_id = urllib.parse.quote(operation_id, safe="")
    api_request(
        base_url, f"/api/v1/capture-operations/{encoded_id}/cancel", timeout, {}
    )


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
        "--ready-timeout", type=float, default=60.0,
        help="Seconds to wait for cameras and PTP synchronization after requesting capture mode",
    )
    value.add_argument(
        "--request-prefix",
        default="",
        help="Capture ID prefix; default includes the current UTC time",
    )
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if (
        args.interval <= 0 or args.count < 0 or args.ready_timeout <= 0
        or len(set(args.group_ids)) != len(args.group_ids)
    ):
        print(
            "interval and ready-timeout must be positive, count non-negative, "
            "and groups unique", file=sys.stderr,
        )
        return 2
    sequence = 0
    next_deadline = time.monotonic()
    request_prefix = args.request_prefix or time.strftime(
        "periodic_%Y%m%dT%H%M%SZ", time.gmtime()
    )
    try:
        prepare_capture_groups(
            args.base_url, args.group_ids, args.timeout, args.ready_timeout
        )
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, ValueError, KeyError) as error:
        print(f"Could not prepare capture groups: {error}", file=sys.stderr)
        return 1
    if args.mode == "save":
        operation_id = ""
        try:
            accepted = start_save_sequence(
                args.base_url, args.group_ids, args.interval, args.count,
                request_prefix, args.timeout,
            )
            operation_id = str(accepted["operation_id"])
            print(
                f"sequence accepted: operation={operation_id} interval={args.interval:.3f} s "
                f"groups={','.join(args.group_ids)}"
            )
            operation = monitor_save_sequence(args.base_url, operation_id, args.timeout)
            return 0 if operation.get("status") == "complete" else 1
        except KeyboardInterrupt:
            if operation_id:
                try:
                    cancel_save_sequence(args.base_url, operation_id, args.timeout)
                    print(f"sequence {operation_id}: cancellation requested")
                except RuntimeError as error:
                    print(f"could not cancel sequence: {error}", file=sys.stderr)
            return 0
        except (RuntimeError, ValueError, KeyError) as error:
            print(f"Periodic trigger-and-save failed: {error}", file=sys.stderr)
            return 1
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
