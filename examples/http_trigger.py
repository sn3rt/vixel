#!/usr/bin/env python3
"""Trigger a Vixel camera group through the loopback HTTP API."""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


def trigger_url(base_url: str, group_id: str) -> str:
    base = base_url.rstrip("/")
    group = urllib.parse.quote(group_id, safe="")
    return f"{base}/api/v1/groups/{group}/trigger"


def trigger_group(
    base_url: str, group_id: str, request_id: str = "", timeout: float = 35.0
) -> dict[str, Any]:
    body = json.dumps({"request_id": request_id}).encode("utf-8")
    request = urllib.request.Request(
        trigger_url(base_url, group_id),
        data=body,
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

    if not isinstance(result, dict):
        raise RuntimeError("Vixel API returned an unexpected response")
    if result.get("success") is False:
        raise RuntimeError(str(result.get("message", "group trigger failed")))
    return result


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Trigger a Vixel group through its HTTP API (no image download)."
    )
    value.add_argument("group_id", help="Synchronization group ID, for example front")
    value.add_argument(
        "--base-url", default="http://127.0.0.1:8080", help="Vixel gateway URL"
    )
    value.add_argument("--request-id", default="", help="Optional correlation ID")
    value.add_argument("--timeout", type=float, default=35.0, help="HTTP timeout in seconds")
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        result = trigger_group(
            args.base_url, args.group_id, args.request_id, args.timeout
        )
    except RuntimeError as error:
        print(f"Trigger failed: {error}", file=sys.stderr)
        return 1

    stamp = result.get("scheduled_time", {})
    participating = ", ".join(result.get("participating_sensor_ids", [])) or "none"
    missing = ", ".join(result.get("missing_sensor_ids", [])) or "none"
    print(f"Capture ID:       {result.get('capture_id', '')}")
    print(f"Scheduled time:   {stamp.get('sec', 0)}.{stamp.get('nanosec', 0):09d}")
    print(f"Published:        {participating}")
    print(f"Missing:          {missing}")
    print(f"Host trigger span: {result.get('trigger_span_ns', 0)} ns")
    print(f"Exposure skew:     {result.get('exposure_skew_ns', 0)} ns")
    print("Images were published on ROS image_raw topics; this HTTP client does not download them.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
