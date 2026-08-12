import asyncio
from types import SimpleNamespace

from vixel_manager.manager_node import (
    InventoryManager,
    _provider_group_is_current,
    _runtime_sensor_is_ready,
)


def test_provider_group_status_must_match_requested_mode():
    status = SimpleNamespace(
        operating_mode="preview", member_ids=["left", "right"]
    )

    assert not _provider_group_is_current(status, ["left", "right"], "capture")
    assert _provider_group_is_current(status, ["right", "left"], "preview")


def test_runtime_sensor_from_previous_mode_is_not_ready():
    sensor = SimpleNamespace(online=True, operating_mode="preview")

    assert not _runtime_sensor_is_ready(sensor, "capture")
    assert _runtime_sensor_is_ready(sensor, "preview")
    assert not _runtime_sensor_is_ready(None, "capture")


def test_group_capture_proxy_awaits_provider_without_blocking_executor_thread():
    provider_response = SimpleNamespace(success=True)
    captured = {}

    async def call_async(request):
        captured["request"] = request
        await asyncio.sleep(0)
        return provider_response

    client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: True,
        call_async=call_async,
    )
    manager = InventoryManager.__new__(InventoryManager)
    manager.registry = SimpleNamespace(inventory={
        "sync_groups": {
            "front": {
                "members": ["left", "right"],
                "missing_policy": "strict",
                "provider": "genicam",
            }
        }
    })
    manager.group_modes = {"front": "capture"}
    manager.default_group_mode = "idle"
    manager.capture_clients = {"genicam": client}
    manager._runtime_group_provider = lambda _group: "genicam"

    result = asyncio.run(manager._request_group_capture(
        "front", "capture_1", trigger_only=False
    ))

    assert result is provider_response
    assert captured["request"].request_id == "capture_1"
    assert captured["request"].member_ids == ["left", "right"]
