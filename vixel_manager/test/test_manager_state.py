import asyncio
import json
import threading
from types import SimpleNamespace

from rclpy.clock import Clock

from vixel_manager.manager_node import (
    InventoryManager,
    _provider_group_is_current,
    _runtime_sensor_is_ready,
)


def test_provider_group_status_must_match_requested_mode():
    status = SimpleNamespace(
        operating_mode="preview", member_ids=["left", "right"],
        requested_capture_interval_ms=0,
    )

    assert not _provider_group_is_current(status, ["left", "right"], "capture")
    assert _provider_group_is_current(status, ["right", "left"], "preview")
    assert not _provider_group_is_current(
        status, ["right", "left"], "preview", 200
    )


def test_provider_group_accepts_shared_faster_camera_cadence():
    status = SimpleNamespace(
        operating_mode="capture", member_ids=["left", "right"],
        requested_capture_interval_ms=250,
    )

    assert _provider_group_is_current(status, ["left", "right"], "capture", 500)
    assert not _provider_group_is_current(status, ["left", "right"], "capture", 200)


def test_idle_selection_accepts_cameras_active_for_an_overlapping_selection():
    status = SimpleNamespace(
        operating_mode="capture", member_ids=["left", "right"],
        requested_capture_interval_ms=250,
    )

    assert _provider_group_is_current(status, ["left", "right"], "idle", 0)


def test_overlapping_groups_aggregate_sensor_mode_and_cadence():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    manager = InventoryManager.__new__(InventoryManager)
    manager.registry = SimpleNamespace(
        inventory={
            "sync_groups": {
                "all": {"members": ["left", "right"], "missing_policy": "strict"},
                "front": {"members": ["left"], "missing_policy": "strict"},
                "back": {"members": ["right"], "missing_policy": "strict"},
            },
        },
        machine={
            "managed_networks": {
                "left_link": {"approved": True},
                "right_link": {"approved": True},
                "spare_link": {"approved": True},
            },
            "providers": {"genicam": {"imaging": {}}},
            "defaults": {"preview_rate_hz": 2.0},
        },
    )
    snapshot = {
        "sensors": {
            "left": {
                "provider": "genicam", "kind": "camera", "serial": "1",
                "network_id": "left_link", "enabled": True,
            },
            "right": {
                "provider": "genicam", "kind": "camera", "serial": "2",
                "network_id": "right_link", "enabled": True,
            },
            "spare": {
                "provider": "genicam", "kind": "camera", "serial": "3",
                "network_id": "spare_link", "enabled": True,
            },
        },
        "known_sensors": {},
        "sync_groups": manager.registry.inventory["sync_groups"],
    }
    manager.group_modes = {"all": "idle", "front": "capture", "back": "capture"}
    manager.group_capture_intervals = {"all": 0, "front": 250, "back": 500}
    manager.default_group_mode = "idle"
    manager.default_sensor_mode = "idle"
    manager.sensor_modes = {}
    manager.camera_profiles = {}
    manager.generation = 4
    manager._clock = Clock()
    publisher = Publisher()
    manager.assignment_publishers = {"genicam": publisher}
    manager._runtime_provider = lambda sensor: sensor["provider"]

    manager._publish_assignments(snapshot)

    assignments = {
        assignment.sensor_id: assignment
        for assignment in publisher.messages[0].assignments
    }
    assert assignments["left"].capture_group_ids == ["all", "front"]
    assert assignments["right"].capture_group_ids == ["all", "back"]
    assert assignments["left"].capture_group_missing_policies == ["strict", "strict"]
    assert assignments["left"].operating_mode == "capture"
    assert assignments["right"].operating_mode == "capture"
    assert assignments["left"].requested_capture_interval_ms == 250
    assert assignments["right"].requested_capture_interval_ms == 500
    assert manager._mode_for_sensor("left") == "capture"
    assert assignments["spare"].capture_group_ids == []
    assert assignments["spare"].operating_mode == "idle"
    assert json.loads(assignments["spare"].provider_settings_json)["trigger_source"] == "Action0"


def test_prepare_capture_groups_sets_mode_and_requested_interval_atomically():
    manager = InventoryManager.__new__(InventoryManager)
    manager.registry = SimpleNamespace(inventory={
        "sync_groups": {"front": {}, "back": {}}
    })
    manager.group_modes = {"front": "idle", "back": "preview"}
    manager.group_capture_intervals = {"front": 0, "back": 0}
    manager.generation = 7
    manager.lock = threading.RLock()
    published = []
    manager._publish_state = lambda: published.append(True)
    request = SimpleNamespace(group_ids=["front", "back"], interval_ms=200)
    response = SimpleNamespace(accepted=False, message="")

    result = manager._prepare_capture_groups(request, response)

    assert result.accepted is True
    assert manager.group_modes == {"front": "capture", "back": "capture"}
    assert manager.group_capture_intervals == {"front": 200, "back": 200}
    assert manager.generation == 8
    assert published == [True]


def test_runtime_sensor_from_previous_mode_is_not_ready():
    sensor = SimpleNamespace(online=True, operating_mode="preview")

    assert not _runtime_sensor_is_ready(sensor, "capture")
    assert _runtime_sensor_is_ready(sensor, "preview")
    assert not _runtime_sensor_is_ready(None, "capture")


def test_unapproved_network_is_visible_but_not_managed():
    manager = InventoryManager.__new__(InventoryManager)
    manager.registry = SimpleNamespace(machine={
        "managed_networks": {
            "spare": {
                "interface": "enp8s0",
                "host_cidr": "192.168.8.1/24",
                "approved": False,
            }
        }
    })
    observation = {
        "interface_name": "enp8s0",
        "current_address": "192.168.8.20",
    }

    assert manager._configured_network_for_observation(observation) == "spare"
    assert manager._network_for_observation(observation) == ""


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
