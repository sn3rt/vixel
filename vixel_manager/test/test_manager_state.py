from types import SimpleNamespace

from vixel_manager.manager_node import (
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
