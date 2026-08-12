import io
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

from vixel_web.gateway import (
    capture_operation_to_dict,
    capture_record_to_dict,
    group_to_dict,
    Handler,
    VixelHTTPServer,
    health_response,
    is_routine_snapshot_request,
    known_sensor_to_dict,
)


def test_group_ptp_locking_members_are_exposed_as_json():
    group = SimpleNamespace(
        group_id="front",
        provider="genicam",
        member_ids=["camera_a", "camera_b"],
        missing_policy="strict",
        trigger_source="Action0",
        operating_mode="capture",
        preview_rate_hz=2.0,
        preferred_master_id="",
        synchronization_method="ptp_relocking",
        ptp_ready=False,
        synchronized_member_ids=["camera_a"],
        locking_member_ids=["camera_b"],
        unsynchronized_member_ids=[],
        max_ptp_offset_ns=1200,
        ready=False,
        online_member_ids=["camera_a", "camera_b"],
        missing_member_ids=[],
        last_error="PTP lock not ready for: camera_b",
    )

    value = group_to_dict(group)

    assert value["locking_member_ids"] == ["camera_b"]
    assert value["ready"] is False
    assert value["synchronization_method"] == "ptp_relocking"


def test_capture_record_is_exposed_as_json():
    class Stamp:
        sec = 12
        nanosec = 34

    class Record:
        capture_id = "capture_test"
        group_id = "pair"
        status = "complete"
        directory = "/tmp/capture_test"
        message = "saved"
        started_at = "2026-07-28T10:00:00.000Z"
        completed_at = "2026-07-28T10:00:01.000Z"
        scheduled_time = Stamp()
        trigger_span_ns = 1200
        requested_sensor_ids = ["a", "b"]
        participating_sensor_ids = ["a", "b"]
        saved_sensor_ids = ["a", "b"]
        missing_sensor_ids = []

    value = capture_record_to_dict(Record())
    assert value["capture_id"] == "capture_test"
    assert value["scheduled_time"] == {"sec": 12, "nanosec": 34}
    assert value["saved_sensor_ids"] == ["a", "b"]
    assert value["trigger_span_ns"] == 1200


def test_capture_operation_is_exposed_as_json():
    stamp = SimpleNamespace(sec=12, nanosec=34)
    operation = SimpleNamespace(
        operation_id="operation_1", kind="sequence", status="running",
        message="capturing", group_ids=["front", "back"], requested_cycles=10,
        scheduled_cycles=4, completed_cycles=2, failed_cycles=0, pending_saves=4,
        capture_ids=["run_1_front"], first_scheduled_time=stamp,
        last_scheduled_time=stamp, interval_ms=500, synchronize_groups=True,
        metadata_json='{"job":"test"}',
    )

    value = capture_operation_to_dict(operation)

    assert value["interval_ms"] == 500
    assert value["group_ids"] == ["front", "back"]
    assert value["metadata"] == {"job": "test"}


def test_health_response_tracks_manager_age_and_counts():
    snapshot = {
        "sensors": [
            {"sensor_id": "camera_a", "enrolled": True, "online": True},
            {"sensor_id": "camera_b", "enrolled": True, "online": False},
            {"sensor_id": "candidate", "enrolled": False, "online": True},
        ]
    }
    frames = {"camera_a": (b"jpeg", 99.0, 1)}

    status, value = health_response(snapshot, 98.0, frames, now=100.0)
    assert status == 200
    assert value == {
        "status": "ok",
        "manager_state_age_ms": 2000,
        "sensor_count": 2,
        "online_sensor_count": 1,
        "preview_sensor_count": 1,
    }

    status, value = health_response(snapshot, 90.0, frames, now=100.0)
    assert status == 503
    assert value["status"] == "degraded"

    status, value = health_response(snapshot, 0.0, frames, now=100.0)
    assert status == 503
    assert value["status"] == "starting"
    assert value["manager_state_age_ms"] is None


class _FakeRequest:
    def __init__(self):
        self.response = b""

    def sendall(self, value):
        self.response += value


class _HTTPConnection:
    def __init__(self, request: bytes):
        self.input = io.BytesIO(request)
        self.response = bytearray()
        self.timeout = None

    def makefile(self, mode, _buffering=None):
        assert mode == "rb"
        return self.input

    def sendall(self, value):
        self.response.extend(value)

    def settimeout(self, value):
        self.timeout = value


class _Logger:
    def debug(self, *_args):
        pass

    def info(self, *_args):
        pass


def test_finite_http_response_closes_connection_immediately():
    node = SimpleNamespace(
        health=lambda: (200, {"status": "ok"}),
        get_logger=lambda: _Logger(),
    )
    server = SimpleNamespace(node=node)
    connection = _HTTPConnection(
        b"GET /api/v1/health HTTP/1.1\r\nHost: localhost\r\n\r\n"
    )

    handler = Handler(connection, ("127.0.0.1", 12345), server)

    response = bytes(connection.response)
    assert response.startswith(b"HTTP/1.1 200 OK\r\n")
    assert b"\r\nConnection: close\r\n" in response
    assert handler.close_connection
    assert connection.timeout == 40.0


def test_http_connection_limit_returns_503_without_starting_handler_thread():
    server = VixelHTTPServer.__new__(VixelHTTPServer)
    server._connection_slots = threading.BoundedSemaphore(1)
    assert server._connection_slots.acquire(blocking=False)
    request = _FakeRequest()
    closed = []
    server.shutdown_request = lambda value: closed.append(value)

    server.process_request(request, ("127.0.0.1", 12345))

    assert request.response.startswith(b"HTTP/1.1 503 Service Unavailable")
    assert b"connection limit" in request.response
    assert closed == [request]


def test_http_connection_slot_is_released_after_handler(monkeypatch):
    server = VixelHTTPServer.__new__(VixelHTTPServer)
    server._connection_slots = threading.BoundedSemaphore(1)
    assert server._connection_slots.acquire(blocking=False)
    monkeypatch.setattr(
        ThreadingHTTPServer,
        "process_request_thread",
        lambda _server, _request, _client_address: None,
    )

    server.process_request_thread(object(), ("127.0.0.1", 12345))

    assert server._connection_slots.acquire(blocking=False)


def test_only_successful_snapshot_polling_is_routine_traffic():
    path = "/api/v1/sensors/camera_ids_1/snapshot"
    assert is_routine_snapshot_request(path, 200)
    assert is_routine_snapshot_request(path, 304)
    assert is_routine_snapshot_request(path + ".jpg?cache=false", "200")
    assert not is_routine_snapshot_request(path, 404)
    assert not is_routine_snapshot_request("/api/v1/sensors", 200)
    assert not is_routine_snapshot_request(
        "/api/v1/sensors/camera_ids_1/stream", 200
    )


def test_known_sensor_json_fields_are_exposed_as_structured_values():
    sensor = SimpleNamespace(
        sensor_id="lucid_1", provider="lucid", kind="camera", vendor="LUCID",
        model="TRI032S-C", serial="1", mac_address="aa:bb", capabilities=["image"],
        catalog_state="archived", enrolled=False, online=False, managed=True,
        first_seen_at="2026-07-20T10:00:00+00:00",
        last_seen_at="2026-07-21T10:00:00+00:00", sighting_count=3,
        transport="gige", interface_name="enp7s0", current_address="192.168.2.11",
        network_id="camera_link_2", notes="spare",
        camera_profile="lucid_indoor",
        provider_settings_json='{"exposure_us":1200}',
        last_configuration_json='{"location_label":"front_left"}',
        changes_json='[{"timestamp":"2026-07-21T10:00:00+00:00"}]',
        replaced_by="",
    )

    value = known_sensor_to_dict(sensor)

    assert value["provider_settings"] == {"exposure_us": 1200}
    assert value["camera_profile"] == "lucid_indoor"
    assert value["last_configuration"]["location_label"] == "front_left"
    assert value["changes"][0]["timestamp"].startswith("2026-07-21")
