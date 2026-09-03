from __future__ import annotations

import json
import os
import pathlib
import socket
import threading
import time
import urllib.parse
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from vixel_interfaces.action import (
    EnrollSensor,
    RecordCapture,
    ResolveSensorPlacement,
    TriggerGroup,
)
from vixel_interfaces.msg import (
    CaptureOperationArray,
    CaptureRecordArray,
    KnownSensorArray,
    ManagedPortArray,
    SensorArray,
    SyncGroupArray,
)
from vixel_interfaces.srv import (
    CancelCaptureOperation,
    DeleteSyncGroup,
    ForgetSensor,
    GetCaptureOperation,
    PrepareCaptureGroups,
    PurgeKnownSensor,
    SetOperatingMode,
    SetPortMode,
    StartCaptureSequence,
    SubmitCaptureBatch,
    UpdateSensorMetadata,
    UpdateKnownSensor,
    UpsertSyncGroup,
)


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def recording_runtime_settings(machine: dict[str, Any]) -> dict[str, int | float]:
    recording = machine.get("recording") or {}
    if not isinstance(recording, dict):
        raise RuntimeError("recording configuration must be a mapping")
    try:
        capture_timeout_ms = int(recording.get("capture_timeout_ms", 10000))
        operation_history_limit = int(recording.get("operation_history_limit", 100))
        max_active_operations = int(recording.get("max_active_operations", 64))
    except (TypeError, ValueError) as error:
        raise RuntimeError("recording limits must be integers") from error
    if not 1000 <= capture_timeout_ms <= 60000:
        raise RuntimeError("recording.capture_timeout_ms must be between 1000 and 60000")
    if not 1 <= operation_history_limit <= 1000:
        raise RuntimeError(
            "recording.operation_history_limit must be between 1 and 1000"
        )
    if not 1 <= max_active_operations <= 256:
        raise RuntimeError(
            "recording.max_active_operations must be between 1 and 256"
        )
    return {
        "capture_result_timeout_sec": (2 * capture_timeout_ms + 20000) / 1000.0,
        "operation_history_limit": operation_history_limit,
        "max_active_operations": max_active_operations,
    }


def load_recording_runtime_settings(path: str) -> dict[str, int | float]:
    try:
        with open(path, "r", encoding="utf-8") as stream:
            machine = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"cannot load machine configuration {path}: {error}") from error
    if not isinstance(machine, dict):
        raise RuntimeError(f"machine configuration {path} must contain a mapping")
    return recording_runtime_settings(machine)


def is_routine_snapshot_request(path: str, status: int | str) -> bool:
    try:
        status_code = int(status)
    except (TypeError, ValueError):
        return False
    parts = [part for part in urllib.parse.urlparse(path).path.split("/") if part]
    return (
        status_code in {HTTPStatus.OK, HTTPStatus.NOT_MODIFIED}
        and len(parts) == 5
        and parts[:3] == ["api", "v1", "sensors"]
        and parts[4] in {"snapshot", "snapshot.jpg", "snapshot.png"}
    )


def health_response(
    snapshot: dict[str, Any],
    last_state_at: float,
    frames: dict[str, tuple[bytes, float, int] | tuple[bytes, float, int, str]],
    *,
    now: float | None = None,
    state_timeout_sec: float = 5.0,
) -> tuple[int, dict[str, Any]]:
    now = time.monotonic() if now is None else now
    age = None if last_state_at <= 0.0 else max(0.0, now - last_state_at)
    healthy = age is not None and age <= state_timeout_sec
    sensors = snapshot.get("sensors", [])
    enrolled = [sensor for sensor in sensors if sensor.get("enrolled")]
    value = {
        "status": "ok" if healthy else ("starting" if age is None else "degraded"),
        "manager_state_age_ms": None if age is None else int(age * 1000),
        "sensor_count": len(enrolled),
        "online_sensor_count": sum(bool(sensor.get("online")) for sensor in enrolled),
        "preview_sensor_count": sum(sensor.get("sensor_id") in frames for sensor in enrolled),
    }
    return (HTTPStatus.OK if healthy else HTTPStatus.SERVICE_UNAVAILABLE), value


def sensor_to_dict(sensor) -> dict[str, Any]:
    return {
        "sensor_id": sensor.sensor_id,
        "provider": sensor.provider,
        "kind": sensor.kind,
        "vendor": sensor.vendor,
        "model": sensor.model,
        "serial": sensor.serial,
        "mac_address": sensor.mac_address,
        "enrolled": sensor.enrolled,
        "enabled": sensor.enabled,
        "online": sensor.online,
        "lifecycle_state": sensor.lifecycle_state,
        "display_name": sensor.display_name,
        "location_label": sensor.location_label,
        "has_pose": sensor.has_pose,
        "parent_frame": sensor.parent_frame,
        "pose": {
            "position": {
                "x": sensor.pose.position.x,
                "y": sensor.pose.position.y,
                "z": sensor.pose.position.z,
            },
            "orientation": {
                "x": sensor.pose.orientation.x,
                "y": sensor.pose.orientation.y,
                "z": sensor.pose.orientation.z,
                "w": sensor.pose.orientation.w,
            },
        },
        "calibration_url": sensor.calibration_url,
        "topic_base": sensor.topic_base,
        "network_id": sensor.network_id,
        "assigned_address": sensor.assigned_address,
        "current_address": sensor.current_address,
        "interface_name": sensor.interface_name,
        "sync_group": sensor.sync_group,
        "capture_group_ids": list(getattr(sensor, "capture_group_ids", [])),
        "operating_mode": sensor.operating_mode,
        "capabilities": list(sensor.capabilities),
        "last_error": sensor.last_error,
        "managed": sensor.managed,
        "pending_action": sensor.pending_action,
        "status_detail": sensor.status_detail,
        "applied_settings": _json_value(sensor.applied_settings_json, {}),
    }


def _json_value(value: str, default):
    try:
        decoded = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return default
    return decoded


def known_sensor_to_dict(sensor) -> dict[str, Any]:
    return {
        "sensor_id": sensor.sensor_id,
        "provider": sensor.provider,
        "kind": sensor.kind,
        "vendor": sensor.vendor,
        "model": sensor.model,
        "serial": sensor.serial,
        "mac_address": sensor.mac_address,
        "capabilities": list(sensor.capabilities),
        "catalog_state": sensor.catalog_state,
        "enrolled": sensor.enrolled,
        "online": sensor.online,
        "managed": sensor.managed,
        "first_seen_at": sensor.first_seen_at,
        "last_seen_at": sensor.last_seen_at,
        "sighting_count": int(sensor.sighting_count),
        "transport": sensor.transport,
        "interface_name": sensor.interface_name,
        "current_address": sensor.current_address,
        "network_id": sensor.network_id,
        "notes": sensor.notes,
        "camera_profile": getattr(sensor, "camera_profile", ""),
        "provider_settings": _json_value(sensor.provider_settings_json, {}),
        "last_configuration": _json_value(sensor.last_configuration_json, None),
        "changes": _json_value(sensor.changes_json, []),
        "replaced_by": sensor.replaced_by,
    }


def group_to_dict(group) -> dict[str, Any]:
    return {
        "group_id": group.group_id,
        "provider": group.provider,
        "member_ids": list(group.member_ids),
        "missing_policy": group.missing_policy,
        "trigger_source": group.trigger_source,
        "operating_mode": group.operating_mode,
        "preview_rate_hz": group.preview_rate_hz,
        "preferred_master_id": group.preferred_master_id,
        "synchronization_method": group.synchronization_method,
        "ptp_ready": group.ptp_ready,
        "synchronized_member_ids": list(group.synchronized_member_ids),
        "locking_member_ids": list(group.locking_member_ids),
        "unsynchronized_member_ids": list(group.unsynchronized_member_ids),
        "max_ptp_offset_ns": int(group.max_ptp_offset_ns),
        "requested_capture_interval_ms": int(
            getattr(group, "requested_capture_interval_ms", 0)
        ),
        "cadence_configured": bool(getattr(group, "cadence_configured", False)),
        "cadence_ready": bool(getattr(group, "cadence_ready", False)),
        "minimum_capture_interval_ms": int(
            getattr(group, "minimum_capture_interval_ms", 0)
        ),
        "maximum_capture_rate_hz": float(
            getattr(group, "maximum_capture_rate_hz", 0.0)
        ),
        "action_queue_size": int(getattr(group, "action_queue_size", 0)),
        "cadence_limit_reason": getattr(group, "cadence_limit_reason", ""),
        "ready": group.ready,
        "online_member_ids": list(group.online_member_ids),
        "missing_member_ids": list(group.missing_member_ids),
        "last_error": group.last_error,
    }


def port_to_dict(port) -> dict[str, Any]:
    return {
        "network_id": port.network_id,
        "interface_name": port.interface_name,
        "interface_mac": port.interface_mac,
        "mode": port.mode,
        "approved": port.approved,
        "auto_enroll": port.auto_enroll,
        "configured": port.configured,
        "link_up": port.link_up,
        "host_cidr": port.host_cidr,
        "capacity": port.capacity,
        "enrolled_sensor_ids": list(port.enrolled_sensor_ids),
        "discovered_candidate_ids": list(port.discovered_candidate_ids),
        "lifecycle_state": port.lifecycle_state,
        "last_error": port.last_error,
    }


def capture_record_to_dict(record) -> dict[str, Any]:
    return {
        "capture_id": record.capture_id,
        "operation_id": getattr(record, "operation_id", ""),
        "cycle": int(getattr(record, "cycle", 0)),
        "metadata": _json_value(getattr(record, "metadata_json", ""), {}),
        "capture_timings": _json_value(getattr(record, "timings_json", ""), {}),
        "group_id": record.group_id,
        "status": record.status,
        "directory": record.directory,
        "message": record.message,
        "started_at": record.started_at,
        "completed_at": record.completed_at,
        "scheduled_time": {
            "sec": record.scheduled_time.sec,
            "nanosec": record.scheduled_time.nanosec,
        },
        "trigger_span_ns": int(record.trigger_span_ns),
        "exposure_skew_ns": int(getattr(record, "exposure_skew_ns", 0)),
        "within_tolerance": bool(getattr(record, "within_tolerance", False)),
        "camera_timings": [
            {
                "sensor_id": timing.sensor_id,
                "device_timestamp_ns": int(timing.device_timestamp_ns),
                "ptp_offset_ns": int(timing.ptp_offset_ns),
                "synchronized": timing.synchronized,
            }
            for timing in getattr(record, "camera_timings", [])
        ],
        "requested_sensor_ids": list(record.requested_sensor_ids),
        "participating_sensor_ids": list(record.participating_sensor_ids),
        "saved_sensor_ids": list(record.saved_sensor_ids),
        "missing_sensor_ids": list(record.missing_sensor_ids),
    }


def capture_operation_to_dict(operation) -> dict[str, Any]:
    return {
        "operation_id": operation.operation_id,
        "kind": operation.kind,
        "status": operation.status,
        "message": operation.message,
        "group_ids": list(operation.group_ids),
        "requested_cycles": int(operation.requested_cycles),
        "scheduled_cycles": int(operation.scheduled_cycles),
        "completed_cycles": int(operation.completed_cycles),
        "failed_cycles": int(operation.failed_cycles),
        "pending_saves": int(operation.pending_saves),
        "capture_ids": list(operation.capture_ids),
        "capture_id_count": int(getattr(operation, "capture_id_count", 0)),
        "capture_ids_truncated": bool(
            getattr(operation, "capture_ids_truncated", False)
        ),
        "first_scheduled_time": {
            "sec": operation.first_scheduled_time.sec,
            "nanosec": operation.first_scheduled_time.nanosec,
        },
        "last_scheduled_time": {
            "sec": operation.last_scheduled_time.sec,
            "nanosec": operation.last_scheduled_time.nanosec,
        },
        "interval_ms": int(operation.interval_ms),
        "synchronize_groups": bool(operation.synchronize_groups),
        "metadata": _json_value(operation.metadata_json, {}),
    }


class GatewayNode(Node):
    def __init__(self):
        super().__init__("web_gateway", namespace="/vixel")
        self.callback_group = ReentrantCallbackGroup()
        self.address = str(self.declare_parameter("address", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 8080).value)
        self.state_timeout_sec = max(
            1.0, float(self.declare_parameter("state_timeout_sec", 5.0).value)
        )
        self.max_http_connections = max(
            4, int(self.declare_parameter("max_http_connections", 64).value)
        )
        self.machine_file = str(
            self.declare_parameter("machine_file", "/etc/vixel/machine.yaml").value
        )
        recording_settings = load_recording_runtime_settings(self.machine_file)
        self.capture_result_timeout_sec = float(
            recording_settings["capture_result_timeout_sec"]
        )
        requested_inventory = str(
            self.declare_parameter("inventory_file", "/var/lib/vixel/inventory.yaml").value
        )
        self.inventory_file = self._resolved_inventory_path(requested_inventory)
        self.static_file = pathlib.Path(get_package_share_directory("vixel_web")) / "static/index.html"
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.generation = 0
        self.last_state_at = 0.0
        self.sensors: dict[str, dict[str, Any]] = {}
        self.known_sensors: dict[str, dict[str, Any]] = {}
        self.groups: dict[str, dict[str, Any]] = {}
        self.ports: dict[str, dict[str, Any]] = {}
        self.operations: dict[str, dict[str, Any]] = {}
        self.operation_history_limit = int(
            recording_settings["operation_history_limit"]
        )
        self.max_active_operations = int(recording_settings["max_active_operations"])
        self.capture_operations: dict[str, dict[str, Any]] = {}
        self.capture_records: list[dict[str, Any]] = []
        self.frames: dict[
            str, tuple[bytes, float, int] | tuple[bytes, float, int, str]
        ] = {}
        self.image_subscriptions = {}
        self.sensor_subscription = self.create_subscription(
            SensorArray, "/vixel/sensors", self._sensors, STATE_QOS,
            callback_group=self.callback_group
        )
        self.known_sensor_subscription = self.create_subscription(
            KnownSensorArray, "/vixel/known_sensors", self._known_sensors, STATE_QOS,
            callback_group=self.callback_group
        )
        self.group_subscription = self.create_subscription(
            SyncGroupArray, "/vixel/sync_groups", self._groups, STATE_QOS,
            callback_group=self.callback_group
        )
        self.port_subscription = self.create_subscription(
            ManagedPortArray, "/vixel/ports", self._ports, STATE_QOS,
            callback_group=self.callback_group
        )
        self.capture_records_subscription = self.create_subscription(
            CaptureRecordArray, "/vixel/capture_records", self._capture_records, STATE_QOS,
            callback_group=self.callback_group
        )
        self.capture_operations_subscription = self.create_subscription(
            CaptureOperationArray, "/vixel/capture_operations", self._capture_operations,
            STATE_QOS, callback_group=self.callback_group
        )
        self.enroll_client = ActionClient(
            self, EnrollSensor, "/vixel/enroll_sensor", callback_group=self.callback_group
        )
        self.resolve_client = ActionClient(
            self, ResolveSensorPlacement, "/vixel/resolve_sensor_placement",
            callback_group=self.callback_group
        )
        self.update_client = self.create_client(
            UpdateSensorMetadata, "/vixel/update_sensor_metadata",
            callback_group=self.callback_group
        )
        self.forget_client = self.create_client(
            ForgetSensor, "/vixel/forget_sensor", callback_group=self.callback_group
        )
        self.update_known_client = self.create_client(
            UpdateKnownSensor, "/vixel/update_known_sensor", callback_group=self.callback_group
        )
        self.purge_known_client = self.create_client(
            PurgeKnownSensor, "/vixel/purge_known_sensor", callback_group=self.callback_group
        )
        self.mode_client = self.create_client(
            SetOperatingMode, "/vixel/set_operating_mode", callback_group=self.callback_group
        )
        self.prepare_capture_groups_client = self.create_client(
            PrepareCaptureGroups, "/vixel/prepare_capture_groups",
            callback_group=self.callback_group
        )
        self.port_mode_client = self.create_client(
            SetPortMode, "/vixel/set_port_mode", callback_group=self.callback_group
        )
        self.group_client = self.create_client(
            UpsertSyncGroup, "/vixel/upsert_sync_group", callback_group=self.callback_group
        )
        self.delete_group_client = self.create_client(
            DeleteSyncGroup, "/vixel/delete_sync_group", callback_group=self.callback_group
        )
        self.record_capture_client = ActionClient(
            self, RecordCapture, "/vixel/record_capture", callback_group=self.callback_group
        )
        self.submit_capture_batch_client = self.create_client(
            SubmitCaptureBatch, "/vixel/submit_capture_batch",
            callback_group=self.callback_group
        )
        self.start_capture_sequence_client = self.create_client(
            StartCaptureSequence, "/vixel/start_capture_sequence",
            callback_group=self.callback_group
        )
        self.get_capture_operation_client = self.create_client(
            GetCaptureOperation, "/vixel/get_capture_operation",
            callback_group=self.callback_group
        )
        self.cancel_capture_operation_client = self.create_client(
            CancelCaptureOperation, "/vixel/cancel_capture_operation",
            callback_group=self.callback_group
        )
        self.trigger_group_client = ActionClient(
            self, TriggerGroup, "/vixel/trigger_group", callback_group=self.callback_group
        )

    @staticmethod
    def _resolved_inventory_path(requested: str) -> str:
        path = pathlib.Path(requested).expanduser()
        writable_parent = next((parent for parent in [path.parent, *path.parents]
                                if parent.exists()), None)
        if path.exists() or (writable_parent and os.access(writable_parent, os.W_OK)):
            return str(path)
        state_home = pathlib.Path(
            os.environ.get("XDG_STATE_HOME", pathlib.Path.home() / ".local/state")
        )
        return str(state_home / "vixel/inventory.yaml")

    def _sensors(self, message: SensorArray):
        with self.changed:
            self.last_state_at = time.monotonic()
            self.generation = max(self.generation + 1, int(message.generation))
            self.sensors = {sensor.sensor_id: sensor_to_dict(sensor) for sensor in message.sensors}
            wanted = {
                sensor.sensor_id: sensor.topic_base + "/image_raw/compressed"
                for sensor in message.sensors
                if sensor.enrolled
                and any(
                    capability in sensor.capabilities
                    for capability in ("compressed_preview", "png_preview", "jpeg_preview")
                )
                and sensor.topic_base
            }
            for sensor_id in list(self.image_subscriptions):
                if sensor_id not in wanted:
                    self.destroy_subscription(self.image_subscriptions.pop(sensor_id))
                    self.frames.pop(sensor_id, None)
            for sensor_id, topic in wanted.items():
                if sensor_id not in self.image_subscriptions:
                    self.image_subscriptions[sensor_id] = self.create_subscription(
                        CompressedImage,
                        topic,
                        lambda frame, identity=sensor_id: self._frame(identity, frame),
                        rclpy.qos.qos_profile_sensor_data,
                        callback_group=self.callback_group,
                    )
            self.changed.notify_all()

    def _groups(self, message: SyncGroupArray):
        with self.changed:
            self.generation = max(self.generation + 1, int(message.generation))
            self.groups = {group.group_id: group_to_dict(group) for group in message.groups}
            self.changed.notify_all()

    def _known_sensors(self, message: KnownSensorArray):
        with self.changed:
            self.generation = max(self.generation + 1, int(message.generation))
            self.known_sensors = {
                sensor.sensor_id: known_sensor_to_dict(sensor) for sensor in message.sensors
            }
            self.changed.notify_all()

    def _ports(self, message: ManagedPortArray):
        with self.changed:
            self.generation = max(self.generation + 1, int(message.generation))
            self.ports = {port.network_id: port_to_dict(port) for port in message.ports}
            self.changed.notify_all()

    def _capture_records(self, message: CaptureRecordArray):
        with self.changed:
            self.generation = max(self.generation + 1, int(message.generation))
            self.capture_records = [
                capture_record_to_dict(record) for record in message.records
            ]
            self.changed.notify_all()

    def _capture_operations(self, message: CaptureOperationArray):
        with self.changed:
            self.generation = max(self.generation + 1, int(message.generation))
            self.capture_operations = {
                operation.operation_id: capture_operation_to_dict(operation)
                for operation in message.operations
            }
            self.changed.notify_all()

    def _frame(self, sensor_id: str, message: CompressedImage):
        image_format = message.format.lower()
        if "png" in image_format:
            content_type = "image/png"
        elif "jpeg" in image_format or "jpg" in image_format:
            content_type = "image/jpeg"
        else:
            self.get_logger().warning(
                f"Ignoring unsupported compressed preview from {sensor_id}: {message.format}"
            )
            return
        with self.changed:
            previous = self.frames.get(sensor_id)
            frame_generation = previous[2] + 1 if previous else 1
            self.frames[sensor_id] = (
                bytes(message.data), time.monotonic(), frame_generation, content_type
            )
            self.changed.notify_all()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "generation": self.generation,
                "sensors": list(self.sensors.values()),
                "known_sensors": list(self.known_sensors.values()),
                "groups": list(self.groups.values()),
                "ports": list(self.ports.values()),
                "operations": list(self.operations.values()),
                "capture_operations": list(self.capture_operations.values()),
                "capture_records": list(self.capture_records),
            }

    def health(self) -> tuple[int, dict[str, Any]]:
        with self.lock:
            return health_response(
                self.snapshot(),
                self.last_state_at,
                self.frames,
                state_timeout_sec=self.state_timeout_sec,
            )

    def system_info(self) -> dict[str, Any]:
        result = {
            "machine_file": self.machine_file,
            "inventory_file": self.inventory_file,
            "managed_networks": [],
            "providers": [],
            "camera_profiles": [],
            "error": "",
        }
        try:
            with open(self.machine_file, "r", encoding="utf-8") as stream:
                machine = yaml.safe_load(stream) or {}
            result["managed_networks"] = [
                {"network_id": key, **value}
                for key, value in (machine.get("managed_networks") or {}).items()
            ]
            profiles_directory = pathlib.Path(
                (machine.get("camera_profiles") or {}).get(
                    "directory", "/etc/vixel/camera-profiles"
                )
            )
            if profiles_directory.is_dir():
                profile_names = []
                for path in profiles_directory.iterdir():
                    if path.suffix.lower() not in {".yaml", ".yml"}:
                        continue
                    try:
                        with path.open("r", encoding="utf-8") as stream:
                            profile = yaml.safe_load(stream) or {}
                        profile_names.append(str(profile.get("name", path.stem)))
                    except (OSError, yaml.YAMLError):
                        continue
                result["camera_profiles"] = sorted(set(profile_names))
            result["providers"] = sorted((machine.get("providers") or {}).keys())
        except (OSError, yaml.YAMLError) as error:
            result["error"] = str(error)
        return result

    @staticmethod
    def wait_future(future, timeout=10.0):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError("ROS request timed out")
        if future.exception():
            raise RuntimeError(str(future.exception()))
        return future.result()

    def call_service(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=2.0):
            service_name = str(getattr(client, "srv_name", "")).strip()
            detail = f" {service_name}" if service_name else ""
            raise RuntimeError(f"Vixel ROS service{detail} is unavailable")
        return self.wait_future(client.call_async(request), timeout)

    def enroll(self, sensor_id: str, body: dict[str, Any], operation_id: str = ""):
        if not self.enroll_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Vixel enrollment action is unavailable")
        goal = EnrollSensor.Goal()
        goal.candidate_id = sensor_id
        goal.network_id = str(body.get("network_id", ""))
        goal.requested_address = str(body.get("requested_address", ""))
        goal.display_name = str(body.get("display_name", ""))
        goal.location_label = str(body.get("location_label", "unknown"))
        handle = self.wait_future(self.enroll_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._operation_feedback(
                operation_id, feedback.feedback
            ) if operation_id else None,
        ), 5.0)
        if not handle.accepted:
            raise RuntimeError("Enrollment goal was rejected; refresh the discovery list")
        wrapped = self.wait_future(handle.get_result_async(), 300.0)
        return {
            "success": wrapped.result.success,
            "message": wrapped.result.message,
            "sensor": sensor_to_dict(wrapped.result.sensor),
        }

    def resolve_placement(self, sensor_id: str, body: dict[str, Any], operation_id: str):
        if not self.resolve_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Vixel placement action is unavailable")
        goal = ResolveSensorPlacement.Goal()
        goal.operation = str(body.get("operation", ""))
        goal.candidate_id = sensor_id
        goal.source_sensor_id = str(body.get("source_sensor_id", ""))
        goal.target_network_id = str(body.get("target_network_id", ""))
        handle = self.wait_future(self.resolve_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._operation_feedback(
                operation_id, feedback.feedback
            ),
        ), 5.0)
        if not handle.accepted:
            raise RuntimeError("Placement goal was rejected; refresh sensor state")
        wrapped = self.wait_future(handle.get_result_async(), 300.0)
        return {
            "success": wrapped.result.success,
            "message": wrapped.result.message,
            "sensor": sensor_to_dict(wrapped.result.sensor),
        }

    def record_capture(self, group_id: str, body: dict[str, Any]):
        if not self.record_capture_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Vixel capture recorder action is unavailable")
        goal = RecordCapture.Goal()
        goal.group_id = group_id
        goal.request_id = str(body.get("request_id", ""))
        handle = self.wait_future(
            self.record_capture_client.send_goal_async(goal), 5.0
        )
        if not handle.accepted:
            raise RuntimeError(
                "Capture was rejected; check the group or whether one of its cameras is busy"
            )
        wrapped = self._wait_for_capture_result(handle)
        return {
            "success": wrapped.result.success,
            "message": wrapped.result.message,
            "capture_id": wrapped.result.capture_id,
            "directory": wrapped.result.directory,
            "saved_sensor_ids": list(wrapped.result.saved_sensor_ids),
            "missing_sensor_ids": list(wrapped.result.missing_sensor_ids),
        }

    def _wait_for_capture_result(self, handle):
        result_future = handle.get_result_async()
        try:
            return self.wait_future(result_future, self.capture_result_timeout_sec)
        except TimeoutError as error:
            if result_future.done():
                return result_future.result()
            try:
                self.wait_future(handle.cancel_goal_async(), 5.0)
            except (RuntimeError, TimeoutError):
                pass
            if result_future.done():
                return result_future.result()
            raise TimeoutError(
                f"Capture exceeded the {self.capture_result_timeout_sec:.0f}s deadline; "
                "cancellation requested"
            ) from error

    @staticmethod
    def _capture_operation_request(body: dict[str, Any]) -> tuple[list[str], str, bool, str]:
        group_ids = body.get("group_ids", [])
        metadata = body.get("metadata", {})
        if not isinstance(group_ids, list) or not all(isinstance(value, str) for value in group_ids):
            raise ValueError("group_ids must be an array of strings")
        if not isinstance(metadata, dict):
            raise ValueError("metadata must be a JSON object")
        return (
            group_ids,
            str(body.get("request_id", "")),
            bool(body.get("synchronize_groups", True)),
            json.dumps(metadata, separators=(",", ":"), sort_keys=True),
        )

    def submit_capture_batch(self, body: dict[str, Any]):
        group_ids, request_id, synchronize, metadata_json = self._capture_operation_request(body)
        request = SubmitCaptureBatch.Request()
        request.group_ids = group_ids
        request.request_id = request_id
        request.synchronize_groups = synchronize
        request.metadata_json = metadata_json
        response = self.call_service(self.submit_capture_batch_client, request)
        if not response.accepted:
            raise RuntimeError(response.message)
        return {
            "accepted": True,
            "message": response.message,
            "operation_id": response.operation_id,
            "scheduled_time": {
                "sec": response.scheduled_time.sec,
                "nanosec": response.scheduled_time.nanosec,
            },
        }

    def start_capture_sequence(self, body: dict[str, Any]):
        group_ids, request_id, synchronize, metadata_json = self._capture_operation_request(body)
        interval_ms = int(body.get("interval_ms", 0))
        count = int(body.get("count", 0))
        if count < 0 or count > 0xFFFFFFFF:
            raise ValueError("count must be zero or a positive 32-bit integer")
        request = StartCaptureSequence.Request()
        request.group_ids = group_ids
        request.request_id = request_id
        request.interval_ms = interval_ms
        request.count = count
        request.synchronize_groups = synchronize
        request.metadata_json = metadata_json
        response = self.call_service(self.start_capture_sequence_client, request)
        if not response.accepted:
            raise RuntimeError(response.message)
        return {
            "accepted": True,
            "message": response.message,
            "operation_id": response.operation_id,
            "first_scheduled_time": {
                "sec": response.first_scheduled_time.sec,
                "nanosec": response.first_scheduled_time.nanosec,
            },
        }

    def cancel_capture_operation(self, operation_id: str):
        request = CancelCaptureOperation.Request()
        request.operation_id = operation_id
        response = self.call_service(self.cancel_capture_operation_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        return {
            "success": True,
            "message": response.message,
            "operation": capture_operation_to_dict(response.operation),
        }

    def get_capture_operation(self, operation_id: str):
        request = GetCaptureOperation.Request()
        request.operation_id = operation_id
        response = self.call_service(self.get_capture_operation_client, request)
        if not response.success:
            raise KeyError(response.message)
        return {
            "success": True,
            "message": response.message,
            "operation": capture_operation_to_dict(response.operation),
        }

    def trigger_group(self, group_id: str, body: dict[str, Any]):
        if not self.trigger_group_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Vixel group trigger action is unavailable")
        goal = TriggerGroup.Goal()
        goal.group_id = group_id
        goal.request_id = str(body.get("request_id", ""))
        handle = self.wait_future(
            self.trigger_group_client.send_goal_async(goal), 5.0
        )
        if not handle.accepted:
            raise RuntimeError(
                "Trigger was rejected; check the group, request ID, and operating mode"
            )
        wrapped = self.wait_future(handle.get_result_async(), 35.0)
        return {
            "success": wrapped.result.success,
            "message": wrapped.result.message,
            "capture_id": wrapped.result.capture_id,
            "scheduled_time": {
                "sec": wrapped.result.scheduled_time.sec,
                "nanosec": wrapped.result.scheduled_time.nanosec,
            },
            "participating_sensor_ids": list(
                wrapped.result.participating_sensor_ids
            ),
            "missing_sensor_ids": list(wrapped.result.missing_sensor_ids),
            "trigger_span_ns": int(wrapped.result.trigger_span_ns),
            "exposure_skew_ns": int(wrapped.result.exposure_skew_ns),
            "within_tolerance": wrapped.result.within_tolerance,
            "camera_timings": [
                {
                    "sensor_id": timing.sensor_id,
                    "device_timestamp_ns": int(timing.device_timestamp_ns),
                    "ptp_offset_ns": int(timing.ptp_offset_ns),
                    "synchronized": timing.synchronized,
                }
                for timing in wrapped.result.camera_timings
            ],
        }

    def start_operation(self, kind: str, target: str, worker) -> dict[str, Any]:
        operation_id = uuid.uuid4().hex
        operation = {
            "operation_id": operation_id,
            "kind": kind,
            "target": target,
            "state": "queued",
            "stage": "queued",
            "detail": "Waiting to start",
            "progress_percent": 0,
            "message": "",
        }
        with self.changed:
            active_count = sum(
                item.get("state") not in {"succeeded", "failed", "cancelled"}
                for item in self.operations.values()
            )
            if active_count >= self.max_active_operations:
                raise RuntimeError("gateway reached the active operation limit")
            self.operations[operation_id] = operation
            self.generation += 1
            self.changed.notify_all()
        threading.Thread(
            target=self._run_operation,
            args=(operation_id, worker),
            daemon=True,
            name=f"vixel-{kind}-{operation_id[:8]}",
        ).start()
        return operation

    def _run_operation(self, operation_id: str, worker) -> None:
        self._update_operation(operation_id, state="running", stage="starting", detail="Starting")
        try:
            result = worker(operation_id)
            if not result.get("success", False):
                raise RuntimeError(result.get("message", "Operation failed"))
            self._update_operation(
                operation_id, state="succeeded", stage="complete",
                detail=result.get("message", "Complete"), progress_percent=100,
                message=result.get("message", ""),
            )
        except Exception as error:
            self._update_operation(
                operation_id, state="failed", stage="failed", detail=str(error), message=str(error)
            )

    def _operation_feedback(self, operation_id: str, feedback) -> None:
        self._update_operation(
            operation_id,
            state="running",
            stage=feedback.stage,
            detail=feedback.detail,
            progress_percent=int(feedback.progress_percent),
        )

    def _update_operation(self, operation_id: str, **values) -> None:
        with self.changed:
            operation = self.operations.get(operation_id)
            if not operation:
                return
            operation.update(values)
            terminal_ids = [
                identity for identity, item in self.operations.items()
                if item.get("state") in {"succeeded", "failed", "cancelled"}
            ]
            for identity in terminal_ids[:-self.operation_history_limit]:
                self.operations.pop(identity, None)
            self.generation += 1
            self.changed.notify_all()

    def update_sensor(self, sensor_id: str, body: dict[str, Any]):
        current = self.sensors.get(sensor_id)
        if not current or not current["enrolled"]:
            raise KeyError(f"unknown enrolled sensor {sensor_id}")
        merged = {**current, **body}
        request = UpdateSensorMetadata.Request()
        request.sensor_id = sensor_id
        request.display_name = str(merged.get("display_name", sensor_id))
        request.location_label = str(merged.get("location_label", "unknown"))
        request.has_pose = bool(merged.get("has_pose", False))
        request.parent_frame = str(merged.get("parent_frame", ""))
        pose = merged.get("pose", {})
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        request.pose.position.x = float(position.get("x", 0.0))
        request.pose.position.y = float(position.get("y", 0.0))
        request.pose.position.z = float(position.get("z", 0.0))
        request.pose.orientation.x = float(orientation.get("x", 0.0))
        request.pose.orientation.y = float(orientation.get("y", 0.0))
        request.pose.orientation.z = float(orientation.get("z", 0.0))
        request.pose.orientation.w = float(orientation.get("w", 1.0))
        request.calibration_url = str(merged.get("calibration_url", ""))
        request.enabled = bool(merged.get("enabled", True))
        response = self.call_service(self.update_client, request)
        return {
            "success": response.success,
            "message": response.message,
            "sensor": sensor_to_dict(response.sensor),
        }

    def update_known_sensor(self, sensor_id: str, body: dict[str, Any]):
        current = self.known_sensors.get(sensor_id)
        if not current:
            raise KeyError(f"unknown sensor {sensor_id}")
        settings = body.get("provider_settings", current.get("provider_settings", {}))
        if not isinstance(settings, dict):
            raise ValueError("provider_settings must be a JSON object")
        request = UpdateKnownSensor.Request()
        request.sensor_id = sensor_id
        request.notes = str(body.get("notes", current.get("notes", "")))
        request.camera_profile = str(
            body.get("camera_profile", current.get("camera_profile", ""))
        )
        request.provider_settings_json = json.dumps(
            settings, separators=(",", ":"), sort_keys=True
        )
        response = self.call_service(self.update_known_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        return {
            "success": True,
            "message": response.message,
            "sensor": known_sensor_to_dict(response.sensor),
        }

    def purge_known_sensor(self, sensor_id: str):
        request = PurgeKnownSensor.Request()
        request.sensor_id = sensor_id
        response = self.call_service(self.purge_known_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        return {"success": True, "message": response.message}

    def set_port_mode(self, network_id: str, body: dict[str, Any]):
        request = SetPortMode.Request()
        request.network_id = network_id
        request.mode = str(body.get("mode", ""))
        response = self.call_service(self.port_mode_client, request)
        return {
            "success": response.success,
            "message": response.message,
            "port": port_to_dict(response.port),
        }


class VixelHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    request_queue_size = 64

    def __init__(self, address, handler, node: GatewayNode, max_connections: int = 64):
        self.node = node
        self._connection_slots = threading.BoundedSemaphore(max(1, max_connections))
        super().__init__(address, handler)

    def process_request(self, request, client_address):
        if not self._connection_slots.acquire(blocking=False):
            body = b'{"success":false,"message":"HTTP connection limit reached"}'
            response = (
                b"HTTP/1.1 503 Service Unavailable\r\n"
                b"Content-Type: application/json\r\n"
                + f"Content-Length: {len(body)}\r\n".encode("ascii")
                + b"Cache-Control: no-store\r\nConnection: close\r\n\r\n"
                + body
            )
            try:
                request.sendall(response)
            finally:
                self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._connection_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._connection_slots.release()


class Handler(BaseHTTPRequestHandler):
    server: VixelHTTPServer
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(40.0)

    def log_message(self, format_string, *args):
        self.server.node.get_logger().info("HTTP " + (format_string % args))

    def log_request(self, code="-", size="-"):
        message = f'HTTP "{self.requestline}" {code} {size}'
        logger = self.server.node.get_logger()
        if is_routine_snapshot_request(self.path, code):
            logger.debug(message)
        else:
            logger.info(message)

    def _json(self, status: int, value: Any):
        body = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status: int, message: str):
        self._json(status, {"success": False, "message": message})

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 1024 * 1024:
            raise ValueError("request body is too large")
        if length == 0:
            return {}
        value = json.loads(self.rfile.read(length))
        if not isinstance(value, dict):
            raise ValueError("JSON request body must be an object")
        return value

    def do_GET(self):
        # Finite dashboard requests must not occupy a server worker until the
        # idle socket timeout. Streaming handlers still remain active until
        # their method returns, after which their connection is closed too.
        self.close_connection = True
        parsed = urllib.parse.urlparse(self.path)
        parts = [part for part in parsed.path.split("/") if part]
        if (
            parsed.path == "/"
            or parsed.path == "/index.html"
            or (len(parts) == 2 and parts[0] == "sensors")
        ):
            body = self.server.node.static_file.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
        elif parsed.path == "/api/v1/sensors":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"], "sensors": snapshot["sensors"]
            })
        elif parsed.path == "/api/v1/known-sensors":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"],
                "known_sensors": snapshot["known_sensors"],
            })
        elif len(parts) == 4 and parts[:3] == ["api", "v1", "known-sensors"]:
            sensor = self.server.node.known_sensors.get(parts[3])
            if sensor is None:
                self._error(HTTPStatus.NOT_FOUND, f"unknown sensor {parts[3]}")
            else:
                self._json(HTTPStatus.OK, {"sensor": sensor})
        elif parsed.path == "/api/v1/groups":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"], "groups": snapshot["groups"]
            })
        elif parsed.path == "/api/v1/ports":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"], "ports": snapshot["ports"]
            })
        elif parsed.path == "/api/v1/operations":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"], "operations": snapshot["operations"]
            })
        elif parsed.path == "/api/v1/capture-operations":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"],
                "capture_operations": snapshot["capture_operations"],
            })
        elif len(parts) == 4 and parts[:3] == ["api", "v1", "capture-operations"]:
            try:
                self._json(
                    HTTPStatus.OK,
                    self.server.node.get_capture_operation(parts[3]),
                )
            except (KeyError, RuntimeError, TimeoutError) as error:
                self._error(HTTPStatus.NOT_FOUND, str(error))
        elif parsed.path == "/api/v1/captures":
            snapshot = self.server.node.snapshot()
            self._json(HTTPStatus.OK, {
                "generation": snapshot["generation"],
                "capture_records": snapshot["capture_records"],
            })
        elif parsed.path == "/api/v1/system/config":
            self._json(HTTPStatus.OK, self.server.node.system_info())
        elif parsed.path == "/api/v1/health":
            status, value = self.server.node.health()
            self._json(status, value)
        elif parsed.path == "/api/v1/events":
            self._events()
        elif (
            len(parts) == 5 and parts[:3] == ["api", "v1", "sensors"]
            and parts[4] in {"snapshot", "snapshot.jpg", "snapshot.png"}
        ):
            self._snapshot(parts[3])
        elif (
            len(parts) == 5 and parts[:3] == ["api", "v1", "sensors"]
            and parts[4] in {"stream", "stream.mjpg"}
        ):
            self._stream(parts[3])
        else:
            self._error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self):
        self._mutate("POST")

    def do_PATCH(self):
        self._mutate("PATCH")

    def do_PUT(self):
        self._mutate("PUT")

    def do_DELETE(self):
        self._mutate("DELETE")

    def _mutate(self, method: str):
        self.close_connection = True
        parsed = urllib.parse.urlparse(self.path)
        parts = [part for part in parsed.path.split("/") if part]
        node = self.server.node
        try:
            body = self._body()
            if method == "POST" and len(parts) == 5 and parts[:3] == ["api", "v1", "sensors"] and parts[4] == "enroll":
                operation = node.start_operation(
                    "enroll", parts[3],
                    lambda operation_id: node.enroll(parts[3], body, operation_id),
                )
                self._json(HTTPStatus.ACCEPTED, {"success": True, **operation})
            elif method == "POST" and len(parts) == 5 and parts[:3] == ["api", "v1", "sensors"] and parts[4] in {"replace", "move"}:
                operation_kind = parts[4]
                request_body = {**body, "operation": operation_kind}
                operation = node.start_operation(
                    operation_kind, parts[3],
                    lambda operation_id: node.resolve_placement(
                        parts[3], request_body, operation_id
                    ),
                )
                self._json(HTTPStatus.ACCEPTED, {"success": True, **operation})
            elif method == "PATCH" and len(parts) == 4 and parts[:3] == ["api", "v1", "sensors"]:
                self._json(HTTPStatus.OK, node.update_sensor(parts[3], body))
            elif (
                method == "PATCH" and len(parts) == 4
                and parts[:3] == ["api", "v1", "known-sensors"]
            ):
                self._json(HTTPStatus.OK, node.update_known_sensor(parts[3], body))
            elif method == "DELETE" and len(parts) == 4 and parts[:3] == ["api", "v1", "sensors"]:
                request = ForgetSensor.Request()
                request.sensor_id = parts[3]
                request.force = bool(body.get("force", False))
                response = node.call_service(node.forget_client, request)
                status = HTTPStatus.OK if response.success else HTTPStatus.CONFLICT
                self._json(status, {"success": response.success, "message": response.message})
            elif (
                method == "DELETE" and len(parts) == 4
                and parts[:3] == ["api", "v1", "known-sensors"]
            ):
                self._json(HTTPStatus.OK, node.purge_known_sensor(parts[3]))
            elif method == "POST" and parsed.path == "/api/v1/mode":
                target_kind = str(body.get("target_kind", ""))
                target_id = str(body.get("target_id", ""))
                mode = str(body.get("mode", ""))
                interval_ms = int(body.get("capture_interval_ms", 0))
                if target_kind == "group" and mode == "capture" and interval_ms > 0:
                    request = PrepareCaptureGroups.Request()
                    request.group_ids = [target_id]
                    request.interval_ms = interval_ms
                    response = node.call_service(node.prepare_capture_groups_client, request)
                    success = response.accepted
                else:
                    request = SetOperatingMode.Request()
                    request.target_kind = target_kind
                    request.target_id = target_id
                    request.mode = mode
                    response = node.call_service(node.mode_client, request)
                    success = response.success
                self._json(HTTPStatus.OK, {"success": success, "message": response.message})
            elif method == "PUT" and len(parts) == 5 and parts[:3] == ["api", "v1", "ports"] and parts[4] == "mode":
                self._json(HTTPStatus.OK, node.set_port_mode(parts[3], body))
            elif method == "PUT" and len(parts) == 4 and parts[:3] == ["api", "v1", "groups"]:
                request = UpsertSyncGroup.Request()
                request.group_id = parts[3]
                request.provider = str(body.get("provider", "genicam"))
                request.member_ids = list(body.get("member_ids", []))
                request.missing_policy = str(body.get("missing_policy", "strict"))
                request.trigger_source = "Action0"
                request.preview_rate_hz = float(body.get("preview_rate_hz", 2.0))
                request.preferred_master_id = ""
                response = node.call_service(node.group_client, request)
                self._json(HTTPStatus.OK, {"success": response.success, "message": response.message})
            elif method == "DELETE" and len(parts) == 4 and parts[:3] == ["api", "v1", "groups"]:
                request = DeleteSyncGroup.Request()
                request.group_id = parts[3]
                response = node.call_service(node.delete_group_client, request)
                self._json(HTTPStatus.OK, {"success": response.success, "message": response.message})
            elif method == "POST" and len(parts) == 5 and parts[:3] == ["api", "v1", "groups"] and parts[4] == "capture":
                self._json(HTTPStatus.OK, node.record_capture(parts[3], body))
            elif method == "POST" and len(parts) == 5 and parts[:3] == ["api", "v1", "groups"] and parts[4] == "trigger":
                self._json(HTTPStatus.OK, node.trigger_group(parts[3], body))
            elif method == "POST" and parsed.path == "/api/v1/capture-operations/batch":
                self._json(HTTPStatus.ACCEPTED, node.submit_capture_batch(body))
            elif method == "POST" and parsed.path == "/api/v1/capture-operations/sequence":
                self._json(HTTPStatus.ACCEPTED, node.start_capture_sequence(body))
            elif (
                method == "POST" and len(parts) == 5
                and parts[:3] == ["api", "v1", "capture-operations"]
                and parts[4] == "cancel"
            ):
                self._json(HTTPStatus.OK, node.cancel_capture_operation(parts[3]))
            else:
                self._error(HTTPStatus.NOT_FOUND, "not found")
        except TimeoutError as error:
            self._error(HTTPStatus.GATEWAY_TIMEOUT, str(error))
        except (ValueError, KeyError, RuntimeError, json.JSONDecodeError) as error:
            self._error(HTTPStatus.BAD_REQUEST, str(error))

    def _snapshot(self, sensor_id: str):
        with self.server.node.lock:
            frame = self.server.node.frames.get(sensor_id)
        if not frame:
            self._error(HTTPStatus.SERVICE_UNAVAILABLE, "no preview frame is available")
            return
        data, received_at, generation = frame[:3]
        content_type = frame[3] if len(frame) > 3 else "image/jpeg"
        etag = f'"{generation}"'
        frame_age_ms = str(int((time.monotonic() - received_at) * 1000))
        if self.headers.get("If-None-Match") == etag:
            self.send_response(HTTPStatus.NOT_MODIFIED)
            self.send_header("ETag", etag)
            self.send_header("X-Vixel-Frame-Age-Ms", frame_age_ms)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("ETag", etag)
        self.send_header("X-Vixel-Frame-Age-Ms", frame_age_ms)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def _stream(self, sensor_id: str):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=vixel-frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_generation = 0
        try:
            while True:
                with self.server.node.changed:
                    self.server.node.changed.wait_for(
                        lambda: self.server.node.frames.get(sensor_id, (b"", 0.0, 0))[2] > last_generation,
                        timeout=15.0,
                    )
                    frame = self.server.node.frames.get(sensor_id)
                if not frame or frame[2] <= last_generation:
                    continue
                data, _, last_generation = frame[:3]
                content_type = frame[3] if len(frame) > 3 else "image/jpeg"
                self.wfile.write(
                    f"--vixel-frame\r\nContent-Type: {content_type}\r\n".encode("ascii")
                )
                self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode("ascii"))
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout):
            return

    def _events(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        generation = -1
        try:
            while True:
                with self.server.node.changed:
                    self.server.node.changed.wait_for(
                        lambda: self.server.node.generation != generation, timeout=15.0
                    )
                    snapshot = self.server.node.snapshot()
                generation = snapshot["generation"]
                payload = json.dumps(snapshot, separators=(",", ":"))
                self.wfile.write(f"event: inventory\ndata: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout):
            return


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    server = None
    result = 0
    try:
        node = GatewayNode()
        server = VixelHTTPServer(
            (node.address, node.port), Handler, node, node.max_http_connections
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        node.get_logger().info(
            f"Vixel web gateway listening on http://{node.address}:{node.port}"
        )
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    except OSError as error:
        result = 1
        if node:
            node.get_logger().fatal(
                f"Cannot bind Vixel web gateway to "
                f"http://{node.address}:{node.port}: {error}"
            )
        else:
            print(f"vixel web gateway: {error}")
    finally:
        if server:
            server.server_close()
        try:
            if executor:
                executor.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if spin_thread:
            spin_thread.join(timeout=2.0)
        try:
            if node:
                node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
    return result


if __name__ == "__main__":
    main()
    GetCaptureOperation,
    StartCaptureSequence,
    SubmitCaptureBatch,
