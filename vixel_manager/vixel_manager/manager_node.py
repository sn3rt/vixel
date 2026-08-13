from __future__ import annotations

import ipaddress
import json
import os
import pathlib
import re
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from vixel_interfaces.action import EnrollSensor, ResolveSensorPlacement, TriggerGroup
from vixel_interfaces.msg import (
    KnownSensor,
    KnownSensorArray,
    ManagedPort,
    ManagedPortArray,
    ProviderAssignment,
    ProviderAssignmentArray,
    Sensor,
    SensorArray,
    SensorObservationArray,
    SyncGroup,
    SyncGroupArray,
)
from vixel_interfaces.srv import (
    CaptureGroup,
    DeleteSyncGroup,
    ForgetSensor,
    PrepareCaptureGroups,
    PurgeKnownSensor,
    ProviderCapture,
    ProvisionSensor,
    ReloadCameraProfiles,
    SetOperatingMode,
    SetPortMode,
    UpdateSensorMetadata,
    UpdateKnownSensor,
    UpsertSyncGroup,
)

from .camera_profiles import (
    CameraProfileError,
    load_camera_profiles,
    resolve_camera_settings,
)
from .registry import Allocation, Registry, RegistryError, VALID_OPERATING_MODES
from .port_status import PortStatus, PortStatusMonitor


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _provider_group_is_current(
    provider_message: SyncGroup | None, members: list[str], requested_mode: str,
    requested_capture_interval_ms: int = 0,
) -> bool:
    return bool(
        provider_message
        and provider_message.operating_mode == requested_mode
        and int(getattr(provider_message, "requested_capture_interval_ms", 0))
        == requested_capture_interval_ms
        and set(provider_message.member_ids) == set(members)
    )


def _runtime_sensor_is_ready(sensor: Sensor | None, requested_mode: str) -> bool:
    return bool(sensor and sensor.online and sensor.operating_mode == requested_mode)


def _pose_to_dict(pose: Pose, parent_frame: str) -> dict[str, Any]:
    return {
        "parent_frame": parent_frame,
        "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        },
    }


def _dict_to_pose(value: dict[str, Any] | None) -> Pose:
    message = Pose()
    message.orientation.w = 1.0
    if not value:
        return message
    position = value.get("position", {})
    orientation = value.get("orientation", {})
    message.position.x = float(position.get("x", 0.0))
    message.position.y = float(position.get("y", 0.0))
    message.position.z = float(position.get("z", 0.0))
    message.orientation.x = float(orientation.get("x", 0.0))
    message.orientation.y = float(orientation.get("y", 0.0))
    message.orientation.z = float(orientation.get("z", 0.0))
    message.orientation.w = float(orientation.get("w", 1.0))
    return message


class InventoryManager(Node):
    def __init__(self) -> None:
        super().__init__("inventory_manager", namespace="/vixel")
        # rclpy may destroy a partially constructed node when startup fails.
        # Establish cleanup state before loading any fallible configuration.
        self.lock = threading.RLock()
        self.catalog_dirty = False
        self.port_status_monitor = None
        self.callback_group = ReentrantCallbackGroup()
        machine_file = self.declare_parameter("machine_file", "/etc/vixel/machine.yaml").value
        inventory_file = self.declare_parameter(
            "inventory_file", "/var/lib/vixel/inventory.yaml"
        ).value
        legacy_file = self.declare_parameter("legacy_file", "").value
        observation_timeout = float(self.declare_parameter("observation_timeout_sec", 6.0).value)
        self.observation_timeout = max(observation_timeout, 2.0)
        port_status_refresh = float(
            self.declare_parameter("port_status_refresh_sec", 2.0).value
        )
        port_profile_refresh = float(
            self.declare_parameter("port_profile_refresh_sec", 30.0).value
        )
        state_publish_period = max(
            0.25, float(self.declare_parameter("state_publish_period_sec", 2.0).value)
        )

        inventory_file = self._writable_inventory_path(str(inventory_file))
        self.registry = Registry(str(machine_file), inventory_file, str(legacy_file))
        self.camera_profiles_directory = str(
            self.registry.machine["camera_profiles"]["directory"]
        )
        self.camera_profiles = load_camera_profiles(self.camera_profiles_directory)
        missing_profiles = sorted({
            str(sensor.get("camera_profile", ""))
            for sensor in self.registry.inventory["known_sensors"].values()
            if sensor.get("camera_profile")
            and sensor.get("camera_profile") not in self.camera_profiles
        })
        if missing_profiles:
            raise RegistryError(
                "inventory selects missing camera profiles: " + ", ".join(missing_profiles)
            )
        self.camera_backend = str(
            self.declare_parameter("camera_backend", "genicam").value
        )
        if self.camera_backend not in self.registry.machine.get("providers", {}):
            raise RegistryError(
                f"camera backend {self.camera_backend} is absent from machine providers"
            )
        configured_startup = str(self.registry.machine["defaults"].get("startup_mode", "idle"))
        start_preview = bool(self.declare_parameter("start_preview", False).value)
        self.default_group_mode = "preview" if start_preview else configured_startup
        self.default_sensor_mode = (
            self.default_group_mode if self.default_group_mode != "capture" else "idle"
        )
        self.get_logger().info(f"Machine configuration: {machine_file}")
        self.get_logger().info(f"Inventory state: {inventory_file}")

        self.generation = 1
        self.observations: dict[str, tuple[dict[str, Any], float]] = {}
        self.known_checkpoint_at: dict[str, float] = {}
        self.runtime: dict[str, Sensor] = {}
        self.provider_groups: dict[str, SyncGroup] = {}
        self.sensor_modes: dict[str, str] = {}
        self.group_modes: dict[str, str] = {}
        self.group_capture_intervals: dict[str, int] = {}
        self.provider_subscriptions = []
        self.assignment_publishers = {}
        self.provision_clients = {}
        self.capture_clients = {}
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.port_status_monitor = PortStatusMonitor(
            self.registry.machine["managed_networks"],
            fast_refresh_sec=port_status_refresh,
            profile_refresh_sec=port_profile_refresh,
        )
        self.port_status_version = 0

        self.sensor_publisher = self.create_publisher(SensorArray, "sensors", STATE_QOS)
        self.known_sensor_publisher = self.create_publisher(
            KnownSensorArray, "known_sensors", STATE_QOS
        )
        self.group_publisher = self.create_publisher(SyncGroupArray, "sync_groups", STATE_QOS)
        self.port_publisher = self.create_publisher(ManagedPortArray, "ports", STATE_QOS)
        providers = set(self.registry.machine.get("providers", {}).keys())
        providers.update(
            sensor["provider"] for sensor in self.registry.inventory["sensors"].values()
        )
        providers.update(
            sensor["provider"] for sensor in self.registry.inventory["known_sensors"].values()
        )
        providers.add(self.camera_backend)
        for provider in sorted(providers):
            self._connect_provider(provider)

        self.update_service = self.create_service(
            UpdateSensorMetadata,
            "update_sensor_metadata",
            self._update_metadata,
            callback_group=self.callback_group,
        )
        self.forget_service = self.create_service(
            ForgetSensor, "forget_sensor", self._forget_sensor, callback_group=self.callback_group
        )
        self.update_known_service = self.create_service(
            UpdateKnownSensor,
            "update_known_sensor",
            self._update_known_sensor,
            callback_group=self.callback_group,
        )
        self.reload_profiles_service = self.create_service(
            ReloadCameraProfiles,
            "reload_camera_profiles",
            self._reload_camera_profiles,
            callback_group=self.callback_group,
        )
        self.purge_known_service = self.create_service(
            PurgeKnownSensor,
            "purge_known_sensor",
            self._purge_known_sensor,
            callback_group=self.callback_group,
        )
        self.mode_service = self.create_service(
            SetOperatingMode,
            "set_operating_mode",
            self._set_mode,
            callback_group=self.callback_group,
        )
        self.prepare_capture_groups_service = self.create_service(
            PrepareCaptureGroups,
            "prepare_capture_groups",
            self._prepare_capture_groups,
            callback_group=self.callback_group,
        )
        self.upsert_group_service = self.create_service(
            UpsertSyncGroup,
            "upsert_sync_group",
            self._upsert_group,
            callback_group=self.callback_group,
        )
        self.delete_group_service = self.create_service(
            DeleteSyncGroup,
            "delete_sync_group",
            self._delete_group,
            callback_group=self.callback_group,
        )
        self.capture_service = self.create_service(
            CaptureGroup,
            "capture_group",
            self._capture_group,
            callback_group=self.callback_group,
        )
        self.enroll_action = ActionServer(
            self,
            EnrollSensor,
            "enroll_sensor",
            execute_callback=self._enroll,
            goal_callback=self._enroll_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.resolve_action = ActionServer(
            self,
            ResolveSensorPlacement,
            "resolve_sensor_placement",
            execute_callback=self._resolve_placement,
            goal_callback=self._resolve_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.trigger_action = ActionServer(
            self,
            TriggerGroup,
            "trigger_group",
            execute_callback=self._trigger_group,
            goal_callback=self._trigger_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.auto_enroll_client = ActionClient(
            self, EnrollSensor, "enroll_sensor", callback_group=self.callback_group
        )
        self.port_mode_service = self.create_service(
            SetPortMode,
            "set_port_mode",
            self._set_port_mode,
            callback_group=self.callback_group,
        )
        self.auto_enroll_inflight: set[str] = set()
        self.auto_enroll_network_inflight: dict[str, str] = {}
        self.auto_enroll_retry_after: dict[str, float] = {}
        self.enrollment_active: set[str] = set()
        self.enrollment_network_active: set[str] = set()
        self.placement_active: set[str] = set()
        self.timer = self.create_timer(
            state_publish_period, self._publish_state, callback_group=self.callback_group
        )
        self.automation_timer = self.create_timer(
            1.0, self._reconcile_automation, callback_group=self.callback_group
        )
        self.catalog_flush_timer = self.create_timer(
            5.0, self._flush_catalog, callback_group=self.callback_group
        )
        self.port_status_monitor.start()
        self._publish_state()

    def _writable_inventory_path(self, requested: str) -> str:
        path = pathlib.Path(requested).expanduser()
        parent = path.parent
        try:
            parent.mkdir(parents=True, exist_ok=True)
            probe = parent / ".vixel-write-test"
            probe.touch(exist_ok=False)
            probe.unlink()
            return str(path)
        except OSError:
            state_home = pathlib.Path(
                os.environ.get("XDG_STATE_HOME", pathlib.Path.home() / ".local/state")
            )
            fallback = state_home / "vixel/inventory.yaml"
            fallback.parent.mkdir(parents=True, exist_ok=True)
            self.get_logger().warning(
                f"Cannot write {requested}; using development inventory {fallback}"
            )
            return str(fallback)

    def _connect_provider(self, provider: str) -> None:
        assignments_topic = f"/vixel/providers/{provider}/assignments"
        self.assignment_publishers[provider] = self.create_publisher(
            ProviderAssignmentArray, assignments_topic, STATE_QOS
        )
        self.provider_subscriptions.append(
            self.create_subscription(
                SensorObservationArray,
                f"/vixel/providers/{provider}/observations",
                lambda message, name=provider: self._observations(name, message),
                STATE_QOS,
                callback_group=self.callback_group,
            )
        )
        self.provider_subscriptions.append(
            self.create_subscription(
                SensorArray,
                f"/vixel/providers/{provider}/status",
                lambda message, name=provider: self._provider_status(name, message),
                STATE_QOS,
                callback_group=self.callback_group,
            )
        )
        self.provider_subscriptions.append(
            self.create_subscription(
                SyncGroupArray,
                f"/vixel/providers/{provider}/group_status",
                lambda message, name=provider: self._provider_group_status(name, message),
                STATE_QOS,
                callback_group=self.callback_group,
            )
        )
        self.provision_clients[provider] = self.create_client(
            ProvisionSensor,
            f"/vixel/providers/{provider}/provision",
            callback_group=self.callback_group,
        )
        self.capture_clients[provider] = self.create_client(
            ProviderCapture,
            f"/vixel/providers/{provider}/capture",
            callback_group=self.callback_group,
        )

    def _reload_camera_profiles(self, _request, response):
        """Atomically replace the profile catalogue after validating selections."""
        try:
            loaded = load_camera_profiles(self.camera_profiles_directory)
            selected = {
                str(sensor.get("camera_profile", ""))
                for sensor in self.registry.inventory["known_sensors"].values()
                if sensor.get("camera_profile")
            }
            missing = sorted(selected - set(loaded))
            if missing:
                raise CameraProfileError(
                    "selected camera profiles are missing: " + ", ".join(missing)
                )
            self.camera_profiles = loaded
            with self.lock:
                self.generation += 1
            self._publish_state()
            response.success = True
            response.message = f"loaded {len(loaded)} camera profile(s)"
            response.profile_names = sorted(loaded)
        except CameraProfileError as error:
            response.success = False
            response.message = str(error)
            response.profile_names = sorted(self.camera_profiles)
        return response

    def _runtime_provider(self, record: dict[str, Any]) -> str:
        return self.camera_backend if record.get("kind", "camera") == "camera" else record["provider"]

    def _runtime_group_provider(self, group: dict[str, Any]) -> str:
        members = group.get("members", [])
        if not members:
            return group["provider"]
        providers = {
            self._runtime_provider(self.registry.inventory["sensors"][member])
            for member in members
        }
        if len(providers) != 1:
            raise RegistryError("synchronization group has mixed runtime providers")
        return providers.pop()

    def _observations(self, provider: str, message: SensorObservationArray) -> None:
        now = time.monotonic()
        with self.lock:
            for observed in message.observations:
                observation = {
                    "provider": provider,
                    "candidate_id": observed.candidate_id,
                    "kind": observed.kind,
                    "vendor": observed.vendor,
                    "model": observed.model,
                    "serial": observed.serial,
                    "mac_address": observed.mac_address,
                    "transport": observed.transport,
                    "interface_name": observed.interface_name,
                    "current_address": observed.current_address,
                    "capabilities": list(observed.capabilities),
                }
                candidate_id = self.registry.sensor_id_for_observation(observation)
                observation["candidate_id"] = candidate_id
                new_session = candidate_id not in self.observations
                self.observations[candidate_id] = (observation, now)
                checkpoint = (
                    now - self.known_checkpoint_at.get(candidate_id, 0.0) >= 60.0
                )
                network_id = self._configured_network_for_observation(observation)
                network = self.registry.machine["managed_networks"].get(network_id, {})
                if self.registry.record_observation(
                    observation,
                    network_id,
                    bool(network.get("approved", False)),
                    new_session=new_session,
                    checkpoint=checkpoint,
                ):
                    self.catalog_dirty = True
                    self.known_checkpoint_at[candidate_id] = now
            self.generation += 1

    def _flush_catalog(self) -> None:
        with self.lock:
            if not self.catalog_dirty:
                return
            try:
                self.registry.save()
                self.catalog_dirty = False
            except (RegistryError, OSError) as error:
                self.get_logger().error(f"Unable to save known-sensor catalogue: {error}")

    def _provider_status(self, provider: str, message: SensorArray) -> None:
        with self.lock:
            provider_ids = {
                sensor_id for sensor_id, sensor in self.registry.inventory["sensors"].items()
                if self._runtime_provider(sensor) == provider
            }
            for sensor_id in provider_ids:
                self.runtime.pop(sensor_id, None)
            for sensor in message.sensors:
                self.runtime[sensor.sensor_id] = sensor
            self.generation += 1

    def _provider_group_status(self, provider: str, message: SyncGroupArray) -> None:
        with self.lock:
            for group_id in [
                key for key, group in self.registry.inventory["sync_groups"].items()
                if self._runtime_group_provider(group) == provider
            ]:
                self.provider_groups.pop(group_id, None)
            for group in message.groups:
                self.provider_groups[group.group_id] = group
            self.generation += 1

    def _sensor_message(self, sensor_id: str, record: dict[str, Any]) -> Sensor:
        message = Sensor()
        message.stamp = self.get_clock().now().to_msg()
        message.sensor_id = sensor_id
        message.provider = self._runtime_provider(record)
        message.kind = record.get("kind", "camera")
        message.vendor = record.get("vendor", record["provider"])
        message.model = record.get("model", "")
        message.serial = record["serial"]
        message.mac_address = record.get("mac_address", "")
        message.enrolled = True
        message.enabled = bool(record.get("enabled", True))
        message.display_name = record.get("display_name", sensor_id)
        message.location_label = record.get("location_label", "unknown")
        pose = record.get("pose")
        message.has_pose = pose is not None
        message.parent_frame = pose.get("parent_frame", "") if pose else ""
        message.pose = _dict_to_pose(pose)
        message.calibration_url = record.get("calibration_url", "")
        message.topic_base = f"/vixel/sensors/{sensor_id}"
        message.network_id = record.get("network_id", "")
        message.assigned_address = record.get("assigned_address", "")
        message.capabilities = list(record.get("capabilities", []))
        network = self.registry.machine["managed_networks"].get(message.network_id, {})
        message.managed = bool(network.get("approved", False))
        message.pending_action = ""
        message.status_detail = ""
        message.sync_group = self._group_for_sensor(sensor_id)
        message.operating_mode = self._mode_for_sensor(sensor_id)
        runtime = self.runtime.get(sensor_id)
        if runtime:
            message.online = runtime.online
            message.lifecycle_state = runtime.lifecycle_state
            message.current_address = runtime.current_address
            message.interface_name = runtime.interface_name
            message.last_error = runtime.last_error
            message.status_detail = runtime.status_detail
            message.applied_settings_json = runtime.applied_settings_json
            if runtime.model:
                message.model = runtime.model
            if runtime.mac_address:
                message.mac_address = runtime.mac_address
        else:
            message.online = False
            message.lifecycle_state = "disabled" if not message.enabled else "offline"
        observed = self.observations.get(sensor_id)
        if observed:
            observation = observed[0]
            configured_network = self._configured_network_for_observation(observation)
            observed_network = self._network_for_observation(observation)
            message.current_address = observation.get("current_address", "")
            message.interface_name = observation.get("interface_name", "")
            if configured_network and not self.registry.machine["managed_networks"][
                configured_network
            ]["approved"]:
                message.online = False
                message.lifecycle_state = "unapproved"
                message.pending_action = "external"
                message.status_detail = (
                    f"Camera is visible on unapproved network {configured_network}"
                )
            elif observed_network and observed_network != message.network_id:
                message.online = False
                message.lifecycle_state = "placement_conflict"
                message.pending_action = "move"
                message.status_detail = (
                    f"Camera is connected through {observed_network}; "
                    f"assigned to {message.network_id}"
                )
        if network and not network["approved"]:
            message.online = False
            message.lifecycle_state = "unapproved"
            message.pending_action = "external"
            message.status_detail = (
                f"Assigned network {message.network_id} is not approved"
            )
        return message

    def _unknown_message(self, candidate_id: str, observation: dict[str, Any]) -> Sensor:
        message = Sensor()
        message.stamp = self.get_clock().now().to_msg()
        message.sensor_id = candidate_id
        message.provider = observation["provider"]
        message.kind = observation.get("kind", "camera")
        message.vendor = observation.get("vendor", observation["provider"])
        message.model = observation.get("model", "")
        message.serial = observation["serial"]
        message.mac_address = observation.get("mac_address", "")
        message.enrolled = False
        message.enabled = False
        message.online = True
        message.lifecycle_state = "discovered"
        message.display_name = candidate_id
        message.location_label = "unknown"
        message.topic_base = ""
        message.current_address = observation.get("current_address", "")
        message.interface_name = observation.get("interface_name", "")
        message.capabilities = list(observation.get("capabilities", []))
        message.operating_mode = "idle"
        network_id = self._configured_network_for_observation(observation)
        message.network_id = network_id
        network = self.registry.machine["managed_networks"].get(network_id, {})
        message.managed = bool(network.get("approved", False))
        message.pending_action = ""
        message.status_detail = ""
        if not message.managed:
            message.pending_action = "external"
            message.status_detail = "Discovered outside an approved sensor interface"
        elif self.registry.inventory["known_sensors"].get(candidate_id, {}).get(
            "catalog_state"
        ) == "retired":
            message.pending_action = "retired"
            message.status_detail = "Previously replaced sensor; manual review required"
        else:
            assigned = [
                sensor_id for sensor_id, sensor in self.registry.inventory["sensors"].items()
                if sensor.get("network_id") == network_id
            ]
            if self.registry.port_mode(network_id) == "direct" and assigned:
                message.pending_action = "replace"
                message.status_detail = f"Replacement candidate for {assigned[0]}"
            elif self.registry.inventory["known_sensors"].get(candidate_id, {}).get(
                "catalog_state"
            ) == "archived":
                message.pending_action = "archived"
                message.status_detail = "Archived sensor; explicit re-enrollment required"
            elif candidate_id in self.auto_enroll_inflight:
                message.pending_action = "auto_enrolling"
                message.status_detail = "Assigning address and saving sensor"
        message.pose.orientation.w = 1.0
        return message

    def _known_sensor_message(
        self, sensor_id: str, record: dict[str, Any], snapshot: dict[str, Any]
    ) -> KnownSensor:
        message = KnownSensor()
        message.stamp = self.get_clock().now().to_msg()
        message.sensor_id = sensor_id
        message.provider = str(record.get("provider", ""))
        message.kind = str(record.get("kind", "camera"))
        message.vendor = str(record.get("vendor", message.provider))
        message.model = str(record.get("model", ""))
        message.serial = str(record.get("serial", ""))
        message.mac_address = str(record.get("mac_address", ""))
        message.capabilities = list(record.get("capabilities", []))
        message.catalog_state = (
            "enrolled" if sensor_id in snapshot["sensors"]
            else str(record.get("catalog_state", "observed"))
        )
        message.enrolled = sensor_id in snapshot["sensors"]
        runtime = self.runtime.get(sensor_id)
        observation_entry = self.observations.get(sensor_id)
        observation = observation_entry[0] if observation_entry else None
        message.online = bool(observation) or bool(runtime and runtime.online)
        latest = dict(record.get("latest_observation", {}))
        if observation:
            network_id = self._configured_network_for_observation(observation)
            network = self.registry.machine["managed_networks"].get(network_id, {})
            latest = {
                "transport": str(observation.get("transport", "")),
                "interface_name": str(observation.get("interface_name", "")),
                "current_address": str(observation.get("current_address", "")),
                "network_id": network_id,
                "managed": bool(network.get("approved", False)),
            }
        elif message.enrolled and not latest:
            enrollment = snapshot["sensors"][sensor_id]
            network_id = str(enrollment.get("network_id", ""))
            network = self.registry.machine["managed_networks"].get(network_id, {})
            latest = {
                "transport": "",
                "interface_name": str(network.get("interface", "")),
                "current_address": str(enrollment.get("assigned_address", "")),
                "network_id": network_id,
                "managed": bool(network.get("approved", False)),
            }
        message.managed = bool(latest.get("managed", False))
        message.first_seen_at = str(record.get("first_seen_at", ""))
        message.last_seen_at = str(record.get("last_seen_at", ""))
        message.sighting_count = int(record.get("sighting_count", 0))
        message.transport = str(latest.get("transport", ""))
        message.interface_name = str(latest.get("interface_name", ""))
        message.current_address = str(latest.get("current_address", ""))
        message.network_id = str(latest.get("network_id", ""))
        message.notes = str(record.get("notes", ""))
        message.camera_profile = str(record.get("camera_profile", ""))
        message.provider_settings_json = json.dumps(
            record.get("provider_settings", {}), separators=(",", ":"), sort_keys=True
        )
        message.last_configuration_json = json.dumps(
            record.get("last_configuration"), separators=(",", ":"), sort_keys=True
        )
        message.changes_json = json.dumps(
            record.get("changes", []), separators=(",", ":"), sort_keys=True
        )
        message.replaced_by = str(record.get("replaced_by", ""))
        return message

    def _configured_network_for_observation(self, observation: dict[str, Any]) -> str:
        interface = str(observation.get("interface_name", ""))
        for network_id, network in self.registry.machine["managed_networks"].items():
            if network["interface"] == interface:
                return network_id
        try:
            address = ipaddress.ip_address(str(observation.get("current_address", "")))
        except ValueError:
            return ""
        for network_id, network in self.registry.machine["managed_networks"].items():
            if address in ipaddress.ip_interface(network["host_cidr"]).network:
                return network_id
        return ""

    def _network_for_observation(self, observation: dict[str, Any]) -> str:
        network_id = self._configured_network_for_observation(observation)
        if not network_id:
            return ""
        network = self.registry.machine["managed_networks"][network_id]
        return network_id if network["approved"] else ""

    def _sync_group_message(self, group_id: str, record: dict[str, Any]) -> SyncGroup:
        provider_message = self.provider_groups.get(group_id)
        requested_mode = self.group_modes.get(group_id, self.default_group_mode)
        requested_interval = self.group_capture_intervals.get(group_id, 0)
        # A provider status from the previous operating mode can remain latched
        # while its camera sessions are being restarted.  Do not relabel that
        # stale status with the new mode: doing so briefly made a capture button
        # available before any of the new capture sessions existed.
        if _provider_group_is_current(
            provider_message, list(record["members"]), requested_mode, requested_interval
        ):
            return provider_message
        result = SyncGroup()
        result.stamp = self.get_clock().now().to_msg()
        result.group_id = group_id
        result.provider = self._runtime_group_provider(record)
        result.member_ids = list(record["members"])
        result.missing_policy = record["missing_policy"]
        result.trigger_source = "Action0"
        result.operating_mode = requested_mode
        result.preview_rate_hz = float(record["preview_rate_hz"])
        result.requested_capture_interval_ms = requested_interval
        result.cadence_configured = requested_interval == 0
        result.cadence_ready = requested_interval == 0
        result.preferred_master_id = ""
        result.online_member_ids = [
            member for member in result.member_ids
            if _runtime_sensor_is_ready(self.runtime.get(member), requested_mode)
        ]
        result.missing_member_ids = [
            member for member in result.member_ids if member not in result.online_member_ids
        ]
        result.ready = result.cadence_ready and (result.operating_mode == "idle" or (
            not result.missing_member_ids or result.missing_policy == "degraded"
        ))
        return result

    def _publish_state(self) -> None:
        with self.lock:
            port_status_version, port_statuses = self.port_status_monitor.snapshot()
            if port_status_version != self.port_status_version:
                self.port_status_version = port_status_version
                self.generation += 1
            now = time.monotonic()
            for candidate_id in list(self.observations):
                if now - self.observations[candidate_id][1] > self.observation_timeout:
                    del self.observations[candidate_id]
            snapshot = self.registry.snapshot()
            enrolled_ids = set(snapshot["sensors"])
            sensor_array = SensorArray()
            sensor_array.header.stamp = self.get_clock().now().to_msg()
            sensor_array.generation = self.generation
            sensor_array.sensors = [
                self._sensor_message(sensor_id, record)
                for sensor_id, record in sorted(snapshot["sensors"].items())
            ]
            sensor_array.sensors.extend(
                self._unknown_message(candidate_id, observation)
                for candidate_id, (observation, _) in sorted(self.observations.items())
                if candidate_id not in enrolled_ids
            )
            known_array = KnownSensorArray()
            known_array.header.stamp = sensor_array.header.stamp
            known_array.generation = self.generation
            known_array.sensors = [
                self._known_sensor_message(sensor_id, record, snapshot)
                for sensor_id, record in sorted(snapshot["known_sensors"].items())
            ]
            group_array = SyncGroupArray()
            group_array.header.stamp = sensor_array.header.stamp
            group_array.generation = self.generation
            group_array.groups = [
                self._sync_group_message(group_id, record)
                for group_id, record in sorted(snapshot["sync_groups"].items())
            ]
            port_array = ManagedPortArray()
            port_array.header.stamp = sensor_array.header.stamp
            port_array.generation = self.generation
            port_array.ports = [
                self._managed_port_message(
                    network_id, network, snapshot, port_statuses.get(network_id)
                )
                for network_id, network in sorted(
                    self.registry.machine["managed_networks"].items()
                )
            ]
            self.sensor_publisher.publish(sensor_array)
            self.known_sensor_publisher.publish(known_array)
            self.group_publisher.publish(group_array)
            self.port_publisher.publish(port_array)
            self._publish_static_transforms(sensor_array)
            self._publish_assignments(snapshot)

    def _managed_port_message(
        self,
        network_id: str,
        network: dict[str, Any],
        snapshot: dict[str, Any] | None = None,
        port_status: PortStatus | None = None,
    ) -> ManagedPort:
        snapshot = snapshot or self.registry.snapshot()
        if port_status is None:
            _, port_statuses = self.port_status_monitor.snapshot()
            port_status = port_statuses.get(network_id)
        message = ManagedPort()
        message.stamp = self.get_clock().now().to_msg()
        message.network_id = network_id
        message.interface_name = str(network["interface"])
        message.interface_mac = str(network.get("interface_mac", ""))
        message.mode = self.registry.port_mode(network_id)
        message.approved = bool(network.get("approved", False))
        message.auto_enroll = bool(network.get("auto_enroll", False))
        message.host_cidr = str(network["host_cidr"])
        message.capacity = self.registry.port_capacity(network_id)
        message.enrolled_sensor_ids = sorted(
            sensor_id for sensor_id, sensor in snapshot["sensors"].items()
            if sensor.get("network_id") == network_id
        )
        message.discovered_candidate_ids = sorted(
            candidate_id for candidate_id, (observation, _) in self.observations.items()
            if self._configured_network_for_observation(observation) == network_id
        )
        if port_status is None:
            message.lifecycle_state = "probing"
            message.last_error = "Waiting for host port status"
            return message
        if not port_status.present:
            message.lifecycle_state = "missing"
            message.last_error = "Interface is not present"
            return message
        if (
            message.interface_mac
            and port_status.actual_mac
            and port_status.actual_mac != message.interface_mac.lower()
        ):
            message.lifecycle_state = "wrong_hardware"
            message.last_error = (
                f"Expected MAC {message.interface_mac}; found {port_status.actual_mac}"
            )
            return message
        message.link_up = port_status.link_up
        message.configured = (
            message.host_cidr in port_status.addresses or port_status.profile_configured
        )
        if not message.configured:
            message.lifecycle_state = "unconfigured"
            message.last_error = "Run vixel-network-setup apply"
            if port_status.error:
                message.last_error += f"; {port_status.error}"
        elif message.mode == "direct" and len(message.discovered_candidate_ids) > 1:
            message.lifecycle_state = "conflict"
            message.last_error = "Multiple cameras detected on a direct port; enable switched mode"
        elif message.link_up:
            message.lifecycle_state = "ready"
            message.last_error = port_status.error
        else:
            message.lifecycle_state = "link_down"
            message.last_error = port_status.error
        return message

    def destroy_node(self):
        if hasattr(self, "registry"):
            self._flush_catalog()
        if self.port_status_monitor is not None:
            self.port_status_monitor.stop()
        return super().destroy_node()

    def _publish_static_transforms(self, sensor_array: SensorArray) -> None:
        transforms = []
        for sensor in sensor_array.sensors:
            if not sensor.enrolled or not sensor.has_pose or not sensor.parent_frame:
                continue
            transform = TransformStamped()
            transform.header.stamp = sensor.stamp
            transform.header.frame_id = sensor.parent_frame
            transform.child_frame_id = f"{sensor.sensor_id}_optical_frame"
            transform.transform.translation.x = sensor.pose.position.x
            transform.transform.translation.y = sensor.pose.position.y
            transform.transform.translation.z = sensor.pose.position.z
            transform.transform.rotation = sensor.pose.orientation
            transforms.append(transform)
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def _publish_assignments(self, snapshot: dict[str, Any]) -> None:
        groups_for_member = {
            member: (group_id, group)
            for group_id, group in snapshot["sync_groups"].items()
            for member in group["members"]
        }
        by_provider: dict[str, list[ProviderAssignment]] = {
            provider: [] for provider in self.assignment_publishers
        }
        for sensor_id, sensor in snapshot["sensors"].items():
            network = self.registry.machine["managed_networks"].get(
                sensor.get("network_id", "")
            )
            if not network or not network["approved"]:
                continue
            assignment = ProviderAssignment()
            assignment.stamp = self.get_clock().now().to_msg()
            assignment.sensor_id = sensor_id
            runtime_provider = self._runtime_provider(sensor)
            assignment.provider = runtime_provider
            assignment.kind = sensor.get("kind", "camera")
            assignment.serial = sensor["serial"]
            assignment.mac_address = sensor.get("mac_address", "")
            assignment.enabled = bool(sensor.get("enabled", True))
            assignment.frame_id = f"{sensor_id}_optical_frame"
            assignment.calibration_url = sensor.get("calibration_url", "")
            assignment.network_id = sensor.get("network_id", "")
            assignment.assigned_address = sensor.get("assigned_address", "")
            group_id, group = groups_for_member.get(sensor_id, ("", {}))
            assignment.sync_group = group_id
            assignment.group_missing_policy = group.get("missing_policy", "")
            assignment.group_trigger_source = "Action0" if group_id else ""
            assignment.preferred_master_id = ""
            assignment.operating_mode = (
                self.group_modes.get(group_id, self.default_group_mode) if group_id
                else self.sensor_modes.get(sensor_id, self.default_sensor_mode)
            )
            assignment.preview_rate_hz = float(
                group.get("preview_rate_hz", self.registry.machine["defaults"]["preview_rate_hz"])
            )
            assignment.requested_capture_interval_ms = (
                self.group_capture_intervals.get(group_id, 0)
                if group_id and assignment.operating_mode == "capture" else 0
            )
            known = snapshot["known_sensors"].get(sensor_id, {})
            provider_config = self.registry.machine["providers"].get(runtime_provider, {})
            portable_defaults = dict(provider_config.get("imaging", {}))
            if "exposure_time_us" in portable_defaults:
                portable_defaults["exposure_us"] = portable_defaults.pop("exposure_time_us")
            if "capture_png_compression" in provider_config:
                portable_defaults["capture_png_compression"] = provider_config[
                    "capture_png_compression"
                ]
            for key in ("preview_width", "preview_format", "png_compression", "jpeg_quality"):
                if key in self.registry.machine["defaults"]:
                    portable_defaults[key] = self.registry.machine["defaults"][key]
            effective_settings = resolve_camera_settings(
                portable_defaults,
                self.camera_profiles,
                str(known.get("camera_profile", "")),
                dict(known.get("provider_settings", {})),
            )
            effective_settings["camera_profile"] = str(known.get("camera_profile", ""))
            if group_id:
                effective_settings["trigger_source"] = "Action0"
            assignment.provider_settings_json = json.dumps(
                effective_settings,
                separators=(",", ":"), sort_keys=True,
            )
            by_provider.setdefault(runtime_provider, []).append(assignment)
        for provider, publisher in self.assignment_publishers.items():
            message = ProviderAssignmentArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.generation = self.generation
            message.assignments = by_provider.get(provider, [])
            publisher.publish(message)

    def _group_for_sensor(self, sensor_id: str) -> str:
        for group_id, group in self.registry.inventory["sync_groups"].items():
            if sensor_id in group["members"]:
                return group_id
        return ""

    def _mode_for_sensor(self, sensor_id: str) -> str:
        group_id = self._group_for_sensor(sensor_id)
        return (
            self.group_modes.get(group_id, self.default_group_mode) if group_id
            else self.sensor_modes.get(sensor_id, self.default_sensor_mode)
        )

    def _reconcile_automation(self) -> None:
        now = time.monotonic()
        with self.lock:
            observations = {
                candidate_id: dict(value[0])
                for candidate_id, value in self.observations.items()
            }
            enrolled = self.registry.snapshot()["sensors"]
            unavailable = {
                sensor_id for sensor_id, record in self.registry.snapshot()["known_sensors"].items()
                if record.get("catalog_state") in {"archived", "retired"}
            }
            inflight = set(self.auto_enroll_inflight)
            busy_networks = set(self.auto_enroll_network_inflight.values())
        for candidate_id, observation in observations.items():
            if candidate_id in enrolled or candidate_id in unavailable:
                continue
            if candidate_id in inflight:
                continue
            if now < self.auto_enroll_retry_after.get(candidate_id, 0.0):
                continue
            network_id = self._network_for_observation(observation)
            network = self.registry.machine["managed_networks"].get(network_id)
            if not network or not network.get("approved") or not network.get("auto_enroll"):
                continue
            if network_id in busy_networks:
                continue
            assigned = [
                sensor for sensor in enrolled.values()
                if sensor.get("network_id") == network_id
            ]
            if self.registry.port_mode(network_id) == "direct" and assigned:
                continue
            if len(assigned) >= self.registry.port_capacity(network_id):
                continue
            self._start_auto_enroll(candidate_id, network_id)
            busy_networks.add(network_id)

    def _start_auto_enroll(self, candidate_id: str, network_id: str) -> None:
        if not self.auto_enroll_client.server_is_ready():
            self.auto_enroll_retry_after[candidate_id] = time.monotonic() + 2.0
            return
        goal = EnrollSensor.Goal()
        goal.candidate_id = candidate_id
        goal.network_id = network_id
        goal.display_name = candidate_id
        goal.location_label = "unknown"
        with self.lock:
            self.auto_enroll_inflight.add(candidate_id)
            self.auto_enroll_network_inflight[candidate_id] = network_id
        self.get_logger().info(
            f"Auto-enrolling {candidate_id} on approved network {network_id}"
        )
        future = self.auto_enroll_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, identity=candidate_id: self._auto_enroll_goal_sent(
                identity, completed
            )
        )

    def _auto_enroll_goal_sent(self, candidate_id: str, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                raise RegistryError("automatic enrollment goal was rejected")
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda completed, identity=candidate_id: self._auto_enroll_finished(
                    identity, completed
                )
            )
        except Exception as error:
            self._auto_enroll_failed(candidate_id, str(error))

    def _auto_enroll_finished(self, candidate_id: str, future) -> None:
        try:
            result = future.result().result
            if not result.success:
                raise RegistryError(result.message)
            self.get_logger().info(result.message)
            self.auto_enroll_retry_after.pop(candidate_id, None)
        except Exception as error:
            self._auto_enroll_failed(candidate_id, str(error))
        finally:
            with self.lock:
                self.auto_enroll_inflight.discard(candidate_id)
                self.auto_enroll_network_inflight.pop(candidate_id, None)
            self._publish_state()

    def _auto_enroll_failed(self, candidate_id: str, message: str) -> None:
        with self.lock:
            self.auto_enroll_inflight.discard(candidate_id)
            self.auto_enroll_network_inflight.pop(candidate_id, None)
        self.auto_enroll_retry_after[candidate_id] = time.monotonic() + 30.0
        self.get_logger().warning(f"Auto-enrollment of {candidate_id} failed: {message}")

    def _resolve_goal(self, goal: ResolveSensorPlacement.Goal) -> GoalResponse:
        if goal.operation not in {"replace", "move"}:
            return GoalResponse.REJECT
        with self.lock:
            return GoalResponse.ACCEPT if goal.candidate_id in self.observations else GoalResponse.REJECT

    async def _resolve_placement(self, goal_handle):
        result = ResolveSensorPlacement.Result()
        goal = goal_handle.request
        claimed_ids: set[str] = set()
        try:
            with self.lock:
                requested_ids = {goal.candidate_id, goal.source_sensor_id} - {""}
                duplicate = requested_ids & self.placement_active
                if duplicate:
                    raise RegistryError(
                        f"placement operation already active for {sorted(duplicate)[0]}"
                    )
                self.placement_active.update(requested_ids)
                claimed_ids = requested_ids
                observation = dict(self.observations[goal.candidate_id][0])
            observed_network = self._network_for_observation(observation)
            if not observed_network:
                raise RegistryError("sensor is not connected through an approved managed port")
            if goal.operation == "replace":
                source = self.registry.inventory["sensors"].get(goal.source_sensor_id)
                if not source:
                    raise RegistryError(f"unknown enrolled sensor {goal.source_sensor_id}")
                network_id = source["network_id"]
                if observed_network != network_id:
                    raise RegistryError(
                        f"replacement is on {observed_network}, not the reserved port {network_id}"
                    )
                if self.registry.port_mode(network_id) != "direct":
                    raise RegistryError("replacement confirmation only applies to a direct port")
                runtime = self.runtime.get(goal.source_sensor_id)
                if runtime and runtime.online:
                    raise RegistryError("reserved sensor is still online; disconnect it first")
                allocation = Allocation(
                    network_id=network_id,
                    address=source["assigned_address"],
                    subnet_mask=str(ipaddress.ip_interface(
                        self.registry.machine["managed_networks"][network_id]["host_cidr"]
                    ).netmask),
                    gateway=str(self.registry.machine["managed_networks"][network_id].get(
                        "gateway", "0.0.0.0"
                    )),
                )
            else:
                sensor_id = goal.source_sensor_id or goal.candidate_id
                if sensor_id not in self.registry.inventory["sensors"]:
                    raise RegistryError(f"unknown enrolled sensor {sensor_id}")
                source = self.registry.inventory["sensors"][sensor_id]
                if goal.target_network_id != observed_network:
                    raise RegistryError(
                        f"camera is observed on {observed_network}, not {goal.target_network_id}"
                    )
                if source.get("network_id") == goal.target_network_id:
                    raise RegistryError("camera is already assigned to this managed port")
                allocation = self.registry.allocate(goal.target_network_id)
                network_id = allocation.network_id
            feedback = ResolveSensorPlacement.Feedback()
            feedback.stage = "provisioning"
            feedback.detail = f"Assigning {allocation.address} on {network_id}"
            feedback.progress_percent = 25
            goal_handle.publish_feedback(feedback)
            response = await self._provision_observation(observation, allocation)
            if not response.success:
                raise RegistryError(response.message)
            feedback.stage = "persisting"
            feedback.detail = "Saving inventory and placement history"
            feedback.progress_percent = 80
            goal_handle.publish_feedback(feedback)
            if goal.operation == "replace":
                sensor_id = self.registry.replace_sensor(goal.source_sensor_id, observation)
            else:
                sensor_id = goal.source_sensor_id or goal.candidate_id
                self.registry.move_sensor(sensor_id, allocation)
            with self.lock:
                self.generation += 1
            self._publish_state()
            result.success = True
            result.message = f"{goal.operation.title()} completed for {sensor_id}"
            result.sensor = self._sensor_message(
                sensor_id, self.registry.inventory["sensors"][sensor_id]
            )
            goal_handle.succeed()
        except (RegistryError, KeyError, ValueError) as error:
            result.success = False
            result.message = str(error)
            goal_handle.abort()
        finally:
            with self.lock:
                self.placement_active.difference_update(claimed_ids)
        return result

    async def _provision_observation(self, observation: dict[str, Any], allocation):
        client = self.provision_clients[observation["provider"]]
        if not client.wait_for_service(timeout_sec=5.0):
            raise RegistryError(f"provider {observation['provider']} is unavailable")
        request = ProvisionSensor.Request()
        request.candidate_id = observation["candidate_id"]
        request.sensor_id = self.registry.sensor_id_for_observation(observation)
        request.serial = observation["serial"]
        request.mac_address = observation.get("mac_address", "")
        request.network_id = allocation.network_id
        request.target_address = allocation.address
        request.subnet_mask = allocation.subnet_mask
        request.gateway = allocation.gateway
        return await client.call_async(request)

    def _set_port_mode(self, request, response):
        try:
            self.registry.set_port_mode(request.network_id, request.mode)
            with self.lock:
                self.generation += 1
            response.success = True
            response.message = f"{request.network_id} is now {request.mode}"
            response.port = self._managed_port_message(
                request.network_id,
                self.registry.machine["managed_networks"][request.network_id],
            )
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    def _enroll_goal(self, goal: EnrollSensor.Goal) -> GoalResponse:
        with self.lock:
            observed = self.observations.get(goal.candidate_id)
            if not observed:
                return GoalResponse.REJECT
            network_id = self._network_for_observation(observed[0])
            requested_network = goal.network_id or network_id
            return (
                GoalResponse.ACCEPT
                if network_id and requested_network == network_id
                else GoalResponse.REJECT
            )

    async def _enroll(self, goal_handle):
        result = EnrollSensor.Result()
        goal = goal_handle.request
        claimed_network = ""
        try:
            with self.lock:
                if goal.candidate_id in self.enrollment_active:
                    raise RegistryError("enrollment is already running for this sensor")
                self.enrollment_active.add(goal.candidate_id)
                observation = dict(self.observations[goal.candidate_id][0])
            sensor_id = self.registry.sensor_id_for_observation(observation)
            if sensor_id in self.registry.inventory["sensors"]:
                raise RegistryError(f"sensor {sensor_id} is already enrolled")
            observed_network = self._network_for_observation(observation)
            if not observed_network:
                raise RegistryError("sensor is not connected through an approved managed port")
            network_id = goal.network_id or observed_network
            if network_id != observed_network:
                raise RegistryError(
                    f"sensor is connected through {observed_network}, not {network_id}"
                )
            with self.lock:
                if network_id in self.enrollment_network_active:
                    raise RegistryError(f"another enrollment is active on {network_id}")
                self.enrollment_network_active.add(network_id)
                claimed_network = network_id
            allocation = self.registry.allocate(network_id, goal.requested_address)
            feedback = EnrollSensor.Feedback()
            feedback.stage = "provisioning"
            feedback.detail = f"Assigning {allocation.address} on {network_id}"
            feedback.progress_percent = 30
            goal_handle.publish_feedback(feedback)

            client = self.provision_clients[observation["provider"]]
            if not client.wait_for_service(timeout_sec=5.0):
                raise RegistryError(f"provider {observation['provider']} is unavailable")
            request = ProvisionSensor.Request()
            request.candidate_id = goal.candidate_id
            request.sensor_id = sensor_id
            request.serial = observation["serial"]
            request.mac_address = observation.get("mac_address", "")
            request.network_id = network_id
            request.target_address = allocation.address
            request.subnet_mask = allocation.subnet_mask
            request.gateway = allocation.gateway
            response = await client.call_async(request)
            if not response.success:
                raise RegistryError(response.message)
            feedback.stage = "persisting"
            feedback.detail = "Camera settings verified; saving inventory"
            feedback.progress_percent = 80
            goal_handle.publish_feedback(feedback)
            self.registry.enroll(
                observation, allocation, goal.display_name, goal.location_label
            )
            with self.lock:
                self.generation += 1
            self._publish_state()
            result.success = True
            result.message = f"Enrolled {sensor_id} at {allocation.address}"
            result.sensor = self._sensor_message(
                sensor_id, self.registry.inventory["sensors"][sensor_id]
            )
            goal_handle.succeed()
        except (RegistryError, KeyError, ValueError) as error:
            result.success = False
            result.message = str(error)
            goal_handle.abort()
        finally:
            with self.lock:
                self.enrollment_active.discard(goal.candidate_id)
                if claimed_network:
                    self.enrollment_network_active.discard(claimed_network)
        return result

    def _infer_network(self, observation: dict[str, Any]) -> str:
        network_id = self._network_for_observation(observation)
        if network_id:
            return network_id
        raise RegistryError("sensor is not connected through an approved managed port")

    def _update_metadata(self, request, response):
        try:
            pose = _pose_to_dict(request.pose, request.parent_frame) if request.has_pose else None
            self.registry.update_sensor(request.sensor_id, {
                "display_name": request.display_name,
                "location_label": request.location_label,
                "pose": pose,
                "calibration_url": request.calibration_url,
                "enabled": request.enabled,
            })
            with self.lock:
                self.generation += 1
            response.success = True
            response.message = "Sensor metadata updated"
            response.sensor = self._sensor_message(
                request.sensor_id, self.registry.inventory["sensors"][request.sensor_id]
            )
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    def _forget_sensor(self, request, response):
        try:
            runtime = self.runtime.get(request.sensor_id)
            if runtime and runtime.online and not request.force:
                raise RegistryError("sensor is online; set force=true to stop and archive it")
            self.registry.forget(request.sensor_id)
            self.sensor_modes.pop(request.sensor_id, None)
            with self.lock:
                self.generation += 1
            response.success = True
            response.message = "Sensor unenrolled and archived; hardware IP was left unchanged"
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    def _update_known_sensor(self, request, response):
        try:
            if len(request.provider_settings_json.encode("utf-8")) > 64 * 1024:
                raise RegistryError("provider_settings exceeds 64 KiB")
            settings = json.loads(request.provider_settings_json or "{}")
            if not isinstance(settings, dict):
                raise RegistryError("provider_settings must be a JSON object")
            profile = str(request.camera_profile)
            if profile and profile not in self.camera_profiles:
                raise RegistryError(f"unknown camera profile {profile}")
            self.registry.update_known_sensor(request.sensor_id, {
                "notes": request.notes,
                "camera_profile": profile,
                "provider_settings": settings,
            })
            with self.lock:
                self.catalog_dirty = False
                self.generation += 1
            snapshot = self.registry.snapshot()
            response.success = True
            response.message = "Known sensor updated"
            response.sensor = self._known_sensor_message(
                request.sensor_id, snapshot["known_sensors"][request.sensor_id], snapshot
            )
            self._publish_state()
        except (RegistryError, json.JSONDecodeError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _purge_known_sensor(self, request, response):
        try:
            self.registry.purge_known_sensor(request.sensor_id)
            with self.lock:
                self.catalog_dirty = False
                self.known_checkpoint_at.pop(request.sensor_id, None)
                self.generation += 1
            response.success = True
            response.message = "Known sensor permanently deleted"
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    def _set_mode(self, request, response):
        try:
            if request.mode not in VALID_OPERATING_MODES:
                raise RegistryError("mode must be idle, preview, or capture")
            if request.target_kind == "group":
                if request.target_id not in self.registry.inventory["sync_groups"]:
                    raise RegistryError(f"unknown sync group {request.target_id}")
                self.group_modes[request.target_id] = request.mode
                self.group_capture_intervals[request.target_id] = 0
            elif request.target_kind == "sensor":
                if request.target_id not in self.registry.inventory["sensors"]:
                    raise RegistryError(f"unknown sensor {request.target_id}")
                if self._group_for_sensor(request.target_id):
                    raise RegistryError("set the mode on the sensor's synchronization group")
                if request.mode == "capture":
                    raise RegistryError("capture mode requires a synchronization group")
                self.sensor_modes[request.target_id] = request.mode
            else:
                raise RegistryError("target_kind must be sensor or group")
            with self.lock:
                self.generation += 1
            response.success = True
            response.message = f"Requested {request.mode} mode"
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    def _prepare_capture_groups(self, request, response):
        try:
            if request.interval_ms < 100 or request.interval_ms > 86_400_000:
                raise RegistryError("interval_ms must be between 100 and 86400000")
            group_ids = list(request.group_ids)
            if not group_ids:
                raise RegistryError("at least one synchronization group is required")
            if len(set(group_ids)) != len(group_ids):
                raise RegistryError("group_ids contains a duplicate")
            for group_id in group_ids:
                if group_id not in self.registry.inventory["sync_groups"]:
                    raise RegistryError(f"unknown sync group {group_id}")
            with self.lock:
                for group_id in group_ids:
                    self.group_modes[group_id] = "capture"
                    self.group_capture_intervals[group_id] = int(request.interval_ms)
                self.generation += 1
            response.accepted = True
            response.message = (
                f"Preparing {len(group_ids)} group(s) for {request.interval_ms} ms capture"
            )
            self._publish_state()
        except RegistryError as error:
            response.accepted = False
            response.message = str(error)
        return response

    def _upsert_group(self, request, response):
        try:
            self.registry.upsert_group(
                request.group_id,
                request.provider,
                request.member_ids,
                request.missing_policy,
                "Action0",
                request.preview_rate_hz,
                "",
            )
            self.group_modes.setdefault(request.group_id, self.default_group_mode)
            self.group_capture_intervals.setdefault(request.group_id, 0)
            with self.lock:
                self.generation += 1
            response.success = True
            response.message = "Synchronization group saved in idle mode"
            response.group = self._sync_group_message(
                request.group_id, self.registry.inventory["sync_groups"][request.group_id]
            )
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    def _delete_group(self, request, response):
        try:
            if self.group_modes.get(request.group_id, self.default_group_mode) != "idle":
                raise RegistryError("set the group to idle before deleting it")
            self.registry.delete_group(request.group_id)
            self.group_modes.pop(request.group_id, None)
            self.group_capture_intervals.pop(request.group_id, None)
            with self.lock:
                self.generation += 1
            response.success = True
            response.message = "Synchronization group deleted"
            self._publish_state()
        except RegistryError as error:
            response.success = False
            response.message = str(error)
        return response

    async def _request_group_capture(
        self, group_id: str, request_id: str, *, trigger_only: bool,
        has_requested_time: bool = False, requested_time=None,
    ):
        group = self.registry.inventory["sync_groups"].get(group_id)
        if not group:
            raise RegistryError(f"unknown sync group {group_id}")
        if self.group_modes.get(group_id, self.default_group_mode) != "capture":
            raise RegistryError("synchronization group is not in capture mode")
        runtime_provider = self._runtime_group_provider(group)
        client = self.capture_clients[runtime_provider]
        if not client.wait_for_service(timeout_sec=3.0):
            raise RegistryError(f"provider {runtime_provider} is unavailable")
        provider_request = ProviderCapture.Request()
        provider_request.group_id = group_id
        provider_request.request_id = request_id
        provider_request.member_ids = list(group["members"])
        provider_request.missing_policy = group["missing_policy"]
        provider_request.preferred_master_id = ""
        provider_request.trigger_only = trigger_only
        provider_request.has_requested_time = has_requested_time
        if has_requested_time and requested_time is not None:
            provider_request.requested_time = requested_time
        try:
            # Await the provider response instead of blocking an executor thread.
            # At 2 Hz, blocking here exhausted all four manager threads before
            # the response callbacks could run, deadlocking the service proxy.
            return await client.call_async(provider_request)
        except Exception as error:
            raise RegistryError(str(error)) from error

    async def _capture_group(self, request, response):
        try:
            provider_response = await self._request_group_capture(
                request.group_id,
                request.request_id,
                trigger_only=request.trigger_only,
                has_requested_time=request.has_requested_time,
                requested_time=request.requested_time,
            )
            response.success = provider_response.success
            response.message = provider_response.message
            response.scheduled_time = provider_response.scheduled_time
            response.capture_id = provider_response.capture_id
            response.participating_sensor_ids = provider_response.participating_sensor_ids
            response.missing_sensor_ids = provider_response.missing_sensor_ids
            response.trigger_span_ns = provider_response.trigger_span_ns
            response.exposure_skew_ns = provider_response.exposure_skew_ns
            response.within_tolerance = provider_response.within_tolerance
            response.camera_timings = provider_response.camera_timings
        except (RegistryError, TimeoutError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _trigger_goal(self, goal_request):
        if not goal_request.group_id:
            return GoalResponse.REJECT
        if goal_request.request_id and not re.fullmatch(
            r"[A-Za-z0-9._-]{1,128}", goal_request.request_id
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _trigger_group(self, goal_handle):
        result = TriggerGroup.Result()
        feedback = TriggerGroup.Feedback()
        feedback.stage = "arming"
        feedback.detail = "Arming grouped camera workers"
        feedback.progress_percent = 10
        goal_handle.publish_feedback(feedback)
        try:
            response = await self._request_group_capture(
                goal_handle.request.group_id,
                goal_handle.request.request_id,
                trigger_only=True,
            )
            result.success = response.success
            result.message = response.message
            result.capture_id = response.capture_id
            result.scheduled_time = response.scheduled_time
            result.participating_sensor_ids = response.participating_sensor_ids
            result.missing_sensor_ids = response.missing_sensor_ids
            result.trigger_span_ns = response.trigger_span_ns
            result.exposure_skew_ns = response.exposure_skew_ns
            result.within_tolerance = response.within_tolerance
            result.camera_timings = response.camera_timings
            if response.success:
                feedback.stage = "published"
                feedback.detail = "Triggered frames published"
                feedback.progress_percent = 100
                goal_handle.publish_feedback(feedback)
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except (RegistryError, TimeoutError) as error:
            result.success = False
            result.message = str(error)
            goal_handle.abort()
        return result

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = InventoryManager()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except (RegistryError, OSError) as error:
        if node:
            node.get_logger().fatal(str(error))
        else:
            print(f"vixel inventory manager: {error}")
        raise
    finally:
        try:
            if executor:
                executor.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass
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


if __name__ == "__main__":
    main()
