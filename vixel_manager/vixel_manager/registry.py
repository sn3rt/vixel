from __future__ import annotations

import copy
import ipaddress
import os
import pathlib
import re
import shutil
import tempfile
import threading
from datetime import datetime, timezone
from dataclasses import dataclass
from typing import Any, Iterable

import yaml


MACHINE_SCHEMA_VERSION = 1
INVENTORY_SCHEMA_VERSION = 3
MAX_KNOWN_CHANGES = 50
VALID_CATALOG_STATES = {"observed", "enrolled", "archived", "retired"}
VALID_NETWORK_MODES = {"direct", "switched"}
VALID_MISSING_POLICIES = {"strict", "degraded"}
VALID_OPERATING_MODES = {"idle", "preview", "capture"}
VALID_TRIGGER_SOURCES = {"FreeRun", "Software", "Action0"}
PLACEHOLDER_PREFIXES = ("REPLACE_", "UNKNOWN", "TODO")


class RegistryError(RuntimeError):
    pass


def stable_sensor_id(provider: str, serial: str) -> str:
    raw = f"{provider}_{serial}".lower()
    value = re.sub(r"[^a-z0-9_]", "_", raw)
    value = re.sub(r"_+", "_", value).strip("_")
    if not value or not value[0].isalpha():
        value = f"sensor_{value}"
    return value


def _identity_part(value: str) -> str:
    return re.sub(r"[^a-z0-9]", "", str(value).lower())


def _normalized_mac(value: str) -> str:
    compact = _identity_part(value)
    return compact if len(compact) == 12 else ""


def _vendor_identity(value: str) -> str:
    normalized = _identity_part(value)
    aliases = {
        "lucid": "lucid",
        "ids": "ids",
        "basler": "basler",
        "alliedvision": "alliedvision",
        "teledyneflir": "teledyneflir",
    }
    for token, canonical in aliases.items():
        if token in normalized:
            return canonical
    return normalized


def hardware_key(sensor: dict[str, Any]) -> str:
    mac = _normalized_mac(sensor.get("mac_address", ""))
    if mac:
        return f"mac:{mac}"
    vendor = _vendor_identity(sensor.get("vendor", sensor.get("provider", "")))
    serial = _identity_part(sensor.get("serial", ""))
    return f"vendor:{vendor}:serial:{serial}"


def new_sensor_id(observation: dict[str, Any]) -> str:
    """Generate an ID for new hardware; existing IDs are deliberately immutable."""
    if (
        observation.get("kind", "camera") == "camera"
        and observation.get("provider") == "genicam"
    ):
        vendor = observation.get("vendor", observation.get("provider", "camera"))
        return stable_sensor_id(f"camera_{vendor}", observation["serial"])
    return stable_sensor_id(observation["provider"], observation["serial"])


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def _known_from_sensor(
    sensor_id: str,
    sensor: dict[str, Any],
    *,
    catalog_state: str,
    timestamp: str,
) -> dict[str, Any]:
    return {
        "provider": sensor.get("provider", ""),
        "kind": sensor.get("kind", "camera"),
        "vendor": sensor.get("vendor", sensor.get("provider", "")),
        "model": sensor.get("model", ""),
        "serial": sensor.get("serial", ""),
        "mac_address": sensor.get("mac_address", ""),
        "capabilities": list(sensor.get("capabilities", [])),
        "catalog_state": catalog_state,
        "first_seen_at": timestamp,
        "last_seen_at": timestamp,
        "sighting_count": 0,
        "latest_observation": {},
        "changes": [],
        "notes": "",
        "camera_profile": "",
        "provider_settings": {},
        "hardware_key": hardware_key(sensor),
        "last_provider": sensor.get("provider", ""),
        "last_configuration": copy.deepcopy(sensor),
        "replaced_by": sensor.get("replaced_by", ""),
    }


def migrate_inventory(data: dict[str, Any], timestamp: str | None = None) -> dict[str, Any]:
    """Return schema v3 while preserving all historical sensor IDs."""
    result = copy.deepcopy(data)
    version = int(result.get("schema_version", 1))
    if version == INVENTORY_SCHEMA_VERSION:
        return result
    if version not in {1, 2}:
        raise RegistryError(f"unsupported inventory schema_version {version}")
    migrated_at = timestamp or _timestamp()
    if version == 1:
        known_sensors = result.setdefault("known_sensors", {})
        for sensor_id, sensor in (result.get("sensors") or {}).items():
            known_sensors.setdefault(
                sensor_id,
                _known_from_sensor(
                    sensor_id, sensor, catalog_state="enrolled", timestamp=migrated_at
                ),
            )
        for sensor_id, sensor in (result.pop("retired_sensors", {}) or {}).items():
            retired_at = str(sensor.get("retired_at", migrated_at))
            known_sensors.setdefault(
                sensor_id,
                _known_from_sensor(
                    sensor_id, sensor, catalog_state="retired", timestamp=retired_at
                ),
            )
    for sensor in (result.get("sensors") or {}).values():
        sensor.setdefault("hardware_key", hardware_key(sensor))
    for sensor in (result.get("known_sensors") or {}).values():
        sensor.setdefault("hardware_key", hardware_key(sensor))
        sensor.setdefault("last_provider", sensor.get("provider", ""))
    result["schema_version"] = INVENTORY_SCHEMA_VERSION
    return result


def _read_yaml(path: pathlib.Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream) or {}
    except OSError as error:
        raise RegistryError(f"cannot read {path}: {error}") from error
    except yaml.YAMLError as error:
        raise RegistryError(f"invalid YAML in {path}: {error}") from error
    if not isinstance(value, dict):
        raise RegistryError(f"{path} must contain a YAML mapping")
    return value


def validate_machine(data: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(data)
    if result.get("schema_version", MACHINE_SCHEMA_VERSION) != MACHINE_SCHEMA_VERSION:
        raise RegistryError("unsupported machine schema_version")
    result["schema_version"] = MACHINE_SCHEMA_VERSION
    networks = result.setdefault("managed_networks", {})
    providers = result.setdefault("providers", {})
    recording = result.setdefault("recording", {})
    camera_profiles = result.setdefault("camera_profiles", {})
    defaults = result.setdefault("defaults", {})
    if (
        not isinstance(networks, dict)
        or not isinstance(providers, dict)
        or not isinstance(recording, dict)
        or not isinstance(camera_profiles, dict)
    ):
        raise RegistryError(
            "managed_networks, providers, recording, and camera_profiles must be mappings"
        )
    camera_profiles.setdefault("directory", "/etc/vixel/camera-profiles")
    if not str(camera_profiles["directory"]).strip():
        raise RegistryError("camera_profiles.directory must not be empty")
    # Releases before the generic backend stored the same camera settings under
    # providers.lucid. Keep existing machines bootable while presenting only
    # the generic provider to the rest of the runtime.
    if "genicam" not in providers and isinstance(providers.get("lucid"), dict):
        legacy = providers.pop("lucid")
        legacy_imaging = legacy.get("imaging", {})
        if not isinstance(legacy_imaging, dict):
            legacy_imaging = {}
        providers["genicam"] = {
            "vendor_allowlist": [],
            "discovery_period_ms": legacy.get("discovery_period_ms", 2000),
            "image_timeout_ms": min(int(legacy.get("image_timeout_ms", 1000)), 1000),
            "buffer_count": 16,
            "socket_buffer_bytes": 33554432,
            "imaging": copy.deepcopy(legacy_imaging),
        }
        providers["genicam"]["imaging"].setdefault("frame_rate_hz", 10.0)
    recording.setdefault("root_directory", "/var/lib/vixel/captures")
    recording.setdefault("minimum_free_bytes", 5 * 1024 * 1024 * 1024)
    recording.setdefault("capture_timeout_ms", 10000)
    recording.setdefault("recent_limit", 100)
    if not str(recording["root_directory"]).strip():
        raise RegistryError("recording.root_directory must not be empty")
    if int(recording["minimum_free_bytes"]) < 0:
        raise RegistryError("recording.minimum_free_bytes must not be negative")
    if not 1000 <= int(recording["capture_timeout_ms"]) <= 60000:
        raise RegistryError("recording.capture_timeout_ms must be between 1000 and 60000")
    if not 1 <= int(recording["recent_limit"]) <= 1000:
        raise RegistryError("recording.recent_limit must be between 1 and 1000")
    defaults.setdefault("preview_rate_hz", 2.0)
    defaults.setdefault("preview_width", 960)
    defaults.setdefault("jpeg_quality", 70)
    defaults.setdefault("startup_mode", "idle")
    if defaults["startup_mode"] not in VALID_OPERATING_MODES:
        raise RegistryError("defaults.startup_mode must be idle, preview, or capture")
    if not 0.0 < float(defaults["preview_rate_hz"]) <= 10.0:
        raise RegistryError("defaults.preview_rate_hz must be greater than 0 and at most 10")
    if int(defaults["preview_width"]) <= 0:
        raise RegistryError("defaults.preview_width must be positive")
    if not 1 <= int(defaults["jpeg_quality"]) <= 100:
        raise RegistryError("defaults.jpeg_quality must be between 1 and 100")

    interfaces: set[str] = set()
    subnets: list[ipaddress.IPv4Network] = []
    for network_id, network in networks.items():
        if not isinstance(network, dict):
            raise RegistryError(f"managed network {network_id} must be a mapping")
        interface = str(network.get("interface", ""))
        interface_mac = str(network.get("interface_mac", "")).lower()
        mode = str(network.get("mode", ""))
        if not interface or interface in interfaces:
            raise RegistryError(f"managed network {network_id} has an empty or duplicate interface")
        if mode not in VALID_NETWORK_MODES:
            raise RegistryError(f"managed network {network_id} mode must be direct or switched")
        if "approved" not in network or type(network["approved"]) is not bool:
            raise RegistryError(
                f"managed network {network_id} approved must be an explicit boolean"
            )
        if "auto_enroll" in network and type(network["auto_enroll"]) is not bool:
            raise RegistryError(
                f"managed network {network_id} auto_enroll must be a boolean"
            )
        interfaces.add(interface)
        network.setdefault("interface_mac", interface_mac)
        network.setdefault("auto_enroll", False)
        try:
            host = ipaddress.ip_interface(str(network["host_cidr"]))
        except (KeyError, ValueError) as error:
            raise RegistryError(f"managed network {network_id} has invalid host_cidr") from error
        if not isinstance(host, ipaddress.IPv4Interface):
            raise RegistryError(f"managed network {network_id} must use IPv4")
        for previous in subnets:
            if previous.overlaps(host.network):
                raise RegistryError(f"managed network {network_id} overlaps {previous}")
        subnets.append(host.network)
        pool = network.get("address_pool", {})
        try:
            start = ipaddress.ip_address(str(pool["start"]))
            end = ipaddress.ip_address(str(pool["end"]))
        except (KeyError, ValueError) as error:
            raise RegistryError(f"managed network {network_id} has an invalid address_pool") from error
        if start not in host.network or end not in host.network or int(start) > int(end):
            raise RegistryError(f"managed network {network_id} pool is outside its subnet")
        if host.ip in (start, end) or int(start) <= int(host.ip) <= int(end):
            raise RegistryError(f"managed network {network_id} pool contains the host address")
        network.setdefault("mtu", 9000)
        network.setdefault("rp_filter", 0)
        network.setdefault("max_devices", 1 if mode == "direct" else int(end) - int(start) + 1)
        network.setdefault("switched_max_devices", int(end) - int(start) + 1)
        network.setdefault("gateway", "0.0.0.0")
        network.setdefault("packet_size", min(int(network["mtu"]), 9000))
        network.setdefault("packet_delay", 100000)
        network.setdefault("transfer_slot_ms", 100)
        if int(network["mtu"]) < 1500 or int(network["max_devices"]) <= 0:
            raise RegistryError(f"managed network {network_id} has invalid MTU or max_devices")
        pool_size = int(end) - int(start) + 1
        if int(network["max_devices"]) > pool_size:
            raise RegistryError(f"managed network {network_id} max_devices exceeds its pool")
        if not 1 <= int(network["switched_max_devices"]) <= pool_size:
            raise RegistryError(
                f"managed network {network_id} switched_max_devices exceeds its pool"
            )
        if mode == "direct" and int(network["max_devices"]) != 1:
            raise RegistryError(f"direct managed network {network_id} must have max_devices 1")
    return result


def validate_inventory(data: dict[str, Any], machine: dict[str, Any]) -> dict[str, Any]:
    result = migrate_inventory(data)
    if result.get("schema_version") != INVENTORY_SCHEMA_VERSION:
        raise RegistryError("unsupported inventory schema_version")
    result["schema_version"] = INVENTORY_SCHEMA_VERSION
    sensors = result.setdefault("sensors", {})
    known_sensors = result.setdefault("known_sensors", {})
    groups = result.setdefault("sync_groups", {})
    port_modes = result.setdefault("port_modes", {})
    history = result.setdefault("history", [])
    if not isinstance(sensors, dict) or not isinstance(groups, dict):
        raise RegistryError("sensors and sync_groups must be mappings")
    if not isinstance(known_sensors, dict):
        raise RegistryError("known_sensors must be a mapping")
    if not isinstance(port_modes, dict) or not isinstance(history, list):
        raise RegistryError("port_modes and history have invalid types")
    for network_id, mode in port_modes.items():
        if network_id not in machine["managed_networks"] or mode not in VALID_NETWORK_MODES:
            raise RegistryError(f"invalid runtime mode for managed network {network_id}")

    identity_keys: set[str] = set()
    addresses: set[tuple[str, str]] = set()
    for sensor_id, sensor in sensors.items():
        if not isinstance(sensor, dict):
            raise RegistryError(f"sensor {sensor_id} must be a mapping")
        provider = str(sensor.get("provider", ""))
        serial = str(sensor.get("serial", ""))
        if not re.fullmatch(r"[a-z][a-z0-9_]*", sensor_id):
            raise RegistryError(f"sensor ID {sensor_id} is not ROS-safe")
        identity = hardware_key(sensor)
        if not provider or not serial or identity in identity_keys:
            raise RegistryError(f"sensor {sensor_id} has an empty or duplicate identity")
        identity_keys.add(identity)
        network_id = str(sensor.get("network_id", ""))
        address = str(sensor.get("assigned_address", ""))
        if network_id:
            if network_id not in machine["managed_networks"]:
                raise RegistryError(f"sensor {sensor_id} references unknown network {network_id}")
            try:
                parsed_address = ipaddress.ip_address(address)
            except ValueError as error:
                raise RegistryError(f"sensor {sensor_id} has invalid assigned_address") from error
            host = ipaddress.ip_interface(machine["managed_networks"][network_id]["host_cidr"])
            if parsed_address not in host.network or parsed_address == host.ip:
                raise RegistryError(f"sensor {sensor_id} address is outside {network_id}")
            if (network_id, address) in addresses:
                raise RegistryError(f"duplicate address {address} on {network_id}")
            addresses.add((network_id, address))
        sensor.setdefault("kind", "camera")
        sensor.setdefault("vendor", provider)
        sensor.setdefault("model", "")
        sensor.setdefault("mac_address", "")
        sensor.setdefault("enabled", True)
        sensor.setdefault("display_name", sensor_id)
        sensor.setdefault("location_label", "unknown")
        sensor.setdefault("pose", None)
        sensor.setdefault("calibration_url", "")
        sensor.setdefault("capabilities", [])
        sensor["hardware_key"] = hardware_key(sensor)

    known_identity_keys: set[tuple[str, str]] = set()
    for sensor_id, sensor in known_sensors.items():
        if not isinstance(sensor, dict):
            raise RegistryError(f"known sensor {sensor_id} must be a mapping")
        provider = str(sensor.get("provider", ""))
        serial = str(sensor.get("serial", ""))
        if not re.fullmatch(r"[a-z][a-z0-9_]*", sensor_id):
            raise RegistryError(f"known sensor ID {sensor_id} is not ROS-safe")
        identity = (provider, serial)
        if not provider or not serial or identity in known_identity_keys:
            raise RegistryError(f"known sensor {sensor_id} has an empty or duplicate identity")
        known_identity_keys.add(identity)
        sensor.setdefault("kind", "camera")
        sensor.setdefault("vendor", provider)
        sensor.setdefault("model", "")
        sensor.setdefault("mac_address", "")
        sensor.setdefault("capabilities", [])
        sensor.setdefault("catalog_state", "observed")
        if sensor["catalog_state"] not in VALID_CATALOG_STATES:
            raise RegistryError(f"known sensor {sensor_id} has invalid catalog_state")
        if sensor_id in sensors:
            sensor["catalog_state"] = "enrolled"
        sensor.setdefault("first_seen_at", "")
        sensor.setdefault("last_seen_at", sensor["first_seen_at"])
        sensor.setdefault("sighting_count", 0)
        sensor.setdefault("latest_observation", {})
        sensor.setdefault("changes", [])
        sensor.setdefault("notes", "")
        sensor.setdefault("camera_profile", "")
        sensor.setdefault("provider_settings", {})
        sensor.setdefault("last_configuration", None)
        sensor.setdefault("replaced_by", "")
        sensor["hardware_key"] = hardware_key(sensor)
        sensor.setdefault("last_provider", provider)
        if not isinstance(sensor["latest_observation"], dict):
            raise RegistryError(f"known sensor {sensor_id} latest_observation must be a mapping")
        if not isinstance(sensor["changes"], list):
            raise RegistryError(f"known sensor {sensor_id} changes must be a list")
        sensor["changes"] = sensor["changes"][-MAX_KNOWN_CHANGES:]
        if not isinstance(sensor["provider_settings"], dict):
            raise RegistryError(f"known sensor {sensor_id} provider_settings must be a mapping")
        if not isinstance(sensor["camera_profile"], str):
            raise RegistryError(f"known sensor {sensor_id} camera_profile must be a string")
        if not isinstance(sensor["last_configuration"], (dict, type(None))):
            raise RegistryError(f"known sensor {sensor_id} last_configuration is invalid")
        try:
            sensor["sighting_count"] = max(0, int(sensor["sighting_count"]))
        except (TypeError, ValueError) as error:
            raise RegistryError(f"known sensor {sensor_id} has invalid sighting_count") from error

    missing_known = sorted(set(sensors) - set(known_sensors))
    if missing_known:
        raise RegistryError(
            f"enrolled sensors missing from known_sensors: {', '.join(missing_known)}"
        )

    member_to_group: dict[str, str] = {}
    for group_id, group in groups.items():
        if not re.fullmatch(r"[a-z][a-z0-9_]*", group_id):
            raise RegistryError(f"sync group ID {group_id} is not ROS-safe")
        if not isinstance(group, dict):
            raise RegistryError(f"sync group {group_id} must be a mapping")
        provider = str(group.get("provider", ""))
        members = list(group.get("members", []))
        if not provider or not members:
            raise RegistryError(f"sync group {group_id} needs a provider and members")
        policy = str(group.get("missing_policy", "strict"))
        if policy not in VALID_MISSING_POLICIES:
            raise RegistryError(f"sync group {group_id} has invalid missing_policy")
        # Group membership defines synchronized capture. Legacy inventories may
        # contain FreeRun or Software; normalize them instead of requiring an
        # operator migration.
        group["trigger_source"] = "Action0"
        if len(members) != len(set(members)):
            raise RegistryError(f"sync group {group_id} contains duplicate members")
        preview_rate = float(group.get("preview_rate_hz", machine["defaults"]["preview_rate_hz"]))
        if not 0.0 < preview_rate <= 10.0:
            raise RegistryError(f"sync group {group_id} has invalid preview_rate_hz")
        for member in members:
            if member not in sensors:
                raise RegistryError(f"sync group {group_id} references unknown sensor {member}")
            if member in member_to_group:
                raise RegistryError(f"sensor {member} belongs to multiple sync groups")
            member_to_group[member] = group_id
        group.setdefault("missing_policy", "strict")
        group["trigger_source"] = "Action0"
        group.setdefault("preview_rate_hz", machine["defaults"]["preview_rate_hz"])
        group["preferred_master_id"] = ""
    return result


@dataclass(frozen=True)
class Allocation:
    network_id: str
    address: str
    subnet_mask: str
    gateway: str


class Registry:
    def __init__(self, machine_path: str, inventory_path: str, legacy_path: str = ""):
        self.machine_path = pathlib.Path(machine_path)
        self.inventory_path = pathlib.Path(inventory_path)
        self.legacy_path = pathlib.Path(legacy_path) if legacy_path else None
        self.lock = threading.RLock()
        if not self.machine_path.exists():
            raise RegistryError(
                f"machine configuration not found: {self.machine_path}; "
                "copy machine.example.yaml to /etc/vixel/machine.yaml and edit it"
            )
        self.machine = validate_machine(_read_yaml(self.machine_path))
        inventory_existed = self.inventory_path.exists()
        if self.inventory_path.exists():
            raw_inventory = _read_yaml(self.inventory_path)
        elif self.legacy_path and self.legacy_path.exists():
            raw_inventory = self._migrate_legacy(_read_yaml(self.legacy_path))
        else:
            raw_inventory = {
                "schema_version": INVENTORY_SCHEMA_VERSION,
                "sensors": {},
                "known_sensors": {},
                "sync_groups": {},
                "port_modes": {},
                "history": [],
            }
        original_version = int(raw_inventory.get("schema_version", 1))
        self.inventory = validate_inventory(raw_inventory, self.machine)
        if not inventory_existed or original_version != INVENTORY_SCHEMA_VERSION:
            self.save()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return copy.deepcopy(self.inventory)

    def save(self) -> None:
        with self.lock:
            validated = validate_inventory(self.inventory, self.machine)
            parent = self.inventory_path.parent
            parent.mkdir(parents=True, exist_ok=True)
            if self.inventory_path.exists():
                shutil.copy2(self.inventory_path, self.inventory_path.with_suffix(".yaml.bak"))
            descriptor, temporary_name = tempfile.mkstemp(
                prefix=f".{self.inventory_path.name}.", dir=parent
            )
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    yaml.safe_dump(validated, stream, sort_keys=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.chmod(temporary_name, 0o640)
                os.replace(temporary_name, self.inventory_path)
                directory_fd = os.open(parent, os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            finally:
                if os.path.exists(temporary_name):
                    os.unlink(temporary_name)
            self.inventory = validated

    def allocate(self, network_id: str, requested: str = "") -> Allocation:
        with self.lock:
            if network_id not in self.machine["managed_networks"]:
                raise RegistryError(f"unknown managed network {network_id}")
            network = self.machine["managed_networks"][network_id]
            if not network["approved"]:
                raise RegistryError(f"managed network {network_id} is not approved")
            host = ipaddress.ip_interface(network["host_cidr"])
            start = ipaddress.ip_address(network["address_pool"]["start"])
            end = ipaddress.ip_address(network["address_pool"]["end"])
            used = {
                ipaddress.ip_address(sensor["assigned_address"])
                for sensor in self.inventory["sensors"].values()
                if sensor.get("network_id") == network_id and sensor.get("assigned_address")
            }
            mode = self.port_mode(network_id)
            capacity = (
                int(network["max_devices"]) if mode == "direct"
                else int(network["switched_max_devices"])
            )
            if len(used) >= capacity:
                raise RegistryError(f"managed network {network_id} reached max_devices")
            if requested:
                candidate = ipaddress.ip_address(requested)
                if candidate not in host.network or not int(start) <= int(candidate) <= int(end):
                    raise RegistryError(f"requested address is outside the pool for {network_id}")
                if candidate in used:
                    raise RegistryError(f"requested address {candidate} is already reserved")
            else:
                candidate = next(
                    (ipaddress.ip_address(value) for value in range(int(start), int(end) + 1)
                     if ipaddress.ip_address(value) not in used),
                    None,
                )
                if candidate is None:
                    raise RegistryError(f"managed network {network_id} has no free address")
            return Allocation(
                network_id=network_id,
                address=str(candidate),
                subnet_mask=str(host.netmask),
                gateway=str(network.get("gateway", "0.0.0.0")),
            )

    def port_mode(self, network_id: str) -> str:
        if network_id not in self.machine["managed_networks"]:
            raise RegistryError(f"unknown managed network {network_id}")
        return str(self.inventory.get("port_modes", {}).get(
            network_id, self.machine["managed_networks"][network_id]["mode"]
        ))

    def port_capacity(self, network_id: str) -> int:
        network = self.machine["managed_networks"][network_id]
        return int(
            network["max_devices"] if self.port_mode(network_id) == "direct"
            else network["switched_max_devices"]
        )

    def set_port_mode(self, network_id: str, mode: str) -> None:
        with self.lock:
            if network_id not in self.machine["managed_networks"]:
                raise RegistryError(f"unknown managed network {network_id}")
            if mode not in VALID_NETWORK_MODES:
                raise RegistryError("port mode must be direct or switched")
            enrolled = [
                sensor_id for sensor_id, sensor in self.inventory["sensors"].items()
                if sensor.get("network_id") == network_id
            ]
            if mode == "direct" and len(enrolled) > 1:
                raise RegistryError(
                    f"cannot set {network_id} to direct while {len(enrolled)} sensors are assigned"
                )
            previous = copy.deepcopy(self.inventory)
            self.inventory.setdefault("port_modes", {})[network_id] = mode
            self._append_history("port_mode", network_id, {"mode": mode})
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def _ensure_known(
        self, sensor_id: str, source: dict[str, Any], catalog_state: str = "observed"
    ) -> dict[str, Any]:
        known = self.inventory["known_sensors"].get(sensor_id)
        if known is None:
            known = _known_from_sensor(
                sensor_id, source, catalog_state=catalog_state, timestamp=_timestamp()
            )
            known["last_configuration"] = None
            self.inventory["known_sensors"][sensor_id] = known
        for key in ("kind", "vendor", "model", "serial", "mac_address", "capabilities"):
            value = source.get(key)
            if value not in (None, "", []):
                known[key] = copy.deepcopy(value)
        known.setdefault("provider", source.get("provider", ""))
        known["last_provider"] = source.get("provider", known.get("last_provider", ""))
        known["hardware_key"] = hardware_key(known)
        return known

    def sensor_id_for_observation(self, observation: dict[str, Any]) -> str:
        """Match hardware independently from the provider currently driving it."""
        with self.lock:
            known_sensors = self.inventory.get("known_sensors", {})
            mac = _normalized_mac(observation.get("mac_address", ""))
            if mac:
                for sensor_id, known in known_sensors.items():
                    if _normalized_mac(known.get("mac_address", "")) == mac:
                        return sensor_id
            vendor = _vendor_identity(
                observation.get("vendor", observation.get("provider", ""))
            )
            serial = _identity_part(observation.get("serial", ""))
            for sensor_id, known in known_sensors.items():
                known_vendor = _vendor_identity(
                    known.get("vendor", known.get("provider", ""))
                )
                if known_vendor == vendor and _identity_part(known.get("serial", "")) == serial:
                    return sensor_id
            candidate = new_sensor_id(observation)
            if candidate not in known_sensors:
                return candidate
            suffix = mac[-6:] if mac else str(len(known_sensors) + 1)
            return f"{candidate}_{suffix}"

    def record_observation(
        self,
        observation: dict[str, Any],
        network_id: str,
        managed: bool,
        *,
        new_session: bool,
        checkpoint: bool,
        timestamp: str | None = None,
    ) -> bool:
        """Update the durable catalogue and return whether it needs saving."""
        with self.lock:
            observed_at = timestamp or _timestamp()
            sensor_id = self.sensor_id_for_observation(observation)
            known = self.inventory["known_sensors"].get(sensor_id)
            created = known is None
            if created:
                known = self._ensure_known(sensor_id, observation)
                known["first_seen_at"] = observed_at
                known["sighting_count"] = 1
            elif new_session:
                known["sighting_count"] = int(known.get("sighting_count", 0)) + 1

            identity_changed: dict[str, dict[str, Any]] = {}
            for key in ("kind", "vendor", "model", "mac_address", "capabilities"):
                value = copy.deepcopy(observation.get(key, [] if key == "capabilities" else ""))
                if value not in ("", []) and known.get(key) != value:
                    identity_changed[key] = {"from": copy.deepcopy(known.get(key)), "to": value}
                    known[key] = value

            latest = {
                "provider": str(observation.get("provider", "")),
                "transport": str(observation.get("transport", "")),
                "interface_name": str(observation.get("interface_name", "")),
                "current_address": str(observation.get("current_address", "")),
                "network_id": network_id,
                "managed": bool(managed),
            }
            previous_latest = known.get("latest_observation", {})
            connection_changed = {
                key: {"from": copy.deepcopy(previous_latest.get(key)), "to": copy.deepcopy(value)}
                for key, value in latest.items()
                if previous_latest.get(key) != value
            }
            meaningful_change = bool(identity_changed or (previous_latest and connection_changed))
            known["latest_observation"] = latest
            known["last_provider"] = str(observation.get("provider", ""))
            known["hardware_key"] = hardware_key(known)
            if sensor_id in self.inventory["sensors"]:
                known["catalog_state"] = "enrolled"
            if meaningful_change:
                known.setdefault("changes", []).append({
                    "timestamp": observed_at,
                    "fields": {**identity_changed, **connection_changed},
                })
                known["changes"] = known["changes"][-MAX_KNOWN_CHANGES:]
            if created or new_session or meaningful_change or checkpoint:
                known["last_seen_at"] = observed_at
            return created or new_session or meaningful_change or checkpoint

    def enroll(self, observation: dict[str, Any], allocation: Allocation,
               display_name: str, location_label: str) -> str:
        with self.lock:
            previous = copy.deepcopy(self.inventory)
            sensor_id = self.sensor_id_for_observation(observation)
            if sensor_id in self.inventory["sensors"]:
                raise RegistryError(f"sensor {sensor_id} is already enrolled")
            known = self._ensure_known(sensor_id, observation)
            if known.get("catalog_state") == "retired":
                raise RegistryError(
                    f"sensor {sensor_id} is retired and cannot be enrolled directly"
                )
            self.inventory["sensors"][sensor_id] = {
                "provider": observation["provider"],
                "kind": observation.get("kind", "camera"),
                "vendor": observation.get("vendor", observation["provider"]),
                "model": observation.get("model", ""),
                "serial": observation["serial"],
                "mac_address": observation.get("mac_address", ""),
                "enabled": True,
                "display_name": display_name or sensor_id,
                "location_label": location_label or "unknown",
                "pose": None,
                "calibration_url": "",
                "network_id": allocation.network_id,
                "assigned_address": allocation.address,
                "capabilities": list(observation.get("capabilities", [])),
                "hardware_key": hardware_key(observation),
            }
            known["catalog_state"] = "enrolled"
            known.pop("archived_at", None)
            known["last_configuration"] = None
            self._append_history("enroll", sensor_id, {
                "network_id": allocation.network_id,
                "assigned_address": allocation.address,
            })
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise
            return sensor_id

    def replace_sensor(self, source_sensor_id: str, observation: dict[str, Any]) -> str:
        with self.lock:
            if source_sensor_id not in self.inventory["sensors"]:
                raise RegistryError(f"unknown enrolled sensor {source_sensor_id}")
            replacement_id = new_sensor_id(observation)
            if replacement_id in self.inventory["sensors"]:
                raise RegistryError(f"sensor {replacement_id} is already enrolled")
            previous = copy.deepcopy(self.inventory)
            source = copy.deepcopy(self.inventory["sensors"].pop(source_sensor_id))
            replacement = copy.deepcopy(source)
            replacement.update({
                "provider": observation["provider"],
                "kind": observation.get("kind", source.get("kind", "camera")),
                "vendor": observation.get("vendor", observation["provider"]),
                "model": observation.get("model", ""),
                "serial": observation["serial"],
                "mac_address": observation.get("mac_address", ""),
                "capabilities": list(observation.get("capabilities", source.get("capabilities", []))),
                "hardware_key": hardware_key(observation),
            })
            replacement_known = self._ensure_known(replacement_id, observation)
            replacement_known["catalog_state"] = "enrolled"
            replacement_known.pop("archived_at", None)
            replacement_known["last_configuration"] = None
            self.inventory["sensors"][replacement_id] = replacement
            for group in self.inventory["sync_groups"].values():
                group["members"] = [
                    replacement_id if member == source_sensor_id else member
                    for member in group["members"]
                ]
                if group.get("preferred_master_id") == source_sensor_id:
                    group["preferred_master_id"] = replacement_id
            source_known = self._ensure_known(source_sensor_id, source)
            source_known["catalog_state"] = "retired"
            source_known["retired_at"] = self._timestamp()
            source_known["replaced_by"] = replacement_id
            source_known["last_configuration"] = source
            self._append_history("replace", replacement_id, {
                "source_sensor_id": source_sensor_id,
                "network_id": replacement["network_id"],
            })
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise
            return replacement_id

    def move_sensor(self, sensor_id: str, allocation: Allocation) -> None:
        with self.lock:
            if sensor_id not in self.inventory["sensors"]:
                raise RegistryError(f"unknown enrolled sensor {sensor_id}")
            previous = copy.deepcopy(self.inventory)
            sensor = self.inventory["sensors"][sensor_id]
            old_network = sensor.get("network_id", "")
            old_address = sensor.get("assigned_address", "")
            sensor["network_id"] = allocation.network_id
            sensor["assigned_address"] = allocation.address
            sensor["location_label"] = "unknown"
            sensor["pose"] = None
            self._append_history("move", sensor_id, {
                "from_network_id": old_network,
                "from_address": old_address,
                "to_network_id": allocation.network_id,
                "to_address": allocation.address,
                "location_requires_review": True,
            })
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _append_history(self, operation: str, sensor_id: str, details: dict[str, Any]) -> None:
        self.inventory.setdefault("history", []).append({
            "timestamp": self._timestamp(),
            "operation": operation,
            "sensor_id": sensor_id,
            **copy.deepcopy(details),
        })

    def update_sensor(self, sensor_id: str, values: dict[str, Any]) -> None:
        with self.lock:
            previous = copy.deepcopy(self.inventory)
            if sensor_id not in self.inventory["sensors"]:
                raise RegistryError(f"unknown enrolled sensor {sensor_id}")
            allowed = {
                "display_name", "location_label", "pose", "calibration_url", "enabled"
            }
            self.inventory["sensors"][sensor_id].update(
                {key: copy.deepcopy(value) for key, value in values.items() if key in allowed}
            )
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def forget(self, sensor_id: str) -> None:
        with self.lock:
            previous = copy.deepcopy(self.inventory)
            if sensor_id not in self.inventory["sensors"]:
                raise RegistryError(f"unknown enrolled sensor {sensor_id}")
            sensor = copy.deepcopy(self.inventory["sensors"].pop(sensor_id))
            known = self._ensure_known(sensor_id, sensor)
            known["catalog_state"] = "archived"
            known["archived_at"] = self._timestamp()
            known["last_configuration"] = sensor
            for group_id in list(self.inventory["sync_groups"]):
                group = self.inventory["sync_groups"][group_id]
                group["members"] = [member for member in group["members"] if member != sensor_id]
                if not group["members"]:
                    del self.inventory["sync_groups"][group_id]
            self._append_history("archive", sensor_id, {
                "network_id": sensor.get("network_id", ""),
                "assigned_address": sensor.get("assigned_address", ""),
            })
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def update_known_sensor(self, sensor_id: str, values: dict[str, Any]) -> None:
        with self.lock:
            if sensor_id not in self.inventory["known_sensors"]:
                raise RegistryError(f"unknown sensor {sensor_id}")
            previous = copy.deepcopy(self.inventory)
            if "provider_settings" in values and not isinstance(values["provider_settings"], dict):
                raise RegistryError("provider_settings must be a JSON object")
            allowed = {"notes", "camera_profile", "provider_settings"}
            self.inventory["known_sensors"][sensor_id].update({
                key: copy.deepcopy(value) for key, value in values.items() if key in allowed
            })
            self._append_history("update_known", sensor_id, {})
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def purge_known_sensor(self, sensor_id: str) -> None:
        with self.lock:
            if sensor_id in self.inventory["sensors"]:
                raise RegistryError("enrolled sensors must be archived before permanent deletion")
            if sensor_id not in self.inventory["known_sensors"]:
                raise RegistryError(f"unknown sensor {sensor_id}")
            previous = copy.deepcopy(self.inventory)
            del self.inventory["known_sensors"][sensor_id]
            self.inventory["history"] = [
                entry for entry in self.inventory.get("history", [])
                if entry.get("sensor_id") != sensor_id
            ]
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def upsert_group(self, group_id: str, provider: str, members: Iterable[str],
                     missing_policy: str, trigger_source: str, preview_rate_hz: float,
                     preferred_master_id: str) -> None:
        with self.lock:
            previous = copy.deepcopy(self.inventory)
            if not re.fullmatch(r"[a-z][a-z0-9_]*", group_id):
                raise RegistryError("group_id must be a lowercase ROS-safe identifier")
            self.inventory["sync_groups"][group_id] = {
                "provider": provider,
                "members": list(members),
                "missing_policy": missing_policy,
                "trigger_source": "Action0",
                "preview_rate_hz": float(preview_rate_hz),
                "preferred_master_id": "",
            }
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def delete_group(self, group_id: str) -> None:
        with self.lock:
            previous = copy.deepcopy(self.inventory)
            if group_id not in self.inventory["sync_groups"]:
                raise RegistryError(f"unknown sync group {group_id}")
            del self.inventory["sync_groups"][group_id]
            try:
                self.save()
            except Exception:
                self.inventory = previous
                raise

    def _migrate_legacy(self, legacy: dict[str, Any]) -> dict[str, Any]:
        migrated: dict[str, Any] = {
            "schema_version": INVENTORY_SCHEMA_VERSION,
            "sensors": {},
            "known_sensors": {},
            "sync_groups": {},
            "port_modes": {},
            "history": [],
        }
        networks_by_role = {
            str(network.get("legacy_role", "")): network_id
            for network_id, network in self.machine["managed_networks"].items()
            if network.get("legacy_role")
        }
        for role, camera in (legacy.get("cameras") or {}).items():
            serial = str((camera or {}).get("serial", ""))
            if not serial or serial.startswith(PLACEHOLDER_PREFIXES):
                continue
            provider = "lucid"
            sensor_id = stable_sensor_id(provider, serial)
            network_id = networks_by_role.get(str(role), "")
            assigned_address = ""
            if network_id:
                legacy_address = self.machine["managed_networks"][network_id].get("legacy_address", "")
                assigned_address = str(legacy_address)
            migrated["sensors"][sensor_id] = {
                "provider": provider,
                "kind": "camera",
                "vendor": "LUCID",
                "model": legacy.get("camera_model", ""),
                "serial": serial,
                "mac_address": "",
                "enabled": True,
                "display_name": str(role).replace("_", " ").title(),
                "location_label": str(role),
                "pose": None,
                "calibration_url": str((camera or {}).get("calibration_url", "")),
                "network_id": network_id,
                "assigned_address": assigned_address,
                "capabilities": ["image", "jpeg_preview"],
            }
            migrated["known_sensors"][sensor_id] = _known_from_sensor(
                sensor_id,
                migrated["sensors"][sensor_id],
                catalog_state="enrolled",
                timestamp=_timestamp(),
            )
        return migrated
