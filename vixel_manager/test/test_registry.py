import pathlib

import pytest
import yaml

from vixel_manager.registry import (
    INVENTORY_SCHEMA_VERSION,
    Registry,
    RegistryError,
    stable_sensor_id,
    validate_machine,
)


MACHINE = {
    "schema_version": 1,
    "managed_networks": {
        "left": {
            "interface": "enp7s0",
            "interface_mac": "02:00:00:00:00:02",
            "approved": True,
            "auto_enroll": True,
            "mode": "direct",
            "host_cidr": "192.168.2.1/24",
            "address_pool": {"start": "192.168.2.10", "end": "192.168.2.20"},
            "legacy_role": "front_left",
            "legacy_address": "192.168.2.11",
        },
        "switch": {
            "interface": "enp10s0",
            "interface_mac": "02:00:00:00:00:03",
            "mode": "switched",
            "host_cidr": "192.168.3.1/24",
            "address_pool": {"start": "192.168.3.10", "end": "192.168.3.20"},
        },
    },
    "providers": {"lucid": {}},
}


def write_yaml(path: pathlib.Path, value):
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def make_registry(tmp_path):
    machine = tmp_path / "machine.yaml"
    inventory = tmp_path / "inventory.yaml"
    write_yaml(machine, MACHINE)
    return Registry(str(machine), str(inventory)), inventory


def test_missing_machine_configuration_has_actionable_error(tmp_path):
    with pytest.raises(RegistryError, match="copy machine.example.yaml"):
        Registry(str(tmp_path / "missing.yaml"), str(tmp_path / "inventory.yaml"))


def observation(serial="TEST0001"):
    return {
        "provider": "lucid",
        "kind": "camera",
        "vendor": "LUCID",
        "model": "TRI032S-C",
        "serial": serial,
        "mac_address": "020000000011",
        "capabilities": ["image", "jpeg_preview"],
    }


def test_stable_sensor_id_is_ros_safe():
    assert stable_sensor_id("LUCID", "23-23 01") == "lucid_23_23_01"


def test_registry_enrolls_allocates_and_writes_atomically(tmp_path):
    registry, inventory_path = make_registry(tmp_path)
    allocation = registry.allocate("left")
    assert allocation.address == "192.168.2.10"
    sensor_id = registry.enroll(observation(), allocation, "Crop camera", "front_left")
    assert sensor_id == "lucid_test0001"
    loaded = yaml.safe_load(inventory_path.read_text(encoding="utf-8"))
    assert loaded["sensors"][sensor_id]["location_label"] == "front_left"


def test_direct_network_maximum_and_duplicate_addresses_are_rejected(tmp_path):
    registry, _ = make_registry(tmp_path)
    registry.enroll(observation("A"), registry.allocate("left"), "A", "unknown")
    with pytest.raises(RegistryError, match="max_devices"):
        registry.allocate("left")


def test_switch_mode_unlocks_the_configured_pool(tmp_path):
    registry, _ = make_registry(tmp_path)
    registry.enroll(observation("A"), registry.allocate("left"), "A", "unknown")
    registry.set_port_mode("left", "switched")
    second = registry.allocate("left")
    assert second.address == "192.168.2.11"


def test_replacement_transfers_slot_metadata_and_group_membership(tmp_path):
    registry, _ = make_registry(tmp_path)
    original = registry.enroll(
        observation("A"), registry.allocate("left"), "Front camera", "front_left"
    )
    registry.upsert_group(
        "pair", "lucid", [original], "strict", "Software", 2.0, original
    )
    replacement = registry.replace_sensor(original, observation("B"))
    assert replacement == "lucid_b"
    record = registry.inventory["sensors"][replacement]
    assert record["assigned_address"] == "192.168.2.10"
    assert record["location_label"] == "front_left"
    assert registry.inventory["sync_groups"]["pair"]["members"] == [replacement]
    assert registry.inventory["sync_groups"]["pair"]["preferred_master_id"] == replacement
    retired = registry.inventory["known_sensors"][original]
    assert retired["catalog_state"] == "retired"
    assert retired["replaced_by"] == replacement


def test_sensor_can_belong_to_only_one_sync_group(tmp_path):
    registry, _ = make_registry(tmp_path)
    sensor_id = registry.enroll(
        observation(), registry.allocate("switch"), "Camera", "unknown"
    )
    registry.upsert_group(
        "group_a", "lucid", [sensor_id], "strict", "Software", 2.0, ""
    )
    with pytest.raises(RegistryError, match="multiple sync groups"):
        registry.upsert_group(
            "group_b", "lucid", [sensor_id], "degraded", "Software", 1.0, ""
        )
    assert "group_b" not in registry.inventory["sync_groups"]


def test_preferred_master_must_be_a_group_member(tmp_path):
    registry, _ = make_registry(tmp_path)
    sensor_id = registry.enroll(
        observation(), registry.allocate("switch"), "Camera", "unknown"
    )
    with pytest.raises(RegistryError, match="preferred master"):
        registry.upsert_group(
            "group_a", "lucid", [sensor_id], "strict", "Software", 2.0,
            "lucid_missing"
        )
    assert registry.inventory["sync_groups"] == {}


def test_existing_group_defaults_to_free_run_and_trigger_source_is_validated(tmp_path):
    registry, _ = make_registry(tmp_path)
    sensor_id = registry.enroll(
        observation(), registry.allocate("switch"), "Camera", "unknown"
    )
    registry.inventory["sync_groups"]["legacy"] = {
        "provider": "lucid",
        "members": [sensor_id],
        "missing_policy": "strict",
        "preview_rate_hz": 2.0,
        "preferred_master_id": "",
    }
    registry.save()
    assert registry.inventory["sync_groups"]["legacy"]["trigger_source"] == "FreeRun"

    with pytest.raises(RegistryError, match="trigger_source"):
        registry.upsert_group(
            "legacy", "lucid", [sensor_id], "strict", "Line0", 2.0, ""
        )


def test_legacy_import_ignores_placeholders(tmp_path):
    machine_path = tmp_path / "machine.yaml"
    inventory_path = tmp_path / "inventory.yaml"
    legacy_path = tmp_path / "legacy.yaml"
    write_yaml(machine_path, MACHINE)
    write_yaml(legacy_path, {
        "camera_model": "TRI032S",
        "cameras": {
            "front_left": {
                "serial": "TEST0001",
                "calibration_url": "package://vixel/front_left.yaml",
            },
            "rear_left": {"serial": "REPLACE_REAR_LEFT_SERIAL"},
        },
    })
    registry = Registry(str(machine_path), str(inventory_path), str(legacy_path))
    assert list(registry.inventory["sensors"]) == ["lucid_test0001"]
    sensor = registry.inventory["sensors"]["lucid_test0001"]
    assert sensor["assigned_address"] == "192.168.2.11"


def test_overlapping_managed_networks_are_rejected():
    broken = yaml.safe_load(yaml.safe_dump(MACHINE))
    broken["managed_networks"]["switch"]["host_cidr"] = "192.168.2.2/24"
    broken["managed_networks"]["switch"]["address_pool"] = {
        "start": "192.168.2.30",
        "end": "192.168.2.40",
    }
    with pytest.raises(RegistryError, match="overlaps"):
        validate_machine(broken)


def test_legacy_lucid_provider_is_migrated_in_memory():
    machine = validate_machine(MACHINE)
    assert "lucid" not in machine["providers"]
    assert machine["providers"]["genicam"]["imaging"]["frame_rate_hz"] == 10.0


def test_recording_configuration_gets_safe_defaults():
    machine = validate_machine(MACHINE)
    assert machine["recording"]["root_directory"] == "/var/lib/vixel/captures"
    assert machine["recording"]["minimum_free_bytes"] == 5 * 1024 * 1024 * 1024
    assert machine["recording"]["capture_timeout_ms"] == 10000


def test_invalid_recording_configuration_is_rejected():
    broken = yaml.safe_load(yaml.safe_dump(MACHINE))
    broken["recording"] = {"png_compression": 12}
    with pytest.raises(RegistryError, match="png_compression"):
        validate_machine(broken)


def test_v1_inventory_is_migrated_with_backup(tmp_path):
    machine_path = tmp_path / "machine.yaml"
    inventory_path = tmp_path / "inventory.yaml"
    write_yaml(machine_path, MACHINE)
    write_yaml(inventory_path, {
        "schema_version": 1,
        "sensors": {
            "lucid_test0001": {
                **observation(),
                "enabled": True,
                "display_name": "Front",
                "location_label": "front",
                "pose": None,
                "calibration_url": "package://front.yaml",
                "network_id": "left",
                "assigned_address": "192.168.2.10",
            }
        },
        "sync_groups": {},
        "port_modes": {},
        "retired_sensors": {
            "lucid_old": {
                **observation("old"),
                "retired_at": "2026-01-01T00:00:00+00:00",
                "replaced_by": "lucid_test0001",
            }
        },
        "history": [],
    })

    registry = Registry(str(machine_path), str(inventory_path))

    assert registry.inventory["schema_version"] == INVENTORY_SCHEMA_VERSION
    assert registry.inventory["known_sensors"]["lucid_test0001"]["catalog_state"] == "enrolled"
    assert registry.inventory["known_sensors"]["lucid_old"]["catalog_state"] == "retired"
    assert yaml.safe_load(inventory_path.with_suffix(".yaml.bak").read_text())["schema_version"] == 1


def test_observations_create_catalogue_and_keep_bounded_changes(tmp_path):
    registry, _ = make_registry(tmp_path)
    observed = {**observation(), "transport": "gige", "interface_name": "enp7s0",
                "current_address": "192.168.2.11"}
    assert registry.record_observation(
        observed, "left", True, new_session=True, checkpoint=True,
        timestamp="2026-07-21T10:00:00+00:00"
    )
    sensor_id = "lucid_test0001"
    known = registry.inventory["known_sensors"][sensor_id]
    assert known["catalog_state"] == "observed"
    assert known["sighting_count"] == 1
    assert known["latest_observation"]["managed"] is True

    for index in range(60):
        changed = {**observed, "current_address": f"192.168.2.{20 + index}"}
        registry.record_observation(
            changed, "left", True, new_session=False, checkpoint=False,
            timestamp=f"2026-07-21T10:{index:02d}:00+00:00"
        )
    assert len(known["changes"]) == 50


def test_archive_requires_explicit_reenrollment_and_purge_is_guarded(tmp_path):
    registry, _ = make_registry(tmp_path)
    sensor_id = registry.enroll(
        observation(), registry.allocate("left"), "Front camera", "front_left"
    )
    with pytest.raises(RegistryError, match="archived"):
        registry.purge_known_sensor(sensor_id)

    registry.forget(sensor_id)
    known = registry.inventory["known_sensors"][sensor_id]
    assert known["catalog_state"] == "archived"
    assert known["last_configuration"]["location_label"] == "front_left"

    registry.enroll(observation(), registry.allocate("left"), "Front camera", "front_left")
    assert registry.inventory["known_sensors"][sensor_id]["catalog_state"] == "enrolled"
    registry.forget(sensor_id)
    registry.purge_known_sensor(sensor_id)
    assert sensor_id not in registry.inventory["known_sensors"]


def test_known_sensor_notes_and_provider_settings_round_trip(tmp_path):
    registry, inventory_path = make_registry(tmp_path)
    registry.record_observation(
        observation(), "", False, new_session=True, checkpoint=True,
        timestamp="2026-07-21T10:00:00+00:00"
    )
    registry.save()
    registry.update_known_sensor("lucid_test0001", {
        "notes": "Spare camera",
        "provider_settings": {"exposure_us": 1200},
    })

    reloaded = Registry(str(registry.machine_path), str(inventory_path))
    known = reloaded.inventory["known_sensors"]["lucid_test0001"]
    assert known["notes"] == "Spare camera"
    assert known["provider_settings"] == {"exposure_us": 1200}


def test_genicam_backend_preserves_existing_lucid_identity_and_metadata(tmp_path):
    registry, _ = make_registry(tmp_path)
    sensor_id = registry.enroll(
        observation(), registry.allocate("left"), "Front crop camera", "front_left"
    )
    registry.update_known_sensor(sensor_id, {
        "notes": "Keep this identity across backend migration",
        "provider_settings": {"exposure_us": 1500},
    })
    generic = {
        **observation(),
        "provider": "genicam",
        "vendor": "Lucid Vision Labs",
        "mac_address": "02:00:00:00:00:11",
    }

    assert registry.sensor_id_for_observation(generic) == sensor_id
    registry.record_observation(
        generic, "left", True, new_session=True, checkpoint=True,
        timestamp="2026-07-22T10:00:00+00:00",
    )

    known = registry.inventory["known_sensors"][sensor_id]
    assert known["notes"] == "Keep this identity across backend migration"
    assert known["provider_settings"] == {"exposure_us": 1500}
    assert known["last_provider"] == "genicam"
    assert registry.inventory["sensors"][sensor_id]["location_label"] == "front_left"


def test_new_genicam_camera_gets_vendor_neutral_camera_id(tmp_path):
    registry, _ = make_registry(tmp_path)
    generic = {
        **observation("new-1"),
        "provider": "genicam",
        "vendor": "IDS Imaging",
        "mac_address": "aa:bb:cc:dd:ee:ff",
    }
    assert registry.sensor_id_for_observation(generic) == "camera_ids_imaging_new_1"
