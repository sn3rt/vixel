from __future__ import annotations

import pytest

from vixel_manager.camera_profiles import (
    CameraProfileError,
    camera_profile_compatible,
    load_camera_profiles,
    normalize_camera_profile,
    resolve_camera_settings,
    save_managed_camera_profiles,
)


def test_profiles_load_and_resolve_with_expected_precedence(tmp_path):
    (tmp_path / "indoor.yaml").write_text(
        """schema_version: 1
name: indoor
description: Indoor automatic exposure
settings:
  exposure_auto: Continuous
  gain_auto: Continuous
  metering_rate_hz: 2.0
  features:
    ExposureAutoUpperLimit:
      type: float
      value: 10000.0
""",
        encoding="utf-8",
    )
    profiles = load_camera_profiles(str(tmp_path))
    resolved = resolve_camera_settings(
        {"pixel_format": "BGR8", "metering_rate_hz": 0.0},
        profiles,
        "indoor",
        {
            "metering_rate_hz": 4.0,
            "features": {"ExposureAutoUpperLimit": {"value": 8000.0}},
        },
    )
    assert resolved["pixel_format"] == "BGR8"
    assert resolved["exposure_auto"] == "Continuous"
    assert resolved["metering_rate_hz"] == 4.0
    assert resolved["features"]["ExposureAutoUpperLimit"] == {
        "type": "float", "value": 8000.0,
    }


def test_missing_profile_directory_is_empty(tmp_path):
    assert load_camera_profiles(str(tmp_path / "missing")) == {}


def test_unknown_profile_is_rejected():
    with pytest.raises(CameraProfileError, match="unknown camera profile"):
        resolve_camera_settings({}, {}, "missing", {})


def test_managed_profiles_round_trip_without_overwriting_admin_profiles(tmp_path):
    admin = tmp_path / "admin"
    admin.mkdir()
    (admin / "fixed.yaml").write_text(
        """schema_version: 2
name: fixed
scope: portable
settings:
  exposure_auto: Off
  exposure_us: 1500.0
""",
        encoding="utf-8",
    )
    managed = tmp_path / "managed.yaml"
    profiles = load_camera_profiles(str(admin), str(managed))
    profiles["indoor_auto"] = normalize_camera_profile({
        "schema_version": 2,
        "name": "indoor_auto",
        "description": "Shared automatic exposure",
        "scope": "portable",
        "settings": {"exposure_auto": "Continuous"},
    })

    save_managed_camera_profiles(str(managed), profiles)
    reloaded = load_camera_profiles(str(admin), str(managed))

    assert reloaded["fixed"]["source"] == "admin"
    assert reloaded["indoor_auto"]["source"] == "managed"
    assert reloaded["indoor_auto"]["settings"] == {
        "exposure_auto": "Continuous"
    }
    assert "fixed:" not in managed.read_text(encoding="utf-8")


def test_portable_profile_rejects_raw_model_specific_features():
    with pytest.raises(CameraProfileError, match="non-portable settings: features"):
        normalize_camera_profile({
            "schema_version": 2,
            "name": "not_portable",
            "scope": "portable",
            "settings": {
                "features": {
                    "VendorSpecificMode": {"type": "enum", "value": "Fast"}
                }
            },
        })


def test_model_profile_requires_and_checks_vendor_model_compatibility():
    profile = normalize_camera_profile({
        "schema_version": 2,
        "name": "lucid_triton",
        "scope": "model",
        "compatibility": {
            "vendor": "Lucid Vision Labs",
            "model": "TRI032S-C",
        },
        "settings": {
            "features": {"BalanceWhiteAuto": {"type": "enum", "value": "Once"}}
        },
    })

    assert camera_profile_compatible(profile, {
        "vendor": "lucid vision labs", "model": "tri032s-c"
    })
    assert not camera_profile_compatible(profile, {
        "vendor": "IDS", "model": "U3-38J0XCP"
    })

    with pytest.raises(CameraProfileError, match="requires compatibility"):
        normalize_camera_profile({
            "schema_version": 2,
            "name": "missing_model",
            "scope": "model",
            "settings": {},
        })


def test_duplicate_name_across_admin_and_managed_profiles_is_rejected(tmp_path):
    admin = tmp_path / "admin"
    admin.mkdir()
    (admin / "shared.yaml").write_text(
        "schema_version: 2\nname: shared\nscope: portable\nsettings: {}\n",
        encoding="utf-8",
    )
    managed = tmp_path / "managed.yaml"
    managed.write_text(
        """schema_version: 1
profiles:
  shared:
    schema_version: 2
    name: shared
    scope: portable
    settings: {}
""",
        encoding="utf-8",
    )

    with pytest.raises(CameraProfileError, match="duplicate camera profile shared"):
        load_camera_profiles(str(admin), str(managed))
