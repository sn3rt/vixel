from __future__ import annotations

import pytest

from vixel_manager.camera_profiles import (
    CameraProfileError,
    load_camera_profiles,
    resolve_camera_settings,
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
