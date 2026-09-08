from __future__ import annotations

import copy
import json
import os
import pathlib
import re
import tempfile
from typing import Any

import yaml


class CameraProfileError(RuntimeError):
    pass


PROFILE_NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]*")
PORTABLE_SETTING_NAMES = {
    "pixel_format", "width", "height", "offset_x", "offset_y",
    "binning_horizontal", "binning_vertical", "decimation_horizontal",
    "decimation_vertical", "reverse_x", "reverse_y", "exposure_auto",
    "exposure_us", "exposure_auto_limit_auto", "exposure_auto_upper_us",
    "gain_auto", "gain_db", "gain_auto_upper_db", "auto_brightness_target",
    "balance_white_auto", "gamma", "black_level", "metering_rate_hz",
    "capture_png_compression", "frame_rate_hz", "packet_size",
    "packet_delay_ns", "trigger_overlap", "transfer_control_mode",
    "preview_width", "preview_format", "png_compression", "jpeg_quality",
}


def _merge_settings(target: dict[str, Any], values: dict[str, Any]) -> None:
    for key, value in values.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge_settings(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def normalize_camera_profile(
    value: dict[str, Any], *, fallback_name: str = "", source: str = "managed",
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CameraProfileError("camera profile must be a mapping")
    try:
        version = int(value.get("schema_version", 1))
    except (TypeError, ValueError) as error:
        raise CameraProfileError(
            "camera profile schema_version must be an integer"
        ) from error
    if version not in {1, 2}:
        raise CameraProfileError("camera profile has unsupported schema_version")
    name = str(value.get("name", fallback_name)).strip()
    if not PROFILE_NAME_PATTERN.fullmatch(name):
        raise CameraProfileError(
            "camera profile name must match [a-z][a-z0-9_]*"
        )
    settings = value.get("settings", {})
    if not isinstance(settings, dict):
        raise CameraProfileError(f"camera profile {name} settings must be a mapping")
    scope = str(value.get("scope", "portable" if version == 2 else "model")).lower()
    if scope not in {"portable", "model"}:
        raise CameraProfileError(f"camera profile {name} scope must be portable or model")
    compatibility = value.get("compatibility", {}) or {}
    if not isinstance(compatibility, dict):
        raise CameraProfileError(f"camera profile {name} compatibility must be a mapping")
    vendor = str(compatibility.get("vendor", "")).strip()
    model = str(compatibility.get("model", "")).strip()
    if scope == "portable":
        unsupported = sorted(set(settings) - PORTABLE_SETTING_NAMES)
        if unsupported:
            raise CameraProfileError(
                f"portable camera profile {name} contains non-portable settings: "
                + ", ".join(unsupported)
            )
        vendor = ""
        model = ""
    elif version == 2 and (not vendor or not model):
        raise CameraProfileError(
            f"model camera profile {name} requires compatibility.vendor and compatibility.model"
        )
    return {
        "schema_version": 2,
        "name": name,
        "description": str(value.get("description", "")),
        "scope": scope,
        "compatibility": {"vendor": vendor, "model": model},
        "settings": copy.deepcopy(settings),
        "source": source,
        "source_path": str(value.get("source_path", "")),
    }


def load_camera_profiles(
    directory: str, managed_file: str = "",
) -> dict[str, dict[str, Any]]:
    """Load read-only administrator profiles and browser-managed profiles."""
    root = pathlib.Path(directory).expanduser()
    if root.exists() and not root.is_dir():
        raise CameraProfileError(f"camera profile path is not a directory: {root}")
    profiles: dict[str, dict[str, Any]] = {}
    paths = sorted((*root.glob("*.yaml"), *root.glob("*.yml"))) if root.exists() else []
    for path in paths:
        try:
            value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as error:
            raise CameraProfileError(f"cannot load camera profile {path}: {error}") from error
        profile = normalize_camera_profile(value, fallback_name=path.stem, source="admin")
        profile["source_path"] = str(path)
        if profile["name"] in profiles:
            raise CameraProfileError(f"duplicate camera profile {profile['name']}")
        profiles[profile["name"]] = profile
    if managed_file:
        managed_path = pathlib.Path(managed_file).expanduser()
        if managed_path.exists():
            try:
                document = yaml.safe_load(managed_path.read_text(encoding="utf-8")) or {}
            except (OSError, yaml.YAMLError) as error:
                raise CameraProfileError(
                    f"cannot load managed camera profiles {managed_path}: {error}"
                ) from error
            if not isinstance(document, dict):
                raise CameraProfileError(
                    "managed camera profiles document must be a mapping"
                )
            values = document.get("profiles", {})
            if not isinstance(values, dict):
                raise CameraProfileError("managed camera profiles must contain a profiles mapping")
            for key, value in values.items():
                profile = normalize_camera_profile(value, fallback_name=str(key), source="managed")
                if profile["name"] != str(key):
                    raise CameraProfileError(
                        f"managed camera profile key {key} does not match name "
                        f"{profile['name']}"
                    )
                if profile["name"] in profiles:
                    raise CameraProfileError(f"duplicate camera profile {profile['name']}")
                profiles[profile["name"]] = profile
    return profiles


def save_managed_camera_profiles(
    path: str, profiles: dict[str, dict[str, Any]],
) -> None:
    target = pathlib.Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    values: dict[str, Any] = {}
    for name, profile in sorted(profiles.items()):
        if profile.get("source") != "managed":
            continue
        normalized = normalize_camera_profile(profile, fallback_name=name, source="managed")
        normalized.pop("source", None)
        normalized.pop("source_path", None)
        values[name] = normalized
    document = {"schema_version": 1, "profiles": values}
    try:
        fd, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=str(target.parent))
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    except OSError as error:
        raise CameraProfileError(f"cannot save managed camera profiles {target}: {error}") from error


def camera_profile_compatible(profile: dict[str, Any], sensor: dict[str, Any]) -> bool:
    if profile.get("scope") == "portable":
        return True
    compatibility = profile.get("compatibility", {})
    return (
        str(compatibility.get("vendor", "")).casefold()
        == str(sensor.get("vendor", "")).casefold()
        and str(compatibility.get("model", "")).casefold()
        == str(sensor.get("model", "")).casefold()
    )


def camera_profiles_json(profiles: dict[str, dict[str, Any]]) -> str:
    return json.dumps(
        [profiles[name] for name in sorted(profiles)],
        separators=(",", ":"), sort_keys=True,
    )


def resolve_camera_settings(
    defaults: dict[str, Any],
    profiles: dict[str, dict[str, Any]],
    profile_name: str,
    overrides: dict[str, Any],
) -> dict[str, Any]:
    if not isinstance(overrides, dict):
        raise CameraProfileError("camera settings overrides must be a mapping")
    result = copy.deepcopy(defaults)
    if profile_name:
        profile = profiles.get(profile_name)
        if profile is None:
            raise CameraProfileError(f"unknown camera profile {profile_name}")
        _merge_settings(result, profile["settings"])
    _merge_settings(result, overrides)
    return result
