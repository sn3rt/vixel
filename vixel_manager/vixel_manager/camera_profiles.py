from __future__ import annotations

import copy
import pathlib
from typing import Any

import yaml


class CameraProfileError(RuntimeError):
    pass


def _merge_settings(target: dict[str, Any], values: dict[str, Any]) -> None:
    for key, value in values.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge_settings(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def load_camera_profiles(directory: str) -> dict[str, dict[str, Any]]:
    """Load administrator-managed camera profiles from a directory."""
    root = pathlib.Path(directory).expanduser()
    if not root.exists():
        return {}
    if not root.is_dir():
        raise CameraProfileError(f"camera profile path is not a directory: {root}")
    profiles: dict[str, dict[str, Any]] = {}
    for path in sorted((*root.glob("*.yaml"), *root.glob("*.yml"))):
        try:
            value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as error:
            raise CameraProfileError(f"cannot load camera profile {path}: {error}") from error
        if not isinstance(value, dict):
            raise CameraProfileError(f"camera profile {path} must be a mapping")
        if int(value.get("schema_version", 1)) != 1:
            raise CameraProfileError(f"camera profile {path} has unsupported schema_version")
        name = str(value.get("name", path.stem))
        settings = value.get("settings", {})
        if not name or not isinstance(settings, dict):
            raise CameraProfileError(f"camera profile {path} needs a name and settings mapping")
        if name in profiles:
            raise CameraProfileError(f"duplicate camera profile {name}")
        profiles[name] = {
            "name": name,
            "description": str(value.get("description", "")),
            "settings": copy.deepcopy(settings),
            "source": str(path),
        }
    return profiles


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
