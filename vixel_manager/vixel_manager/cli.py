from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory

from .registry import RegistryError, validate_inventory, validate_machine


def default_machine() -> str:
    override = os.environ.get("VIXEL_MACHINE_FILE")
    if override:
        return override
    system = pathlib.Path("/etc/vixel/machine.yaml")
    return str(system)


def example_machine() -> str:
    return str(
        pathlib.Path(get_package_share_directory("vixel"))
        / "config/machine.example.yaml"
    )


def default_inventory() -> str:
    override = os.environ.get("VIXEL_INVENTORY_FILE")
    if override:
        return override
    system = pathlib.Path("/var/lib/vixel/inventory.yaml")
    if system.exists() or os.access(system.parent, os.W_OK):
        return str(system)
    state_home = pathlib.Path(
        os.environ.get("XDG_STATE_HOME", pathlib.Path.home() / ".local/state")
    )
    return str(state_home / "vixel/inventory.yaml")


def _load(path: str):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _paths(_args) -> int:
    print(f"machine:   {default_machine()}")
    print(f"example:   {example_machine()}")
    print(f"inventory: {default_inventory()}")
    return 0


def _show(args) -> int:
    path = default_machine() if args.which == "machine" else default_inventory()
    print(pathlib.Path(path).read_text(encoding="utf-8"), end="")
    return 0


def _validate(_args) -> int:
    machine = validate_machine(_load(default_machine()))
    validate_inventory(_load(default_inventory()), machine)
    print("Vixel machine and inventory YAML are valid")
    return 0


def _edit(_args) -> int:
    editor = os.environ.get("SUDO_EDITOR") or os.environ.get("EDITOR") or "nano"
    machine = default_machine()
    if pathlib.Path(machine).is_relative_to("/etc"):
        return subprocess.call(["sudoedit", machine], env={**os.environ, "SUDO_EDITOR": editor})
    return subprocess.call([editor, machine])


def _validated_inventory():
    machine = validate_machine(_load(default_machine()))
    return validate_inventory(_load(default_inventory()), machine)


def _inventory_list(args) -> int:
    inventory = _validated_inventory()
    if args.all:
        sensors = inventory.get("known_sensors", {})
        if not sensors:
            print("No known sensors")
            return 0
        for sensor_id, sensor in sensors.items():
            latest = sensor.get("latest_observation", {})
            enrollment = inventory.get("sensors", {}).get(sensor_id, {})
            connection = (
                latest.get("network_id")
                or latest.get("interface_name")
                or enrollment.get("network_id")
                or "-"
            )
            print(
                f"{sensor_id}\t{sensor.get('catalog_state', 'observed')}\t"
                f"{sensor.get('model', '-')}\t{sensor.get('serial', '-')}\t"
                f"{sensor.get('last_seen_at', '-')}\t"
                f"{connection}"
            )
        return 0
    sensors = inventory.get("sensors", {})
    if not sensors:
        print("No enrolled sensors")
        return 0
    for sensor_id, sensor in sensors.items():
        print(
            f"{sensor_id}\t{sensor.get('location_label', 'unknown')}\t"
            f"{sensor.get('network_id', '-')}\t{sensor.get('assigned_address', '-')}"
        )
    return 0


def _inventory_show_sensor(args) -> int:
    inventory = _validated_inventory()
    known = inventory.get("known_sensors", {}).get(args.sensor_id)
    if known is None:
        raise RegistryError(f"unknown sensor {args.sensor_id}")
    value = {"sensor_id": args.sensor_id, **known}
    enrollment = inventory.get("sensors", {}).get(args.sensor_id)
    if enrollment is not None:
        value["active_enrollment"] = enrollment
    print(yaml.safe_dump(value, sort_keys=False), end="")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="vixel")
    subcommands = parser.add_subparsers(required=True)
    config = subcommands.add_parser("config")
    config_commands = config.add_subparsers(required=True)
    for name, callback in (
        ("paths", _paths), ("show", _show), ("validate", _validate), ("edit", _edit)
    ):
        command = config_commands.add_parser(name)
        command.set_defaults(callback=callback)
        if name == "show":
            command.add_argument("which", choices=("machine", "inventory"))
    inventory = subcommands.add_parser("inventory")
    inventory_commands = inventory.add_subparsers(required=True)
    listing = inventory_commands.add_parser("list")
    listing.add_argument("--all", action="store_true", help="include every known sensor")
    listing.set_defaults(callback=_inventory_list)
    showing = inventory_commands.add_parser("show")
    showing.add_argument("sensor_id")
    showing.set_defaults(callback=_inventory_show_sensor)
    args = parser.parse_args(argv)
    try:
        return args.callback(args)
    except (OSError, yaml.YAMLError, RegistryError) as error:
        print(f"vixel: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
