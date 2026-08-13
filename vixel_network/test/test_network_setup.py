import pathlib

import pytest
import yaml

from vixel_network.network_setup import (
    NetworkSetupError,
    commands_for,
    host_tuning_commands,
    load_networks,
    require_linuxptp,
)


def write_machine(path: pathlib.Path, networks):
    path.write_text(yaml.safe_dump({
        "schema_version": 1,
        "managed_networks": {
            key: {"approved": True, **value} for key, value in networks.items()
        },
    }), encoding="utf-8")


def test_builds_persistent_network_manager_profile(tmp_path):
    machine = tmp_path / "machine.yaml"
    write_machine(machine, {
        "front": {
            "interface": "enp7s0",
            "interface_mac": "02:00:00:00:00:02",
            "host_cidr": "192.168.2.1/24",
            "mtu": 9000,
            "rp_filter": 0,
        }
    })
    network = load_networks(str(machine))[0]
    commands = commands_for(network, profile_exists=False)
    flattened = [item for command in commands for item in command]
    assert "vixel-front" in flattened
    assert "192.168.2.1/24" in flattened
    assert "9000" in flattened
    assert "ipv4.never-default" in flattened
    assert "02:00:00:00:00:02" in flattened
    assert ["ethtool", "-G", "enp7s0", "rx", "4096"] in commands


def test_down_port_profile_is_prepared_without_activation(tmp_path):
    machine = tmp_path / "machine.yaml"
    write_machine(machine, {
        "spare": {
            "interface": "enp8s0",
            "interface_mac": "02:00:00:00:00:04",
            "host_cidr": "192.168.4.1/24",
        }
    })
    network = load_networks(str(machine))[0]
    commands = commands_for(network, profile_exists=False, activate=False)
    assert not any(command[:3] == ["nmcli", "connection", "up"] for command in commands)
    assert any("connection.autoconnect" in command for command in commands)


def test_host_tuning_defaults_support_full_resolution_udp_frames(tmp_path):
    machine = tmp_path / "machine.yaml"
    write_machine(machine, {
        "front": {"interface": "enp7s0", "host_cidr": "192.168.2.1/24"}
    })

    commands = host_tuning_commands(str(machine))

    assert ["sysctl", "-w", "net.core.rmem_max=33554432"] in commands
    assert ["sysctl", "-w", "net.core.netdev_max_backlog=4096"] in commands


def test_duplicate_interface_is_rejected(tmp_path):
    machine = tmp_path / "machine.yaml"
    write_machine(machine, {
        "one": {"interface": "enp7s0", "host_cidr": "192.168.2.1/24"},
        "two": {"interface": "enp7s0", "host_cidr": "192.168.3.1/24"},
    })
    with pytest.raises(NetworkSetupError, match="managed more than once"):
        load_networks(str(machine))


def test_overlapping_subnet_is_rejected(tmp_path):
    machine = tmp_path / "machine.yaml"
    write_machine(machine, {
        "one": {"interface": "enp7s0", "host_cidr": "192.168.2.1/24"},
        "two": {"interface": "enp10s0", "host_cidr": "192.168.2.2/24"},
    })
    with pytest.raises(NetworkSetupError, match="overlaps"):
        load_networks(str(machine))


@pytest.mark.parametrize("approved", ["false", 0, 1, None])
def test_approval_must_be_an_explicit_boolean(tmp_path, approved):
    machine = tmp_path / "machine.yaml"
    value = {
        "interface": "enp7s0",
        "host_cidr": "192.168.2.1/24",
    }
    if approved is not None:
        value["approved"] = approved
    machine.write_text(yaml.safe_dump({
        "schema_version": 1,
        "managed_networks": {"front": value},
    }), encoding="utf-8")

    with pytest.raises(NetworkSetupError, match="approved must be an explicit boolean"):
        load_networks(str(machine))


def test_missing_linuxptp_has_install_command():
    with pytest.raises(NetworkSetupError, match="sudo apt install linuxptp"):
        require_linuxptp(lambda _command: None)


def test_linuxptp_check_accepts_both_commands():
    require_linuxptp(lambda command: f"/usr/sbin/{command}")
