import json
from pathlib import Path
import subprocess
import time

import pytest

from vixel_manager.port_status import PortStatusMonitor


def networks(count):
    return {
        f"camera_link_{index}": {
            "interface": f"enp{index}s0",
            "host_cidr": f"192.168.{index}.1/24",
        }
        for index in range(2, count + 2)
    }


def write_interface(root: Path, name: str, mac="02:00:00:00:00:02", carrier="1"):
    path = root / name
    path.mkdir()
    (path / "address").write_text(mac, encoding="ascii")
    (path / "carrier").write_text(carrier, encoding="ascii")


@pytest.mark.parametrize("count", [2, 8, 10])
def test_fast_refresh_uses_one_ip_command_for_all_ports(tmp_path, count):
    configured = networks(count)
    for network in configured.values():
        write_interface(tmp_path, network["interface"])
    commands = []

    def runner(command, _timeout):
        commands.append(command)
        if command[0] == "ip":
            return json.dumps([
                {
                    "ifname": network["interface"],
                    "addr_info": [{
                        "family": "inet",
                        "local": network["host_cidr"].split("/")[0],
                        "prefixlen": 24,
                    }],
                }
                for network in configured.values()
            ])
        network_id = command[-1].removeprefix("vixel-")
        return configured[network_id]["host_cidr"] + "\n"

    monitor = PortStatusMonitor(
        configured,
        sysfs_root=tmp_path,
        command_runner=runner,
        profile_refresh_sec=30.0,
    )
    statuses = monitor.refresh_once(now=0.0)
    first_version, _ = monitor.snapshot()
    monitor.refresh_once(now=2.0)
    second_version, _ = monitor.snapshot()

    assert len(statuses) == count
    assert all(status.present and status.link_up for status in statuses.values())
    assert all(status.profile_configured for status in statuses.values())
    assert len([command for command in commands if command[0] == "ip"]) == 2
    assert len([command for command in commands if command[0] == "nmcli"]) == count
    assert first_version == second_version


def test_probe_keeps_last_addresses_after_transient_ip_failure(tmp_path):
    configured = networks(1)
    write_interface(tmp_path, "enp2s0", carrier="0")
    ip_attempts = 0

    def runner(command, _timeout):
        nonlocal ip_attempts
        if command[0] == "nmcli":
            return "192.168.2.1/24\n"
        ip_attempts += 1
        if ip_attempts == 1:
            return json.dumps([{
                "ifname": "enp2s0",
                "addr_info": [{"family": "inet", "local": "192.168.2.1", "prefixlen": 24}],
            }])
        raise subprocess.TimeoutExpired(command, 1.0)

    monitor = PortStatusMonitor(
        configured, sysfs_root=tmp_path, command_runner=runner
    )
    monitor.refresh_once(now=0.0)
    status = monitor.refresh_once(now=2.0)["camera_link_2"]

    assert status.addresses == ("192.168.2.1/24",)
    assert status.profile_configured
    assert "IP status probe failed" in status.error


def test_probe_reports_missing_interface_and_profile_error(tmp_path):
    def runner(command, _timeout):
        if command[0] == "ip":
            return "[]"
        raise subprocess.CalledProcessError(10, command)

    monitor = PortStatusMonitor(
        networks(1), sysfs_root=tmp_path, command_runner=runner
    )
    status = monitor.refresh_once(now=0.0)["camera_link_2"]

    assert not status.present
    assert not status.profile_configured
    assert "NetworkManager profile probe failed" in status.error


def test_monitor_worker_stops_promptly(tmp_path):
    monitor = PortStatusMonitor(
        {}, sysfs_root=tmp_path, command_runner=lambda _command, _timeout: "[]",
        fast_refresh_sec=10.0,
    )
    monitor.start()
    time.sleep(0.02)
    monitor.stop(timeout=0.5)
    assert monitor._thread is None
