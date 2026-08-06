import pathlib

import pytest

from vixel_network.network_setup import ManagedNetwork, NetworkSetupError
from vixel_network.ptp_supervisor import (
    PtpPort,
    discover_ptp_ports,
    phc2sys_command,
    ptp4l_command,
    render_ptp4l_config,
)


def network(network_id, interface):
    return ManagedNetwork(network_id, interface, "", "192.168.1.1/24", 9000, 0, 4096)


def test_discovers_and_sorts_dynamic_hardware_clocks(tmp_path):
    for interface, clock in (("enp10s0", "ptp5"), ("enp7s0", "ptp1")):
        directory = tmp_path / interface / "device" / "ptp"
        directory.mkdir(parents=True)
        (directory / clock).touch()
    ports = discover_ptp_ports(
        [network("rear", "enp10s0"), network("front", "enp7s0")], tmp_path
    )
    assert ports == [
        PtpPort("front", "enp7s0", "/dev/ptp1"),
        PtpPort("rear", "enp10s0", "/dev/ptp5"),
    ]


def test_missing_hardware_clock_is_actionable(tmp_path):
    with pytest.raises(NetworkSetupError, match="no hardware PTP clock"):
        discover_ptp_ports([network("front", "enp7s0")], tmp_path)


def test_linuxptp_commands_use_e2e_and_primary_phc():
    primary = PtpPort("a", "enp7s0", "/dev/ptp1")
    follower = PtpPort("b", "enp8s0", "/dev/ptp2")
    assert ptp4l_command(primary, "/etc/linuxptp/vixel-ptp4l.conf")[-2:] == [
        "enp7s0", "-m"
    ]
    assert phc2sys_command(primary, follower) == [
        "phc2sys", "-s", "/dev/ptp1", "-c", "/dev/ptp2",
        "-O", "0", "-R", "16", "-S", "0.001", "-m",
    ]
    config = render_ptp4l_config()
    assert "delay_mechanism E2E" in config
    assert "serverOnly 1" in config


def test_supervisor_requires_a_readable_installed_config(tmp_path):
    from vixel_network.ptp_supervisor import PtpSupervisor

    supervisor = PtpSupervisor(
        [PtpPort("front", "enp7s0", "/dev/ptp1")],
        config_path=tmp_path / "missing.conf",
    )
    with pytest.raises(NetworkSetupError, match="re-run vixel-network-setup"):
        supervisor.start()
