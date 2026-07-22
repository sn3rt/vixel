from __future__ import annotations

import argparse
import ipaddress
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Callable

import yaml


class NetworkSetupError(RuntimeError):
    pass


@dataclass(frozen=True)
class ManagedNetwork:
    network_id: str
    interface: str
    interface_mac: str
    host_cidr: str
    mtu: int
    rp_filter: int
    rx_ring_size: int

    @property
    def profile(self) -> str:
        return f"vixel-{self.network_id}"


def _load_machine(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as stream:
            root = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as error:
        raise NetworkSetupError(f"cannot read {path}: {error}") from error
    if root.get("schema_version", 1) != 1:
        raise NetworkSetupError("unsupported machine schema_version")
    return root


def load_networks(path: str) -> list[ManagedNetwork]:
    root = _load_machine(path)
    raw_networks = root.get("managed_networks") or {}
    if not isinstance(raw_networks, dict) or not raw_networks:
        raise NetworkSetupError("machine YAML defines no managed_networks")
    networks = []
    seen_interfaces: set[str] = set()
    seen_subnets: list[ipaddress.IPv4Network] = []
    for network_id, value in raw_networks.items():
        try:
            interface = str(value["interface"])
            interface_mac = str(value.get("interface_mac", "")).lower()
            host = ipaddress.ip_interface(str(value["host_cidr"]))
            mtu = int(value.get("mtu", 9000))
            rp_filter = int(value.get("rp_filter", 0))
            rx_ring_size = int(value.get("rx_ring_size", 4096))
        except (KeyError, TypeError, ValueError) as error:
            raise NetworkSetupError(f"invalid managed network {network_id}") from error
        if not isinstance(host, ipaddress.IPv4Interface):
            raise NetworkSetupError(f"managed network {network_id} must use IPv4")
        if interface in seen_interfaces:
            raise NetworkSetupError(f"interface {interface} is managed more than once")
        if any(host.network.overlaps(previous) for previous in seen_subnets):
            raise NetworkSetupError(f"subnet {host.network} overlaps another managed network")
        if mtu < 1500 or rp_filter not in (0, 1, 2) or rx_ring_size < 256:
            raise NetworkSetupError(
                f"managed network {network_id} has invalid MTU, rp_filter, or RX ring size"
            )
        seen_interfaces.add(interface)
        seen_subnets.append(host.network)
        networks.append(ManagedNetwork(
            network_id, interface, interface_mac, str(host), mtu, rp_filter, rx_ring_size
        ))
    return networks


def host_tuning_commands(path: str) -> list[list[str]]:
    tuning = _load_machine(path).get("host_tuning") or {}
    try:
        receive_buffer = int(tuning.get("udp_receive_buffer_bytes", 33554432))
        netdev_backlog = int(tuning.get("netdev_max_backlog", 4096))
    except (TypeError, ValueError) as error:
        raise NetworkSetupError("invalid host_tuning values") from error
    if receive_buffer < 1048576:
        raise NetworkSetupError("udp_receive_buffer_bytes must be at least 1048576")
    if netdev_backlog < 1000:
        raise NetworkSetupError("netdev_max_backlog must be at least 1000")
    return [
        ["sysctl", "-w", f"net.core.rmem_max={receive_buffer}"],
        ["sysctl", "-w", f"net.core.netdev_max_backlog={netdev_backlog}"],
    ]


def default_route_interfaces(run: Callable = subprocess.run) -> set[str]:
    result = run(
        ["ip", "-o", "route", "show", "default"],
        check=True,
        text=True,
        capture_output=True,
    )
    interfaces = set()
    words = result.stdout.split()
    for index, word in enumerate(words[:-1]):
        if word == "dev":
            interfaces.add(words[index + 1])
    return interfaces


def commands_for(network: ManagedNetwork, profile_exists: bool,
                 activate: bool = True) -> list[list[str]]:
    commands: list[list[str]] = []
    if not profile_exists:
        commands.append([
            "nmcli", "connection", "add", "type", "ethernet",
            "ifname", network.interface, "con-name", network.profile,
        ])
    commands.append([
        "nmcli", "connection", "modify", network.profile,
        "connection.interface-name", network.interface,
        "connection.autoconnect", "yes",
        "connection.autoconnect-priority", "100",
        "ipv4.method", "manual",
        "ipv4.addresses", network.host_cidr,
        "ipv4.gateway", "",
        "ipv4.never-default", "yes",
        "ipv4.ignore-auto-dns", "yes",
        "ipv6.method", "disabled",
        "802-3-ethernet.mtu", str(network.mtu),
    ])
    if network.interface_mac:
        commands[-1].extend(["802-3-ethernet.mac-address", network.interface_mac])
    if activate:
        commands.append(["nmcli", "connection", "up", network.profile])
    commands.append(["ethtool", "-G", network.interface, "rx", str(network.rx_ring_size)])
    commands.append([
        "sysctl", "-w", f"net.ipv4.conf.{network.interface}.rp_filter={network.rp_filter}"
    ])
    return commands


def profile_names(run: Callable = subprocess.run) -> set[str]:
    result = run(
        ["nmcli", "-t", "-f", "NAME", "connection", "show"],
        check=True,
        text=True,
        capture_output=True,
    )
    return {line for line in result.stdout.splitlines() if line}


def apply_networks(machine_file: str, dry_run: bool = False,
                   run: Callable = subprocess.run) -> None:
    networks = load_networks(machine_file)
    defaults = default_route_interfaces(run)
    existing = profile_names(run) if not dry_run else set()
    for command in host_tuning_commands(machine_file):
        if dry_run:
            print(shlex.join(command))
        else:
            run(command, check=True)
    for network in networks:
        sysfs = pathlib.Path("/sys/class/net") / network.interface
        if not sysfs.exists():
            raise NetworkSetupError(f"managed interface {network.interface} does not exist")
        if network.interface == "lo" or network.interface in defaults:
            raise NetworkSetupError(
                f"refusing to configure default-route interface {network.interface}"
            )
        actual_mac = (sysfs / "address").read_text(encoding="ascii").strip().lower()
        if network.interface_mac and actual_mac != network.interface_mac:
            raise NetworkSetupError(
                f"managed interface {network.interface} has MAC {actual_mac}, "
                f"expected {network.interface_mac}"
            )
        carrier_path = sysfs / "carrier"
        try:
            carrier = carrier_path.read_text(encoding="ascii").strip() == "1"
        except OSError:
            carrier = False
        for command in commands_for(
            network, network.profile in existing, activate=carrier
        ):
            if dry_run:
                print(shlex.join(command))
            else:
                run(command, check=True)


def install_service(machine_file: str, executable: str) -> None:
    if os.geteuid() != 0:
        raise NetworkSetupError("install-service must run as root")
    executable_path = pathlib.Path(executable).resolve()
    prefix = executable_path.parents[2]
    setup_candidates = [
        prefix.parent / "setup.bash",
        prefix / "share/vixel_network/local_setup.bash",
    ]
    local_setup = next((path for path in setup_candidates if path.exists()), None)
    if local_setup is None:
        raise NetworkSetupError("cannot locate the Vixel workspace/package setup script")
    command = (
        f"source /opt/ros/lyrical/setup.bash && "
        f"source {shlex.quote(str(local_setup))} && "
        f"exec {shlex.quote(str(executable_path))} "
        f"--machine-file {shlex.quote(str(pathlib.Path(machine_file).resolve()))} apply"
    )
    unit = "\n".join([
        "[Unit]",
        "Description=Configure Vixel managed sensor networks",
        "After=NetworkManager.service",
        "Wants=NetworkManager.service",
        "",
        "[Service]",
        "Type=oneshot",
        f"ExecStart=/bin/bash -lc {shlex.quote(command)}",
        "RemainAfterExit=yes",
        "",
        "[Install]",
        "WantedBy=multi-user.target",
        "",
    ])
    unit_path = pathlib.Path("/etc/systemd/system/vixel-network-setup.service")
    unit_path.write_text(unit, encoding="utf-8")
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "enable", "vixel-network-setup.service"], check=True)
    print(f"Installed {unit_path}; run 'sudo systemctl start vixel-network-setup'")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="vixel-network-setup")
    parser.add_argument("--machine-file", default="/etc/vixel/machine.yaml")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("validate")
    apply = commands.add_parser("apply")
    apply.add_argument("--dry-run", action="store_true")
    commands.add_parser("install-service")
    args = parser.parse_args(argv)
    try:
        if args.command == "validate":
            networks = load_networks(args.machine_file)
            print(f"Valid configuration for {len(networks)} managed network(s)")
        elif args.command == "apply":
            if not args.dry_run and os.geteuid() != 0:
                raise NetworkSetupError("apply must run as root (or use --dry-run)")
            apply_networks(args.machine_file, dry_run=args.dry_run)
        else:
            executable = shutil.which("vixel-network-setup") or sys.argv[0]
            install_service(args.machine_file, executable)
        return 0
    except (NetworkSetupError, subprocess.CalledProcessError) as error:
        print(f"vixel-network-setup: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
