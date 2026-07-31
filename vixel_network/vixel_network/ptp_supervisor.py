from __future__ import annotations

import argparse
import os
import pathlib
import signal
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable

from .network_setup import ManagedNetwork, NetworkSetupError, load_networks


@dataclass(frozen=True)
class PtpPort:
    network_id: str
    interface: str
    phc: str


def discover_ptp_ports(
    networks: list[ManagedNetwork],
    sysfs_root: pathlib.Path = pathlib.Path("/sys/class/net"),
) -> list[PtpPort]:
    ports: list[PtpPort] = []
    for network in sorted(networks, key=lambda item: item.network_id):
        ptp_dir = sysfs_root / network.interface / "device" / "ptp"
        try:
            clocks = sorted(path.name for path in ptp_dir.iterdir() if path.name.startswith("ptp"))
        except OSError as error:
            raise NetworkSetupError(
                f"camera interface {network.interface} has no hardware PTP clock"
            ) from error
        if len(clocks) != 1:
            raise NetworkSetupError(
                f"camera interface {network.interface} exposes {len(clocks)} PTP clocks"
            )
        ports.append(PtpPort(network.network_id, network.interface, f"/dev/{clocks[0]}"))
    return ports


def ptp4l_command(port: PtpPort, config_path: str) -> list[str]:
    return ["ptp4l", "-f", config_path, "-i", port.interface, "-m"]


def phc2sys_command(primary: PtpPort, follower: PtpPort) -> list[str]:
    return [
        "phc2sys", "-s", primary.phc, "-c", follower.phc,
        "-O", "0", "-R", "16", "-m",
    ]


def render_ptp4l_config() -> str:
    return "\n".join([
        "[global]",
        "time_stamping hardware",
        "network_transport UDPv4",
        "delay_mechanism E2E",
        "serverOnly 1",
        "gmCapable 1",
        "priority1 10",
        "priority2 10",
        "logging_level 6",
        "",
    ])


class PtpSupervisor:
    def __init__(
        self,
        ports: list[PtpPort],
        runtime_directory: pathlib.Path = pathlib.Path("/run/vixel"),
        popen: Callable = subprocess.Popen,
    ):
        if not ports:
            raise NetworkSetupError("no managed PTP camera interfaces are configured")
        self.ports = ports
        self.runtime_directory = runtime_directory
        self.popen = popen
        self.children: list[subprocess.Popen] = []
        self.stopping = False

    def start(self) -> None:
        self.runtime_directory.mkdir(parents=True, exist_ok=True)
        config_path = self.runtime_directory / "ptp4l.conf"
        config_path.write_text(render_ptp4l_config(), encoding="ascii")
        primary = self.ports[0]
        for port in self.ports:
            self.children.append(self.popen(ptp4l_command(port, str(config_path))))
        for port in self.ports[1:]:
            self.children.append(self.popen(phc2sys_command(primary, port)))
        print(
            f"vixel-ptp: primary={primary.interface} ({primary.phc}), "
            f"ports={','.join(port.interface for port in self.ports)}",
            flush=True,
        )

    def stop(self) -> None:
        self.stopping = True
        for child in self.children:
            if child.poll() is None:
                child.terminate()
        deadline = time.monotonic() + 5.0
        for child in self.children:
            if child.poll() is None:
                try:
                    child.wait(timeout=max(0.0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    child.kill()

    def run(self) -> int:
        self.start()
        while not self.stopping:
            for child in self.children:
                result = child.poll()
                if result is not None:
                    print(
                        f"vixel-ptp: child process exited with status {result}",
                        file=sys.stderr,
                    )
                    self.stop()
                    return 1
            time.sleep(0.5)
        return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="vixel-ptp-supervisor")
    parser.add_argument("--machine-file", default="/etc/vixel/machine.yaml")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    try:
        missing = [command for command in ("ptp4l", "phc2sys") if not shutil.which(command)]
        if missing:
            raise NetworkSetupError(
                "linuxptp is not installed; run: sudo apt install linuxptp "
                f"(missing: {', '.join(missing)})"
            )
        if os.geteuid() != 0 and not args.check:
            raise NetworkSetupError("PTP supervisor must run as root")
        ports = discover_ptp_ports(load_networks(args.machine_file))
        if args.check:
            for port in ports:
                print(f"{port.network_id}: {port.interface} -> {port.phc}")
            return 0
        supervisor = PtpSupervisor(ports)
        signal.signal(signal.SIGTERM, lambda *_: supervisor.stop())
        signal.signal(signal.SIGINT, lambda *_: supervisor.stop())
        return supervisor.run()
    except (NetworkSetupError, OSError, subprocess.SubprocessError) as error:
        print(f"vixel-ptp: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
