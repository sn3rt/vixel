from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Callable


CommandRunner = Callable[[list[str], float], str]


def run_command(command: list[str], timeout: float) -> str:
    result = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return result.stdout


@dataclass(frozen=True)
class PortStatus:
    present: bool = False
    actual_mac: str = ""
    link_up: bool = False
    addresses: tuple[str, ...] = ()
    profile_configured: bool = False
    error: str = ""
    updated_at: float = 0.0


class PortStatusMonitor:
    """Collect host port state without blocking ROS executor threads."""

    def __init__(
        self,
        networks: dict,
        *,
        fast_refresh_sec: float = 2.0,
        profile_refresh_sec: float = 30.0,
        command_timeout_sec: float = 1.0,
        sysfs_root: Path = Path("/sys/class/net"),
        command_runner: CommandRunner = run_command,
    ):
        self.networks = {
            str(network_id): {
                "interface": str(network["interface"]),
                "host_cidr": str(network["host_cidr"]),
            }
            for network_id, network in networks.items()
        }
        self.fast_refresh_sec = max(float(fast_refresh_sec), 0.1)
        self.profile_refresh_sec = max(float(profile_refresh_sec), self.fast_refresh_sec)
        self.command_timeout_sec = max(float(command_timeout_sec), 0.1)
        self.sysfs_root = Path(sysfs_root)
        self.command_runner = command_runner
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._version = 0
        self._statuses: dict[str, PortStatus] = {}
        self._addresses: dict[str, tuple[str, ...]] = {}
        self._profile_configured: dict[str, bool] = {}
        self._profile_errors: dict[str, str] = {}
        self._next_profile_refresh = 0.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="vixel-port-status",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
            self._thread = None

    def snapshot(self) -> tuple[int, dict[str, PortStatus]]:
        with self._lock:
            return self._version, dict(self._statuses)

    def refresh_once(self, now: float | None = None) -> dict[str, PortStatus]:
        now = time.monotonic() if now is None else float(now)
        ip_error = ""
        try:
            output = self.command_runner(
                ["ip", "-j", "-4", "addr", "show"], self.command_timeout_sec
            )
            self._addresses = self._parse_ip_addresses(output)
        except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError) as error:
            ip_error = f"IP status probe failed: {error}"

        if now >= self._next_profile_refresh:
            self._refresh_profiles()
            self._next_profile_refresh = now + self.profile_refresh_sec

        statuses = {}
        for network_id, network in self.networks.items():
            interface = network["interface"]
            path = self.sysfs_root / interface
            present = path.exists()
            actual_mac = ""
            link_up = False
            errors = [value for value in (ip_error, self._profile_errors.get(network_id, "")) if value]
            if present:
                try:
                    actual_mac = (path / "address").read_text(encoding="ascii").strip().lower()
                except OSError as error:
                    errors.append(f"Cannot read {interface} MAC: {error}")
                try:
                    link_up = (path / "carrier").read_text(encoding="ascii").strip() == "1"
                except OSError as error:
                    errors.append(f"Cannot read {interface} carrier: {error}")
            statuses[network_id] = PortStatus(
                present=present,
                actual_mac=actual_mac,
                link_up=link_up,
                addresses=self._addresses.get(interface, ()),
                profile_configured=self._profile_configured.get(network_id, False),
                error="; ".join(errors),
                updated_at=now,
            )

        with self._lock:
            changed = self._observable(statuses) != self._observable(self._statuses)
            self._statuses = statuses
            if changed:
                self._version += 1
            return dict(self._statuses)

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self.refresh_once(started)
            except Exception as error:  # Keep monitoring after unexpected host/probe failures.
                with self._lock:
                    failed = {
                        network_id: PortStatus(
                            error=f"Port status monitor failed: {error}", updated_at=started
                        )
                        for network_id in self.networks
                    }
                    changed = self._observable(failed) != self._observable(self._statuses)
                    self._statuses = failed
                    if changed:
                        self._version += 1
            elapsed = time.monotonic() - started
            self._stop.wait(max(0.0, self.fast_refresh_sec - elapsed))

    def _refresh_profiles(self) -> None:
        for network_id, network in self.networks.items():
            try:
                output = self.command_runner(
                    [
                        "nmcli", "-g", "ipv4.addresses", "connection", "show",
                        f"vixel-{network_id}",
                    ],
                    self.command_timeout_sec,
                )
                configured_addresses = {
                    value.strip()
                    for line in output.splitlines()
                    for value in line.split(",")
                    if value.strip()
                }
                self._profile_configured[network_id] = (
                    network["host_cidr"] in configured_addresses
                )
                self._profile_errors.pop(network_id, None)
            except subprocess.CalledProcessError as error:
                self._profile_configured[network_id] = False
                self._profile_errors[network_id] = (
                    f"NetworkManager profile probe failed: {error}"
                )
            except (OSError, subprocess.SubprocessError) as error:
                self._profile_errors[network_id] = (
                    f"NetworkManager profile probe failed: {error}"
                )

    @staticmethod
    def _parse_ip_addresses(output: str) -> dict[str, tuple[str, ...]]:
        result: dict[str, tuple[str, ...]] = {}
        for interface in json.loads(output):
            name = str(interface["ifname"])
            addresses = sorted({
                f"{entry['local']}/{entry['prefixlen']}"
                for entry in interface.get("addr_info", [])
                if entry.get("family") == "inet" or "." in str(entry.get("local", ""))
            })
            result[name] = tuple(addresses)
        return result

    @staticmethod
    def _observable(statuses: dict[str, PortStatus]) -> dict[str, tuple]:
        return {
            network_id: (
                status.present,
                status.actual_mac,
                status.link_up,
                status.addresses,
                status.profile_configured,
                status.error,
            )
            for network_id, status in statuses.items()
        }
