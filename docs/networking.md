# Networking and camera provisioning

Vixel manages only interfaces listed under `managed_networks` in
`/etc/vixel/machine.yaml`. Each entry binds a Linux interface name to its
hardware MAC address, preventing an interface rename from applying sensor
network settings to the wrong NIC.

## Safety model

- The default route interface, loopback, and MAC mismatches are rejected.
- A network must be explicitly marked `approved` before enrollment or ForceIP.
- Every managed interface has its own subnet and address pool.
- Vixel uses volatile GVCP ForceIP assignments and does not overwrite camera
  persistent IP, DHCP, or link-local settings.
- Root privileges are needed for host network and PTP clock services, not
  normal ROS nodes.

`approved` is required for every configured entry and must be the YAML boolean
`true` or `false`; quoted strings such as `"false"` are rejected. Start a new
or changed link with `approved: false`. Cameras on that configured link remain
visible for inventory and diagnostics, but Vixel does not treat them as
managed, assign addresses, enroll them, or open capture sessions. Set
`auto_enroll: true` only as a separate, deliberate choice after approving the
link.

Always inspect the commands first:

```bash
ros2 run vixel_network vixel-network-setup -- \
  --machine-file /etc/vixel/machine.yaml apply --dry-run
```

For a new or changed link, use this order:

1. Keep `approved: false`, set the expected interface MAC and subnet, and run
   both `vixel -- config validate` and the network setup dry run.
2. Confirm the interface, MAC, subnet, address pool, mode, and printed host
   commands against the physical port.
3. Change `approved` to `true`; enable `auto_enroll` only if unattended
   enrollment is intended.
4. Validate again, apply the host network configuration, and restart the Vixel
   stack. Static configuration is not hot-reloaded consistently across nodes.

Apply them using the installed executable's absolute path so `sudo` does not
depend on a development shell's `PATH`:

```bash
sudo env "PYTHONPATH=$PYTHONPATH" \
  "$(ros2 pkg prefix vixel_network)/lib/vixel_network/vixel-network-setup" \
  --machine-file /etc/vixel/machine.yaml apply
```

The setup creates persistent, no-gateway NetworkManager profiles, configures
MTU and reverse-path filtering, increases UDP receive/backlog limits, and
adjusts NIC RX rings when supported. Disconnected ports remain configured and
activate when link carrier appears.

Camera `packet_delay` is equally important when several NICs share one PCIe
uplink. The 100,000 ns default paces image transfer while leaving PTP exposure
timing unchanged. Validate a machine under simultaneous capture with
`ethtool -S <interface>`; increasing `rx_missed_errors` or `rx_fifo_errors`
means aggregate camera traffic still exceeds the host path.

Every managed camera NIC with a hardware clock participates in the PC-mastered
PTP domain. `vixel-ptp-supervisor` discovers interfaces dynamically, runs one
`ptp4l` grandmaster port per link using E2E delay measurement, and disciplines
secondary NIC clocks to a deterministic primary with `phc2sys`.
Follower offsets above 1 ms are stepped immediately: some NIC hardware clocks
jump by several seconds when a direct camera link is unplugged, and slewing such
an offset would otherwise keep that camera out of PTP lock for minutes.
Its generated configuration is installed at `/etc/linuxptp/vixel-ptp4l.conf`
so it works with the default Ubuntu `ptp4l` AppArmor profile.

## Direct and switched ports

`direct` mode reserves a port for one enrolled camera. A different camera on an
occupied port is reported as a replacement and needs operator confirmation.

`switched` mode permits multiple cameras behind one interface and allocates
addresses from the configured pool up to `switched_max_devices`. Use separate,
non-overlapping subnets for every managed interface.

## Boot setup

After validating the configuration, provision both boot services once:

```bash
sudo env "PYTHONPATH=$PYTHONPATH" \
  "$(ros2 pkg prefix vixel_network)/lib/vixel_network/vixel-network-setup" \
  --machine-file /etc/vixel/machine.yaml install-service
```

The command creates, enables, and immediately starts
`vixel-network-setup.service` and `vixel-ptp.service`. Systemd starts them on
every later boot, so no manual PTP start is required.

Re-run validation, `apply`, and the Vixel stack restart after tuning or
interface changes. The example configuration is documentation only; never
apply it without replacing its synthetic values.
