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
- Root privileges are needed only for host network setup, not normal ROS nodes.

Always inspect the commands first:

```bash
ros2 run vixel_network vixel-network-setup -- \
  --machine-file /etc/vixel/machine.yaml apply --dry-run
```

Apply them using the installed executable's absolute path so `sudo` does not
depend on a development shell's `PATH`:

```bash
sudo "$(ros2 pkg prefix vixel_network)/lib/vixel_network/vixel-network-setup" \
  --machine-file /etc/vixel/machine.yaml apply
```

The setup creates persistent, no-gateway NetworkManager profiles, configures
MTU and reverse-path filtering, increases UDP receive/backlog limits, and
adjusts NIC RX rings when supported. Disconnected ports remain configured and
activate when link carrier appears.

## Direct and switched ports

`direct` mode reserves a port for one enrolled camera. A different camera on an
occupied port is reported as a replacement and needs operator confirmation.

`switched` mode permits multiple cameras behind one interface and allocates
addresses from the configured pool up to `switched_max_devices`. Use separate,
non-overlapping subnets for every managed interface.

## Boot setup

After validating the configuration, install the network setup service:

```bash
sudo "$(ros2 pkg prefix vixel_network)/lib/vixel_network/vixel-network-setup" \
  --machine-file /etc/vixel/machine.yaml install-service
sudo systemctl enable --now vixel-network-setup.service
```

Re-run `apply` after tuning or interface changes. The example configuration is
documentation only; never apply it without replacing its synthetic values.
