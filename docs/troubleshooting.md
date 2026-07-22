# Troubleshooting

## Dashboard does not load

Check that only one gateway owns the port and that the stack is healthy:

```bash
ss -ltnp | grep ':8080'
curl --fail http://127.0.0.1:8080/api/v1/health
```

Keep the SSH tunnel session open and browse to the client's
`http://127.0.0.1:8080`, not the camera computer's private address.

## Camera is not discovered

For GigE Vision cameras, verify carrier, the configured host address, and the
camera's current visibility:

```bash
ip -br link
ip -br -4 addr
ros2 run vixel_manager vixel -- config validate
```

Confirm that the interface name and MAC match `/etc/vixel/machine.yaml`. An
unapproved interface may report a camera but will not provision or enroll it.

For USB cameras, distinguish USB3 Vision models from proprietary USB models.
Being connected over USB 3 does not by itself make a camera USB3 Vision or
Aravis-compatible.

## Incomplete GigE frames

Incomplete buffers usually mean dropped UDP packets. Check all of the following:

- jumbo MTU is supported end-to-end, or reduce both host and camera packet size;
- receive buffers and network backlog were applied by `vixel-network-setup`;
- NIC RX ring tuning succeeded;
- packet delay and frame rate leave enough bandwidth;
- each direct camera uses the intended dedicated interface;
- cables, PoE power, and switches are reliable.

The raw payload of a 2048×1536 BGR8 image is about 9 MiB. Multiple cameras can
overrun buffers even when their average preview traffic appears low.

## Camera is busy or access is denied

Stop vendor tools, old ROS launches, and other applications controlling the
camera. Only one process should hold exclusive control. If a previous process
was killed, power-cycle the camera after confirming no owner remains.

## Slow or stale preview

The dashboard polls cached compressed snapshots and does not control the raw
camera rate. Check provider acquisition warnings, preview rate, JPEG encoding
load, network loss, and browser tunnel latency. A successful HTTP response can
still contain an old cached frame when acquisition has stalled.
