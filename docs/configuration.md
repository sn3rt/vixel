# Configuration and GenICam features

Static host and provider settings live in `/etc/vixel/machine.yaml`. Dynamic
sensor identity, enrollment, location, groups, notes, and camera settings live
in `/var/lib/vixel/inventory.yaml` and should normally be changed through the
dashboard or ROS interfaces.

Useful commands:

```bash
ros2 run vixel_manager vixel -- config paths
ros2 run vixel_manager vixel -- config validate
ros2 run vixel_manager vixel -- config show machine
ros2 run vixel_manager vixel -- config show inventory
ros2 run vixel_manager vixel -- inventory list
ros2 run vixel_manager vixel -- inventory list --all
ros2 run vixel_manager vixel -- inventory show acme_cam_test0001
```

`VIXEL_MACHINE_FILE` and `VIXEL_INVENTORY_FILE` override the default paths for
development or testing.

## Capture recording

Triggered capture sets are configured in the static machine file:

```yaml
recording:
  root_directory: /var/lib/vixel/captures
  minimum_free_bytes: 5368709120
  capture_timeout_ms: 10000
  png_compression: 3
  recent_limit: 100
```

The recorder rejects a new capture when available space is below
`minimum_free_bytes`; it never deletes old captures automatically.
`capture_timeout_ms` applies after the group trigger is accepted.
`png_compression` is OpenCV's lossless PNG compression level from 0 through 9.
`recent_limit` limits the history loaded into ROS and the dashboard, not the
number of capture directories retained on disk.

## Portable camera settings

The **Known sensor details → Camera settings** field accepts JSON. Common
portable settings include:

```json
{
  "pixel_format": "BGR8",
  "width": 2048,
  "height": 1536,
  "exposure_auto": "Off",
  "exposure_us": 1500.0,
  "gain_auto": "Off",
  "gain_db": 2.0,
  "frame_rate_hz": 4.0,
  "packet_size": 9000,
  "packet_delay_ns": 100000,
  "trigger_source": "FreeRun",
  "transfer_control_mode": "Basic"
}
```

`FreeRun` is the interoperable default. Use `Software` only when the camera
supports a software `FrameStart` trigger. When SFNC `TransferControlMode` is
available, Vixel defaults it to `Basic` to avoid frames being held by settings
left behind by another application.

`packet_delay_ns` paces GigE Vision image packets without changing exposure
time or PTP synchronization. The conservative 100,000 ns default prevents
simultaneous full-resolution streams from overrunning a shared PCIe uplink.
Machines with independently provisioned NIC bandwidth may lower it after
checking the NIC drop counters under simultaneous capture.

Grouped cameras automatically request `Action0` and PTP; there is no group
trigger-source selector. Cameras without the required nodes fall back to the
software barrier and remain part of the group, with their result marked
unsynchronized. Individual settings remain effective for ungrouped cameras.

`providers.genicam.software_trigger_lead_time_ms` controls the short interval
between arming all group workers and releasing their software triggers. It
defaults to 10 ms and accepts values from 1 through 100 ms; it is preparation
time, not a requested exposure offset.

PTP scheduled capture defaults can be tuned under `providers.genicam.ptp`:

```yaml
ptp:
    action_lead_time_ms: 500
  tolerance_ns: 100000
  action_device_key: 1
```

The lead time is how far into the common PTP clock a group action is queued.
The tolerance is used for camera lock status and measured exposure skew.

Arbitrary GenICam nodes can be set without vendor code:

```json
{
  "features": {
    "BalanceWhiteAuto": {"type": "enum", "value": "Continuous"}
  }
}
```

Supported types are `boolean`, `integer`, `float`, `string`, `enum`, and
`command`. Unsupported or invalid writes fail the camera session with the
underlying GenICam error instead of being ignored.

Inspect standard live features with:

```bash
ros2 service call /vixel/providers/genicam/features \
  vixel_interfaces/srv/GetCameraFeatures \
  "{sensor_id: acme_cam_test0001, names: []}"
```

## Calibration

Calibration is not a camera exposure or gain profile. It describes lens
intrinsics and distortion for ROS consumers. Generate a calibration using the
ROS camera calibration tools, store it outside the repository, and enter its
`file://` or `package://` URL in the sensor metadata. A zero-valued template is
installed as `config/camera_calibration.example.yaml`.
