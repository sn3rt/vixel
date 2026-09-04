# Configuration and GenICam features

Static host and provider settings live in `/etc/vixel/machine.yaml`. Dynamic
sensor identity, enrollment, location, capture selections, notes, and camera settings live
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
  sequence_prepare_timeout_ms: 60000
  sequence_dispatch_lead_ms: 150
  recent_limit: 100
  max_inflight_captures: 32
  operation_history_limit: 100
  operation_capture_id_limit: 100
  max_active_operations: 64
  capture_session_ttl_ms: 15000
  gps:
    enabled: false
    topic: /fix
    max_age_ms: 2000
```

The recorder rejects a new capture when available space is below
`minimum_free_bytes`; it never deletes old captures automatically.
`capture_timeout_ms` applies after the group trigger is accepted.
`recent_limit` limits the history loaded into ROS and the dashboard, not the
number of capture directories retained on disk.
`max_inflight_captures` bounds frames being received or saved. A managed
sequence stops scheduling new exposures at this limit and drains everything
already accepted. GPS is optional and never delays a capture; a recent valid
ROS 2 `sensor_msgs/NavSatFix` is copied into the manifest when enabled. ROS 1
GPS publishers can be exposed through `ros1_bridge`.

`operation_history_limit` retains the newest completed, failed, or cancelled
operations for status lookup; active operations are never pruned.
`operation_capture_id_limit` retains only the newest capture IDs in each
operation message while `capture_id_count` continues to report the total and
`capture_ids_truncated` identifies a shortened list. Capture directories and
manifests remain on disk. `max_active_operations` rejects new batches or
sequences once that many operations are active, without disturbing work that
is already running.

`capture_session_ttl_ms` is the lease lifetime for ROS and HTTP capture owners.
Active controllers renew before it expires; abandoned leases are discarded and
the cameras return to the launch baseline. The supported range is 5000–300000
ms.

`sequence_prepare_timeout_ms` covers capture-mode configuration, camera
restart, PTP relock, the first clean automatic-metering frame, and the final
idle/`TriggerArmed` check. Sequence preparation fails without firing a trigger
if the requested cadence cannot be configured before this timeout.

`sequence_dispatch_lead_ms` controls how early a managed sequence sends its
scheduled PTP actions. Vixel bounds that lead by the smallest reported camera
action queue, assuming one entry when `ActionQueueSize` is unavailable.

All recording values are static configuration. After editing `machine.yaml`,
run `ros2 run vixel_manager vixel -- config validate` and restart the Vixel
stack so the manager, recorder, and web gateway use the same limits. The web
gateway derives its synchronous capture deadline from `capture_timeout_ms` and
requests action cancellation before returning HTTP 504 when that deadline is
exceeded.

## Portable camera settings

Reusable administrator-managed profiles are loaded from
`/etc/vixel/camera-profiles/*.yaml` by default. A camera selects a profile in
**Known sensor details** and may add per-camera JSON overrides. Resolution is
machine defaults, then profile settings, then per-camera overrides; the GenICam
runtime finally enforces `Action0` and PTP.

Portable constrained-auto and fixed manual examples are installed under
`share/vixel/config/camera-profiles/`. Copy the profiles you approve into the
configured admin directory, then call `/vixel/reload_camera_profiles` or restart
the stack. The dashboard can select profiles and set overrides but cannot edit
the administrator-owned profile files.

The **Known sensor details → Per-camera overrides** field accepts JSON. Common
portable settings include:

```json
{
  "pixel_format": "BGR8",
  "width": 2048,
  "height": 1536,
  "exposure_auto": "Off",
  "exposure_us": 1500.0,
  "exposure_auto_limit_auto": false,
  "exposure_auto_upper_us": 10000.0,
  "gain_auto": "Off",
  "gain_db": 2.0,
  "gain_auto_upper_db": 12.0,
  "auto_brightness_target": 50.0,
  "metering_rate_hz": 2.0,
  "capture_png_compression": 1,
  "frame_rate_hz": 4.0,
  "packet_size": 9000,
  "packet_delay_ns": 100000,
  "trigger_overlap": "PreviousFrame",
  "trigger_source": "FreeRun",
  "transfer_control_mode": "Basic"
}
```

`FreeRun` is the interoperable default. Software-trigger selection is
capability-driven: Vixel prefers `FrameStart`, then `ExposureStart`, then
`AcquisitionStart`, and verifies that the selected trigger advertises the
`Software` source. This supports cameras such as IDS models whose software
trigger is attached to `ExposureStart`. When SFNC `TransferControlMode` is
available, Vixel defaults it to `Basic` to avoid frames being held by settings
left behind by another application.

`packet_delay_ns` paces GigE Vision image packets without changing exposure
time or PTP synchronization. The conservative 100,000 ns default prevents
simultaneous full-resolution streams from overrunning a shared PCIe uplink.
Machines with independently provisioned NIC bandwidth may lower it after
checking the NIC drop counters under simultaneous capture.

Managed sequences derive camera limits from their requested interval; no
camera-model or rate-specific profile is needed. For example, a 200 ms
sequence with the default 10 ms safety margin caps exposure at no more than
190 ms, with a smaller ceiling when exposure and sensor readout cannot overlap.
Manual `ExposureTime` is clamped to that ceiling. Automatic exposure uses a
readable upper-limit node; because that node is not standardized by SFNC,
Vixel probes known aliases and accepts `exposure_auto_upper_feature` and
`exposure_auto_limit_auto_feature` overrides. If automatic exposure cannot be
bounded, preparation is rejected before cycle one instead of risking skipped
or incorrectly exposed frames. Limit-auto controls are handled as either
GenICam Boolean nodes or enumerations such as `Off`/`Disabled`/`Manual`.

`trigger_overlap` is capability-driven and optional. If no explicit override
is present and a sequence requests a cadence, Vixel prefers `PreviousFrame`
and then `ReadOut` from the camera's advertised `TriggerOverlap` values.
Unsupported cameras keep their current behavior and advertise a safe cadence
that includes exposure plus readout. The strictest group member determines
whether the sequence can start.

`providers.genicam.cadence_safety_margin_ms` sets the reserved time subtracted
from every sequence period; it defaults to 10 ms. This protects the trigger
boundary from camera/transport jitter and is applied uniformly to every
GenICam camera.

Sequence negotiation does not silently lower `packet_delay_ns`: that value is
a link-capacity policy and reducing it can overload cameras that share a NIC or
PCIe uplink. If the camera reports that readout still cannot fit the requested
period, preparation names that physical cadence limit. Installations with
dedicated camera links can lower the managed-network packet delay in
`machine.yaml` independently of camera brand.

Grouped cameras automatically request `Action0` and PTP; there is no group
trigger-source selector. PTP clock synchronization and scheduled action support
are detected separately. Cameras that expose a PTP clock but not the complete
scheduled-action feature set keep their clock enabled and fall back to the
software barrier. They remain part of the group, but their exposure is marked
unsynchronized. The `ptp_clock_available` capability means only that the clock
controls were found and enabled; the reported PTP state and capture timing say
whether synchronization was actually achieved. A camera with no compatible
software trigger stays available for free-running preview and reports capture
as unavailable instead of failing its entire session. Individual settings
remain effective for ungrouped cameras.

The dashboard keeps this distinction concise: scheduled cameras report current
PTP readiness and offset, while fallback cameras report that their exposure is
software-triggered and unsynchronized. Firmware is omitted from normal status,
and missing GenICam-node diagnostics remain in provider startup logs.

Manual exposure uses `exposure_auto: "Off"` with `exposure_us`, and manual
gain uses `gain_auto: "Off"` with `gain_db`. With either automatic control
enabled, `metering_rate_hz` defaults to 2 Hz and requests unsaved, unencoded
metering frames while a capture group has no prepared cadence. Preparing a
sequence allows one clean initial metering frame, then quiesces background
metering so its PTP action cannot overlap cycle one. A prepared group becomes
capture-ready only after every automatic camera has returned that frame, its
receive pipeline is empty, and `TriggerArmed` is true when the camera exposes
it. That preparation result is latched for the lifetime of the configured
camera session, so an in-flight sequence frame does not temporarily revoke the
group's readiness. Saved sequence frames continue to update camera auto-exposure/gain.
Scheduled frames are matched to their PTP action timestamp; an old metering
frame can never be relabelled as a later saved capture. Static upper-limit
settings remain useful as a tighter image quality or motion limit; the
sequence-derived limit never relaxes a configured `exposure_auto_upper_us`
ceiling.

The live feature service reports cadence-related nodes when available:
`TriggerOverlap`, `TriggerArmed`, `ActionQueueSize`,
`ExposureAutoUpperLimit` (or its known alias), and the synthetic nanosecond
`PacketDelayNs` readback. Synchronization-group status reports the calculated
minimum interval, maximum reliable rate, limiting camera/reason, and smallest
action queue.

`providers.genicam.software_trigger_lead_time_ms` controls the short interval
between arming all group workers and releasing their software triggers. It
defaults to 10 ms and accepts values from 1 through 100 ms; it is preparation
time, not a requested exposure offset.

PTP scheduled capture defaults can be tuned under `providers.genicam.ptp`:

```yaml
ptp:
  action_lead_time_ms: 100
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
