# Operations and interfaces

## Launch

```bash
ros2 launch vixel cameras_launch.py web_preview:=true
```

The camera backend is the Aravis-based GenICam provider.

The launch shuts down the stack if a core node exits, avoiding stale processes
holding cameras or the dashboard port.

## Dashboard

The web gateway listens on `127.0.0.1:8080`. Forward it from a trusted client:

```bash
ssh -N -L 8080:127.0.0.1:8080 user@camera-host
```

Open <http://127.0.0.1:8080>. The dashboard discovers camera cards dynamically,
shows known and enrolled sensors, controls port modes and groups, and edits
sensor metadata.

Readiness endpoint:

```bash
curl --fail http://127.0.0.1:8080/api/v1/health
```

Cached snapshot example:

```bash
curl --max-time 5 -o camera.jpg \
  http://127.0.0.1:8080/api/v1/sensors/acme_cam_test0001/snapshot
```

## Modes and groups

Sensors use `idle`, `preview`, or `capture` modes. Preview publishes compressed
frames for the dashboard at the configured rate while retaining full-resolution
ROS image topics. Capture belongs to a named sync group.

A `strict` group requires every member. A `degraded` group captures ready
members when another member is unavailable. The generic backend uses host
software triggers and reports captures as not PTP synchronized.

To persist a capture, put the group in capture mode and use **Capture and save**
in the dashboard. A ROS client can invoke the same action:

```bash
ros2 action send_goal /vixel/record_capture \
  vixel_interfaces/action/RecordCapture \
  "{group_id: inspection_pair, request_id: ''}" --feedback
```

An empty request ID generates one. A supplied ID may contain letters, numbers,
`.`, `_`, and `-`. Only one recording runs at a time.

Successful sets are stored below
`/var/lib/vixel/captures/YYYY/MM/DD/<capture_id>/` by default. Each directory
contains one full-resolution lossless PNG per participating camera and a
`manifest.json` with identity, timestamps, camera information, settings,
missing members, and synchronization status. Failed attempts are retained as
`<capture_id>.failed` manifests for diagnosis. Vixel does not automatically
delete captures.

## ROS interfaces

Primary state topics:

- `/vixel/sensors`
- `/vixel/known_sensors`
- `/vixel/ports`
- `/vixel/sync_groups`
- `/vixel/capture_records`

Per-camera topics:

- `/vixel/sensors/<sensor_id>/image_raw`
- `/vixel/sensors/<sensor_id>/image_raw/compressed`
- `/vixel/sensors/<sensor_id>/camera_info`
- `/vixel/sensors/<sensor_id>/image_capture/chunks` (internal reliable,
  lossless capture transport)

Triggered recording uses the `/vixel/record_capture` action. Recent capture
records are also available from `GET /api/v1/captures`; the dashboard currently
lists results but intentionally provides no download or delete endpoint.

Control actions and services cover enrollment, placement resolution, operating
mode, metadata, port mode, known-sensor maintenance, sync groups, capture, and
GenICam feature inspection. Their versioned definitions are in
`vixel_interfaces`.

The HTTP API mirrors the dashboard operations under `/api/v1`. It is an
unauthenticated local control API and must not be exposed directly to untrusted
networks.
