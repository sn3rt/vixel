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
members when another member is unavailable. Group membership automatically
selects `Action0`: cameras with PTP scheduled-action support synchronize to the
PC grandmaster, while unsupported cameras remain in the capture using the
software barrier and are reported as unsynchronized.
Capability detection follows GenICam/SFNC features rather than camera brand. It
accepts current `Ptp*` nodes and legacy `GevIEEE1588*` timestamp aliases used by
some vendors. Capture requests for a capable camera that is still acquiring
lock are rejected before any trigger is dispatched; scheduled `Action0` capture
becomes available automatically after lock.
`PtpOffsetFromMaster` is optional because some compliant cameras do not expose
it. For those cameras Vixel uses `PtpStatus=Slave` for readiness and validates
the resulting synchronization from the per-frame device timestamps and reported
exposure skew. Some cameras expose the optional offset node only after reaching
`Slave`; Vixel re-probes the node after lock so a hotplug does not require a mode
change or session restart.

Grouped cameras keep PTP enabled in every active mode. Preview mode uses a
software trigger so the dashboard can fetch independent frames; changing the
group to capture mode rebuilds the session with `Action0` armed for synchronized
group requests.

Results include per-camera timing records, `exposure_skew_ns` for synchronized
members, and `within_tolerance`. `trigger_span_ns` remains command-dispatch
timing and must not be interpreted as exposure skew. A grouped capture with two
or more synchronized frames fails when its measured exposure skew exceeds the
configured PTP tolerance; writing both files is not considered success.

Use **Trigger and publish** for processing without writing files. Subscribe to
each member's `image_raw` topic before triggering; the resulting images use the
shared `scheduled_time` returned by the request for correlation:

```bash
ros2 action send_goal /vixel/trigger_group \
  vixel_interfaces/action/TriggerGroup \
  "{group_id: inspection_pair, request_id: ''}" --feedback
```

The HTTP equivalent is:

```bash
curl -X POST -H 'Content-Type: application/json' -d '{}' \
  http://127.0.0.1:8080/api/v1/groups/inspection_pair/trigger
```

Runnable Python clients for HTTP triggering and ROS image reception are in
[the examples guide](../examples/README.md).

Vixel can schedule saved captures internally. The sequence service assigns
exposure timestamps ahead of time, so PNG encoding and disk writes from earlier
cycles do not delay later triggers. Trigger-only applications may still call the
action at their own cadence.

To persist a capture, put the group in capture mode and use **Capture and save**
in the dashboard. A ROS client can invoke the same action:

```bash
ros2 action send_goal /vixel/record_capture \
  vixel_interfaces/action/RecordCapture \
  "{group_id: inspection_pair, request_id: ''}" --feedback
```

An empty request ID generates one. A supplied ID may contain letters, numbers,
`.`, `_`, and `-`. Recordings for disjoint groups and successive recordings on
the same cameras may overlap. Acquisition, PNG encoding, chunk transport, and
disk persistence are separate pipeline stages bounded by configured limits.

The example clients expose both operations with the same mode names:

```bash
# ROS action client
python3 examples/ros_trigger.py front --mode publish
python3 examples/ros_trigger.py front --mode save

# HTTP client
python3 examples/http_trigger.py front --mode publish
python3 examples/http_trigger.py front --mode save
```

Only save mode produces files. Publish mode returns a trigger ID and publishes
the frames on the camera topics for subscribers.

To record groups on a fixed interval, use the server-managed sequence:

```bash
python3 examples/http_trigger_groups_periodic.py \
  front back --interval 0.5 --mode save --count 10
```

The server accepts the request in a `preparing` state, atomically reserves both
groups, requests capture mode with the 500 ms interval, and waits for camera,
cadence, PTP, automatic-metering, pipeline-idle, and trigger-armed readiness.
Only then does it schedule the
first cycle. A camera that cannot bound automatic exposure or cannot meet the
interval fails preparation before any trigger is fired. The preparation
deadline is configured with `recording.sequence_prepare_timeout_ms`.

Each cycle creates a separate capture directory and manifest per group. The
example requests one shared timestamp for every included group. The requested
500 ms cadence is based on exposure timestamps; completed files may appear
later and out of order. If a queue reaches its configured limit, Vixel stops
scheduling that sequence and finishes every capture it already accepted.

The equivalent HTTP request returns `202 Accepted` immediately:

```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '{"group_ids":["front","back"],"interval_ms":500,"count":10,"synchronize_groups":true,"metadata":{"job":"example"}}' \
  http://127.0.0.1:8080/api/v1/capture-operations/sequence
```

Poll `GET /api/v1/capture-operations/<operation_id>` or subscribe to
`/vixel/capture_operations`. A count of zero runs until cancelled with
`POST /api/v1/capture-operations/<operation_id>/cancel`. One-shot asynchronous
batches use `POST /api/v1/capture-operations/batch`.

Operation status includes `capture_id_count`, the recent `capture_ids` list,
and `capture_ids_truncated`. By default, Vixel keeps the newest 100 IDs per
operation and the newest 100 terminal operations for later lookup. Active
operations are retained until they finish, and new work is rejected when the
configured active-operation limit is reached. These bounds affect only status
memory and messages: capture directories and manifests stay on disk. Configure
the limits with `recording.operation_capture_id_limit`,
`recording.operation_history_limit`, and `recording.max_active_operations`.

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
- `/vixel/capture_operations`

Per-camera topics:

- `/vixel/sensors/<sensor_id>/image_raw`
- `/vixel/sensors/<sensor_id>/image_raw/compressed`
- `/vixel/sensors/<sensor_id>/camera_info`
- `/vixel/sensors/<sensor_id>/image_capture/chunks` (internal reliable,
  lossless capture transport)

Triggered recording uses the `/vixel/record_capture` action. Recent capture
records are also available from `GET /api/v1/captures`; the dashboard currently
lists results but intentionally provides no download or delete endpoint.

Trigger-only acquisition uses the `/vixel/trigger_group` action. The HTTP API
exposes the same operation at `POST /api/v1/groups/<group_id>/trigger`; saved
capture remains at `POST /api/v1/groups/<group_id>/capture`.

Asynchronous recording uses `/vixel/submit_capture_batch` and
`/vixel/start_capture_sequence`; the recorder coordinates sequence setup with
the internal `/vixel/prepare_capture_groups` service. Operation status and
cancellation use `/vixel/get_capture_operation` and
`/vixel/cancel_capture_operation`. Caller
metadata is stored as a JSON object in every resulting manifest. Optional GPS
metadata is sampled without waiting for a fix.

Control actions and services cover enrollment, placement resolution, operating
mode, metadata, port mode, known-sensor maintenance, sync groups, capture, and
GenICam feature inspection. Their versioned definitions are in
`vixel_interfaces`.

The HTTP API mirrors the dashboard operations under `/api/v1`. It is an
unauthenticated local control API and must not be exposed directly to untrusted
networks.
