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
shows known and enrolled sensors, controls port modes and selections, and edits
sensor metadata. It is an inspection and configuration surface: production
capture is intentionally started by ROS or a client script, not by dashboard
buttons. Camera and selection cards provide a saved **Test shot** for setup
checks. Click an active camera preview to open its focused view at
`/sensors/<sensor_id>`. That view keeps the latest cached frame visible while
showing the same camera details and controls as the dashboard card.

Archiving a camera is blocked while it belongs to any selection. Edit or delete
those selections first; Vixel never silently changes selection membership as a
side effect of archiving.

Readiness endpoint:

```bash
curl --fail http://127.0.0.1:8080/api/v1/health
```

Cached snapshot example:

```bash
curl --max-time 5 -o camera.jpg \
  http://127.0.0.1:8080/api/v1/sensors/acme_cam_test0001/snapshot
```

## Modes and capture selections

Sensors use `idle`, `preview`, or `capture` modes. In preview mode, camera
exposures run at the configured rate only while the dashboard or another raw or
compressed ROS image subscriber is present. Dashboard demand expires five
seconds after its last image request. Capture targets a named camera selection
and is independent of browser preview demand. The existing ROS and HTTP APIs
retain the `group` and `SyncGroup` names for compatibility.

A camera may belong to multiple selections, such as `front`, `back`, and `all`.
A `strict` selection requires every member. A `degraded` selection captures
ready members when another member is unavailable. Enrolled GenICam cameras use
`Action0` so capable cameras remain synchronized to the PC grandmaster even
while unselected or idle. Selecting cameras does not make them fire; the capture
operation supplies the target time and only its selected cameras receive an
action.
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

Enrolled GenICam cameras on approved networks keep PTP enabled, including while
they are unselected or every containing selection is idle. Preview mode uses a
software trigger so the dashboard can fetch independent frames; changing the
selection to capture mode rebuilds the session with `Action0` armed for scheduled
requests.

For example, these selections may coexist:

```text
front = front_left, front_right
back  = back_left, back_right
all   = front_left, front_right, back_left, back_right
```

Create or replace `all` through the HTTP API:

```bash
curl -X PUT -H 'Content-Type: application/json' \
  -d '{"member_ids":["front_left","front_right","back_left","back_right"],"missing_policy":"strict"}' \
  http://127.0.0.1:8080/api/v1/groups/all
```

Disjoint selections can run independently, for example periodic `front` and
machine-position-triggered `back`. Selections with shared cameras and the same
requested timestamp are deduplicated into one physical exposure and fanned out
to each logical capture. Different timestamps are accepted when they satisfy
the camera's negotiated minimum interval; a request that is too close is
rejected rather than silently shifted. Disjoint selections given the same
requested timestamp also use the same camera-clock action time.

Results include per-camera timing records, `exposure_skew_ns` for synchronized
members, and `within_tolerance`. `trigger_span_ns` remains command-dispatch
timing and must not be interpreted as exposure skew. A grouped capture with two
or more synchronized frames fails when its measured exposure skew exceeds the
configured PTP tolerance; writing both files is not considered success.

Use the trigger action for processing without writing files. Subscribe to each
member's `image_raw` topic before triggering; the resulting images use the
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

To persist a production capture, invoke the recorder action from a ROS node or
client script:

```bash
ros2 action send_goal /vixel/record_capture \
  vixel_interfaces/action/RecordCapture \
  "{group_id: inspection_pair, request_id: ''}" --feedback
```

An empty request ID generates one. A supplied ID may contain letters, numbers,
`.`, `_`, and `-`. Recordings for disjoint selections may overlap. Captures
selecting the same cameras share an exposure at an identical target or are
accepted only when their distinct targets meet the negotiated interval.
Acquisition, PNG encoding, chunk transport, and
disk persistence are separate pipeline stages bounded by configured limits.
One-shot actions without a session ID automatically acquire a temporary capture
session, wait for the target to become ready, and restore the configured idle or
preview baseline afterward.

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

Dashboard test shots are saved separately below
`/var/lib/vixel/captures/test-shots/YYYY/MM/DD/<capture_id>/`. A selection test
uses the same PTP scheduled-action, acquisition, lossless PNG, and manifest path
as production. A camera test captures only that camera. Tests take an exclusive
session and are rejected when any selected camera is already owned by active
capture work. Test results have `capture_kind=test_shot`; they are shown in a
separate dashboard history and are never deleted automatically.

## Capture sessions

Long-running ROS controllers should acquire a lease before starting work,
include its `session_id` in every trigger or record goal, renew it while active,
and release it on shutdown. A session declares its sensor or selection targets,
requested cadence, owner label, and whether it is exclusive. Expired leases are
removed automatically, so crashed clients do not leave cameras stuck in capture
mode. The manager then restores each camera to the launch baseline: preview when
`web_preview:=true`, otherwise idle.

The ROS services are `/vixel/acquire_capture_session`,
`/vixel/renew_capture_session`, and `/vixel/release_capture_session`. Current
leases are published on `/vixel/capture_sessions`. The HTTP equivalents are:

```text
POST   /api/v1/capture-sessions
PATCH  /api/v1/capture-sessions/<session_id>
DELETE /api/v1/capture-sessions/<session_id>
GET    /api/v1/capture-sessions
```

For example, acquire a 250 ms lease for two selections and copy the returned
`session.session_id` into subsequent trigger requests:

```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '{"owner_id":"machine-controller","target_kind":"group","target_ids":["front","back"],"requested_interval_ms":250,"exclusive":false}' \
  http://127.0.0.1:8080/api/v1/capture-sessions
```

The periodic publish example manages this lifecycle automatically. Saved
server-side sequences also own and renew their session internally.

## ROS interfaces

Primary state topics:

- `/vixel/sensors`
- `/vixel/known_sensors`
- `/vixel/ports`
- `/vixel/sync_groups`
- `/vixel/capture_records`
- `/vixel/capture_operations`
- `/vixel/capture_sessions`

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

Dashboard test shots use `/vixel/start_test_shot`. They appear as asynchronous
capture operations, so the browser remains responsive while cameras prepare and
files are written.

Control actions and services cover enrollment, placement resolution, operating
mode, metadata, port mode, known-sensor maintenance, sync groups, capture, and
GenICam feature inspection. Their versioned definitions are in
`vixel_interfaces`.

The HTTP API mirrors the dashboard operations under `/api/v1`. It is an
unauthenticated local control API and must not be exposed directly to untrusted
networks.
