# Vixel client examples

Build Vixel and source the workspace before using these examples:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

The target group must be online. One-shot clients acquire a temporary capture
session automatically; long-running clients should hold and renew their own.
Grouped cameras use PTP scheduled actions when supported and software fallback
otherwise.

## Trigger through HTTP: publish or save

This asks the `front` group to publish a new frame and prints the trigger
result. Publish mode is the default and does not save a file:

```bash
python3 examples/http_trigger.py front --mode publish
```

Use save mode to make Vixel's recorder persist one lossless PNG per camera and
print the resulting directory:

```bash
python3 examples/http_trigger.py front --mode save
```

Use `--base-url` when the gateway is forwarded to a different local port:

```bash
python3 examples/http_trigger.py front --base-url http://127.0.0.1:9080
```

The equivalent request without Python is:

```bash
curl -X POST -H 'Content-Type: application/json' -d '{}' \
  http://127.0.0.1:8080/api/v1/groups/front/trigger
```

For trigger-and-save, replace the final `trigger` with `capture`. A successful
response contains `directory` and `saved_sensor_ids`.

## Trigger through ROS: publish or save

The ROS client selects the corresponding Vixel action from the same mode:

```bash
python3 examples/ros_trigger.py front --mode publish
python3 examples/ros_trigger.py front --mode save
```

Publish mode calls `/vixel/trigger_group`. Save mode calls
`/vixel/record_capture` and prints the server-side directory below
`/var/lib/vixel/captures`.

## Trigger front and back every two seconds

The periodic example sends the `front` and `back` HTTP requests concurrently:

```bash
python3 examples/http_trigger_groups_periodic.py front back --interval 2
```

Use `--count 10` for a finite run; the default continues until Ctrl-C. This
example is publish-only unless `--mode save` is supplied. To persist all front
and back cameras on every cycle:

```bash
python3 examples/http_trigger_groups_periodic.py \
  front back --interval 2 --mode save
```

In publish mode, the helper acquires one capture session for every named group,
renews it while the script runs, releases it on exit, and waits until the cameras
are ready; `--ready-timeout` changes the default 60-second wait. Save mode
resolves multiple selections to an existing aggregate group with the same
camera membership (`front back` resolves to `all4` on this machine), then
submits one server-managed sequence. This keeps one scheduled-action request
and one capture directory per cycle. The server owns its capture session,
applies an exposure ceiling derived from `--interval`, waits for camera/PTP/
metering and trigger-armed readiness, and only then starts the sequence.

Use `--interval 0.5 --count 10` for ten 2 Hz cycles. Pass `--separate-groups`
only when separate front/back manifests are required; those requests are phased
evenly across the interval to reduce transport contention. Add
`--synchronize-groups` only when exact cross-group timing is required and has
been validated on the installed cameras.

Camera acquisition continues on schedule while previous PNGs are encoded and
saved. If a bounded queue fills, the sequence stops scheduling new cycles and
drains all captures already accepted.

## Trigger through ROS and save in the client

This client discovers every member of `front`, subscribes to their full-size
`image_raw` topics, triggers the group, matches frames by the returned
timestamp, and writes PNG files:

```bash
python3 examples/ros_trigger_and_receive.py front \
  --output-dir ./triggered-images
```

Output is written as:

```text
triggered-images/<capture_id>/
├── <sensor_id>.png
├── <sensor_id>.png
└── trigger_result.json
```

The JSON file records the scheduled timestamp, participating and missing
cameras, dispatch span, measured exposure skew, and per-camera synchronization
state.

The equivalent ROS action by itself is:

```bash
ros2 action send_goal /vixel/trigger_group \
  vixel_interfaces/action/TriggerGroup \
  "{group_id: front, request_id: ''}" --feedback
```

To inspect one camera topic without saving:

```bash
ros2 topic echo --once \
  /vixel/sensors/camera_lucid_vision_labs_232400742/image_raw \
  sensor_msgs/msg/Image
```

Start the topic subscriber before sending the trigger. Trigger-only frames are
not retained for subscribers that connect afterward.

Use `ros_trigger.py --mode save` or `http_trigger.py --mode save` when Vixel
itself should persist production PNGs and a full capture manifest under its
configured recording directory. The dashboard's **Test shot** control is for
setup validation and stores its results separately under `test-shots/`.
