# Vixel client examples

Build Vixel and source the workspace before using these examples:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

The target group must be online and in `capture` mode. Grouped cameras use PTP
scheduled actions when supported and software fallback otherwise.

## Trigger through HTTP

This asks the `front` group to publish a new frame and prints the trigger
result. It does not download image data because images use ROS topics.

```bash
python3 examples/http_trigger.py front
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

## Trigger through ROS and save the images

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

Use **Capture and save** in the dashboard, the `/vixel/record_capture` action,
or `POST /api/v1/groups/front/capture` when Vixel itself should persist PNGs and
a full capture manifest under its configured recording directory.
