# Vixel provider contract (v2)

A provider adapts one hardware family to the stable Vixel platform API. The
inventory manager and web gateway do not import a vendor SDK. A future USB
camera, thermal camera, or lidar package follows the same control-plane
contract and may publish its own data-plane message types.

For provider name `<provider>`, the provider owns:

- `/vixel/providers/<provider>/observations` (`SensorObservationArray`): all
  currently discoverable hardware, including hardware that is not enrolled.
- `/vixel/providers/<provider>/assignments` (`ProviderAssignmentArray`): a
  transient-local subscription populated by the inventory manager.
- `/vixel/providers/<provider>/status` (`SensorArray`): runtime status for
  enrolled assignments.
- `/vixel/providers/<provider>/group_status` (`SyncGroupArray`): readiness for
  provider-specific synchronization groups.
- `/vixel/providers/<provider>/provision` (`ProvisionSensor`): optional
  enrollment-time hardware provisioning.
- `/vixel/providers/<provider>/capture` (`ProviderCapture`): group capture.
- `/vixel/providers/<provider>/features` (`GetCameraFeatures`): optional live
  camera-feature discovery.

Sensor IDs are immutable inventory keys, not driver names. New generic cameras
use `camera_<vendor>_<serial>`, while IDs created by older providers (for
example `acme_cam_test0001`) remain unchanged. The manager reconciles a provider
observation to known hardware by normalized MAC first and vendor/serial second.
Location and the selected runtime backend must never become part of identity.

`ProviderAssignment.provider_settings_json` contains durable camera settings.
Providers apply supported settings when opening a session and report the
effective document in `Sensor.applied_settings_json`. Invalid requested settings
must fail visibly instead of being silently accepted.

Providers must accept arbitrary assignment counts, remove endpoints for removed
assignments, and avoid changing the persistent inventory themselves.
Enrollment-time network provisioning should be reversible by default. The
generic GigE Vision provider uses volatile GVCP ForceIP assignments and must not
overwrite a camera's persistent static, DHCP, or link-local configuration.

Camera providers publish beneath `/vixel/sensors/<sensor_id>`; the generic
GenICam provider publishes `image_raw`, `image_raw/compressed`, and `camera_info`.
Providers that support persistent camera capture also publish
`image_capture/chunks` as reliable `CaptureFrameChunk` messages. Every chunk for
one frame uses the `capture_id` and timestamp returned by `ProviderCapture`;
indexes start at zero, `chunk_count` is constant, and the reassembled bytes must
be a full-resolution lossless image. The current recorder accepts PNG and
persists decoded BGR8 PNG files.

Chunking is required because full-resolution raw images or a single compressed
sample can exceed practical synchronous DDS sample sizes on multi-interface
camera hosts. Preview transport remains independent and best-effort.

`ProviderCapture.trigger_only` distinguishes publish-only acquisition from the
persistent recorder transport. Providers complete the call after participating
frames have been published, return failed members separately, and report
`trigger_span_ns` when they can measure host dispatch timing. Trigger-only
requests need not publish lossless capture chunks; standard `image_raw` and
camera-info topics remain the processing interface.

Other sensor kinds can use the same base with conventional ROS data topics.
Provider packages should keep transport-library objects and exceptions behind
their package boundary.
