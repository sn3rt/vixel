builtin_interfaces/Time stamp
string sensor_id
string provider
string kind
string vendor
string model
string serial
string mac_address
bool enrolled
bool enabled
bool online
string lifecycle_state
string display_name
string location_label
bool has_pose
string parent_frame
geometry_msgs/Pose pose
string calibration_url
string topic_base
string network_id
string assigned_address
string current_address
string interface_name
# The first capture selection is retained in sync_group for API compatibility.
string sync_group
string[] capture_group_ids
string operating_mode
# True once capture mode has completed initial metering and its trigger pipeline
# is idle and armed. False while capture preparation is still in progress.
bool capture_ready
string[] capabilities
string last_error
bool managed
string pending_action
string status_detail
string applied_settings_json
# Cumulative GenICam transport health. Counters survive an automatic stream
# rebuild for as long as the provider process remains alive.
string stream_health_state
uint64 stream_completed_buffers
uint64 stream_failed_buffers
uint64 stream_underruns
uint64 stream_resent_packets
uint64 stream_missing_packets
uint64 stream_restart_count
