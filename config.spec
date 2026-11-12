config:
  # string, default: ros2; accepted values: ros2, sdk.
  # ros2 preserves the package's original behavior. sdk selects the direct
  # piper_sdk MCP/gripper/health implementation.
  backend: ros2

  # Common CAN interface setting. The ros2-compatible default is can_piper;
  # SDK deployments commonly override this with can_left or can_right.
  can_port: can_piper

  # ROS 2 backend options. Ignored by the SDK backend.
  can_bitrate: 1000000
  can_usb_address: ""
  auto_can_setup: false
  auto_enable: true
  gripper_exist: true
  gripper_val_mutiple: 2
  # float, metres, default: 0.10; range: 0.001..0.10.
  # 夹爪机械行程。Piper SDK 默认按 gripper_range=[0, 0.07] 把夹爪命令钳到
  # 70 mm（piper_param_manager.py），但这台夹爪上位机里显示的行程是 100 mm；
  # 不放开的话上层永远开不到满开口，>70 mm 的物体就会"夹不住"。
  # ros2 backend 会在驱动节点里关掉该 SDK 钳位并用本值做软件限位。
  gripper_stroke_m: 0.10
  arm_namespace: /arm
  sentinel_timeout_s: 30.0
  # joint_states_topic: /arm/joint_states_single
  # joint_command_topic: /arm/joint_command
  # arm_status_topic: /arm/arm_status
  # end_pose_topic: /arm/end_pose
  # pos_command_topic: /arm/pos_command

  # Direct-SDK backend options below. Ignored by the ROS 2 backend.
  # string; deployment instance label used in responses and diagnostics.
  # Use a stable value such as left or right.
  arm_label: left

  # boolean, default: false. Initial deployment safety-gate state; the
  # ROBONIX_ALLOW_MOTION environment switch can override it at boot. Runtime
  # unlock uses the operator-only motion_lock gRPC interface and session token.
  allow_motion: false

  # float, degrees, default: 5.0; must be > 0.
  # Maximum relative movement accepted by one jog request.
  max_jog_deg: 5.0

  # integer, default: 32; range: 1..256.
  max_trajectory_waypoints: 32

  # integer, percent; 1 <= default_speed_percent <= max_speed_percent <= 100.
  default_speed_percent: 8
  max_speed_percent: 15

  # float, seconds; both values must be > 0.
  feedback_timeout_s: 6.0
  motion_timeout_s: 12.0

  # float, degrees, default: 1.5; range: 0.1..5.0.
  # Every joint must remain within this error to complete a waypoint.
  joint_position_tolerance_deg: 1.5

  # float, metres, default: 0.07; must be > 0.
  gripper_max_opening_m: 0.07

  # float, default: 1.0; must be > 0. Converts metres to the SDK command scale.
  gripper_multiplier: 1.0

  # float, newton-metres, default: 1.5; range: 0.5..5.0.
  max_gripper_effort_nm: 1.5

  # float, metres, default: 0.01; range: 0.001..gripper_max_opening_m.
  # Reject zeroing when the manually closed encoder offset exceeds this value.
  gripper_zero_max_offset_m: 0.01

  # float, metres, default: 0.0005; range:
  # 0.0001..gripper_zero_max_offset_m. Maximum absolute feedback accepted after
  # the 0xAE zero command. Tune per gripper from measured hardware behavior.
  gripper_zero_confirmation_tolerance_m: 0.0005

  # float, seconds, default: 1.5; range: 0..10.
  gripper_zero_settle_s: 1.5

  # float, metres, default: 0.001; range: 0.0001..0.005.
  gripper_position_tolerance_m: 0.001

  # float, seconds, default: 1.25; range: 0 <= value < feedback_timeout_s.
  # Require arm-driver feedback to remain enabled after gripper commands.
  gripper_arm_stability_s: 1.25

  # float, metres, default: 0.035; range: 0..gripper_max_opening_m.
  # Only closing targets at or below this value use contact detection.
  gripper_contact_max_target_m: 0.035

  # float, metres, default: 0.003; greater than both gripper position
  # thresholds and <= gripper_max_opening_m. A closing grasp must reduce the
  # opening by at least this much before a stable obstruction counts as contact.
  gripper_contact_min_closure_m: 0.003

  # float, seconds, default: 1.0; range: 0.01 <= value < motion_timeout_s.
  gripper_contact_stability_s: 1.0

  # float, metres, default: 0.001; range: 0.00005..0.001.
  # Maximum feedback movement during the stable-contact interval.
  gripper_contact_position_epsilon_m: 0.001

  # float, degrees, default: 3.0; range: 0..5.0.
  # Encoder offset or gravity-sag allowance outside nominal URDF limits.
  joint_limit_tolerance_deg: 3.0

  # float, seconds, default: 0.02; range: 0.005..0.1.
  # Period used to refresh Piper streaming control targets.
  command_period_s: 0.02

  # boolean, default: true. Stop and disable hardware used by this provider
  # when the managed deployment shuts down.
  stop_on_shutdown: true
