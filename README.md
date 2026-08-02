# primitive-agilex-piper-arm-rbnx

Robonix package wrapping the **AgileX Piper 6-DoF arm** hardware driver. It
provides the original ROS 2 backend and a direct `piper_sdk` backend behind one
package identity and one `primitive/arm/*` namespace.

Catalog name: `robonix.primitive.agilex.piper.arm`.

> Naming note: internal directory / binary name is `piper_ctl` (`_ctl` = low-level control), while the contract namespace is `arm`. Piper is an arm, so calling this `piper_chassis_rbnx` would mislead.

## Backends

Set `backend` in the deployment instance's `config` block:

| Backend | Default | Control surface |
| --- | --- | --- |
| `ros2` | yes | Original ROS topics for joint feedback, joint/Cartesian commands and Piper status |
| `sdk` | no | MCP arm/gripper control, absolute joint trajectories and Vitals health telemetry |

The default remains `ros2`; existing deployment manifests therefore keep the
same startup and runtime behavior. Robonix delivers instance configuration only
after the package process registers through `Driver(CMD_INIT)`, so
`piper_ctl.main` performs the backend dispatch at that lifecycle boundary.

## Capability surface

### ROS 2 backend

| Contract                                | Mode       | Transport | Source / handler                                     |
| --------------------------------------- | ---------- | --------- | ---------------------------------------------------- |
| `robonix/primitive/arm/driver`          | rpc        | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle gate     |
| `robonix/primitive/arm/joint_states`    | topic_out  | ROS 2     | `/<ns>/joint_states_single` (sensor_msgs/JointState) |
| `robonix/primitive/arm/joint_command`   | topic_in   | ROS 2     | `/<ns>/joint_command` (sensor_msgs/JointState) — driver subscribes; consumers publish |
| `robonix/primitive/arm/end_pose`        | topic_out  | ROS 2     | `/<ns>/end_pose` (geometry_msgs/Pose)                |
| `robonix/primitive/arm/pos_command`     | topic_in   | ROS 2     | `/<ns>/pos_command` (geometry_msgs/Pose) — driver subscribes; consumers publish |
| `robonix/primitive/arm/arm_status`      | topic_out  | ROS 2     | `/<ns>/arm_status` (piper_msgs/PiperStatusMsg) — vendor extension |

The five **standard** contracts (driver / joint_states / joint_command / end_pose / pos_command) mirror robonix's global `capabilities/primitive/arm/*.v1.toml`: same id, version, IDL, and mode. Keep them byte-compatible with the global tree so a future consolidation is a no-op.

`arm_status` is a Piper-specific **vendor extension** and remains package-local. Its IDL `piper_msgs/PiperStatusMsg.msg` is shipped at package level under `capabilities/lib/piper_msgs/msg/PiperStatusMsg.msg`.

> **Single-source-of-truth** for the `PiperStatusMsg` IDL is the vendored `src/piper_msgs/msg/` directory. The mirrored copy under `capabilities/lib/piper_msgs/msg/` is what `rbnx codegen` / atlas's contract registry actually scan. Keep the two in sync — plain file copies are used rather than symlinks to keep `git diff` legible.

### Direct-SDK backend

| Contract | Transport | Purpose |
| --- | --- | --- |
| `robonix/primitive/arm/status` | MCP | Joint, gripper and driver-health status |
| `robonix/primitive/arm/enable_arm` / `disable_arm` | MCP | Independently enable or disable J1-J6 |
| `robonix/primitive/arm/enable_gripper` / `disable_gripper` | MCP | Independently enable or disable the gripper |
| `robonix/primitive/arm/jog_joint` | MCP | Bounded relative movement of one joint |
| `robonix/primitive/arm/execute_joint_trajectory` | MCP | Validated absolute J1-J6 waypoint sequence |
| `robonix/primitive/arm/set_gripper` | MCP | Verified opening or stable-contact grasp |
| `robonix/primitive/arm/set_gripper_zero` | MCP | Operator-confirmed encoder zeroing |
| `robonix/primitive/arm/emergency_stop` | MCP | Piper fast stop |
| `robonix/primitive/health/state` / `stream` | gRPC | Vitals-compatible telemetry |

`set_enabled` remains as a backward-compatible arm-enable operation. The SDK
backend validates finite inputs, Piper hard limits, speed, waypoint count,
feedback age, settling tolerance and hardware faults. Motion is locked by
default. Set `allow_motion: true`, `ROBONIX_ALLOW_MOTION=1`, or the per-arm
override only after clearing the workspace and checking the emergency stop. See
[`CAPABILITY.md`](CAPABILITY.md) for its complete behavioral contract.

### Joint-space vs Cartesian control paths

Two independent command channels; each one bypasses the other:

- **Joint-space (default in the vertical-grasp deploy)**: consumers publish `sensor_msgs/JointState` on `arm/joint_command`. `roboarm_ik` already solves IK, so the driver just forwards the target angles to the Piper SDK's `JointCtrl` + `GripperCtrl`. The `gripper` entry in the JointState carries finger opening.
- **Cartesian**: consumers publish `geometry_msgs/Pose` on `arm/pos_command`. The driver converts the quaternion to xyz-euler and forwards to the Piper SDK's `EndPoseCtrl` (SDK-side / firmware IK). **Gripper is NOT part of this contract** — command it separately through `joint_command`.

## Boot ordering

Boot this **before** any consumer of `primitive/arm/*`. In the vertical-grasp pipeline:

- `primitive-agilex-piper-description-rbnx` consumes `arm/joint_states` to drive `robot_state_publisher`;
- `service-roboarm-ik-rbnx` consumes `arm/joint_states` and publishes to `arm/joint_command` (default path);
- `service-piper-moveit-rbnx` consumes `arm/arm_status` and publishes to `arm/pos_command` (Cartesian path);
- `skill-pick-vertical-grasp-rbnx` polls `arm/arm_status` between grasps and monitors `/arm/joint_states_single` to verify that the gripper is holding an object.

rbnx-cli has no defer/retry, so providers MUST come first in YAML declaration order.

## Driver-init lifecycle

`start.sh` brings up one shared provider process. It does not connect CAN or
spawn ROS. The provider registers `primitive/arm/driver`, then blocks on
`Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as JSON. The lifecycle then runs:

1. `CMD_INIT`: choose `backend` (default `ros2`) and validate only that backend's configuration;
2. `CMD_ACTIVATE` with `ros2`: optionally set up CAN, spawn the ROS driver,
   wait for `JointState`, then declare the five ROS data topics;
3. `CMD_ACTIVATE` with `sdk`: import `piper_sdk`, connect the configured CAN
   interface and publish the MCP/health endpoints without enabling or moving hardware.

`CMD_DEACTIVATE` / `CMD_SHUTDOWN` release the selected backend idempotently:
the ROS backend kills its process group, while the SDK backend disconnects and
applies its configured shutdown stop policy.

## CAN bring-up — TWO paths

The Piper arm uses USB-CAN (MCP251xFD or similar). The CAN interface must be **up** and named `can_piper` (or whatever you set `can_port:` to) before `piper_ctrl_single_node` can talk to it. The CAN interface name is config-driven — the package never hardcodes a device name.

If the configured USB address or interface name looks wrong, inspect
the current host mapping first:

```bash
bash scripts/find_all_can_port.sh
```

Use the reported USB bus path, for example `1-4.2:1.0`, as
`can_usb_address`. Do not guess when multiple CAN interfaces are
present; Jetson onboard `*.mttcan` interfaces and USB-CAN adapters can
coexist.

### Path A — recommended for production (default)

Bring `can_piper` up **outside `rbnx boot`**, ahead of time. On a Jetson with udev rules, this is once-per-boot:

```bash
# Run once per host boot, BEFORE `rbnx boot`:
bash scripts/can_activate.sh can_piper 1000000 "1-4.2:1.0"
# Replace "1-4.2:1.0" with the USB bus path of your Piper.
# To list candidates:
#   lsusb -t
#   sudo ethtool -i can0 | grep bus-info
```

`auto_can_setup` stays `false` in the manifest. This path keeps the deploy free of sudo coupling — the operator approves the sudo prompt once, interactively, then `rbnx boot` runs unattended.

### Path B — convenience for dev laptops

Set `auto_can_setup: true` in the manifest config block. `on_activate` will run `scripts/can_activate.sh` itself before spawning the driver. Requires **passwordless sudo** for the operator (otherwise the script blocks on the prompt and `CMD_ACTIVATE` times out).

```yaml
- name: piper_ctl
  path: ../packages/piper_ctl_rbnx
  config:
    can_port:        can_piper
    can_bitrate:     1000000
    can_usb_address: "1-4.2:1.0"
    auto_can_setup:  true
```

Path B is more convenient when the USB-CAN's bus path shifts between sessions, but couples the deploy to sudoers state — not ideal for production.

## Layout

```
primitive-agilex-piper-arm-rbnx/
├── package_manifest.yaml
├── config.spec                              # union config; backend selector
├── CAPABILITY.md                            # direct-SDK behavior and safety
├── capabilities/
│   ├── primitive/arm/
│   │   ├── driver.v1.toml
│   │   ├── joint_states.v1.toml
│   │   ├── joint_command.v1.toml
│   │   ├── end_pose.v1.toml
│   │   ├── pos_command.v1.toml
│   │   ├── arm_status.v1.toml               # Piper vendor extension
│   │   └── status / enable / trajectory / gripper MCP contracts
│   └── lib/
│       ├── piper_msgs/msg/PiperStatusMsg.msg
│       └── piper_arm/                       # shared SDK MCP IDL
├── piper_ctl/
│   └── main.py                              # shared lifecycle + ROS backend
├── piper_arm_sdk/
│   ├── main.py                              # MCP/health handlers
│   └── controller.py                        # bounded direct-SDK control
├── tests/                                   # offline SDK/lifecycle tests
├── scripts/
│   ├── build.sh                             # colcon + rbnx codegen
│   ├── start.sh                             # source ROS, exec main
│   ├── find_all_can_port.sh                 # print CAN interface ↔ USB bus mapping
│   └── can_activate.sh                      # vendored from upstream piper_ros
└── src/                                     # vendored (no .git, no build/install)
    ├── piper/                               # main ROS 2 driver (rclpy)
    ├── piper_msgs/                          # PiperStatusMsg + Enable.srv (PosCmd kept only for the legacy graspnet_to_poscmd_node)
    └── graspnet_msgs/                       # GraspPose.msg — required by piper/package.xml
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```yaml
backend:             ros2             # default; preserves original behavior
can_port:            can_piper        # CAN interface name
can_bitrate:         1000000
can_usb_address:     ""               # pin via USB bus path; "" = first detected
auto_can_setup:      false            # see "CAN bring-up" above
auto_enable:         true             # forwarded to start_single_piper auto_enable
gripper_exist:       true
gripper_val_mutiple: 2                # upstream typo preserved
arm_namespace:       /arm             # see note in main.py docstring
sentinel_timeout_s:  30.0             # max wait for first JointState in on_activate
# Topic-name overrides (rarely needed; default derives from arm_namespace):
# joint_states_topic:   /arm/joint_states_single
# joint_command_topic:  /arm/joint_command
# arm_status_topic:     /arm/arm_status
# end_pose_topic:       /arm/end_pose
# pos_command_topic:    /arm/pos_command
```

### Selecting a backend in a deployment

Select the backend in the deployment repository's `robonix_manifest.yaml`,
under the `config:` block of each Piper primitive instance. Do not edit this
package's `config.spec` to select a backend, and do not put `backend` in
`.env`. If `backend` is omitted, the instance uses `ros2`.

A dual-arm direct-SDK deployment is configured as follows:

```yaml
primitive:
  - name: left_piper
    path: ./primitives/piper_arm
    config:
      backend: sdk
      arm_label: left
      can_port: can_left
      allow_motion: false

  - name: right_piper
    path: ./primitives/piper_arm
    config:
      backend: sdk
      arm_label: right
      can_port: can_right
      allow_motion: false
```

`scripts/build.sh` automatically fetches the tested `piper_sdk` 0.2.20 source
at commit `4eddfcf817cd87de9acee316a72cf5b988025378`. The checkout is stored under
`rbnx-build/piper_sdk_parent/`, validated without opening CAN, and selected by
`scripts/start.sh`; no host-level SDK installation or manual `PYTHONPATH` is
required.

For controlled offline builds, set `RBNX_PIPER_SDK_PARENT` to an existing
directory that contains a `piper_sdk/` checkout. To skip SDK preparation on a
ROS-only host, set `RBNX_PIPER_SDK_BUILD=0`; starting the SDK backend then
requires the caller to provide `PIPER_SDK_PARENT`. The repository and full
commit SHA may be overridden for a mirror or a deliberately tested revision:

```bash
RBNX_PIPER_SDK_PARENT=/opt/agilex-sdk-parent bash scripts/build.sh
RBNX_PIPER_SDK_BUILD=0 bash scripts/build.sh
RBNX_PIPER_SDK_REPO=https://mirror.example/piper_sdk.git \
RBNX_PIPER_SDK_REF=4eddfcf817cd87de9acee316a72cf5b988025378 \
    bash scripts/build.sh
```

All SDK limits and timeout defaults are documented in
[`config.spec`](config.spec).

## Build / run standalone

```bash
bash scripts/build.sh                                  # ROS build + shared codegen/venv
ROBONIX_ATLAS=127.0.0.1:50051 \
    bash scripts/start.sh                              # registers, awaits Init
```

Offline validation (does not open CAN or command hardware):

```bash
rbnx validate .
rbnx build
PYTHONPATH=. rbnx-build/venv/bin/python -m unittest discover -s tests -v
```

To drive the lifecycle manually (without `rbnx boot`), call the arm's `Driver` service with `CMD_INIT` and a JSON config blob, then call `CMD_ACTIVATE`. Init only validates configuration; Activate returns after the first JointState is observed and the five data topics are declared.

## Verification

```bash
rbnx caps | grep arm
# Expected: piper_ctl provider with
#   robonix/primitive/arm/{driver, joint_states, joint_command,
#                          end_pose, pos_command, arm_status}

ros2 topic hz /arm/joint_states_single             # ~200 Hz
ros2 topic echo /arm/arm_status --once             # PiperStatusMsg fields
ros2 topic echo /arm/end_pose --once               # geometry_msgs/Pose

# Cartesian cmd path (CAREFUL — moves the arm; clear the workspace first):
ros2 topic pub --once /arm/pos_command geometry_msgs/msg/Pose \
    "{position: {x: 0.30, y: 0.0, z: 0.25},
      orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}"

# Joint-space cmd path (also moves the arm — same warning):
ros2 topic pub --once /arm/joint_command sensor_msgs/msg/JointState \
    "{name: [joint1, joint2, joint3, joint4, joint5, joint6, gripper],
      position: [0.0, 0.5, -0.7, 0.0, 0.6, 0.0, 0.04],
      velocity: [], effort: []}"
```

## Vendor / upstream

`src/piper/`, `src/piper_msgs/`, `src/graspnet_msgs/` are verbatim copies from [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros). The other packages in the upstream workspace are deliberately **not** vendored here and land in their own robonix packages:

- `piper_description` / `piper_with_gripper_moveit` → `primitive-agilex-piper-description-rbnx`.
- `piper_humble` / `piper_moveit_control` → `service-piper-moveit-rbnx`.
- `piper_gazebo` / `piper_mujoco` / `piper_sim` → not migrated (sim-only).

If anything diverges from upstream, drop a `*.patch` alongside `src/` documenting the diff.

## License

The existing package and ROS integration are Apache-2.0. The contributed
direct-SDK implementation retains its MIT notice in [`LICENSE-SDK`](LICENSE-SDK).
Vendored piper_ros / piper_msgs / graspnet_msgs keep their respective licenses.
