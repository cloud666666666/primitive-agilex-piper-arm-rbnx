---
description: Control one AgileX Piper arm through a selectable ROS 2 or direct-SDK backend.
---

# Piper arm primitive

This package has one manifest, provider identity and contract source. Select
`backend: ros2` (the default) for the original ROS topic interface, or
`backend: sdk` for the MCP and Vitals interface below.

## ROS 2 backend

The ROS backend preserves the original `joint_states`, `joint_command`,
`end_pose`, `pos_command` and vendor `arm_status` topic behavior. It starts the
vendored `piper_ros` driver during activation and stops its process group on
deactivation or shutdown. SDK-only MCP calls are unavailable unless the
instance selects `backend: sdk`.

## Direct-SDK backend

Each deployment instance owns exactly one configured CAN interface. A typical
dual-arm deployment maps `left_piper` to `can_left` and `right_piper` to
`can_right`.

Each instance also exposes the shared health state and stream contracts. They
only read the active SDK connection and report per-joint voltage, current,
motor/driver temperature, enable state, communication state, and fault codes
for Soma and Vitals.

Use `status` before motion and for on-demand health queries. Its joint arrays
are ordered J1 through J6 and include angle, voltage, current, motor/driver
temperature, driver enable, communication, and fault code. Use `jog_joint`
only for small relative joint movements. Joint numbers are 1 through 6 and
positive/negative deltas follow the Piper joint convention. A request is
rejected if it exceeds the configured per-call delta, speed, local URDF limit,
if any current joint would be clamped by Piper firmware, or if hardware motion
is locked.

Use `execute_joint_trajectory` only with operator-recorded and verified
joint-space paths. Each waypoint has a unique name and exactly six absolute
angles in J1-through-J6 order. The complete trajectory is validated before any
enable or motion command, is limited to 32 waypoints by deployment policy, and
uses the same 15 percent speed ceiling as joint jogging. A disabled arm is
enabled automatically and remains enabled; the gripper state is not changed.
Each waypoint requires three consecutive feedback samples with all six joints
inside the configured 1.5 degree position tolerance before execution advances.
A failure stops the sequence and reports the completed count, failed waypoint,
and final feedback. The capability performs no Cartesian interpolation,
collision checking, or perception; intermediate clearance waypoints are
therefore mandatory.

The deployment tolerates up to 3 degrees of feedback outside a nominal joint
limit to accommodate encoder zero offset and slight gravity sag after disable.
Within that tolerance Piper firmware may still clamp an out-of-range axis to
the nominal boundary when any grouped joint command is sent.
Joint motion is considered complete when feedback is within 1.5 degrees of
the requested target.

`enable_arm`, `disable_arm`, `enable_gripper`, and `disable_gripper` expose the
two independent hardware enable states as explicit, idempotent operations.
`enable_arm` and `disable_arm` address joint drivers 1 through 6 individually
because the Piper all-motor selector also changes the factory gripper state on
this hardware. `enable_gripper` and `disable_gripper` send only gripper control
commands and leave all six joint-driver enable states unchanged.
`jog_joint` automatically enables a disabled arm before moving. `set_gripper`
automatically enables the arm and gripper when either is disabled. Motion
waits through the Piper firmware's delayed joint-driver feedback after gripper
motion and restores any joint driver that the firmware drops. Motion commands
retain the resulting enable states after success. An explicit disable changes
only the requested hardware, while a configured shutdown stop first sends a
fast emergency stop and then disables all six arm joint drivers. It also
disables the gripper when that primitive process operated the gripper.

`set_gripper` accepts jaw opening in metres from 0.0 (closed) through 0.07
(open), verifies targets with a 1 mm tolerance, and restores arm joint-driver
enable state even when gripper motion times out. It rejects commands until the
gripper zero has been established. For operator-verified grasp targets at or
below 0.035 m, a closing command also succeeds with `contact_detected=true`
when the jaws close by at least 3 mm from their starting opening, stop before
the requested target, and remain within a 1 mm band for one second while
enabled and fault-free. Contact is based on closing travel and stability, not
the object's absolute width or its distance above the target. This heuristic
is not used for opening or release commands.
`set_gripper_zero` is a one-time calibration rather than a prerequisite repeated
before every movement. The operator must first call `disable_gripper`, verify
`gripper_enabled=false`, manually close the gripper, and then call
`set_gripper_zero` with `fully_closed_confirmed=true`. The six arm joint drivers
may remain enabled because their state is independent. Zeroing rejects an
enabled gripper without sending a command, rejects encoder offsets greater than
10 mm, follows the Piper disabled-command-and-0xAE sequence, verifies zero
feedback, and leaves the gripper disabled. A timed-out gripper command reports
failure without implicitly disabling the arm or gripper.
`emergency_stop` is always available, including while motion is locked. It also
closes the runtime motion gate, so a stopped operation cannot resume or send a
later trajectory frame. The same immediate stop behavior is used by the
gRPC-only `motion_lock` operator capability. Lock requests are deliberately
fail-safe and require no credential. Unlock requests are rejected unless they
carry the owner-only token generated for the current provider activation. The
token is stored as `rbnx-build/data/<provider>.motion-token` with mode `0600`,
is replaced on activation, and is removed on deactivate or shutdown. This
operator capability is intentionally not registered as an MCP tool.

`allow_motion`, `ROBONIX_ALLOW_MOTION`, and the per-arm environment override
choose only the initial motion-gate state during activation. They do not bypass
authorization for a later runtime unlock.

## Locked-motion operator handoff

When `status` reports `motion_unlocked=false`, or a motion capability reports
that hardware motion is locked, Pilot must not retry the motion or attempt to
unlock the arm itself. Pilot must tell the user to run the following command on
the robot host from this Piper package's root directory:

```bash
rbnx-build/venv/bin/python scripts/motion_lock.py unlock \
  --provider <provider-id>
```

Replace `<provider-id>` with the provider selected for the failed request, such
as `left_piper` or `right_piper`. The command must run as the OS user that owns
the active provider's local `0600` token file. Pilot must not ask the user to
reveal the token, print it, or try to call the gRPC unlock capability directly.

After the user reports that the command succeeded, Pilot must call `status` on
the same provider and verify `motion_unlocked=true` before retrying any enable,
joint, trajectory, gripper, or gripper-zero command. If the script or the status
check fails, keep motion blocked and ask the operator to inspect that provider's
logs rather than retrying movement.

This handoff is also included directly in the `detail` field whenever `status`
reports a locked gate or an MCP motion request is rejected by that gate. The
returned command uses the active provider ID and absolute package paths, so Pilot
must quote that exact command to the operator instead of giving a generic safety
console or hardware-authorization instruction.

The primitive does not provide Cartesian, collision-aware, perception, or
camera-guided motion. Never infer a named pose from this interface.
