# AgileX Piper SDK Primitive for Robonix

Robonix primitive for controlling one AgileX Piper arm and its factory CAN
gripper directly through `piper_sdk`. The same package can be instantiated
twice for a dual-arm deployment by assigning separate provider names and
SocketCAN interfaces.

This package is a community SDK backend, not a replacement for the official
[`syswonder/primitive-agilex-piper-arm-rbnx`](https://github.com/syswonder/primitive-agilex-piper-arm-rbnx)
ROS-oriented primitive. Prefer the official package when it provides the
contracts required by your deployment. Use this repository when direct SDK
control is required or when the official package does not yet expose the MCP
arm, gripper, trajectory, and health capabilities listed below. If these
extensions are adopted upstream, this README will be updated to direct new
deployments to the official implementation.

This package was extracted from the Piper SDK adapter developed and field
tested in
[`syswonder/robot-agilex-dual-piper`](https://github.com/syswonder/robot-agilex-dual-piper).
The independent repository contains no workcell waypoints, robot-specific
environment file or private deployment credentials.

## Capability Compatibility

The package keeps the standard Robonix contract IDs, so consumers bind by
capability rather than by this repository or package name.

| Contract | Transport | Purpose |
| --- | --- | --- |
| `robonix/primitive/arm/status` | MCP | Arm, gripper, joint and health status |
| `robonix/primitive/arm/enable_arm` | MCP | Enable J1-J6 without changing gripper state |
| `robonix/primitive/arm/disable_arm` | MCP | Disable J1-J6 without changing gripper state |
| `robonix/primitive/arm/enable_gripper` | MCP | Enable only the factory gripper |
| `robonix/primitive/arm/disable_gripper` | MCP | Disable only the factory gripper |
| `robonix/primitive/arm/jog_joint` | MCP | Bounded relative movement of one joint |
| `robonix/primitive/arm/execute_joint_trajectory` | MCP | Validated absolute J1-J6 waypoint sequence |
| `robonix/primitive/arm/set_gripper` | MCP | Verified opening or stable-contact grasp |
| `robonix/primitive/arm/set_gripper_zero` | MCP | Operator-confirmed gripper encoder zero |
| `robonix/primitive/arm/emergency_stop` | MCP | Piper fast stop |
| `robonix/primitive/health/state` | gRPC | One Vitals-compatible health snapshot |
| `robonix/primitive/health/stream` | gRPC stream | Continuous Vitals-compatible health telemetry |

The legacy `robonix/primitive/arm/set_enabled` contract is retained for
compatibility. New integrations should use the independent arm and gripper
enable contracts.

The following community skills require capabilities supplied by this package:

- [`skill-dual-piper-initialize-rbnx`](https://github.com/syswonder/skill-dual-piper-initialize-rbnx): `status`, `set_gripper_zero`, `enable_arm`, and `enable_gripper`.
- [`skill-dual-piper-transfer-rbnx`](https://github.com/syswonder/skill-dual-piper-transfer-rbnx): `status`, `execute_joint_trajectory`, and `set_gripper`.

## Requirements

- Linux with a configured SocketCAN interface for each Piper arm.
- Robonix `rbnx` 0.1.0 with `robonix-api` registered.
- Python 3 and `uv`.
- AgileX `piper_sdk` 0.2.20 at tested commit
  `4eddfcf817cd87de9acee316a72cf5b988025378`.

Clone the tested SDK revision and expose its parent directory when running
Robonix:

```bash
git clone https://github.com/agilexrobotics/piper_sdk.git
git -C piper_sdk checkout --detach 4eddfcf817cd87de9acee316a72cf5b988025378
export PIPER_SDK_PARENT="$(pwd)"
```

`PIPER_SDK_PARENT` must contain the `piper_sdk/` checkout. For example, when
the SDK is `/opt/agilex/piper_sdk`, set it to `/opt/agilex`.

## Deployment

Use two instances of the same remote package for a dual-Piper robot:

```yaml
primitive:
  - name: left_piper
    url: https://github.com/SJTU-DDST/primitive-agilex-piper-arm-sdk-rbnx
    branch: main
    config:
      arm_label: left
      can_port: can_left
      allow_motion: false

  - name: right_piper
    url: https://github.com/SJTU-DDST/primitive-agilex-piper-arm-sdk-rbnx
    branch: main
    config:
      arm_label: right
      can_port: can_right
      allow_motion: false
```

See [`config.spec`](config.spec) for all limits, timeouts, tolerances and
gripper contact-detection settings. Keep deployment-specific CAN names and
calibration outside this repository.

## Safety

Motion is locked by default. Set `ROBONIX_ALLOW_MOTION=1` in the `rbnx boot`
environment only after checking the physical workspace and emergency stop.
Per-arm `ROBONIX_ALLOW_LEFT_MOTION` and `ROBONIX_ALLOW_RIGHT_MOTION` overrides
take precedence when present.

The primitive validates joint limits, speed, waypoint sizes, finite numeric
targets, motion timeout and feedback tolerance. Gripper zeroing requires an
explicit fully-closed confirmation. Independent arm/gripper enable operations
avoid the Piper all-motor selector because it changes both states on the tested
hardware. With `stop_on_shutdown: true`, deactivation or shutdown sends a fast
stop and disables hardware used by that provider.

This package has no perception, camera input, inverse kinematics, Cartesian
planner or collision checker. Absolute trajectories must be recorded and
validated for the physical workcell under direct operator supervision.

## Build and Test

These checks do not connect to CAN or command hardware:

```bash
rbnx validate .
rbnx build
PYTHONPATH=. rbnx-build/venv/bin/python -m unittest discover -s tests -v
```

Hardware activation and motion tests require the real Piper SDK, configured
SocketCAN interface, a clear workspace and an operator ready to stop the
system.

## License

MIT. See [`LICENSE`](LICENSE).
