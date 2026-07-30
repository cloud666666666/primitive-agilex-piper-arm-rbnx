# Direct Piper SDK Backend Proposal

## Purpose

This contribution adds the complete direct `piper_sdk` primitive under
[`sdk_backend/`](sdk_backend/) for upstream review. The implementation was
developed for a physical dual-Piper deployment and is also published as the
standalone community package
[`SJTU-DDST/primitive-agilex-piper-arm-sdk-rbnx`](https://github.com/SJTU-DDST/primitive-agilex-piper-arm-sdk-rbnx)
at source commit `88feb0d7fce896e061c9a83d7529885d4d3cabc0`.

The current root package remains the official ROS 2 backend. This proposal
does not replace it, change its declared capabilities, or alter its build and
start paths. The two implementations are intentionally presented side by side
so the official ARM primitive maintainers can choose the long-term structure.

## Capability comparison

| Area | Existing ROS 2 backend | Candidate direct-SDK backend |
| --- | --- | --- |
| Lifecycle | Robonix driver starts and stops the vendored ROS driver | Robonix driver connects and disconnects `piper_sdk` directly |
| Joint feedback | `joint_states` ROS 2 topic | MCP `status` plus Vitals health state/stream |
| Joint commands | `joint_command` ROS 2 topic | MCP bounded jog and validated absolute trajectory |
| Cartesian commands | `pos_command` ROS 2 topic | Not provided |
| Gripper commands | Gripper entry in `JointState` | MCP verified opening/contact and explicit zeroing |
| Enable control | Existing ROS driver enable service and startup behavior | Independent arm and gripper enable/disable operations |
| Emergency stop | Managed through the ROS driver lifecycle | Explicit MCP emergency-stop operation |
| Health | Piper status ROS 2 topic | Per-joint voltage, current, temperature, communication, enable and fault telemetry |

Overlapping concepts have not been deduplicated in this proposal. Preserving
both implementations makes behavior and safety differences visible during
review instead of silently changing the existing ROS control path.

## Added SDK contracts

The candidate backend declares these additional contracts:

- `robonix/primitive/arm/status`
- `robonix/primitive/arm/set_enabled`
- `robonix/primitive/arm/enable_arm`
- `robonix/primitive/arm/disable_arm`
- `robonix/primitive/arm/enable_gripper`
- `robonix/primitive/arm/disable_gripper`
- `robonix/primitive/arm/jog_joint`
- `robonix/primitive/arm/execute_joint_trajectory`
- `robonix/primitive/arm/set_gripper`
- `robonix/primitive/arm/set_gripper_zero`
- `robonix/primitive/arm/emergency_stop`
- `robonix/primitive/health/state`
- `robonix/primitive/health/stream`

The SDK backend keeps its own nested `package_manifest.yaml`, capability IDLs,
build/start scripts, configuration specification, tests, README, and MIT
license. This keeps the submitted implementation independently buildable and
avoids claiming that the root ROS package already exposes these contracts.

## Safety behavior

The direct-SDK implementation defaults to motion locked and requires an
explicit environment switch before joint or gripper motion. It validates
finite numeric inputs, Piper joint limits, trajectory size and speed, feedback
freshness, motion timeout, joint settling tolerance, gripper zero state, and
hardware faults before reporting success.

It also avoids Piper's all-motor selector for independent arm enable/disable,
because the selector changed the factory gripper state on the tested hardware.
Gripper commands verify the final opening or a configurable stable-contact
condition and restore arm joint drivers if Piper firmware drops them after a
gripper command. Shutdown can issue a fast stop and disable hardware operated
by that provider.

The implementation has no perception, collision checker, Cartesian planner,
or inverse kinematics. Absolute joint trajectories must be recorded and
validated for the target workcell under operator supervision.

## Offline validation

The candidate backend can be checked without opening CAN or commanding
hardware:

```bash
cd sdk_backend
rbnx validate .
rbnx build
PYTHONPATH=. rbnx-build/venv/bin/python -m unittest discover -s tests -v
```

The submitted test suite covers 49 controller and lifecycle cases, including
input finiteness, hard limits, independent enables, gripper contact detection,
trajectory prevalidation, transient arm-driver drops, motion locking, shutdown
behavior, and read-only health snapshots.

## Maintainer integration options

The upstream maintainers can choose among several follow-up directions:

1. Keep the SDK implementation as a separately named package and link it from
   the official primitive when deployments require its MCP capabilities.
2. Add an explicit backend selector while retaining ROS 2 as the default.
3. Reuse the submitted MCP contracts and safety checks in the ROS backend,
   translating operations through ROS topics and services.
4. Extract shared Piper policy and validation code while leaving transport and
   lifecycle implementations separate.

No option is imposed by this contribution. Its goal is to provide the tested
implementation, contracts, and evidence needed for that decision without
regressing the official backend.
