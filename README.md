# so101-legs

Open-source 3D-printable humanoid legs built around the same STS3215 serial bus servos used by the [SO-101](https://github.com/TheRobotStudio/SO-ARM100) leader/follower arm kit.

The SO-101 arm made low-cost robot arms something anyone can print and assemble at home. This project aims to do the same for a bipedal lower body: a pair of legs that use an SO-101 motor set, print on a consumer FDM printer, and give a cheap platform for learning bipedal control and collecting training data for learned (VLA-style) policies.

**Status: early — CAD in progress, first parts printed.** Nothing here is finished or walking yet.

## Design goals

- Same actuators as an SO-101 follower/leader pair (Feetech STS3215) — no expensive actuators, accepting the torque/speed limits that come with that
- Fully 3D-printable structure, no machined parts
- Carries the two SO-101 control boards and optionally a battery, or runs tethered
- Lightweight printed torso to mount an IMU
- Foot force sensing planned
- Control software and sim-to-real work to follow once the hardware is proven

## Joints per leg

| Joint | Part(s) |
|---|---|
| Hip yaw | `hip_yaw_L`, `hip_yaw_R` |
| Hip roll | `hip_roll_v2` |
| Hip pitch | `hip_pitch`, `hip_upper_v3` |
| Knee | `leg_upper_v2`, `knee_collar`, `leg_lower_v2` |
| Ankle / foot | `feet_v2` |

Shared: `pelvis`, `torso`.

## Repository layout

- `*.SLDPRT` / `*.SLDASM` — SolidWorks source. `nip_v2.SLDASM` is the current full assembly; `leg_assembly.SLDASM` / `lower_assem.SLDASM` are earlier single-leg assemblies.
- `*.STL` — print-ready exports of the current parts.
- `servo-motor_sts3215-v10.step` — STS3215 servo model used for fit checks.
- Files without a version suffix or with `_v1` are superseded but kept for reference.

## Printing

PLA/PETG, 0.2 mm layers. Parts are oriented for minimal supports; the hip and pelvis parts print as one plate.

## Licence

Hardware and documentation: [CC BY-SA 4.0](LICENSE). Software (when added) will be MIT.
