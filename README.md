# so101-legs

3D-printable humanoid legs that reuse the parts from an [SO-101](https://github.com/TheRobotStudio/SO-ARM100) leader and follower arm kit: the Feetech STS3215 serial bus servos and the control boards. The aim is a pair of legs that anyone with an SO-101 kit can print and build, for learning bipedal control and collecting training data for learned policies.

Write-up: [tgreen-fe.github.io/so101-legs.html](https://tgreen-fe.github.io/so101-legs.html)

**Status: work in progress.** Version 3 is designed and simulated in MuJoCo. Nothing has been assembled or walked yet.

## Design constraints

- Only STS3215 servos from an SO-101 kit, accepting their torque and speed limits
- Fully 3D-printable on a consumer FDM printer, no machined parts
- Carries the SO-101 control boards and optionally a battery, or runs tethered
- Light printed torso for an IMU; foot force sensing planned

## Version 3

Four joints per leg: hip yaw, hip pitch, knee, ankle pitch. Hip roll was removed to bring the legs closer together. Compared with version 2: smaller feet, more knee range, a shorter lower leg.

Servos (8 from one leader and follower kit):

| Joint | Servo |
|---|---|
| Hip pitch, knee, ankle (both legs) | Follower STS3215, 12 V, 1/345, about 2.9 Nm stall |
| Hip yaw (both legs) | Leader STS3215, 7.4 V, 1/191, matched pair |

### MuJoCo model

`v3/mujoco/build_model.py` builds the model from the SolidWorks STL exports in `v3/stl/sim/`:

- joint axes found from the servo horn geometry in the meshes
- link masses estimated from mesh volume (20% infill PLA) plus 55 g per servo; about 1.05 kg in total
- joint limits found by sweeping each joint to collision and backing off 5°
- right leg mirrored from the left

```sh
pip install numpy scipy trimesh mujoco
cd v3/mujoco
python build_model.py            # rebuild and re-sweep joint limits
python build_model.py --no-sweep # rebuild using limits.json
python -m mujoco.viewer --mjcf scene.xml
```

`scene.xml` has a floating pelvis and `scene_fixed.xml` pins it. The model has position actuators with a torque limit per motor, an IMU on the pelvis, a touch sensor under each foot and joint position sensing.

Joint limits from the sweep: hip pitch −80° to +110°, knee −26° to +150°, ankle −93° to +98°, hip yaw −11° to +9°.

### Known issue: no sideways balance

Every joint except yaw is a pitch joint, so nothing can shift the body sideways. The centre of mass sits on the centre line, 18 mm inside the inner edge of each foot, and the robot tips towards a lifted foot. Options being considered: add a roll joint at the hip or ankle (one more servo per leg; the kit has spares), reshape the feet, or keep v3 and try an RL rocking gait.

Motor stiffness, damping, armature and friction in the model are estimates and still need tuning against real servos.

## Repository layout

- `v3/parts/`, `v3/pelvis_v3.SLDPRT`: version 3 SolidWorks source; `legs_v3.SLDASM` is the assembly
- `v3/stl/sim/`: per-link STL exports used to build the simulation
- `v3/mujoco/`: model builder, generated MJCF, meshes and joint limits
- Root: version 2 (five joints per leg, hip roll and pitch in one module). `nip_v2.SLDASM` is the v2 assembly; STLs are print-ready v2 parts
- `servo-motor_sts3215-v10.step`: STS3215 model used for fit checks

## Printing

PLA or PETG, 0.2 mm layers. Version 3 print files will be added once the design settles.

## Licence

Hardware and documentation: [CC BY-SA 4.0](LICENSE). Software in `v3/mujoco/`: [MIT](v3/mujoco/LICENSE).
