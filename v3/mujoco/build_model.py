"""Build the MuJoCo model for the SO-101 legs (v3) from the SolidWorks STL exports.

Inputs : ../stl/sim/{pelvis,hip,upper_leg,lower_leg,foot}.STL
         Exported from the assembly in one shared frame (mm, Y up, -Z forward),
         left leg only, knee bent.
Outputs: meshes/*.stl, so101_legs_body.xml, so101_legs_common.xml, limits.json

Run:  python build_model.py            (re-sweeps joint limits)
      python build_model.py --no-sweep (reuses limits.json)

Needs: numpy, scipy, trimesh, mujoco
"""
import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "stl" / "sim"
MESH_DIR = HERE / "meshes"

# ---------------------------------------------------------------------------
# CAD frame -> MuJoCo frame
# CAD: X = lateral (left leg at -X), Y = up, -Z = forward. Units mm.
# MuJoCo: x forward, y left, z up. Units m.
# Origin: pelvis centre (centre line between the two hip yaw axes).
# ---------------------------------------------------------------------------
R_CAD2MJ = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)
ORIGIN_CAD = np.array([690.6, 892.5, 1336.05])


def to_mj(p):
    return (np.asarray(p, float) - ORIGIN_CAD) @ R_CAD2MJ.T / 1000.0


# Joint axis points, measured from the servo output bosses in the STLs (CAD mm).
# X for the pitch joints is the hip pitch servo centre (any point on the axis works).
JOINT_CAD = {
    "hip_yaw":   np.array([653.0, 897.0, 1336.1]),   # vertical axis, top of yaw horn
    "hip_pitch": np.array([614.35, 838.8, 1337.05]),
    "knee":      np.array([614.35, 728.7, 1300.3]),
    "ankle":     np.array([614.35, 621.8, 1333.65]),
}
JOINT_AXIS = {"hip_yaw": [0, 0, 1], "hip_pitch": [0, 1, 0], "knee": [0, 1, 0], "ankle": [0, 1, 0]}

# link name -> (source STL, proximal joint, number of servos mounted on it)
LINKS = {
    "pelvis":    ("pelvis", None, 2),
    "hip":       ("hip", "hip_yaw", 1),
    "thigh":     ("upper_leg", "hip_pitch", 1),
    "shank":     ("lower_leg", "knee", 1),
    "foot":      ("foot", "ankle", 0),
}
CHAIN = ["hip", "thigh", "shank", "foot"]

# Face counts of the STS3215 STEP model's sub-bodies (case, horns, bosses, screws).
# Used to split each link STL into printed part(s) vs servo.
SERVO_FACE_COUNTS = {15822, 15826, 12918, 12934, 12106, 12122, 5712,
                     4024, 4034, 2470, 2472, 2522, 1092}
SERVO_CASE_FACES = 5712

# Mass estimate
SERVO_MASS = 0.055        # kg, STS3215
PLA_DENSITY = 1.24        # g/cm^3
WALL = 0.12               # cm, 3 perimeters x 0.4 mm
INFILL = 0.20


def printed_mass(m):
    """kg. Walls solid, interior at INFILL."""
    area = m.area / 100.0          # cm^2
    vol = abs(m.volume) / 1000.0   # cm^3
    shell = min(vol, area * WALL)
    return PLA_DENSITY * (shell + INFILL * (vol - shell)) / 1000.0


def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotate_about(points, R, pivot):
    return (points - pivot) @ R.T + pivot


# ---------------------------------------------------------------------------
# 1. Load, split printed/servo, move into MuJoCo frame
# ---------------------------------------------------------------------------
def load_links():
    links = {}
    for name, (src, _, n_servo) in LINKS.items():
        m = trimesh.load(SRC / f"{src}.STL")
        printed, servo = [], []
        for part in m.split(only_watertight=False):
            (servo if len(part.faces) in SERVO_FACE_COUNTS else printed).append(part)
        n_case = sum(len(p.faces) == SERVO_CASE_FACES for p in servo)
        if n_case != n_servo:
            raise RuntimeError(f"{name}: found {n_case} servo cases, expected {n_servo}. "
                               "Servo STEP model changed? Update SERVO_FACE_COUNTS.")
        mass = sum(printed_mass(p) for p in printed)
        pm = trimesh.util.concatenate(printed)
        sm = trimesh.util.concatenate(servo) if servo else None
        for mm in (pm, sm):
            if mm is not None:
                mm.vertices = to_mj(mm.vertices)
        links[name] = {"printed": pm, "servo": sm, "printed_mass": mass, "n_servo": n_servo}
    return links


# ---------------------------------------------------------------------------
# 2. Straighten the leg: zero pose = straight leg, foot flat
# ---------------------------------------------------------------------------
def straighten(links):
    J = {k: to_mj(v) for k, v in JOINT_CAD.items()}
    H, K, A = J["hip_pitch"], J["knee"], J["ankle"]
    tilt_thigh = np.arctan2(K[0] - H[0], H[2] - K[2])   # +ve = knee ahead of hip
    tilt_shank = np.arctan2(A[0] - K[0], K[2] - A[2])
    # absolute rotation applied to each link to reach the straight pose
    rot = {"hip": 0.0, "thigh": tilt_thigh, "shank": tilt_shank, "foot": 0.0}

    K_new = rotate_about(K[None], rot_y(tilt_thigh), H)[0]
    A_new = K_new + rot_y(tilt_shank) @ (A - K)
    pivot_old = {"thigh": H, "shank": K, "foot": A}
    pivot_new = {"thigh": H, "shank": K_new, "foot": A_new}
    for name in ("thigh", "shank", "foot"):
        R = rot_y(rot[name])
        for key in ("printed", "servo"):
            mm = links[name][key]
            if mm is not None:
                mm.vertices = (mm.vertices - pivot_old[name]) @ R.T + pivot_new[name]
    joints = {"hip_yaw": J["hip_yaw"], "hip_pitch": H, "knee": K_new, "ankle": A_new}
    # exported (crouched) pose expressed as joint angles from the straight zero pose
    home = {"hip_yaw": 0.0, "hip_pitch": -tilt_thigh,
            "knee": tilt_thigh - tilt_shank, "ankle": tilt_shank}
    return joints, home


# ---------------------------------------------------------------------------
# 3. Joint-limit sweep (point-cloud contact test between link surfaces)
# ---------------------------------------------------------------------------
def mirror_pts(p):
    q = p.copy()
    q[:, 1] *= -1
    return q


def sweep_limits(links, joints, step_deg=1.0, max_deg=170.0, backoff_deg=5.0,
                 depth=0.0006, search=0.004, axis_exclusion=0.013, density=1.5e6, seed=0):
    """For each left-leg joint, rotate the subtree until its surface penetrates any
    other link (incl. parent and the mirrored right leg) by more than `depth`.
    Penetration = moving surface point lies behind the nearest static surface sample
    (along that sample's outward normal), so flush faces sliding past each other
    don't count. Points within `axis_exclusion` of the joint axis are ignored
    (horn/boss interface)."""
    rng = np.random.default_rng(seed)

    def sample(name):
        pts, nrm = [], []
        for key in ("printed", "servo"):
            mm = links[name][key]
            if mm is not None:
                n = max(2000, int(mm.area * density))
                p, fi = trimesh.sample.sample_surface(mm, n, seed=int(rng.integers(1 << 31)))
                pts.append(p)
                nrm.append(mm.face_normals[fi])
        return np.vstack(pts), np.vstack(nrm)

    S = {n: sample(n) for n in LINKS}

    def mirrored(n):
        p, q = S[n]
        return mirror_pts(p), mirror_pts(q)

    limits, report = {}, {}
    for i, jname in enumerate(["hip_yaw", "hip_pitch", "knee", "ankle"]):
        moving_links = CHAIN[i:]
        static = [S[n] for n in ["pelvis"] + CHAIN[:i]] + [mirrored(n) for n in CHAIN]
        sp = np.vstack([a for a, _ in static])
        sn = np.vstack([b for _, b in static])
        tree = cKDTree(sp)
        moving = np.vstack([S[n][0] for n in moving_links])
        pivot = joints[jname]
        axis = np.array(JOINT_AXIS[jname], float)
        rel = moving - pivot
        radial = np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1)
        moving = moving[radial > axis_exclusion]

        def contacts(deg):
            th = np.radians(deg)
            R = rot_z(th) if jname == "hip_yaw" else rot_y(th)
            p = rotate_about(moving, R, pivot)
            d, idx = tree.query(p, distance_upper_bound=search, workers=-1)
            ok = np.isfinite(d)
            pen = np.einsum("ij,ij->i", p[ok] - sp[idx[ok]], sn[idx[ok]])
            return int((pen < -depth).sum())

        base = contacts(0.0)
        lim = []
        for sign in (-1, 1):
            hit = None
            deg = step_deg
            while deg <= max_deg:
                if contacts(sign * deg) > base + 20:
                    hit = sign * deg
                    break
                deg += step_deg
            lim.append(sign * max_deg if hit is None else hit - sign * backoff_deg)
        limits[jname] = [float(np.radians(lim[0])), float(np.radians(lim[1]))]
        report[jname] = {"deg": [lim[0], lim[1]], "baseline_penetrating_points": base}
        print(f"  {jname:9s} {lim[0]:+7.1f} .. {lim[1]:+7.1f} deg  (baseline {base})")
    return limits, report


# ---------------------------------------------------------------------------
# 4. Write meshes in link frames, and the MJCF fragments
# ---------------------------------------------------------------------------
def export_meshes(links, joints):
    MESH_DIR.mkdir(exist_ok=True)
    origin = {"pelvis": np.zeros(3), "hip": joints["hip_yaw"], "thigh": joints["hip_pitch"],
              "shank": joints["knee"], "foot": joints["ankle"]}
    info = {}
    for name, L in links.items():
        sides = [("", False)] if name == "pelvis" else [("_L", False), ("_R", True)]
        for suffix, mirror in sides:
            for key in ("printed", "servo"):
                mm = L[key]
                if mm is None:
                    continue
                m = mm.copy()
                o = origin[name].copy()
                if mirror:
                    m.vertices = mirror_pts(m.vertices)
                    m.invert()
                    o[1] *= -1
                m.vertices = m.vertices - o
                m.export(MESH_DIR / f"{name}{suffix}_{key}.stl")
        info[name] = origin[name]
    return info


def f(v):
    return " ".join(f"{x:.6g}" for x in v)


def leg_xml(side, joints, links, limits, foot_box):
    s = 1 if side == "L" else -1
    def P(p):
        p = np.array(p, float).copy()
        p[1] *= s
        return p

    def lim(j):
        lo, hi = limits[j]
        if j == "hip_yaw" and side == "R":   # mirrored hardware: yaw range mirrors
            lo, hi = -hi, -lo
        return lo, hi

    def geoms(link, indent):
        L = links[link]
        pad = " " * indent
        out = [f'{pad}<geom class="visual_printed" mesh="{link}_{side}_printed" mass="{L["printed_mass"]:.4f}"/>']
        if L["servo"] is not None:
            out.append(f'{pad}<geom class="visual_servo" mesh="{link}_{side}_servo" mass="{SERVO_MASS * L["n_servo"]:.4f}"/>')
        if link != "foot":
            out.append(f'{pad}<geom class="collision" mesh="{link}_{side}_printed"/>')
        return "\n".join(out)

    hy, hp, kn, an = (P(joints[k]) for k in ("hip_yaw", "hip_pitch", "knee", "ankle"))
    yaw_class = "sts3215_191"
    fb_c, fb_h = P(foot_box[0]) - an, foot_box[1]
    return f"""
    <body name="hip_{side}" pos="{f(hy)}">
      <joint name="hip_yaw_{side}" class="{yaw_class}" axis="0 0 1" range="{lim('hip_yaw')[0]:.4f} {lim('hip_yaw')[1]:.4f}"/>
{geoms('hip', 6)}
      <body name="thigh_{side}" pos="{f(hp - hy)}">
        <joint name="hip_pitch_{side}" class="sts3215_345" axis="0 1 0" range="{lim('hip_pitch')[0]:.4f} {lim('hip_pitch')[1]:.4f}"/>
{geoms('thigh', 8)}
        <body name="shank_{side}" pos="{f(kn - hp)}">
          <joint name="knee_{side}" class="sts3215_345" axis="0 1 0" range="{lim('knee')[0]:.4f} {lim('knee')[1]:.4f}"/>
{geoms('shank', 10)}
          <body name="foot_{side}" pos="{f(an - kn)}">
            <joint name="ankle_{side}" class="sts3215_345" axis="0 1 0" range="{lim('ankle')[0]:.4f} {lim('ankle')[1]:.4f}"/>
{geoms('foot', 12)}
            <geom name="foot_{side}" class="foot" type="box" pos="{f(fb_c)}" size="{f(fb_h)}"/>
            <site name="foot_{side}" type="box" pos="{f(fb_c - [0, 0, fb_h[2]])}" size="{f([fb_h[0] + 0.003, fb_h[1] + 0.003, 0.004])}" rgba="1 0 0 0.3" group="4"/>
          </body>
        </body>
      </body>
    </body>"""


def write_xml(links, joints, limits, home):
    # Foot sole box (MuJoCo frame, straight pose), from the foot's largest flat printed piece
    fp = links["foot"]["printed"]
    sole = max(fp.split(only_watertight=False), key=lambda m: m.area)
    lo, hi = sole.bounds
    foot_box = ((lo + hi) / 2, (hi - lo) / 2)

    body = f"""<mujoco>
  <!-- Generated by build_model.py. Pelvis contents + both legs. Include inside the pelvis body. -->
  <geom class="visual_printed" mesh="pelvis_printed" mass="{links['pelvis']['printed_mass']:.4f}"/>
  <geom class="visual_servo" mesh="pelvis_servo" mass="{SERVO_MASS * 2:.4f}"/>
  <geom class="collision" mesh="pelvis_printed"/>
  <site name="imu" pos="0 0 0" size="0.008" rgba="0 0.6 1 1" group="4"/>
{leg_xml('L', joints, links, limits, foot_box)}
{leg_xml('R', joints, links, limits, foot_box)}
</mujoco>
"""
    (HERE / "so101_legs_body.xml").write_text(body)

    mesh_lines = ['    <mesh name="pelvis_printed" file="pelvis_printed.stl" inertia="shell"/>',
                  '    <mesh name="pelvis_servo" file="pelvis_servo.stl" inertia="convex"/>']
    for link in CHAIN:
        for side in ("L", "R"):
            mesh_lines.append(f'    <mesh name="{link}_{side}_printed" file="{link}_{side}_printed.stl" inertia="shell"/>')
            if links[link]["servo"] is not None:
                mesh_lines.append(f'    <mesh name="{link}_{side}_servo" file="{link}_{side}_servo.stl" inertia="convex"/>')

    acts = []
    for side in ("L", "R"):
        for j in ("hip_yaw", "hip_pitch", "knee", "ankle"):
            cls = "sts3215_191" if j == "hip_yaw" else "sts3215_345"
            acts.append(f'    <position name="{j}_{side}" joint="{j}_{side}" class="{cls}" inheritrange="1"/>')

    sens = ['    <framequat name="imu_quat" objtype="site" objname="imu"/>',
            '    <gyro name="imu_gyro" site="imu"/>',
            '    <accelerometer name="imu_acc" site="imu"/>',
            '    <velocimeter name="imu_vel" site="imu"/>',
            '    <touch name="foot_touch_L" site="foot_L"/>',
            '    <touch name="foot_touch_R" site="foot_R"/>']
    for side in ("L", "R"):
        for j in ("hip_yaw", "hip_pitch", "knee", "ankle"):
            sens.append(f'    <jointpos name="{j}_{side}_pos" joint="{j}_{side}"/>')

    common = f"""<mujoco>
  <!-- Generated by build_model.py. Shared defaults, assets, actuators and sensors. -->
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>

  <default>
    <!-- STS3215 12V 1/345 (SO-101 follower): ~2.9 Nm stall -->
    <default class="sts3215_345">
      <joint damping="0.6" armature="0.028" frictionloss="0.052"/>
      <position kp="17.8" kv="0" forcerange="-2.94 2.94"/>
    </default>
    <!-- STS3215 7.4V 1/191 (SO-101 leader): torque scaled from 1/345 by gear ratio, ~1.06 Nm -->
    <default class="sts3215_191">
      <joint damping="0.33" armature="0.0086" frictionloss="0.029"/>
      <position kp="9.9" kv="0" forcerange="-1.06 1.06"/>
    </default>
    <default class="visual_printed">
      <geom type="mesh" contype="0" conaffinity="0" group="2" rgba="0.92 0.55 0.2 1"/>
    </default>
    <default class="visual_servo">
      <geom type="mesh" contype="0" conaffinity="0" group="2" rgba="0.15 0.15 0.15 1"/>
    </default>
    <default class="collision">
      <geom type="mesh" contype="1" conaffinity="0" group="3" mass="0" rgba="0.5 0.5 1 0.3"/>
    </default>
    <default class="foot">
      <!-- feet collide with floor and each other -->
      <geom contype="3" conaffinity="2" group="3" mass="0" friction="0.8 0.02 0.001" condim="4" rgba="0.5 0.5 1 0.3"/>
    </default>
  </default>

  <asset>
{chr(10).join(mesh_lines)}
  </asset>

  <actuator>
{chr(10).join(acts)}
  </actuator>

  <sensor>
{chr(10).join(sens)}
  </sensor>
</mujoco>
"""
    (HERE / "so101_legs_common.xml").write_text(common)
    return foot_box


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-sweep", action="store_true")
    args = ap.parse_args()

    links = load_links()
    joints, home = straighten(links)
    print("Masses (kg):")
    for n, L in links.items():
        print(f"  {n:7s} printed {L['printed_mass']:.3f} + servos {SERVO_MASS * L['n_servo']:.3f}")
    print("Home (exported) pose, deg:", {k: round(np.degrees(v), 1) for k, v in home.items()})

    lim_file = HERE / "limits.json"
    if args.no_sweep and lim_file.exists():
        limits = json.loads(lim_file.read_text())["limits_rad"]
    else:
        print("Sweeping joint limits:")
        limits, report = sweep_limits(links, joints)
        lim_file.write_text(json.dumps({"limits_rad": limits, "sweep": report}, indent=2))

    export_meshes(links, joints)
    foot_box = write_xml(links, joints, limits, home)
    sole_z = foot_box[0][2] - foot_box[1][2]
    (HERE / "pose.json").write_text(json.dumps(
        {"home_deg": {k: float(np.degrees(v)) for k, v in home.items()},
         "home_rad": home, "sole_z_straight": float(sole_z)}, indent=2))
    print(f"Sole height below pelvis origin (straight legs): {-sole_z:.4f} m")


if __name__ == "__main__":
    main()
