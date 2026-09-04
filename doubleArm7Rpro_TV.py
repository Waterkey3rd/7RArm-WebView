#!/usr/bin/env python3
"""
MuJoCo dual-arm gravity compensation check.

This script builds a MuJoCo model in Python using the same geometry and
installation convention as roboArmTV/doubleArm7R.m.

Install in WSL:

    python3 -m pip install mujoco numpy
    python3 mujoco_double_arm_gravity_check.py

The model uses metres and radians.  The printed error compares:

    tau_mujoco  = data.qfrc_bias at qvel = 0
    tau_formula = closed-form 7R gravity compensation formula

The visualizer also applies the closed-form gravity compensation torque, not
MuJoCo's qfrc_bias.  qfrc_bias is kept only as a reference for verification.
"""

from __future__ import annotations

import math
import tempfile
import time
import argparse
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np


def rot_y(theta: float) -> np.ndarray:
    """Rotation about +Y, using the same right-handed convention as the arm model."""
    s = math.sin(theta)
    c = math.cos(theta)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


# ----------------------------- robot constants -----------------------------

MM = 1.0e-3

UPPER_ARM = 158.50 * MM
ELBOW_OFFSET = 60.0 * MM
FOREARM = 249.38 * MM
TOOL_LENGTH = 92.57 * MM
BASE_SEPARATION_Y = 220.0 * MM

# Logical joint limits deployed by Arm/Src/ArmMiddleware.cpp.  MuJoCo names
# these hinges q1..q7, while the firmware calls the same entries J0..J6.
DEPLOY_JOINT_LIMITS_DEG = {
    "left": np.array(
        [[-150.0, 150.0], [-179.0, 10.0], [-179.0, 179.0],
         [0.0, 200.0], [-179.0, 179.0], [-120.0, 120.0],
         [-179.0, 269.0]],
        dtype=float,
    ),
    "right": np.array(
        [[-150.0, 150.0], [-179.0, 10.0], [-179.0, 179.0],
         [0.0, 200.0], [-179.0, 179.0], [-120.0, 120.0],
         [-269.0, 179.0]],
        dtype=float,
    ),
}
DEPLOY_JOINT_LIMITS_RAD = {
    side: np.deg2rad(limits) for side, limits in DEPLOY_JOINT_LIMITS_DEG.items()
}

WAIST_UPRIGHT_ANGLE = 0.0
WAIST_LYING_ANGLE = math.radians(90.0)
WAIST_STEP = math.radians(5.0)
WAIST_JOINT_NAME = "waist_pitch"
WAIST_BODY_NAME = "waist_pitch_body"
WAIST_RGBA = "0.25 0.72 0.50 1"
J0_FROM_PITCH_X = 192.5 * MM
J0_FROM_PITCH_Z = 365.0 * MM
WAIST_LINK_HALF_WIDTH = 50.0 * MM
WAIST_LINK_HALF_THICKNESS = 30.0 * MM
WAIST_AXIS_RADIUS = 18.0 * MM
WAIST_AXIS_LENGTH = 100.0 * MM

# Keep the arm mass model identical to makeDeployIdealGravityModel() in
# ArmMiddleware.cpp.
UPPER_ARM_MASS = 0.938 + (43.0 + 30.15) / 1000.0
FOREARM_MASS = 1.051 + (100.0 + 10.625) / 1000.0
TOOL_MASS = 1.055 + (80.0 + 18.49) / 1000.0

UPPER_ARM_COM = np.array([-0.11190, 0.0, 0.0])
FOREARM_COM = np.array([0.07461, 0.0, 0.08903])
TOOL_COM = np.array([-0.01064, 0.0, 0.0])

# Pitch.cpp equivalent mass and COM.  CAD obtains this COM by replacing both
# complete arms with one 11 kg point mass at the midpoint of the two J0 axes.
PITCH_MASS = 12.13
PITCH_COM = np.array([0.17998, 0.0, 0.34771])

# Physical mass split supplied for deployment:
#   fixed pitch structure (excluding both J0 motors): 1.13 kg
#   two complete arms including their J0 motors:      11.00 kg
# The deployed 7R gravity model contains only the masses that create torque
# about J0.  Add the missing mass at each J0 axis; it creates pitch torque but
# exactly zero J0 torque.
PITCH_FIXED_MASS = 1.13
COMPLETE_ARM_MASS = 11.0 / 2.0
ARM_GRAVITY_MODEL_MASS = UPPER_ARM_MASS + FOREARM_MASS + TOOL_MASS
J0_AXIS_LUMP_MASS = COMPLETE_ARM_MASS - ARM_GRAVITY_MODEL_MASS

# Recover the fixed structure's own COM from the specified total first moment.
PITCH_FIXED_COM = (
    PITCH_MASS * PITCH_COM
    - 11.0 * np.array([J0_FROM_PITCH_X, 0.0, J0_FROM_PITCH_Z])
) / PITCH_FIXED_MASS

# MuJoCo requires every moving body to have positive mass and inertia.  The
# generated kinematic carrier bodies below are not physical links, so give
# them a numerically negligible inertial placeholder.
DUMMY_BODY_MASS = 1.0e-12
DUMMY_BODY_INERTIA = 1.0e-12

GRAVITY_ACCEL = 9.81

# Constant bias term b_i in the requested compensation law:
#   tau_g_i = sigma * g * dH/dq_i + b_i
# Put calibrated static/friction offsets here if needed.
GRAVITY_BIAS_LEFT = np.zeros(7)
GRAVITY_BIAS_RIGHT = np.zeros(7)

LEFT_ARM_RGBA = "0.10 0.45 0.95 1"
RIGHT_ARM_RGBA = "0.95 0.45 0.10 1"
JOINT_RGBA = "0.08 0.08 0.08 1"
COM_RGBA = "0.95 0.10 0.10 0.75"

TRANSLATION_STEP = 0.01  # metres per key press/repeat
ROTATION_STEP = math.radians(5.0)  # radians per key press/repeat
IK_DAMPING = 1.0e-3
IK_MAX_Q_STEP = math.radians(8.0)

# Product-of-exponentials data equivalent to the current Deploy MDH model.
# Axes and link vectors are expressed in each moving local frame.
H = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)

P = np.array(
    [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [-UPPER_ARM, 0.0, 0.0],
        [FOREARM, 0.0, ELBOW_OFFSET],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [TOOL_LENGTH, 0.0, 0.0],
    ],
    dtype=float,
)

# Mass-weighted vectors in the requested potential-height formula:
#   H(q) = e_y^T (R03*A + R04*B + R07*C)
# Units are kg*m, so g*dH/dq has units N*m.
#
# The three COM vectors and masses below are copied from ArmMiddleware.cpp.
A_GRAV = UPPER_ARM_MASS * UPPER_ARM_COM + (FOREARM_MASS + TOOL_MASS) * P[3]
B_GRAV = FOREARM_MASS * FOREARM_COM + TOOL_MASS * P[4]
C_GRAV = TOOL_MASS * TOOL_COM

# Current doubleArm7R.m installation.  RWorldFromRobot maps a local robot
# vector into the world frame.
R_WORLD_FROM_LEFT = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ]
)
P_WORLD_LEFT = np.array([0.0, BASE_SEPARATION_Y / 2.0, 0.0])

R_WORLD_FROM_RIGHT = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ]
)
P_WORLD_RIGHT = np.array([0.0, -BASE_SEPARATION_Y / 2.0, 0.0])

# The pitch shaft is the model origin.  In its zero-angle (arm-raised) frame,
# both J0 axes are x=192.5 mm and z=365.0 mm; their lateral separation is
# unchanged.  Positive pitch rotates the assembly from raised toward lying.
P_WORLD_WAIST = np.zeros(3)
P_WAIST_LEFT = np.array([J0_FROM_PITCH_X, BASE_SEPARATION_Y / 2.0, J0_FROM_PITCH_Z])
P_WAIST_RIGHT = np.array([J0_FROM_PITCH_X, -BASE_SEPARATION_Y / 2.0, J0_FROM_PITCH_Z])
# At waist=0 the scene keeps the physical pitch-to-J0 structure.  Deployment
# CartesianSpace, however, is defined around the J0 bases and omits this common
# structure offset.  Visualizer UI/export code must subtract this vector.
DEPLOY_SCENE_ORIGIN_OFFSET_M = np.array([J0_FROM_PITCH_X, 0.0, J0_FROM_PITCH_Z])
R_WAIST_FROM_LEFT = R_WORLD_FROM_LEFT
R_WAIST_FROM_RIGHT = R_WORLD_FROM_RIGHT

# In the pitch-body frame, this vector points from the pitch shaft to the
# midpoint between the two J0 axes at the raised (motor 0 deg) position.
P_WAIST_ARM_CENTER = 0.5 * (P_WAIST_LEFT + P_WAIST_RIGHT)
WAIST_LINK_CENTER = 0.5 * P_WAIST_ARM_CENTER
WAIST_LINK_HALF_LENGTH = 0.5 * float(np.linalg.norm(P_WAIST_ARM_CENTER))


@dataclass(frozen=True)
class ArmSpec:
    prefix: str
    base_pos: np.ndarray
    base_rot: np.ndarray


ARMS = (
    ArmSpec("left", P_WAIST_LEFT, R_WAIST_FROM_LEFT),
    ArmSpec("right", P_WAIST_RIGHT, R_WAIST_FROM_RIGHT),
)


# ----------------------------- math helpers --------------------------------


def fmt_vec(v: Iterable[float]) -> str:
    return " ".join(f"{x:.12g}" for x in v)


def mat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to MuJoCo's w x y z quaternion."""
    R = np.asarray(R, dtype=float)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qw, qx, qy, qz], dtype=float)
    return q / np.linalg.norm(q)


def rot_axis(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def inertial_xml(name: str, pos: np.ndarray, mass: float) -> str:
    # Small isotropic inertia.  It is not meant to identify rotational link
    # inertia; for a gravity-only check, mass and COM location are decisive.
    inertia = max(mass * 1.0e-4, 1.0e-6)
    return (
        f'<body name="{name}" pos="{fmt_vec(pos)}">\n'
        f'  <inertial pos="0 0 0" mass="{mass:.12g}" '
        f'diaginertia="{inertia:.12g} {inertia:.12g} {inertia:.12g}"/>\n'
        f'  <geom type="sphere" size="0.025" contype="0" conaffinity="0" '
        f'rgba="{COM_RGBA}"/>\n'
        f'</body>\n'
    )


def dummy_inertial_xml(indent: str) -> str:
    return (
        f'{indent}<inertial pos="0 0 0" mass="{DUMMY_BODY_MASS:.12g}" '
        f'diaginertia="{DUMMY_BODY_INERTIA:.12g} '
        f'{DUMMY_BODY_INERTIA:.12g} {DUMMY_BODY_INERTIA:.12g}"/>'
    )


def visual_link_xml(indent: str, name: str, vector: np.ndarray, rgba: str) -> str:
    if np.linalg.norm(vector) < 1.0e-9:
        return (
            f'{indent}<geom name="{name}_joint_marker" type="sphere" '
            f'size="0.018" contype="0" conaffinity="0" rgba="{JOINT_RGBA}"/>'
        )
    return (
        f'{indent}<geom name="{name}_link" type="capsule" '
        f'fromto="0 0 0 {fmt_vec(vector)}" size="0.012" '
        f'contype="0" conaffinity="0" rgba="{rgba}"/>'
    )


# ----------------------------- MJCF builder --------------------------------


def build_arm_xml(arm: ArmSpec) -> str:
    qbase = mat_to_quat_wxyz(arm.base_rot)
    arm_rgba = LEFT_ARM_RGBA if arm.prefix == "left" else RIGHT_ARM_RGBA
    s = [
        f'<body name="{arm.prefix}_base" pos="{fmt_vec(arm.base_pos)}" '
        f'quat="{fmt_vec(qbase)}">'
    ]
    s.append(
        f'  <geom name="{arm.prefix}_base_marker" type="cylinder" '
        f'size="0.04 0.018" contype="0" conaffinity="0" rgba="{arm_rgba}"/>'
    )
    s.append(
        "  "
        + inertial_xml(
            f"{arm.prefix}_j0_axis_lumped_mass",
            np.zeros(3),
            J0_AXIS_LUMP_MASS,
        ).replace("\n", "\n  ").rstrip()
    )

    indent = "  "
    # P[0] is p01.  It is zero for the current Deploy arm, but kept here so
    # the model documents the same H/P structure as the MATLAB solver.
    s.append(f'{indent}<body name="{arm.prefix}_p01" pos="{fmt_vec(P[0])}">')
    indent += "  "

    for i in range(7):
        axis = H[i]
        joint_range = DEPLOY_JOINT_LIMITS_RAD[arm.prefix][i]
        s.append(f'{indent}<body name="{arm.prefix}_j{i + 1}" pos="0 0 0">')
        s.append(dummy_inertial_xml(indent + "  "))
        s.append(
            f'{indent}  <geom name="{arm.prefix}_j{i + 1}_axis" type="sphere" '
            f'size="0.016" contype="0" conaffinity="0" rgba="{JOINT_RGBA}"/>'
        )
        s.append(
            f'{indent}  <joint name="{arm.prefix}_q{i + 1}" type="hinge" '
            f'axis="{fmt_vec(axis)}" limited="true" '
            f'range="{fmt_vec(joint_range)}" damping="0" armature="0"/>'
        )

        if i == 2:
            s.append(
                indent
                + "  "
                + inertial_xml(
                    f"{arm.prefix}_upper_arm_com",
                    UPPER_ARM_COM,
                    UPPER_ARM_MASS,
                ).replace("\n", "\n" + indent + "  ").rstrip()
            )
        elif i == 3:
            s.append(
                indent
                + "  "
                + inertial_xml(
                    f"{arm.prefix}_forearm_com",
                    FOREARM_COM,
                    FOREARM_MASS,
                ).replace("\n", "\n" + indent + "  ").rstrip()
            )
        elif i == 6:
            s.append(
                indent
                + "  "
                + inertial_xml(
                    f"{arm.prefix}_tool_com",
                    TOOL_COM,
                    TOOL_MASS,
                ).replace("\n", "\n" + indent + "  ").rstrip()
            )

        # Link vector after this joint, before the next joint.
        s.append(
            visual_link_xml(
                indent + "  ",
                f"{arm.prefix}_p{i + 2:02d}",
                P[i + 1],
                arm_rgba,
            )
        )
        s.append(
            f'{indent}  <body name="{arm.prefix}_p{i + 2:02d}" '
            f'pos="{fmt_vec(P[i + 1])}">'
        )
        indent += "    "
        s.append(dummy_inertial_xml(indent))

    # Add a site at the end of the tool for visual/debug convenience.
    s.append(
        f'{indent}<site name="{arm.prefix}_tool_tip" type="sphere" '
        f'size="0.015" rgba="0.1 0.1 1 1"/>'
    )

    # Close p08, j7, p07, j6, ...
    for _ in range(7):
        indent = indent[:-4]
        s.append(f"{indent}  </body>")
        s.append(f"{indent}</body>")

    indent = indent[:-2]
    s.append(f"{indent}</body>")
    s.append("</body>")
    return "\n".join(s)


def build_mjcf() -> str:
    arms_xml = "\n".join(
        "      " + line if line else line
        for arm in ARMS
        for line in build_arm_xml(arm).splitlines()
    )
    arm_actuators = "\n".join(
        f'<motor name="{arm.prefix}_tau{i}" joint="{arm.prefix}_q{i}" '
        f'gear="1" ctrllimited="false"/>'
        for arm in ARMS
        for i in range(1, 8)
    )
    actuators = "\n".join(
        [
            # The real pitch motor's positive torque is opposite MuJoCo's
            # +Y generalized joint torque.
            f'<motor name="waist_tau" joint="{WAIST_JOINT_NAME}" '
            f'gear="-1" ctrllimited="false"/>',
            arm_actuators,
        ]
    )
    return f"""
<mujoco model="deploy_dual_7r_waist_link_no_plane">
  <compiler angle="radian" coordinate="local"/>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <visual>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.75 0.75 0.75" specular="0.2 0.2 0.2"/>
  </visual>
  <default>
    <joint limited="false"/>
    <geom density="0" friction="0 0 0"/>
  </default>
  <worldbody>
    <light name="key_light" pos="0 -1.5 1.2" dir="0 1 -1" diffuse="0.9 0.9 0.9"/>
    <body name="{WAIST_BODY_NAME}" pos="{fmt_vec(P_WORLD_WAIST)}">
      <inertial pos="0 0 0" mass="{DUMMY_BODY_MASS:.12g}" diaginertia="{DUMMY_BODY_INERTIA:.12g} {DUMMY_BODY_INERTIA:.12g} {DUMMY_BODY_INERTIA:.12g}"/>
      <joint name="{WAIST_JOINT_NAME}" type="hinge" axis="0 1 0" limited="true" range="{WAIST_UPRIGHT_ANGLE:.12g} {WAIST_LYING_ANGLE:.12g}" damping="0" armature="0"/>
      {inertial_xml("pitch_fixed_remainder_com", PITCH_FIXED_COM, PITCH_FIXED_MASS)}
      <geom name="waist_center_link" type="box" pos="{fmt_vec(WAIST_LINK_CENTER)}" size="{WAIST_LINK_HALF_LENGTH:.12g} {WAIST_LINK_HALF_WIDTH:.12g} {WAIST_LINK_HALF_THICKNESS:.12g}" contype="0" conaffinity="0" rgba="{WAIST_RGBA}"/>
      <geom name="waist_pitch_axis" type="cylinder" size="{WAIST_AXIS_RADIUS:.12g} {0.5 * WAIST_AXIS_LENGTH:.12g}" euler="1.57079632679 0 0" contype="0" conaffinity="0" rgba="0.08 0.08 0.08 1"/>
{arms_xml}
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
</mujoco>
""".strip()


# ------------------------- gravity compensation check -----------------------


def joint_dof_indices(model: mujoco.MjModel, prefix: str) -> list[int]:
    ids = []
    for i in range(1, 8):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}_q{i}")
        ids.append(int(model.jnt_dofadr[joint_id]))
    return ids


def mass_body_names(prefix: str) -> tuple[str, str, str]:
    return (
        f"{prefix}_upper_arm_com",
        f"{prefix}_forearm_com",
        f"{prefix}_tool_com",
    )


def tool_site_name(prefix: str) -> str:
    return f"{prefix}_tool_tip"


def arm_qpos_indices(model: mujoco.MjModel, prefix: str) -> list[int]:
    indices = []
    for i in range(1, 8):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}_q{i}")
        indices.append(int(model.jnt_qposadr[joint_id]))
    return indices


def waist_joint_id(model: mujoco.MjModel) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, WAIST_JOINT_NAME)


def waist_qpos_index(model: mujoco.MjModel) -> int:
    return int(model.jnt_qposadr[waist_joint_id(model)])


def waist_dof_index(model: mujoco.MjModel) -> int:
    return int(model.jnt_dofadr[waist_joint_id(model)])


def clamp_waist_angle(angle: float) -> float:
    return float(np.clip(angle, WAIST_UPRIGHT_ANGLE, WAIST_LYING_ANGLE))


def set_waist_angle(model: mujoco.MjModel, data: mujoco.MjData, angle: float) -> None:
    data.qpos[waist_qpos_index(model)] = clamp_waist_angle(angle)
    data.qvel[waist_dof_index(model)] = 0.0
    data.qacc[waist_dof_index(model)] = 0.0


def all_arm_dof_indices(model: mujoco.MjModel) -> list[int]:
    dofs: list[int] = []
    for arm in ARMS:
        dofs.extend(joint_dof_indices(model, arm.prefix))
    return dofs


def gravity_test_poses() -> list[tuple[np.ndarray, np.ndarray]]:
    tests = [
        (
            np.zeros(7),
            np.zeros(7),
        ),
        (
            np.array([0.2, -0.4, 0.3, -0.8, 0.5, 0.4, -0.2]),
            np.array([-0.2, -0.4, -0.3, -0.8, -0.5, 0.4, 0.2]),
        ),
        (
            np.array([-0.6, 0.8, -0.5, 1.0, 0.4, -0.7, 0.3]),
            np.array([0.6, 0.8, 0.5, 1.0, -0.4, -0.7, -0.3]),
        ),
    ]

    rng = np.random.default_rng(7)
    for _ in range(10):
        ql = rng.uniform(-1.2, 1.2, 7)
        qr = np.array([-ql[0], ql[1], -ql[2], ql[3], -ql[4], ql[5], -ql[6]])
        tests.append((ql, qr))
    return tests


def gravity_direction_for_arm_local(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmSpec) -> np.ndarray:
    """Return world +Z expressed in the current local arm-base frame."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{arm.prefix}_base")
    rotation_world_from_arm = np.array(data.xmat[body_id]).reshape(3, 3)
    direction = rotation_world_from_arm.T @ np.array([0.0, 0.0, 1.0])
    norm = np.linalg.norm(direction)
    if norm <= 1.0e-12:
        raise ValueError(f"{arm.prefix}: invalid gravity direction in local frame")
    return direction / norm


def gravity_bias_for_arm(arm: ArmSpec) -> np.ndarray:
    if arm.prefix == "left":
        return GRAVITY_BIAS_LEFT
    if arm.prefix == "right":
        return GRAVITY_BIAS_RIGHT
    raise ValueError(f"unknown arm prefix: {arm.prefix}")


def gravity_formula_7r_local(
    q: np.ndarray,
    gravity_direction: np.ndarray,
    bias: np.ndarray,
    A: np.ndarray = A_GRAV,
    B: np.ndarray = B_GRAV,
    C: np.ndarray = C_GRAV,
) -> np.ndarray:
    """Closed-form 7R gravity compensation for any local gravity direction.

    Implements:

        tau_g_i = g * u^T * dS(q)/dq_i + b_i
        S(q) = R03*A + R04*B + R07*C

    u is world +Z expressed in the current local arm-base frame.  A/B/C are
    mass-weighted local vectors in kg*m, so the output is N*m.
    """
    q = np.asarray(q, dtype=float).reshape(7)
    gravity_direction = np.asarray(gravity_direction, dtype=float).reshape(3)
    bias = np.asarray(bias, dtype=float).reshape(7)
    q1, q2, q3, q4, q5, q6, q7 = q
    ux, uy, uz = gravity_direction

    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s3, c3 = math.sin(q3), math.cos(q3)
    s4, c4 = math.sin(q4), math.cos(q4)
    s5, c5 = math.sin(q5), math.cos(q5)
    s6, c6 = math.sin(q6), math.cos(q6)
    s7, c7 = math.sin(q7), math.cos(q7)

    Ax, Ay, Az = A
    Bx, By, Bz = B
    Cx, Cy, Cz = C

    Dx = Cx * c6 - Cy * s6 * s7 - Cz * s6 * c7
    Dy = (
        -Cx * s5 * s6
        + Cy * (c5 * c7 - s5 * c6 * s7)
        + Cz * (-c5 * s7 - s5 * c6 * c7)
    )
    Dz = (
        Cx * c5 * s6
        + Cy * (s5 * c7 + c5 * c6 * s7)
        + Cz * (-s5 * s7 + c5 * c6 * c7)
    )

    X = Bx + Dx
    Y = By + Dy
    Z = Bz + Dz

    alpha_x = c1 * c2 * c4 - c1 * s2 * c3 * s4 - s1 * s3 * s4
    beta_x = -s1 * c3 + c1 * s2 * s3
    gamma_x = -s1 * s3 * c4 - c1 * s2 * c3 * c4 - c1 * c2 * s4

    alpha_y = s1 * c2 * c4 - s1 * s2 * c3 * s4 + c1 * s3 * s4
    beta_y = s1 * s2 * s3 + c1 * c3
    gamma_y = -s1 * s2 * c3 * c4 - s1 * c2 * s4 + c1 * s3 * c4

    alpha_z = s2 * c4 + c2 * c3 * s4
    beta_z = -c2 * s3
    gamma_z = -s2 * s4 + c2 * c3 * c4

    Tx1 = (
        -Ax * s1 * c2
        + Ay * (-s1 * s2 * s3 - c1 * c3)
        + Az * (s1 * s2 * c3 - c1 * s3)
        + X * (s1 * s2 * c3 * s4 - s1 * c2 * c4 - c1 * s3 * s4)
        + Y * (-s1 * s2 * s3 - c1 * c3)
        + Z * (s1 * s2 * c3 * c4 + s1 * c2 * s4 - c1 * s3 * c4)
    )
    Ty1 = (
        Ax * c1 * c2
        + Ay * (-s1 * c3 + c1 * s2 * s3)
        + Az * (-s1 * s3 - c1 * s2 * c3)
        + X * alpha_x
        + Y * beta_x
        + Z * gamma_x
    )
    Tz1 = 0.0

    Tx2 = (
        -Ax * c1 * s2
        + Ay * c1 * c2 * s3
        - Az * c1 * c2 * c3
        + X * (-c1 * s2 * c4 - c1 * c2 * c3 * s4)
        + Y * (c1 * c2 * s3)
        + Z * (c1 * s2 * s4 - c1 * c2 * c3 * c4)
    )
    Ty2 = (
        -Ax * s1 * s2
        + Ay * s1 * c2 * s3
        - Az * s1 * c2 * c3
        + X * (-s1 * s2 * c4 - s1 * c2 * c3 * s4)
        + Y * (s1 * c2 * s3)
        + Z * (s1 * s2 * s4 - s1 * c2 * c3 * c4)
    )
    Tz2 = (
        Ax * c2
        + Ay * s2 * s3
        - Az * s2 * c3
        + X * (c2 * c4 - s2 * c3 * s4)
        + Y * (s2 * s3)
        + Z * (-s2 * c3 * c4 - c2 * s4)
    )

    Tx3 = (
        Ay * (s1 * s3 + c1 * s2 * c3)
        + Az * (-s1 * c3 + c1 * s2 * s3)
        + X * (-s1 * c3 * s4 + c1 * s2 * s3 * s4)
        + Y * (s1 * s3 + c1 * s2 * c3)
        + Z * (-s1 * c3 * c4 + c1 * s2 * s3 * c4)
    )
    Ty3 = (
        Ay * (s1 * s2 * c3 - c1 * s3)
        + Az * (s1 * s2 * s3 + c1 * c3)
        + X * (s1 * s2 * s3 * s4 + c1 * c3 * s4)
        + Y * (s1 * s2 * c3 - c1 * s3)
        + Z * (s1 * s2 * s3 * c4 + c1 * c3 * c4)
    )
    Tz3 = (
        -Ay * c2 * c3
        - Az * c2 * s3
        + X * (-c2 * s3 * s4)
        + Y * (-c2 * c3)
        + Z * (-c2 * s3 * c4)
    )

    Tx4 = (
        X * (-s1 * s3 * c4 - c1 * s2 * c3 * c4 - c1 * c2 * s4)
        + Z * (s1 * s3 * s4 + c1 * s2 * c3 * s4 - c1 * c2 * c4)
    )
    Ty4 = (
        X * (-s1 * s2 * c3 * c4 - s1 * c2 * s4 + c1 * s3 * c4)
        + Z * (s1 * s2 * c3 * s4 - s1 * c2 * c4 - c1 * s3 * s4)
    )
    Tz4 = X * (-s2 * s4 + c2 * c3 * c4) + Z * (-s2 * c4 - c2 * c3 * s4)

    Dy5 = (
        -Cx * c5 * s6
        + Cy * (-s5 * c7 - c5 * c6 * s7)
        + Cz * (s5 * s7 - c5 * c6 * c7)
    )
    Dz5 = (
        -Cx * s5 * s6
        + Cy * (c5 * c7 - s5 * c6 * s7)
        + Cz * (-c5 * s7 - s5 * c6 * c7)
    )

    Dx6 = -Cx * s6 - Cy * c6 * s7 - Cz * c6 * c7
    Dy6 = -Cx * s5 * c6 + Cy * s5 * s6 * s7 + Cz * s5 * s6 * c7
    Dz6 = Cx * c5 * c6 - Cy * c5 * s6 * s7 - Cz * c5 * s6 * c7

    Dx7 = -Cy * s6 * c7 + Cz * s6 * s7
    Dy7 = Cy * (-s5 * c6 * c7 - c5 * s7) + Cz * (s5 * c6 * s7 - c5 * c7)
    Dz7 = Cy * (-s5 * s7 + c5 * c6 * c7) + Cz * (-s5 * c7 - c5 * c6 * s7)

    dHx = np.array(
        [
            Tx1,
            Tx2,
            Tx3,
            Tx4,
            beta_x * Dy5 + gamma_x * Dz5,
            alpha_x * Dx6 + beta_x * Dy6 + gamma_x * Dz6,
            alpha_x * Dx7 + beta_x * Dy7 + gamma_x * Dz7,
        ],
        dtype=float,
    )
    dHy = np.array(
        [
            Ty1,
            Ty2,
            Ty3,
            Ty4,
            beta_y * Dy5 + gamma_y * Dz5,
            alpha_y * Dx6 + beta_y * Dy6 + gamma_y * Dz6,
            alpha_y * Dx7 + beta_y * Dy7 + gamma_y * Dz7,
        ],
        dtype=float,
    )
    dHz = np.array(
        [
            Tz1,
            Tz2,
            Tz3,
            Tz4,
            beta_z * Dy5 + gamma_z * Dz5,
            alpha_z * Dx6 + beta_z * Dy6 + gamma_z * Dz6,
            alpha_z * Dx7 + beta_z * Dy7 + gamma_z * Dz7,
        ],
        dtype=float,
    )
    return GRAVITY_ACCEL * (ux * dHx + uy * dHy + uz * dHz) + bias


def closed_form_gravity_bias(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return dual-arm gravity compensation torques from the closed-form law."""
    tau = np.zeros(model.nv)
    arm_tau: dict[str, np.ndarray] = {}
    for arm in ARMS:
        q = np.array([data.qpos[i] for i in arm_qpos_indices(model, arm.prefix)])
        local_tau = gravity_formula_7r_local(
            q,
            gravity_direction=gravity_direction_for_arm_local(model, data, arm),
            bias=gravity_bias_for_arm(arm),
        )
        arm_tau[arm.prefix] = local_tau
        for joint_index, dof in enumerate(joint_dof_indices(model, arm.prefix)):
            tau[dof] = local_tau[joint_index]
    pitch_angle = float(data.qpos[waist_qpos_index(model)])
    # Pitch.cpp reads encoder load torque, whereas ArmMiddleware computes
    # motor feed-forward torque.  In ideal static equilibrium encoder load is
    # the opposite of motor compensation: enc_L=-ff_L, enc_R=-ff_R.  Hence
    # enc_L-enc_R = ff_R-ff_L for the deployed installation matrices.
    tau[waist_dof_index(model)] = (
        arm_tau["right"][0]
        - arm_tau["left"][0]
        + PITCH_MASS
        * GRAVITY_ACCEL
        * (PITCH_COM[0] * math.cos(pitch_angle) + PITCH_COM[2] * math.sin(pitch_angle))
    )
    return tau


def apply_motor_ctrl_from_dof_torque(model: mujoco.MjModel, data: mujoco.MjData, tau: np.ndarray) -> None:
    """Map motor-side compensation torque to MuJoCo actuator ctrl."""
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        dof = int(model.jnt_dofadr[joint_id])
        data.ctrl[actuator_id] = tau[dof]

def set_dual_qpos(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_left: np.ndarray,
    q_right: np.ndarray,
    waist_angle: float = WAIST_UPRIGHT_ANGLE,
) -> None:
    set_waist_angle(model, data, waist_angle)
    for arm, q in zip(ARMS, (q_left, q_right)):
        for i, qi in enumerate(q, start=1):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{arm.prefix}_q{i}")
            qadr = int(model.jnt_qposadr[joint_id])
            data.qpos[qadr] = qi
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def damped_least_squares_delta(jacobian: np.ndarray, task_delta: np.ndarray) -> np.ndarray:
    lhs = jacobian @ jacobian.T + (IK_DAMPING**2) * np.eye(jacobian.shape[0])
    return jacobian.T @ np.linalg.solve(lhs, task_delta)


def apply_tool_delta(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    prefix: str,
    translation_delta: np.ndarray,
    rotation_delta: np.ndarray,
) -> None:
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, tool_site_name(prefix))
    dofs = joint_dof_indices(model, prefix)
    qpos_indices = arm_qpos_indices(model, prefix)

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    jacobian = np.vstack((jacp[:, dofs], jacr[:, dofs]))
    task_delta = np.concatenate((translation_delta, rotation_delta))
    dq = damped_least_squares_delta(jacobian, task_delta)
    dq = np.clip(dq, -IK_MAX_Q_STEP, IK_MAX_Q_STEP)

    data.qpos[qpos_indices] += dq
    data.qvel[dofs] = 0.0
    data.qacc[dofs] = 0.0
    mujoco.mj_forward(model, data)


def apply_dual_tool_delta(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    translation_delta: np.ndarray,
    rotation_delta: np.ndarray,
) -> None:
    for arm in ARMS:
        apply_tool_delta(model, data, arm.prefix, translation_delta, rotation_delta)


def keyboard_command_delta(key: str, rotate_mode: bool) -> tuple[np.ndarray, np.ndarray] | None:
    translation = np.zeros(3)
    rotation = np.zeros(3)

    if rotate_mode:
        # Pitch: W/S about world Y, yaw: A/D about world Z, roll: Q/Z about world X.
        mapping = {
            "w": np.array([0.0, ROTATION_STEP, 0.0]),
            "s": np.array([0.0, -ROTATION_STEP, 0.0]),
            "a": np.array([0.0, 0.0, ROTATION_STEP]),
            "d": np.array([0.0, 0.0, -ROTATION_STEP]),
            "q": np.array([ROTATION_STEP, 0.0, 0.0]),
            "z": np.array([-ROTATION_STEP, 0.0, 0.0]),
        }
        if key not in mapping:
            return None
        rotation = mapping[key]
    else:
        # World-frame translation: W/S forward/back, A/D left/right, Q/Z up/down.
        mapping = {
            "w": np.array([TRANSLATION_STEP, 0.0, 0.0]),
            "s": np.array([-TRANSLATION_STEP, 0.0, 0.0]),
            "a": np.array([0.0, TRANSLATION_STEP, 0.0]),
            "d": np.array([0.0, -TRANSLATION_STEP, 0.0]),
            "q": np.array([0.0, 0.0, TRANSLATION_STEP]),
            "z": np.array([0.0, 0.0, -TRANSLATION_STEP]),
        }
        if key not in mapping:
            return None
        translation = mapping[key]

    return translation, rotation


def run_check() -> None:
    xml = build_mjcf()
    tmp_path = Path(tempfile.gettempdir()) / "deploy_dual_7r_waist_link_no_plane.xml"
    tmp_path.write_text(xml, encoding="utf-8")
    print(f"Wrote MJCF to: {tmp_path}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    max_abs_error = 0.0
    arm_dofs = all_arm_dof_indices(model)
    waist_angles = [
        WAIST_UPRIGHT_ANGLE,
        math.radians(60.0),
        math.radians(30.0),
        WAIST_LYING_ANGLE,
    ]
    case_index = 0
    for waist_angle in waist_angles:
        for q_left, q_right in gravity_test_poses():
            case_index += 1
            set_dual_qpos(model, data, q_left, q_right, waist_angle)
            tau_mujoco = np.array(data.qfrc_bias)
            # Report/compare actuator-side motor torque. Pitch has gear=-1.
            tau_mujoco[waist_dof_index(model)] *= -1.0
            tau_formula = closed_form_gravity_bias(model, data)
            checked_dofs = [waist_dof_index(model), *arm_dofs]
            error = tau_formula[checked_dofs] - tau_mujoco[checked_dofs]
            max_abs_error = max(max_abs_error, float(np.max(np.abs(error))))

            print(f"\ncase {case_index:02d}  waist={math.degrees(waist_angle):.1f} deg")
            print("  tau_mujoco  =", np.array2string(tau_mujoco, precision=6, suppress_small=True))
            print("  tau_formula =", np.array2string(tau_formula, precision=6, suppress_small=True))
            print("  max|pitch/arm error| =", np.max(np.abs(error)))

    print("\nsummary")
    print(f"  nv = {model.nv}, total_mass = {np.sum(model.body_mass):.6f} kg")
    print(f"  max absolute pitch/arm torque error = {max_abs_error:.6e} N*m")
    if max_abs_error < 1.0e-8:
        print("  OK: pitch and 7R gravity compensation match MuJoCo qfrc_bias across pitch angles.")
    else:
        print("  WARNING: sign/frame/model mismatch remains; inspect the largest-error case.")


def run_visualizer(initial_pose: int = 1) -> None:
    import mujoco.viewer
    import glfw

    xml = build_mjcf()
    tmp_path = Path(tempfile.gettempdir()) / "deploy_dual_7r_waist_link_no_plane.xml"
    tmp_path.write_text(xml, encoding="utf-8")
    print(f"Wrote MJCF to: {tmp_path}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    poses = gravity_test_poses()
    state = {
        "gravity_comp": True,
        "pose_index": max(0, min(initial_pose - 1, len(poses) - 1)),
        "reset_pose": True,
        "control_mode": "translate",
        "waist_angle": WAIST_UPRIGHT_ANGLE,
    }
    command_queue = deque()

    def apply_pose() -> None:
        q_left, q_right = poses[state["pose_index"]]
        set_dual_qpos(model, data, q_left, q_right, state["waist_angle"])
        print(
            f"Loaded pose {state['pose_index'] + 1:02d}/{len(poses)}, "
            f"waist={math.degrees(state['waist_angle']):.1f} deg"
        )

    def shift_is_pressed() -> bool:
        try:
            window = glfw.get_current_context()
            if window is None:
                return False
            return (
                glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
            )
        except glfw.GLFWError:
            return False

    def key_callback(keycode: int) -> None:
        key = chr(keycode).lower() if 0 <= keycode < 256 else ""
        if key == "g":
            state["gravity_comp"] = not state["gravity_comp"]
            mode = "ON" if state["gravity_comp"] else "OFF"
            print(f"Gravity compensation: {mode}")
        elif key == "n":
            state["pose_index"] = (state["pose_index"] + 1) % len(poses)
            state["reset_pose"] = True
        elif key == "r":
            state["reset_pose"] = True
        elif key == "x":
            state["control_mode"] = (
                "rotate" if state["control_mode"] == "translate" else "translate"
            )
            print(f"Control mode: {state['control_mode']}")
        elif key in {"e", "c"} and shift_is_pressed():
            direction = -1.0 if key == "e" else 1.0
            state["waist_angle"] = clamp_waist_angle(
                state["waist_angle"] + direction * WAIST_STEP
            )
            set_waist_angle(model, data, state["waist_angle"])
            print(f"Waist pitch: {math.degrees(state['waist_angle']):.1f} deg")
        elif key in {"w", "s", "a", "d", "q", "z"}:
            rotate_mode = shift_is_pressed() or state["control_mode"] == "rotate"
            command = keyboard_command_delta(key, rotate_mode)
            if command is not None:
                command_queue.append(command)

    print("\nViewer controls")
    print("  G: toggle gravity compensation torque")
    print("  N: load next test pose")
    print("  R: reset current pose")
    print("  W/S: move both tool tips forward/back along world X")
    print("  A/D: move both tool tips left/right along world Y")
    print("  Q/Z: move both tool tips up/down along world Z")
    print("  Shift+W/S: pitch both tool tips")
    print("  Shift+A/D: yaw both tool tips")
    print("  Shift+Q/Z: roll both tool tips")
    print("  Shift+E/C: stand up / lie down the common waist pitch")
    print("  X: toggle translation/rotation mode if Shift is not detected")
    print("  ESC / close window: quit")
    print("\nGravity compensation starts ON. Toggle it OFF to watch the arms fall.")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = np.array([0.1, 0.0, 0.12])
        viewer.cam.distance = 0.85
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25

        while viewer.is_running():
            step_start = time.time()

            if state["reset_pose"]:
                apply_pose()
                state["reset_pose"] = False

            while command_queue:
                translation_delta, rotation_delta = command_queue.popleft()
                apply_dual_tool_delta(model, data, translation_delta, rotation_delta)

            set_waist_angle(model, data, state["waist_angle"])

            # Update kinematics at the current qpos, then compute gravity
            # compensation with the closed-form 7R formula. MuJoCo qfrc_bias
            # is used only by --check as an external reference.
            mujoco.mj_forward(model, data)
            if state["gravity_comp"]:
                # Motor gear is 1, so each motor ctrl is the joint torque.
                # Arm torques use g * u^T * dS/dq + b; pitch uses the
                # Pitch.cpp law: left J0 - right J0 + equivalent-COM torque.
                apply_motor_ctrl_from_dof_torque(
                    model, data, closed_form_gravity_bias(model, data)
                )
            else:
                # With compensation disabled, the arms are driven only by passive dynamics and gravity.
                data.ctrl[:] = 0.0

            # Step forward using the torque currently stored in data.ctrl.
            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MuJoCo dual-arm gravity compensation check and visualizer."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="run the numeric gravity-compensation check instead of opening the viewer",
    )
    parser.add_argument(
        "--visualize",
        "--view",
        action="store_true",
        help="open the interactive MuJoCo viewer, kept for compatibility",
    )
    parser.add_argument(
        "--pose",
        type=int,
        default=1,
        help="initial visualizer pose index, from 1 to 13",
    )
    args = parser.parse_args()

    if args.check:
        run_check()
    else:
        run_visualizer(args.pose)


if __name__ == "__main__":
    main()
