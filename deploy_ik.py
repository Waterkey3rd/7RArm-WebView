"""Python wrapper for the exact AdaptiveHybrid IK used by deployed Arm code."""
from __future__ import annotations

import ctypes
import json
import math
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "deploy_ik_bridge.cpp"
HEADERS = (
    ROOT / "deploy_arm_model.hpp",
    *tuple((ROOT / "Arm" / "Lib" / "myikine7R").glob("*.hpp")),
)
LIBRARY = Path(tempfile.gettempdir()) / (
    "roboarm_deploy_ik.dll" if sys.platform == "win32" else "libroboarm_deploy_ik.so"
)

JOINT_LIMITS_DEG = {
    "left": np.array([[-150, 150], [-179, 10], [-179, 179], [0, 200],
                      [-179, 179], [-120, 120], [-179, 269]], dtype=float),
    "right": np.array([[-150, 150], [-179, 10], [-179, 179], [0, 200],
                       [-179, 179], [-120, 120], [-269, 179]], dtype=float),
}
JOINT_LIMITS_RAD = {side: np.deg2rad(value) for side, value in JOINT_LIMITS_DEG.items()}


@dataclass(frozen=True)
class IkResult:
    q: np.ndarray
    position_error_mm: float
    orientation_error_rad: float
    route: int
    position_delta_mm: float
    orientation_delta_rad: float
    dls_iterations: int
    hybrid_stage: int


class DeployIK:
    def __init__(self) -> None:
        self._build_if_needed()
        self._library = ctypes.CDLL(str(LIBRARY))
        pointer = ctypes.POINTER(ctypes.c_double)
        self._library.deploy_ik_solve.argtypes = [
            ctypes.c_int, pointer, pointer, pointer, pointer, pointer,
        ]
        self._library.deploy_ik_solve.restype = ctypes.c_int
        self._library.deploy_ik_scalar_bytes.restype = ctypes.c_int
        if self._library.deploy_ik_scalar_bytes() != 4:
            raise RuntimeError("部署 IK bridge 未使用固件 float 标量类型")

    @staticmethod
    def _build_if_needed() -> None:
        newest_input = max(path.stat().st_mtime for path in (SOURCE, *HEADERS))
        if LIBRARY.exists() and LIBRARY.stat().st_mtime >= newest_input:
            return
        compiler = shutil.which("g++")
        if compiler is None:
            raise RuntimeError("未找到 g++，无法编译部署 IK bridge")
        command = [
            compiler, "-std=c++20", "-O3", "-shared", "-static-libgcc",
            "-static-libstdc++", str(SOURCE), "-o", str(LIBRARY),
        ]
        subprocess.run(command, cwd=ROOT, check=True)

    @staticmethod
    def _pointer(array: np.ndarray):
        return array.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    def solve(self, side: str, rotation: np.ndarray, position_m: np.ndarray,
              q_current: np.ndarray) -> IkResult:
        if side not in ("left", "right"):
            raise ValueError(f"未知机械臂: {side}")
        rotation = np.ascontiguousarray(rotation, dtype=np.float64).reshape(9)
        position_mm = np.ascontiguousarray(position_m, dtype=np.float64).reshape(3) * 1000.0
        q_current = np.ascontiguousarray(q_current, dtype=np.float64).reshape(7)
        q_out = np.empty(7, dtype=np.float64)
        diagnostics = np.zeros(8, dtype=np.float64)
        ok = self._library.deploy_ik_solve(
            0 if side == "left" else 1,
            self._pointer(rotation), self._pointer(position_mm),
            self._pointer(q_current), self._pointer(q_out),
            self._pointer(diagnostics),
        )
        if not ok or diagnostics[0] == 0.0:
            route = "DLS" if round(diagnostics[3]) == 1 else "Hybrid"
            raise ValueError(
                f"{side} IK 无有效解（{route}，初始变化 "
                f"{diagnostics[4]:.1f} mm / "
                f"{np.degrees(diagnostics[5]):.1f} deg）"
            )
        return IkResult(
            q=q_out,
            position_error_mm=float(diagnostics[1]),
            orientation_error_rad=float(diagnostics[2]),
            route=int(round(diagnostics[3])),
            position_delta_mm=float(diagnostics[4]),
            orientation_delta_rad=float(diagnostics[5]),
            dls_iterations=int(round(diagnostics[6])),
            hybrid_stage=int(round(diagnostics[7])),
        )


def validate_joint_target(side: str, q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(7)
    limits = JOINT_LIMITS_RAD[side]
    invalid = np.flatnonzero((q < limits[:, 0]) | (q > limits[:, 1]))
    if invalid.size:
        details = ", ".join(
            f"J{i + 1}={np.degrees(q[i]):.1f}° 不在 "
            f"[{JOINT_LIMITS_DEG[side][i, 0]:.1f}°, "
            f"{JOINT_LIMITS_DEG[side][i, 1]:.1f}°]"
            for i in invalid
        )
        raise ValueError(f"{side} 关节目标越界: {details}")
    return q.copy()


def self_check(sample_count: int = 100) -> None:
    """Cross-check MuJoCo FK against the DLL's deployed model at random poses."""
    import mujoco
    import doubleArm7Rpro_TV as plant

    model = mujoco.MjModel.from_xml_string(plant.build_mjcf())
    data = mujoco.MjData(model)
    ik = DeployIK()
    for side in ("left", "right"):
        q_range = np.array([
            model.jnt_range[
                mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_q{i}"
                )
            ]
            for i in range(1, 8)
        ])
        if not np.allclose(q_range, JOINT_LIMITS_RAD[side], atol=1.0e-12):
            raise RuntimeError(f"{side} MuJoCo joint limits differ from deployment")
    rng = np.random.default_rng(260904)
    max_position_delta_mm = 0.0
    max_orientation_delta_rad = 0.0
    max_joint_delta_rad = 0.0
    for side in ("left", "right"):
        limits = JOINT_LIMITS_RAD[side]
        for _ in range(sample_count):
            q = rng.uniform(limits[:, 0] + 0.02, limits[:, 1] - 0.02)
            q_left = q if side == "left" else np.zeros(7)
            q_right = q if side == "right" else np.zeros(7)
            plant.set_dual_qpos(model, data, q_left, q_right)
            site = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SITE, plant.tool_site_name(side)
            )
            position = (
                data.site_xpos[site] - plant.DEPLOY_SCENE_ORIGIN_OFFSET_M
            )
            rotation = data.site_xmat[site].reshape(3, 3).copy()
            result = ik.solve(side, rotation, position, q)
            max_position_delta_mm = max(
                max_position_delta_mm, result.position_delta_mm
            )
            max_orientation_delta_rad = max(
                max_orientation_delta_rad, result.orientation_delta_rad
            )
            max_joint_delta_rad = max(
                max_joint_delta_rad, float(np.max(np.abs(result.q - q)))
            )
    print(
        f"FK/IK alignment: {2 * sample_count} cases; max model mismatch "
        f"{max_position_delta_mm:.6g} mm / "
        f"{np.degrees(max_orientation_delta_rad):.6g} deg; max IK joint change "
        f"{np.degrees(max_joint_delta_rad):.6g} deg"
    )
    print(f"MuJoCo limited joints: {int(np.sum(model.jnt_limited))}/{model.njnt}")
    if max_position_delta_mm > 1.0e-3 or max_joint_delta_rad > 1.0e-5:
        raise RuntimeError("MuJoCo and deployed IK kinematic models are not aligned")


def check_action_sequence(path: Path) -> None:
    """Verify that every frame can be consumed with deployed limits and IK."""
    from action_sequence_format import sequence_to_history

    payload = json.loads(path.read_text(encoding="utf-8"))
    history = sequence_to_history(payload)
    ik = DeployIK()
    current = {"left": np.zeros(7), "right": np.zeros(7)}
    cartesian_count = 0
    for frame_index, item in enumerate(history, 1):
        solved: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            frame = item["frame_targets"][side]
            if frame["space"] == "JointAngleSpace":
                solved[side] = validate_joint_target(
                    side, np.asarray(frame["target"]["jointAngles"], dtype=float)
                )
                continue
            target = frame["target"]
            yaw, pitch, roll = target["yaw"], target["pitch"], target["roll"]
            cy, sy = math.cos(yaw), math.sin(yaw)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cr, sr = math.cos(roll), math.sin(roll)
            rotation = np.array([
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ])
            position = np.array([target["x"], target["y"], target["z"]]) / 1000.0
            try:
                solved[side] = ik.solve(
                    side, rotation, position, current[side]
                ).q
            except ValueError as exc:
                raise ValueError(f"frame {frame_index}, {side}: {exc}") from exc
            cartesian_count += 1
        current = solved
    print(
        f"Action sequence OK: {path} ({len(history)} transitions, "
        f"{cartesian_count} Cartesian arm frames)"
    )


if __name__ == "__main__":
    self_check()
    default_sequence = ROOT / "actionsequence.json"
    if default_sequence.exists():
        check_action_sequence(default_sequence)
