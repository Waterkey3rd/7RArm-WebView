#!/usr/bin/env python3
"""Interactive 3-D MuJoCo dual-arm viewer.

Uses the MuJoCo geometry from doubleArm7Rpro_TV.py and the exact deployed
AdaptiveHybrid IK implementation from Arm/Lib/myikine7R.
Run with:  python mujoco_dual_arm_visualizer.py
"""
from __future__ import annotations

import math
import json
import time
import copy
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path
import tempfile

import numpy as np
import mujoco
import mujoco.viewer

from doubleArm7Rpro_TV import (
    DEPLOY_SCENE_ORIGIN_OFFSET_M, WAIST_UPRIGHT_ANGLE, build_mjcf,
    set_dual_qpos, tool_site_name,
)
from deploy_ik import DeployIK, validate_joint_target
from trajectory_dialogs import FunctionTrajectoryDialog, HistoryPointDialog
from action_sequence_format import (
    FORMAT as ACTION_SEQUENCE_FORMAT,
    cartesian_frame_target,
    history_to_sequence,
    joint_frame_target,
    sequence_to_history,
)


INITIAL = {
    # Zero logical-joint pose for both 7R arms. Motor encoder signs/offsets are
    # applied later by firmware SpaceMapper and do not belong in this model.
    "left": np.zeros(7, dtype=float),
    "right": np.zeros(7, dtype=float),
}


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler rotation, matching the displayed Roll/Pitch/Yaw convention."""
    def rot(axis, angle):
        c, s = math.cos(angle), math.sin(angle)
        if axis == "x": return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        if axis == "y": return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return rot("z", yaw) @ rot("y", pitch) @ rot("x", roll)


def matrix_to_yaw_pitch_roll(R: np.ndarray) -> tuple[float, float, float]:
    """Return ZYX yaw/pitch/roll in radians."""
    sy = math.hypot(float(R[0, 0]), float(R[1, 0]))
    if sy > 1e-9:
        return (math.atan2(float(R[1, 0]), float(R[0, 0])),
                math.atan2(float(-R[2, 0]), sy),
                math.atan2(float(R[2, 1]), float(R[2, 2])))
    return (math.atan2(float(-R[0, 1]), float(R[1, 1])),
            math.atan2(float(-R[2, 0]), sy), 0.0)


class App:
    def __init__(self, root: tk.Tk, viewer, model, data, ik: DeployIK):
        self.root, self.viewer, self.model, self.data, self.ik = root, viewer, model, data, ik
        self.current = {k: v.copy() for k, v in INITIAL.items()}
        self.target = {k: v.copy() for k, v in INITIAL.items()}
        self.start = {k: v.copy() for k, v in INITIAL.items()}
        self.motion_t0 = 0.0; self.motion_duration = 2.0; self.moving = False
        self.history: list[dict] = []
        self.play_queue: list[tuple[dict[str, np.ndarray], int]] = []
        self._build_ui()
        set_dual_qpos(model, data, self.current["left"], self.current["right"], WAIST_UPRIGHT_ANGLE)
        self._sync_all_fields()
        self.root.after(16, self._tick)

    def _build_ui(self):
        self.root.title("双臂 RoboArm · MuJoCo 3D 控制")
        self.root.geometry("560x960"); self.root.minsize(520, 860); self.root.configure(bg="#101827")
        # Keep the numeric control panel above the native MuJoCo window so it
        # remains editable while the 3-D viewer is being manipulated.
        try: self.root.attributes("-topmost", True)
        except tk.TclError: pass
        style = ttk.Style(); style.theme_use("clam")
        style.configure("TNotebook", background="#101827", borderwidth=0)
        style.configure("TFrame", background="#101827"); style.configure("TLabel", background="#101827", foreground="#e9f1ff")
        style.configure("TLabelframe", background="#101827", foreground="#b9c9e8"); style.configure("TLabelframe.Label", background="#101827", foreground="#b9c9e8")
        tk.Label(self.root, text="双臂 RoboArm", bg="#101827", fg="#eff5ff", font=("Segoe UI", 18, "bold")).pack(anchor="w", padx=18, pady=(15, 0))
        tk.Label(self.root, text="MuJoCo 原生三维可视化 · 目标位姿与运动历史", bg="#101827", fg="#91a4c6").pack(anchor="w", padx=19, pady=(0, 12))
        self.notebook = ttk.Notebook(self.root); self.notebook.pack(fill="x", padx=14)
        self.joint_frame = ttk.Frame(self.notebook); self.cart_frame = ttk.Frame(self.notebook); self.delta_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.joint_frame, text="关节角 (deg)"); self.notebook.add(self.cart_frame, text="笛卡尔坐标 (mm)"); self.notebook.add(self.delta_frame, text="Delta (mm)")
        self.joint_entries = {}; self.cart_entries = {}; self.delta_entries = {}
        for side, title, color in (("left", "左臂", "#5ca7ff"), ("right", "右臂", "#ff9c62")):
            box = ttk.LabelFrame(self.joint_frame, text=title); box.pack(fill="x", padx=8, pady=7)
            self.joint_entries[side] = []
            for i in range(7):
                tk.Label(box, text=f"J{i+1}", bg="#101827", fg="#91a4c6", width=4).grid(row=i//4, column=(i%4)*2, padx=(5,0), pady=5)
                e = tk.Entry(box, width=8, bg="#18243b", fg="white", insertbackground="white", relief="flat")
                e.insert(0, f"{math.degrees(INITIAL[side][i]):.1f}"); e.grid(row=i//4, column=(i%4)*2+1, padx=(0,8), pady=5); self.joint_entries[side].append(e)
            cbox = ttk.LabelFrame(self.cart_frame, text=title); cbox.pack(fill="x", padx=8, pady=7); self.cart_entries[side] = []
            labels = ("X", "Y", "Z", "Yaw", "Pitch", "Roll")
            for i, lab in enumerate(labels):
                tk.Label(cbox, text=lab, bg="#101827", fg="#91a4c6", width=6).grid(row=i//3, column=(i%3)*2, padx=(5,0), pady=5)
                e=tk.Entry(cbox,width=8,bg="#18243b",fg="white",insertbackground="white",relief="flat");e.insert(0,"0.0");e.grid(row=i//3,column=(i%3)*2+1,padx=(0,8),pady=5);self.cart_entries[side].append(e)
            dbox = ttk.LabelFrame(self.delta_frame, text=title); dbox.pack(fill="x", padx=8, pady=7); self.delta_entries[side] = []
            for i, lab in enumerate(("ΔX", "ΔY", "ΔZ", "ΔYaw", "ΔPitch", "ΔRoll")):
                tk.Label(dbox, text=lab, bg="#101827", fg="#91a4c6", width=6).grid(row=i//3, column=(i%3)*2, padx=(5,0), pady=5)
                e=tk.Entry(dbox,width=8,bg="#18243b",fg="white",insertbackground="white",relief="flat");e.insert(0,"0.0");e.grid(row=i//3,column=(i%3)*2+1,padx=(0,8),pady=5);self.delta_entries[side].append(e)
            tk.Label(dbox, text="位置单位 mm，角度单位 deg；相对当前末端", bg="#101827", fg="#7184a9").grid(row=2, column=0, columnspan=6, sticky="w", padx=6, pady=(1,5))
        bar = tk.Frame(self.root, bg="#101827"); bar.pack(fill="x", padx=22, pady=9)
        tk.Label(bar, text="运动时长 (s)", bg="#101827", fg="#91a4c6").pack(side="left")
        self.duration = tk.DoubleVar(value=2.0); tk.Scale(bar, from_=0.3, to=8, resolution=.1, orient="horizontal", variable=self.duration, bg="#101827", fg="white", highlightthickness=0, troughcolor="#263958", length=170).pack(side="left")
        tk.Button(bar, text="执行运动", command=self.execute, bg="#396fda", fg="white", relief="flat", padx=12).pack(side="right")
        tk.Button(self.root, text="↶ 回退上一状态", command=self.undo, bg="#1d2c47", fg="white", relief="flat").pack(fill="x", padx=22, pady=2)
        tk.Button(self.root, text="重置初始姿态", command=self.reset, bg="#1d2c47", fg="white", relief="flat").pack(fill="x", padx=22, pady=2)
        function_bar = tk.Frame(self.root, bg="#101827"); function_bar.pack(fill="x", padx=22, pady=(5, 2))
        tk.Button(function_bar, text="ƒ(t) 添加函数轨迹", command=self.add_function_trajectory, bg="#355f90", fg="white", relief="flat").pack(side="left", fill="x", expand=True, padx=(0, 3))
        tk.Button(function_bar, text="编辑选中历史点 (F2)", command=self.edit_selected_history, bg="#355f90", fg="white", relief="flat").pack(side="left", fill="x", expand=True, padx=(3, 0))
        history_box = tk.Frame(self.root, bg="#101827"); history_box.pack(fill="x", expand=False, padx=22, pady=(8, 4))
        history_scroll = tk.Scrollbar(history_box, orient="vertical")
        self.history_list = tk.Listbox(history_box, height=9, selectmode="extended", bg="#0c1424", fg="#dce8ff", selectbackground="#345d9d", relief="flat", yscrollcommand=history_scroll.set)
        history_scroll.configure(command=self.history_list.yview); history_scroll.pack(side="right", fill="y"); self.history_list.pack(side="left", fill="both", expand=True); self.history_list.bind("<Double-Button-1>", self.restore_selected)
        tk.Button(self.root, text="▶ 播放选中历史段（可 Shift 多选）", command=self.play_selected, bg="#284d83", fg="white", relief="flat").pack(fill="x", padx=22, pady=2)
        io_bar = tk.Frame(self.root, bg="#101827"); io_bar.pack(fill="x", padx=22, pady=2)
        tk.Button(io_bar, text="导出动作序列 JSON", command=self.export_history, bg="#1d2c47", fg="white", relief="flat").pack(side="left", fill="x", expand=True, padx=(0, 3))
        tk.Button(io_bar, text="导入动作序列 JSON", command=self.import_history, bg="#1d2c47", fg="white", relief="flat").pack(side="left", fill="x", expand=True, padx=(3, 0))
        export_mode_bar = tk.Frame(self.root, bg="#101827"); export_mode_bar.pack(fill="x", padx=22, pady=(4, 1))
        tk.Label(export_mode_bar, text="导出目标空间", bg="#101827", fg="#91a4c6").pack(side="left")
        self.export_space = tk.StringVar(value="保留记录空间（混合）")
        ttk.Combobox(export_mode_bar, textvariable=self.export_space, state="readonly",
                     values=("保留记录空间（混合）", "全部 JointAngleSpace"), width=25).pack(side="right")
        tk.Label(self.root, text="双击历史记录可定位 · F2 可修改关键点 · MuJoCo 窗口支持鼠标旋转/缩放", bg="#101827", fg="#7184a9").pack(anchor="w", padx=22, pady=(0,10))
        self.root.bind("<Control-Return>", lambda _e: self.execute())
        self.root.bind("<Control-z>", lambda _e: self.undo())
        self.root.bind("<F2>", lambda _e: self.edit_selected_history())

    def _read_joint_target(self):
        out = {}
        try:
            for side in ("left", "right"): out[side] = np.array([float(e.get()) for e in self.joint_entries[side]], dtype=float) * math.pi/180
        except ValueError: raise ValueError("关节角必须是数字")
        return out

    def _snapshot_values(self, snapshot: dict[str, np.ndarray]) -> dict[str, dict[str, list[float]]]:
        """Return editable logical-joint and deployment Cartesian values."""
        set_dual_qpos(
            self.model, self.data, snapshot["left"], snapshot["right"],
            WAIST_UPRIGHT_ANGLE,
        )
        result: dict[str, dict[str, list[float]]] = {}
        for side in ("left", "right"):
            site = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, tool_site_name(side)
            )
            position_mm = (
                self.data.site_xpos[site] - DEPLOY_SCENE_ORIGIN_OFFSET_M
            ) * 1000.0
            yaw, pitch, roll = matrix_to_yaw_pitch_roll(
                self.data.site_xmat[site].reshape(3, 3)
            )
            result[side] = {
                "JointAngleSpace": [math.degrees(value) for value in snapshot[side]],
                "CartesianSpace": [
                    *position_mm.tolist(), math.degrees(yaw),
                    math.degrees(pitch), math.degrees(roll),
                ],
            }
        set_dual_qpos(
            self.model, self.data, self.current["left"], self.current["right"],
            WAIST_UPRIGHT_ANGLE,
        )
        return result

    @staticmethod
    def _frame_target_from_values(space: str, values: list[float]) -> dict:
        if space == "JointAngleSpace":
            return joint_frame_target(np.deg2rad(np.asarray(values, dtype=float)))
        if space == "CartesianSpace":
            if len(values) != 6:
                raise ValueError("坐标位姿必须包含 X/Y/Z/Yaw/Pitch/Roll")
            return cartesian_frame_target(
                values[0], values[1], values[2],
                math.radians(values[3]), math.radians(values[4]),
                math.radians(values[5]),
            )
        raise ValueError(f"不支持的目标空间: {space}")

    def _solve_frame_targets(
        self, frame_targets: dict[str, dict], current: dict[str, np.ndarray]
    ) -> dict[str, np.ndarray]:
        solved: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            frame = frame_targets[side]
            if frame["space"] == "JointAngleSpace":
                solved[side] = validate_joint_target(
                    side, np.asarray(frame["target"]["jointAngles"], dtype=float)
                )
            elif frame["space"] == "CartesianSpace":
                target = frame["target"]
                position = np.array(
                    [target["x"], target["y"], target["z"]], dtype=float
                ) / 1000.0
                rotation = rpy_matrix(
                    target["roll"], target["pitch"], target["yaw"]
                )
                solved[side] = self.ik.solve(
                    side, rotation, position, current[side]
                ).q
            else:
                raise ValueError(f"{side} 使用了未知目标空间")
        return solved

    def _refresh_history_list(self) -> None:
        self.history_list.delete(0, "end")
        for number, item in enumerate(self.history, 1):
            self._append_history_label(item, number)

    @staticmethod
    def _split_duration(total_ms: int, count: int) -> list[int]:
        edges = [round(index * total_ms / count) for index in range(count + 1)]
        durations = [edges[index + 1] - edges[index] for index in range(count)]
        if any(duration <= 0 for duration in durations):
            raise ValueError("总时长过短，关键点间隔小于 1 ms")
        return durations

    def _sync_all_fields(self):
        for side in ("left", "right"):
            for entry, value in zip(self.joint_entries[side], self.current[side]):
                entry.delete(0, "end"); entry.insert(0, f"{math.degrees(value):.1f}")
            site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, tool_site_name(side))
            # The rendered pitch structure places both J0 axes at a common
            # +X/+Z offset. Firmware CartesianSpace deliberately omits it.
            xyz_mm = (
                self.data.site_xpos[site] - DEPLOY_SCENE_ORIGIN_OFFSET_M
            ) * 1000.0
            for entry, value in zip(self.cart_entries[side], xyz_mm):
                entry.delete(0, "end"); entry.insert(0, f"{value:.1f}")
            R = self.data.site_xmat[site].reshape(3, 3)
            pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
            roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
            yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
            for entry, value in zip(self.cart_entries[side][3:], (yaw, pitch, roll)):
                entry.delete(0, "end"); entry.insert(0, f"{math.degrees(value):.1f}")

    def execute(self):
        try:
            mode = self.notebook.index(self.notebook.select())
            if mode == 0: target = self._read_joint_target(); label = "关节目标"
            else:
                target = {"left": None, "right": None}; label = "笛卡尔目标"
                for side in ("left", "right"):
                    if mode == 1:
                        vals = [float(e.get()) for e in self.cart_entries[side]]
                        target[side] = (np.array(vals[:3], dtype=float) / 1000.0, rpy_matrix(vals[5]*math.pi/180, vals[4]*math.pi/180, vals[3]*math.pi/180))
                    else:
                        vals = [float(e.get()) for e in self.delta_entries[side]]
                        delta = np.array(vals[:3], dtype=float) / 1000.0
                        site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, tool_site_name(side))
                        current_p = (
                            self.data.site_xpos[site].copy()
                            - DEPLOY_SCENE_ORIGIN_OFFSET_M
                        )
                        current_R = self.data.site_xmat[site].reshape(3, 3).copy()
                        target[side] = (current_p + delta, current_R @ rpy_matrix(vals[5]*math.pi/180, vals[4]*math.pi/180, vals[3]*math.pi/180))
                        label = "Delta 位姿移动"
            start_snapshot = {k: v.copy() for k, v in self.current.items()}
            solved = {}
            frame_targets = {}
            for side in ("left", "right"):
                if mode == 0:
                    solved[side] = validate_joint_target(side, target[side])
                    frame_targets[side] = joint_frame_target(solved[side])
                else:
                    solved[side] = self.ik.solve(
                        side, target[side][1], target[side][0], self.current[side]
                    ).q
                    yaw, pitch, roll = matrix_to_yaw_pitch_roll(target[side][1])
                    position_mm = target[side][0] * 1000.0
                    frame_targets[side] = cartesian_frame_target(
                        position_mm[0], position_mm[1], position_mm[2], yaw, pitch, roll)
            self._begin_motion(solved)
            entry = {
                "start": start_snapshot,
                "target": {k: v.copy() for k, v in solved.items()},
                "frame_targets": frame_targets,
                "label": label,
                "time": time.strftime("%H:%M:%S"),
                "duration_ms": int(round(max(.1, float(self.duration.get())) * 1000.0)),
                "timeout_ms": 0,
            }
            self.history.append(entry); self._append_history_label(entry)
        except ValueError as exc: messagebox.showerror("输入错误", str(exc))

    def add_function_trajectory(self) -> None:
        self.moving = False
        self.play_queue.clear()
        initial_values = self._snapshot_values(self.current)
        dialog = FunctionTrajectoryDialog(self.root, initial_values)
        if dialog.result is None:
            return
        spec = dialog.result
        try:
            start_values = {
                side: [formula.evaluate(spec["t_start"]) for formula in spec["compiled"][side]]
                for side in ("left", "right")
            }
            mismatches: list[str] = []
            for side in ("left", "right"):
                expected = np.asarray(initial_values[side][spec["space"]], dtype=float)
                actual = np.asarray(start_values[side], dtype=float)
                if spec["space"] == "JointAngleSpace":
                    if float(np.max(np.abs(actual - expected))) > 0.5:
                        mismatches.append(f"{side} 最大关节差 {np.max(np.abs(actual - expected)):.2f}°")
                else:
                    position_error = float(np.max(np.abs(actual[:3] - expected[:3])))
                    angle_delta = (actual[3:] - expected[3:] + 180.0) % 360.0 - 180.0
                    angle_error = float(np.max(np.abs(angle_delta)))
                    if position_error > 1.0 or angle_error > 1.0:
                        mismatches.append(
                            f"{side} 最大差 {position_error:.2f} mm / {angle_error:.2f}°"
                        )
            if mismatches and not messagebox.askyesno(
                "t 起点与当前状态不同",
                "公式在 t 起点没有接上当前状态：\n" + "\n".join(mismatches)
                + "\n\n继续时会从当前状态直接插值到第一个关键点，是否继续？",
                parent=self.root,
            ):
                return

            count = spec["keypoint_count"]
            durations = self._split_duration(spec["duration_ms"], count)
            t_values = np.linspace(spec["t_start"], spec["t_end"], count + 1)[1:]
            current = {side: value.copy() for side, value in self.current.items()}
            generated: list[dict] = []
            group_number = 1 + sum(
                1 for item in self.history
                if item.get("function", {}).get("point_index") == 1
            )
            stamp = time.strftime("%H:%M:%S")
            for point_index, (t_value, duration_ms) in enumerate(
                zip(t_values, durations), 1
            ):
                values = {
                    side: [
                        formula.evaluate(float(t_value))
                        for formula in spec["compiled"][side]
                    ]
                    for side in ("left", "right")
                }
                frame_targets = {
                    side: self._frame_target_from_values(spec["space"], values[side])
                    for side in ("left", "right")
                }
                solved = self._solve_frame_targets(frame_targets, current)
                entry = {
                    "start": {side: value.copy() for side, value in current.items()},
                    "target": {side: value.copy() for side, value in solved.items()},
                    "frame_targets": frame_targets,
                    "label": (
                        f"函数轨迹 {group_number} · {point_index}/{count} · "
                        f"t={float(t_value):.5g}"
                    ),
                    "time": stamp,
                    "duration_ms": duration_ms,
                    "timeout_ms": 0,
                    "function": {
                        "group": group_number,
                        "point_index": point_index,
                        "point_count": count,
                        "t": float(t_value),
                        "t_start": spec["t_start"],
                        "t_end": spec["t_end"],
                        "space": spec["space"],
                        "sources": copy.deepcopy(spec["sources"]),
                    },
                }
                generated.append(entry)
                current = solved
            self.history.extend(generated)
            self._refresh_history_list()
            self.history_list.see("end")
            self.play_queue = [
                (item["target"], int(item["duration_ms"])) for item in generated[1:]
            ]
            self._begin_motion(generated[0]["target"], generated[0]["duration_ms"])
        except (KeyError, TypeError, ValueError) as exc:
            messagebox.showerror("函数轨迹生成失败", str(exc), parent=self.root)

    def _recompute_history_from(self, index: int) -> None:
        if not 0 <= index < len(self.history):
            return
        current = (
            {side: value.copy() for side, value in self.history[index]["start"].items()}
            if index == 0
            else {side: value.copy() for side, value in self.history[index - 1]["target"].items()}
        )
        for item in self.history[index:]:
            item["start"] = {side: value.copy() for side, value in current.items()}
            solved = self._solve_frame_targets(item["frame_targets"], current)
            item["target"] = {side: value.copy() for side, value in solved.items()}
            current = solved

    def edit_selected_history(self) -> None:
        selection = self.history_list.curselection()
        if not selection:
            messagebox.showinfo("编辑历史点", "请先选择一个历史关键点", parent=self.root)
            return
        index = int(selection[0])
        if index >= len(self.history):
            return
        self.moving = False
        self.play_queue.clear()
        alternatives = self._snapshot_values(self.history[index]["target"])
        dialog = HistoryPointDialog(self.root, self.history[index], alternatives)
        if dialog.result is None:
            return
        backup = copy.deepcopy(self.history)
        try:
            result = dialog.result
            item = self.history[index]
            item["label"] = result["label"]
            item["duration_ms"] = result["duration_ms"]
            item["frame_targets"] = {
                side: self._frame_target_from_values(
                    result["spaces"][side], result["values"][side]
                )
                for side in ("left", "right")
            }
            item.pop("function", None)
            self._recompute_history_from(index)
            self._refresh_history_list()
            self.history_list.selection_set(index)
            self.history_list.see(index)
            self._begin_motion(self.history[index]["target"], item["duration_ms"])
        except (KeyError, TypeError, ValueError) as exc:
            self.history = backup
            messagebox.showerror(
                "历史点修改失败",
                f"修改会导致当前或后续关键点无有效解，已恢复原数据。\n\n{exc}",
                parent=self.root,
            )

    def _q_text(self, snapshot):
        return "L:" + ",".join(f"{math.degrees(x):.0f}" for x in snapshot["left"]) + "  R:" + ",".join(f"{math.degrees(x):.0f}" for x in snapshot["right"])
    def _append_history_label(self, entry, number=None):
        number = len(self.history) if number is None else number
        duration_ms = int(entry.get("duration_ms", 2000))
        self.history_list.insert("end", f"{number:02d} {entry['label']} {entry['time']}  {duration_ms} ms\n  {self._q_text(entry['start'])}  ->  {self._q_text(entry['target'])}")

    def export_history(self):
        if not self.history:
            messagebox.showinfo("导出历史", "当前没有可导出的历史记录"); return
        filename = filedialog.asksaveasfilename(title="导出 PerformanceAction 动作序列", defaultextension=".json", filetypes=(("JSON 文件", "*.json"), ("所有文件", "*.*")))
        if not filename: return
        try:
            force_joint = self.export_space.get() == "全部 JointAngleSpace"
            payload = history_to_sequence(
                self.history, int(round(float(self.duration.get()) * 1000.0)),
                force_joint_space=force_joint)
            with open(filename, "w", encoding="utf-8") as f: json.dump(payload, f, ensure_ascii=False, indent=2)
            space_text = "全部 JointAngleSpace" if force_joint else "保留 Joint/Cartesian 记录类型"
            messagebox.showinfo("导出动作序列", f"已导出 {len(self.history) + 1} 帧\n{space_text}\n位置: mm，角度: rad，时间: ms")
        except (OSError, KeyError, TypeError, ValueError) as exc: messagebox.showerror("导出失败", str(exc))

    def import_history(self):
        filename = filedialog.askopenfilename(title="导入 PerformanceAction 动作序列", filetypes=(("JSON 文件", "*.json"), ("所有文件", "*.*")))
        if not filename: return
        try:
            with open(filename, "r", encoding="utf-8") as f: payload = json.load(f)
            if not isinstance(payload, dict) or payload.get("format") != ACTION_SEQUENCE_FORMAT:
                raise ValueError(f"仅支持 {ACTION_SEQUENCE_FORMAT}；旧文件请先运行转换脚本")
            loaded = sequence_to_history(payload)
            if loaded:
                current = {k: np.asarray(v, dtype=float) for k, v in loaded[0]["start"].items()}
            else:
                current = {k: v.copy() for k, v in INITIAL.items()}
            for item in loaded:
                item["start"] = {k: v.copy() for k, v in current.items()}
                solved = {}
                for side in ("left", "right"):
                    frame = item["frame_targets"][side]
                    if frame["space"] == "JointAngleSpace":
                        solved[side] = validate_joint_target(
                            side, np.asarray(frame["target"]["jointAngles"], dtype=float)
                        )
                    else:
                        target = frame["target"]
                        position = np.array([target["x"], target["y"], target["z"]], dtype=float) / 1000.0
                        rotation = rpy_matrix(target["roll"], target["pitch"], target["yaw"])
                        solved[side] = self.ik.solve(
                            side, rotation, position, current[side]
                        ).q
                item["target"] = {k: v.copy() for k, v in solved.items()}
                current = solved
            self.history = loaded; self.play_queue.clear(); self.history_list.delete(0, "end")
            for number, item in enumerate(self.history, 1): self._append_history_label(item, number)
            messagebox.showinfo("导入历史", f"已导入 {len(self.history)} 条记录")
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc: messagebox.showerror("导入失败", str(exc))
    def undo(self):
        if self.history:
            self.history.pop(); self.history_list.delete("end"); self._begin_motion(self.history[-1]["target"] if self.history else INITIAL)
    def reset(self): self.history.clear(); self.history_list.delete(0, "end"); self.play_queue.clear(); self._begin_motion(INITIAL)
    def restore_selected(self, _event=None):
        idx = self.history_list.curselection()
        if idx and idx[0] < len(self.history): self._begin_motion(self.history[idx[0]]["target"])

    def play_selected(self):
        indices = self.history_list.curselection()
        if not indices: messagebox.showinfo("播放历史", "请先选择一条或连续多条历史记录"); return
        indices = sorted(indices)
        if indices != list(range(indices[0], indices[-1] + 1)):
            messagebox.showwarning("播放历史", "请选择连续的历史记录"); return
        first = self.history[indices[0]]
        # First move to the selected segment's start, then play every target.
        first_duration = int(first.get("duration_ms", 2000))
        self.play_queue = [(first["start"], first_duration)] + [
            (self.history[i]["target"], int(self.history[i].get("duration_ms", 2000)))
            for i in indices
        ]
        target, duration_ms = self.play_queue.pop(0)
        self._begin_motion(target, duration_ms)

    def _begin_motion(self, target, duration_ms=None):
        self.target = {k: np.asarray(v, dtype=float).copy() for k, v in target.items()}
        self.start = {k: v.copy() for k, v in self.current.items()}
        duration_s = float(self.duration.get()) if duration_ms is None else float(duration_ms) / 1000.0
        self.motion_t0 = time.perf_counter(); self.motion_duration = max(.1, duration_s); self.moving = True

    def _tick(self):
        if self.moving:
            t=min(1,(time.perf_counter()-self.motion_t0)/self.motion_duration); e=t*t*(3-2*t)
            q={k:self.start[k]+e*(self.target[k]-self.start[k]) for k in ("left","right")}; set_dual_qpos(self.model,self.data,q["left"],q["right"],WAIST_UPRIGHT_ANGLE); self.current=q
            if t>=1:
                if self.play_queue:
                    target, duration_ms = self.play_queue.pop(0)
                    self._begin_motion(target, duration_ms)
                else:
                    self.moving=False
                    self._sync_all_fields()
        if self.viewer.is_running(): self.viewer.sync(); self.root.after(16,self._tick)
        else: self.root.destroy()


def main():
    print("RoboArm MuJoCo visualizer (direct qpos update) starting...")
    xml=build_mjcf(); path=Path(tempfile.gettempdir())/"roboarm_dual_visualizer.xml"; path.write_text(xml,encoding="utf-8")
    model=mujoco.MjModel.from_xml_string(xml); data=mujoco.MjData(model); ik=DeployIK(); root=tk.Tk()
    with mujoco.viewer.launch_passive(model,data) as viewer:
        viewer.cam.lookat[:]=[.12,0,.18]; viewer.cam.distance=.9; viewer.cam.azimuth=135; viewer.cam.elevation=-25
        App(root,viewer,model,data,ik); root.mainloop()

if __name__ == "__main__": main()
