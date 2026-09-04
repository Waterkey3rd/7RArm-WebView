"""Tk dialogs for formula trajectories and editable history key points."""
from __future__ import annotations

import math
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

from latex_formula import Formula


SIDES = ("left", "right")
SIDE_TITLES = {"left": "左臂", "right": "右臂"}
SPACE_LABELS = {"关节角": "JointAngleSpace", "坐标位姿": "CartesianSpace"}
SPACE_NAMES = {value: key for key, value in SPACE_LABELS.items()}
JOINT_LABELS = tuple(f"J{i}" for i in range(1, 8))
CARTESIAN_LABELS = ("X", "Y", "Z", "Yaw", "Pitch", "Roll")


def _entry(parent, value: Any = "", width: int = 18) -> tk.Entry:
    widget = tk.Entry(
        parent, width=width, bg="#18243b", fg="white",
        insertbackground="white", relief="flat",
    )
    widget.insert(0, str(value))
    return widget


class FunctionTrajectoryDialog:
    def __init__(self, parent: tk.Misc, initial: dict[str, dict[str, list[float]]]):
        self.result: dict[str, Any] | None = None
        self.initial = initial
        self.window = tk.Toplevel(parent)
        self.window.title("添加关于 t 的函数轨迹")
        self.window.geometry("860x650")
        self.window.configure(bg="#101827")
        self.window.transient(parent)
        self.window.grab_set()

        settings = tk.Frame(self.window, bg="#101827")
        settings.pack(fill="x", padx=14, pady=10)
        self.space = tk.StringVar(value="关节角")
        self.t_start = tk.StringVar(value="0")
        self.t_end = tk.StringVar(value="1")
        self.duration = tk.StringVar(value="4.0")
        self.keypoints = tk.StringVar(value="20")
        for column, (label, variable, width) in enumerate((
            ("空间", self.space, 10), ("t 起点", self.t_start, 8),
            ("t 终点", self.t_end, 8), ("总时长(s)", self.duration, 8),
            ("关键点数", self.keypoints, 8),
        )):
            tk.Label(settings, text=label, bg="#101827", fg="#91a4c6").grid(
                row=0, column=2 * column, padx=(5, 2), pady=3
            )
            if label == "空间":
                widget = ttk.Combobox(
                    settings, textvariable=variable, state="readonly",
                    values=tuple(SPACE_LABELS), width=width,
                )
                widget.bind("<<ComboboxSelected>>", lambda _event: self._rebuild())
            else:
                widget = _entry(settings, variable.get(), width)
                widget.configure(textvariable=variable)
            widget.grid(row=0, column=2 * column + 1, padx=(0, 8), pady=3)

        tk.Label(
            self.window,
            text=(r"LaTeX 示例：30\sin(2\pi t)、\frac{1}{2}t^2。"
                  "关节/Yaw/Pitch/Roll 单位 deg，位置单位 mm。"
                  "当前状态视为 t 起点，导出仅包含下列采样关键点。"),
            bg="#101827", fg="#91a4c6", justify="left",
        ).pack(fill="x", padx=20, pady=(0, 8))

        self.formula_host = tk.Frame(self.window, bg="#101827")
        self.formula_host.pack(fill="both", expand=True, padx=14)
        self.formula_entries: dict[str, list[tk.Entry]] = {}
        self._rebuild()

        buttons = tk.Frame(self.window, bg="#101827")
        buttons.pack(fill="x", padx=16, pady=12)
        tk.Button(
            buttons, text="生成并执行", command=self._accept,
            bg="#396fda", fg="white", relief="flat", padx=18,
        ).pack(side="right", padx=4)
        tk.Button(
            buttons, text="取消", command=self.window.destroy,
            bg="#1d2c47", fg="white", relief="flat", padx=18,
        ).pack(side="right", padx=4)
        self.window.bind("<Escape>", lambda _event: self.window.destroy())
        self.window.wait_window()

    def _rebuild(self) -> None:
        for child in self.formula_host.winfo_children():
            child.destroy()
        space = SPACE_LABELS[self.space.get()]
        labels = JOINT_LABELS if space == "JointAngleSpace" else CARTESIAN_LABELS
        self.formula_entries = {}
        for column, side in enumerate(SIDES):
            box = ttk.LabelFrame(self.formula_host, text=SIDE_TITLES[side])
            box.grid(row=0, column=column, sticky="nsew", padx=7, pady=4)
            self.formula_host.grid_columnconfigure(column, weight=1)
            values = self.initial[side][space]
            entries: list[tk.Entry] = []
            for row, (label, value) in enumerate(zip(labels, values)):
                tk.Label(box, text=f"{label}(t) =", bg="#101827", fg="#b9c9e8", width=10).grid(
                    row=row, column=0, padx=(8, 3), pady=7, sticky="e"
                )
                formula = _entry(box, f"{value:.10g}", 30)
                formula.grid(row=row, column=1, padx=(0, 8), pady=7, sticky="ew")
                box.grid_columnconfigure(1, weight=1)
                entries.append(formula)
            self.formula_entries[side] = entries

    def _accept(self) -> None:
        try:
            t_start = float(self.t_start.get())
            t_end = float(self.t_end.get())
            duration_s = float(self.duration.get())
            keypoint_count = int(self.keypoints.get())
            if not all(math.isfinite(value) for value in (t_start, t_end, duration_s)):
                raise ValueError("t 和时长必须是有限数字")
            if t_end == t_start:
                raise ValueError("t 终点不能等于起点")
            if duration_s <= 0:
                raise ValueError("总时长必须大于 0")
            if not 2 <= keypoint_count <= 500:
                raise ValueError("关键点数必须在 2 到 500 之间")
            if duration_s * 1000.0 < keypoint_count:
                raise ValueError("总时长过短，无法为每个关键点分配至少 1 ms")
            sources = {
                side: [entry.get().strip() for entry in self.formula_entries[side]]
                for side in SIDES
            }
            compiled = {
                side: [Formula.compile(source) for source in sources[side]]
                for side in SIDES
            }
            for side in SIDES:
                for formula in compiled[side]:
                    formula.evaluate(t_start)
                    formula.evaluate(t_end)
            self.result = {
                "space": SPACE_LABELS[self.space.get()],
                "t_start": t_start,
                "t_end": t_end,
                "duration_ms": int(round(duration_s * 1000.0)),
                "keypoint_count": keypoint_count,
                "sources": sources,
                "compiled": compiled,
            }
            self.window.destroy()
        except ValueError as exc:
            messagebox.showerror("函数轨迹输入错误", str(exc), parent=self.window)


class HistoryPointDialog:
    def __init__(self, parent: tk.Misc, entry: dict[str, Any],
                 alternatives: dict[str, dict[str, list[float]]]):
        self.result: dict[str, Any] | None = None
        self.entry = entry
        self.alternatives = alternatives
        self.window = tk.Toplevel(parent)
        self.window.title("编辑历史关键点")
        self.window.geometry("760x560")
        self.window.configure(bg="#101827")
        self.window.transient(parent)
        self.window.grab_set()

        header = tk.Frame(self.window, bg="#101827")
        header.pack(fill="x", padx=14, pady=10)
        tk.Label(header, text="名称", bg="#101827", fg="#91a4c6").pack(side="left")
        self.label = _entry(header, entry.get("label", "历史点"), 28)
        self.label.pack(side="left", padx=5)
        tk.Label(header, text="时长(ms)", bg="#101827", fg="#91a4c6").pack(side="left", padx=(15, 0))
        self.duration = _entry(header, int(entry.get("duration_ms", 2000)), 10)
        self.duration.pack(side="left", padx=5)

        self.side_host = tk.Frame(self.window, bg="#101827")
        self.side_host.pack(fill="both", expand=True, padx=14)
        self.space_vars: dict[str, tk.StringVar] = {}
        self.value_hosts: dict[str, tk.Frame] = {}
        self.value_entries: dict[str, list[tk.Entry]] = {}
        self.cached_values: dict[str, dict[str, list[float]]] = {}
        for column, side in enumerate(SIDES):
            box = ttk.LabelFrame(self.side_host, text=SIDE_TITLES[side])
            box.grid(row=0, column=column, sticky="nsew", padx=7, pady=4)
            self.side_host.grid_columnconfigure(column, weight=1)
            frame = entry["frame_targets"][side]
            initial_space = frame["space"]
            self.cached_values[side] = {
                name: list(values) for name, values in alternatives[side].items()
            }
            if initial_space == "JointAngleSpace":
                self.cached_values[side][initial_space] = [
                    math.degrees(value) for value in frame["target"]["jointAngles"]
                ]
            else:
                target = frame["target"]
                self.cached_values[side][initial_space] = [
                    target["x"], target["y"], target["z"],
                    math.degrees(target["yaw"]), math.degrees(target["pitch"]),
                    math.degrees(target["roll"]),
                ]
            variable = tk.StringVar(value=SPACE_NAMES[initial_space])
            self.space_vars[side] = variable
            selector = ttk.Combobox(
                box, textvariable=variable, state="readonly",
                values=tuple(SPACE_LABELS), width=14,
            )
            selector.pack(fill="x", padx=8, pady=7)
            selector.bind("<<ComboboxSelected>>", lambda _event, s=side: self._rebuild_side(s))
            host = tk.Frame(box, bg="#101827")
            host.pack(fill="both", expand=True, padx=4)
            self.value_hosts[side] = host
            self._rebuild_side(side)

        tk.Label(
            self.window, text="位置单位 mm；关节角和姿态单位 deg。修改后会重新计算该点及其后的历史起点。",
            bg="#101827", fg="#91a4c6",
        ).pack(fill="x", padx=20, pady=5)
        buttons = tk.Frame(self.window, bg="#101827")
        buttons.pack(fill="x", padx=16, pady=12)
        tk.Button(buttons, text="保存", command=self._accept, bg="#396fda", fg="white", relief="flat", padx=18).pack(side="right", padx=4)
        tk.Button(buttons, text="取消", command=self.window.destroy, bg="#1d2c47", fg="white", relief="flat", padx=18).pack(side="right", padx=4)
        self.window.bind("<Escape>", lambda _event: self.window.destroy())
        self.window.wait_window()

    def _rebuild_side(self, side: str) -> None:
        host = self.value_hosts[side]
        for child in host.winfo_children():
            child.destroy()
        space = SPACE_LABELS[self.space_vars[side].get()]
        labels = JOINT_LABELS if space == "JointAngleSpace" else CARTESIAN_LABELS
        values = self.cached_values[side][space]
        entries: list[tk.Entry] = []
        for row, (label, value) in enumerate(zip(labels, values)):
            tk.Label(host, text=label, bg="#101827", fg="#b9c9e8", width=7).grid(row=row, column=0, padx=3, pady=6)
            widget = _entry(host, f"{value:.10g}", 20)
            widget.grid(row=row, column=1, padx=3, pady=6, sticky="ew")
            host.grid_columnconfigure(1, weight=1)
            entries.append(widget)
        self.value_entries[side] = entries

    def _accept(self) -> None:
        try:
            duration_ms = int(self.duration.get())
            if duration_ms <= 0:
                raise ValueError("时长必须是正整数毫秒")
            values = {}
            spaces = {}
            for side in SIDES:
                spaces[side] = SPACE_LABELS[self.space_vars[side].get()]
                values[side] = [float(widget.get()) for widget in self.value_entries[side]]
                if not all(math.isfinite(value) for value in values[side]):
                    raise ValueError(f"{SIDE_TITLES[side]}包含非有限数字")
            self.result = {
                "label": self.label.get().strip() or "历史点",
                "duration_ms": duration_ms,
                "spaces": spaces,
                "values": values,
            }
            self.window.destroy()
        except ValueError as exc:
            messagebox.showerror("历史点输入错误", str(exc), parent=self.window)
