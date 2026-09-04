# Dual-arm RoboArm MuJoCo 3D visualizer

This is the native MuJoCo viewer (no web UI). It reuses the MJCF builder in
`doubleArm7Rpro_TV.py` and calls the exact deployed C++ `AdaptiveHybrid` IK in
`Arm/Lib/myikine7R` through a small DLL bridge.

## Run

```bash
conda activate mjlab_env
python -m pip install -r requirements_visualizer.txt
python mujoco_dual_arm_visualizer.py
```

If `conda activate` is unavailable in PowerShell, use the equivalent one-shot command:

```powershell
conda run -n mjlab_env python mujoco_dual_arm_visualizer.py
```

The script opens a MuJoCo 3D window plus a small Tk control panel. The panel accepts joint angles J1..J7, absolute Cartesian X/Y/Z + Yaw/Pitch/Roll pose targets, or relative Delta X/Y/Z + Delta Yaw/Pitch/Roll pose moves for both arms.

At first launch, `deploy_ik_bridge.cpp` is compiled with `g++` to a DLL in the
system temporary directory. Cartesian requests therefore use the same
geometry, installation matrices, asymmetric left/right limits, tolerances and
DLS/Hybrid routing as `Arm/Src/ArmMiddleware.cpp`. An unreachable target or an
out-of-limit direct joint request is rejected instead of being silently
accepted. Motion is smoothly interpolated from the current state to the solved
target.

The displayed/exported Cartesian coordinates use the firmware
`CartesianSpace` origin at the J0 arm bases. The rendered waist structure has a
physical `(192.5, 0, 365) mm` scene offset, which the UI removes; do not add it
to action files.

## Function trajectories

Click `ƒ(t) 添加函数轨迹` to define either all seven logical joint angles or a
full Cartesian pose as functions of `t` for both arms. Formulas use LaTeX, for
example `30\sin(2\pi t)` or `\frac{1}{2}t^2`. Joint/orientation results are in
degrees and position results are in millimetres. Set the `t` interval, total
duration and number of key points; the current arm state is treated as the
state at the start of the interval.

Every sampled key point appears as a normal history item and can be restored or
replayed. Rendering interpolation between points is not saved. JSON/C++ export
therefore contains only the sampled key points.

Select one history item and click `编辑选中历史点` (or press F2) to change its
label, duration, target-space type and values. The edited item and all later
Cartesian items are solved again from their new preceding states. If any point
becomes invalid or unreachable, the entire edit is rolled back.

The history panel also has `导出历史 JSON` and `导入历史 JSON` buttons. The JSON stores source/target poses, action labels and timestamps, so an imported sequence can be selected and replayed like a live session.

The model uses the deployed arm axis order, mirrored bases, link dimensions
(158.5 mm upper arm, 60 mm elbow offset, 249.38 mm forearm, 92.57 mm tool),
joint limits, masses and COMs. Its synthetic isotropic inertias are sufficient
for kinematics and gravity verification, but are not identified CAD inertias
and must not be treated as a high-fidelity transient dynamics model.

## Generate PerformanceAction C++

Convert the exported JSON into two constexpr `PerformanceAction::makeSequence(...)` definitions:

```bash
conda activate mjlab_env
python action_sequence_to_cpp.py actionsequence.json actionsequence.generated.hpp
```

Optional names:

```bash
python action_sequence_to_cpp.py actionsequence.json generated.hpp --namespace MyActions --left-name kLeftDance --right-name kRightDance
```

The v2 action JSON stores `space` on every frame, so one sequence may contain both `JointAngleSpace` and `CartesianSpace` frames exactly like the heterogeneous tuple in `ArmSequenceManager.hpp`. Canonical units are joint/orientation angles in `rad`, Cartesian positions in `mm`, and timing in `ms`.

The visualizer export selector offers either the recorded mixed spaces or an all-`JointAngleSpace` export using the MuJoCo IK solutions. The C++ generator emits the matching `PerformanceAction::frame(...)` overload for every frame. Angles smaller than `--zero-epsilon` can be normalized to `0.0_rad`; the conservative default is `1e-10` rad.
