#!/usr/bin/env python3
"""JSON interchange matching heterogeneous PerformanceAction::Frame types."""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any

FORMAT = "performance-action-sequence-v2"
JOINT_ONLY_FORMAT = "performance-action-sequence-v1"
LEGACY_FORMAT = "roboarm-history-v1"
SIDES = ("left", "right")
JOINT_COUNT = 7
UNITS = {
    "JointAngleSpace.jointAngles": "rad",
    "CartesianSpace.position": "mm",
    "CartesianSpace.orientation": "rad",
    "Frame.duration": "ms",
    "Frame.timeout": "ms",
}


def _finite(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _joint_array(values: Any, unit: str = "rad") -> list[float]:
    if isinstance(values, (str, bytes)):
        raise ValueError("JointAngleSpace.jointAngles must contain exactly 7 values")
    try:
        values = list(values)
    except TypeError as exc:
        raise ValueError("JointAngleSpace.jointAngles must contain exactly 7 values") from exc
    if len(values) != JOINT_COUNT:
        raise ValueError("JointAngleSpace.jointAngles must contain exactly 7 values")
    result = [_finite(value, "joint angle") for value in values]
    if unit == "deg": result = [math.radians(value) for value in result]
    elif unit != "rad": raise ValueError(f"unsupported joint angle unit: {unit!r}")
    return result


def _milliseconds(value: Any, unit: str = "ms") -> int:
    number = _finite(value, "time")
    if unit == "s": number *= 1000.0
    elif unit != "ms": raise ValueError(f"unsupported time unit: {unit!r}")
    if number < 0: raise ValueError("duration/timeout cannot be negative")
    return int(round(number))


def joint_frame_target(values: Any) -> dict[str, Any]:
    return {"space": "JointAngleSpace", "target": {"jointAngles": _joint_array(values)}}


def cartesian_frame_target(x: Any, y: Any, z: Any,
                           yaw: Any, pitch: Any, roll: Any) -> dict[str, Any]:
    return {
        "space": "CartesianSpace",
        "target": {
            "x": _finite(x, "Cartesian x"), "y": _finite(y, "Cartesian y"),
            "z": _finite(z, "Cartesian z"),
            "yaw": _finite(yaw, "Cartesian yaw"),
            "pitch": _finite(pitch, "Cartesian pitch"),
            "roll": _finite(roll, "Cartesian roll"),
        },
    }


def _normalize_target(frame: dict[str, Any], units: dict[str, str]) -> dict[str, Any]:
    space, target = frame.get("space"), frame.get("target")
    if not isinstance(target, dict): raise ValueError("frame.target must be an object")
    if space == "JointAngleSpace":
        return joint_frame_target(_joint_array(
            target.get("jointAngles"), units.get("JointAngleSpace.jointAngles", "rad")))
    if space == "CartesianSpace":
        p_unit = units.get("CartesianSpace.position", "mm")
        r_unit = units.get("CartesianSpace.orientation", "rad")
        scale_p = 1000.0 if p_unit == "m" else 1.0
        scale_r = math.pi / 180.0 if r_unit == "deg" else 1.0
        if p_unit not in ("mm", "m"): raise ValueError(f"unsupported position unit: {p_unit!r}")
        if r_unit not in ("rad", "deg"): raise ValueError(f"unsupported orientation unit: {r_unit!r}")
        return cartesian_frame_target(
            _finite(target.get("x"), "x") * scale_p,
            _finite(target.get("y"), "y") * scale_p,
            _finite(target.get("z"), "z") * scale_p,
            _finite(target.get("yaw"), "yaw") * scale_r,
            _finite(target.get("pitch"), "pitch") * scale_r,
            _finite(target.get("roll"), "roll") * scale_r)
    raise ValueError(f"unsupported frame space: {space!r}")


def _with_timing(target: dict[str, Any], duration: Any, timeout: Any) -> dict[str, Any]:
    return {**target, "duration": _milliseconds(duration), "timeout": _milliseconds(timeout)}


def history_to_sequence(history: list[dict[str, Any]], default_duration_ms: int = 2000,
                        force_joint_space: bool = False) -> dict[str, Any]:
    if not history: raise ValueError("cannot export an empty history")
    default_duration_ms = _milliseconds(default_duration_ms)
    first_start = history[0].get("start")
    if not isinstance(first_start, dict): raise ValueError("first history item has no start state")
    sequences = {side: {"frames": []} for side in SIDES}
    initial_duration = _milliseconds(history[0].get("duration_ms", default_duration_ms))
    for side in SIDES:
        sequences[side]["frames"].append(
            _with_timing(joint_frame_target(first_start[side]), initial_duration, 0))
    labels = ["initial"]
    for index, item in enumerate(history, 1):
        duration = _milliseconds(item.get("duration_ms", default_duration_ms))
        timeout = _milliseconds(item.get("timeout_ms", 0))
        frame_targets = item.get("frame_targets", {})
        solved_target = item.get("target", {})
        for side in SIDES:
            source = None if force_joint_space else (frame_targets.get(side) if isinstance(frame_targets, dict) else None)
            canonical = _normalize_target(source, UNITS) if isinstance(source, dict) else joint_frame_target(solved_target[side])
            sequences[side]["frames"].append(_with_timing(canonical, duration, timeout))
        labels.append(str(item.get("label", f"frame {index}")))
    return {"format": FORMAT, "units": dict(UNITS), "sequences": sequences,
            "metadata": {"synchronized": True, "frameLabels": labels}}


def validate_sequence(payload: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    if not isinstance(payload, dict) or payload.get("format") != FORMAT:
        raise ValueError(f"expected format {FORMAT!r}")
    units = payload.get("units", {})
    sequences = payload.get("sequences")
    if not isinstance(sequences, dict): raise ValueError("sequences must be an object")
    parsed: dict[str, list[dict[str, Any]]] = {}
    for side in SIDES:
        seq = sequences.get(side)
        frames = seq.get("frames") if isinstance(seq, dict) else None
        if not isinstance(frames, list) or not frames: raise ValueError(f"sequences.{side}.frames must be non-empty")
        parsed[side] = []
        for frame in frames:
            if not isinstance(frame, dict): raise ValueError("frame must be an object")
            target = _normalize_target(frame, units)
            parsed[side].append(_with_timing(
                target,
                _milliseconds(frame.get("duration", 0), units.get("Frame.duration", "ms")),
                _milliseconds(frame.get("timeout", 0), units.get("Frame.timeout", "ms"))))
    if len(parsed["left"]) != len(parsed["right"]): raise ValueError("left/right frame count differs")
    for index in range(len(parsed["left"])):
        if parsed["left"][index]["duration"] != parsed["right"][index]["duration"]:
            raise ValueError(f"left/right duration differs at frame {index}")
        if parsed["left"][index]["timeout"] != parsed["right"][index]["timeout"]:
            raise ValueError(f"left/right timeout differs at frame {index}")
    return parsed


def sequence_to_history(payload: dict[str, Any]) -> list[dict[str, Any]]:
    parsed = validate_sequence(payload)
    if any(parsed[side][0]["space"] != "JointAngleSpace" for side in SIDES):
        raise ValueError("frame zero must be JointAngleSpace so playback has a deterministic start")
    labels = payload.get("metadata", {}).get("frameLabels", [])
    initial = {side: parsed[side][0]["target"]["jointAngles"] for side in SIDES}
    history: list[dict[str, Any]] = []
    for index in range(1, len(parsed["left"])):
        frames = {side: parsed[side][index] for side in SIDES}
        history.append({
            "label": str(labels[index]) if index < len(labels) else f"imported frame {index}",
            "time": "", "start": initial if index == 1 else None, "target": None,
            "frame_targets": {side: {"space": frames[side]["space"], "target": frames[side]["target"]} for side in SIDES},
            "duration_ms": frames["left"]["duration"], "timeout_ms": frames["left"]["timeout"],
        })
    return history


def _upgrade_joint_v1(payload: dict[str, Any]) -> dict[str, Any]:
    units = payload.get("units", {})
    sequences = payload.get("sequences", {})
    result = {side: {"frames": []} for side in SIDES}
    for side in SIDES:
        seq = sequences.get(side, {})
        if seq.get("space") != "JointAngleSpace": raise ValueError("v1 only supports JointAngleSpace")
        for frame in seq.get("frames", []):
            canonical = joint_frame_target(_joint_array(
                frame.get("target", {}).get("jointAngles"), units.get("JointAngleSpace.jointAngles", "rad")))
            result[side]["frames"].append(_with_timing(
                canonical,
                _milliseconds(frame.get("duration", 0), units.get("Frame.duration", "ms")),
                _milliseconds(frame.get("timeout", 0), units.get("Frame.timeout", "ms"))))
    return {"format": FORMAT, "units": dict(UNITS), "sequences": result,
            "metadata": payload.get("metadata", {"synchronized": True})}


def normalize_payload(payload: Any, default_duration_ms: int = 2000) -> dict[str, Any]:
    if isinstance(payload, dict) and payload.get("format") == FORMAT:
        parsed = validate_sequence(payload)
        return {"format": FORMAT, "units": dict(UNITS),
                "sequences": {side: {"frames": parsed[side]} for side in SIDES},
                "metadata": payload.get("metadata", {"synchronized": True})}
    if isinstance(payload, dict) and payload.get("format") == JOINT_ONLY_FORMAT:
        return _upgrade_joint_v1(payload)
    history = payload.get("history") if isinstance(payload, dict) else payload
    if not isinstance(history, list): raise ValueError("legacy history must be an array")
    return history_to_sequence(history, default_duration_ms)


def main() -> None:
    parser = argparse.ArgumentParser(description="Normalize RoboArm JSON to mixed-space PerformanceAction v2")
    parser.add_argument("input", type=Path); parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--duration-ms", type=int, default=2000)
    parser.add_argument("--backup", action="store_true")
    args = parser.parse_args(); output = args.output or args.input
    payload = json.loads(args.input.read_text(encoding="utf-8"))
    converted = normalize_payload(payload, args.duration_ms)
    if args.backup and output.resolve() == args.input.resolve():
        backup = args.input.with_name(args.input.stem + ".pre-v2-backup" + args.input.suffix)
        shutil.copy2(args.input, backup); print(f"backup: {backup}")
    output.write_text(json.dumps(converted, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(f"converted: {output} ({len(converted['sequences']['left']['frames'])} frames/arm)")


if __name__ == "__main__": main()
