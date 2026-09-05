import { DEG } from './math';
import { cloneState, SIDES, type ActionSequence, type ArmState, type FrameTarget, type HistoryPoint, type SequenceFrame, type Side } from './types';
import type { DeployIK } from './wasm';

export const UNITS = {
  'JointAngleSpace.jointAngles': 'rad',
  'CartesianSpace.position': 'mm',
  'CartesianSpace.orientation': 'rad',
  'Frame.duration': 'ms',
  'Frame.timeout': 'ms',
};

const finite = (value: unknown, name: string): number => {
  const n = Number(value);
  if (!Number.isFinite(n)) throw new Error(`${name} 必须是有限数字`);
  return n;
};

export function jointTarget(q: number[]): FrameTarget {
  if (q.length !== 7) throw new Error('关节目标必须包含 7 个角度');
  return { space: 'JointAngleSpace', target: { jointAngles: q.map((v, i) => finite(v, `J${i + 1}`)) } };
}

export function cartesianTarget(values: number[]): FrameTarget {
  if (values.length !== 6) throw new Error('位姿必须包含 X/Y/Z/Yaw/Pitch/Roll');
  values = values.map((v, i) => finite(v, `位姿分量 ${i + 1}`));
  return { space: 'CartesianSpace', target: { x: values[0], y: values[1], z: values[2], yaw: values[3], pitch: values[4], roll: values[5] } };
}

function canonicalTarget(raw: any, units: Record<string, string>): FrameTarget {
  if (!raw || typeof raw !== 'object' || !raw.target) throw new Error('动作帧缺少 target');
  if (raw.space === 'JointAngleSpace') {
    let q = Array.from(raw.target.jointAngles ?? [], Number);
    if (units['JointAngleSpace.jointAngles'] === 'deg') q = q.map(v => v * DEG);
    else if ((units['JointAngleSpace.jointAngles'] ?? 'rad') !== 'rad') throw new Error('不支持的关节角单位');
    return jointTarget(q);
  }
  if (raw.space === 'CartesianSpace') {
    const positionScale = units['CartesianSpace.position'] === 'm' ? 1000 : 1;
    const orientationScale = units['CartesianSpace.orientation'] === 'deg' ? DEG : 1;
    if (!['mm', 'm'].includes(units['CartesianSpace.position'] ?? 'mm')) throw new Error('不支持的位置单位');
    if (!['rad', 'deg'].includes(units['CartesianSpace.orientation'] ?? 'rad')) throw new Error('不支持的姿态单位');
    return cartesianTarget([
      raw.target.x * positionScale, raw.target.y * positionScale, raw.target.z * positionScale,
      raw.target.yaw * orientationScale, raw.target.pitch * orientationScale, raw.target.roll * orientationScale,
    ]);
  }
  throw new Error(`未知目标空间：${raw.space}`);
}

function timeMs(value: unknown, unit: string | undefined): number {
  const n = finite(value ?? 0, '时间') * (unit === 's' ? 1000 : 1);
  if (unit && unit !== 'ms' && unit !== 's') throw new Error(`不支持的时间单位：${unit}`);
  if (n < 0) throw new Error('时间不能为负数');
  return Math.round(n);
}

export function solveTarget(ik: DeployIK, side: Side, frame: FrameTarget, current: number[]): number[] {
  if (frame.space === 'JointAngleSpace') {
    const { lower, upper } = ik.limits(side);
    frame.target.jointAngles.forEach((q, i) => {
      // Limits originate in the firmware float model; allow its sub-microradian
      // representation error when JSON stores an exact degree boundary.
      if (q < lower[i] - 2e-6 || q > upper[i] + 2e-6) throw new Error(`${side === 'left' ? '左' : '右'}臂 J${i + 1} 超出实机限位`);
    });
    return [...frame.target.jointAngles];
  }
  const t = frame.target;
  const { rpyMatrix } = requireMath();
  return ik.solve(side, rpyMatrix(t.roll, t.pitch, t.yaw), [t.x, t.y, t.z], current).q;
}

// Kept synchronous and tree-shakeable while avoiding circular model dependencies.
function requireMath() {
  return { rpyMatrix: (roll: number, pitch: number, yaw: number) => {
    const [cr, sr, cp, sp, cy, sy] = [Math.cos(roll), Math.sin(roll), Math.cos(pitch), Math.sin(pitch), Math.cos(yaw), Math.sin(yaw)];
    return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr];
  }};
}

export function recomputeHistory(ik: DeployIK, history: HistoryPoint[], base: ArmState, from = 0): void {
  let current = from === 0 ? cloneState(base) : cloneState(history[from - 1].target);
  for (let i = from; i < history.length; i++) {
    history[i].start = cloneState(current);
    const next = {} as ArmState;
    for (const side of SIDES) next[side] = solveTarget(ik, side, history[i].frameTargets[side], current[side]);
    history[i].target = cloneState(next);
    current = next;
  }
}

export function exportSequence(history: HistoryPoint[], base: ArmState, forceJoint = false): ActionSequence {
  if (!history.length) throw new Error('没有可导出的历史关键点');
  const sequences = { left: { frames: [] as SequenceFrame[] }, right: { frames: [] as SequenceFrame[] } };
  for (const side of SIDES) {
    sequences[side].frames.push({ ...jointTarget(base[side]), duration: history[0].durationMs, timeout: 0 } as SequenceFrame);
    for (const point of history) {
      const target = forceJoint ? jointTarget(point.target[side]) : point.frameTargets[side];
      sequences[side].frames.push({ ...structuredClone(target), duration: point.durationMs, timeout: point.timeoutMs } as SequenceFrame);
    }
  }
  return { format: 'performance-action-sequence-v2', units: { ...UNITS }, sequences, metadata: { synchronized: true, frameLabels: ['initial', ...history.map(point => point.label)] } };
}

export function importSequence(payload: any, ik: DeployIK): { base: ArmState; history: HistoryPoint[] } {
  if (!payload || payload.format !== 'performance-action-sequence-v2') throw new Error('仅支持 performance-action-sequence-v2');
  const units = payload.units ?? {};
  const parsed = {} as Record<Side, SequenceFrame[]>;
  for (const side of SIDES) {
    const frames = payload.sequences?.[side]?.frames;
    if (!Array.isArray(frames) || frames.length === 0) throw new Error(`${side} 动作序列为空`);
    parsed[side] = frames.map((frame: any) => ({ ...canonicalTarget(frame, units), duration: timeMs(frame.duration, units['Frame.duration']), timeout: timeMs(frame.timeout, units['Frame.timeout']) } as SequenceFrame));
    if (parsed[side][0].space !== 'JointAngleSpace') throw new Error('第 0 帧必须是 JointAngleSpace');
  }
  if (parsed.left.length !== parsed.right.length) throw new Error('左右臂帧数不同');
  const base = { left: [...(parsed.left[0] as any).target.jointAngles], right: [...(parsed.right[0] as any).target.jointAngles] };
  const labels: string[] = payload.metadata?.frameLabels ?? [];
  const history: HistoryPoint[] = [];
  for (let i = 1; i < parsed.left.length; i++) {
    if (parsed.left[i].duration !== parsed.right[i].duration || parsed.left[i].timeout !== parsed.right[i].timeout) throw new Error(`第 ${i} 帧左右臂时间不同`);
    history.push({ label: labels[i] || `导入关键点 ${i}`, start: cloneState(base), target: cloneState(base), frameTargets: { left: parsed.left[i], right: parsed.right[i] }, durationMs: parsed.left[i].duration, timeoutMs: parsed.left[i].timeout });
  }
  recomputeHistory(ik, history, base);
  return { base, history };
}
