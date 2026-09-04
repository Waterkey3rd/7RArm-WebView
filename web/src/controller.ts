import { DEG, matrixToYpr, multiply3, rpyMatrix, smoothstep } from './math';
import { LatexFormula } from './formula';
import { cartesianTarget, exportSequence, importSequence, jointTarget, recomputeHistory, solveTarget } from './sequence';
import { cloneState, SIDES, type ActionSequence, type ArmState, type FrameTarget, type HistoryPoint, type Side, type Space, zeroState } from './types';
import { DeployIK } from './wasm';

export interface CommandOptions { label?: string; durationMs?: number; timeoutMs?: number }
export interface FunctionTrajectoryOptions extends CommandOptions {
  space: Space;
  sources: Record<Side, string[]>;
  tStart: number;
  tEnd: number;
  keypointCount: number;
  allowDiscontinuity?: boolean;
}
export interface HistoryEdit {
  label?: string;
  durationMs?: number;
  timeoutMs?: number;
  targets?: Partial<Record<Side, FrameTarget>>;
}
export interface PlaybackFrame { start: ArmState; target: ArmState; durationMs: number; historyIndex: number }
export type ControllerListener = (controller: RoboArmController) => void;

const validDuration = (value = 2000): number => {
  if (!Number.isFinite(value) || value < 1) throw new Error('运动时长必须不少于 1 ms');
  return Math.round(value);
};

export class RoboArmController {
  readonly ik: DeployIK;
  readonly wasmBase: string;
  base: ArmState = zeroState();
  current: ArmState = zeroState();
  history: HistoryPoint[] = [];
  private readonly listeners = new Set<ControllerListener>();
  private workerId = 0;

  private constructor(ik: DeployIK, wasmBase: string) { this.ik = ik; this.wasmBase = wasmBase; }

  static async create(wasmBase: string): Promise<RoboArmController> {
    const normalized = wasmBase.endsWith('/') ? wasmBase : `${wasmBase}/`;
    const ik = await DeployIK.load(normalized);
    return new RoboArmController(ik, normalized);
  }

  subscribe(listener: ControllerListener): () => void {
    this.listeners.add(listener); return () => this.listeners.delete(listener);
  }
  private changed(): void { this.listeners.forEach(listener => listener(this)); }

  private append(frameTargets: Record<Side, FrameTarget>, options: CommandOptions = {}): HistoryPoint {
    const start = cloneState(this.current), target = {} as ArmState;
    for (const side of SIDES) target[side] = solveTarget(this.ik, side, frameTargets[side], start[side]);
    const point: HistoryPoint = {
      label: options.label ?? '目标关键点', start, target: cloneState(target), frameTargets: structuredClone(frameTargets),
      durationMs: validDuration(options.durationMs), timeoutMs: Math.max(0, Math.round(options.timeoutMs ?? 0)),
    };
    this.history.push(point); this.current = cloneState(target); this.changed(); return point;
  }

  commandJoints(valuesDeg: Record<Side, number[]>, options: CommandOptions = {}): HistoryPoint {
    return this.append({ left: jointTarget(valuesDeg.left.map(v => v * DEG)), right: jointTarget(valuesDeg.right.map(v => v * DEG)) }, { label: '关节目标', ...options });
  }

  commandCartesian(values: Record<Side, number[]>, options: CommandOptions = {}): HistoryPoint {
    const targets = {} as Record<Side, FrameTarget>;
    for (const side of SIDES) {
      if (values[side].length !== 6) throw new Error('位姿需要 X/Y/Z/Yaw/Pitch/Roll 六个分量');
      const v = [...values[side]]; for (let i = 3; i < 6; i++) v[i] *= DEG;
      targets[side] = cartesianTarget(v);
    }
    return this.append(targets, { label: '笛卡尔目标', ...options });
  }

  commandDelta(values: Record<Side, number[]>, options: CommandOptions = {}): HistoryPoint {
    const absolute = {} as Record<Side, number[]>;
    for (const side of SIDES) {
      const delta = values[side];
      if (delta.length !== 6) throw new Error('Delta 需要 ΔX/ΔY/ΔZ/ΔYaw/ΔPitch/ΔRoll 六个分量');
      const fk = this.ik.fk(side, this.current[side]);
      const rotation = multiply3(fk.rotation, rpyMatrix(delta[5] * DEG, delta[4] * DEG, delta[3] * DEG));
      const [yaw, pitch, roll] = matrixToYpr(rotation);
      absolute[side] = [fk.position[0] + delta[0], fk.position[1] + delta[1], fk.position[2] + delta[2], yaw / DEG, pitch / DEG, roll / DEG];
    }
    return this.commandCartesian(absolute, { label: 'Delta 位姿移动', ...options });
  }

  snapshotValues(state = this.current): Record<Side, Record<Space, number[]>> {
    const result = {} as Record<Side, Record<Space, number[]>>;
    for (const side of SIDES) {
      const fk = this.ik.fk(side, state[side]); const [yaw, pitch, roll] = matrixToYpr(fk.rotation);
      result[side] = { JointAngleSpace: state[side].map(q => q / DEG), CartesianSpace: [...fk.position, yaw / DEG, pitch / DEG, roll / DEG] };
    }
    return result;
  }

  async addFunctionTrajectory(options: FunctionTrajectoryOptions): Promise<HistoryPoint[]> {
    const componentCount = options.space === 'JointAngleSpace' ? 7 : 6;
    if (!Number.isInteger(options.keypointCount) || options.keypointCount < 1) throw new Error('关键点数量必须是正整数');
    if (!(options.tEnd > options.tStart)) throw new Error('tEnd 必须大于 tStart');
    const compiled = {} as Record<Side, LatexFormula[]>;
    for (const side of SIDES) {
      if (options.sources[side].length !== componentCount) throw new Error(`${side} 公式数量不正确`);
      compiled[side] = options.sources[side].map(LatexFormula.compile);
    }
    const initial = this.snapshotValues();
    if (!options.allowDiscontinuity) {
      for (const side of SIDES) {
        const start = compiled[side].map(formula => formula.evaluate(options.tStart));
        const actual = initial[side][options.space];
        const positionCount = options.space === 'CartesianSpace' ? 3 : 0;
        for (let i = 0; i < componentCount; i++) {
          const delta = i >= positionCount && options.space === 'CartesianSpace' ? Math.abs((start[i] - actual[i] + 180) % 360 - 180) : Math.abs(start[i] - actual[i]);
          const tolerance = options.space === 'JointAngleSpace' ? .5 : (i < 3 ? 1 : 1);
          if (delta > tolerance) throw new Error(`公式 t 起点未衔接当前状态：${side} 分量 ${i + 1} 相差 ${delta.toFixed(3)}`);
        }
      }
    }
    const totalMs = validDuration(options.durationMs);
    const edges = Array.from({ length: options.keypointCount + 1 }, (_, i) => Math.round(i * totalMs / options.keypointCount));
    if (edges.some((v, i) => i > 0 && v <= edges[i - 1])) throw new Error('总时长过短，关键点间隔不足 1 ms');
    const points = Array.from({ length: options.keypointCount }, (_, i) => {
      const t = options.tStart + (options.tEnd - options.tStart) * (i + 1) / options.keypointCount;
      const values = {} as Record<Side, number[]>;
      for (const side of SIDES) {
        values[side] = compiled[side].map(formula => formula.evaluate(t));
        if (options.space === 'JointAngleSpace') values[side] = values[side].map(v => v * DEG);
        else for (let j = 3; j < 6; j++) values[side][j] *= DEG;
      }
      return { t, values, durationMs: edges[i + 1] - edges[i] };
    });
    const solved = await this.solveFunctionWorker(options.space, points);
    const group = 1 + this.history.filter(point => point.function?.pointIndex === 1).length;
    const generated: HistoryPoint[] = solved.map((item: any, i: number) => ({
      label: options.label ?? `函数轨迹 ${group} · ${i + 1}/${solved.length} · t=${item.t.toPrecision(6)}`,
      start: cloneState(item.start), target: cloneState(item.target),
      frameTargets: {
        left: options.space === 'JointAngleSpace' ? jointTarget(item.values.left) : cartesianTarget(item.values.left),
        right: options.space === 'JointAngleSpace' ? jointTarget(item.values.right) : cartesianTarget(item.values.right),
      },
      durationMs: item.durationMs, timeoutMs: Math.max(0, Math.round(options.timeoutMs ?? 0)),
      function: { group, pointIndex: i + 1, pointCount: solved.length, t: item.t, tStart: options.tStart, tEnd: options.tEnd, space: options.space, sources: structuredClone(options.sources) },
    }));
    this.history.push(...generated); this.current = cloneState(generated.at(-1)!.target); this.changed(); return generated;
  }

  private solveFunctionWorker(space: Space, points: Array<{ t: number; values: Record<Side, number[]>; durationMs: number }>): Promise<any[]> {
    const id = ++this.workerId;
    const worker = new Worker(new URL('./kinematics.worker.ts', import.meta.url), { type: 'module' });
    return new Promise((resolve, reject) => {
      worker.onmessage = event => {
        if (event.data.id !== id) return;
        worker.terminate(); event.data.ok ? resolve(event.data.results) : reject(new Error(event.data.error));
      };
      worker.onerror = event => { worker.terminate(); reject(new Error(event.message)); };
      worker.postMessage({ id, wasmBase: this.wasmBase, start: cloneState(this.current), space, points });
    });
  }

  editHistory(index: number, edit: HistoryEdit): HistoryPoint {
    if (!this.history[index]) throw new Error('历史点索引不存在');
    const backup = structuredClone(this.history);
    try {
      const point = this.history[index];
      if (edit.label !== undefined) point.label = edit.label;
      if (edit.durationMs !== undefined) point.durationMs = validDuration(edit.durationMs);
      if (edit.timeoutMs !== undefined) point.timeoutMs = Math.max(0, Math.round(edit.timeoutMs));
      for (const side of SIDES) if (edit.targets?.[side]) point.frameTargets[side] = structuredClone(edit.targets[side]!);
      delete point.function;
      recomputeHistory(this.ik, this.history, this.base, index);
      this.current = cloneState(this.history.at(-1)?.target ?? this.base); this.changed(); return point;
    } catch (error) { this.history = backup; throw error; }
  }

  undo(): void { if (this.history.length) this.history.pop(); this.current = cloneState(this.history.at(-1)?.target ?? this.base); this.changed(); }
  reset(): void { this.history = []; this.base = zeroState(); this.current = zeroState(); this.changed(); }
  restore(index: number): ArmState { if (!this.history[index]) throw new Error('历史点索引不存在'); this.current = cloneState(this.history[index].target); this.changed(); return cloneState(this.current); }

  playback(from = 0, to = this.history.length - 1): PlaybackFrame[] {
    if (from < 0 || to >= this.history.length || from > to) throw new Error('播放范围无效');
    return this.history.slice(from, to + 1).map((point, offset) => ({ start: cloneState(point.start), target: cloneState(point.target), durationMs: point.durationMs, historyIndex: from + offset }));
  }
  interpolate(frame: PlaybackFrame, elapsedMs: number): ArmState {
    const ratio = smoothstep(Math.max(0, Math.min(1, elapsedMs / frame.durationMs))); const state = {} as ArmState;
    for (const side of SIDES) state[side] = frame.start[side].map((q, i) => q + ratio * (frame.target[side][i] - q));
    return state;
  }

  export(forceJointSpace = false): ActionSequence { return exportSequence(this.history, this.base, forceJointSpace); }
  import(payload: unknown): void { const loaded = importSequence(payload, this.ik); this.base = cloneState(loaded.base); this.history = loaded.history; this.current = cloneState(this.base); this.changed(); }
}
