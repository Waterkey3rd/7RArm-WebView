import type { Side } from './types';

interface EmscriptenModule {
  HEAPF64: Float64Array;
  _malloc(size: number): number;
  _free(pointer: number): void;
  _deploy_ik_solve(side: number, rotation: number, position: number, current: number, output: number, diagnostics: number): number;
  _deploy_fk(side: number, q: number, rotation: number, position: number): number;
  _deploy_fk_chain(side: number, q: number, positions: number, axes: number): number;
  _deploy_get_joint_limits(side: number, lower: number, upper: number): number;
  _deploy_model_version(): number;
}

type ModuleFactory = (options?: Record<string, unknown>) => Promise<EmscriptenModule>;
const sideIndex = (side: Side): number => side === 'left' ? 0 : 1;

export function resolveWasmBase(baseUrl = './wasm/', pageBase?: string): string {
  const withSlash = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
  const runtimeBase = pageBase ?? (typeof document !== 'undefined'
    ? document.baseURI
    : (typeof location !== 'undefined' ? location.href : undefined));
  if (runtimeBase) return new URL(withSlash, runtimeBase).href;
  // Node tests and server-side callers must already provide an absolute URL.
  return new URL(withSlash).href;
}

export interface IkResult {
  q: number[];
  positionErrorMm: number;
  orientationErrorRad: number;
  route: number;
}

export interface FkResult { rotation: number[]; position: number[] }
export interface ChainResult { positions: number[][]; axes: number[][] }

export class DeployIK {
  private constructor(private readonly module: EmscriptenModule) {}

  static async load(baseUrl = './wasm/', options: { wasmBinary?: Uint8Array } = {}): Promise<DeployIK> {
    const resolvedBase = resolveWasmBase(baseUrl);
    const factory = (await import(/* @vite-ignore */ `${resolvedBase}deploy_ik.js`)).default as ModuleFactory;
    const module = await factory({ locateFile: (name: string) => `${resolvedBase}${name}`, ...options });
    if (module._deploy_model_version() !== 1) throw new Error('不支持的机械臂 WASM 模型版本');
    return new DeployIK(module);
  }

  private arrays(lengths: number[], action: (pointers: number[]) => void): number[][] {
    const pointers = lengths.map(length => this.module._malloc(length * 8));
    try {
      action(pointers);
      return pointers.map((pointer, i) => Array.from(this.module.HEAPF64.subarray(pointer / 8, pointer / 8 + lengths[i])));
    } finally {
      pointers.forEach(pointer => this.module._free(pointer));
    }
  }

  solve(side: Side, rotation: number[], positionMm: number[], current: number[]): IkResult {
    let ok = 0;
    const [q, diagnostics] = this.arrays([9, 3, 7, 7, 8], pointers => {
      this.module.HEAPF64.set(rotation, pointers[0] / 8);
      this.module.HEAPF64.set(positionMm, pointers[1] / 8);
      this.module.HEAPF64.set(current, pointers[2] / 8);
      ok = this.module._deploy_ik_solve(sideIndex(side), pointers[0], pointers[1], pointers[2], pointers[3], pointers[4]);
    }).slice(3);
    if (!ok || diagnostics[0] !== 1) {
      throw new Error(`${side === 'left' ? '左' : '右'}臂 IK 无有效解（位置误差 ${diagnostics[1].toFixed(3)} mm，姿态误差 ${(diagnostics[2] * 180 / Math.PI).toFixed(3)}°）`);
    }
    return { q, positionErrorMm: diagnostics[1], orientationErrorRad: diagnostics[2], route: diagnostics[3] };
  }

  fk(side: Side, q: number[]): FkResult {
    let ok = 0;
    const [rotation, position] = this.arrays([7, 9, 3], pointers => {
      this.module.HEAPF64.set(q, pointers[0] / 8);
      ok = this.module._deploy_fk(sideIndex(side), pointers[0], pointers[1], pointers[2]);
    }).slice(1);
    if (!ok) throw new Error('FK 调用失败');
    return { rotation, position };
  }

  chain(side: Side, q: number[]): ChainResult {
    let ok = 0;
    const [positionsFlat, axesFlat] = this.arrays([7, 24, 21], pointers => {
      this.module.HEAPF64.set(q, pointers[0] / 8);
      ok = this.module._deploy_fk_chain(sideIndex(side), pointers[0], pointers[1], pointers[2]);
    }).slice(1);
    if (!ok) throw new Error('连杆 FK 调用失败');
    const chunk = (values: number[], n: number) => Array.from({ length: values.length / n }, (_, i) => values.slice(i * n, (i + 1) * n));
    return { positions: chunk(positionsFlat, 3), axes: chunk(axesFlat, 3) };
  }

  limits(side: Side): { lower: number[]; upper: number[] } {
    const [lower, upper] = this.arrays([7, 7], pointers => {
      if (!this.module._deploy_get_joint_limits(sideIndex(side), pointers[0], pointers[1])) throw new Error('关节限位读取失败');
    });
    return { lower, upper };
  }
}
