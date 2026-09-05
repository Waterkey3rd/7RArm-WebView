import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { resolveWasmBase } from '../src/wasm';

interface Module {
  HEAPF64: Float64Array; _malloc(n: number): number; _free(p: number): void;
  _deploy_fk(side: number, q: number, r: number, p: number): number;
  _deploy_ik_solve(side: number, r: number, p: number, current: number, out: number, d: number): number;
  _deploy_get_joint_limits(side: number, lower: number, upper: number): number;
  _deploy_model_version(): number;
  _deploy_trajectory_duration(start: number, target: number, requestedMs: number): number;
  _deploy_trajectory_sample(start: number, target: number, durationMs: number, elapsedMs: number, position: number, velocity: number): number;
}

async function moduleInstance(): Promise<Module> {
  const jsUrl = new URL('../public/wasm/deploy_ik.js', import.meta.url);
  const wasm = await readFile(fileURLToPath(new URL('../public/wasm/deploy_ik.wasm', import.meta.url)));
  const factory = (await import(jsUrl.href)).default;
  return factory({ wasmBinary: wasm });
}

function memory(module: Module, lengths: number[], action: (p: number[]) => void): number[][] {
  const p = lengths.map(n => module._malloc(n * 8));
  try { action(p); return p.map((pointer, i) => Array.from(module.HEAPF64.subarray(pointer / 8, pointer / 8 + lengths[i]))); }
  finally { p.forEach(pointer => module._free(pointer)); }
}

describe('deployed IK WASM ABI', () => {
  it('resolves a relative public path against the page, not src/wasm.ts', () => {
    expect(resolveWasmBase('./wasm/', 'http://localhost:5173/')).toBe('http://localhost:5173/wasm/');
    expect(resolveWasmBase('./wasm/', 'https://host.example/robot/index.html')).toBe('https://host.example/robot/wasm/');
  });

  it('exports model version and asymmetric real limits', async () => {
    const m = await moduleInstance(); expect(m._deploy_model_version()).toBe(1);
    const left = memory(m, [7, 7], p => { expect(m._deploy_get_joint_limits(0, p[0], p[1])).toBe(1); });
    const right = memory(m, [7, 7], p => { expect(m._deploy_get_joint_limits(1, p[0], p[1])).toBe(1); });
    expect(left[1][6] * 180 / Math.PI).toBeCloseTo(269, 3);
    expect(right[0][6] * 180 / Math.PI).toBeCloseTo(-269, 3);
  });

  it('round-trips FK targets through AdaptiveHybrid IK', async () => {
    const m = await moduleInstance();
    for (const side of [0, 1]) {
      const q = [0, -.4, .2, .7, -.15, .25, side ? -.3 : .3];
      const [rotation, position] = memory(m, [7, 9, 3], p => { m.HEAPF64.set(q, p[0] / 8); expect(m._deploy_fk(side, p[0], p[1], p[2])).toBe(1); }).slice(1);
      const [solved, diagnostics] = memory(m, [9, 3, 7, 7, 8], p => {
        m.HEAPF64.set(rotation, p[0] / 8); m.HEAPF64.set(position, p[1] / 8); m.HEAPF64.set(q, p[2] / 8);
        expect(m._deploy_ik_solve(side, p[0], p[1], p[2], p[3], p[4])).toBe(1);
      }).slice(3);
      expect(diagnostics[0]).toBe(1);
      expect(Math.max(...solved.map((v, i) => Math.abs(v - q[i])))).toBeLessThan(2e-4);
    }
  });

  it('uses the firmware quintic planner with synchronized limits', async () => {
    const m = await moduleInstance();
    const start = Array(7).fill(0);
    const target = [0, .8, 0, 0, 0, 0, 0];
    let duration = 0;
    memory(m, [7, 7], pointers => {
      m.HEAPF64.set(start, pointers[0] / 8);
      m.HEAPF64.set(target, pointers[1] / 8);
      duration = m._deploy_trajectory_duration(pointers[0], pointers[1], 500);
    });
    // Doubled J2 acceleration limit (4 rad/s^2) is the active constraint.
    const expectedDuration = Math.sqrt((10 / Math.sqrt(3)) * .8 / 4) * 1000;
    expect(duration).toBeCloseTo(expectedDuration, 2);
    const [position, velocity] = memory(m, [7, 7, 7, 7], pointers => {
      m.HEAPF64.set(start, pointers[0] / 8);
      m.HEAPF64.set(target, pointers[1] / 8);
      expect(m._deploy_trajectory_sample(pointers[0], pointers[1], duration, duration / 2, pointers[2], pointers[3])).toBe(1);
    }).slice(2);
    expect(position[1]).toBeCloseTo(.4, 6);
    expect(velocity[1]).toBeCloseTo(1.875 * .8 / (duration / 1000), 6);
    const [endPosition, endVelocity] = memory(m, [7, 7, 7, 7], pointers => {
      m.HEAPF64.set(start, pointers[0] / 8);
      m.HEAPF64.set(target, pointers[1] / 8);
      m._deploy_trajectory_sample(pointers[0], pointers[1], duration, duration, pointers[2], pointers[3]);
    }).slice(2);
    expect(endPosition[1]).toBeCloseTo(.8, 6);
    expect(endVelocity[1]).toBe(0);
  });
});
