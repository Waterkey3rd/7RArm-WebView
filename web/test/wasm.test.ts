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
});
