import { readFile } from 'node:fs/promises';
import { describe, expect, it } from 'vitest';
import { exportSequence, importSequence, jointTarget, recomputeHistory } from '../src/sequence';
import { zeroState, type HistoryPoint } from '../src/types';
import { DeployIK } from '../src/wasm';

const mockIk = {
  limits: () => ({ lower: Array(7).fill(-10), upper: Array(7).fill(10) }),
  solve: () => { throw new Error('not expected'); },
} as unknown as DeployIK;

describe('PerformanceAction v2', () => {
  it('round-trips joint keypoints', () => {
    const base = zeroState();
    const q1 = Array(7).fill(.1), q2 = Array(7).fill(.2);
    const history: HistoryPoint[] = [
      { label: 'one', start: zeroState(), target: { left: q1, right: q1 }, frameTargets: { left: jointTarget(q1), right: jointTarget(q1) }, durationMs: 500, timeoutMs: 0 },
      { label: 'two', start: { left: q1, right: q1 }, target: { left: q2, right: q2 }, frameTargets: { left: jointTarget(q2), right: jointTarget(q2) }, durationMs: 700, timeoutMs: 10 },
    ];
    const payload = exportSequence(history, base);
    const loaded = importSequence(payload, mockIk);
    expect(loaded.history).toHaveLength(2);
    expect(loaded.history[1].target.left).toEqual(q2);
    expect(loaded.history[1].durationMs).toBe(700);
  });

  it('recomputes every downstream point', () => {
    const history: HistoryPoint[] = Array.from({ length: 3 }, (_, i) => ({
      label: String(i), start: zeroState(), target: zeroState(),
      frameTargets: { left: jointTarget(Array(7).fill(i + 1)), right: jointTarget(Array(7).fill(i + 1)) },
      durationMs: 100, timeoutMs: 0,
    }));
    recomputeHistory(mockIk, history, zeroState());
    expect(history[2].start.left[0]).toBe(2);
    expect(history[2].target.left[0]).toBe(3);
  });

  it('imports and solves the project actionsequence.json', async () => {
    const wasmBase = new URL('../public/wasm/', import.meta.url).href;
    const wasmBinary = await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url));
    const actualIk = await DeployIK.load(wasmBase, { wasmBinary });
    const payload = JSON.parse(await readFile(new URL('../../actionsequence.json', import.meta.url), 'utf8'));
    const loaded = importSequence(payload, actualIk);
    expect(loaded.history).toHaveLength(21);
    const cartesianArmFrames = loaded.history.reduce((count, point) => count + Number(point.frameTargets.left.space === 'CartesianSpace') + Number(point.frameTargets.right.space === 'CartesianSpace'), 0);
    expect(cartesianArmFrames).toBe(26);
  });
});
