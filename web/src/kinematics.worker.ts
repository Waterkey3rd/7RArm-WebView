import { rpyMatrix } from './math';
import type { ArmState, Side, Space } from './types';
import { SIDES } from './types';
import { DeployIK } from './wasm';

interface Request {
  id: number;
  wasmBase: string;
  start: ArmState;
  space: Space;
  points: Array<{ t: number; values: Record<Side, number[]>; durationMs: number; isStartTransition?: boolean }>;
}

self.onmessage = async (event: MessageEvent<Request>) => {
  const request = event.data;
  try {
    const ik = await DeployIK.load(request.wasmBase);
    let current: ArmState = { left: [...request.start.left], right: [...request.start.right] };
    const results = [];
    for (const point of request.points) {
      const next = {} as ArmState;
      for (const side of SIDES) {
        const values = point.values[side];
        if (request.space === 'JointAngleSpace') {
          const limits = ik.limits(side);
          values.forEach((value, i) => {
            if (value < limits.lower[i] - 2e-6 || value > limits.upper[i] + 2e-6) throw new Error(`t=${point.t}: ${side} J${i + 1} 超出实机限位`);
          });
          next[side] = [...values];
        } else {
          next[side] = ik.solve(side, rpyMatrix(values[5], values[4], values[3]), values.slice(0, 3), current[side]).q;
        }
      }
      results.push({ ...point, start: current, target: next });
      current = next;
    }
    self.postMessage({ id: request.id, ok: true, results });
  } catch (error) {
    self.postMessage({ id: request.id, ok: false, error: error instanceof Error ? error.message : String(error) });
  }
};

export {};
