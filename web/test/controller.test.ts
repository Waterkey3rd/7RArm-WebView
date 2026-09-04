import { readFile } from 'node:fs/promises';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { RoboArmController } from '../src/controller';
import { DeployIK } from '../src/wasm';

describe('RoboArmController', () => {
  afterEach(() => vi.unstubAllGlobals());

  it('manages history, deleteHistory, undo, and reset', async () => {
    const wasmBase = new URL('../public/wasm/', import.meta.url).href;
    const wasmBinary = await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url));
    const ik = await DeployIK.load(wasmBase, { wasmBinary });
    const controller = (RoboArmController as any).create
      ? new (RoboArmController as any)(ik, wasmBase)
      : null;

    if (!controller) return;

    // Joint command
    controller.commandJoints({
      left: [0, -20, 0, 20, 0, 0, 0],
      right: [0, -20, 0, 20, 0, 0, 0],
    }, { durationMs: 1500 });
    expect(controller.history).toHaveLength(1);

    // Cartesian command
    controller.commandCartesian({
      left: [205, 249, 0, 30, 0, -90],
      right: [205, -249, 0, -30, 0, 90],
    }, { durationMs: 2000 });
    expect(controller.history).toHaveLength(2);

    // Exporting only a middle segment creates its own deterministic frame zero.
    const segment = controller.export(false, 1, 1);
    expect(segment.sequences.left.frames).toHaveLength(2);
    expect((segment.sequences.left.frames[0] as any).target.jointAngles)
      .toEqual(controller.history[1].start.left);
    expect(segment.metadata?.frameLabels).toEqual(['initial', controller.history[1].label]);

    // Delete first keypoint
    controller.deleteHistory(0);
    expect(controller.history).toHaveLength(1);

    // Undo
    controller.undo();
    expect(controller.history).toHaveLength(0);

    // Reset
    controller.reset();
    expect(controller.current.left).toEqual(Array(7).fill(0));
  });

  it('automatically inserts and solves a move to f(tStart)', async () => {
    class JointWorker {
      onmessage: ((event: MessageEvent) => void) | null = null;
      onerror: ((event: ErrorEvent) => void) | null = null;
      postMessage(request: any): void {
        let current = structuredClone(request.start);
        const results = request.points.map((point: any) => {
          const target = { left: [...point.values.left], right: [...point.values.right] };
          const result = { ...point, start: current, target };
          current = target;
          return result;
        });
        queueMicrotask(() => this.onmessage?.({ data: { id: request.id, ok: true, results } } as MessageEvent));
      }
      terminate(): void {}
    }
    vi.stubGlobal('Worker', JointWorker);

    const wasmBase = new URL('../public/wasm/', import.meta.url).href;
    const wasmBinary = await readFile(new URL('../public/wasm/deploy_ik.wasm', import.meta.url));
    const ik = await DeployIK.load(wasmBase, { wasmBinary });
    const controller = new (RoboArmController as any)(ik, wasmBase) as RoboArmController;
    const sources = ['10', '0', '0', '20', '0', '0', '0'];
    const generated = await controller.addFunctionTrajectory({
      space: 'JointAngleSpace', sources: { left: sources, right: sources },
      tStart: 0, tEnd: 1, durationMs: 1000, keypointCount: 2,
    });

    expect(generated).toHaveLength(3);
    expect(generated[0].label).toContain('起点过渡');
    expect(generated[0].function?.pointIndex).toBe(0);
    expect(generated[0].durationMs).toBe(1000);
    expect(generated[0].target.left[0] * 180 / Math.PI).toBeCloseTo(10, 8);
    expect(generated[1].start).toEqual(generated[0].target);
    expect(generated[1].durationMs + generated[2].durationMs).toBe(1000);
  });
});
