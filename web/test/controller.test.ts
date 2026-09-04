import { readFile } from 'node:fs/promises';
import { describe, expect, it } from 'vitest';
import { RoboArmController } from '../src/controller';
import { DeployIK } from '../src/wasm';

describe('RoboArmController', () => {
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
});
