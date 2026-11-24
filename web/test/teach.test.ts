import { describe, expect, it } from 'vitest';
import { deleteFrames, editFrameRange, frameTable, insertFrames, parseRecording, resample, splitTrack } from '../src/teach';

const header = 'mcu_us,host_t,v0,v1,v2,v3,v4,v5,v6,v7';
const row = (us: number, side: number, q: number) => `${us},1,${side},${q},-0.7,0,0.8,0,0,0`;

describe('HostLink recordings', () => {
  it('splits clock regressions and retains the latest duplicate per arm', () => {
    const result = parseRecording([
      header, row(125000, 0, 0), row(124000, 1, 9), row(145000, 0, 1),
      row(145000, 0, 2), row(35000, 0, 3), row(55000, 0, 4), '60000,1,0,3,0',
    ].join('\n'));
    expect(result.truncated).toBe(true);
    expect(splitTrack(result.samples, 0).map(track => track.map(sample => sample.q[0])))
      .toEqual([[0, 2], [3, 4]]);
  });

  it('retains the endpoints and the exact recording duration', () => {
    const result = parseRecording([header, row(1000, 0, 0), row(206000, 0, 1)].join('\n'));
    const frames = resample(result.samples, 120);
    expect(frames.map(frame => frame.q[0])).toEqual([0, 0.5, 1]);
    expect(frames.map(frame => frame.ms)).toEqual([103, 102, 0]);
  });

  it('crops inclusively and removes the outgoing interval of the new last frame', () => {
    const frames = [10, 20, 30, 40, 0].map((ms, i) => ({ q: [i], ms }));
    const result = editFrameRange(frames, 1, 3, 'crop');
    expect(result).toEqual([{ q: [1], ms: 20 }, { q: [2], ms: 30 }, { q: [3], ms: 0 }]);
    result[0].q[0] = 99;
    expect(frames[1]).toEqual({ q: [1], ms: 20 });
  });

  it('removes selected frames, preserves adjacent intervals and rejects a one-frame result', () => {
    const frames = [10, 20, 30, 40, 0].map((ms, i) => ({ q: [i], ms }));
    expect(editFrameRange(frames, 1, 2, 'delete')).toEqual([
      { q: [0], ms: 10 }, { q: [3], ms: 40 }, { q: [4], ms: 0 },
    ]);
    expect(editFrameRange(frames, 3, 4, 'delete').at(-1)?.ms).toBe(0);
    expect(() => editFrameRange(frames, 0, 3, 'delete')).toThrow('至少需要两帧');
  });

  it('deletes disjoint point and segment selections together', () => {
    const frames = [10, 20, 30, 40, 50, 0].map((ms, i) => ({ q: [i], ms }));
    expect(deleteFrames(frames, new Set([1, 3, 4]))).toEqual([
      { q: [0], ms: 10 }, { q: [2], ms: 30 }, { q: [5], ms: 0 },
    ]);
  });

  it('can insert an interior point, prepend a segment or append a segment', () => {
    const q = (j0: number) => [j0, 0, 0, 0, 0, 0, 0];
    const frames = [{ q: q(0), ms: 120 }, { q: q(1), ms: 0 }];
    const point = { q: q(0.5), ms: 0 };
    const segment = [{ q: q(1), ms: 100 }, { q: q(2), ms: 0 }];
    expect(insertFrames(frames, 1, [point], 40).map(frame => [frame.q[0], frame.ms]))
      .toEqual([[0, 120], [0.5, 40], [1, 0]]);
    expect(insertFrames(frames, 0, segment, 60).map(frame => frame.ms)).toEqual([100, 60, 120, 0]);
    expect(insertFrames(frames, 2, segment, 60).map(frame => frame.ms)).toEqual([120, 60, 100, 0]);
    expect(frames[1].ms).toBe(0);
  });

  it('exports scoped aliases and frame intervals in microseconds', () => {
    const q = [0, 0, 0, 0, 0, 0, 0];
    const output = frameTable([{ q, ms: 12 }, { q, ms: 0 }], q);
    expect(output).toContain('using Target = ConceptArm::JointTarget<7>;');
    expect(output).toContain('Duration{12000.0f}');
  });
});
