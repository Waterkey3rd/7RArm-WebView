export type Sample = { us: number; side: number; q: number[] };
export type Frame = { q: number[]; ms: number };
export type Vec = [number, number, number];
export type Recorded = [number, ...Vec];
export type Planned = [number, ...Vec, number[]];
export type Preview = {
  start: number[]; model: number[][]; tcpHome: Vec; tcpBasis: Vec[];
  recorded: Recorded[]; planned: Planned[]; submitted: number; end: number; rejected: number;
};

export function sample(us: unknown, side: unknown, q: unknown[]): Sample | null {
  if (typeof us !== 'number' || !Number.isSafeInteger(us) || us < 0 ||
      (side !== 0 && side !== 1) || q.length !== 7 ||
      !q.every((value): value is number => typeof value === 'number' && Number.isFinite(value))) return null;
  return { us, side, q };
}

export function parseRecording(text: string): { samples: Sample[]; truncated: boolean } {
  const [header, ...rows] = text.trim().replace(/^\uFEFF/, '').split(/\r?\n/);
  if (header !== 'mcu_us,host_t,v0,v1,v2,v3,v4,v5,v6,v7') throw Error('请选择 HostLink 的 side_q.csv');
  const samples: Sample[] = [];
  let truncated = false;
  rows.forEach((line, i) => {
    const fields = line.split(',');
    if (i === rows.length - 1 && fields.length >= 3 && fields.length < 10) {
      truncated = true;
      return;
    }
    const [us, host, side, ...q] = fields.map(Number);
    const parsed = sample(us, side, q);
    if (!parsed || !Number.isFinite(host) || fields.some((field) => !field.trim())) {
      throw Error(`CSV 第 ${i + 2} 行无效`);
    }
    samples.push(parsed);
  });
  if (!samples.length) throw Error('没有完整的关节记录');
  return { samples, truncated };
}

export function splitTrack(samples: Sample[], side: number): Sample[][] {
  const tracks: Sample[][] = [[]];
  for (const item of samples) {
    if (item.side !== side) continue;
    const track = tracks[tracks.length - 1];
    const last = track[track.length - 1];
    if (last && item.us < last.us) tracks.push([item]);
    else if (last && item.us === last.us) track[track.length - 1] = item;
    else track.push(item);
  }
  return tracks.filter((track) => track.length > 1);
}

export function resample(track: Sample[], step: number): Frame[] {
  if (!Number.isInteger(step) || step < 1) throw Error('采样间隔须为正整数');
  if (track.length < 2) throw Error('该侧至少需要两条记录');
  const first = track[0].us;
  const span = track[track.length - 1].us - first;
  const ms = Math.round(span / 1000);
  const count = Math.ceil(ms / step);
  if (ms < 1 || ms > 600_000 || count >= 10_000) throw Error('请选择 10 分钟以内、10000 帧以内的录制段');
  let next = 1;
  return Array.from({ length: count + 1 }, (_, i) => {
    const time = first + span * i / count;
    while (track[next].us < time) next++;
    const a = track[next - 1], b = track[next];
    const f = (time - a.us) / (b.us - a.us);
    return { q: a.q.map((angle, axis) => angle + f * (b.q[axis] - angle)),
      ms: i === count ? 0 : Math.round(ms * (i + 1) / count) - Math.round(ms * i / count) };
  });
}

export function nearest<T>(points: T[], time: number, key: (point: T) => number): number {
  let lo = 0, hi = points.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (key(points[mid]) < time) lo = mid + 1;
    else hi = mid;
  }
  return lo > 0 && time - key(points[lo - 1]) < key(points[lo]) - time ? lo - 1 : lo;
}

export function editFrameRange(
  frames: Frame[], start: number, end: number, operation: 'crop' | 'delete',
): Frame[] {
  if (!Number.isInteger(start) || !Number.isInteger(end) || start < 0 || end >= frames.length || start > end) {
    throw Error('选段无效');
  }
  if (operation === 'delete') {
    return deleteFrames(frames, new Set(Array.from({ length: end - start + 1 }, (_, i) => start + i)));
  }
  return copyTrack(frames.slice(start, end + 1));
}

function copyTrack(frames: Frame[]): Frame[] {
  if (frames.length < 2) throw Error('轨迹至少需要两帧');
  return frames.map((frame, i) => ({ q: frame.q.slice(), ms: i === frames.length - 1 ? 0 : frame.ms }));
}

export function deleteFrames(frames: Frame[], indices: ReadonlySet<number>): Frame[] {
  return copyTrack(frames.filter((_, i) => !indices.has(i)));
}

// An interior insertion retains the interval leading out of the preceding frame.
// The extra interval connects the new segment back to the original path.
export function insertFrames(frames: Frame[], index: number, additions: Frame[], interval: number): Frame[] {
  const addedDuration = additions.slice(0, -1).reduce((sum, frame) => sum + frame.ms, interval);
  if (!Number.isInteger(index) || index < 0 || index > frames.length || frames.length < 2 ||
      !additions.length || frames.length + additions.length > 10000 ||
      additions.some(({ q }) => q.length !== 7 || !q.every(Number.isFinite)) ||
      !Number.isInteger(interval) || interval < 1 ||
      frames.reduce((sum, frame) => sum + frame.ms, addedDuration) > 600000) {
    throw Error('请输入有效的七轴角度和正整数衔接间隔，整段最长 10 分钟、最多 10000 帧');
  }
  const result = frames.map((frame) => ({ q: frame.q.slice(), ms: frame.ms }));
  const appended = index === result.length;
  if (appended) result[index - 1].ms = interval;
  result.splice(index, 0, ...additions.map((frame, i) => ({
    q: frame.q.slice(),
    ms: i < additions.length - 1 ? frame.ms : appended ? 0 : interval,
  })));
  return result;
}

export function frameTable(frames: Frame[], start: number[]): string {
  const rows = frames.map((frame, i) => {
    const angles = (i ? frame.q : start).map((angle) => `Angle{${angle.toExponential(8)}f}`).join(', ');
    return `        {Target{{${angles}}}, Duration{${frame.ms * 1000}.0f}},`;
  });
  return [
    'inline constexpr auto taughtFrames = [] {',
    '    using ConceptArm::Angle;',
    '    using ConceptRobo::Duration;',
    '    using Target = ConceptArm::JointTarget<7>;',
    `    return MoveSegment<${frames.length}>{{`, ...rows, '    }};', '}();', '',
  ].join('\n');
}
