export type Side = 'left' | 'right';
export type ArmState = Record<Side, number[]>;
export type Space = 'JointAngleSpace' | 'CartesianSpace';

export interface JointTarget {
  space: 'JointAngleSpace';
  target: { jointAngles: number[] };
}

export interface CartesianTarget {
  space: 'CartesianSpace';
  target: { x: number; y: number; z: number; yaw: number; pitch: number; roll: number };
}

export type FrameTarget = JointTarget | CartesianTarget;

export interface FunctionMetadata {
  group: number;
  pointIndex: number;
  pointCount: number;
  t: number;
  tStart: number;
  tEnd: number;
  space: Space;
  sources: Record<Side, string[]>;
}

export interface HistoryPoint {
  label: string;
  start: ArmState;
  target: ArmState;
  frameTargets: Record<Side, FrameTarget>;
  durationMs: number;
  timeoutMs: number;
  function?: FunctionMetadata;
}

export type SequenceFrame = FrameTarget & { duration: number; timeout: number };

export interface ActionSequence {
  format: 'performance-action-sequence-v2';
  units: Record<string, string>;
  sequences: Record<Side, { frames: SequenceFrame[] }>;
  metadata?: { synchronized?: boolean; frameLabels?: string[] };
}

export const SIDES: Side[] = ['left', 'right'];
export const zeroState = (): ArmState => ({ left: Array(7).fill(0), right: Array(7).fill(0) });
export const cloneState = (state: ArmState): ArmState => ({ left: [...state.left], right: [...state.right] });
