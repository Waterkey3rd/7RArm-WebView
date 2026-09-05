export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;

export function rpyMatrix(roll: number, pitch: number, yaw: number): number[] {
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(roll), Math.sin(roll), Math.cos(pitch), Math.sin(pitch), Math.cos(yaw), Math.sin(yaw)];
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

export function matrixToYpr(r: number[]): [number, number, number] {
  const sy = Math.hypot(r[0], r[3]);
  if (sy > 1e-9) return [Math.atan2(r[3], r[0]), Math.atan2(-r[6], sy), Math.atan2(r[7], r[8])];
  return [Math.atan2(-r[1], r[4]), Math.atan2(-r[6], sy), 0];
}

export function multiply3(a: number[], b: number[]): number[] {
  const out = Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) out[3 * i + j] += a[3 * i + k] * b[3 * k + j];
  return out;
}

export const smoothstep = (t: number): number => t * t * (3 - 2 * t);
