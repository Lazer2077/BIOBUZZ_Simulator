export type BallType = 'pollen' | 'nectar';
export type ShotStatus = 'hit' | 'collision' | 'miss';
export type Vec3 = { x: number; y: number; z: number };

export interface ShotInput {
  x?: number;              // launch position, metres
  y?: number;
  z?: number;
  speed?: number;          // launch speed, m/s
  angleDeg?: number;       // elevation above horizontal
  yawDeg?: number;         // 0 = +X, 90 = +Y
  drag?: number;           // dimensionless Cd; exploratory range 0.3–1.0
  ballType?: BallType;
  releaseTorque?: number;  // tip threshold, N·m
}

export interface HiveState {
  up: -1 | 1;              // side of upward CELL on field Y axis
  addedMass: number;       // kg added since last stable state
  pollen: number;
  nectar: number;
  tips: number;
  angle: number;           // tip animation coordinate, radians
  omega: number;           // tip angular speed, rad/s
  moving: boolean;
}

export interface FieldState {
  units: { position: 'm'; speed: 'm/s'; angle: 'deg'; torque: 'N·m'; time: 's' };
  nearestHive: 'red' | 'blue';
  targetCellSide: -1 | 1;
  target: Vec3;
  hives: { red: HiveState; blue: HiveState };
}

export interface Prediction {
  status: ShotStatus;
  team: 'red' | 'blue';
  cellSide: -1 | 1;
  angleDeg: number;
  yawDeg: number;
  speed: number;
  ballType: BallType;
  flightTime: number | null;
  hitPoint: Vec3 | null;
  collision: ({ kind: string; t: number } & Vec3) | null;
  dragCoefficient: number;
  internalCollisions: ({ kind: string; t: number } & Vec3)[];
  retained: boolean;      // soft-contact CELL model; settled inside, independent of contact count
  target: Vec3;
}

export interface HiveControlAPI {
  getState(): FieldState;
  predict(input?: ShotInput): Prediction;
  solve(input?: ShotInput): (Prediction & { openingClearance: number }) |
    { status: 'unreachable'; team: 'red' | 'blue'; cellSide: -1 | 1; target: Vec3 };
  fire(input?: ShotInput): { shot: Prediction; tipped: boolean; state: FieldState };
  reset(): FieldState;
}

declare global {
  interface Window { HiveControl: HiveControlAPI }
  const HiveControl: HiveControlAPI;
}
