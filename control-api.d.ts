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
  useVision?: boolean;     // solve() aims from the camera pose estimate
  cameraHeight?: number;   // m above the tiles
  cameraPitchDeg?: number; // up positive
  cameraFovDeg?: number;   // horizontal field of view
  pixelNoise?: number;     // corner noise σ, px
  frames?: number;         // frames fused per estimate
  seed?: number;           // integer noise seed
  resolution?: '640x480' | '1280x720' | '1280x800';
  pitchErrorDeg?: number;  // real camera pitch minus its calibration (chassis tilt, mounting error)
  hideTagIds?: number[];   // tags left out of the scene, as if covered
  estimatePitch?: boolean; // solve camera pitch with a ±3° prior around the calibration (default true)
  mode?: 'image' | 'geometry'; // rendered frame + AprilTag detector, or projected corners + noise
  sensorNoise?: number;    // image mode: Gaussian read noise, gray levels
  exposureGain?: number;
  exposureMs?: number;
  driveSpeed?: number;     // m/s forward during the exposure (motion blur)
  turnRateDegS?: number;   // °/s during the exposure (motion blur)
  k1?: number;             // radial distortion; corners are undistorted with this known value
  defocus?: number;        // Gaussian blur σ, px
  decimate?: number;       // detector quad_decimate
}

export type TagRejection = 'behind' | 'range' | 'angle' | 'fov' | 'small' | 'occluded' | 'dropout';
export interface Pose2D { x: number; y: number; headingDeg: number }
export interface FieldTag {  // Manual §9.9: IDs 30–45, 3.25 in, four per CELL bottom; moves when the HIVE tips
  id: number; hive: 'red' | 'blue'; side: -1 | 1; cluster: string; x: number; y: number; z: number; size: number;
  normal: Vec3; up: Vec3;  // tag faces along normal; up points to its top edge
}
export interface QualityGate { ok: boolean; blurPx: number; reasons: ('blur' | 'singleFarTag' | 'none')[] }
export interface Localization {
  mode: 'image' | 'geometry';
  gate: QualityGate | null;
  pitchDeg: number | null; // estimated camera pitch
  aim: { yawErrDeg: number; lateralAtTarget: number; rangeError: number } | null; // effect of the pose error on the shot
  hiveUp: { red: -1 | 1 | null; blue: -1 | 1 | null } | null; // upward CELL side per HIVE from the best-fitting layout
  timing: { renderMs: number; detectMs: number } | null; // image mode
  missedTagIds: number[];  // image mode: in view geometrically but not detected
  truth: Pose2D;
  estimate: (Pose2D & { std: { x: number; y: number; headingDeg: number } | null; rmsPx: number; outlierTagIds: number[] }) | null;
  error: { x: number; y: number; position: number; headingDeg: number } | null;
  detectedTagIds: number[];
  rejected: { id: number; reason: TagRejection | 'hamming' | 'margin' | 'unknown' }[]; // image mode lists discarded decodes
  framesUsed: number;
  detections: { id: number; corners: { u: number; v: number }[]; ideal: { u: number; v: number }[] }[];
}
export interface VisionAim { ok: boolean; estimate: Pose2D | null; error: Localization['error']; rejectedBecause: QualityGate['reasons'] }

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
  solve(input?: ShotInput): (Prediction & { openingClearance: number; vision: VisionAim | null }) |
    { status: 'unreachable'; team: 'red' | 'blue'; cellSide: -1 | 1; target: Vec3; vision: VisionAim | null };
  localize(input?: ShotInput): Localization | null;
  getTags(): FieldTag[];
  whenDetectorReady(): Promise<boolean>;
  fire(input?: ShotInput): { shot: Prediction; tipped: boolean; state: FieldState };
  reset(): FieldState;
}

declare global {
  interface Window { HiveControl: HiveControlAPI }
  const HiveControl: HiveControlAPI;
}
