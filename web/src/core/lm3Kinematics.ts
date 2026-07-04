import type { Vector3 } from './types';

export const LM3_HOME_JOINTS = [0, 0, 0, 0, 0, 0] as const;

export const LM3_JOINT_LIMITS = [
  { min: -Math.PI, max: Math.PI },
  { min: -Math.PI, max: Math.PI },
  { min: -Math.PI, max: Math.PI },
  { min: -Math.PI, max: Math.PI },
  { min: -Math.PI, max: Math.PI },
  { min: -Math.PI, max: Math.PI },
] as const;

export interface Lm3Pose {
  position: Vector3;
  direction: Vector3;
}

export interface Lm3IkResult {
  joints: number[];
  error: number;
  directionScore: number;
  reachable: boolean;
  iterations: number;
}

export interface Lm3CartesianWaypoint {
  id: string;
  position: Vector3;
  targetDirection?: Vector3;
  gripper: 'open' | 'close' | 'hold';
  speed?: number;
  blendRadius?: number;
}

export interface Lm3JointTrajectoryPoint {
  waypointId: string;
  q: number[];
  tcpPosition: Vector3;
  timeFromStart: number;
  reachable: boolean;
  error: number;
  directionScore: number;
}

type Axis = 'x' | 'y' | 'z';

interface JointSpec {
  axis: Axis;
  offset: Vector3;
}

const JOINTS: JointSpec[] = [
  { axis: 'y', offset: { x: 0, y: 0.20332999527454376, z: 0 } },
  { axis: 'z', offset: { x: 0, y: 0.012500002980232239, z: -0.08833000063896179 } },
  { axis: 'z', offset: { x: 0, y: 0.2800000011920929, z: 0.04602999985218048 } },
  { axis: 'z', offset: { x: 0, y: 0.25999999046325684, z: -0.07833000272512436 } },
  { axis: 'y', offset: { x: 0, y: 0.029999971389770508, z: 0 } },
  { axis: 'z', offset: { x: 0, y: 0.06832998991012573, z: -0.03793000429868698 } },
];

const GRIPPER_ROOT_OFFSET: Vector3 = { x: 0, y: 0, z: -0.030000001192092896 };
const TCP_OFFSET_IN_GRIPPER: Vector3 = { x: 0, y: -0.09, z: 0 };
const GRIPPER_ROT_X_90 = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, Math.PI / 2);

export function forwardKinematicsLm3(joints: readonly number[]): Lm3Pose {
  let position: Vector3 = { x: 0, y: 0, z: 0 };
  let rotation = identityQuat();

  for (let index = 0; index < JOINTS.length; index++) {
    const spec = JOINTS[index];
    position = add(position, rotateVector(rotation, spec.offset));
    rotation = normalizeQuat(multiplyQuat(rotation, quatFromAxisAngle(axisVector(spec.axis), joints[index] ?? 0)));
  }

  position = add(position, rotateVector(rotation, GRIPPER_ROOT_OFFSET));
  rotation = normalizeQuat(multiplyQuat(rotation, GRIPPER_ROT_X_90));
  position = add(position, rotateVector(rotation, TCP_OFFSET_IN_GRIPPER));

  return {
    position,
    direction: rotateVector(rotation, { x: 0, y: -1, z: 0 }),
  };
}

export function inverseKinematicsLm3(
  target: Vector3,
  initialJoints: readonly number[] = LM3_HOME_JOINTS,
  options: { maxIterations?: number; tolerance?: number; targetDirection?: Vector3 } = {}
): Lm3IkResult {
  if (options.targetDirection && !options.maxIterations) {
    return inverseKinematicsMultiStart(target, initialJoints, options);
  }

  return solveKinematicsLm3(target, initialJoints, options);
}

function inverseKinematicsMultiStart(
  target: Vector3,
  initialJoints: readonly number[],
  options: { maxIterations?: number; tolerance?: number; targetDirection?: Vector3 }
): Lm3IkResult {
  const seeds = createIkSeeds(target, initialJoints);
  let best = solveKinematicsLm3(target, seeds[0], { ...options, maxIterations: 120 });
  let bestScore = ikScore(best.error, best.directionScore, true);

  for (const seed of seeds.slice(1)) {
    const result = solveKinematicsLm3(target, seed, { ...options, maxIterations: 120 });
    const score = ikScore(result.error, result.directionScore, true);
    if (score < bestScore) {
      best = result;
      bestScore = score;
    }
  }

  return best;
}

function solveKinematicsLm3(
  target: Vector3,
  initialJoints: readonly number[] = LM3_HOME_JOINTS,
  options: { maxIterations?: number; tolerance?: number; targetDirection?: Vector3 } = {}
): Lm3IkResult {
  const maxIterations = options.maxIterations ?? 120;
  const tolerance = options.tolerance ?? 0.018;
  const targetDirection = options.targetDirection ? normalize(options.targetDirection) : null;
  let joints = clampJoints([...initialJoints]);
  let bestJoints = [...joints];
  let bestPose = forwardKinematicsLm3(joints);
  let bestError = distance(bestPose.position, target);
  let bestDirectionScore = targetDirection ? directionAlignment(bestPose.direction, targetDirection) : 1;
  let bestScore = ikScore(bestError, bestDirectionScore, Boolean(targetDirection));

  for (let iteration = 0; iteration < maxIterations; iteration++) {
    for (let jointIndex = JOINTS.length - 1; jointIndex >= 0; jointIndex--) {
      const state = computeLm3State(joints);
      const tcp = state.tcp;
      const frame = state.frames[jointIndex];
      const currentVector = projectOnPlane(subtract(tcp, frame.origin), frame.axisWorld);
      const targetVector = projectOnPlane(subtract(target, frame.origin), frame.axisWorld);

      if (norm(currentVector) < 1e-6 || norm(targetVector) < 1e-6) {
        continue;
      }

      const signed = Math.atan2(
        dot(frame.axisWorld, cross(currentVector, targetVector)),
        dot(normalize(currentVector), normalize(targetVector))
      );
      joints[jointIndex] = clamp(
        joints[jointIndex] + clamp(signed, -0.22, 0.22),
        LM3_JOINT_LIMITS[jointIndex].min,
        LM3_JOINT_LIMITS[jointIndex].max
      );
    }

    if (targetDirection) {
      for (let jointIndex = JOINTS.length - 1; jointIndex >= 3; jointIndex--) {
        const state = computeLm3State(joints);
        const frame = state.frames[jointIndex];
        const currentVector = projectOnPlane(state.direction, frame.axisWorld);
        const targetVector = projectOnPlane(targetDirection, frame.axisWorld);

        if (norm(currentVector) < 1e-6 || norm(targetVector) < 1e-6) {
          continue;
        }

        const signed = Math.atan2(
          dot(frame.axisWorld, cross(currentVector, targetVector)),
          dot(normalize(currentVector), normalize(targetVector))
        );
        joints[jointIndex] = clamp(
          joints[jointIndex] + clamp(signed, -0.025, 0.025),
          LM3_JOINT_LIMITS[jointIndex].min,
          LM3_JOINT_LIMITS[jointIndex].max
        );
      }
    }

    const currentPose = forwardKinematicsLm3(joints);
    const currentError = distance(currentPose.position, target);
    const currentDirectionScore = targetDirection ? directionAlignment(currentPose.direction, targetDirection) : 1;
    const currentScore = ikScore(currentError, currentDirectionScore, Boolean(targetDirection));
    if (currentScore < bestScore) {
      bestScore = currentScore;
      bestError = currentError;
      bestDirectionScore = currentDirectionScore;
      bestJoints = [...joints];
    }
    if (currentError <= tolerance && (!targetDirection || currentDirectionScore >= 0.82)) {
      return {
        joints,
        error: currentError,
        directionScore: currentDirectionScore,
        reachable: true,
        iterations: iteration,
      };
    }
  }

  return {
    joints: bestJoints,
    error: bestError,
    directionScore: bestDirectionScore,
    reachable: bestError <= tolerance * (targetDirection ? 4.5 : 1.6) && (!targetDirection || bestDirectionScore >= 0.72),
    iterations: maxIterations,
  };
}

function createIkSeeds(target: Vector3, initialJoints: readonly number[]): number[][] {
  const base = clamp(Math.atan2(target.x, -target.z || 1e-6), LM3_JOINT_LIMITS[0].min, LM3_JOINT_LIMITS[0].max);
  return [
    clampJoints([...initialJoints]),
    clampJoints([...LM3_HOME_JOINTS]),
    clampJoints([base, 0, 0, 0, 0, 0]),
    clampJoints([base, -0.55, 0.9, -0.45, 0, 0]),
    clampJoints([base, 0.55, -0.9, 0.45, 0, 0]),
    clampJoints([base * 0.65, -0.75, 1.05, -0.55, 0.35, 0]),
    clampJoints([base * 0.65, 0.75, -1.05, 0.55, -0.35, 0]),
    clampJoints([base, -1.1, 1.2, -0.8, 0.5, 0]),
    clampJoints([base, 1.1, -1.2, 0.8, -0.5, 0]),
  ];
}

function computeLm3State(joints: readonly number[]): {
  frames: Array<{ origin: Vector3; axisWorld: Vector3 }>;
  tcp: Vector3;
  direction: Vector3;
} {
  let position: Vector3 = { x: 0, y: 0, z: 0 };
  let rotation = identityQuat();
  const frames: Array<{ origin: Vector3; axisWorld: Vector3 }> = [];

  for (let index = 0; index < JOINTS.length; index++) {
    const spec = JOINTS[index];
    position = add(position, rotateVector(rotation, spec.offset));
    frames.push({
      origin: position,
      axisWorld: normalize(rotateVector(rotation, axisVector(spec.axis))),
    });
    rotation = normalizeQuat(multiplyQuat(rotation, quatFromAxisAngle(axisVector(spec.axis), joints[index] ?? 0)));
  }

  position = add(position, rotateVector(rotation, GRIPPER_ROOT_OFFSET));
  rotation = normalizeQuat(multiplyQuat(rotation, GRIPPER_ROT_X_90));
  position = add(position, rotateVector(rotation, TCP_OFFSET_IN_GRIPPER));
  const direction = rotateVector(rotation, { x: 0, y: -1, z: 0 });

  return { frames, tcp: position, direction };
}

export function createLm3JointTrajectory(
  waypoints: Lm3CartesianWaypoint[],
  initialJoints: readonly number[] = LM3_HOME_JOINTS
): Lm3JointTrajectoryPoint[] {
  const points: Lm3JointTrajectoryPoint[] = [];
  let current = [...initialJoints];
  let time = 0;

  for (const waypoint of waypoints) {
    const result = inverseKinematicsLm3(waypoint.position, current, {
      targetDirection: waypoint.targetDirection,
    });
    current = result.joints;
    time += waypoint.gripper === 'hold' ? 1.0 : 0.55;
    points.push({
      waypointId: waypoint.id,
      q: result.joints,
      tcpPosition: forwardKinematicsLm3(result.joints).position,
      timeFromStart: round3(time),
      reachable: result.reachable,
      error: round3(result.error),
      directionScore: round3(result.directionScore),
    });
  }

  return points;
}

function axisVector(axis: Axis): Vector3 {
  if (axis === 'x') return { x: 1, y: 0, z: 0 };
  if (axis === 'y') return { x: 0, y: 1, z: 0 };
  return { x: 0, y: 0, z: 1 };
}

function clampJoints(joints: number[]): number[] {
  return joints.map((joint, index) => clamp(joint, LM3_JOINT_LIMITS[index].min, LM3_JOINT_LIMITS[index].max));
}

function identityQuat(): [number, number, number, number] {
  return [0, 0, 0, 1];
}

function quatFromAxisAngle(axis: Vector3, angle: number): [number, number, number, number] {
  const half = angle / 2;
  const s = Math.sin(half);
  return [axis.x * s, axis.y * s, axis.z * s, Math.cos(half)];
}

function multiplyQuat(a: readonly number[], b: readonly number[]): [number, number, number, number] {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

function normalizeQuat(q: readonly number[]): [number, number, number, number] {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function rotateVector(q: readonly number[], v: Vector3): Vector3 {
  const vectorQuat: [number, number, number, number] = [v.x, v.y, v.z, 0];
  const conjugate: [number, number, number, number] = [-q[0], -q[1], -q[2], q[3]];
  const rotated = multiplyQuat(multiplyQuat(q, vectorQuat), conjugate);
  return { x: rotated[0], y: rotated[1], z: rotated[2] };
}

function add(a: Vector3, b: Vector3): Vector3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

function subtract(a: Vector3, b: Vector3): Vector3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function scale(v: Vector3, factor: number): Vector3 {
  return { x: v.x * factor, y: v.y * factor, z: v.z * factor };
}

function dot(a: Vector3, b: Vector3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function cross(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}

function norm(v: Vector3): number {
  return Math.hypot(v.x, v.y, v.z);
}

function normalize(v: Vector3): Vector3 {
  const n = norm(v) || 1;
  return { x: v.x / n, y: v.y / n, z: v.z / n };
}

function projectOnPlane(v: Vector3, normal: Vector3): Vector3 {
  return subtract(v, scale(normal, dot(v, normal)));
}

function distance(a: Vector3, b: Vector3): number {
  return norm(subtract(a, b));
}

function directionAlignment(a: Vector3, b: Vector3): number {
  return clamp(dot(normalize(a), normalize(b)), -1, 1);
}

function ikScore(positionError: number, directionScore: number, hasDirectionTarget: boolean): number {
  if (!hasDirectionTarget) {
    return positionError;
  }
  return positionError + Math.max(0, 1 - directionScore) * 0.025;
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function round3(value: number): number {
  return Math.round(value * 1000) / 1000;
}
