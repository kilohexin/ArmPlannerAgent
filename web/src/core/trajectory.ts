import { LM3_PROFILE } from './robotProfile';
import { preGraspCenter } from './grasp';
import type { GraspSpec, SceneObject, TrajectoryScore, Vector3 } from './types';

export const GRIPPER_HOLD_OFFSET = 0.09;

export function distanceXZ(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.z - b.z);
}

export function distance3(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function heldObjectCenter(endEffector: Vector3, grasp?: GraspSpec): Vector3 {
  if (grasp) {
    return { ...endEffector };
  }

  return {
    x: round3(endEffector.x),
    y: round3(endEffector.y - GRIPPER_HOLD_OFFSET),
    z: round3(endEffector.z),
  };
}

export function stackedCenter(target: SceneObject, destination: SceneObject): Vector3 {
  return {
    x: round3(destination.position.x),
    y: round3(destination.position.y + destination.size.y / 2 + target.size.y / 2),
    z: round3(destination.position.z),
  };
}

/**
 * Generate pick-and-place trajectory waypoints.
 * Each waypoint is an end-effector target (x, y, z).
 * The IK solver converts these to joint angles during execution.
 */
export function createPickPlaceTrajectory(
  target: SceneObject,
  destination: SceneObject,
  grasp?: GraspSpec
): Vector3[] {
  const graspCenter = grasp?.center ?? {
    x: target.position.x,
    y: target.position.y,
    z: target.position.z,
  };
  const preGrasp = grasp ? preGraspCenter(grasp) : { x: target.position.x, y: LM3_PROFILE.safeZ, z: target.position.z };
  const lift = { x: graspCenter.x, y: LM3_PROFILE.safeZ, z: graspCenter.z };
  const placeCenter = stackedCenter(target, destination);

  return [
    // 0: Pre-grasp center, near the object but not touching it.
    preGrasp,
    // 1: Grasp center, the center between the gripper fingers.
    graspCenter,
    // 2: Lift held object to safe height.
    lift,
    // 3: Pre-place above destination.
    { x: placeCenter.x, y: LM3_PROFILE.safeZ, z: placeCenter.z },
    // 4: Place center on the destination top surface.
    placeCenter,
    // 5: Retreat after release.
    { x: placeCenter.x, y: LM3_PROFILE.safeZ, z: placeCenter.z },
  ];
}

/**
 * Score trajectory quality for the course-level LM3 simulator.
 */
export function scoreTrajectory(
  points: Vector3[],
  maxReach: number,
  base: Vector3
): TrajectoryScore {
  const maxDistance = Math.max(...points.map((point) => distanceXZ(point, base)), 0);
  const reachability = clamp01(1 - Math.max(0, maxDistance - maxReach) / Math.max(maxReach, 0.01));
  const length = points.slice(1).reduce((total, point, index) => total + distance3(points[index], point), 0);
  const smoothness = clamp01(1 - Math.max(0, length - 1.5) / 1.5);
  const collision = points.every((point) => point.y >= 0.015) ? 1 : 0.4;

  return {
    total: Math.round((reachability * 0.45 + smoothness * 0.25 + collision * 0.25 + 0.05) * 100),
    reachability: round2(reachability),
    smoothness: round2(smoothness),
    collision: round2(collision),
    jointLimits: maxDistance <= maxReach ? 'OK' : 'WARN',
  };
}

function clamp01(value: number): number {
  return Math.min(1, Math.max(0, value));
}

function round2(value: number): number {
  return Math.round(value * 100) / 100;
}

function round3(value: number): number {
  return Math.round(value * 1000) / 1000;
}
