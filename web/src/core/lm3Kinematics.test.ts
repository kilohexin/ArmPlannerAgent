import { describe, expect, it } from 'vitest';
import { createLm3JointTrajectory, forwardKinematicsLm3, inverseKinematicsLm3, LM3_HOME_JOINTS } from './lm3Kinematics';
import type { Vector3 } from './types';

function distance(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function dot(a: Vector3, b: Vector3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function jointDistance(a: number[], b: number[]): number {
  return Math.sqrt(a.reduce((sum, angle, index) => {
    const delta = angle - b[index];
    return sum + delta * delta;
  }, 0));
}

describe('LM3 kinematics', () => {
  it('computes a stable home-pose TCP from the GLB joint chain', () => {
    const pose = forwardKinematicsLm3(LM3_HOME_JOINTS);

    expect(Math.abs(pose.position.x)).toBeLessThan(0.005);
    expect(pose.position.y).toBeGreaterThan(0.8);
    expect(pose.position.z).toBeLessThan(-0.2);
  });

  it('solves a nearby Cartesian target with numerical IK', () => {
    const start = [...LM3_HOME_JOINTS];
    const startPose = forwardKinematicsLm3(start);
    const target = {
      x: startPose.position.x + 0.035,
      y: startPose.position.y,
      z: startPose.position.z + 0.025,
    };

    const result = inverseKinematicsLm3(target, start);
    const solvedPose = forwardKinematicsLm3(result.joints);

    expect(result.reachable).toBe(true);
    expect(distance(solvedPose.position, target)).toBeLessThan(0.025);
  });

  it('keeps the TCP pointing downward for top grasp waypoints', () => {
    const target = { x: 0.28, y: 0.12, z: 0.16 };
    const result = inverseKinematicsLm3(target, LM3_HOME_JOINTS, {
      targetDirection: { x: 0, y: -1, z: 0 },
    });
    const solvedPose = forwardKinematicsLm3(result.joints);

    expect(result.reachable).toBe(true);
    expect(distance(solvedPose.position, target)).toBeLessThan(0.07);
    expect(dot(solvedPose.direction, { x: 0, y: -1, z: 0 })).toBeGreaterThan(0.72);
  });

  it('prefers a more vertical wrist configuration for close top grasps', () => {
    const target = { x: 0.28, y: 0.12, z: 0.16 };
    const result = inverseKinematicsLm3(target, LM3_HOME_JOINTS, {
      targetDirection: { x: 0, y: -1, z: 0 },
    });

    expect(result.reachable).toBe(true);
    expect(result.error).toBeLessThan(0.07);
    expect(result.directionScore).toBeGreaterThan(0.86);
  });

  it('keeps adjacent execution targets on the same joint branch', () => {
    const first = inverseKinematicsLm3({ x: 0.28, y: 0.12, z: 0.16 }, LM3_HOME_JOINTS, {
      maxIterations: 120,
      targetDirection: { x: 0, y: -1, z: 0 },
    });
    const second = inverseKinematicsLm3({ x: 0.282, y: 0.12, z: 0.16 }, first.joints, {
      maxIterations: 120,
      targetDirection: { x: 0, y: -1, z: 0 },
    });

    expect(first.reachable).toBe(true);
    expect(second.reachable).toBe(true);
    expect(jointDistance(first.joints, second.joints)).toBeLessThan(0.2);
  });

  it('creates one joint trajectory point per Cartesian waypoint', () => {
    const waypoints = [
      { id: 'a', position: { x: 0.05, y: 0.72, z: -0.1 }, gripper: 'open' as const },
      { id: 'b', position: { x: 0.08, y: 0.68, z: -0.08 }, gripper: 'close' as const },
    ];

    const trajectory = createLm3JointTrajectory(waypoints, LM3_HOME_JOINTS);

    expect(trajectory).toHaveLength(2);
    expect(trajectory[0].waypointId).toBe('a');
    expect(trajectory[1].q).toHaveLength(6);
    expect(trajectory[1].timeFromStart).toBeGreaterThan(trajectory[0].timeFromStart);
  });
});
