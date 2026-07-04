import type { RobotProfile, RobotState } from './types';

export const LM3_PROFILE: RobotProfile = {
  id: 'lebai-lm3-lmg90',
  name: 'Lebai LM3',
  gripperName: 'LMG-90 / generic parallel gripper',
  maxReach: 0.85,
  payloadKg: 3,
  gripperStrokeMm: 90,
  safeZ: 0.35,
  tableHeight: 0,
};

export const LM3_JOINT_NAMES = {
  joint1: 'Joint1',
  joint2: 'Joint2',
  joint3: 'Joint3',
  joint4: 'Joint4',
  joint5: 'Joint5',
  joint6: 'Joint6',
} as const;

export function createDefaultRobotState(): RobotState {
  return {
    base: { x: 0, y: 0, z: 0 },
    maxReach: LM3_PROFILE.maxReach,
    holding: null,
    joints: [0, 0, 0, 0, 0, 0],
  };
}
