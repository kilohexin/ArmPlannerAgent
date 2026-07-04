import { describe, expect, it } from 'vitest';
import {
  createGraspSpec,
  GRIPPER_TCP_CALIBRATION,
  GRIPPER_TOOL_OFFSET,
  inferGraspMode,
  toolTargetFromGraspCenter,
} from './grasp';
import type { SceneObject } from './types';

const cube: SceneObject = {
  id: 'red_cube',
  label: 'red cube',
  type: 'cube',
  color: 'red',
  position: { x: 0.28, y: 0.03, z: 0.16 },
  size: { x: 0.06, y: 0.06, z: 0.06 },
  movable: true,
};

describe('grasp geometry', () => {
  it('always uses top grasp to keep the course simulator predictable', () => {
    expect(inferGraspMode('put red cube on blue cube')).toBe('top');
    expect(inferGraspMode('侧向抓取红色方块')).toBe('top');
  });

  it('uses top grasp when the instruction explicitly asks for it', () => {
    expect(inferGraspMode('从顶部抓取红色方块')).toBe('top');
    expect(inferGraspMode('top grasp the red cube')).toBe('top');
  });

  it('keeps the top-grasp tool above the gripper center', () => {
    const grasp = createGraspSpec(cube, 'top', { x: 0, y: 0, z: 0 });
    const toolTarget = toolTargetFromGraspCenter(grasp.center, grasp);

    expect(grasp.center).toEqual(cube.position);
    expect(toolTarget.x).toBeCloseTo(grasp.center.x + GRIPPER_TCP_CALIBRATION.x);
    expect(toolTarget.y).toBeCloseTo(grasp.center.y + GRIPPER_TOOL_OFFSET + GRIPPER_TCP_CALIBRATION.y);
    expect(toolTarget.z).toBeCloseTo(grasp.center.z + GRIPPER_TCP_CALIBRATION.z);
  });
});
