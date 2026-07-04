import { describe, expect, it } from 'vitest';
import { heldObjectCenter, stackedCenter } from './trajectory';
import type { SceneObject } from './types';

const cube: SceneObject = {
  id: 'red_cube',
  label: 'red cube',
  type: 'cube',
  color: 'red',
  position: { x: 0.28, y: 0.03, z: 0.16 },
  size: { x: 0.06, y: 0.06, z: 0.06 },
  movable: true
};

describe('object placement helpers', () => {
  it('keeps a held object below the end-effector at the configured gripper offset', () => {
    expect(heldObjectCenter({ x: 0.2, y: 0.4, z: -0.1 })).toEqual({
      x: 0.2,
      y: 0.31,
      z: -0.1
    });
  });

  it('places the released object on top of the destination without penetration', () => {
    const destination: SceneObject = {
      ...cube,
      id: 'blue_cube',
      position: { x: 0.46, y: 0.03, z: -0.08 }
    };

    expect(stackedCenter(cube, destination)).toEqual({
      x: 0.46,
      y: 0.09,
      z: -0.08
    });
  });
});
