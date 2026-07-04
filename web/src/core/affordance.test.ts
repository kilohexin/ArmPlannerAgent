import { describe, expect, it } from 'vitest';
import { generateAffordances } from './affordance';
import type { SceneObject } from './types';

describe('generateAffordances', () => {
  it('creates side grasp and top place candidates for a cube', () => {
    const cube: SceneObject = {
      id: 'red_cube',
      label: 'red cube',
      type: 'cube',
      color: 'red',
      position: { x: 0.3, y: 0.04, z: 0.1 },
      size: { x: 0.06, y: 0.06, z: 0.06 },
      movable: true
    };

    const candidates = generateAffordances(cube);

    expect(candidates.map((item) => item.kind)).toContain('side_grasp');
    expect(candidates.map((item) => item.kind)).toContain('top_place');
    expect(candidates.find((item) => item.kind === 'top_place')?.pose.y).toBeCloseTo(0.07);
  });

  it('side grasp score matches the parallel gripper rating', () => {
    const cube: SceneObject = {
      id: 'red_cube',
      label: 'red cube',
      type: 'cube',
      color: 'red',
      position: { x: 0.3, y: 0.04, z: 0.1 },
      size: { x: 0.06, y: 0.06, z: 0.06 },
      movable: true
    };

    const candidates = generateAffordances(cube);
    const sideGrasp = candidates.find((item) => item.kind === 'side_grasp');
    expect(sideGrasp?.score).toBe(0.93);
  });
});
