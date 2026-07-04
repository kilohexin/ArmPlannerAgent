import { describe, expect, it } from 'vitest';
import { createLocalPlan, validatePlan } from './planner';
import type { ManipulationPlan, SceneState } from './types';

const testScene: SceneState = {
  robot: {
    base: { x: 0, y: 0, z: 0 },
    maxReach: 0.85,
    holding: null,
  },
  objects: [
    {
      id: 'red_cube',
      label: 'red cube',
      type: 'cube',
      color: 'red',
      position: { x: 0.28, y: 0.03, z: 0.16 },
      size: { x: 0.06, y: 0.06, z: 0.06 },
      movable: true,
    },
    {
      id: 'blue_cube',
      label: 'blue cube',
      type: 'cube',
      color: 'blue',
      position: { x: 0.46, y: 0.03, z: -0.08 },
      size: { x: 0.06, y: 0.06, z: 0.06 },
      movable: true,
    },
    {
      id: 'green_cube',
      label: 'green cube',
      type: 'cube',
      color: 'green',
      position: { x: 0.18, y: 0.03, z: -0.22 },
      size: { x: 0.06, y: 0.06, z: 0.06 },
      movable: true,
    },
  ],
  history: [],
};

describe('createLocalPlan', () => {
  it('generates a plan for "put red cube on blue cube"', () => {
    const plan = createLocalPlan(testScene, 'put red cube on blue cube');

    expect(plan.status).toBe('ready');
    expect(plan.source).toBe('local');
    expect(plan.targetObjectId).toBe('red_cube');
    expect(plan.destinationObjectId).toBe('blue_cube');
    expect(plan.steps).toHaveLength(8);
    expect(plan.trajectory).toHaveLength(7);
    expect(plan.executionMode).toBe('sim');
    expect(plan.robotModel?.id).toBe('lebai-lm3');
    expect(plan.cartesianWaypoints).toHaveLength(7);
    expect(plan.cartesianWaypoints![0].id).toBe('start_tcp');
    expect(plan.cartesianWaypoints![0].frame).toBe('tcp');
    expect(plan.jointTrajectory).toHaveLength(7);
    expect(plan.jointTrajectory![0].q).toHaveLength(6);
  });

  it('generates a plan for a real Chinese instruction', () => {
    const plan = createLocalPlan(testScene, '把红色方块放到蓝色方块上');

    expect(plan.status).toBe('ready');
    expect(plan.targetObjectId).toBe('red_cube');
    expect(plan.destinationObjectId).toBe('blue_cube');
  });

  it('returns needs_repair for unrecognized objects', () => {
    const plan = createLocalPlan(testScene, 'pick up the invisible widget');

    expect(plan.status).toBe('needs_repair');
  });

  it('generates a pick-only plan that keeps holding the object', () => {
    const plan = createLocalPlan(testScene, 'pick up red cube');

    expect(plan.status).toBe('ready');
    expect(plan.taskType).toBe('pick_only');
    expect(plan.targetObjectId).toBe('red_cube');
    expect(plan.destinationObjectId).toBeNull();
    expect(plan.trajectory).toHaveLength(4);
    expect(plan.cartesianWaypoints?.[0].id).toBe('start_tcp');
    expect(plan.steps.some((step) => step.action === 'grasp')).toBe(true);
    expect(plan.steps.some((step) => step.action === 'release')).toBe(false);
  });

  it('places the currently held object to a relative right-side position', () => {
    const holdingScene: SceneState = {
      ...testScene,
      robot: { ...testScene.robot, holding: 'red_cube' },
    };

    const plan = createLocalPlan(holdingScene, 'move it right a little');

    expect(plan.status).toBe('ready');
    expect(plan.taskType).toBe('relative_place');
    expect(plan.targetObjectId).toBe('red_cube');
    expect(plan.steps.some((step) => step.action === 'grasp')).toBe(false);
    expect(plan.steps.some((step) => step.action === 'release')).toBe(true);
    expect(plan.steps.find((step) => step.action === 'release')?.objectId).toBeUndefined();
    expect(plan.trajectory.at(-2)?.x).toBeGreaterThan(testScene.objects[0].position.x);
    expect(plan.trajectory.at(-2)?.y).toBeCloseTo(testScene.objects[0].size.y / 2);
  });

  it('rejects picking a different object while already holding one', () => {
    const holdingScene: SceneState = {
      ...testScene,
      robot: { ...testScene.robot, holding: 'red_cube' },
    };

    const plan = createLocalPlan(holdingScene, 'pick up blue cube');

    expect(plan.status).toBe('needs_repair');
    expect(plan.repairHint).toContain('red_cube');
  });
});

describe('validatePlan', () => {
  it('accepts a valid plan', () => {
    const plan = createLocalPlan(testScene, 'put red cube on blue cube');
    const result = validatePlan(testScene, plan);

    expect(result.ok).toBe(true);
  });

  it('accepts a pick-only plan without requiring release', () => {
    const plan = createLocalPlan(testScene, 'pick up red cube');
    const result = validatePlan(testScene, plan);

    expect(result.ok).toBe(true);
  });

  it('allows mild low-grasp IK residuals from the course-level simulator', () => {
    const plan = createLocalPlan(testScene, 'put red cube on blue cube');
    const graspIndex = plan.jointTrajectory!.findIndex((point) => point.waypointId === 'grasp');
    plan.jointTrajectory![graspIndex] = {
      ...plan.jointTrajectory![graspIndex],
      waypointId: 'grasp',
      reachable: false,
      error: 0.055,
    };

    const result = validatePlan(testScene, plan);

    expect(result.ok).toBe(true);
  });

  it('rejects severe IK residuals', () => {
    const plan = createLocalPlan(testScene, 'put red cube on blue cube');
    const graspIndex = plan.jointTrajectory!.findIndex((point) => point.waypointId === 'grasp');
    plan.jointTrajectory![graspIndex] = {
      ...plan.jointTrajectory![graspIndex],
      waypointId: 'grasp',
      reachable: false,
      error: 0.12,
    };

    const result = validatePlan(testScene, plan);

    expect(result.ok).toBe(false);
    expect(result.issues.some((issue) => issue.code === 'target_unreachable')).toBe(true);
  });

  it('rejects plan with unreachable target', () => {
    const farScene: SceneState = {
      ...testScene,
      objects: [
        ...testScene.objects.slice(1),
        {
          id: 'far_cube',
          label: 'far cube',
          type: 'cube',
          color: 'red',
          position: { x: 2.0, y: 0.03, z: 0 },
          size: { x: 0.06, y: 0.06, z: 0.06 },
          movable: true,
        },
      ],
    };
    const plan = createLocalPlan(farScene, 'put far cube on blue cube');
    const result = validatePlan(farScene, plan);

    expect(result.ok).toBe(false);
  });

  it('rejects an LLM plan that has motion points but no grasp and release actions', () => {
    const plan: ManipulationPlan = {
      id: 'bad-llm-plan',
      status: 'ready',
      taskType: 'pick_place',
      instruction: 'put red cube on blue cube',
      targetObjectId: null,
      destinationObjectId: null,
      steps: [
        {
          action: 'move_to',
          objectId: 'red_cube',
          pose: { x: 0.28, y: 0.35, z: 0.16 },
          description: 'move above red cube',
        },
      ],
      trajectory: [{ x: 0.28, y: 0.35, z: 0.16 }],
      score: { total: 100, reachability: 1, smoothness: 1, collision: 1, jointLimits: 'OK' },
      repairHint: null,
      source: 'llm',
      confidence: 0.8,
    };

    const result = validatePlan(testScene, plan);

    expect(result.ok).toBe(false);
    expect(result.issues.some((issue) => issue.code === 'empty_plan')).toBe(true);
  });
});
