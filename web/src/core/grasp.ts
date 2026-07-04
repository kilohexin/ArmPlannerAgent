import { LM3_PROFILE } from './robotProfile';
import { DEFAULT_GRIPPER_TCP_CALIBRATION, getTcpCalibration } from './tcpCalibration';
import type { GraspMode, GraspSpec, SceneObject, Vector3 } from './types';

export const GRIPPER_TOOL_OFFSET = 0;
export const GRIPPER_TCP_CALIBRATION: Vector3 = DEFAULT_GRIPPER_TCP_CALIBRATION;
const GRASP_CENTER_TOLERANCE = 0.025;

export interface GraspValidationResult {
  ok: boolean;
  reason: string;
}

export function inferGraspMode(instruction: string): GraspMode {
  void instruction;
  return 'top';
}

export function createGraspSpec(
  object: SceneObject,
  mode: GraspMode,
  robotBase: Vector3
): GraspSpec {
  void mode;
  void robotBase;
  const approach = { x: 0, y: 1, z: 0 };
  const closingAxis = chooseTopClosingAxis(object);
  const toolOffset = add(scale(approach, GRIPPER_TOOL_OFFSET), getTcpCalibration());

  return {
    mode: 'top',
    objectId: object.id,
    center: { ...object.position },
    approach,
    closingAxis,
    toolOffset,
    requiredOpeningMm: requiredOpeningMm(object, closingAxis),
    maxOpeningMm: LM3_PROFILE.gripperStrokeMm,
  };
}

export function toolTargetFromGraspCenter(center: Vector3, grasp: GraspSpec): Vector3 {
  return add(center, grasp.toolOffset);
}

export function graspCenterFromToolTarget(toolTarget: Vector3, grasp: GraspSpec): Vector3 {
  return subtract(toolTarget, grasp.toolOffset);
}

export function preGraspCenter(grasp: GraspSpec): Vector3 {
  return add(grasp.center, scale(grasp.approach, 0.12));
}

export function validateGraspCenter(
  object: SceneObject,
  center: Vector3,
  grasp: GraspSpec
): GraspValidationResult {
  if (grasp.requiredOpeningMm > grasp.maxOpeningMm) {
    return {
      ok: false,
      reason: `目标宽度 ${grasp.requiredOpeningMm.toFixed(0)}mm 超过夹爪最大开口 ${grasp.maxOpeningMm.toFixed(0)}mm。`,
    };
  }

  const centerError = distance(center, object.position);
  if (centerError > GRASP_CENTER_TOLERANCE) {
    return {
      ok: false,
      reason: `夹爪中心偏离目标中心 ${(centerError * 1000).toFixed(0)}mm，不能可靠抓取。`,
    };
  }

  if (grasp.approach.y < 0.75) {
    return {
      ok: false,
      reason: '顶部抓取要求夹爪从物体上方向下接近。',
    };
  }

  return { ok: true, reason: '抓取姿态校验通过。' };
}

function requiredOpeningMm(object: SceneObject, closingAxis: Vector3): number {
  const width =
    Math.abs(closingAxis.x) * object.size.x +
    Math.abs(closingAxis.y) * object.size.y +
    Math.abs(closingAxis.z) * object.size.z;
  return width * 1000;
}

function chooseTopClosingAxis(object: SceneObject): Vector3 {
  return object.size.x <= object.size.z ? { x: 1, y: 0, z: 0 } : { x: 0, y: 0, z: 1 };
}

function add(a: Vector3, b: Vector3): Vector3 {
  return { x: round3(a.x + b.x), y: round3(a.y + b.y), z: round3(a.z + b.z) };
}

function subtract(a: Vector3, b: Vector3): Vector3 {
  return { x: round3(a.x - b.x), y: round3(a.y - b.y), z: round3(a.z - b.z) };
}

function scale(vector: Vector3, scalar: number): Vector3 {
  return {
    x: round3(vector.x * scalar),
    y: round3(vector.y * scalar),
    z: round3(vector.z * scalar),
  };
}

function distance(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function round3(value: number): number {
  return Math.round(value * 1000) / 1000;
}
