import { createLm3JointTrajectory, forwardKinematicsLm3, LM3_HOME_JOINTS } from './lm3Kinematics';
import { createGraspSpec, graspCenterFromToolTarget, inferGraspMode, toolTargetFromGraspCenter } from './grasp';
import { findObjectByReference } from './sceneGraph';
import { getTcpCalibration } from './tcpCalibration';
import { createPickPlaceTrajectory, distanceXZ, scoreTrajectory, stackedCenter } from './trajectory';
import type {
  CartesianWaypoint,
  GraspSpec,
  LLMPlanStep,
  ManipulationPlan,
  PlanStep,
  PlanTaskType,
  SceneObject,
  SceneState,
  ValidationIssue,
  ValidationResult,
  Vector3,
} from './types';

const LM3_MODEL = {
  id: 'lebai-lm3' as const,
  source: 'glb' as const,
  tcpOffset: { x: 0, y: -0.09, z: 0 },
};

const COURSE_IK_BLOCKING_ERROR = 0.09;
const SAFE_Y = 0.28;
const RELATIVE_STEP = 0.1;

export function createPlanFromLLM(
  scene: SceneState,
  instruction: string,
  llmSteps: LLMPlanStep[],
  confidence: number
): ManipulationPlan {
  const steps: PlanStep[] = llmSteps.map((step, index) => ({
    action: step.action,
    objectId: step.objectId,
    pose: step.targetPosition,
    description: step.description ?? `步骤 ${index + 1}: ${step.action} ${step.objectId}`,
  }));

  const targetObjectId =
    llmSteps.find((step) => step.gripperAction === 'close')?.objectId ??
    llmSteps.find((step) => step.action === 'grasp')?.objectId ??
    null;
  const destinationObjectId =
    llmSteps.find((step) => step.gripperAction === 'open')?.objectId ??
    llmSteps.find((step) => step.action === 'release')?.objectId ??
    null;
  const target = targetObjectId ? scene.objects.find((object) => object.id === targetObjectId) : null;
  const grasp = target ? createGraspSpec(target, inferGraspMode(instruction), scene.robot.base) : undefined;
  const motionTrajectory = steps.filter((step) => step.pose).map((step) => step.pose!);
  const trajectory = withStartTcp(scene, motionTrajectory, grasp);
  const cartesianWaypoints = createCartesianWaypointsFromSteps(scene, steps, grasp);
  const jointTrajectory = createLm3JointTrajectory(
    cartesianWaypoints.map((waypoint) => toLm3Waypoint(waypoint, grasp)),
    initialJoints(scene)
  );
  const score = trajectory.length > 0
    ? scoreTrajectory(trajectory, scene.robot.maxReach, scene.robot.base)
    : { total: 0, reachability: 0, smoothness: 0, collision: 0, jointLimits: 'OK' as const };

  return {
    id: `plan-${Date.now()}`,
    status: 'ready',
    taskType: 'pick_place',
    instruction,
    targetObjectId,
    destinationObjectId,
    steps,
    trajectory,
    cartesianWaypoints,
    jointTrajectory,
    executionMode: 'sim',
    robotModel: LM3_MODEL,
    grasp,
    score,
    repairHint: null,
    source: 'llm',
    confidence,
  };
}

export function validateLLMPlan(scene: SceneState, plan: ManipulationPlan): ValidationResult {
  const issues: ValidationIssue[] = [];

  if (!plan.steps.length) {
    issues.push({ code: 'empty_plan', message: '规划结果为空，没有可执行步骤。' });
    return { ok: false, issues, repairHint: '未生成可执行步骤，请换一个更具体的任务描述。' };
  }

  const taskType = plan.taskType ?? 'pick_place';
  const hasGrasp = plan.steps.some((step) => step.action === 'grasp');
  const hasRelease = plan.steps.some((step) => step.action === 'release');
  const isHeldTarget = Boolean(plan.targetObjectId && scene.robot.holding === plan.targetObjectId);
  const hasRequiredActions =
    taskType === 'pick_only'
      ? hasGrasp && Boolean(plan.targetObjectId)
      : taskType === 'relative_place'
        ? hasRelease && Boolean(plan.targetObjectId)
        : hasRelease && Boolean(plan.targetObjectId) && Boolean(plan.destinationObjectId) && (hasGrasp || isHeldTarget);

  if (!hasRequiredActions) {
    issues.push({
      code: 'empty_plan',
      message: '规划缺少当前任务类型所需的抓取或释放动作。',
    });
  }

  const objectIds = new Set(scene.objects.map((object) => object.id));
  for (const step of plan.steps) {
    if (step.objectId && !objectIds.has(step.objectId)) {
      issues.push({
        code: 'target_missing',
        message: `规划引用了不存在的物体：${step.objectId}`,
        objectId: step.objectId,
      });
    }
  }

  for (const step of plan.steps) {
    if (step.pose && distanceXZ(step.pose, scene.robot.base) > scene.robot.maxReach) {
      issues.push({
        code: 'target_unreachable',
        message: `目标点 (${step.pose.x.toFixed(2)}, ${step.pose.y.toFixed(2)}, ${step.pose.z.toFixed(2)}) 超出机械臂 ${scene.robot.maxReach}m 工作半径。`,
        objectId: step.objectId,
      });
    }
  }

  const target = plan.targetObjectId
    ? scene.objects.find((object) => object.id === plan.targetObjectId)
    : null;
  if (target && !target.movable) {
    issues.push({
      code: 'object_not_movable',
      message: `${target.id} 被标记为不可移动。`,
      objectId: target.id,
    });
  }

  for (const step of plan.steps) {
    if (step.pose && step.pose.y < 0.01) {
      issues.push({
        code: 'destination_unreachable',
        message: `轨迹点高度 y=${step.pose.y.toFixed(2)} 低于桌面安全高度。`,
      });
    }
  }

  const unreachableJoint = plan.jointTrajectory?.find((point) => (
    !point.reachable &&
    (!Number.isFinite(point.error) || point.error > COURSE_IK_BLOCKING_ERROR)
  ));
  if (unreachableJoint) {
    issues.push({
      code: 'target_unreachable',
      message: `LM3 数值 IK 未能稳定到达 ${unreachableJoint.waypointId}，末端误差 ${unreachableJoint.error.toFixed(3)}m。`,
    });
  }

  const graspIndex = plan.steps.findIndex((step) => step.action === 'grasp');
  const releaseIndex = plan.steps.findIndex((step) => step.action === 'release');
  if (graspIndex >= 0 && releaseIndex >= 0 && releaseIndex <= graspIndex) {
    issues.push({
      code: 'empty_plan',
      message: '释放动作必须发生在抓取动作之后。',
    });
  }

  const MAX_STEPS = 30;
  if (plan.steps.length > MAX_STEPS) {
    issues.push({
      code: 'empty_plan',
      message: `规划步骤过多：${plan.steps.length}，最多允许 ${MAX_STEPS} 步。`,
    });
  }

  const ok = issues.length === 0;
  return {
    ok,
    issues,
    repairHint: ok ? '规划通过安全校验。' : issues[0].message,
  };
}

export function validatePlan(scene: SceneState, plan: ManipulationPlan): ValidationResult {
  return validateLLMPlan(scene, plan);
}

export function createLocalPlan(scene: SceneState, instruction: string): ManipulationPlan {
  const taskType = detectTaskType(instruction);
  const heldObject = scene.robot.holding
    ? scene.objects.find((object) => object.id === scene.robot.holding) ?? null
    : null;
  const explicitTarget = findObjectByReference(scene, instruction);
  const target = referencesHeldObject(instruction) && heldObject
    ? heldObject
    : explicitTarget;

  if (heldObject && taskType === 'pick_only' && target && target.id !== heldObject.id) {
    return createRepairPlan(
      instruction,
      'pick_only',
      `当前已夹持 ${heldObject.id}，请先放下后再抓取 ${target.id}。`,
      target.id,
      null
    );
  }

  if (taskType === 'pick_only') {
    if (!target) {
      return createRepairPlan(instruction, taskType);
    }
    return createPickOnlyPlan(scene, instruction, target);
  }

  if (taskType === 'relative_place') {
    const relativeTarget = target ?? heldObject;
    if (!relativeTarget) {
      return createRepairPlan(instruction, taskType);
    }
    return createRelativePlacePlan(scene, instruction, relativeTarget, Boolean(heldObject?.id === relativeTarget.id));
  }

  const pickPlaceTarget = target ?? heldObject;
  const destination = pickPlaceTarget
    ? findObjectByReference(scene, instruction, { excludeIds: [pickPlaceTarget.id], preferDestination: true })
    : null;

  if (!pickPlaceTarget || !destination) {
    return createRepairPlan(instruction, 'pick_place', undefined, pickPlaceTarget?.id ?? null, destination?.id ?? null);
  }

  if (heldObject?.id === pickPlaceTarget.id) {
    return createHeldPlacePlan(scene, instruction, pickPlaceTarget, destination, 'pick_place');
  }

  return createPickPlacePlanInternal(scene, instruction, pickPlaceTarget, destination);
}

function createPickPlacePlanInternal(
  scene: SceneState,
  instruction: string,
  target: SceneObject,
  destination: SceneObject
): ManipulationPlan {
  const grasp = createGraspSpec(target, inferGraspMode(instruction), scene.robot.base);
  const motionTrajectory = createPickPlaceTrajectory(target, destination, grasp);
  const trajectory = withStartTcp(scene, motionTrajectory, grasp);
  const cartesianWaypoints = createPickPlaceWaypoints(scene, trajectory);
  const jointTrajectory = createLm3JointTrajectory(
    cartesianWaypoints.map((waypoint) => toLm3Waypoint(waypoint, grasp)),
    initialJoints(scene)
  );
  const score = scoreTrajectory(trajectory, scene.robot.maxReach, scene.robot.base);
  const steps: PlanStep[] = [
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[0], graspMode: grasp.mode, description: `移动到 ${target.id} 上方，并旋转腕部使夹爪朝下。` },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[1], graspMode: grasp.mode, description: `保持夹爪开口，对准 ${target.id} 两侧并下降。` },
    { action: 'grasp', objectId: target.id, graspMode: grasp.mode, description: '夹爪闭合，贴合方块两侧完成顶部抓取。' },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[2], graspMode: grasp.mode, description: '保持夹爪中心并抬升到安全高度。' },
    { action: 'move_to', objectId: destination.id, pose: motionTrajectory[3], graspMode: grasp.mode, description: `移动到 ${destination.id} 上方。` },
    { action: 'move_to', objectId: destination.id, pose: motionTrajectory[4], graspMode: grasp.mode, description: `下降到 ${destination.id} 的放置中心。` },
    { action: 'release', objectId: destination.id, graspMode: grasp.mode, description: '夹爪张开，释放物体。' },
    { action: 'retreat', pose: motionTrajectory[5], graspMode: grasp.mode, description: '退回安全高度。' },
  ];

  return createReadyPlan(scene, instruction, 'pick_place', target, destination, steps, trajectory, grasp, cartesianWaypoints, jointTrajectory, score);
}

function createPickOnlyPlan(scene: SceneState, instruction: string, target: SceneObject): ManipulationPlan {
  const grasp = createGraspSpec(target, inferGraspMode(instruction), scene.robot.base);
  const motionTrajectory = [preGraspFromGrasp(grasp), grasp.center, { x: grasp.center.x, y: SAFE_Y, z: grasp.center.z }];
  const trajectory = withStartTcp(scene, motionTrajectory, grasp);
  const cartesianWaypoints = createCartesianWaypoints(
    trajectory,
    ['start_tcp', 'pre_grasp', 'grasp', 'lift'],
    ['open', 'open', 'open', 'close']
  );
  const jointTrajectory = createLm3JointTrajectory(
    cartesianWaypoints.map((waypoint) => toLm3Waypoint(waypoint, grasp)),
    initialJoints(scene)
  );
  const score = scoreTrajectory(trajectory, scene.robot.maxReach, scene.robot.base);
  const steps: PlanStep[] = [
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[0], graspMode: grasp.mode, description: `移动到 ${target.id} 上方。` },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[1], graspMode: grasp.mode, description: `下降并对准 ${target.id}。` },
    { action: 'grasp', objectId: target.id, graspMode: grasp.mode, description: '夹爪闭合并保持夹持。' },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[2], graspMode: grasp.mode, description: '抬升到安全高度并保持夹持。' },
  ];

  return createReadyPlan(scene, instruction, 'pick_only', target, null, steps, trajectory, grasp, cartesianWaypoints, jointTrajectory, score);
}

function createRelativePlacePlan(
  scene: SceneState,
  instruction: string,
  target: SceneObject,
  alreadyHolding: boolean
): ManipulationPlan {
  const direction = detectRelativeDirection(instruction) ?? { x: RELATIVE_STEP, y: 0, z: 0 };
  const placeCenter = {
    x: round3(target.position.x + direction.x),
    y: round3(target.size.y / 2),
    z: round3(target.position.z + direction.z),
  };

  if (alreadyHolding) {
    return createHeldPlacePlan(scene, instruction, target, null, 'relative_place', placeCenter);
  }

  const grasp = createGraspSpec(target, inferGraspMode(instruction), scene.robot.base);
  const motionTrajectory = [
    preGraspFromGrasp(grasp),
    grasp.center,
    { x: grasp.center.x, y: SAFE_Y, z: grasp.center.z },
    { x: placeCenter.x, y: SAFE_Y, z: placeCenter.z },
    placeCenter,
    { x: placeCenter.x, y: SAFE_Y, z: placeCenter.z },
  ];
  const trajectory = withStartTcp(scene, motionTrajectory, grasp);
  const cartesianWaypoints = createPickPlaceWaypoints(scene, trajectory);
  const jointTrajectory = createLm3JointTrajectory(
    cartesianWaypoints.map((waypoint) => toLm3Waypoint(waypoint, grasp)),
    initialJoints(scene)
  );
  const score = scoreTrajectory(trajectory, scene.robot.maxReach, scene.robot.base);
  const steps: PlanStep[] = [
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[0], graspMode: grasp.mode, description: `移动到 ${target.id} 上方。` },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[1], graspMode: grasp.mode, description: `下降并对准 ${target.id}。` },
    { action: 'grasp', objectId: target.id, graspMode: grasp.mode, description: '夹爪闭合。' },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[2], graspMode: grasp.mode, description: '抬升到安全高度。' },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[3], graspMode: grasp.mode, description: '移动到相对放置点上方。' },
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[4], graspMode: grasp.mode, description: '下降到相对放置点。' },
    { action: 'release', graspMode: grasp.mode, description: '释放物体。' },
    { action: 'retreat', pose: motionTrajectory[5], graspMode: grasp.mode, description: '退回安全高度。' },
  ];

  return createReadyPlan(scene, instruction, 'relative_place', target, null, steps, trajectory, grasp, cartesianWaypoints, jointTrajectory, score);
}

function createHeldPlacePlan(
  scene: SceneState,
  instruction: string,
  target: SceneObject,
  destination: SceneObject | null,
  taskType: PlanTaskType,
  explicitPlaceCenter?: Vector3
): ManipulationPlan {
  const grasp = createGraspSpec(target, inferGraspMode(instruction), scene.robot.base);
  const placeCenter = explicitPlaceCenter ?? stackedCenter(target, destination!);
  const motionTrajectory = [
    { x: target.position.x, y: SAFE_Y, z: target.position.z },
    { x: placeCenter.x, y: SAFE_Y, z: placeCenter.z },
    placeCenter,
    { x: placeCenter.x, y: SAFE_Y, z: placeCenter.z },
  ];
  const trajectory = withStartTcp(scene, motionTrajectory, grasp);
  const cartesianWaypoints = createCartesianWaypoints(
    trajectory,
    ['start_tcp', 'lift', 'pre_place', 'place', 'retreat'],
    ['close', 'close', 'close', 'close', 'open']
  );
  const jointTrajectory = createLm3JointTrajectory(
    cartesianWaypoints.map((waypoint) => toLm3Waypoint(waypoint, grasp)),
    initialJoints(scene)
  );
  const score = scoreTrajectory(trajectory, scene.robot.maxReach, scene.robot.base);
  const steps: PlanStep[] = [
    { action: 'move_to', objectId: target.id, pose: motionTrajectory[0], graspMode: grasp.mode, description: '保持夹持并抬升到安全高度。' },
    { action: 'move_to', objectId: destination?.id ?? target.id, pose: motionTrajectory[1], graspMode: grasp.mode, description: '移动到放置点上方。' },
    { action: 'move_to', objectId: destination?.id ?? target.id, pose: motionTrajectory[2], graspMode: grasp.mode, description: '下降到放置中心。' },
    { action: 'release', objectId: destination?.id, graspMode: grasp.mode, description: '夹爪张开，释放物体。' },
    { action: 'retreat', pose: motionTrajectory[3], graspMode: grasp.mode, description: '退回安全高度。' },
  ];

  return createReadyPlan(scene, instruction, taskType, target, destination, steps, trajectory, grasp, cartesianWaypoints, jointTrajectory, score);
}

function createReadyPlan(
  scene: SceneState,
  instruction: string,
  taskType: PlanTaskType,
  target: SceneObject,
  destination: SceneObject | null,
  steps: PlanStep[],
  trajectory: Vector3[],
  grasp: GraspSpec,
  cartesianWaypoints: CartesianWaypoint[],
  jointTrajectory: ReturnType<typeof createLm3JointTrajectory>,
  score: ReturnType<typeof scoreTrajectory>
): ManipulationPlan {
  void scene;
  return {
    id: `plan-${Date.now()}`,
    status: 'ready',
    taskType,
    instruction,
    targetObjectId: target.id,
    destinationObjectId: destination?.id ?? null,
    steps,
    trajectory,
    cartesianWaypoints,
    jointTrajectory,
    executionMode: 'sim',
    robotModel: LM3_MODEL,
    grasp,
    score,
    repairHint: null,
    source: 'local',
  };
}

function createRepairPlan(
  instruction: string,
  taskType: PlanTaskType,
  repairHint = '无法识别目标物体或目标位置。请使用场景中可见物体的名称，例如“把红色方块放到蓝色方块上”。',
  targetObjectId: string | null = null,
  destinationObjectId: string | null = null
): ManipulationPlan {
  return {
    id: `plan-${Date.now()}`,
    status: 'needs_repair',
    taskType,
    instruction,
    targetObjectId,
    destinationObjectId,
    steps: [],
    trajectory: [],
    cartesianWaypoints: [],
    jointTrajectory: [],
    executionMode: 'sim',
    robotModel: LM3_MODEL,
    score: { total: 0, reachability: 0, smoothness: 0, collision: 0, jointLimits: 'OK' },
    repairHint,
    source: 'local',
  };
}

function createPickPlaceWaypoints(scene: SceneState, trajectory: Vector3[]): CartesianWaypoint[] {
  return createCartesianWaypoints(
    trajectory,
    ['start_tcp', 'pre_grasp', 'grasp', 'lift', 'pre_place', 'place', 'retreat'],
    [startGripper(scene), 'open', 'open', 'close', 'close', 'close', 'open']
  );
}

function createCartesianWaypoints(
  trajectory: Vector3[],
  ids: string[],
  grippers: CartesianWaypoint['gripper'][]
): CartesianWaypoint[] {
  const topDown = { x: 0, y: -1, z: 0 };
  return trajectory.map((position, index) => ({
    id: ids[index] ?? `wp_${index + 1}`,
    position,
    frame: 'grasp_center' as const,
    targetDirection: topDown,
    gripper: grippers[index] ?? 'hold',
    speed: index === 1 || index === 2 ? 0.05 : 0.15,
    blendRadius: index === 1 || index === 2 ? 0 : 0.02,
    description: ids[index] ?? `waypoint ${index + 1}`,
  }));
}

function createCartesianWaypointsFromSteps(scene: SceneState, steps: PlanStep[], grasp?: GraspSpec): CartesianWaypoint[] {
  const topDown = { x: 0, y: -1, z: 0 };
  const motionSteps = steps.filter((step): step is PlanStep & { pose: Vector3 } => Boolean(step.pose));
  if (!motionSteps.length) {
    return [];
  }

  return [
    {
      id: 'start_tcp',
      position: startGraspCenter(scene, grasp),
      frame: 'grasp_center' as const,
      targetDirection: topDown,
      gripper: startGripper(scene),
      speed: 0.15,
      blendRadius: 0.02,
      description: 'current grasp center before planning',
    },
    ...motionSteps.map((step, index) => ({
      id: `${step.action}_${index + 1}`,
      position: step.pose,
      frame: 'grasp_center' as const,
      targetDirection: topDown,
      gripper: gripperForAction(step.action),
      speed: 0.12,
      blendRadius: 0.01,
      description: step.description,
    })),
  ];
}

function toLm3Waypoint(waypoint: CartesianWaypoint, grasp?: GraspSpec) {
  // 所有航点的 position 现在都是 grasp_center 坐标
  // 需要通过 toolOffset 转为 TCP 坐标给 IK 求解
  const position = grasp
    ? toolTargetFromGraspCenter(waypoint.position, grasp)
    : addTcpCalibration(waypoint.position);
  return {
    id: waypoint.id,
    position,
    targetDirection: waypoint.targetDirection,
    gripper: waypoint.gripper,
  };
}

function addTcpCalibration(pos: Vector3): Vector3 {
  const cal = getTcpCalibration();
  return { x: pos.x + cal.x, y: pos.y + cal.y, z: pos.z + cal.z };
}

function withStartTcp(scene: SceneState, motionTrajectory: Vector3[], grasp?: GraspSpec): Vector3[] {
  if (!motionTrajectory.length) {
    return [];
  }
  return [startGraspCenter(scene, grasp), ...motionTrajectory];
}

function startGraspCenter(scene: SceneState, grasp?: GraspSpec): Vector3 {
  // 使用解析 FK 而非 GLB 模型的 tcp，确保与 IK 同源、坐标系自洽
  const tcp = forwardKinematicsLm3(initialJoints(scene)).position;
  if (grasp) {
    return graspCenterFromToolTarget(tcp, grasp);
  }
  // 无 grasp 时用 TCP 校准偏移做逆变换：grasp_center = tcp - toolOffset
  const cal = getTcpCalibration();
  return { x: tcp.x - cal.x, y: tcp.y - cal.y, z: tcp.z - cal.z };
}

function initialJoints(scene: SceneState): readonly number[] {
  return scene.robot.joints?.length === 6 ? scene.robot.joints : LM3_HOME_JOINTS;
}

function startGripper(scene: SceneState): CartesianWaypoint['gripper'] {
  return scene.robot.holding ? 'close' : 'open';
}

function gripperForAction(action: PlanStep['action']): CartesianWaypoint['gripper'] {
  if (action === 'release') {
    return 'open';
  }
  if (action === 'grasp') {
    return 'close';
  }
  return 'hold';
}

function detectTaskType(instruction: string): PlanTaskType {
  if (detectRelativeDirection(instruction)) {
    return 'relative_place';
  }
  if (/\b(pick up|grab|grasp|hold|lift)\b|夹起|抓起|拿起|夹住/.test(instruction.toLowerCase())) {
    return 'pick_only';
  }
  return 'pick_place';
}

function detectRelativeDirection(instruction: string): Vector3 | null {
  const text = instruction.toLowerCase();
  if (/\bright\b|右/.test(text)) {
    return { x: RELATIVE_STEP, y: 0, z: 0 };
  }
  if (/\bleft\b|左/.test(text)) {
    return { x: -RELATIVE_STEP, y: 0, z: 0 };
  }
  if (/\b(front|forward)\b|前/.test(text)) {
    return { x: 0, y: 0, z: -RELATIVE_STEP };
  }
  if (/\b(back|behind|backward)\b|后/.test(text)) {
    return { x: 0, y: 0, z: RELATIVE_STEP };
  }
  return null;
}

function referencesHeldObject(instruction: string): boolean {
  return /\bit\b|\bthis\b|\bheld\b|\bcurrent\b|它|当前|这个/.test(instruction.toLowerCase());
}

function preGraspFromGrasp(grasp: GraspSpec): Vector3 {
  return {
    x: grasp.center.x,
    y: round3(grasp.center.y + 0.12),
    z: grasp.center.z,
  };
}

function round3(value: number): number {
  return Math.round(value * 1000) / 1000;
}

export { createLocalPlan as createPickPlacePlan };
