export type ObjectType = 'cube' | 'cylinder';

export type ActionName =
  | 'move_to'
  | 'grasp'
  | 'release'
  | 'retreat';

export type PlanTaskType = 'pick_place' | 'pick_only' | 'relative_place';

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Size3 {
  x: number;
  y: number;
  z: number;
}

export interface SceneObject {
  id: string;
  label: string;
  type: ObjectType;
  color: string;
  position: Vector3;
  size: Size3;
  movable: boolean;
}

export interface RobotState {
  base: Vector3;
  maxReach: number;
  holding: string | null;
  tcp?: Vector3;
  joints?: number[];
}

export interface SceneState {
  robot: RobotState;
  objects: SceneObject[];
  history: ExecutionRecord[];
}

export interface VisualSnapshot {
  id: string;
  dataUrl: string;
  mimeType: 'image/jpeg' | 'image/png';
  capturedAt: string;
  camera: {
    name: 'eye_to_hand';
    position: Vector3;
    target: Vector3;
  };
}

export interface ExecutionRecord {
  instruction: string;
  targetObjectId: string | null;
  destinationObjectId: string | null;
  status: 'ready' | 'blocked' | 'executed';
}

export type AffordanceKind = 'top_grasp' | 'side_grasp' | 'top_place' | 'center_place';
export type GraspMode = 'top' | 'side';

export interface AffordanceCandidate {
  id: string;
  objectId: string;
  kind: AffordanceKind;
  pose: Vector3;
  approach: Vector3;
  score: number;
  note: string;
}

export interface GraspSpec {
  mode: GraspMode;
  objectId: string;
  center: Vector3;
  approach: Vector3;
  closingAxis: Vector3;
  toolOffset: Vector3;
  requiredOpeningMm: number;
  maxOpeningMm: number;
}

export interface PlanStep {
  action: ActionName;
  objectId?: string;
  pose?: Vector3;
  graspMode?: GraspMode;
  description: string;
}

export interface CartesianWaypoint {
  id: string;
  position: Vector3;
  frame?: 'tcp' | 'grasp_center';
  targetDirection?: Vector3;
  gripper: 'open' | 'close' | 'hold';
  speed: number;
  blendRadius: number;
  description: string;
}

export interface JointTrajectoryPoint {
  waypointId: string;
  q: number[];
  tcpPosition: Vector3;
  timeFromStart: number;
  reachable: boolean;
  error: number;
  directionScore: number;
}

export interface RobotModelRef {
  id: 'lebai-lm3';
  source: 'glb';
  tcpOffset: Vector3;
}

export interface ManipulationPlan {
  id: string;
  status: 'ready' | 'needs_repair';
  taskType: PlanTaskType;
  instruction: string;
  targetObjectId: string | null;
  destinationObjectId: string | null;
  steps: PlanStep[];
  trajectory: Vector3[];
  cartesianWaypoints?: CartesianWaypoint[];
  jointTrajectory?: JointTrajectoryPoint[];
  executionMode?: 'sim' | 'real';
  robotModel?: RobotModelRef;
  grasp?: GraspSpec;
  score: TrajectoryScore;
  repairHint: string | null;
  source: 'llm' | 'local';
  confidence?: number;
}

export interface ValidationIssue {
  code:
    | 'target_missing'
    | 'destination_missing'
    | 'target_unreachable'
    | 'destination_unreachable'
    | 'object_not_movable'
    | 'empty_plan';
  message: string;
  objectId?: string;
}

export interface ValidationResult {
  ok: boolean;
  issues: ValidationIssue[];
  repairHint: string;
}

export interface TrajectoryScore {
  total: number;
  reachability: number;
  smoothness: number;
  collision: number;
  jointLimits: 'OK' | 'WARN';
}

export interface RobotProfile {
  id: string;
  name: string;
  gripperName: string;
  maxReach: number;
  payloadKg: number;
  gripperStrokeMm: number;
  safeZ: number;
  tableHeight: number;
}

/** LLM-generated plan step */
export interface LLMPlanStep {
  action: ActionName;
  objectId: string;
  targetPosition: Vector3;
  gripperAction?: 'close' | 'open';
  description?: string;
}

/** LLM planning response */
export interface LLMPlanResponse {
  steps: LLMPlanStep[];
  confidence: number;
  explanation?: string;
}
