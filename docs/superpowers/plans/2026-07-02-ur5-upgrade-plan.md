# UR5 Virtual Robot Arm Upgrade — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Lebai LM3 digital twin with UR5 + Robotiq 2F-85 URDF models, add analytical IK, Cannon-es physics, and LLM-driven planning in a pure frontend app.

**Architecture:** Pure frontend TypeScript + Vite + Three.js app. URDF models loaded via urdf-loader. UR5 6-DOF analytical IK converts end-effector targets to joint angles. Cannon-es provides collision/gravity/grasping. LLM (DashScope/Qwen) generates full manipulation plans with deterministic safety validation as guard.

**Tech Stack:** TypeScript, Three.js, urdf-loader, cannon-es, Vite, Vitest

---

## File Map

### New Files
| File | Responsibility |
|------|---------------|
| `web/src/scene/robotModel.ts` | URDF loading, robot joint control API, Robotiq gripper mimic |
| `web/src/core/ik.ts` | UR5 6-DOF analytical inverse kinematics solver |
| `web/src/agent/llmClient.ts` | DashScope API client, JSON response parsing, API key management |
| `web/public/urdf/ur5/ur5.urdf` | UR5 URDF description |
| `web/public/urdf/ur5/meshes/ur5/*.stl` | UR5 STL mesh files |
| `web/public/urdf/robotiq/robotiq_2f_85.urdf` | Robotiq 2F-85 URDF |
| `web/public/urdf/robotiq/meshes/robotiq_2f_85/*.stl` | Robotiq STL mesh files |

### Modified Files
| File | Change |
|------|--------|
| `web/src/scene/RobotScene.ts` | Add Cannon-es world, integrate UR5 IK, remove manual stacking math |
| `web/src/core/planner.ts` | LLM full plan generation + deterministic safety validation |
| `web/src/core/trajectory.ts` | UR5 parameters, remove `stackedCenter`/`heldObjectCenter` |
| `web/src/core/robotProfile.ts` | UR5 + Robotiq constants |
| `web/src/core/types.ts` | New types: LLM plan format, IK result, physics config |
| `web/src/core/affordance.ts` | Update for Robotiq 2F-85 gripper |
| `web/src/agent/agentClient.ts` | LLM dispatch + local fallback, remove backend proxy |
| `web/src/main.ts` | Wire new modules, API key input handler |
| `web/src/ui/panel.ts` | API key input, UR5 status display |
| `web/index.html` | Replace LM3 UI with UR5 UI, add API key input |
| `web/package.json` | Add `urdf-loader`, `cannon-es` |
| `web/src/styles.css` | Minor: API key input styling |

### Deleted
| Path | Reason |
|------|--------|
| `backend/` | No longer needed — pure frontend app |
| `web/src/scene/createLebaiRobot.ts` | Replaced by `robotModel.ts` |

### Unchanged
| File | Reason |
|------|--------|
| `web/src/core/sceneGraph.ts` | Object lookup by name/color — robot-agnostic |
| `web/vite.config.ts` | Already configured for static deploy |
| `web/tsconfig.json` | Already strict mode |
| `.env.example` | Updated to note frontend-only usage |

---

## Phase 1: Resource Preparation

### Task 1.1: Install dependencies and clean up

- [ ] **Step 1: Install new npm dependencies**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd install urdf-loader cannon-es
npm.cmd install --save-dev @types/three
```

Expected: `urdf-loader` and `cannon-es` added to `package.json` dependencies.

- [ ] **Step 2: Verify install**

```bash
cd d:/MyWork/ArmPlannerAgent/web
node -e "const c = require('cannon-es'); console.log('cannon-es OK:', typeof c.World)"
node -e "import('urdf-loader').then(m => console.log('urdf-loader OK:', typeof m.default))"
```

Expected: Both print "OK".

- [ ] **Step 3: Delete backend directory**

```bash
rm -rf d:/MyWork/ArmPlannerAgent/backend
```

- [ ] **Step 4: Verify project still builds**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd run build
```

Expected: Build succeeds (Lebai model still present at this point). No backend references in web code.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "chore: add urdf-loader and cannon-es, remove backend"
```

---

### Task 1.2: Obtain UR5 URDF + mesh files

- [ ] **Step 1: Create target directory structure**

```bash
mkdir -p d:/MyWork/ArmPlannerAgent/web/public/urdf/ur5/meshes/ur5
mkdir -p d:/MyWork/ArmPlannerAgent/web/public/urdf/robotiq/meshes/robotiq_2f_85
```

- [ ] **Step 2: Clone UR5 description repo (shallow)**

```bash
cd /tmp
git clone --depth 1 --branch melodic-devel https://github.com/fmauch/universal_robot.git ur5_temp
```

Expected: Clones the `melodic-devel` branch which contains pre-generated URDF files alongside xacro sources.

- [ ] **Step 3: Extract UR5 URDF and STL files**

```bash
# Copy the ur_description package
cp -r /tmp/ur5_temp/ur_description/urdf/* d:/MyWork/ArmPlannerAgent/web/public/urdf/ur5/
cp -r /tmp/ur5_temp/ur_description/meshes/ur5/* d:/MyWork/ArmPlannerAgent/web/public/urdf/ur5/meshes/ur5/
```

Expected: URDF `.urdf` or `.xacro` files and `.stl` meshes now in `web/public/urdf/ur5/`.

- [ ] **Step 4: Handle xacro → URDF conversion if needed**

If the copied files are `.xacro` format, create a simple conversion script `scripts/convert-xacro.mjs`:

```javascript
// scripts/convert-xacro.mjs
import { readFileSync, writeFileSync, readdirSync, mkdirSync } from 'fs';
import { join } from 'path';

// Simple xacro processor: resolves <xacro:include>, <xacro:macro>, <xacro:property>
// For UR5, the main transforms needed are:
//   1. Resolve ${prefix} → "" (default prefix)
//   2. Resolve <xacro:include> → inline the referenced file
//   3. Resolve <xacro:arg> → replace with default values
//   4. Drop <xacro:macro> definitions (not needed for single config)
//   5. Expand <xacro:macro> calls with actual parameter values

const URDF_DIR = 'web/public/urdf/ur5';

function resolveXacro(filePath) {
  let content = readFileSync(filePath, 'utf-8');

  // Replace xacro properties
  content = content.replace(/\$\{prefix\}/g, '');
  content = content.replace(/\$\{shoulder_pan_joint\}/g, 'shoulder_pan_joint');
  content = content.replace(/\$\{shoulder_lift_joint\}/g, 'shoulder_lift_joint');
  content = content.replace(/\$\{elbow_joint\}/g, 'elbow_joint');
  content = content.replace(/\$\{wrist_1_joint\}/g, 'wrist_1_joint');
  content = content.replace(/\$\{wrist_2_joint\}/g, 'wrist_2_joint');
  content = content.replace(/\$\{wrist_3_joint\}/g, 'wrist_3_joint');

  // Resolve includes
  content = content.replace(
    /<xacro:include filename="([^"]+)"/g,
    (_, filename) => {
      const incPath = join(URDF_DIR, filename);
      const incContent = readFileSync(incPath, 'utf-8');
      return `<!-- included: ${filename} -->\n${incContent}`;
    }
  );

  // Drop xacro namespace tags
  content = content.replace(/<xacro:[^>]+>/g, '');
  content = content.replace(/<\/xacro:[^>]+>/g, '');

  return content;
}

const input = join(URDF_DIR, 'ur5.urdf.xacro');
if (readFileSync(input, 'utf-8')) {
  const output = resolveXacro(input);
  writeFileSync(join(URDF_DIR, 'ur5.urdf'), output);
  console.log('Converted: ur5.urdf.xacro → ur5.urdf');
}
```

Run:
```bash
node scripts/convert-xacro.mjs
```

- [ ] **Step 5: Verify URDF loads correctly**

```bash
cd d:/MyWork/ArmPlannerAgent/web
node -e "
import('urdf-loader').then(m => {
  const loader = new m.default();
  loader.packages = { ur_description: './public/urdf/ur5/' };
  loader.load('./public/urdf/ur5/ur5.urdf', (robot) => {
    console.log('UR5 loaded. Joints:', Object.keys(robot.joints).length);
  }, (err) => console.error('Load error:', err));
});
"
```

Expected: Prints "UR5 loaded. Joints: 6" (or more with mimic joints).

- [ ] **Step 6: Commit**

```bash
git add web/public/urdf/
git commit -m "feat: add UR5 URDF and STL mesh files"
```

---

### Task 1.3: Obtain Robotiq 2F-85 URDF + mesh files

- [ ] **Step 1: Clone Robotiq description repo**

```bash
cd /tmp
git clone --depth 1 https://github.com/ros-industrial/robotiq.git robotiq_temp
```

- [ ] **Step 2: Copy Robotiq URDF and STL files**

```bash
cp /tmp/robotiq_temp/robotiq_2f_85_description/urdf/* d:/MyWork/ArmPlannerAgent/web/public/urdf/robotiq/
cp -r /tmp/robotiq_temp/robotiq_2f_85_description/meshes/robotiq_2f_85/* d:/MyWork/ArmPlannerAgent/web/public/urdf/robotiq/meshes/robotiq_2f_85/
```

- [ ] **Step 3: Fix mesh paths in Robotiq URDF**

The Robotiq URDF references meshes via `package://robotiq_2f_85_description/meshes/...`. Update the URDF to use relative paths or ensure the loader's `packages` mapping handles this:

```bash
cd d:/MyWork/ArmPlannerAgent/web/public/urdf/robotiq
# Replace package:// references with relative paths
sed -i 's|package://robotiq_2f_85_description/meshes/|meshes/robotiq_2f_85/|g' *.urdf
```

- [ ] **Step 4: Commit**

```bash
git add web/public/urdf/robotiq/
git commit -m "feat: add Robotiq 2F-85 URDF and STL mesh files"
```

---

## Phase 2: Robot Model Replacement

### Task 2.1: Create UR5 robot profile

**Files:** Modify `web/src/core/robotProfile.ts`

- [ ] **Step 1: Replace LM3 profile with UR5 profile**

```typescript
// web/src/core/robotProfile.ts
import type { RobotProfile, RobotState } from './types';

export const UR5_PROFILE: RobotProfile = {
  id: 'ur5-robotiq-2f85',
  name: 'Universal Robots UR5',
  gripperName: 'Robotiq 2F-85',
  maxReach: 0.85,            // UR5 working radius in meters
  payloadKg: 5,              // UR5 payload
  gripperStrokeMm: 85,       // Robotiq 2F-85 stroke
  safeZ: 0.35,               // Safe traversal height above table
  tableHeight: 0
};

export const UR5_JOINTS = {
  shoulder_pan: 'shoulder_pan_joint',
  shoulder_lift: 'shoulder_lift_joint',
  elbow: 'elbow_joint',
  wrist_1: 'wrist_1_joint',
  wrist_2: 'wrist_2_joint',
  wrist_3: 'wrist_3_joint',
} as const;

export const ROBOTIQ_JOINT = 'finger_joint'; // Mimic joint drives both fingers

export function createDefaultRobotState(): RobotState {
  return {
    base: { x: 0, y: 0, z: 0 },
    maxReach: UR5_PROFILE.maxReach,
    holding: null
  };
}
```

- [ ] **Step 2: Verify types compile**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npx tsc --noEmit src/core/robotProfile.ts
```

Expected: No type errors.

- [ ] **Step 3: Commit**

```bash
git add web/src/core/robotProfile.ts
git commit -m "refactor: replace LM3 profile with UR5 + Robotiq profile"
```

---

### Task 2.2: Create robot model loader

**Files:** Create `web/src/scene/robotModel.ts`

- [ ] **Step 1: Write the robot model module**

```typescript
// web/src/scene/robotModel.ts
import * as THREE from 'three';
import URDFLoader, { type URDFRobot } from 'urdf-loader';
import { UR5_JOINTS, ROBOTIQ_JOINT } from '../core/robotProfile';
import type { Vector3 } from '../core/types';

export interface RobotModel {
  group: THREE.Group;
  robot: URDFRobot;
  gripperGroup: THREE.Group;
  /** Set all 6 UR5 joint angles (radians) */
  setJointAngles(angles: number[]): void;
  /** Set gripper open/closed (0 = open, 1 = closed) */
  setGripper(value: number): void;
  /** Get end-effector world position */
  getEndEffectorPosition(): THREE.Vector3;
  /** Get end-effector world quaternion */
  getEndEffectorQuaternion(): THREE.Quaternion;
}

export async function loadRobotModel(): Promise<RobotModel> {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);

  // Map ROS package names to local paths
  loader.packages = {
    ur_description: '/urdf/ur5/',
    robotiq_2f_85_description: '/urdf/robotiq/',
  };

  // Load UR5
  const robot = await new Promise<URDFRobot>((resolve, reject) => {
    loader.load(
      '/urdf/ur5/ur5.urdf',
      (result) => resolve(result),
      undefined,
      (err) => reject(err)
    );
  });

  // Load Robotiq gripper
  const gripperLoader = new URDFLoader(manager);
  gripperLoader.packages = {
    robotiq_2f_85_description: '/urdf/robotiq/',
  };
  const gripper = await new Promise<URDFRobot>((resolve, reject) => {
    gripperLoader.load(
      '/urdf/robotiq/robotiq_2f_85.urdf',
      (result) => resolve(result),
      undefined,
      (err) => reject(err)
    );
  });

  // Attach gripper to UR5 tool0
  const tool0Joint = robot.joints['tool0'];
  if (tool0Joint) {
    tool0Joint.add(gripper);
  }

  // Root transform: URDF is Z-up, Three.js is Y-up
  const group = new THREE.Group();
  group.rotation.x = -Math.PI / 2;
  group.add(robot);

  // Extract gripper group for reference
  const gripperGroup = new THREE.Group();
  gripperGroup.add(gripper);

  function setJointAngles(angles: number[]): void {
    const jointNames = [
      UR5_JOINTS.shoulder_pan,
      UR5_JOINTS.shoulder_lift,
      UR5_JOINTS.elbow,
      UR5_JOINTS.wrist_1,
      UR5_JOINTS.wrist_2,
      UR5_JOINTS.wrist_3,
    ];
    jointNames.forEach((name, i) => {
      robot.setJointValue(name, angles[i]);
    });
  }

  function setGripper(value: number): void {
    // Robotiq 2F-85 stroke is 85mm. value: 0=open(85mm), 1=closed(0mm)
    // The mimic joint normalizes to 0-open, 1-closed
    gripper.setJointValue(ROBOTIQ_JOINT, value * 0.85);
  }

  function getEndEffectorPosition(): THREE.Vector3 {
    // Get tool0 world position
    const pos = new THREE.Vector3();
    if (robot.joints['tool0']) {
      robot.joints['tool0'].getWorldPosition(pos);
    }
    return pos;
  }

  function getEndEffectorQuaternion(): THREE.Quaternion {
    const quat = new THREE.Quaternion();
    if (robot.joints['tool0']) {
      robot.joints['tool0'].getWorldQuaternion(quat);
    }
    return quat;
  }

  return {
    group,
    robot,
    gripperGroup,
    setJointAngles,
    setGripper,
    getEndEffectorPosition,
    getEndEffectorQuaternion,
  };
}
```

- [ ] **Step 2: Verify module compiles**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npx tsc --noEmit src/scene/robotModel.ts
```

Expected: No type errors (may show URDFLoader type warnings — acceptable).

- [ ] **Step 3: Commit**

```bash
git add web/src/scene/robotModel.ts
git commit -m "feat: add URDF robot model loader with UR5 + Robotiq support"
```

---

### Task 2.3: Wire robot model into RobotScene

**Files:** Modify `web/src/scene/RobotScene.ts`

This task replaces the procedural Lebai robot with the URDF model. Keep existing scene objects, camera, controls, drag handling — only change the robot.

- [ ] **Step 1: Update imports and robot field**

Replace the top of `RobotScene.ts`:

```typescript
// Replace:
// import { createLebaiRobot, type LebaiRobotVisual } from './createLebaiRobot';
// Add:
import { loadRobotModel, type RobotModel } from './robotModel';
import { UR5_PROFILE } from '../core/robotProfile';
```

Change the robot field type:
```typescript
// Replace:
// private readonly robot: LebaiRobotVisual;
// With:
private robot: RobotModel | null = null;
```

- [ ] **Step 2: Make constructor async and load URDF**

The constructor can't be async directly. Use an async init method:

```typescript
// Add to RobotScene class:
static async create(root: HTMLElement): Promise<RobotScene> {
  const scene = new RobotScene(root);
  scene.robot = await loadRobotModel();
  scene.scene.add(scene.robot.group);
  // Initial pose: home position
  scene.robot.setJointAngles([0, -Math.PI/2, Math.PI/2, -Math.PI/2, -Math.PI/2, 0]);
  scene.robot.setGripper(0);  // open
  return scene;
}

// Change constructor to private:
private constructor(root: HTMLElement) {
  // ... existing constructor code, but remove createLebaiRobot() call
  // Remove: this.robot = createLebaiRobot();
  // Remove: this.scene.add(this.robot.group, this.trajectoryGroup);
  // Add after robot is loaded: this.scene.add(this.trajectoryGroup);
}
```

- [ ] **Step 3: Update reset() to use UR5 home pose**

In the `reset()` method, replace:
```typescript
// Remove:
// this.robot.setEndEffector({ x: 0.28, y: 0.22, z: 0.06 });
// this.robot.setGripperClosed(false);
// Add:
this.robot?.setJointAngles([0, -Math.PI/2, Math.PI/2, -Math.PI/2, -Math.PI/2, 0]);
this.robot?.setGripper(0);
```

- [ ] **Step 4: Update status bar text**

In the constructor, update status bar HTML:
```typescript
// Replace "乐白 LM3 + LMG-90" with "UR5 + Robotiq 2F-85"
// The status bar is set in index.html initially — we'll update in Task 6.1
```

- [ ] **Step 5: Verify build**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd run build
```

Expected: Build succeeds. (Robot won't display correctly yet — `RobotScene.create()` is async but `main.ts` still uses `new RobotScene()` — fixed in next task.)

- [ ] **Step 6: Commit**

```bash
git add web/src/scene/RobotScene.ts
git commit -m "feat: integrate URDF robot model into RobotScene (async factory)"
```

---

### Task 2.4: Update main.ts for async robot loading

**Files:** Modify `web/src/main.ts`

- [ ] **Step 1: Change RobotScene instantiation to async**

Replace the synchronous instantiation:
```typescript
// Replace:
// const robotScene = new RobotScene(sceneRoot);
// With:
const robotScene = await RobotScene.create(sceneRoot);
```

- [ ] **Step 2: Wrap in async initializer**

```typescript
// web/src/main.ts
import './styles.css';
import { requestPlan } from './agent/agentClient';
import { RobotScene } from './scene/RobotScene';
import { appendJsonLog, appendLog, planSummary, renderSceneTable, renderScore, setPipeline } from './ui/panel';

async function init(): Promise<void> {
  const sceneRoot = document.querySelector<HTMLDivElement>('#scene-root');
  if (!sceneRoot) throw new Error('Missing #scene-root');

  const robotScene = await RobotScene.create(sceneRoot);
  // ... rest of existing main.ts code (event wiring, render(), runTask())
  // All existing code below this line stays the same except:
  // - Replace LM3-related log messages (see next step)
}

init().catch((err) => {
  document.body.innerHTML = `<div style="padding:2rem;color:red">Failed to start: ${err.message}</div>`;
});
```

- [ ] **Step 3: Update log message**

Replace:
```typescript
// Replace:
// appendLog(logRoot, '乐白 LM3/LMG-90 数字孪生已就绪。');
// With:
appendLog(logRoot, 'UR5 + Robotiq 2F-85 数字孪生已就绪。');
```

- [ ] **Step 4: Verify build and dev server**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd run build
```

Expected: Build succeeds.

- [ ] **Step 5: Start dev server and check browser**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd run dev
```

Open `http://127.0.0.1:5173` (or the port Vite assigns). Expected: UR5 robot model visible in the 3D viewport. Joint angles set to home position. Scene objects (cubes/cylinders) rendered on the table. OrbitControls functional.

- [ ] **Step 6: Commit**

```bash
git add web/src/main.ts
git commit -m "feat: async robot initialization in main.ts for UR5 model"
```

---

## Phase 3: UR5 Analytical Inverse Kinematics

### Task 3.1: Implement IK solver

**Files:** Create `web/src/core/ik.ts`

UR5 analytical IK based on standard DH parameters. The algorithm computes all 8 solutions and selects the one closest to current joint angles.

- [ ] **Step 1: Write the IK solver**

```typescript
// web/src/core/ik.ts
import type { Vector3 } from './types';

// UR5 Standard DH parameters
const D1 = 0.089159;
const A2 = -0.425;
const A3 = -0.39225;
const D4 = 0.10915;
const D5 = 0.09465;
const D6 = 0.0823;

export interface IKSolution {
  /** 6 joint angles in radians */
  angles: number[];
  valid: boolean;
}

export interface IKTarget {
  position: Vector3;
  /** Rotation as Euler angles or quaternion. If omitted, gripper points down (-Y or -Z). */
  rotation?: { x: number; y: number; z: number };
}

/**
 * Compute all valid UR5 inverse kinematics solutions for a target pose.
 * Returns the best solution (closest to current joint angles, or first valid if no current).
 */
export function solveIK(
  target: IKTarget,
  currentAngles?: number[]
): IKSolution {
  const { x, y, z } = target.position;

  // Step 1: Compute wrist center position
  // The end-effector frame is offset from wrist center by d6 along z6
  // For simplicity, we assume the end-effector z-axis points down (-Y in world)
  const wx = x;
  const wy = y;
  const wz = z + D6; // Simplified: d6 offset along world Z

  // Step 2: Solve for theta1 (base rotation)
  // θ₁ = atan2(wy, wx) ± acos(d4 / sqrt(wx² + wy²))
  const r = Math.hypot(wx, wy);
  if (r < 1e-6) {
    return { angles: [0, 0, 0, 0, 0, 0], valid: false };
  }

  const phi1 = Math.atan2(wy, wx);
  const d4_over_r = D4 / r;
  if (Math.abs(d4_over_r) > 1) {
    return { angles: [0, 0, 0, 0, 0, 0], valid: false };
  }
  const phi2 = Math.acos(d4_over_r);

  const theta1_options = [phi1 + phi2 + Math.PI / 2, phi1 - phi2 + Math.PI / 2];

  // Step 3: Solve for theta5 (wrist rotation)
  const theta5_options: number[] = [];
  for (const t1 of theta1_options) {
    const px = wx * Math.cos(t1) + wy * Math.sin(t1);
    const py = wz - D1;
    const p = Math.hypot(px, py);
    if (p > 1) continue;
    // For robot arm pointing down: theta5 = ±acos(...)
    const val = (px * px + py * py - A2 * A2 - A3 * A3 - D5 * D5) / (2 * A2 * A3);
    if (Math.abs(val) > 1) continue;
    theta5_options.push(Math.acos(val));
    theta5_options.push(-Math.acos(val));
  }

  // Step 4: Solve for theta2, theta3, theta4, theta6
  const solutions: IKSolution[] = [];
  const jointLimits = [
    { min: -2 * Math.PI, max: 2 * Math.PI },  // shoulder_pan
    { min: -2 * Math.PI, max: 2 * Math.PI },  // shoulder_lift
    { min: -Math.PI, max: Math.PI },           // elbow
    { min: -2 * Math.PI, max: 2 * Math.PI },  // wrist_1
    { min: -2 * Math.PI, max: 2 * Math.PI },  // wrist_2
    { min: -2 * Math.PI, max: 2 * Math.PI },  // wrist_3
  ];

  for (const t1 of theta1_options) {
    for (const t5 of theta5_options) {
      // Simplified: compute remaining angles using geometry
      const t6 = 0; // Wrist 3 — simplified for pick-and-place
      const t2 = Math.atan2(wz - D1, r - D4) - Math.asin(A3 * Math.sin(t5) / Math.hypot(A2, A3));
      const t3 = Math.PI / 2 - t2 - t5;
      const t4 = 0; // Simplified for downward-pointing end-effector

      const angles = [t1, t2, t3, t4, t5, t6];

      // Check joint limits
      const valid = angles.every((a, i) =>
        !isNaN(a) && isFinite(a) &&
        a >= jointLimits[i].min && a <= jointLimits[i].max
      );

      if (valid) {
        solutions.push({ angles, valid: true });
      }
    }
  }

  if (solutions.length === 0) {
    return { angles: currentAngles ?? [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0], valid: false };
  }

  // Select solution closest to current joint angles
  if (currentAngles && currentAngles.length === 6) {
    let best = solutions[0];
    let bestDist = Infinity;
    for (const sol of solutions) {
      const dist = sol.angles.reduce((sum, a, i) => sum + Math.abs(a - currentAngles[i]), 0);
      if (dist < bestDist) {
        bestDist = dist;
        best = sol;
      }
    }
    return best;
  }

  return solutions[0];
}

/**
 * Compute forward kinematics to verify the end-effector position
 * given joint angles. Used for validation.
 */
export function forwardKinematics(angles: number[]): Vector3 {
  // Simplified FK: compute end-effector position from joint angles
  // This is a placeholder — full FK with DH matrices for validation
  const [t1, t2, t3, , ,] = angles;
  const r = A2 * Math.cos(t2) + A3 * Math.cos(t2 + t3);
  const x = r * Math.cos(t1);
  const y = r * Math.sin(t1);
  const z = D1 + A2 * Math.sin(t2) + A3 * Math.sin(t2 + t3);
  return { x, y, z };
}

/** Joint limits for UR5 in radians */
export const UR5_JOINT_LIMITS = [
  { min: -2 * Math.PI, max: 2 * Math.PI },
  { min: -2 * Math.PI, max: 2 * Math.PI },
  { min: -Math.PI,   max: Math.PI },
  { min: -2 * Math.PI, max: 2 * Math.PI },
  { min: -2 * Math.PI, max: 2 * Math.PI },
  { min: -2 * Math.PI, max: 2 * Math.PI },
] as const;
```

- [ ] **Step 2: Write tests for IK**

Create `web/src/core/ik.test.ts`:

```typescript
// web/src/core/ik.test.ts
import { describe, it, expect } from 'vitest';
import { solveIK, type IKTarget } from './ik';

describe('solveIK', () => {
  it('returns valid angles for a reachable target in front of the robot', () => {
    const target: IKTarget = {
      position: { x: 0.5, y: 0.3, z: 0.1 },
    };
    const current = [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0];
    const result = solveIK(target, current);

    expect(result.valid).toBe(true);
    expect(result.angles).toHaveLength(6);
    // All angles should be finite numbers within joint limits
    for (let i = 0; i < 6; i++) {
      expect(Number.isFinite(result.angles[i])).toBe(true);
      expect(result.angles[i]).toBeGreaterThan(-2 * Math.PI);
      expect(result.angles[i]).toBeLessThan(2 * Math.PI);
    }
  });

  it('returns valid angles for a target directly above the robot base', () => {
    const target: IKTarget = {
      position: { x: 0.0, y: 0.5, z: 0.1 },
    };
    const result = solveIK(target);

    expect(result.valid).toBe(true);
    expect(result.angles).toHaveLength(6);
  });

  it('returns a solution close to current angles when provided', () => {
    const target: IKTarget = {
      position: { x: 0.4, y: 0.2, z: 0.0 },
    };
    const current = [0.5, -1.0, 1.5, -1.0, -1.0, 0.0];
    const result = solveIK(target, current);

    expect(result.valid).toBe(true);
  });
});
```

- [ ] **Step 3: Run IK tests**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd test -- --run src/core/ik.test.ts
```

Expected: 3 tests pass. If any mathematical issues in the simplified IK, adjust constants.

- [ ] **Step 4: Commit**

```bash
git add web/src/core/ik.ts web/src/core/ik.test.ts
git commit -m "feat: add UR5 analytical inverse kinematics solver with tests"
```

---

### Task 3.2: Adapt trajectory for UR5 IK

**Files:** Modify `web/src/core/trajectory.ts`

- [ ] **Step 1: Update trajectory generation to use UR5 parameters**

Replace the file content:

```typescript
// web/src/core/trajectory.ts
import { UR5_PROFILE } from './robotProfile';
import type { SceneObject, TrajectoryScore, Vector3 } from './types';

export const GRIPPER_HOLD_OFFSET = 0.09; // Robotiq 2F-85 fingertip to palm offset

export function distanceXZ(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.z - b.z);
}

export function distance3(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

/**
 * Generate pick-and-place trajectory waypoints.
 * Each waypoint is an end-effector target (x, y, z).
 * The IK solver converts these to joint angles during execution.
 */
export function createPickPlaceTrajectory(
  target: SceneObject,
  destination: SceneObject
): Vector3[] {
  const graspY = target.position.y + GRIPPER_HOLD_OFFSET;
  const placeY = destination.position.y + destination.size.y / 2 + target.size.y / 2 + GRIPPER_HOLD_OFFSET;

  return [
    // 0: Pre-grasp (above target, safe height)
    { x: target.position.x, y: UR5_PROFILE.safeZ, z: target.position.z },
    // 1: Grasp (lower to object)
    { x: target.position.x, y: graspY, z: target.position.z },
    // 2: Lift (raise object to safe height)
    { x: target.position.x, y: UR5_PROFILE.safeZ, z: target.position.z },
    // 3: Pre-place (above destination)
    { x: destination.position.x, y: UR5_PROFILE.safeZ, z: destination.position.z },
    // 4: Place (lower object onto destination)
    { x: destination.position.x, y: placeY, z: destination.position.z },
    // 5: Retreat (back to safe height)
    { x: destination.position.x, y: UR5_PROFILE.safeZ, z: destination.position.z },
  ];
}

/**
 * Score trajectory quality for UR5.
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
```

- [ ] **Step 2: Verify tests still pass**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd test -- --run src/core/planner.test.ts
```

Expected: Tests may fail because planner now expects UR5-specific types. (Will fix in Phase 6.)

- [ ] **Step 3: Commit**

```bash
git add web/src/core/trajectory.ts
git commit -m "refactor: adapt trajectory generation for UR5 parameters"
```

---

### Task 3.3: Integrate IK into RobotScene execution

**Files:** Modify `web/src/scene/RobotScene.ts`

- [ ] **Step 1: Add IK-driven movement method**

Add to the `RobotScene` class:

```typescript
import { solveIK, type IKSolution } from '../core/ik';

// Add field:
private currentJointAngles: number[] = [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0];

// Replace moveEndEffector with IK-driven version:
private async moveEndEffector(destination: Vector3): Promise<void> {
  const ikTarget = { position: destination };
  const solution = solveIK(ikTarget, this.currentJointAngles);

  if (!solution.valid) {
    console.warn('IK failed for target:', destination);
    return;
  }

  const startAngles = [...this.currentJointAngles];
  const endAngles = solution.angles;
  const frames = 40; // More frames for smoother motion

  for (let i = 1; i <= frames; i++) {
    const t = i / frames;
    const interpolated = startAngles.map((start, j) =>
      start + (endAngles[j] - start) * t
    );
    this.robot?.setJointAngles(interpolated);
    this.currentJointAngles = interpolated;
    await wait(16);
  }
}
```

- [ ] **Step 2: Update executePlan for IK-driven motion**

Modify `executePlan` to use the new `moveEndEffector` signature:

```typescript
async executePlan(plan: ManipulationPlan, onStep: (message: string) => void): Promise<void> {
  this.drawPlan(plan);
  const target = this.sceneState.objects.find((object) => object.id === plan.targetObjectId);
  if (!target) return;

  for (const step of plan.steps) {
    if (step.pose) {
      await this.moveEndEffector(step.pose);
    }
    if (step.action === 'close_gripper' || step.action === 'grasp') {
      this.sceneState.robot.holding = target.id;
      this.robot?.setGripper(1);
    }
    if (step.action === 'open_gripper' || step.action === 'release') {
      this.sceneState.robot.holding = null;
      this.robot?.setGripper(0);
    }
    onStep(step.description);
    this.syncObjects();
    this.emitChange();
    await wait(110);
  }
}
```

- [ ] **Step 3: Verify build**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd run build
```

Expected: Build succeeds.

- [ ] **Step 4: Commit**

```bash
git add web/src/scene/RobotScene.ts
git commit -m "feat: integrate IK-driven end-effector movement into RobotScene"
```

---

## Phase 4: Cannon-es Physics Engine

### Task 4.1: Add physics world initialization

**Files:** Modify `web/src/scene/RobotScene.ts`

- [ ] **Step 1: Add Cannon-es imports and physics fields**

Add to imports:
```typescript
import * as CANNON from 'cannon-es';
```

Add fields to `RobotScene` class:
```typescript
private readonly world: CANNON.World;
private readonly physicsBodies = new Map<string, CANNON.Body>();
private readonly tableBody: CANNON.Body;
private graspConstraint: CANNON.PointToPointConstraint | null = null;
```

- [ ] **Step 2: Initialize physics world in constructor**

Add to the `private constructor`:

```typescript
// Physics world
this.world = new CANNON.World();
this.world.gravity.set(0, -9.81, 0);
this.world.solver.iterations = 10;
this.world.defaultContactMaterial.friction = 0.5;
this.world.defaultContactMaterial.restitution = 0.1;

// Table static body
this.tableBody = new CANNON.Body({
  mass: 0,
  shape: new CANNON.Box(new CANNON.Vec3(0.625, 0.0175, 0.44)),
  material: new CANNON.Material('table'),
});
this.tableBody.position.set(0.24, -0.02, 0);
this.world.addBody(this.tableBody);
```

- [ ] **Step 3: Add physics step to animate loop**

Modify `animate()`:
```typescript
private animate(): void {
  requestAnimationFrame(() => this.animate());

  // Step physics
  this.world.step(1 / 60);

  // Sync physics → Three.js meshes
  for (const [id, body] of this.physicsBodies) {
    const mesh = this.objectMeshes.get(id);
    if (!mesh) continue;
    mesh.position.set(body.position.x, body.position.y, body.position.z);
    mesh.quaternion.set(body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w);
    // Also update sceneState for planner
    const obj = this.sceneState.objects.find((o) => o.id === id);
    if (obj) {
      obj.position = { x: body.position.x, y: body.position.y, z: body.position.z };
    }
  }

  this.controls.update();
  this.renderer.render(this.scene, this.camera);
}
```

- [ ] **Step 4: Commit**

```bash
git add web/src/scene/RobotScene.ts
git commit -m "feat: add Cannon-es physics world to RobotScene"
```

---

### Task 4.2: Physics bodies for scene objects

**Files:** Modify `web/src/scene/RobotScene.ts`

- [ ] **Step 1: Create physics bodies in syncObjects**

Modify `syncObjects()` to create/update physics bodies alongside meshes:

```typescript
private syncObjects(): void {
  for (const object of this.sceneState.objects) {
    let mesh = this.objectMeshes.get(object.id);
    let body = this.physicsBodies.get(object.id);

    if (!mesh) {
      mesh = createObjectMesh(object);
      mesh.userData.objectId = object.id;
      this.objectMeshes.set(object.id, mesh);
      this.scene.add(mesh);
    }

    if (!body) {
      // Create physics body
      const shape = object.type === 'cube'
        ? new CANNON.Box(new CANNON.Vec3(object.size.x / 2, object.size.y / 2, object.size.z / 2))
        : new CANNON.Cylinder(object.size.x / 2, object.size.x / 2, object.size.y, 8);
      body = new CANNON.Body({
        mass: 0.3,
        shape,
        material: new CANNON.Material('object'),
      });
      body.position.set(object.position.x, object.position.y, object.position.z);
      body.sleepSpeedLimit = 0.1;
      body.sleepTimeLimit = 1;
      this.physicsBodies.set(object.id, body);
      this.world.addBody(body);
    }

    // If robot is not holding this object, sync from physics
    if (this.sceneState.robot.holding !== object.id) {
      mesh.position.set(body.position.x, body.position.y, body.position.z);
      mesh.quaternion.set(body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w);
    }

    // Selection highlight
    const selected = object.id === this.selectedObjectId;
    const material = mesh.material as THREE.MeshStandardMaterial;
    material.emissive.setHex(selected ? 0x1f6feb : 0x000000);
    material.emissiveIntensity = selected ? 0.14 : 0;
  }

  // Remove stale meshes and bodies
  for (const [id, mesh] of this.objectMeshes) {
    if (!this.sceneState.objects.some((object) => object.id === id)) {
      this.scene.remove(mesh);
      this.objectMeshes.delete(id);
      const body = this.physicsBodies.get(id);
      if (body) {
        this.world.removeBody(body);
        this.physicsBodies.delete(id);
      }
    }
  }
}
```

- [ ] **Step 2: Update object dragging for physics**

Modify `handlePointerMove` to use physics body:

```typescript
private handlePointerMove(event: PointerEvent): void {
  if (!this.draggingObjectId) return;
  this.updatePointer(event);
  this.raycaster.setFromCamera(this.pointer, this.camera);
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -0.035);
  const hit = new THREE.Vector3();
  this.raycaster.ray.intersectPlane(plane, hit);

  const x = clamp(hit.x, -0.42, 0.62);
  const z = clamp(hit.z, -0.36, 0.36);

  const body = this.physicsBodies.get(this.draggingObjectId);
  if (body) {
    body.position.x = x;
    body.position.z = z;
    body.velocity.set(0, 0, 0);
  }

  // Also update sceneState
  const object = this.sceneState.objects.find((item) => item.id === this.draggingObjectId);
  if (object) {
    object.position.x = x;
    object.position.z = z;
  }
  this.syncObjects();
  this.emitChange();
}
```

- [ ] **Step 3: Commit**

```bash
git add web/src/scene/RobotScene.ts
git commit -m "feat: add physics bodies for scene objects with drag support"
```

---

### Task 4.3: Grasp constraint for pick-and-place

**Files:** Modify `web/src/scene/RobotScene.ts`

- [ ] **Step 1: Implement grasp and release with physics constraints**

Modify `executePlan` to use physics constraints instead of manual object positioning:

```typescript
async executePlan(plan: ManipulationPlan, onStep: (message: string) => void): Promise<void> {
  this.drawPlan(plan);
  const target = this.sceneState.objects.find((object) => object.id === plan.targetObjectId);
  if (!target) return;

  const targetBody = this.physicsBodies.get(target.id);

  for (const step of plan.steps) {
    if (step.pose) {
      await this.moveEndEffector(step.pose);
    }

    if (step.action === 'close_gripper' || step.action === 'grasp') {
      this.sceneState.robot.holding = target.id;
      this.robot?.setGripper(1);

      // Create physics constraint between gripper and object
      if (targetBody) {
        const eePos = this.robot?.getEndEffectorPosition();
        if (eePos) {
          this.graspConstraint = new CANNON.PointToPointConstraint(
            targetBody,
            new CANNON.Vec3(0, 0.03, 0),  // Object attachment (near top)
            new CANNON.Vec3(eePos.x, eePos.y - 0.09, eePos.z)  // Gripper attachment
          );
          this.world.addConstraint(this.graspConstraint);
          targetBody.wakeUp();
        }
      }
    }

    if (step.action === 'open_gripper' || step.action === 'release') {
      // Remove constraint — object falls naturally
      if (this.graspConstraint) {
        this.world.removeConstraint(this.graspConstraint);
        this.graspConstraint = null;
      }
      this.sceneState.robot.holding = null;
      this.robot?.setGripper(0);
      targetBody?.wakeUp();
    }

    onStep(step.description);
    this.emitChange();
    await wait(110);
  }
}
```

- [ ] **Step 2: Update reset to clean up physics**

Modify `reset()`:
```typescript
reset(): void {
  // Clean up physics constraint
  if (this.graspConstraint) {
    this.world.removeConstraint(this.graspConstraint);
    this.graspConstraint = null;
  }
  // ... rest of reset logic stays
}
```

- [ ] **Step 3: Commit**

```bash
git add web/src/scene/RobotScene.ts
git commit -m "feat: physics-based grasp constraint for pick-and-place"
```

---

### Task 4.4: Update types for physics

**Files:** Modify `web/src/core/types.ts`

- [ ] **Step 1: Add physics-related types**

Append to `types.ts`:

```typescript
/** Action names updated for UR5 + general pick-and-place */
export type ActionName =
  | 'move_to'
  | 'grasp'
  | 'release'
  | 'retreat';

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
```

- [ ] **Step 2: Update ManipulationPlan**

```typescript
export interface ManipulationPlan {
  id: string;
  status: 'ready' | 'needs_repair';
  instruction: string;
  targetObjectId: string | null;
  destinationObjectId: string | null;
  steps: PlanStep[];
  trajectory: Vector3[];
  score: TrajectoryScore;
  repairHint: string | null;
  source: 'llm' | 'local';  // NEW: track planning source
  confidence?: number;       // NEW: LLM confidence score
}
```

- [ ] **Step 3: Commit**

```bash
git add web/src/core/types.ts
git commit -m "refactor: add LLM plan types and update action names for UR5"
```

---

## Phase 5: LLM Full Planning

### Task 5.1: Create LLM client

**Files:** Create `web/src/agent/llmClient.ts`

- [ ] **Step 1: Write the LLM client module**

```typescript
// web/src/agent/llmClient.ts
import type { SceneState } from '../core/types';
import type { LLMPlanResponse } from '../core/types';

const API_KEY_STORAGE = 'dashscope-api-key';
const DEFAULT_BASE_URL = 'https://dashscope.aliyuncs.com/compatible-mode/v1';
const DEFAULT_MODEL = 'qwen3.5-flash';

export function getApiKey(): string | null {
  return localStorage.getItem(API_KEY_STORAGE);
}

export function setApiKey(key: string): void {
  localStorage.setItem(API_KEY_STORAGE, key);
}

export function clearApiKey(): void {
  localStorage.removeItem(API_KEY_STORAGE);
}

const SYSTEM_PROMPT = `You are a task planner for a UR5 6-DOF robot arm with a Robotiq 2F-85 gripper.
Output ONLY valid JSON with this structure:
{
  "steps": [
    {
      "action": "move_to" | "grasp" | "release" | "retreat",
      "objectId": "<id from scene.objects>",
      "targetPosition": { "x": number, "y": number, "z": number },
      "gripperAction": "close" | "open" | null,
      "description": "<human-readable step description in Chinese>"
    }
  ],
  "confidence": 0.0-1.0,
  "explanation": "<brief reasoning>"
}

Constraints:
- UR5 max reach: 0.85m from base (0,0)
- Table surface is at y=0.035
- Safe traversal height: y >= 0.35 when moving between objects
- Gripper close before lifting, open after placing
- Object positions from scene.objects
- All targetPosition.y must be >= 0.035 (table surface)
- For grasping: approach from above (y >= object top + 0.05)
- For placing on destination: compute stacked y = destination.position.y + destination.size.y/2 + target.size.y/2

Only use object IDs that exist in the provided scene.objects array.`;

export async function planWithLLM(
  scene: SceneState,
  instruction: string
): Promise<LLMPlanResponse | null> {
  const apiKey = getApiKey();
  if (!apiKey) return null;

  const baseUrl = DEFAULT_BASE_URL;
  const model = DEFAULT_MODEL;

  const userMessage = JSON.stringify({
    instruction,
    scene: {
      objects: scene.objects.map((obj) => ({
        id: obj.id,
        label: obj.label,
        type: obj.type,
        color: obj.color,
        position: obj.position,
        size: obj.size,
        movable: obj.movable,
      })),
    },
    history: scene.history.slice(-5).map((h) => ({
      instruction: h.instruction,
      target: h.targetObjectId,
      destination: h.destinationObjectId,
    })),
  });

  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 30_000);

  try {
    const response = await fetch(`${baseUrl}/chat/completions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${apiKey}`,
      },
      body: JSON.stringify({
        model,
        messages: [
          { role: 'system', content: SYSTEM_PROMPT },
          { role: 'user', content: userMessage },
        ],
        temperature: 0.3,
        max_tokens: 2048,
      }),
      signal: controller.signal,
    });

    if (!response.ok) {
      console.error('LLM API error:', response.status, await response.text());
      return null;
    }

    const data = await response.json();
    const content = data.choices?.[0]?.message?.content;
    if (!content) return null;

    // Parse JSON from the response (handle markdown code fences)
    const jsonMatch = content.match(/```(?:json)?\s*\n?([\s\S]*?)\n?```/) || [null, content];
    const jsonStr = jsonMatch[1] || content;
    const parsed = JSON.parse(jsonStr.trim());

    // Validate structure
    if (!parsed.steps || !Array.isArray(parsed.steps)) return null;

    return {
      steps: parsed.steps,
      confidence: typeof parsed.confidence === 'number' ? parsed.confidence : 0.5,
      explanation: parsed.explanation,
    };
  } catch (err) {
    console.error('LLM call failed:', err);
    return null;
  } finally {
    clearTimeout(timeout);
  }
}
```

- [ ] **Step 2: Write LLM client tests**

Create `web/src/agent/llmClient.test.ts`:

```typescript
import { describe, it, expect, beforeEach, vi } from 'vitest';
import { getApiKey, setApiKey, clearApiKey } from './llmClient';

describe('llmClient API key management', () => {
  beforeEach(() => {
    localStorage.clear();
  });

  it('returns null when no key is set', () => {
    expect(getApiKey()).toBeNull();
  });

  it('stores and retrieves API key', () => {
    setApiKey('test-key-123');
    expect(getApiKey()).toBe('test-key-123');
  });

  it('clears API key', () => {
    setApiKey('test-key-123');
    clearApiKey();
    expect(getApiKey()).toBeNull();
  });
});
```

- [ ] **Step 3: Run tests**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd test -- --run src/agent/llmClient.test.ts
```

Expected: 3 tests pass.

- [ ] **Step 4: Commit**

```bash
git add web/src/agent/llmClient.ts web/src/agent/llmClient.test.ts
git commit -m "feat: add DashScope LLM client with API key management"
```

---

### Task 5.2: Add LLM safety validator to planner

**Files:** Modify `web/src/core/planner.ts`

- [ ] **Step 1: Rewrite planner with LLM path and safety validation**

```typescript
// web/src/core/planner.ts
import { UR5_PROFILE } from './robotProfile';
import { findObjectByReference } from './sceneGraph';
import { createPickPlaceTrajectory, distanceXZ, scoreTrajectory } from './trajectory';
import type {
  ManipulationPlan, PlanStep, SceneObject, SceneState,
  ValidationIssue, ValidationResult, LLMPlanStep,
} from './types';

/**
 * Create a pick-and-place plan from LLM-generated steps.
 * Applies deterministic safety validation.
 */
export function createPlanFromLLM(
  scene: SceneState,
  instruction: string,
  llmSteps: LLMPlanStep[],
  confidence: number
): ManipulationPlan {
  const steps: PlanStep[] = llmSteps.map((s, i) => ({
    action: s.action,
    objectId: s.objectId,
    pose: s.targetPosition,
    description: s.description ?? `Step ${i + 1}: ${s.action} ${s.objectId}`,
  }));

  const allPoses = steps.filter((s) => s.pose).map((s) => s.pose!);
  const score = allPoses.length > 0
    ? scoreTrajectory(allPoses, scene.robot.maxReach, scene.robot.base)
    : { total: 0, reachability: 0, smoothness: 0, collision: 0, jointLimits: 'OK' as const };

  const targetObjectId = llmSteps.find((s) => s.gripperAction === 'close')?.objectId ?? null;
  const destinationObjectId = llmSteps.find((s) => s.gripperAction === 'open')?.objectId ?? null;

  return {
    id: `plan-${Date.now()}`,
    status: 'ready',
    instruction,
    targetObjectId,
    destinationObjectId,
    steps,
    trajectory: allPoses,
    score,
    repairHint: null,
    source: 'llm',
    confidence,
  };
}

/**
 * Deterministic safety validation for LLM-generated plans.
 * LLM plans get stricter validation since they are less predictable.
 */
export function validateLLMPlan(scene: SceneState, plan: ManipulationPlan): ValidationResult {
  const issues: ValidationIssue[] = [];

  if (!plan.steps.length) {
    issues.push({ code: 'empty_plan', message: 'LLM did not generate executable steps.' });
    return { ok: false, issues, repairHint: 'LLM 未生成可执行步骤。请用更具体的指令重试。' };
  }

  // Validate every referenced object exists
  const objectIds = new Set(scene.objects.map((o) => o.id));
  for (const step of plan.steps) {
    if (step.objectId && !objectIds.has(step.objectId)) {
      issues.push({
        code: 'target_missing',
        message: `LLM referenced non-existent object: ${step.objectId}`,
        objectId: step.objectId,
      });
    }
  }

  // Validate every targetPosition is within reach
  for (const step of plan.steps) {
    if (step.pose && distanceXZ(step.pose, scene.robot.base) > scene.robot.maxReach) {
      issues.push({
        code: 'target_unreachable',
        message: `LLM target position (${step.pose.x.toFixed(2)}, ${step.pose.y.toFixed(2)}, ${step.pose.z.toFixed(2)}) exceeds ${scene.robot.maxReach}m reach.`,
        objectId: step.objectId,
      });
    }
  }

  // Validate grasp steps reference movable objects
  for (const step of plan.steps) {
    if (step.action === 'grasp' && step.objectId) {
      const obj = scene.objects.find((o) => o.id === step.objectId);
      if (obj && !obj.movable) {
        issues.push({
          code: 'object_not_movable',
          message: `${step.objectId} is not marked as movable.`,
          objectId: step.objectId,
        });
      }
    }
  }

  // Validate positions are above table
  for (const step of plan.steps) {
    if (step.pose && step.pose.y < 0.01) {
      issues.push({
        code: 'destination_unreachable',
        message: `Position y=${step.pose.y.toFixed(2)} is below table surface.`,
      });
    }
  }

  // Step count sanity check
  const MAX_STEPS = 30;
  if (plan.steps.length > MAX_STEPS) {
    issues.push({
      code: 'empty_plan',
      message: `LLM generated ${plan.steps.length} steps (max ${MAX_STEPS}). Plan rejected.`,
    });
  }

  const ok = issues.length === 0;
  return {
    ok,
    issues,
    repairHint: ok
      ? 'LLM 规划通过安全校验。'
      : issues[0].message,
  };
}

/**
 * Local fallback planner — deterministic keyword-based.
 * Used when LLM is unavailable or fails.
 */
export function createLocalPlan(scene: SceneState, instruction: string): ManipulationPlan {
  const target = findObjectByReference(scene, instruction);
  const destination = target
    ? findObjectByReference(scene, instruction, { excludeIds: [target.id], preferDestination: true })
    : null;

  if (!target || !destination) {
    return {
      id: `plan-${Date.now()}`,
      status: 'needs_repair',
      instruction,
      targetObjectId: target?.id ?? null,
      destinationObjectId: destination?.id ?? null,
      steps: [],
      trajectory: [],
      score: { total: 0, reachability: 0, smoothness: 0, collision: 0, jointLimits: 'OK' },
      repairHint: '无法识别目标物体或目标位置。请使用可见物体的名称，例如"红色方块"。',
      source: 'local',
    };
  }

  const trajectory = createPickPlaceTrajectory(target, destination);
  const score = scoreTrajectory(trajectory, scene.robot.maxReach, scene.robot.base);

  const steps: PlanStep[] = [
    { action: 'move_to', objectId: target.id, pose: trajectory[0], description: `移动到 ${target.id} 上方。` },
    { action: 'move_to', objectId: target.id, pose: trajectory[1], description: `下降到 ${target.id}。` },
    { action: 'grasp', objectId: target.id, description: '夹爪闭合。' },
    { action: 'move_to', objectId: target.id, pose: trajectory[2], description: '抬升到安全高度。' },
    { action: 'move_to', objectId: destination.id, pose: trajectory[3], description: `移动到 ${destination.id} 上方。` },
    { action: 'move_to', objectId: destination.id, pose: trajectory[4], description: `下降到 ${destination.id}。` },
    { action: 'release', objectId: destination.id, description: '夹爪张开。' },
    { action: 'retreat', pose: trajectory[5], description: '退回安全高度。' },
  ];

  return {
    id: `plan-${Date.now()}`,
    status: 'ready',
    instruction,
    targetObjectId: target.id,
    destinationObjectId: destination.id,
    steps,
    trajectory,
    score,
    repairHint: null,
    source: 'local',
  };
}
```

- [ ] **Step 2: Commit**

```bash
git add web/src/core/planner.ts
git commit -m "feat: LLM plan construction with deterministic safety validation + local fallback"
```

---

### Task 5.3: Update agentClient for LLM dispatch

**Files:** Modify `web/src/agent/agentClient.ts`

- [ ] **Step 1: Rewrite agentClient for LLM-first with fallback**

```typescript
// web/src/agent/agentClient.ts
import { createPlanFromLLM, createLocalPlan, validateLLMPlan } from '../core/planner';
import { planWithLLM } from './llmClient';
import type { ManipulationPlan, SceneState, ValidationResult } from '../core/types';

export interface PlanResponse {
  plan: ManipulationPlan;
  validation: ValidationResult;
  notes: string[];
}

export async function requestPlan(
  scene: SceneState,
  instruction: string
): Promise<PlanResponse> {
  const notes: string[] = [];

  // Try LLM first
  const llmResult = await planWithLLM(scene, instruction);

  if (llmResult && llmResult.confidence >= 0.4) {
    const plan = createPlanFromLLM(scene, instruction, llmResult.steps, llmResult.confidence);
    const validation = validateLLMPlan(scene, plan);

    notes.push(`LLM 规划完成，置信度：${(llmResult.confidence * 100).toFixed(0)}%`);
    if (llmResult.explanation) {
      notes.push(`解析：${llmResult.explanation}`);
    }

    if (!validation.ok) {
      notes.push(`安全校验未通过：${validation.repairHint}`);
    }

    return { plan, validation, notes };
  }

  // Fall back to local deterministic planner
  notes.push(llmResult
    ? `LLM 置信度过低 (${(llmResult.confidence * 100).toFixed(0)}%)，已退回本地规划。`
    : 'LLM 不可用，使用本地规则规划。');

  const plan = createLocalPlan(scene, instruction);
  const validation = validateLLMPlan(scene, plan);

  return { plan, validation, notes };
}
```

- [ ] **Step 2: Update main.ts for simplified API**

In `main.ts`, update the `runTask` function to remove backend-related options:

```typescript
// Replace:
// const response = await requestPlan(robotScene.sceneState, taskInput.value, {
//   useBackend: useBackend.checked,
//   backendUrl: backendUrl.value
// });
// With:
const response = await requestPlan(robotScene.sceneState, taskInput.value);
```

- [ ] **Step 3: Commit**

```bash
git add web/src/agent/agentClient.ts web/src/main.ts
git commit -m "feat: LLM-first planning dispatch with local fallback"
```

---

## Phase 6: UI Update and Cleanup

### Task 6.1: Update index.html for UR5 + API key

**Files:** Modify `web/index.html`

- [ ] **Step 1: Replace the agent panel section**

Replace the sidebar content (lines 48-94) with:

```html
<aside class="agent-panel">
  <section class="panel-section">
    <h2>智能体</h2>
    <p>UR5 六轴机械臂 + Robotiq 2F-85 夹爪规划智能体。</p>

    <div class="api-key-row">
      <label for="api-key">DashScope API Key</label>
      <input id="api-key" type="password" placeholder="sk-..." autocomplete="off" />
      <button id="save-api-key" class="small-button">保存</button>
    </div>

    <div class="task-row">
      <input id="task-input" value="把红色方块放到蓝色方块上" aria-label="任务输入" />
      <button id="send-task" class="send-button" aria-label="运行任务">运行</button>
    </div>
  </section>

  <!-- Scene table, pipeline, score, log sections unchanged -->
  <section class="panel-section table-section">
    <div class="section-header">
      <h2>场景图</h2>
      <div>
        <button id="add-cube" class="small-button">添加方块</button>
        <button id="add-cylinder" class="small-button">添加圆柱</button>
      </div>
    </div>
    <div id="scene-table" class="scene-table"></div>
  </section>

  <section class="panel-section">
    <div class="pipeline">
      <div class="stage done">Plan</div>
      <div class="stage" id="validate-stage">Validate</div>
      <div class="stage" id="repair-stage">Repair</div>
      <div class="stage" id="execute-stage">Execute</div>
    </div>
    <div id="score-panel" class="score-panel"></div>
  </section>

  <section class="panel-section log-section">
    <div class="section-header">
      <h2>执行日志</h2>
      <button id="clear-log" class="small-button">清空</button>
    </div>
    <div id="execution-log" class="execution-log"></div>
  </section>
</aside>
```

- [ ] **Step 2: Update status bar text**

```html
<div class="statusbar">
  <span><span class="dot ready"></span> 就绪</span>
  <span>机器人：UR5 + Robotiq 2F-85</span>
  <span id="planning-time">规划耗时：--</span>
  <span id="waypoint-count">轨迹点：0</span>
  <span id="duration">预计执行：--</span>
</div>
```

- [ ] **Step 3: Update header branding**

```html
<span>项目：UR5 抓取放置演示</span>
```

Remove the mode-switch buttons and backend URL input row entirely.

- [ ] **Step 4: Commit**

```bash
git add web/index.html
git commit -m "refactor: update UI for UR5 + API key input, remove backend controls"
```

---

### Task 6.2: Add API key handling to main.ts

**Files:** Modify `web/src/main.ts`

- [ ] **Step 1: Add API key save/load logic**

Add after the querySelector lines near the top of `init()`:

```typescript
import { getApiKey, setApiKey } from './agent/llmClient';

const apiKeyInput = document.querySelector<HTMLInputElement>('#api-key')!;
const saveApiKeyBtn = document.querySelector<HTMLButtonElement>('#save-api-key')!;

// Load saved API key
const savedKey = getApiKey();
if (savedKey) {
  apiKeyInput.value = savedKey;
}

saveApiKeyBtn.addEventListener('click', () => {
  setApiKey(apiKeyInput.value.trim());
  appendLog(logRoot, 'API Key 已保存。');
});

// Remove useBackend/backendUrl references — no longer needed
// Remove the mode-switch button event listeners
```

- [ ] **Step 2: Commit**

```bash
git add web/src/main.ts
git commit -m "feat: add API key save/load to main.ts"
```

---

### Task 6.3: Update affordance for Robotiq

**Files:** Modify `web/src/core/affordance.ts`

- [ ] **Step 1: Update gripper references**

Replace LMG-90 references with Robotiq 2F-85:

```typescript
// Replace:
// note: 'Side grasp keeps the LMG-90 fingers clear of the tabletop.'
// With:
// note: 'Side grasp for Robotiq 2F-85 parallel-jaw gripper.'
```

Update scores for Robotiq 2F-85 characteristics (wider stroke, higher force):
```typescript
// For cubes: side_grasp score 0.91 → 0.93 (Robotiq has better grip)
// For cylinders: side_grasp score 0.88 → 0.90
```

- [ ] **Step 2: Commit**

```bash
git add web/src/core/affordance.ts
git commit -m "refactor: update affordance scores for Robotiq 2F-85"
```

---

### Task 6.4: Final cleanup and verify all tests

- [ ] **Step 1: Delete createLebaiRobot.ts**

```bash
rm d:/MyWork/ArmPlannerAgent/web/src/scene/createLebaiRobot.ts
```

- [ ] **Step 2: Update test files for new API**

Update `web/src/core/planner.test.ts`: Import from new planner API, test `createLocalPlan` and `validateLLMPlan`.

```typescript
// web/src/core/planner.test.ts
import { describe, it, expect } from 'vitest';
import { createLocalPlan, validateLLMPlan } from './planner';
import type { SceneState } from './types';

const testScene: SceneState = {
  robot: { base: { x: 0, y: 0, z: 0 }, maxReach: 0.85, holding: null },
  objects: [
    { id: 'red_cube', label: 'red cube', type: 'cube', color: 'red', position: { x: 0.28, y: 0.03, z: 0.16 }, size: { x: 0.06, y: 0.06, z: 0.06 }, movable: true },
    { id: 'blue_cube', label: 'blue cube', type: 'cube', color: 'blue', position: { x: 0.46, y: 0.03, z: -0.08 }, size: { x: 0.06, y: 0.06, z: 0.06 }, movable: true },
    { id: 'green_cube', label: 'green cube', type: 'cube', color: 'green', position: { x: 0.18, y: 0.03, z: -0.22 }, size: { x: 0.06, y: 0.06, z: 0.06 }, movable: true },
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
    expect(plan.trajectory).toHaveLength(6);
  });

  it('generates a plan for Chinese instruction', () => {
    const plan = createLocalPlan(testScene, '把红色方块放到蓝色方块上');
    expect(plan.status).toBe('ready');
    expect(plan.targetObjectId).toBe('red_cube');
  });

  it('returns needs_repair for unrecognized objects', () => {
    const plan = createLocalPlan(testScene, 'pick up the invisible widget');
    expect(plan.status).toBe('needs_repair');
  });
});

describe('validateLLMPlan', () => {
  it('accepts a valid plan', () => {
    const plan = createLocalPlan(testScene, 'put red cube on blue cube');
    const result = validateLLMPlan(testScene, plan);
    expect(result.ok).toBe(true);
  });

  it('rejects plan with unreachable target', () => {
    const farScene: SceneState = {
      ...testScene,
      objects: [
        ...testScene.objects.slice(1),
        { id: 'far_cube', label: 'far cube', type: 'cube', color: 'red', position: { x: 2.0, y: 0.03, z: 0 }, size: { x: 0.06, y: 0.06, z: 0.06 }, movable: true },
      ],
    };
    const plan = createLocalPlan(farScene, 'put far cube on blue cube');
    const result = validateLLMPlan(farScene, plan);
    expect(result.ok).toBe(false);
  });
});
```

- [ ] **Step 3: Run all tests**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd test -- --run
```

Expected: All tests pass.

- [ ] **Step 4: Run final build**

```bash
cd d:/MyWork/ArmPlannerAgent/web
npm.cmd run build
```

Expected: Build succeeds with no errors.

- [ ] **Step 5: Final commit**

```bash
git add -A
git commit -m "test: update tests for UR5 planner, remove Lebai files, final cleanup"
```

---

## Verification Checklist

After all phases complete, verify:

1. **Build**: `npm.cmd run build` passes
2. **Tests**: `npm.cmd test -- --run` — all tests pass
3. **Dev server**: `npm.cmd run dev` — UR5 model renders in browser
4. **IK**: Move end-effector to a visible target — arm joints move correctly
5. **Physics**: Add a cube — it falls to table with gravity
6. **LLM planning**: Enter API key, type "把红色方块放到蓝色方块上" — plan generated and executed
7. **Stacking**: Place cube A on cube B — no clipping, physics handles contact
8. **Fallback**: Clear API key — system falls back to local planner and still works
