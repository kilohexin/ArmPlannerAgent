import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { ManipulationPlan, SceneObject, SceneState, Vector3, VisualSnapshot } from '../core/types';
import { createDefaultRobotState } from '../core/robotProfile';
import { heldObjectCenter, stackedCenter } from '../core/trajectory';
import { graspCenterFromToolTarget, toolTargetFromGraspCenter, validateGraspCenter } from '../core/grasp';
import type { GraspSpec } from '../core/types';
import { inverseKinematicsLm3, LM3_HOME_JOINTS } from '../core/lm3Kinematics';
import { loadRobotModel, type RobotModel } from './robotModel';

const COLORS: Record<string, number> = {
  red: 0xe3483e,
  blue: 0x1d75d8,
  green: 0x37a957,
  yellow: 0xf0c849,
  purple: 0x8b5ccf,
};

const HOME_JOINTS = [...LM3_HOME_JOINTS];

export class RobotScene {
  readonly sceneState: SceneState;
  private readonly root: HTMLElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene: THREE.Scene;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly eyeToHandCamera: THREE.PerspectiveCamera;
  private readonly eyeToHandTarget = new THREE.Vector3(0.24, 0.18, 0.02);
  private readonly eyeToHandHelper: THREE.Object3D;
  private readonly controls: OrbitControls;
  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private readonly objectMeshes = new Map<string, THREE.Mesh>();
  private readonly trajectoryGroup = new THREE.Group();
  private readonly world: CANNON.World;
  private readonly physicsBodies = new Map<string, CANNON.Body>();
  private readonly gripperAnchor: CANNON.Body;
  private robot: RobotModel | null = null;
  private selectedObjectId: string | null = null;
  private draggingObjectId: string | null = null;
  private onChange: (() => void) | null = null;
  private currentJointAngles: number[] = [...HOME_JOINTS];
  private activeGrasp: GraspSpec | null = null;
  private commandedGraspCenter: Vector3 | null = null;
  private heldObjectOffset: Vector3 | null = null;

  static async create(root: HTMLElement): Promise<RobotScene> {
    const scene = new RobotScene(root);
    scene.robot = await loadRobotModel();
    scene.scene.add(scene.robot.group);
    scene.robot.setJointAngles(HOME_JOINTS);
    scene.robot.setGripper(0);
    scene.syncRobotState();
    return scene;
  }

  private constructor(root: HTMLElement) {
    this.root = root;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xfbfcfe);

    this.camera = new THREE.PerspectiveCamera(48, 1, 0.01, 20);
    this.camera.position.set(1.38, 1.05, 1.22);

    this.eyeToHandCamera = new THREE.PerspectiveCamera(68, 4 / 3, 0.01, 8);
    this.eyeToHandCamera.position.set(1.18, 0.9, 0.9);
    this.eyeToHandCamera.lookAt(this.eyeToHandTarget);
    this.eyeToHandHelper = createEyeToHandMarker(this.eyeToHandCamera.position, this.eyeToHandTarget);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.root.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.24, 0.28, 0.02);
    this.controls.enableDamping = true;

    this.sceneState = {
      robot: createDefaultRobotState(),
      objects: createSeedObjects(),
      history: [],
    };

    this.world = new CANNON.World();
    this.world.gravity.set(0, -9.81, 0);
    (this.world.solver as CANNON.GSSolver).iterations = 10;
    this.world.defaultContactMaterial.friction = 0.7;
    this.world.defaultContactMaterial.restitution = 0.05;

    const tableBody = new CANNON.Body({
      mass: 0,
      shape: new CANNON.Box(new CANNON.Vec3(0.625, 0.0175, 0.44)),
      material: new CANNON.Material('table'),
    });
    tableBody.position.set(0.24, -0.02, 0);
    this.world.addBody(tableBody);

    this.gripperAnchor = new CANNON.Body({
      mass: 0,
      type: CANNON.Body.KINEMATIC,
    });
    this.gripperAnchor.position.set(0, 0.3, 0);
    this.world.addBody(this.gripperAnchor);

    this.scene.add(this.trajectoryGroup);
    this.setupWorld();
    this.syncObjects();
    this.bindEvents();
    this.resize();
    this.animate();
  }

  setChangeHandler(handler: () => void): void {
    this.onChange = handler;
  }

  getSelectedObjectId(): string | null {
    return this.selectedObjectId;
  }

  setView(view: 'top' | 'front' | 'iso' | 'eye'): void {
    this.eyeToHandHelper.visible = view !== 'eye';

    if (view === 'top') {
      this.camera.position.set(0.24, 1.45, 0.01);
      this.controls.target.set(0.24, 0.13, 0.02);
    } else if (view === 'front') {
      this.camera.position.set(0.24, 0.46, 1.38);
      this.controls.target.set(0.24, 0.13, 0.02);
    } else if (view === 'eye') {
      this.camera.position.copy(this.eyeToHandCamera.position);
      this.controls.target.copy(this.eyeToHandTarget);
    } else {
      this.camera.position.set(1.38, 1.05, 1.22);
      this.controls.target.set(0.24, 0.28, 0.02);
    }
    this.controls.update();
  }

  captureEyeToHandSnapshot(width = 512, height = 384): VisualSnapshot {
    const renderTarget = new THREE.WebGLRenderTarget(width, height, {
      samples: 4,
      type: THREE.UnsignedByteType,
    });
    const pixels = new Uint8Array(width * height * 4);
    const wasHelperVisible = this.eyeToHandHelper.visible;
    this.eyeToHandHelper.visible = false;

    this.renderer.setRenderTarget(renderTarget);
    this.renderer.render(this.scene, this.eyeToHandCamera);
    this.renderer.readRenderTargetPixels(renderTarget, 0, 0, width, height, pixels);
    this.renderer.setRenderTarget(null);
    this.eyeToHandHelper.visible = wasHelperVisible;
    renderTarget.dispose();

    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('无法创建相机截图画布。');
    }

    const image = ctx.createImageData(width, height);
    for (let row = 0; row < height; row++) {
      const sourceStart = row * width * 4;
      const targetStart = (height - row - 1) * width * 4;
      image.data.set(pixels.subarray(sourceStart, sourceStart + width * 4), targetStart);
    }
    ctx.putImageData(image, 0, 0);

    return {
      id: `eye-to-hand-${Date.now()}`,
      dataUrl: canvas.toDataURL('image/jpeg', 0.86),
      mimeType: 'image/jpeg',
      capturedAt: new Date().toISOString(),
      camera: {
        name: 'eye_to_hand',
        position: toVector3(this.eyeToHandCamera.position),
        target: toVector3(this.eyeToHandTarget),
      },
    };
  }

  addObject(type: SceneObject['type']): void {
    const color = nextColor(this.sceneState.objects.length);
    const id = `${color}_${type}_${this.sceneState.objects.length + 1}`;
    this.sceneState.objects.push({
      id,
      label: `${color} ${type}`,
      type,
      color,
      position: {
        x: 0.18 + Math.random() * 0.28,
        y: type === 'cube' ? 0.03 : 0.04,
        z: -0.24 + Math.random() * 0.45,
      },
      size: type === 'cube'
        ? { x: 0.06, y: 0.06, z: 0.06 }
        : { x: 0.07, y: 0.08, z: 0.07 },
      movable: true,
    });
    this.syncObjects();
    this.emitChange();
  }

  reset(): void {
    for (const mesh of this.objectMeshes.values()) {
      this.scene.remove(mesh);
    }
    for (const body of this.physicsBodies.values()) {
      this.world.removeBody(body);
    }
    this.objectMeshes.clear();
    this.physicsBodies.clear();
    this.sceneState.objects.splice(0, this.sceneState.objects.length, ...createSeedObjects());
    this.sceneState.history.splice(0);
    this.sceneState.robot.holding = null;
    this.activeGrasp = null;
    this.commandedGraspCenter = null;
    this.heldObjectOffset = null;
    this.selectedObjectId = null;
    this.clearTrajectory();
    this.syncObjects();
    this.robot?.setJointAngles(HOME_JOINTS);
    this.robot?.setGripper(0);
    this.currentJointAngles = [...HOME_JOINTS];
    this.syncRobotState();
    this.emitChange();
  }

  drawPlan(plan: ManipulationPlan): void {
    this.clearTrajectory();
    if (!plan.trajectory.length) {
      return;
    }

    // 轨迹存储的是 grasp_center 坐标，可视化时转成 TCP 坐标
    // 起点直接用 GLB 模型实际 TCP 位置，避免 IK 与 GLB 模型的固定偏差
    const currentTcp = this.robot?.getEndEffectorPosition();
    const tcpPoints = plan.trajectory.map((point, i) => {
      if (i === 0 && currentTcp) {
        return { x: currentTcp.x, y: currentTcp.y, z: currentTcp.z };
      }
      return plan.grasp
        ? toolTargetFromGraspCenter(point, plan.grasp!)
        : point;
    });
    const material = new THREE.LineBasicMaterial({ color: 0x0b6fe8 });
    const points = tcpPoints.map((point) => new THREE.Vector3(point.x, point.y, point.z));
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), material);
    this.trajectoryGroup.add(line);

    points.forEach((point, index) => {
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.014, 18, 10),
        new THREE.MeshStandardMaterial({ color: index >= 4 ? 0x2fac5a : 0x0b6fe8 })
      );
      marker.position.copy(point);
      this.trajectoryGroup.add(marker);
    });
  }

  async executePlan(plan: ManipulationPlan, onStep: (message: string) => void): Promise<boolean> {
    this.drawPlan(plan);
    const target = this.sceneState.objects.find((object) => object.id === plan.targetObjectId);
    if (!target) {
      onStep('执行失败：目标物体不存在。');
      return false;
    }

    let jointCursor = plan.cartesianWaypoints?.[0]?.id === 'start_tcp' ? 1 : 0;
    for (const step of plan.steps) {
      if (step.pose) {
        const jointTarget = plan.jointTrajectory?.[jointCursor]?.q;
        jointCursor += 1;
        const moved = await this.moveEndEffector(step.pose, jointTarget, plan.grasp);
        if (!moved) {
          onStep(`执行失败：轨迹点不可达 (${step.pose.x.toFixed(2)}, ${step.pose.y.toFixed(2)}, ${step.pose.z.toFixed(2)})。`);
          return false;
        }
      }

      if (step.action === 'grasp') {
        const grasp = plan.grasp;
        if (grasp) {
          const graspCenter = this.commandedGraspCenter ?? grasp.center;
          const result = graspCenter
            ? validateGraspCenter(target, graspCenter, grasp)
            : { ok: false, reason: '无法读取夹爪中心位置。' };
          if (!result.ok) {
            onStep(`抓取失败：${result.reason}`);
            return false;
          }
          onStep(`抓取姿态校验通过：顶部抓取，开口 ${grasp.requiredOpeningMm.toFixed(0)}mm。`);
        }
        this.activeGrasp = grasp ?? null;
        const graspCenter = this.commandedGraspCenter ?? grasp?.center ?? target.position;
        this.commandedGraspCenter = graspCenter;
        this.heldObjectOffset = {
          x: target.position.x - graspCenter.x,
          y: target.position.y - graspCenter.y,
          z: target.position.z - graspCenter.z,
        };
        this.sceneState.robot.holding = target.id;
        this.robot?.setGripper(1);
        this.disablePhysics(target.id);
        this.syncHeldObjectToEndEffector();
      }

      if (step.action === 'release') {
        const destination = this.sceneState.objects.find((object) =>
          object.id === plan.destinationObjectId || object.id === step.objectId
        );
        const releasePosition = destination ? stackedCenter(target, destination) : target.position;
        this.sceneState.robot.holding = null;
        this.activeGrasp = null;
        this.commandedGraspCenter = null;
        this.heldObjectOffset = null;
        this.placeObject(target.id, releasePosition, true);
        this.robot?.setGripper(0);
      }

      onStep(step.description);
      this.emitChange();
      await wait(110);
    }

    return true;
  }

  private async moveEndEffector(destination: Vector3, jointTarget?: number[], grasp?: GraspSpec): Promise<boolean> {
    if (grasp) {
      return this.moveGraspCenter(destination, grasp);
    }

    const toolDestination = grasp ? toolTargetFromGraspCenter(destination, grasp) : destination;
    const targetAngles = jointTarget ?? inverseKinematicsLm3(toolDestination, this.currentJointAngles).joints;
    if (!targetAngles.every((angle) => Number.isFinite(angle))) {
      console.warn('IK failed for target:', destination);
      return false;
    }

    const startAngles = [...this.currentJointAngles];
    const endAngles = targetAngles;
    const jointDistance = Math.max(...startAngles.map((s, i) => Math.abs(endAngles[i] - s)));
    const frames = Math.max(30, Math.min(80, Math.round(jointDistance * 40) + 30));
    const startGraspCenter = grasp
      ? this.commandedGraspCenter ?? { ...destination }
      : null;

    for (let i = 1; i <= frames; i++) {
      const t = easeInOut(i / frames);
      const interpolated = startAngles.map((start, index) =>
        start + (endAngles[index] - start) * t
      );
      this.robot?.setJointAngles(interpolated);
      this.currentJointAngles = interpolated;
      if (grasp && startGraspCenter) {
        this.commandedGraspCenter = {
          x: startGraspCenter.x + (destination.x - startGraspCenter.x) * t,
          y: startGraspCenter.y + (destination.y - startGraspCenter.y) * t,
          z: startGraspCenter.z + (destination.z - startGraspCenter.z) * t,
        };
      }
      this.syncHeldObjectToEndEffector();
      await wait(16);
    }

    return true;
  }

  private async moveGraspCenter(destination: Vector3, grasp: GraspSpec): Promise<boolean> {
    const start = this.commandedGraspCenter ??
      this.getCurrentGraspCenter(grasp) ??
      { ...destination };
    const distance = Math.hypot(
      destination.x - start.x,
      destination.y - start.y,
      destination.z - start.z
    );
    const frames = Math.max(30, Math.min(80, Math.round(distance * 200) + 30));

    for (let i = 1; i <= frames; i++) {
      const t = easeInOut(i / frames);
      const center = {
        x: start.x + (destination.x - start.x) * t,
        y: start.y + (destination.y - start.y) * t,
        z: start.z + (destination.z - start.z) * t,
      };
      const toolDestination = toolTargetFromGraspCenter(center, grasp);
      const result = inverseKinematicsLm3(toolDestination, this.currentJointAngles, {
        maxIterations: 120,
        targetDirection: { x: 0, y: -1, z: 0 },
      });
      if (!result.joints.every((angle) => Number.isFinite(angle))) {
        console.warn('IK failed for grasp center:', center);
        return false;
      }

      this.robot?.setJointAngles(result.joints);
      this.currentJointAngles = result.joints;
      this.commandedGraspCenter = center;
      this.syncHeldObjectToEndEffector();
      await wait(16);
    }

    return true;
  }

  private setupWorld(): void {
    const hemi = new THREE.HemisphereLight(0xffffff, 0xd7dde8, 2.2);
    this.scene.add(hemi);

    const key = new THREE.DirectionalLight(0xffffff, 2.8);
    key.position.set(0.3, 1.1, 0.6);
    key.castShadow = true;
    this.scene.add(key);

    const table = new THREE.Mesh(
      new THREE.BoxGeometry(1.25, 0.035, 0.88),
      new THREE.MeshStandardMaterial({ color: 0xe9edf2, roughness: 0.72 })
    );
    table.position.set(0.24, -0.02, 0);
    table.receiveShadow = true;
    this.scene.add(table);

    const grid = new THREE.GridHelper(1.25, 18, 0xc7ced8, 0xdde2ea);
    grid.position.set(0.24, 0.002, 0);
    this.scene.add(grid);

    this.eyeToHandHelper.name = 'eye_to_hand_camera_helper';
    this.scene.add(this.eyeToHandHelper);
  }

  private syncObjects(): void {
    const currentIds = new Set(this.sceneState.objects.map((object) => object.id));

    for (const [id, mesh] of this.objectMeshes) {
      if (!currentIds.has(id)) {
        this.scene.remove(mesh);
        this.objectMeshes.delete(id);
        const body = this.physicsBodies.get(id);
        if (body) {
          this.world.removeBody(body);
          this.physicsBodies.delete(id);
        }
      }
    }

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
        body = this.createBody(object);
        this.physicsBodies.set(object.id, body);
        this.world.addBody(body);
      }

      if (this.sceneState.robot.holding === object.id) {
        this.syncHeldObjectToEndEffector();
      } else {
        mesh.position.set(body.position.x, body.position.y, body.position.z);
        mesh.quaternion.set(body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w);
        object.position = { x: body.position.x, y: body.position.y, z: body.position.z };
      }

      const material = mesh.material as THREE.MeshStandardMaterial;
      const selected = object.id === this.selectedObjectId;
      material.emissive.setHex(selected ? 0x1f6feb : 0x000000);
      material.emissiveIntensity = selected ? 0.14 : 0;
    }
  }

  private createBody(object: SceneObject): CANNON.Body {
    const shape = object.type === 'cube'
      ? new CANNON.Box(new CANNON.Vec3(object.size.x / 2, object.size.y / 2, object.size.z / 2))
      : new CANNON.Cylinder(object.size.x / 2, object.size.x / 2, object.size.y, 16);
    const body = new CANNON.Body({
      mass: 0,
      shape,
      material: new CANNON.Material('object'),
      type: CANNON.Body.STATIC,
    });
    body.position.set(object.position.x, object.position.y, object.position.z);
    body.sleepSpeedLimit = 0.1;
    body.sleepTimeLimit = 1;
    body.sleep();
    return body;
  }

  private clearTrajectory(): void {
    this.trajectoryGroup.clear();
  }

  private bindEvents(): void {
    window.addEventListener('resize', () => this.resize());
    this.renderer.domElement.addEventListener('pointerdown', (event) => this.handlePointerDown(event));
    this.renderer.domElement.addEventListener('pointermove', (event) => this.handlePointerMove(event));
    window.addEventListener('pointerup', () => {
      this.draggingObjectId = null;
      this.controls.enabled = true;
    });
  }

  private handlePointerDown(event: PointerEvent): void {
    this.updatePointer(event);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const hits = this.raycaster.intersectObjects([...this.objectMeshes.values()]);
    if (!hits.length) {
      this.selectedObjectId = null;
      this.syncObjects();
      this.emitChange();
      return;
    }

    const id = hits[0].object.userData.objectId as string;
    this.selectedObjectId = id;
    this.draggingObjectId = id;
    this.controls.enabled = false;
    this.syncObjects();
    this.emitChange();
  }

  private handlePointerMove(event: PointerEvent): void {
    if (!this.draggingObjectId) {
      return;
    }
    this.updatePointer(event);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -0.035);
    const hit = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(plane, hit)) {
      return;
    }

    const object = this.sceneState.objects.find((item) => item.id === this.draggingObjectId);
    if (!object) {
      return;
    }

    const position = {
      x: clamp(hit.x, -0.42, 0.62),
      y: object.size.y / 2,
      z: clamp(hit.z, -0.36, 0.36),
    };
    this.placeObject(object.id, position, true);
    this.emitChange();
  }

  private updatePointer(event: PointerEvent): void {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -(((event.clientY - rect.top) / rect.height) * 2 - 1);
  }

  private resize(): void {
    const width = this.root.clientWidth;
    const height = this.root.clientHeight;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  private animate(): void {
    requestAnimationFrame(() => this.animate());
    this.world.step(1 / 60);
    this.syncRobotState();
    this.syncHeldObjectToEndEffector();

    for (const [id, body] of this.physicsBodies) {
      if (this.sceneState.robot.holding === id) {
        continue;
      }
      const mesh = this.objectMeshes.get(id);
      if (!mesh) {
        continue;
      }
      mesh.position.set(body.position.x, body.position.y, body.position.z);
      mesh.quaternion.set(body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w);
      const object = this.sceneState.objects.find((item) => item.id === id);
      if (object) {
        object.position = { x: body.position.x, y: body.position.y, z: body.position.z };
      }
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  private disablePhysics(objectId: string): void {
    const body = this.physicsBodies.get(objectId);
    if (!body) {
      return;
    }
    body.type = CANNON.Body.KINEMATIC;
    body.mass = 0;
    body.collisionResponse = false;
    body.velocity.set(0, 0, 0);
    body.angularVelocity.set(0, 0, 0);
    body.updateMassProperties();
  }

  private syncHeldObjectToEndEffector(): void {
    const heldId = this.sceneState.robot.holding;
    const graspCenter = this.commandedGraspCenter ??
      this.getCurrentGraspCenter(this.activeGrasp ?? undefined);
    if (!heldId || !graspCenter) {
      return;
    }

    const center = this.heldObjectOffset
      ? {
          x: graspCenter.x + this.heldObjectOffset.x,
          y: graspCenter.y + this.heldObjectOffset.y,
          z: graspCenter.z + this.heldObjectOffset.z,
        }
      : heldObjectCenter(graspCenter, this.activeGrasp ?? undefined);
    this.placeObject(heldId, center, false);
    this.gripperAnchor.position.set(center.x, center.y, center.z);
    this.gripperAnchor.velocity.set(0, 0, 0);
  }

  private syncRobotState(): void {
    this.sceneState.robot.joints = this.currentJointAngles.map((angle) => Math.round(angle * 10000) / 10000);

    if (!this.robot) {
      return;
    }

    this.robot.robot.updateMatrixWorld(true);
    const tcp = this.robot.getEndEffectorPosition();
    this.sceneState.robot.tcp = {
      x: Math.round(tcp.x * 10000) / 10000,
      y: Math.round(tcp.y * 10000) / 10000,
      z: Math.round(tcp.z * 10000) / 10000,
    };
  }

  private getCurrentGraspCenter(grasp?: GraspSpec): Vector3 | null {
    const eePos = this.robot?.getEndEffectorPosition();
    if (!eePos) {
      return null;
    }
    const toolPosition = { x: eePos.x, y: eePos.y, z: eePos.z };
    return grasp ? graspCenterFromToolTarget(toolPosition, grasp) : heldObjectCenter(toolPosition);
  }

  private placeObject(objectId: string, position: Vector3, enablePhysics: boolean): void {
    const object = this.sceneState.objects.find((item) => item.id === objectId);
    const mesh = this.objectMeshes.get(objectId);
    const body = this.physicsBodies.get(objectId);
    if (!object || !mesh || !body) {
      return;
    }

    object.position = { ...position };
    mesh.position.set(position.x, position.y, position.z);
    mesh.quaternion.identity();
    body.position.set(position.x, position.y, position.z);
    body.quaternion.set(0, 0, 0, 1);
    body.velocity.set(0, 0, 0);
    body.angularVelocity.set(0, 0, 0);

    if (enablePhysics) {
      body.type = CANNON.Body.STATIC;
      body.mass = 0;
      body.collisionResponse = true;
      body.updateMassProperties();
      body.sleep();
    }
  }

  private emitChange(): void {
    this.onChange?.();
  }
}

function createObjectMesh(object: SceneObject): THREE.Mesh {
  const material = new THREE.MeshStandardMaterial({
    color: COLORS[object.color] ?? 0x8b95a5,
    roughness: 0.45,
    metalness: 0.05,
  });
  const geometry =
    object.type === 'cube'
      ? new THREE.BoxGeometry(object.size.x, object.size.y, object.size.z)
      : new THREE.CylinderGeometry(object.size.x / 2, object.size.x / 2, object.size.y, 32);
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function createSeedObjects(): SceneObject[] {
  return [
    makeObject('red_cube', 'red cube', 'cube', 'red', { x: 0.28, y: 0.03, z: 0.16 }),
    makeObject('blue_cube', 'blue cube', 'cube', 'blue', { x: 0.46, y: 0.03, z: -0.08 }),
    makeObject('green_cube', 'green cube', 'cube', 'green', { x: 0.18, y: 0.03, z: -0.22 }),
    makeObject('yellow_cylinder', 'yellow cylinder', 'cylinder', 'yellow', { x: 0.54, y: 0.04, z: 0.2 }),
    makeObject('purple_cylinder', 'purple cylinder', 'cylinder', 'purple', { x: 0.05, y: 0.04, z: 0.26 }),
  ];
}

function makeObject(
  id: string,
  label: string,
  type: SceneObject['type'],
  color: string,
  position: Vector3
): SceneObject {
  return {
    id,
    label,
    type,
    color,
    position,
    size: type === 'cube' ? { x: 0.06, y: 0.06, z: 0.06 } : { x: 0.07, y: 0.08, z: 0.07 },
    movable: true,
  };
}

function nextColor(index: number): string {
  return ['red', 'blue', 'green', 'yellow', 'purple'][index % 5];
}

function createEyeToHandMarker(position: THREE.Vector3, target: THREE.Vector3): THREE.Object3D {
  const group = new THREE.Group();

  const poleBase = new THREE.Vector3(position.x, 0.003, 0.4);
  const poleTop = new THREE.Vector3(position.x, position.y - 0.02, poleBase.z);
  const standMaterial = new THREE.MeshStandardMaterial({ color: 0x3b4658, roughness: 0.55 });
  const cameraMaterial = new THREE.MeshStandardMaterial({ color: 0x223047, roughness: 0.5 });
  const lensMaterial = new THREE.MeshStandardMaterial({ color: 0x0b6fe8, roughness: 0.35 });

  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.032, 0.032, 0.01, 32),
    standMaterial
  );
  base.position.copy(poleBase);
  base.castShadow = true;
  group.add(base);

  const poleHeight = poleTop.y - poleBase.y;
  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.005, 0.005, poleHeight, 18),
    standMaterial
  );
  pole.position.set(poleBase.x, poleBase.y + poleHeight / 2, poleBase.z);
  pole.castShadow = true;
  group.add(pole);

  const bracketStart = poleTop;
  const bracketEnd = new THREE.Vector3(position.x, position.y, position.z - 0.035);
  const bracketVector = bracketEnd.clone().sub(bracketStart);
  const bracket = new THREE.Mesh(
    new THREE.CylinderGeometry(0.004, 0.004, bracketVector.length(), 16),
    standMaterial
  );
  bracket.position.copy(bracketStart).addScaledVector(bracketVector, 0.5);
  bracket.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), bracketVector.clone().normalize());
  bracket.castShadow = true;
  group.add(bracket);

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.036, 0.026, 0.024),
    cameraMaterial
  );
  const lens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.009, 0.009, 0.012, 18),
    lensMaterial
  );
  lens.rotation.x = Math.PI / 2;
  lens.position.z = 0.017;

  const cameraHead = new THREE.Group();
  cameraHead.position.copy(position);
  cameraHead.lookAt(target);
  cameraHead.add(body, lens);
  group.add(cameraHead);

  return group;
}

function toVector3(vector: THREE.Vector3): Vector3 {
  return {
    x: round3(vector.x),
    y: round3(vector.y),
    z: round3(vector.z),
  };
}

function wait(ms: number): Promise<void> {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

function easeInOut(t: number): number {
  return t * t * (3 - 2 * t);
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function round3(value: number): number {
  return Math.round(value * 1000) / 1000;
}
