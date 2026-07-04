import * as THREE from 'three';
import { GLTFLoader, type GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';

const JOINTS = [
  { key: 'joint1', name: 'Joint1', axis: 'y' },
  { key: 'joint2', name: 'Joint2', axis: 'z' },
  { key: 'joint3', name: 'Joint3', axis: 'z' },
  { key: 'joint4', name: 'Joint4', axis: 'z' },
  { key: 'joint5', name: 'Joint5', axis: 'y' },
  { key: 'joint6', name: 'Joint6', axis: 'z' },
] as const;

export interface RobotModel {
  group: THREE.Group;
  robot: THREE.Object3D;
  gripperGroup: THREE.Object3D;
  setJointAngles(angles: number[]): void;
  setGripper(value: number): void;
  alignGripperTipDirection(direction: THREE.Vector3): void;
  getEndEffectorPosition(): THREE.Vector3;
  getEndEffectorQuaternion(): THREE.Quaternion;
}

export function resolvePublicAsset(path: string): string {
  const rawBase = import.meta.env.BASE_URL || './';
  const normalizedBase = rawBase === '/' ? './' : rawBase;
  const base = normalizedBase.endsWith('/')
    ? normalizedBase
    : `${normalizedBase}/`;
  return `${base}${path}`;
}

export async function loadRobotModel(): Promise<RobotModel> {
  const loader = new GLTFLoader();
  const gltf = await loader.loadAsync(resolvePublicAsset('models/Lebai_LM3.glb'));
  return createLebaiModel(gltf);
}

function createLebaiModel(gltf: GLTF): RobotModel {
  const source = gltf.scene;
  const group = new THREE.Group();
  group.name = 'Lebai_LM3_visual';

  normalizeModelPlacement(source);
  group.add(source);

  const jointRefs = new Map<string, THREE.Object3D>();
  const originalRotations = new Map<string, THREE.Euler>();
  let gripperGroup: THREE.Object3D | null = null;
  let tcpFrame: THREE.Object3D | null = null;

  source.traverse((child) => {
    for (const joint of JOINTS) {
      if (child.name === joint.name) {
        jointRefs.set(joint.key, child);
        originalRotations.set(joint.key, child.rotation.clone());
      }
    }
    if (child.name === 'robotgrabber') {
      gripperGroup = child;
    }
    if (child.name === 'grabber') {
      tcpFrame = child;
    }
  });

  const toolFrame = tcpFrame ?? gripperGroup ?? jointRefs.get('joint6') ?? source;
  const gripperRoot = gripperGroup ?? toolFrame;
  const mixer = gltf.animations.length ? new THREE.AnimationMixer(source) : null;
  const gripperClip = gltf.animations.find((clip) => clip.name === 'Take 001') ?? gltf.animations[0];
  const gripperAction = mixer && gripperClip ? mixer.clipAction(gripperClip) : null;
  const [gripperStart, gripperEnd] = getGripperFrameRange(gripperClip);

  if (gripperAction) {
    gripperAction.play();
    gripperAction.paused = true;
    gripperAction.time = gripperStart;
    mixer?.update(0);
  }

  function setJointAngles(angles: number[]): void {
    JOINTS.forEach((joint, index) => {
      const node = jointRefs.get(joint.key);
      const baseRotation = originalRotations.get(joint.key);
      if (!node || !baseRotation) {
        return;
      }

      node.rotation.copy(baseRotation);
      node.rotation[joint.axis] += clampAngle(angles[index] ?? 0);
    });
  }

  function setGripper(value: number): void {
    if (!gripperAction || !mixer) {
      return;
    }
    const t = gripperStart + (gripperEnd - gripperStart) * clamp01(value);
    gripperAction.time = t;
    gripperAction.paused = true;
    mixer.update(0);
  }

  function alignGripperTipDirection(direction: THREE.Vector3): void {
    const target = direction.clone().normalize();
    if (!Number.isFinite(target.lengthSq()) || target.lengthSq() < 1e-6) {
      return;
    }

    source.updateMatrixWorld(true);
    const rootPos = new THREE.Vector3();
    const tipPos = new THREE.Vector3();
    gripperRoot.getWorldPosition(rootPos);
    toolFrame.getWorldPosition(tipPos);
    const current = tipPos.sub(rootPos).normalize();
    if (!Number.isFinite(current.lengthSq()) || current.lengthSq() < 1e-6) {
      return;
    }

    const correction = new THREE.Quaternion().setFromUnitVectors(current, target);
    const currentWorld = new THREE.Quaternion();
    gripperRoot.getWorldQuaternion(currentWorld);
    const desiredWorld = correction.multiply(currentWorld);

    const parentWorld = new THREE.Quaternion();
    gripperRoot.parent?.getWorldQuaternion(parentWorld);
    gripperRoot.quaternion.copy(parentWorld.invert().multiply(desiredWorld));
    gripperRoot.updateMatrixWorld(true);
  }

  function getEndEffectorPosition(): THREE.Vector3 {
    const pos = new THREE.Vector3();
    toolFrame.getWorldPosition(pos);
    return pos;
  }

  function getEndEffectorQuaternion(): THREE.Quaternion {
    const quat = new THREE.Quaternion();
    toolFrame.getWorldQuaternion(quat);
    return quat;
  }

  return {
    group,
    robot: source,
    gripperGroup: toolFrame,
    setJointAngles,
    setGripper,
    alignGripperTipDirection,
    getEndEffectorPosition,
    getEndEffectorQuaternion,
  };
}

function normalizeModelPlacement(model: THREE.Object3D): void {
  const box = new THREE.Box3().setFromObject(model);
  model.position.y -= box.min.y;
}

function getGripperFrameRange(clip?: THREE.AnimationClip): [number, number] {
  if (!clip?.tracks[0]) {
    return [0, 1];
  }

  const times = clip.tracks[0].times;
  if (times.length > 20) {
    return [times[0], times[20]];
  }
  return [0, clip.duration || 1];
}

function clampAngle(value: number): number {
  return Math.min(Math.PI, Math.max(-Math.PI, value));
}

function clamp01(value: number): number {
  return Math.min(1, Math.max(0, value));
}
