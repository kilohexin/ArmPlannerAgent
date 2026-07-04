import type { Vector3 } from './types';

export const TCP_CALIBRATION_STORAGE_KEY = 'arm-planner-tcp-calibration';
export const DEFAULT_GRIPPER_TCP_CALIBRATION: Vector3 = {
  x: 0.016,
  y: 0.065,
  z: 0,
};

interface CalibrationStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

let currentCalibration: Vector3 = { ...DEFAULT_GRIPPER_TCP_CALIBRATION };

export function getTcpCalibration(): Vector3 {
  return { ...currentCalibration };
}

export function setTcpCalibration(calibration: Vector3): void {
  currentCalibration = sanitizeCalibration(calibration);
}

export function loadTcpCalibration(storage: CalibrationStorage = window.localStorage): Vector3 {
  const raw = storage.getItem(TCP_CALIBRATION_STORAGE_KEY);
  if (!raw) {
    currentCalibration = { ...DEFAULT_GRIPPER_TCP_CALIBRATION };
    return getTcpCalibration();
  }

  try {
    currentCalibration = sanitizeCalibration(JSON.parse(raw));
  } catch {
    currentCalibration = { ...DEFAULT_GRIPPER_TCP_CALIBRATION };
  }
  return getTcpCalibration();
}

export function saveTcpCalibration(
  calibration: Vector3,
  storage: CalibrationStorage = window.localStorage
): Vector3 {
  setTcpCalibration(calibration);
  storage.setItem(TCP_CALIBRATION_STORAGE_KEY, JSON.stringify(currentCalibration));
  return getTcpCalibration();
}

function sanitizeCalibration(value: Partial<Vector3>): Vector3 {
  return {
    x: sanitizeAxis(value.x),
    y: sanitizeAxis(value.y),
    z: sanitizeAxis(value.z),
  };
}

function sanitizeAxis(value: unknown): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return 0;
  }
  return Math.round(Math.max(-0.08, Math.min(0.08, value)) * 1000) / 1000;
}
