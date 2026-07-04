import { describe, expect, it } from 'vitest';
import {
  DEFAULT_GRIPPER_TCP_CALIBRATION,
  getTcpCalibration,
  loadTcpCalibration,
  setTcpCalibration,
} from './tcpCalibration';

describe('TCP calibration', () => {
  it('uses the default gripper TCP calibration initially', () => {
    setTcpCalibration(DEFAULT_GRIPPER_TCP_CALIBRATION);

    expect(getTcpCalibration()).toEqual(DEFAULT_GRIPPER_TCP_CALIBRATION);
  });

  it('loads a persisted calibration from storage', () => {
    const storage = new Map<string, string>();
    const fakeStorage = {
      getItem: (key: string) => storage.get(key) ?? null,
      setItem: (key: string, value: string) => storage.set(key, value),
    };
    fakeStorage.setItem('arm-planner-tcp-calibration', JSON.stringify({ x: -0.012, y: 0.004, z: 0.006 }));

    loadTcpCalibration(fakeStorage);

    expect(getTcpCalibration()).toEqual({ x: -0.012, y: 0.004, z: 0.006 });
  });
});
