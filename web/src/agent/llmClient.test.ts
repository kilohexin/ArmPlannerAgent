import { describe, it, expect, beforeEach } from 'vitest';
import { buildUserContent, getApiKey, setApiKey, clearApiKey } from './llmClient';
import type { SceneState, VisualSnapshot } from '../core/types';

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

  it('builds text-only user content without a camera snapshot', () => {
    const content = buildUserContent(testScene, '把红色方块放到蓝色方块上');

    expect(typeof content).toBe('string');
    expect(content).toContain('"instruction"');
    expect(content).toContain('red_cube');
  });

  it('builds multimodal user content with an eye-to-hand camera snapshot', () => {
    const snapshot: VisualSnapshot = {
      id: 'eye-to-hand-1',
      dataUrl: 'data:image/jpeg;base64,abc123',
      mimeType: 'image/jpeg',
      capturedAt: '2026-07-03T14:00:00.000Z',
      camera: {
        name: 'eye_to_hand',
        position: { x: 0.72, y: 0.56, z: 0.62 },
        target: { x: 0.24, y: 0.1, z: 0 },
      },
    };

    const content = buildUserContent(testScene, '把红色方块放到蓝色方块上', snapshot);

    expect(Array.isArray(content)).toBe(true);
    expect(content[0]).toMatchObject({ type: 'text' });
    expect(content[1]).toEqual({
      type: 'image_url',
      image_url: { url: snapshot.dataUrl },
    });
  });
});

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
  ],
  history: [],
};
