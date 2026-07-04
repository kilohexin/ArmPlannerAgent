import { describe, expect, it } from 'vitest';
import { resolvePublicAsset } from './robotModel';

describe('resolvePublicAsset', () => {
  it('builds a public model URL from the Vite base URL', () => {
    expect(resolvePublicAsset('models/Lebai_LM3.glb')).toBe('./models/Lebai_LM3.glb');
  });
});
