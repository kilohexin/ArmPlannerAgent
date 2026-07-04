import { describe, expect, it } from 'vitest';
import { formatJsonForLog } from './panel';

describe('formatJsonForLog', () => {
  it('formats structured payloads as readable indented JSON', () => {
    const formatted = formatJsonForLog({ plan: { targetObjectId: 'red_cube' } });

    expect(formatted).toContain('"targetObjectId": "red_cube"');
    expect(formatted.split('\n').length).toBeGreaterThan(2);
  });
});

