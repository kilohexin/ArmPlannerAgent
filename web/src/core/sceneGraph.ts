import type { SceneObject, SceneState } from './types';

const COLOR_ALIASES: Record<string, string[]> = {
  red: ['red', '红色', '红'],
  blue: ['blue', '蓝色', '蓝'],
  green: ['green', '绿色', '绿'],
  yellow: ['yellow', '黄色', '黄'],
  purple: ['purple', '紫色', '紫'],
};

const TYPE_ALIASES: Record<string, string[]> = {
  cube: ['cube', 'box', '方块', '立方体', '块'],
  cylinder: ['cylinder', '圆柱', '柱体'],
};

export function findObjectByReference(
  scene: SceneState,
  instruction: string,
  options: { excludeIds?: string[]; preferDestination?: boolean } = {}
): SceneObject | null {
  const text = instruction.toLowerCase();
  const exclude = new Set(options.excludeIds ?? []);
  const candidates = scene.objects.filter((object) => !exclude.has(object.id));

  for (const object of candidates) {
    if (text.includes(object.id.toLowerCase()) || text.includes(object.label.toLowerCase())) {
      return object;
    }
  }

  const mentionedColors = Object.entries(COLOR_ALIASES)
    .filter(([, aliases]) => aliases.some((alias) => text.includes(alias)))
    .map(([color]) => color);
  const color = options.preferDestination ? mentionedColors.at(-1) : mentionedColors[0];

  const mentionedTypes = Object.entries(TYPE_ALIASES)
    .filter(([, aliases]) => aliases.some((alias) => text.includes(alias)))
    .map(([type]) => type);
  const type = mentionedTypes[0];

  if (color) {
    const exact = candidates.find((candidate) =>
      candidate.color.toLowerCase() === color &&
      (!type || candidate.type === type)
    );
    if (exact) {
      return exact;
    }

    const sameColor = candidates.find((candidate) => candidate.color.toLowerCase() === color);
    if (sameColor) {
      return sameColor;
    }
  }

  if (/\bit\b|它|刚才|上一个/.test(text)) {
    const lastTarget = [...scene.history].reverse().find((item) => item.targetObjectId)?.targetObjectId;
    return candidates.find((object) => object.id === lastTarget) ?? null;
  }

  return null;
}

export function relationSummary(scene: SceneState): string[] {
  const relations: string[] = [];
  for (const left of scene.objects) {
    for (const right of scene.objects) {
      if (left.id === right.id) {
        continue;
      }
      if (Math.abs(left.position.x - right.position.x) > 0.12) {
        relations.push(`${left.id} is ${left.position.x > right.position.x ? 'right of' : 'left of'} ${right.id}`);
      }
      if (Math.abs(left.position.z - right.position.z) > 0.12) {
        relations.push(`${left.id} is ${left.position.z > right.position.z ? 'behind' : 'in front of'} ${right.id}`);
      }
    }
  }
  return relations.slice(0, 12);
}
