import type { AffordanceCandidate, SceneObject, Vector3 } from './types';

const UP: Vector3 = { x: 0, y: 1, z: 0 };

export function generateAffordances(object: SceneObject): AffordanceCandidate[] {
  const halfHeight = object.size.y / 2;
  const topPose = {
    x: object.position.x,
    y: object.position.y + halfHeight,
    z: object.position.z
  };

  if (object.type === 'cylinder') {
    return [
      {
        id: `${object.id}:side_grasp`,
        objectId: object.id,
        kind: 'side_grasp',
        pose: { x: object.position.x + object.size.x / 2, y: object.position.y, z: object.position.z },
        approach: { x: -1, y: 0, z: 0 },
        score: 0.90,
        note: 'Cylinders are most stable when the gripper closes on the side wall.'
      },
      {
        id: `${object.id}:top_place`,
        objectId: object.id,
        kind: 'top_place',
        pose: topPose,
        approach: UP,
        score: 0.72,
        note: 'Top center can be used as a placement target.'
      }
    ];
  }

  return [
    {
      id: `${object.id}:side_grasp`,
      objectId: object.id,
      kind: 'side_grasp',
      pose: { x: object.position.x + object.size.x / 2, y: object.position.y, z: object.position.z },
      approach: { x: -1, y: 0, z: 0 },
      score: 0.93,
      note: 'Side grasp for the LMG-90 or generic parallel-jaw gripper.'
    },
    {
      id: `${object.id}:top_grasp`,
      objectId: object.id,
      kind: 'top_grasp',
      pose: topPose,
      approach: UP,
      score: 0.7,
      note: 'Top grasp is available for small cubes but has lower stability.'
    },
    {
      id: `${object.id}:top_place`,
      objectId: object.id,
      kind: 'top_place',
      pose: topPose,
      approach: UP,
      score: 0.95,
      note: 'Top center is the preferred stacking placement point.'
    }
  ];
}

export function selectBestAffordance(
  object: SceneObject,
  preferredKinds: Array<AffordanceCandidate['kind']>
): AffordanceCandidate {
  const candidates = generateAffordances(object);
  const preferred = candidates
    .filter((candidate) => preferredKinds.includes(candidate.kind))
    .sort((a, b) => b.score - a.score)[0];
  return preferred ?? candidates.sort((a, b) => b.score - a.score)[0];
}
