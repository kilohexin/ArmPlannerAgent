import type { LLMPlanResponse, SceneState, VisualSnapshot } from '../core/types';

const API_KEY_STORAGE = 'dashscope-api-key';
const MODEL_STORAGE = 'dashscope-model-id';
const DEFAULT_BASE_URL = 'https://dashscope.aliyuncs.com/compatible-mode/v1';
const DEFAULT_MODEL = 'qwen3.5-flash';

export type OpenAICompatibleUserContent =
  | string
  | Array<
      | { type: 'text'; text: string }
      | { type: 'image_url'; image_url: { url: string } }
    >;

export function getApiKey(): string | null {
  return localStorage.getItem(API_KEY_STORAGE);
}

export function setApiKey(key: string): void {
  localStorage.setItem(API_KEY_STORAGE, key);
}

export function clearApiKey(): void {
  localStorage.removeItem(API_KEY_STORAGE);
}

export function getModelId(): string {
  return localStorage.getItem(MODEL_STORAGE) || DEFAULT_MODEL;
}

export function setModelId(modelId: string): void {
  localStorage.setItem(MODEL_STORAGE, modelId);
}

const SYSTEM_PROMPT = `You are a task planner for a Lebai LM3 6-DOF robot arm with a parallel gripper.
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
- Robot max reach: 0.85m from base (0,0)
- Table surface is at y=0.035
- Safe traversal height: y >= 0.35 when moving between objects
- Gripper close before lifting, open after placing
- Object positions from scene.objects are authoritative
- If an eye-to-hand camera image is provided, use it only as visual context to cross-check the structured scene graph
- All targetPosition.y must be >= 0.035 (table surface)
- targetPosition means the gripper center between the two fingers, not the robot flange or wrist joint
- Always use top grasp in this course simulator: pre-grasp above the object, then align the gripper center with object.position before closing
- Never close the gripper when the gripper center is below the object or far from the object center
- For placing on destination: compute stacked y = destination.position.y + destination.size.y/2 + target.size.y/2
- You generate Cartesian task waypoints only. The frontend will convert them to LM3 joint trajectories using inverse kinematics.
- The natural-language descriptions should be concise Chinese.

Only use object IDs that exist in the provided scene.objects array.`;

export function buildUserContent(
  scene: SceneState,
  instruction: string,
  visualSnapshot?: VisualSnapshot | null
): OpenAICompatibleUserContent {
  const payload = {
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
    visualPrompt: visualSnapshot
      ? {
          id: visualSnapshot.id,
          capturedAt: visualSnapshot.capturedAt,
          camera: visualSnapshot.camera,
          note: 'The attached image is rendered by a fixed eye-to-hand camera in the 3D simulator.',
        }
      : null,
  };
  const text = JSON.stringify(payload);

  if (!visualSnapshot) {
    return text;
  }

  return [
    { type: 'text', text },
    { type: 'image_url', image_url: { url: visualSnapshot.dataUrl } },
  ];
}

export async function planWithLLM(
  scene: SceneState,
  instruction: string,
  visualSnapshot?: VisualSnapshot | null
): Promise<LLMPlanResponse | null> {
  const apiKey = getApiKey();
  if (!apiKey) return null;

  const baseUrl = DEFAULT_BASE_URL;
  const model = getModelId();
  const userContent = buildUserContent(scene, instruction, visualSnapshot);
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 30_000);

  try {
    const response = await fetch(`${baseUrl}/chat/completions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${apiKey}`,
      },
      body: JSON.stringify({
        model,
        messages: [
          { role: 'system', content: SYSTEM_PROMPT },
          { role: 'user', content: userContent },
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

    const jsonMatch = content.match(/```(?:json)?\s*\n?([\s\S]*?)\n?```/) || [null, content];
    const jsonStr = jsonMatch[1] || content;
    const parsed = JSON.parse(jsonStr.trim());

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
