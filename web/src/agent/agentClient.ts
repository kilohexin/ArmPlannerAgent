import { createLocalPlan, createPlanFromLLM, validateLLMPlan } from '../core/planner';
import { planWithLLM } from './llmClient';
import type { ManipulationPlan, SceneState, ValidationResult, VisualSnapshot } from '../core/types';

export interface PlanResponse {
  plan: ManipulationPlan;
  validation: ValidationResult;
  notes: string[];
}

export async function requestPlan(
  scene: SceneState,
  instruction: string,
  visualSnapshot?: VisualSnapshot | null
): Promise<PlanResponse> {
  const notes: string[] = [];
  const llmResult = await planWithLLM(scene, instruction, visualSnapshot);

  if (visualSnapshot) {
    notes.push('已附加手外眼相机截图作为多模态视觉提示。');
  }

  if (llmResult && llmResult.confidence >= 0.4) {
    const plan = createPlanFromLLM(scene, instruction, llmResult.steps, llmResult.confidence);
    const validation = validateLLMPlan(scene, plan);

    notes.push(`LLM 规划完成，置信度：${(llmResult.confidence * 100).toFixed(0)}%。`);
    if (llmResult.explanation) {
      notes.push(`解析说明：${llmResult.explanation}`);
    }
    if (!validation.ok) {
      notes.push(`安全校验未通过：${validation.repairHint}`);
    }

    return { plan, validation, notes };
  }

  notes.push(llmResult
    ? `LLM 置信度过低（${(llmResult.confidence * 100).toFixed(0)}%），已回退到本地规则规划。`
    : 'LLM 不可用或模型不支持当前输入，使用本地规则规划。');

  const plan = createLocalPlan(scene, instruction);
  const validation = validateLLMPlan(scene, plan);

  return { plan, validation, notes };
}
