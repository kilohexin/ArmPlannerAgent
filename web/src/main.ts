import './styles.css';
import { requestPlan } from './agent/agentClient';
import { getApiKey, getModelId, setApiKey, setModelId } from './agent/llmClient';
import { loadTcpCalibration, saveTcpCalibration } from './core/tcpCalibration';
import type { Vector3, VisualSnapshot } from './core/types';
import { RobotScene } from './scene/RobotScene';
import {
  appendJsonLog,
  appendLog,
  appendSnapshotLog,
  planSummary,
  renderSceneTable,
  renderScore,
  setPipeline,
} from './ui/panel';

let robotScene: RobotScene;
let latestSnapshot: VisualSnapshot | null = null;

async function init(): Promise<void> {
  const sceneRoot = document.querySelector<HTMLDivElement>('#scene-root');
  if (!sceneRoot) {
    throw new Error('Missing #scene-root');
  }

  robotScene = await RobotScene.create(sceneRoot);

  const sceneTable = document.querySelector<HTMLDivElement>('#scene-table')!;
  const scorePanel = document.querySelector<HTMLDivElement>('#score-panel')!;
  const logRoot = document.querySelector<HTMLDivElement>('#execution-log')!;
  const taskInput = document.querySelector<HTMLInputElement>('#task-input')!;
  const waypointCount = document.querySelector<HTMLSpanElement>('#waypoint-count')!;
  const planningTime = document.querySelector<HTMLSpanElement>('#planning-time')!;
  const duration = document.querySelector<HTMLSpanElement>('#duration')!;
  const cameraPreview = document.querySelector<HTMLImageElement>('#camera-preview')!;
  const cameraPlaceholder = document.querySelector<HTMLSpanElement>('#camera-placeholder')!;
  const cameraStatus = document.querySelector<HTMLParagraphElement>('#camera-status')!;
  const useVision = document.querySelector<HTMLInputElement>('#use-vision')!;

  robotScene.setChangeHandler(render);
  render();
  renderScore(scorePanel, null);
  appendLog(logRoot, '乐白 LM3 + 夹爪仿真环境已就绪。');

  const apiKeyInput = document.querySelector<HTMLInputElement>('#api-key')!;
  const saveApiKeyBtn = document.querySelector<HTMLButtonElement>('#save-api-key')!;
  const modelInput = document.querySelector<HTMLInputElement>('#model-id')!;
  const saveModelBtn = document.querySelector<HTMLButtonElement>('#save-model-id')!;
  const tcpXInput = document.querySelector<HTMLInputElement>('#tcp-x')!;
  const tcpYInput = document.querySelector<HTMLInputElement>('#tcp-y')!;
  const tcpZInput = document.querySelector<HTMLInputElement>('#tcp-z')!;
  const saveTcpBtn = document.querySelector<HTMLButtonElement>('#save-tcp')!;

  const savedKey = getApiKey();
  if (savedKey) {
    apiKeyInput.value = savedKey;
  }
  modelInput.value = getModelId();
  applyTcpInputs(loadTcpCalibration());

  saveApiKeyBtn.addEventListener('click', () => {
    setApiKey(apiKeyInput.value.trim());
    appendLog(logRoot, 'API Key 已保存到浏览器本地存储。');
  });

  saveModelBtn.addEventListener('click', () => {
    setModelId(modelInput.value.trim() || 'qwen3.5-flash');
    modelInput.value = getModelId();
    appendLog(logRoot, `模型 ID 已保存：${modelInput.value}`);
  });

  saveTcpBtn.addEventListener('click', () => {
    const calibration = saveTcpCalibration({
      x: mmToMeters(tcpXInput.value),
      y: mmToMeters(tcpYInput.value),
      z: mmToMeters(tcpZInput.value),
    });
    applyTcpInputs(calibration);
    appendLog(
      logRoot,
      `夹爪 TCP 校准已保存：x=${calibration.x.toFixed(3)}m, y=${calibration.y.toFixed(3)}m, z=${calibration.z.toFixed(3)}m`
    );
  });

  document.querySelector('#send-task')?.addEventListener('click', runTask);
  document.querySelector('#run-task')?.addEventListener('click', runTask);
  document.querySelector('#add-cube')?.addEventListener('click', () => robotScene.addObject('cube'));
  document.querySelector('#add-cylinder')?.addEventListener('click', () => robotScene.addObject('cylinder'));
  document.querySelector('#capture-camera')?.addEventListener('click', captureCamera);
  document.querySelector('#clear-camera')?.addEventListener('click', clearCamera);
  document.querySelector('#reset-scene')?.addEventListener('click', () => {
    robotScene.reset();
    clearCamera();
    renderScore(scorePanel, null);
    setPipeline(null, false);
    appendLog(logRoot, '场景已重置。');
  });
  document.querySelector('#clear-log')?.addEventListener('click', () => {
    logRoot.innerHTML = '';
  });
  document.querySelector('#save-scene')?.addEventListener('click', () => {
    localStorage.setItem('arm-planner-scene', JSON.stringify(robotScene.sceneState));
    appendLog(logRoot, '场景图已保存到浏览器本地存储。');
  });
  document.querySelectorAll<HTMLButtonElement>('[data-view]').forEach((button) => {
    button.addEventListener('click', () => robotScene.setView(button.dataset.view as 'top' | 'front' | 'iso' | 'eye'));
  });

  function captureCamera(): void {
    latestSnapshot = robotScene.captureEyeToHandSnapshot();
    cameraPreview.src = latestSnapshot.dataUrl;
    cameraPreview.classList.add('visible');
    cameraPlaceholder.classList.add('hidden');
    cameraStatus.textContent = `已拍摄：${new Date(latestSnapshot.capturedAt).toLocaleTimeString()}，可作为 VLM 视觉提示。`;
    appendLog(logRoot, '手外眼相机完成一次场景截图。');
    appendSnapshotLog(logRoot, latestSnapshot);
  }

  function clearCamera(): void {
    latestSnapshot = null;
    cameraPreview.removeAttribute('src');
    cameraPreview.classList.remove('visible');
    cameraPlaceholder.classList.remove('hidden');
    cameraStatus.textContent = '固定相机位于工作台外侧，用于模拟 RGB-D/VLM 的视觉输入。';
  }

  async function runTask(): Promise<void> {
    const instruction = taskInput.value.trim();
    if (!instruction) {
      appendLog(logRoot, '请输入任务指令。', 'WARN');
      return;
    }

    const started = performance.now();
    const visualSnapshot = useVision.checked ? latestSnapshot : null;
    setPipeline(null, false);
    appendLog(logRoot, `收到任务：${instruction}`);
    appendJsonLog(logRoot, '输入 Scene Graph JSON', robotScene.sceneState);
    if (visualSnapshot) {
      appendSnapshotLog(logRoot, visualSnapshot);
    }

    try {
      const response = await requestPlan(robotScene.sceneState, instruction, visualSnapshot);
      const elapsed = performance.now() - started;
      planningTime.textContent = `规划耗时：${(elapsed / 1000).toFixed(2)} s`;
      waypointCount.textContent = `轨迹点：${response.plan.trajectory.length}`;
      duration.textContent = `预计执行：${(response.plan.steps.length * 0.42).toFixed(2)} s`;
      robotScene.drawPlan(response.plan);
      renderScore(scorePanel, response.plan.score);
      setPipeline(response.validation, false);
      appendLog(logRoot, planSummary(response.plan), response.validation.ok ? 'INFO' : 'WARN');
      appendJsonLog(logRoot, '规划响应 JSON', response);
      for (const note of response.notes) {
        appendLog(logRoot, note);
      }

      if (response.validation.ok) {
        const executed = await robotScene.executePlan(response.plan, (message) => appendLog(logRoot, message));
        setPipeline(response.validation, executed);
        robotScene.sceneState.history.push({
          instruction,
          targetObjectId: response.plan.targetObjectId,
          destinationObjectId: response.plan.destinationObjectId,
          status: executed ? 'executed' : 'blocked',
        });
        appendLog(
          logRoot,
          executed ? '仿真执行完成。' : '仿真执行失败，请查看控制台或修复规划。',
          executed ? 'INFO' : 'WARN'
        );
      } else {
        appendLog(logRoot, response.validation.repairHint, 'WARN');
      }
    } catch (error) {
      appendLog(logRoot, error instanceof Error ? error.message : '未知规划错误', 'WARN');
    }
  }

  function render(): void {
    renderSceneTable(sceneTable, robotScene.sceneState, robotScene.getSelectedObjectId());
  }

  function applyTcpInputs(calibration: Vector3): void {
    tcpXInput.value = metersToMm(calibration.x);
    tcpYInput.value = metersToMm(calibration.y);
    tcpZInput.value = metersToMm(calibration.z);
  }

  function mmToMeters(value: string): number {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed / 1000 : 0;
  }

  function metersToMm(value: number): string {
    return String(Math.round(value * 1000));
  }
}

init().catch((err) => {
  const body = document.body;
  if (body) {
    body.innerHTML = `<div style="padding:2rem;color:red">启动失败：${err instanceof Error ? err.message : String(err)}</div>`;
  }
});
