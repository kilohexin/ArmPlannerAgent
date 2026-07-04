import type { ManipulationPlan, SceneState, TrajectoryScore, ValidationResult, VisualSnapshot } from '../core/types';

export function renderSceneTable(root: HTMLElement, scene: SceneState, selectedId: string | null): void {
  root.innerHTML = '';

  const head = document.createElement('div');
  head.className = 'scene-head';
  for (const label of ['名称', '类型', '位姿 (x,y,z)', '显示']) {
    const span = document.createElement('span');
    span.textContent = label;
    head.appendChild(span);
  }
  root.appendChild(head);

  for (const object of scene.objects) {
    const row = document.createElement('div');
    row.className = `scene-row ${object.id === selectedId ? 'selected' : ''}`;
    for (const text of [
      object.id,
      object.type === 'cube' ? '方块' : '圆柱',
      `${object.position.x.toFixed(2)}, ${object.position.y.toFixed(2)}, ${object.position.z.toFixed(2)}`,
      '可见',
    ]) {
      const span = document.createElement('span');
      span.textContent = text;
      row.appendChild(span);
    }
    root.appendChild(row);
  }
}

export function renderScore(root: HTMLElement, score: TrajectoryScore | null): void {
  const value = score?.total ?? 0;
  root.innerHTML = `
    <div class="score-ring" style="--score:${value}"><span>${value}<small>/100</small></span></div>
    <div class="score-grid">
      <div class="metric">碰撞<strong>${score ? (score.collision >= 1 ? '无碰撞' : '警告') : '--'}</strong></div>
      <div class="metric">平滑度<strong>${score?.smoothness ?? '--'}</strong></div>
      <div class="metric">可达性<strong>${score?.reachability ?? '--'}</strong></div>
      <div class="metric">关节限制<strong>${score?.jointLimits === 'OK' ? '正常' : (score?.jointLimits ?? '--')}</strong></div>
    </div>
  `;
}

export function setPipeline(validation: ValidationResult | null, executed: boolean): void {
  const validate = document.querySelector('#validate-stage');
  const repair = document.querySelector('#repair-stage');
  const execute = document.querySelector('#execute-stage');
  validate?.classList.toggle('done', Boolean(validation?.ok));
  repair?.classList.toggle('warn', Boolean(validation && !validation.ok));
  repair?.classList.toggle('done', Boolean(validation?.ok));
  execute?.classList.toggle('done', executed);
}

export function appendLog(root: HTMLElement, message: string, level: 'INFO' | 'WARN' = 'INFO'): void {
  const line = document.createElement('div');
  line.className = 'log-line';

  const time = document.createElement('span');
  time.textContent = new Date().toLocaleTimeString();

  const body = document.createElement('span');
  body.textContent = message;

  const tag = document.createElement('span');
  tag.className = `tag ${level === 'WARN' ? 'warn' : ''}`;
  tag.textContent = level;

  line.append(time, body, tag);
  root.appendChild(line);
  root.scrollTop = root.scrollHeight;
}

export function formatJsonForLog(payload: unknown): string {
  return JSON.stringify(payload, null, 2);
}

export function appendJsonLog(root: HTMLElement, title: string, payload: unknown): void {
  const line = document.createElement('div');
  line.className = 'log-line log-json';

  const time = document.createElement('span');
  time.textContent = new Date().toLocaleTimeString();

  const details = document.createElement('details');
  const summary = document.createElement('summary');
  summary.textContent = title;
  const pre = document.createElement('pre');
  pre.textContent = formatJsonForLog(payload);
  details.append(summary, pre);

  const tag = document.createElement('span');
  tag.className = 'tag';
  tag.textContent = 'JSON';

  line.append(time, details, tag);
  root.appendChild(line);
  root.scrollTop = root.scrollHeight;
}

export function appendSnapshotLog(root: HTMLElement, snapshot: VisualSnapshot): void {
  appendJsonLog(root, '手外眼相机快照元数据', {
    id: snapshot.id,
    mimeType: snapshot.mimeType,
    capturedAt: snapshot.capturedAt,
    dataUrlBytes: snapshot.dataUrl.length,
    camera: snapshot.camera,
  });
}

export function planSummary(plan: ManipulationPlan): string {
  if (plan.status !== 'ready') {
    return plan.repairHint ?? '计划需要修复。';
  }
  const jointCount = plan.jointTrajectory?.length ?? 0;
  return `已生成计划：${plan.targetObjectId} -> ${plan.destinationObjectId}，末端轨迹 ${plan.trajectory.length} 点，关节轨迹 ${jointCount} 点。`;
}
