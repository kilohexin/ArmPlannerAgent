# UR5 虚拟机械臂升级设计

## 背景

当前项目基于乐白 LM3 + LMG-90 程序化建模，机械臂动作不符合物理规范，堆叠穿模。乐白缺少完备的 3D 模型文件，决定全面升级为 UR5 + Robotiq 2F-85 方案。

## 升级目标

| 维度 | 当前 | 升级后 |
|------|------|--------|
| 机械臂 | 乐白 LM3 程序化积木模型 | UR5 URDF 工业模型 |
| 夹爪 | LMG-90 简化夹爪 | Robotiq 2F-85 URDF |
| 物理 | 无，手动坐标计算 | Cannon-es 物理引擎 |
| 运动学 | IK-free 硬编码姿态 | UR5 解析逆运动学 |
| 规划 | LLM 仅意图解析 + 确定性规划 | LLM 全权规划 + 确定性安全校验 |
| 部署 | 前端 + FastAPI 后端 | 纯前端静态应用 |
| API Key | 后端 .env 保护 | 前端用户自填，localStorage 持久化 |

## 架构

```
index.html
├── 3D 视口 (RobotScene)
│   ├── UR5 机器人 (urdf-loader + 解析 IK)
│   ├── Robotiq 2F-85 夹爪 (mimic joint)
│   ├── Cannon-es 物理世界 (桌面、物体刚体、夹爪约束)
│   └── Three.js 渲染 + OrbitControls
├── Agent 面板
│   ├── API Key 输入框
│   ├── 任务输入框
│   └── LLM 全权规划 → 安全校验 → 执行
└── 场景控制
```

## 模块变更清单

### 新文件
- `web/src/scene/robotModel.ts` — URDF 加载 + 关节控制封装
- `web/src/core/ik.ts` — UR5 6-DOF 解析逆运动学 (~300行)
- `web/src/agent/llmClient.ts` — DashScope API 直连客户端
- `web/public/urdf/ur5/` — UR5 URDF + STL 网格
- `web/public/urdf/robotiq/` — Robotiq 2F-85 URDF + STL 网格

### 重写
- `web/src/scene/RobotScene.ts` — 加 Cannon-es 物理世界，适配 UR5 IK 驱动
- `web/src/core/planner.ts` — LLM 输出完整 plan，确定性代码仅做安全校验
- `web/src/agent/agentClient.ts` — 去除后端代理

### 修改
- `web/src/core/types.ts` — 扩展 UR5 相关类型
- `web/src/core/robotProfile.ts` — LM3 参数 → UR5 参数
- `web/src/core/trajectory.ts` — 适配 UR5 工作半径，删除 stackedCenter/heldObjectCenter
- `web/src/core/affordance.ts` — 适配 Robotiq 夹爪
- `web/src/ui/panel.ts` — 增加 API Key 输入
- `web/src/main.ts` — 适配新模块接口
- `web/index.html` — API Key 输入框
- `web/package.json` — 加 urdf-loader, cannon-es 依赖

### 删除
- `web/src/scene/createLebaiRobot.ts` — 程序化模型
- `backend/` — 整个目录
- `trajectory.ts` 中 `stackedCenter()`, `heldObjectCenter()` — 物理引擎替代

### 不改
- `web/src/core/sceneGraph.ts` — 物体查找逻辑通用
- `web/vite.config.ts`
- `web/tsconfig.json`

## LLM 规划层

### 请求结构

```
System: 你是 UR5 六轴机械臂 + Robotiq 2F-85 夹爪的任务规划器。
        输出 JSON: { steps: [...], confidence: 0-1 }
        steps 字段: action, objectId, targetPosition{x,y,z}, gripperAction
        约束: 工作半径 0.85m, 桌面 x[-0.4,0.6] z[-0.35,0.35]

User:   { instruction, scene: { objects }, history }
```

### 安全校验（确定性，不用 LLM）

- 每个 targetPosition 在 UR5 0.85m 工作半径内
- 引用物体在 sceneState.objects 中存在
- 物体标记为 movable
- z 坐标不低于桌面
- 步数不超过上限

### 降级策略

LLM 不可用时（Key 无效/网络错误/confidence < 阈值），降级为当前规则规划器兜底。

### 多步任务示例

输入 "先把红色方块放到蓝色方块上，再把黄色圆柱体放到红色方块旁边"，LLM 输出约 12 步的完整动作序列，包含每步的目标坐标。

## 物理引擎 (Cannon-es)

- World gravity: -9.81m/s²
- 桌面: 静态刚体 (mass=0)
- 方块/圆柱体: 动态刚体 (mass=0.3kg)，每帧同步 physics → mesh → sceneState
- 夹持: PointToPointConstraint（close_gripper 创建，open_gripper 销毁）
- 拖拽: 临时切 kinematic 模式

## 逆运动学 (UR5 解析解)

基于 UR5 标准 DH 参数：

| Joint | a(m) | α(rad) | d(m) |
|-------|------|--------|------|
| 1 | 0 | π/2 | 0.089159 |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.39225 | 0 | 0 |
| 4 | 0 | π/2 | 0.10915 |
| 5 | 0 | -π/2 | 0.09465 |
| 6 | 0 | 0 | 0.0823 |

流程：末端位姿 → 腕心位置 → θ₁(2解) → θ₅(2解) → θ₂,θ₃(2解) → θ₄,θ₆(2解) → 8组解筛选最优。

## 实施阶段

| 阶段 | 内容 | 工作量 |
|------|------|--------|
| 1 | 资源准备：下载 URDF+STL，npm install，删除 backend | 0.5天 |
| 2 | 机器人模型替换：URDF 加载 + 关节控制 | 1天 |
| 3 | 逆运动学：UR5 解析解 + 末端驱动 | 1.5-2天 |
| 4 | 物理引擎：Cannon-es 集成 + 夹爪约束 | 1天 |
| 5 | LLM 全权规划：DashScope 直连 + 安全校验 | 1天 |
| 6 | 清理 + 测试适配 | 0.5天 |

**总计: 5.5-6天**

## 验证

- 阶段2验证：浏览器中 UR5 + Robotiq 模型正确渲染
- 阶段3验证：输入 (x,y,z) 坐标，末端正确到达目标
- 阶段4验证：物体自然掉落、堆叠无穿模、夹持稳定
- 阶段5验证：多步复杂指令成功执行
- 最终验证：`npm test && npm run build` 通过
