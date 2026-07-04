# ArmPlannerAgent

面向乐白 LM3 机械臂的动作自动规划智能体课程原型。当前版本采用静态 Web 前端实现：Three.js 展示数字孪生场景，DashScope/Qwen 负责自然语言任务规划，本地工具层负责校验、轨迹生成、LM3 数值 IK 和仿真执行。

## 当前功能

- 乐白 LM3 GLB 外观模型加载与 6 关节动画。
- 结构化 Scene Graph：物体 ID、类型、颜色、尺寸、位置、可移动状态。
- 中文任务输入，例如“把红色方块放到蓝色方块上”。
- LLM 规划优先，LLM 不可用时回退本地规则规划。
- Plan / Validate / Repair / Execute 流程展示。
- 输出完整执行日志和 JSON：输入场景、规划结果、笛卡尔路点、关节轨迹、校验结果。
- 手外眼虚拟相机：可从固定外部视角拍摄 3D 场景截图，并作为可选视觉提示发送给支持图像输入的千问模型。
- 方块/圆柱可拖拽摆放；释放后采用静态放置，避免课程演示中堆叠滑落。

## 运行

```powershell
cd web
npm.cmd install
npm.cmd run dev
```

打开 Vite 提供的本地地址后，可在界面右侧输入 DashScope API Key。Key 只保存在浏览器本地存储，静态网页部署到 GitHub Pages 时也可使用。

如果要测试视觉提示，请把模型 ID 改为支持图像输入的千问多模态模型；如果模型不支持图片输入，系统会回退到本地规则规划。

## 验证

```powershell
cd web
npm.cmd test
npm.cmd run build
```

## 设计边界

当前课程版不直接控制真实机械臂。后续毕业设计可在现有 JSON 输出基础上增加 Python/FastAPI 后端，将 `jointTrajectory` 或更高层的 `cartesianWaypoints` 转换为乐白 Python SDK 指令，并加入真实设备的限位、急停、手眼标定和 dry-run 校验。
