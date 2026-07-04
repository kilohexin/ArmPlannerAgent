# ArmPlannerAgent Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a runnable Lebai-first ArmPlannerAgent MVP with a Three.js static frontend and FastAPI backend-agent service.

**Architecture:** The browser maintains and visualizes a structured 3D scene graph with a simplified LM3/LMG-90 digital twin. Shared planning concepts are implemented as small deterministic functions. The backend wraps the same workflow behind an API and reserves a driver boundary for future `lebai_sdk` control.

**Tech Stack:** TypeScript, Vite, Three.js, Vitest, Python, FastAPI, Pydantic, httpx, pytest.

---

## File Structure

- `web/`: static frontend app for GitHub Pages and local development.
- `web/src/core/`: typed scene graph, affordance, trajectory, planning, validation logic.
- `web/src/scene/`: Three.js renderer, simplified LM3 model, gripper, and object interaction.
- `web/src/agent/`: backend/static planner client.
- `web/src/ui/`: DOM rendering helpers.
- `backend/app/`: FastAPI service and robot-agent workflow.
- `backend/app/tools/`: deterministic robot tools and Lebai profile constraints.
- `backend/app/agent/`: planner, validator, orchestrator.
- `backend/tests/`: API and tool tests.
- `docs/`: course, interface, development, and paper notes.

## Tasks

### Task 1: Core Contracts and Tests

**Files:**
- Create: `web/src/core/types.ts`
- Create: `web/src/core/robotProfile.ts`
- Create: `web/src/core/affordance.test.ts`
- Create: `web/src/core/planner.test.ts`
- Create: `backend/tests/test_agent_core.py`

- [ ] Write tests for affordance generation, plan generation, and validation failure.
- [ ] Run frontend and backend tests and confirm they fail because implementation modules are missing.

### Task 2: Deterministic Planning Core

**Files:**
- Create: `web/src/core/affordance.ts`
- Create: `web/src/core/sceneGraph.ts`
- Create: `web/src/core/trajectory.ts`
- Create: `web/src/core/planner.ts`
- Create: `backend/app/tools/affordance.py`
- Create: `backend/app/tools/scene_graph.py`
- Create: `backend/app/tools/trajectory.py`
- Create: `backend/app/agent/heuristic_planner.py`
- Create: `backend/app/agent/plan_validator.py`

- [ ] Implement just enough logic to pass the tests.
- [ ] Add repair reasons for missing objects and unreachable targets.

### Task 3: FastAPI Backend Agent

**Files:**
- Create: `backend/app/schemas.py`
- Create: `backend/app/services/llm_client.py`
- Create: `backend/app/agent/orchestrator.py`
- Create: `backend/app/main.py`
- Create: `backend/tests/test_api.py`

- [ ] Implement `/health` and `/api/plan`.
- [ ] Use qwen3.7-plus through an OpenAI-compatible endpoint when configured.
- [ ] Fall back to deterministic planning when no API key exists.

### Task 4: Three.js Frontend

**Files:**
- Create: `web/index.html`
- Create: `web/src/main.ts`
- Create: `web/src/styles.css`
- Create: `web/src/scene/RobotScene.ts`
- Create: `web/src/scene/createLebaiRobot.ts`
- Create: `web/src/agent/agentClient.ts`
- Create: `web/src/ui/panel.ts`

- [ ] Render tabletop, simplified LM3-style robot arm, LMG-90-style gripper, cubes, cylinders, and trajectory markers.
- [ ] Support adding objects, selecting objects, and dragging objects on the tabletop.
- [ ] Support local planning and optional backend planning.
- [ ] Animate semantic plan steps and update scene graph state.

### Task 5: Documentation and Verification

**Files:**
- Create: `docs/技术方案.md`
- Create: `docs/接口说明.md`
- Create: `docs/开发记录.md`
- Create: `docs/论文素材记录.md`
- Create: `README.md`

- [ ] Document setup, runtime modes, API key handling, and project limitations.
- [ ] Run frontend tests, backend tests, frontend build, and backend import checks.
- [ ] Start dev servers and verify the primary interaction path.
