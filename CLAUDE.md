# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ArmPlannerAgent is a prototype for a robotic arm action-planning intelligent agent targeting the **UR5 + Robotiq 2F-85** platform. It implements an end-to-end pipeline: natural language instruction → LLM task understanding → structured scene graph → Plan-Validate-Repair loop → simulated execution in a browser-based Three.js + Cannon-es 3D digital twin.

The app is a **pure frontend** TypeScript + Vite + Three.js application deployable to any static host (GitHub Pages, etc.). The backend has been removed.

## Repository Layout

```
web/                  # TypeScript + Vite + Three.js frontend (standalone)
  public/urdf/        # UR5 and Robotiq URDF model files
  src/
    scene/            # RobotScene (Three.js + Cannon-es), robotModel (urdf-loader)
    core/             # IK solver, planner, trajectory, affordance, sceneGraph, types
    agent/            # LLM client (DashScope), agent dispatch
    ui/               # Panel rendering, log display
docs/                 # Chinese-language design docs and dev log
乐白机械臂/           # CAD STEP models of Lebai hardware (legacy reference)
```

## Commands

```bash
cd web
npm.cmd install                     # Install dependencies
npm.cmd run dev                     # Dev server at http://127.0.0.1:5173
npm.cmd run build                   # Type-check (tsc) + production build (vite)
npm.cmd run preview                 # Preview production build
npm.cmd test                        # Run Vitest tests
npm.cmd test -- --run path/to/test  # Run a single test file
```

The `.env` / `.env.example` files are retained for reference. The LLM API key is now entered directly in the browser UI and persisted to `localStorage`.

## Architecture

### Pure Frontend

The app is a standalone TypeScript + Vite frontend. No backend is required. Two planning modes are available:

1. **LLM mode** (default when API key is set): Sends scene state to DashScope (Qwen) for full plan generation, then applies deterministic safety validation as a guard.
2. **Local mode** (fallback): Uses keyword-based planning in-browser when no API key is set or LLM is unavailable.

### LLM-First Planning with Safety Guard

- **LLM** generates full pick-and-place plans with step-by-step target positions
- **Deterministic safety validation** checks every plan before execution:
  - All referenced objects exist in the scene
  - All target positions are within UR5 0.85m working radius
  - All grasp targets are movable
  - All positions are above table surface
  - Step count is within limits
- **Fallback**: If LLM is unavailable or confidence is too low, a local keyword-based planner handles the task

### UR5 Analytical Inverse Kinematics

`web/src/core/ik.ts` implements UR5 6-DOF analytical IK:
- Based on standard DH parameters (D1=0.089159, A2=-0.425, A3=-0.39225, D4=0.10915)
- Computes all valid solutions by wrist center decomposition
- Selects the solution closest to current joint angles for smooth motion
- End-effector targets are converted to joint angles during trajectory execution

### Physics Engine (Cannon-es)

- World gravity: -9.81 m/s squared
- Table: static rigid body (mass=0)
- Scene objects: dynamic rigid bodies (mass=0.3kg)
- Grasping: `PointToPointConstraint` via kinematic gripper anchor body
- Drag: direct position control with velocity zeroing
- Physics-to-mesh sync runs every animation frame

### Robot Model (URDF via urdf-loader)

`web/src/scene/robotModel.ts` loads URDF files:
- UR5 6-DOF robot arm
- Robotiq 2F-85 gripper (with mimic joint)
- URDF files are in `web/public/urdf/` with primitive geometry (no STL meshes needed)

## Key Files

| File | Role |
|------|------|
| `web/src/scene/RobotScene.ts` | Main 3D scene: Three.js + Cannon-es integration, IK-driven motion |
| `web/src/scene/robotModel.ts` | URDF loading and robot joint control API |
| `web/src/core/ik.ts` | UR5 6-DOF analytical inverse kinematics |
| `web/src/core/planner.ts` | LLM plan construction, safety validation, local fallback |
| `web/src/core/trajectory.ts` | Trajectory waypoint generation and scoring |
| `web/src/core/robotProfile.ts` | UR5 + Robotiq constants and defaults |
| `web/src/agent/llmClient.ts` | DashScope API client with localStorage key management |
| `web/src/agent/agentClient.ts` | LLM-first dispatch with local fallback |
| `web/src/core/types.ts` | All TypeScript types including LLM plan types |

## Key Constraints

- **UI language is Chinese** — all user-facing text, error messages, and repair hints
- **No CI/CD pipeline** exists — no GitHub Actions, no linting config
- **No linting/formatting tools** are configured — TypeScript strict mode provides compile-time checks only
- Scene state is persisted to `localStorage` under key `arm-planner-scene`
- The UR5 profile constants (`maxReach=0.85m`, `payloadKg=5`, `gripperStrokeMm=85`, `safeZ=0.35m`) are the single source of truth
- The backend (`backend/`) has been deprecated — it remains on disk but is not used
