# ArmPlannerAgent Design

## Goal

ArmPlannerAgent is a lightweight embodied-planning demo for the AI4S course. It targets the lab's Lebai LM3 arm and LMG-90 electric gripper as the real equipment path, while the first deliverable runs safely in a Web digital-twin workspace. A user places simple objects in the 3D scene, asks a natural-language manipulation task, and watches the agent produce, validate, repair, and execute a structured pick-and-place plan.

## Scope

The first version focuses on reliable planning and explainable execution, not high-fidelity robot dynamics. The system supports cubes and cylinders on a tabletop, color/name references, simple spatial relations, pick-and-place plans, object binding during grasp, and trajectory-keypoint visualization. The robot rendered in the browser is an LM3-style simplified kinematic model with an LMG-90-style gripper.

Out of scope for the first version:

- Full CAD assembly animation from STEP files.
- Contact-friction-based physical grasping.
- Real RGB-D perception.
- Direct command execution on the physical LM3.
- Training LoRA or other robot foundation models.

## Lebai Equipment Strategy

The project includes the lab's downloaded STEP assets:

- `乐白机械臂/乐白机器人三维模型-LM3.stp`
- `乐白机械臂/电动夹爪模型MG-90.stp`

These files are treated as source CAD assets. The course version does not depend on converting or segmenting them. The browser first renders a code-generated LM3-style digital twin with the correct role, workspace scale, and gripper behavior. A later asset pipeline can convert STEP to GLB for visual fidelity, and a later CAD step can split assemblies by joint for accurate animation.

## Architecture

The project has two runtime modes.

Static mode runs on GitHub Pages. The browser owns the scene graph, uses a local heuristic planner, and optionally calls a user-provided backend URL or browser API key for model planning. This mode is safe for demos but not for production secrets.

Backend-agent mode uses the same frontend and a FastAPI backend. The backend receives the user instruction, scene graph, robot state, and conversation history. It calls qwen3.7-plus when configured, falls back to deterministic planning when no key is present, validates the plan with robot tools, and returns an executable plan plus diagnostics.

The backend exposes a robot-driver boundary:

- `SimLebaiDriver`: executes validated plans in the browser/simulator.
- `RealLebaiDriver`: reserved for future `lebai_sdk` integration and disabled by default.

## Agent Workflow

1. Scene graph extraction: convert the 3D workspace into structured objects, positions, dimensions, relations, and robot state.
2. Task parsing: resolve natural-language references such as "red cube", "it", "the target", or "move it right".
3. Affordance-lite generation: compute grasp/place candidates from object geometry.
4. Plan generation: create a sequence of semantic manipulation steps.
5. Validation: check object existence, reachability, occupied targets, and trajectory safety.
6. Repair: if validation fails, provide a concrete correction or user-facing reason.
7. Execution: animate keypoints and update the scene graph.

## Innovation Points

- Structured scene graph as a controllable proxy for RGB-D perception output.
- Plan-Validate-Repair loop instead of direct LLM-to-action execution.
- Geometry-based Affordance-lite candidates for cubes and cylinders.
- Trajectory keypoint scoring for reachability, path length, and obstacle clearance.
- Conversation-aware object reference resolution for follow-up commands.

## Frontend Design

The app is a tool surface, not a landing page. The left side is a large 3D workspace with a grid table, LM3-style arm, LMG-90-style gripper, colored objects, and trajectory markers. The right side contains the agent panel: task input, scene graph, plan/validate/repair status, trajectory score, and execution log. The top bar has `Plan Mode` and `Real Robot` as explicit modes; `Real Robot` is shown as reserved until the safety layer and SDK connection are implemented.

The UI uses restrained colors: white and light gray surfaces, charcoal text, blue/teal action accents, and green/yellow/red status indicators. It keeps dense information readable for course screenshots and paper figures.

## Testing Strategy

Core behavior is tested before UI polish:

- Object reference resolution and relation extraction.
- Affordance candidate generation.
- Pick-and-place plan creation.
- Plan validation and repair diagnostics.
- FastAPI endpoint shape and fallback planner behavior.

Browser rendering and interaction are verified after implementation with a dev server and visual inspection.
