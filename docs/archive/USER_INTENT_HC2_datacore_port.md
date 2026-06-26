# User Intent

- If there are unexpected file changes while the Coding Agent is working, it is the user who is editing the codebase alongside the Coding Agent, and the Coding Agent should (a) think to understand what the user is doing, or (b) ask the user if unsure about anything.

---

## 1. Communication Style

- Write in plain, highly readable language; lead with context (what/why) before details.
- Prefer simple phrasing over jargon, but do not sacrifice clarity for simplicity. Avoid dense wording, invented terms, or unnecessary abstraction language.
- Describe behavior with concrete outcomes (returns, raises, valid/invalid inputs); no abstract phrasing or smarmy/overly clever wording or coding.
- Code must be written simply and clearly — even when behavior is complex — and must be skimmable by a human reader.
- For clarifying questions, be explicit about the exact field/behavior in question and present concrete options/examples so the user does not have to infer unstated assumptions.

---

## 2. Tooling / Environment / Testing

### uv & Docker

- Prefer `uv`/`uv pip` over raw `pip` for Python package/environment management in this repo and related runtime containers.
- Docker workflows that bind-mount this repo: container `uv` commands use a container-local environment (canonical defaults: `UV_PROJECT_ENVIRONMENT=/tmp/datacore-docker-venv`, `UV_PYTHON=/usr/bin/python3.12`, `UV_CACHE_DIR=/tmp/uv-cache`) so the container never rewrites the host repo `.venv`.
- For containerized operator-console use: keep the bridge default bind local (`127.0.0.1`) but env-configurable. Recommended Docker deployment publishes `127.0.0.1:8080:8080` on host and sets `HIVE_UI_BRIDGE_HOST=0.0.0.0` inside the container.
- For containerized deployments talking to host Postgres, override `DATABASE_URL` / `TEST_DATABASE_URL` to use `host.docker.internal` rather than inheriting repo `.env` loopback URLs (host-side `127.0.0.1` is not valid from inside the Docker network namespace).
- Host-side Vite dev for the operator console: HTTP API and asset requests may stay same-origin through the Vite proxy, but the browser websocket connects directly to the bridge on `127.0.0.1:8080` (NOT through `/ws` proxy). Vite must also proxy `/hdr` to the bridge on `127.0.0.1:8080`, or `ViewerEngine.init()` fails on `/hdr/wooden_studio_15_4k.hdr` and the 3D viewer never comes up.
- FastAPI routes should import request-owned DB and RequestContext dependencies from `api.dependencies`. `data_model.model.engine.get_db` remains an `async with` helper for non-FastAPI call sites; HTTP tests should override `api.dependencies.get_database_session` with the `db_session` fixture.
- The Datacore HTTP API exposes domain enum values as exact UPPER_SNAKE_CASE member names. Internal DB/model/service DTOs keep using `IntEnum` storage; route wrappers convert request enum-name strings into domain enums before service DTO construction and serialize response enums back to `.name` strings. Raw integer enum values are not valid at the HTTP boundary.
- FastAPI OpenAPI should describe those enum-name HTTP schemas directly. Do not expose v2 service DTOs with raw `IntEnum` fields as public FastAPI request/response models, because Pydantic emits integer enum schemas for them; use thin API transport schemas that convert to/from canonical service DTOs.

### HC2 testing tiers

- Three-tier split: macOS/source tests may mock ROS; Docker Linux Tier 2 exercises real ROS packages piecewise; Docker Linux Tier 3 launches the full `real_sim` backend graph end to end. Automated Docker ROS tests use isolated container names, isolated DBs, developer-friendly defaults — must NOT piggyback on the destructive shared `full_rebuild_reseed_hydrate.sh` workflow or shared dev/prod DB state.
- For Dockerized HC2 ROS test seeding, use the real YAML production seed mechanism from `src/data_model/seed/hive_seed/` but seed from test-owned copies of `HIVE-D.yaml` and `HIVE-D-labware.yaml` under `test/` so test topology/inventory does not drift when production HIVE seed config changes.
- Default macOS development/test runs must not fail collecting Linux-only ROS package tests. Any `hc2_ws` test module depending on installed ROS packages (e.g. `hive_ui_bridge`) skips before those imports unless on Linux with a ROS install exposing the package.
- HC2 runtime-shell tests that exercise UI summary or manual-ops projection use real `Live*` dataclasses for parts and tools, not ad-hoc namespace objects.
- For HC2 and UI-bridge testing, keep both layers of coverage when possible: source-level tests with monkeypatched ROS/generated-interface modules keep working outside ROS; separate installed-package / built-ROS tests cover the real runtime environment when available.

### Build & rebuild flows

- For KUKA startup or `ros2_control` hardware-plugin changes, audit both `datacore/hc2_ws` launch/runtime expectations AND sibling `kuka_ws` plugin implementations together. The build compiles both the real RSI driver and the mock hardware plugin even when runtime later launches only one mode — a "single warning" during rebuild can require fixes on both sides.
- `kuka_external_control_sdk_examples` is not runtime-relevant for HC2. Exclude it from runtime `kuka_ws` rebuilds unless a task is specifically about upstream SDK examples.
- `iiqka_moveit_example` is an upstream MoveIt demo package, not an HC2 runtime dependency. Keep it out of runtime `kuka_ws` rebuilds; also keep the top-level `kuka_drivers` metapackage out unless that metapackage itself becomes runtime-relevant, because it depends on the demo package.
- HC2's local `kuka_agilus_support` source intentionally removes the upstream Gazebo mode branch and `kuka_gazebo` dependency. Keep `kuka_gazebo` out of runtime `kuka_ws` rebuilds unless HC2 explicitly reintroduces Gazebo support.
- HIVE wipe-and-reseed flow is intentional in this repo's test/dev workflow. Do NOT remove destructive delete-and-recreate behavior from `src/data_model/seed/hive_seed/reseed_hive.py` or adjacent export helpers just to avoid duplicate-name errors on ad hoc reruns; fix the specific failing script path instead.
- HC2 developer rebuild flow (`scripts/full_rebuild_reseed_hydrate.sh`) is a shared-state reset path, not a test harness. It destroys/recreates the default `hc2-live-kuka` container, reseeds the main HIVE DB used by the running stack, and hydrates the live runtime session; automated runs use isolated container names, ports, DB URLs.
- The HC2 developer rebuild flow defaults to hydrating the runtime and entering `MANUAL_OP` without loading a schedule. Passing `--sched=true` makes it wait for hydration to finish, generate the Freezer Drawer 4 tray-loop `RuntimeScheduleRecord` from the current reseeded HIVE IDs, load that schedule into HC2, and then enter `MANUAL_OP`.
- Reboot-from-scratch HC2 bringup uses a separate pre-launch cleanup helper that terminates prior HC2 / bridge / KUKA ROS processes before relaunch. Do NOT embed self-kill logic inside `activation_sequence.py` or other in-process runtime activation code.
- HC2 has a minimal same-container fail-fast guard against accidentally launching a second runtime graph. Guard runtime-owning launch surfaces (`production_runtime.launch.py`, `real_sim_launch.py`, paths that include them); bridge-only restart stays exempt.
- The `hive_ui_bridge` frontend source lives at repo-root `frontend/`, but installed deployments serve the built bundle from `share/hive_ui_bridge/frontend/dist`. Repo-root `frontend/dist` is a source-checkout dev fallback, NOT the installed runtime contract. During active iteration in the bind-mounted Docker dev flow, bridge restarts survive stale installed hashed-asset symlinks and pick a newer repo-root `frontend/dist` automatically.
- The operator-console frontend uses pnpm, not npm. Use `pnpm --dir frontend install --frozen-lockfile`, `pnpm --dir frontend typecheck`, `pnpm --dir frontend test`, and `pnpm --dir frontend build`; do not regenerate npm lockfiles for `frontend/`.
- Frontend-owned HDR environment assets live at repo-root `frontend/public/hdr/`. The bridge and standalone viewer dev server may expose them at `/hdr/*`.
- Runtime viewer STL and URDF requests use `RuntimeManifest.asset_roots.asset_version` as the cache-buster. The version is derived from served STL/URDF bytes, and asset-version changes must force a viewer scene rebuild even when frame/entity topology is unchanged.
- Runtime schedule records persist authored action definitions and computed schedule diffs, not baked hardware command params. Command params are dispatch-time state because they may depend on live geometry and on completion of earlier actions in the same schedule.
- Schedule restart means rewinding the loaded study schedule as if it had just been loaded: every action goes back to `QUEUED`, runtime-only fields such as baked command params, start times, status details, and completion feedback are cleared, the run is unblocked, and the release cursor returns to the head. This is distinct from retrying one failed action.

### Refactor / performance preferences

- In `hc2_ws/`, prefer targeted refactors that reduce code length, cut routing/passthrough layers, and simplify ownership boundaries without changing runtime behavior. Favor cleaner direct flows over extra wrappers, round-trips, or non-canonical ROS indirection.
- For HC2 action-time wait optimizations, prefer leaf-local conditions/events and simulator-owned exact deadlines over shared wait frameworks. Real hardware confirmation and probe-contact paths stay direct and low-latency; only in-memory simulators that already own scheduled transition deadlines block on those deadlines instead of fixed polling.
- HC2 full-auto dispatch must stay event/frontier-driven. The hot path is `ExecutionManager.next_dispatchable_action()`, which should use indexed temporal eligibility and precise requirement invalidation rather than schedule-wide scans. Any structural mutation to a live schedule lane must rebuild that lane's `DispatchFrontier`.
- High-frequency part telemetry should not rebuild broad scheduler snapshots. It should update the single affected `RequirementWatchSnapshot` row and requeue only actions parked on the changed watch key; whole watch refreshes are reserved for mixed reducers or lifecycle paths that can touch several state tables at once.

---

## 3. KUKA Arm Frame Triad (`ati_tcp` / `live_pose_cs` / `dc_kuka_convention_cs`)

Three distinct frames govern arm/tool semantics and must never be conflated:

- **`ati_tcp`** — native ROS/MoveIt TCP and physical toolchanger coupling point. MoveIt semantic planning, MoveIt terminal pose feedback, and measured arm telemetry all describe `ati_tcp` — never `tool0`. The `tool0` vs `ati_tcp` ~31 mm wrist offset is a real correctness bug, not a naming mismatch.
- **`live_pose_cs`** — the moving Datacore semantic arm frame, position-coincident with `ati_tcp` but in the rectified Datacore convention basis. Telemetry/current-state only — never used to derive static native-vs-semantic transforms, and never assumed basis-interchangeable with `ati_tcp`.
- **`dc_kuka_convention_cs`** — static parent-adaptor frame above `live_pose_cs` at zero translation with a pure rotation. Encodes the seeded `native_home_pose_cs → dc_kuka_convention_cs` rectifier for converting authored Datacore target poses into native `ati_tcp` MoveIt request poses.
- Tesseract OPW IK for HIVE-D uses the analytic KR10 chain tip `tool0`; motion requests remain authored and reported at `ati_tcp`. The fixed `tool0 → ati_tcp` transform is an explicit TCP offset from the URDF, not part of the OPW `c4` geometry. OPW solutions must be filtered with the narrowed `vivo_kr10_r1100_2` joint limits, not the broader generic KR10 limits.
- HIVE-D KR10 `joint_6` uses the URDF right-hand-rule convention around the joint's local `+Z` axis. From the operator ego view looking from elbow to wrist, positive `joint_6` is counterclockwise. The intended narrowed `joint_6` range is `[-270 deg, +90 deg]` (`[-4.71238898038469, +1.5707963267948966]` rad).

Conversion rules:

- Authored `MoveArm` / approach-path targets are always in the rectified Datacore basis. Plain target conversion AND object-centric backsolver conversion must require `dc_kuka_convention_cs` and convert through it; missing the rectifier must fail fast, not fall back to interpreting authored bases as native TCP.
- `LiveArm.hc_update_live_pose_from_quaternion(...)` must convert measured native `ati_tcp` poses (in `base_link`) through the seeded rectifier before mutating `live_pose_cs`.
- The seeded `live_pose_cs` is created at hydration time as an identity child of `dc_kuka_convention_cs` (not authored in YAML).
- HIVE-D long-term direction: when URDF/native `ati_tcp` is realigned to the Datacore convention with the same fixed `tool_interface` offset, `dc_kuka_convention_cs` becomes an identity rectifier and direct-tool SceneBridge attachment uses identity pose on `ati_tcp`.

---

## 4. Tool Attach / Coupling Frames (`tool_interface` / `tool_mount_cs`)

- `tool_interface` is the wrist/toolchanger collision-mesh link. The physical coupling point that mates with a tool's pickup mount is `ati_tcp` (see §3), not `tool_interface`.
- For directly mounted tools, `tool_mount_cs` is the mating interface. On latch it snaps coincident to the arm coupling seam with identity local pose — never preserve or project a measured live offset for the direct latch pose. Browser-side mounted-tool first attach after reload/reset follows the same rule (snap to canonical local mount under the arm anchor, or wait for coherent frame data; do not preserve pre-reparent world offset).
- `SceneBridge` is the only encoder of MoveIt attach/detach diffs. The canonical placement contract is the stored `AttachmentBinding` (`parent_frame_id` + `attachment_pose` + `touch_links`); MoveIt attached geometry uses that exact placement, not a copy of a world collision object reinterpreted through a separate retained helper.

---

## 5. Collision Policy & ACM Scopes

Three separate ACM scopes for tool/attached objects (do not widen one scope by copying another):

- **Attached** objects use the persistent touch-link list `tool_interface`, `flange`, `link_6`, `link_5`.
- **Unladen pickup approach** allows only `tool_interface` and `flange` against the parked tool via `MoveitRequest.allowed_collision_pairs`.
- **Startup parked-tool ACM seeding** is `tool_interface` only.

Additional policies:

- Robot-link `tool_interface` is also allowed against every startup collision object in the tool collision group — seed those ACM pairs only for objects labeled `collision_group="tool"`.
- For `MoveArm` to a `LidPosition` with a suction cup: if the suction cup is empty and the target LidPosition currently holds a lid, allow suction-cup-to-lid collision for pickup. If the suction cup is already holding a lid and moves to a LidPosition, allow held-lid-to-owner collision where the owner is the container, tip rack, or disk that owns that LidPosition. Do not substitute the owner object for the parked lid during pickup.
- When a tray gripper carries a tray, the carried collision group includes the held tray, its direct contents and lids, and any trays stacked in TrayPositions owned by that tray, recursively including those child trays' contents. This whole subtree moves with the arm and should be internally ACM-allowed as one carried cluster.
- One robot-vs-world ACM exception: when a startup world collision object uses `ObjectMeshModel.PART_MICROSCOPE`, SceneBridge allows that object to collide with `base_link`. Do not seed when no microscope object exists.
- Startup loading is intentionally two-phase for mounted tools: every meshful tool (including a tool already mounted on an arm) must appear in the startup world manifest and load into SceneBridge as a world object first; activation then replays attach requests for the mounted-tool subset, while parked tools stay world-resident.
- Tesseract collision mesh mode has three explicit Dev Overlay choices. `convex_hull_only` converts every non-robot mesh collision object to a convex hull. `arm_mounted_tool_meshes` keeps world objects as convex hulls but uses exact meshes for attached/arm-mounted tools. `mesh_only` uses exact meshes for all non-robot mesh collision objects. The KUKA arm itself is always convexified by the Tesseract URDF load path, regardless of this operator mode.

---

## 6. Datacore-vs-ROS Unit & Frame Contracts

- Datacore spatial state stays in **millimeters**; convert only at ROS generic-geometry boundaries. `FrameStore`, hydration geometry, baked `QuaternionPose`, and browser frame payloads are mm. `geometry_msgs/Pose`, TF translation fields, and TF-like XYZ arrays for MoveIt / probe / TF projection / SceneBridge use **meters**. `CalibrateTips.tip_end_offset_pose` is an explicit mm exception for now.
- MoveStage action/profile inputs expose acceleration, not separate deceleration. Unless explicit decel fields are later added, `linear_accel_mm_s2` must govern both linear acceleration and deceleration, and `angular_accel_deg_s2` must govern both angular acceleration and deceleration.
- Browsers/frame consumers do not read `LiveArm.live_pose_cs` directly — they follow projected TF and bridge `frame_updates`. Arm `live_pose_cs` is projected as a *dynamic* TF frame and refreshed via `tf_projection/update_frame_pose`, not via semantic caches or UI-specific arm telemetry payloads.
- `FrameStore` is the persistent live topology, not a scratchpad. Read-only helpers such as distance checks, `expressed_in(...)`, manual-ops default projection, and UI manifest projection must create transient frames without registering them unless the caller explicitly asks to persist a frame.
- Robot root placement and the `base_link_cs` / `world → base_link` split are governed by §"KUKA Arm Startup...".

---

## 7. `MoveArm` Request / Execution Contract

- Canonical retained `MoveitRequest` shape for `MoveArm`: optional validation-only `start_pose`, optional `departure_stub_poses`, optional `arrival_stub_poses`, one exact `destination_pose`. The leaf triages plain pose-goal vs segmented motion from those fields. No legacy flat `poses[]` list.
- `APPROACH_PATH_AUTO` for MoveArm approach paths should resolve a concrete `ToolApproachType` when live state and authored approach paths make one determinable. The manual-ops UI should display that same resolved type in its selector even when the user leaves Auto selected or leaves hidden departure controls unexpanded. Only an explicit Disabled path sends `None`; `DEFAULT` is a real `ToolApproachType`, not the Auto token.
- `start_pose` validation runs ONLY when the caller provides a true expected current pose. If absent, leave `validate_start_pose=false` — never fabricate from `departure_stub_poses` or the first departure waypoint.
- `departure_stub_poses` carries every authored departure stub in traversal order and **excludes** the current pose. The current pose enters the leaf only via `move_group_->setStartState(current_state)`. The leaf consumes all departure stubs; the first arrival stub is consumed separately as `bridge_goal_pose`.
- If the arm has no current semantic destination (`current_destination_id=None`), auto departure stays empty: no path lookup, no waypoints, no invented start coordinate-system ID.
- Waypoint-only motion contract at the ROS boundary: leaf starts from current robot state; the terminal target is the last pose in the concatenated departure/arrival waypoint list. Datacore does NOT send separate source-position or destination-position endpoint poses — any required landing pose must already exist in the authored approach-path waypoints.
- For `MoveArmToPosition`, explicit `departure_approach_path=None` / `arrival_approach_path=None` is intentional pathless motion (NOT AUTO fallback). AUTO must still raise on missing required stored paths. Pathless `MoveArmToPosition` still appends the target position frame as the terminal arrival pose; if the mounted tool has multiple pickup-centerpoint CSes, the action requires explicit `arrival_tool_approach_type`.
- AUTO path lookup on positions: if the arm has a mounted tool, the selector must use that mounted tool's definition ID even on `ToolPosition` targets — never special-case to fall back to the `tool_definition_id=None` pickup bucket.
- `MoveArm.resolved_departure_tool_approach_type` / `resolved_arrival_tool_approach_type` are baked runtime metadata, not authored schedule inputs. JSON round-trips rebuild only from init fields and must not feed those resolved fields back into `__init__`.
- Manual `MoveArm` thin specs may carry separate `velocity_fraction` / `acceleration_fraction` and an optional `motion_mode` (`ArmIntermediateMotionMode` enum name) in `action_def_params`. Bake copies them into `MoveArmCommandParams`; the ROS boundary maps them to `MoveitRequest.velocity_scaling`, `MoveitRequest.acceleration_scaling`, and the constraint preset fields.
- `MoveArmToChuteLane` requires `PartStateSnapshot.chute_lanes[(chute_id, lane_index)].dropoff_coordinate_system_id` — invalid otherwise.
- A successful `MoveitTrajectory` completion currently requires `success=True` AND a `final_pose` in the requested `pose_root_frame_id`; the runtime treats missing or frame-mismatched `final_pose` as a failed move-arm completion.
- Tesseract `MoveArm` planning must start from a complete finite manipulator joint map. If the resolved live/env start state is missing any manipulator-group joint, fail the dispatch with `start_state_missing` rather than filling zeros, treating it as a cache miss, or continuing into OMPL/IFOPT.
- Tesseract plan-cache direct-hit validation is a fast reuse gate, not the original planner's safety proof. It should use Tesseract's discrete LVS collision check (`LVS_DISCRETE`) with clear performance logs; do not move this cache path back to continuous LVS without fresh crash/performance evidence.

---

## 8. Segmented `MoveArm` / Bridge Planning

- Departure and arrival approach paths are boundary-condition stubs for ONE fused motion — not separate trajectories that may stop or reverse at the welds. With seam tangents from two authored stub waypoints, the bridge blends continuously and must avoid explicit seam-stop behavior.
- For Tesseract v3, the seed planner should prioritize producing ANY collision-valid global seed quickly over seed smoothness. Target budget is roughly 500 ms for pre-IFOPT seed generation, leaving roughly 500 ms for IFOPT to smooth/refine before execution. OMPL/Descartes should stop as soon as they have a usable path; IFOPT owns path quality.
- The Tesseract staged seed planner should preserve cheap feasibility breadth before IFOPT: Descartes top-terminal seam sets on departure and arrival, path-consistent `joint_6` winding expansion, all cheap direct seam-pair checks, then one bounded multi-start/multi-goal RRTConnect bridge solve. Do not downscope this to one selected seam or one-start/many-goal OMPL just to make the adapter smaller.
- Tesseract approach-path motion should follow authored sequential ApproachPath waypoints as local Cartesian straight-line moves when the waypoints share orientation. It should not use arbitrary joint-space OMPL detours between adjacent approach stubs and then rely on TOTG/IFOPT to hide the shape.
- ApproachPaths are terminal guidance for tight physical corridors, not just soft hints. Radius values may be used to round corners and smooth entry into the path, but the final insertion/extraction portion of an ApproachPath often needs to remain a true straight Cartesian line: pipette tips into wells, toolchanger into a toolholder, tools into racks, etc.
- Approach corridor blending should treat the whole authored ApproachPath as one local Cartesian corridor, not as independent point-to-point joint-space goals. Radius values create local fillets/tubes around interior corners; zero or critical terminal spans stay exact straight-line Cartesian segments. The bridge should smooth into the corridor tangent at the seam, but must not deform the critical insertion/extraction line.
- For a nonzero-radius ApproachPath corner, the corner waypoint's orientation remains a path key even though the arm no longer passes through the corner position. Map that orientation to the fillet's path-length midpoint and generate sample orientations with quaternion interpolation/spline through the exact neighboring zero-radius anchors; do not interpolate orientations in Euler angles or leave generated fillet samples with copied/stale endpoint orientations.
- ApproachPath first/last points do not create positional corner fillets from their own `radius_mm`, but their orientations are still exact endpoint keys in the path-length orientation curve. If the endpoint orientation differs from its neighbor, the tool should rotate smoothly along the adjacent path span rather than holding a stale endpoint orientation until the first interior corner.
- KUKA wrist winding is a first-class seed-planning concern. The important branch ambiguity is often `joint_6` near its +/-240 degree limits, not elbow/forearm inversion. Seed generation should consider equivalent TCP orientations / IK solutions that unwind `joint_6` away from the current limit instead of greedily taking the numerically nearby solution that drives farther into the limit.
- Current HIVE-D branch reality is narrower than the original seam-pool design assumed: `joint_6` is intended to stay within `[-270 deg, +90 deg]`, `joint_4` cannot rotate through 360 degrees, and the elbow is restricted from jumping to its second valid IK branch. In normal moves there is typically one reasonable departure branch and one reasonable arrival branch, so bridge speed work should focus first on faster convergence for that single bridge problem rather than on broad seam-pair search.
- When staged seed planning has multiple departure/arrival seam branches, prefer stub paths with the smallest cumulative bounded-real `joint_6` travel first, then pair those low-travel stubs by smallest seam `joint_6` difference. Departure candidates must preserve the live bounded-real start joint state; do not prioritize a seam alignment that required a 2π wrist spin inside an approach/departure stub. The broader candidate pool should remain as fallback.
- Tesseract v3 defaults to preferred-only wrist-winding search. Reserve/non-preferred +/- 2π joint_6 winding fallbacks are an explicit IFOPT Dev Overlay opt-in via `enable_wrist_winding_fallbacks`; keep the default off unless the operator chooses broader fallback search.
- Tesseract staged-seed OMPL fallback must preserve wrist-winding priority by trying preferred low-travel / low-`joint_6`-delta seam pairs first, but it must not burn a protected wall-clock window retrying the same preferred pair. If a preferred pair fails immediately, especially because its bridge endpoint is colliding, move on to the next unique pair or report the endpoint collision.
- Executed bridge endpoints stay on the authored seam poses. Do not append synthetic beyond-seam handle poses as commanded waypoints or as bridge start/goal — that creates visible overshoot and turn-back.
- `ToolApproachType` is a coarse selector for the right `ApproachPath` family (`TOP_GRIPPER` vs `SIDEWAYS_GRIPPER`). Exact bridge orientation is NOT encoded on `ToolApproachType`; it comes from the seam waypoint poses (last departure + first arrival) plus traversability-map channel quaternions for the selected tool.
- The bridge between departure exit and arrival entry is not a direct/near-straight connector. In the HIVE enclosure it routes around real world geometry and self-collision, so the practical first-pass design treats departure/arrival as local takeoff/landing corridors and the middle as a topological enclosure route through curated transit regions, solved leg-by-leg.
- ApproachPaths own endpoint insertion/extraction, including moving a tray into or out of a fridge/freezer drawer. If both endpoint ApproachPaths are feasible but the direct bridge intersects an obstacle, the bridge planner is expected to route over/around that obstacle; do not diagnose that as an ApproachPath drawer issue without evidence from the approach segment itself.
- Approach/corridor waypoint `radius_mm` values are tolerance spheres for corridor-following and join smoothing, not dead metadata. In the Tesseract segmented path they are carried to the leaf as toleranced Cartesian corridor constraints; only true start/end anchors are hard-constrained, while interior departure/bridge/arrival guide points initialize the optimizer and remain movable so IFOPT can round seam corners.
- Tesseract v3's `inert_approach_paths` tuning mode is the deliberate exception to radius-sliding: every ApproachPath waypoint before/after the bridge is emitted as a fixed joint-state waypoint regardless of `radius_mm`, so MoveIt's approach paths stay exactly intact while IFOPT smooths only the bridge against those fixed boundary conditions.
- Even when ApproachPath `radius_mm` values are transported for request/cache parity and for non-inert debug mode, the intended v3 staged-seed path with `inert_approach_paths=true` must ignore those radii for departure/arrival motion freedom and emit fixed `StateWaypoint`s at the IFOPT handoff.
- Tesseract v3's `allow_descartes_approach_collisions` tuning mode is an operator debug escape hatch for departure/arrival approach geometry: Descartes should keep collision distance as a cost but not reject approach-path samples or edges, and the staged bridge/IFOPT handoff must not count the two approach-owned seam endpoint states as bridge collisions. Bridge interiors still collision-check normally. Default is strict collision rejection.
- Allowed-collision Descartes costs must stay valid for the graph search. Collision penetration may be represented as cost, but it must not become a negative Dijkstra edge weight that prevents an otherwise connected approach path from producing a seed.
- Tesseract keeps one live scene environment per collision mesh mode. The Dev Overlay exposes separate mesh-mode selectors for seed-path collision checks and IFOPT/contact checks: default seed path = full meshes, default IFOPT = convex hulls. Switching either selector chooses an already-mirrored environment instantly and must not trigger a scene rebuild.
- Tesseract does not mirror MoveIt through `/monitored_planning_scene` or `/get_planning_scene` in production. SceneBridge is the source of truth for Tesseract planning scene data.
- SceneBridge serves `scene_bridge/get_tesseract_projection`: a flat planning snapshot with current world-resident object poses, current arm-carried object poses, versioned geometry only when Tesseract lacks it, and the full projection-level allowed-collision pair set.
- Tesseract must pull and apply the flat projection before every planning dispatch. `projection_revision` is for logging/diagnostics only; it must not be used to skip pose application because TF-only pose changes can happen without a revision bump.
- Runtime `MoveArm` dispatch must not wait on Tesseract drain/resync services. The runtime may still wait on `GeometrySyncCoordinator` for MoveIt/SceneBridge geometry convergence before scene-gated actions; Tesseract then pulls current projection data itself.
- Future Tesseract trajectory preplanning should treat HIVE planning state as deterministic from the current runtime boundary plus future schedule diffs: inventory, held objects, mounted tools, moving drawers/deck/hotels, stage pose, and canonical arm IK under the narrowed joint limits. Reprobing is the explicit dirtying exception: if it moves a scene-object frame, all trajectory caches derived from the old scene must be marked dirty and preplanning restarted from the new current state.
- Optional Tesseract prewarm is only a call to the same projection pull/apply path used by planning. Do not add a separate push payload or teach Tesseract HIVE frame/drawer/tray semantics.
- MoveIt's planning-scene path remains separate for MoveIt/RViz/debug consumers. Do not tune `/monitored_planning_scene` publication around Tesseract needs.
- The final arrival span into `destination_pose` is a terminal insertion rail. When an arrival path has at least one authored arrival waypoint, Tesseract staged seed planning should generate a few intermediate Cartesian samples between the final authored arrival waypoint and the destination so Descartes realizes that last span as one continuous IK branch, without requiring operators to author extra waypoints.
- Tesseract v3 exposes two mutually exclusive seed-shape toggles in the IFOPT Dev Overlay: terminal arrival rail samples and ApproachPath corner rounding. Terminal rail samples are the default simple mode; corner rounding owns the whole sampled approach corridor and disables the rail-only insertion mode while it is enabled.
- Bridge planner: stay leaf-local in `kuka_moveit_cpp` and use MoveIt as an IK/collision/bridge-planning toolkit. Use OMPL `APSConfigDefault` for one bridge request to search through multiple internal `RRTConnect` planners and hybridize/shorten within the time budget. Plain no-stub `MoveArm` stays on the existing pilz-owned branch.
- Generate several valid bridge candidates per execute. Current selection priority is intentionally narrowed to: (1) shortest TCP path, (2) alignment to the direct bridge-start/bridge-goal line. Keep `joint6_total_rotation` and `joint1_curvature` in bridge debug metadata only — do not use them to choose the winner until the user re-enables them.
- Time parameterization runs ONCE over the final stitched joint path. The first-pass timing stack is explicit leaf-local TOTG followed by Ruckig smoothing using real per-joint velocity/acceleration/jerk limits.
- Bridge shoulder shaping is derived locally from authored stub tangents: departure uses last-two stub points or `current+one`; arrival uses first-two or `one+destination`. The welded path is stitched once and retimed once after handles are inserted.
- Source-side and arrival-side seam states must each be represented by ONE shared joint state before global timing. Do not stitch two different waypoint states at the same absolute seam timestamp; normalize each solved piece to its exact input start state, drop the duplicate seam waypoint, and let global timing assign roll-through velocity across that single seam state.
- When HC2 already has a fixed continuous Cartesian arm path from an upstream bridge/traversability solver, do not re-discover the enclosure route in MoveIt. Preserve the solved translation path, inherit departure/arrival waypoint orientation from authored waypoint frames, interpolate bridge orientation only between seam quaternions, and run one leaf-local TOTG + Ruckig pass over the realized joint chain. The realized retimed trajectory must still be collision-checked against the real MoveIt scene before execution.
- When realizing oriented arm trajectory from a preplanned translation path, the sparse native waypoint quaternions and the translational path must come from the same resolved authored motion bundle (same resolved departure/arrival paths, same reversal, same backsolver choice, same terminal-destination append policy).
- For traversability-aware `MoveArm`, `traversability_runtime_node` is the direct caller of the HC2-owned `n1` translational solver because it already owns the traversability volumes in memory. `MoveArmCommandParams` carries planning inputs only; `execution_manager_node` keeps schedule dispatch and requests the Cartesian path over a compact typed runtime service seam (no cross-process map copy).

---

## 9. Traversability Runtime, Masks, Scan Modes

### Ownership & masks

- Traversability is a dedicated-runtime HC2 feature. Browser selects tool/config/show state and invokes `Map/Pause/Reset/Save`; the dedicated `traversability_runtime_node` owns the serial voxel scan, return-to-start, matrix mutation, persistence, shell extraction, and future path planning. NOT browser-owned, NOT `ExecutionManagerNode`-owned.
- Traversability masks are tool-specific durable data on a `HiveInstance`. The dedicated runtime is the SOLE live owner of in-memory matrices — `HiveLiveState` does not also own a second copy. The selected tool owns which mask is loaded/shown/resumed/reset/saved even if a different tool is physically mounted; on mismatch the runtime keeps separate provenance for the actual mounted tool.
- The mismatch case is intentional, not a validation gap — operators map tool A using a conservative larger safety mesh while still saving under tool A's `destination_tool_id`.

### Matrix shape

- World-frame mm voxel grid stored as unsigned NumPy `uint8`/`uint16`/`uint32` bitmask. Highest bit of the chosen dtype is the reserved `tested` bit; lower bits are ordered channel-result bits.
- Runtime/viewer layout uses the unsigned voxel-major word array directly. Postgres archive is intentionally different: one packed compressed bitplane per nonzero used bit (measured plane + one per used channel bit), not a monolithic runtime-word blob.

### Scan execution & seeding

- Reuse `runtime/command_arm_pose` motion semantics in sim. IK failure → no move and `0` channel result. IK success → controller motion is sent even when collision is reported; collision affects the stored result bit, not whether motion is attempted. This reuse is intentional because the workflow is sim-only — treat the seam as a debug/sim boundary, NOT the canonical long-term production arm-motion boundary for traversability.
- During mock/real-sim KUKA scan execution, the runtime temporarily disables the mock hardware's `realistic_joint_following` via the private low-level `set_realistic_joint_following` service. Snap-to-command stays active for the whole scan + return-to-start, then normal realistic following is restored after pause/finish/abort. This avoids seeding later voxel IK from a lagging measured catch-up pose.
- Mask progress is voxel-atomic, not channel-atomic. The reserved `tested` bit is set only after all configured channels for that voxel are measured and packed; resume finds the first voxel whose tested bit is `0` in canonical scan order.
- For `BATCH`, semantics are independent static occupancy checks for the next ordered voxel targets from one fixed cached batch seed — not chained through one solved target into the next, and not from the arm's live current pose. In-batch targets reuse that same cached seed.
- For `BATCH`, real per-request fractional IK timeout matters near the arm base. Do NOT route batch IK through MoveIt's `/compute_ik` (the installed service drops `timeout.nanosec`, creating a hard `sec >= 1` cutoff). Keep batch IK local in `kuka_moveit_interface` with direct `RobotState::setFromIK(..., double timeout, ...)`.
- In `BATCH`, inter-batch arm motion is operator visualization only: must not become the authoritative seed for later batch solves and its failure must not write voxel truth or fail the scan.
- In `SINGLE`, each voxel/channel test is the real `CommandArmPose` attempt; the arm's current position is the natural next IK seed because failed targets do not move the arm. `SINGLE` does NOT depend on a separate cached fallback/reference anchor.
- Batch seeding is not optional. `BATCH` must use the one cached known-good seed captured before the scan starts; `SINGLE` must use the arm's actual current pose as the next seed after each real voxel move.
- The dedicated traversability runtime process must spin with `MultiThreadedExecutor` because `ManageTraversabilityScan START` synchronously queries `runtime/get_manual_arm_pose_context` from inside its own service callback before the scan worker starts. Plain single-threaded `rclpy.spin(node)` deadlocks that nested wait.

### Subset rescans & direct edits

- Subset rescans are targeted overwrites inside one existing tool map. They reuse stored map bounds, voxel size, root frame, and channel set; they must NOT reset the map, auto-save it, or widen the scan outside the selected sub-volume.
- Rescan workflow is intentionally available only after the selected tool map is `COMPLETE`. This preserves the one-measured-bit-per-voxel model and avoids partial-map channel-scoped resume semantics.
- Direct region edits (`Set Traversable` / `Set Non-Traversable`) mark touched voxels measured, preserve untouched channel bits, and apply to the selected channel or all channels via the UI's all-channel modifier.

### Viewer overlay

- Debug overlay only. Do not add traversability geometry to manifest topology or websocket hot state. Publish only a compact retained global scan-status topic; fetch selected-tool shell-surface chunk payloads on demand.
- Pose Ghosting is viewer-local display state over backend-published trajectories. Fresh page loads should default Auto Ghost Display to on and sample every trajectory waypoint (`Waypoint interval = 1`).
- Surfaces keep semantic traversable/nontraversable colors but use the same lit translucent material family as `PART_STAGE_BASE` (normals + shadow casting/receiving), not flat unlit debug materials.
- Opacity is a local viewer control layered on overlay materials — updates the current overlay immediately without changing runtime data, saved maps, or scan request payloads.
- Map-volume and rescan-volume boxes are viewer-only preview overlays with session-local mode memory. Out of manifest topology, runtime transport, hover/raycast, and outline selection. Map-volume outline preview shows while the Traversability section is open even if `Show Traversability` is off; solid map preview follows the actual map-visibility toggle. Rescan preview only when both Traversability and Rescan sections are open. Both use authored bounds + render-only anti-intersection padding (`x/y min -0.5 mm`, `x/y/z max +0.5 mm`, `z min +0.5 mm`).
- Default new volume draft: `x=300..600 mm`, `y=100..500 mm`, `z=100..700 mm`, unless a persisted map view overrides.
- Authored slider index ranges are part of the operator's local working state for the currently selected tool — survive same-tool refreshes and `Rescan Subset`, snap back to full min/max defaults only when the selected tool changes.
- Rescan sliders use voxel-boundary indices as authored state; displayed mm labels derive from `lower_bound_mm + index * voxel_size_mm`. Do not treat the stored authored max bound as the interactive discrete limit.
- Axis names like `Y+`, `Y-`, `YZ plane` refer to scene/root-world axes, not the viewer's internal Three.js Y-up reorientation.

### Solver canonicalization

- The new corridor-based motion solver under `src/state/live/motion_solvers/new/` canonicalizes raw traversability inputs onto one solver scale: default `resampled_voxel_size_base = 25.0 mm`, keep input grid only when its voxel size is in `[20.0, 30.0] mm`, otherwise conservatively resample so any target voxel that overlaps even slightly with a blocked source voxel becomes blocked. Topology search chooses only the branch through free space; the final centerline comes from one global smooth curve solve inside a unified corridor (departure/arrival frusta + bridge safe balls), not from the raw discrete search polyline.

---

## 10. Stage / Chiplet / Disk Geometry

### Stage actions

- Stage scheduler actions are coordinate-system alignment actions, not standalone carousel-slot rotation actions. Single-target chiplet alignment is `MoveStageToChiplet` (wired on `ActionType.MOVE_STAGE_TO_DISK_CHIPLET`) — always targets a named location on a chiplet and aligns it either to the microscope optical-axis projection or to a pipette-channel tip projection. `MoveStageToNamedLocations` is the multi-target batch variant. `current_disk_index` is a derived semantic convenience updated on completion for disk-targeted moves.
- For single-chiplet stage targeting, split XY and theta intentionally. Goal origin = projection onto the named-location plane. Goal in-plane orientation for theta = microscope optical-axis X projected into the named-location plane (rotate stage so named-location X aligns with that projected X). Microscope-only mode: XY also from optical-axis projection. Pipette mode: XY from the projected pipette `tip_end_cs`, but theta still from microscope optical-axis X.

### Stage ROS boundary

- Numeric stage boundary includes `linear_accel_mm_s2` alongside `linear_speed_max_mm_s`, `rot_speed_max_deg_s`, `angular_accel_deg_s2`, `stage_motion_type`. Realistic simulation honors per-goal linear acceleration override; the ACS hardware driver currently accepts the field but ignores per-goal motion tuning (matching its existing treatment of other per-goal stage tuning fields).
- Numeric stage goals must NOT be clamped or preflight-rejected at the ROS retained stage leaf based on static `x/y/theta` travel parameters. The HC2 stage leaf forwards the authored/backsolved target unchanged; Datacore `StageDefinition` travel bounds are metadata only, not a leaf-side execution gate.
- Stage leaf controller status is separate from lifecycle/instance status. `StageState.status` flows into a telemetry-owned `LiveStage.controller_status`; do not reuse `LiveStage.current_status` for that leaf-reported string.

### Chiplet / disk geometry

- Chiplet geometry is split: nominal chiplet XY layout comes from definition-level `DiskChipletCoordinateAssociation` (`FormFactorDefinition` = chiplet form factor); per-disk per-chiplet focus-Z is runtime calibration because physical disks have subtle non-flat bow/saddle behavior measured optically.
- Canonical runtime autofocus sample XY = microscope optical-axis position expressed in the disk's registered coordinate system at autofocus completion time. Do not mix nominal target XY with measured optical-axis XY when fitting/caching disk focus surfaces.
- Disk autofocus model semantics are split: every real autofocus measurement may contribute a surface-fit sample, but exact per-target cache entries are written only when the measured optical-axis XY is within the exact-target tolerance of the requested chiplet/named-location. Off-target measurements must not overwrite exact target caches.
- Disk autofocus and disk registration are runtime-owned coordinator workflows inside `execution_manager_node`, not standalone retained device leaves — they orchestrate stage/microscope/vision leaf calls from the runtime owner.
- `DiskDefinitionCalibrationMarkerAssociation` stores `coordinate_system_id` (not `coordinate_id`) so marker poses remain full frames aligned with FrameStore/hydration parenting.
- Runtime invariant for disk-chiplet motion: every hydrated disk is expected to have a registered disk coordinate system in the FrameStore; chiplet frame parenting uses that registered CS only — missing registered CS is an invariant violation, not a fallback case.
- Hydrated live state does NOT materialize a persistent chiplet/named-location frame subtree per disk. Chiplet and named-location geometry stay on definition-side info DTOs (`DiskInfo` / `FormFactorInfo`) and resolve transiently under the disk's registered coordinate system on demand.
- The HIVE-specific labware seed at `src/data_model/seed/hive_seed/seed_labware.py` carries its own copied Vivodyne 3-chamber disk geometry for `HIVE-D-labware.yaml` disk definitions. Do NOT mutate or couple the canonical `src/data_model/seed/disk_seeds.py` module just to populate HIVE-D labware.
- HIVE labware may define tray-owned `LidPositionDefinition` / `LidPositionInstance` rows for temporary lid storage. These are real LidPositions parented to `TrayDefinition` / `TrayInstance`, not inferred tray-slot occupancy. For HIVE-D, the `tray_disk_lid` tray uses `ObjectMeshModel.CONSUMABLE_TRAY_DISK_LID`, served from `hc2_ws/src/hive_description/mesh_files/parts/tray_disk_lid.STL`, with its LidPosition centered at `[0, 0, 6.35]` mm.

---

## 11. Fridge Drawer Kinematics

- Two runtime kinematic modes with different moving frames:
  - **Sliding drawers**: `drawer_live_cs` is the moving root for the drawer body AND all hosted positions.
  - **FLAP drawers**: `drawer_live_cs` is the generic OPEN/CLOSED drawer access frame; the flap moves only through `hinge_cs`, parented under `drawer_live_cs`.
- Frame-role split (not ad-hoc offsets): `drawer_home_cs` = closed/home anchor; `drawer_open_reference_cs` = calibrated drawer access pose; `drawer_live_cs` = stable runtime/public access frame that copies the open-reference pose when open and resets to closed-home identity when closed.
- Sliding-drawer nominal open reference = explicit seeded extension distance. FLAP open-reference seeded as identity by default, but probing may update it later like any other drawer.
- FLAP articulation stays on the hinge frame only. `hinge_cs` is the flap-only moving frame; opening rotates it about local `X` by `-hinge_rotation_angle_deg` while the hinge origin stays fixed. Browser-side synthetic flap joints and FLAP-specific bridge summary transport are NOT part of the contract.
- FLAP viewer split-body: keep main body on `drawer_live_cs`; hydrate `flap_object_mesh_model` as a real definition/info field; root `::flap` mesh directly at `hinge_cs` through normal frame updates, not through browser articulation metadata.
- HIVE seed configs may leave FLAP `object_mesh_model` empty; the flap-specific `flap_object_mesh_model` is the important optional input for FLAP drawers.
- Drawer/deck actuation ghosts are frontend-owned viewer effects driven by pushed `actuation_state` transitions (`OPENING`/`CLOSING` → terminal `OPEN`/`CLOSED`). Extra browser data is limited to small static setup facts the viewer cannot infer (drawer open-reference frame ID, FLAP hinge angle, sliding-deck extension distance). NOT manifest topology, NOT SceneBridge geometry, NOT `ManualOpsContext` data.

---

## 12. Hotel Kinematics & Telemetry

- Signed semantic angle end-to-end: `0` is untilted; real tilt is negative about local X. Canonical endpoints in the definition layer: `neutral_tilt_deg = 0`, `outward_tilt_deg = -15`. Runtime `current_tilt_deg` and UI `target_tilt_deg` use range `[-15, 0]`. Shared Zaber transport may keep its existing one-sided positive mechanical stroke internally but must translate at the transport seam.
- Authority split: `HotelTilt.action` feedback = authoritative measured final tilt for action completion. `/hotel_runtime_state` = retained latest-state topic that publishes the first configured sample, publishes on real change, and otherwise falls back to a `1 Hz` unchanged heartbeat.
- Hotel deterministic convergence uses measured final tilt from the action/transport completion path, not optimistic `target_tilt_deg` writes and not a second telemetry-only barrier. Shared Zaber hotel transport success already means motion-complete; if hotel joins tracked-root convergence it does so via final hotel feedback and root `hinge_live_cs`, while telemetry remains the steady-state/background projection path.
- Hotel viewer rendering: hotel body mesh fixed at the mounting frame; tilting rack is a synthetic articulated mesh rooted at `hinge_home_cs` driven by `HotelRuntimeState.current_tilt_deg`. No hotel URDF unless the codebase later adds a canonical one.
- Hotel persistent viewer card uses measured live `current_tilt_deg` as visibility authority and hydrated `neutral_tilt_deg` / `outward_tilt_deg` as action targets. One quick action: `Tilt Hotel` only at neutral, otherwise `Untilt Hotel` back to neutral.

---

## 13. Manual Operator Lane

- Manual operator actions live in a separate manual execution lane (`manual_schedule` + own `ExecutionRun` / `ExecutionManager`) that dispatches only in `MANUAL_OP`. Study lane dispatches only in `SEMI_AUTO` / `FULL_AUTO`. Manual actions are NOT free-floating commands and NOT spliced into the study schedule.
- Manual injection reuses the canonical scheduler action format and the same scheduler action registry. There is no separate manual action registry, no manual allowlist in runtime code; the difference is which lane owns the action and that injected actions are tagged `ActionSource.INJECTED`.
- The manual lane keeps the same structural shape as the study lane: real `Schedule` of `Action` objects with the same linked-list semantics, requirements, bake path, dispatch path, completion path. Difference is workflow + source, not data structure.
- Browser/manual-ops UI submits thin manual action specs (NOT full serialized runtime `Action` rows). The public boundary field name is `action_def_params`. Runtime owns constructing injected `Action` objects, including IDs, `ActionSource.INJECTED`, and schedule/runtime internals.
- Manual injection validates hardware-instance UUIDs against currently hydrated live state before mutating the manual queue. Stale/missing IDs → reject the request, enqueue nothing, keep shell out of `FAULT`, surface rejection through Monitoring/recent-events as a `REJECTED` execution event with concrete error text.
- Rejection split: per-action queue-time hardware-ID failures emit a Monitoring `REJECTED` execution event for the rejected action; outer request-envelope failures (wrong mode, no session, bad schema version) stay on the immediate `accepted=false` response path only.
- Rejected manual requests must NOT mutate manual queue projections: no `manual_schedule` change, no `manual_ops_context_revision` bump, no shell progress wake just to surface rejection.
- Automation that chains manual `EXECUTE` actions must wait for the specific submitted `action_id` to reach a terminal `ExecutionEvent` (`COMPLETED` + `success=true` or terminal failure) before submitting the next action. `manual_queue` can briefly read empty before the action event appears, so queue-empty alone is not a completion gate.
- Queue-time hardware validation is action-specific and respects semantically inactive optional IDs. Maintenance/free-space actions must not be rejected just because `arm_id`/`disk_id`/`position_id` are intentionally `None` in that mode.
- Hardware-UUID check stays as a small local guard in `_handle_inject_actions(...)` — not a validator table, registry, or generic ID-scanning framework.
- Operator UI must NOT automate runtime sequencing by chaining accepted commands client-side (no enqueue-then-step as a fake EXECUTE button). If a higher-level workflow like prepend-and-dispatch is needed, the runtime shell owns it as a first-class backend behavior.
- Manual `submission_mode="EXECUTE"` means prepend the injected actions to `manual_schedule` and make that new head the next manual dispatch candidate immediately; rewinding the manual execution cursor is part of that behavior.
- For manual `MoveArm` authoring, `ManualOpsContext` suggestions are current-state hints only. `EXECUTE` uses live state now; queued actions are rechecked against live state at dispatch time and may later wait or fail if tool/destination state changed.
- In manual Move Arm pickup flows with no tool mounted, the selector bucket is the no-tool `ToolPosition` pickup bucket (`tool_definition_id=None`); UI defaults arrival to `ToolApproachType.DEFAULT` when that bucket publishes a DEFAULT path.
- Queued manual actions persist when leaving `MANUAL_OP` and become dormant outside that mode. Once a manual action reaches a terminal state, it is removed from the outstanding manual queue; its history comes from `ExecutionEvent`, not from retaining terminal actions in the manual schedule.
- Manual-actuation clearance requirements tolerate freshly hydrated arms with `PartStateSnapshot.ArmStateRow.destination_id=None`: `ArmNotBlockingDrawer` and `ArmNotBlockingSlidingDeck` treat that as "not currently blocking a specific target" rather than raising and killing the runtime during `MANUAL_OP` dispatch.
- UI summary semantics are split by lane: study execution counts / next candidate describe the study lane only; manual outstanding work is exposed separately as `manual_queue`.
- Operator-console Study Schedule tab should render a full ordered study-lane row projection from `UiExecutionSummary.study_schedule`, not reconstruct rows browser-side from execution events and not show a "deferred projection" placeholder. Manual Queue remains separate under `manual_queue`.
- Study Schedule retry is runtime-owned. The browser may show a retry button on every study row, but only backend-projected `can_retry=true` rows should be enabled; retry resets terminal failure statuses (`FAILED`, `REJECTED`, `CANCELED`) to `QUEUED`, clears baked/terminal action runtime fields, rewinds the study execution cursor, and unblocks release. It is not a browser-side resubmit and it must not retry completed or already queued actions.

---

## 14. UI Bridge / Browser Transport Contracts

- The browser-side HIVE UI transport is a tightly controlled contract boundary. Malformed WebSocket envelopes, unsupported message kinds, unknown frame/entity IDs in manifest or hot-state payloads, missing `mesh_catalog` entries, and unknown URDF `package://` prefixes are producer bugs that raise explicit errors — never skip, coerce, or pass through unchanged.
- Bridge connect bootstrap must not duplicate append-only execution history: `init.recent_events` excludes still-pending incremental `ExecutionEvent` records so those pending events arrive exactly once afterward on the incremental `execution_event` stream.
- Full page reload fetches one explicit bridge bootstrap snapshot before opening the websocket. That HTTP snapshot is the authoritative first-render source for `manifest`, `runtime_status`, `ui_summary`, `manual_ops_context`, `viewer_state`. Websocket `init` is only later catch-up merge, NOT the primary reload contract.
- Websocket reconnect/reset bootstrap seeds the same retained semantic session slices the browser needs after manifest replacement, including `manual_ops_context`. Reconnect/reset must NOT rely on Overview/Form tabs incidentally refetching manual state before viewer/tooltips/manual surfaces recover.
- After `hydrate_hive` reaches `INACTIVE`, the bridge/bootstrap path already exposes the hydrated semantic session (`UiSessionManifest`, `UiExecutionSummary`, `ManualOpsContext`, retained viewer joints) before activation. TF- and MoveIt-derived debug surfaces can still be empty at that point — activation and initial projection have not completed.
- Manifest-driven browser reset is authoritative for topology changes: when a new UI manifest triggers a reset/init cycle, same-cycle pending hot-state and same-cycle non-manifest semantic deltas must NOT leak through as additional stale websocket pushes around that reset bootstrap.
- UI session-manifest rebuild is part of the projection contract, not best-effort chrome. After hydrate/load, if `UiSessionManifest` rebuild fails, the runtime publishes an explicit empty manifest to clear stale browser topology and then enters `FAULT` with the precise rebuild error.
- Quaternion-only projection may omit non-orthonormal `FrameStore` frames only when no projected consumer depends on them. If an omitted frame would break TF dynamic mappings or any emitted UI-manifest entity/root/parent reference, the producer fails with a precise contract error instead of silently dropping that frame.
- Browser lifecycle controls treat hydrate/load HTTP responses as acceptance/rejection only. Completion + success/failure messaging follows pushed runtime state (`RuntimeStatus.active_lifecycle_operation`, `UiExecutionSummary.loaded`, `UiSessionManifest`/`init`, pushed `FAULT`).

---

## 15. UI Summary / ManualOpsContext / Viewer-Only State

- `UiExecutionSummary` and `ManualOpsContext` are separate low-rate projections with different invalidation needs. Browser learns about `ManualOpsContext` changes through `UiExecutionSummary.manual_ops_context_revision`, but runtime only bumps that revision when current `ManualOpsContext` fields actually changed — not for every summary-only telemetry update.
- `ManualOpsContext` owns microscope/manual imaging defaults: per microscope, choose default disk by minimum global Y, then chiplet on that disk closest to the optical axis, then that chiplet's main imaging named location when one exists. Do not recompute browser-side from partial payload data.
- Stage-to-chiplet named-location viewer previews stay on-demand. Do not bloat `ManualOpsContext`, `UiSessionManifest`, or websocket state with every chiplet/named-location frame just to support form preview — frontend requests one resolved frame from the runtime only for the currently selected `disk_id + chiplet_index + named_location_id`.
- `LiveArm` retains the last successful pipette-alignment `MoveArm` target context (`CONTAINER_WELLS` or `DISK_CHIPLETS`) so downstream manual pipette actions can autofill target IDs and well/chiplet context from the actual last alignment move — not by guessing from `destination_id` alone.
- Operator-console MoveStage forms start from shared tuned motion defaults: `rot_speed_max_deg_s=120`, `angular_accel_deg_s2=360`, `linear_speed_max_mm_s=500`, `linear_accel_mm_s2=2000`.
- MoveArm form DiskGripper + Disk Position selection preview stays frontend-local, recoloring the selected disk-position occupant disk and stage clamp using the existing traversability teal/red colors. Authority = typed selection state + `ManualOpsContext` occupancy + `held_disk_id`. Do not widen runtime transport. Effect is pane-scoped selection feedback, must clear on execute, queue, pane close, action-form switch.
- Frontend Dev/Debug Overlay planner toggles are debug-only control surfaces: dedicated debug HTTP/runtime seams, NOT widened manual action payloads/websocket state/`ManualOpsContext`. The `MoveArm` Ruckig toggle's intended scope is only the segmented stub-based path in `kuka_moveit_interface`; plain pilz-planned motions and probe motion stay unchanged.
- Approach-path viewer visualization is a viewer-local debug surface, NOT bootstrap/store/websocket state. Frontend fetches via a dedicated one-shot UI route, caches in `HiveViewer`, invalidates on hydrate completion, renders triads/lines parented under each owning position frame from position-frame-local waypoint poses.
- The temporary `Show Curve` viewer debug feature stays viewer-local: load static curve JSON via one dedicated debug HTTP route, keep selected-curve + transform state local in `HiveViewer`, render as a world-relative frontend overlay (NOT websocket/bootstrap/runtime state). The debug line uses depth testing so scene geometry occludes it — different from always-on-top axes/helper chrome.
- Operator-console recent-events panel renders the canonical `ExecutionEvent` history only — do not synthesize hydrate/load/mode-change entries unless the runtime exposes a separate authoritative event stream for them.
- Monitoring-tab top-row telemetry must not duplicate header-owned operation mode, readiness, or fault status. Use that row for telemetry-specific context: signal freshness/cadence, HIVE chamber temperatures, measured CO2, HEPA readback speeds, and active work/motion.
- Monitoring-tab fridge/freezer telemetry should group fridge-level temperatures and all drawer states together. Drawer actuation history is not useful there; show current drawer state and drawer temperature/setpoint data from semantic runtime state projected through `UiExecutionSummary.part_state`.
- Stage clamp authority + viewer: measured IO becomes shared runtime truth on `InventoryState.disk_positions[*].is_clamp_engaged` (NOT a separate `LiveStage._clamp_states` side channel); `UiExecutionSummary`, `ManualOpsContext`, and bridge-derived `viewer_state` / hot-state `stage_clamps` all follow that inventory-row value. The manifest adds one clamp mesh per `DiskPosition` at local origin `(0,0,0)` under `live_stage_cs` with identity rotation; frontend-only open-clamp visual lifts the clamp mesh `+20 mm` in local Z when `is_clamp_engaged` is false. Closed clamps use viewer_db clamp color `#8c3a25`; open clamps switch only base color to `#999999`; other material properties unchanged.
- Stage viewer rendering stays frame-driven from the hydrated `live_stage_cs` subtree. Carousel uses `ObjectMeshModel.PART_STAGE_CAROUSEL_EMPTY`.
- Viewer persistent tooltip actuation pills use the existing tooltip EXECUTE path and canonical thin manual-action payloads. Fridge drawers and sliding decks follow live `UiSummary.part_state.*.actuation_state`, NOT `ManualOpsContext` booleans (`ACTUATE_FRIDGE_DRAWER` with `drawer_id`/`drawer_state`; `ACTUATE_SLIDING_DECK` with `sliding_deck_id`/`sliding_deck_state`). Hotel tooltip exposes a one-click `TILT_HOTEL` action submitting `hotel_id` with fixed `target_tilt_deg = -15`.
- Overview/manual-ops inventory tip-rack summaries bucket by canonical `TipVolumeType` enum only: 50 µL, 300 µL, 1000 µL. Odd capacities in isolated tests are fixture artifacts, not production scenarios.
- Overview/manual-ops inventory location category counts for disks include only disks currently on known positions. Disks transiently held by grippers stay excluded from category buckets, while title/count total may use full `inventory.disks`.

---

## 16. Telemetry / Projection / Convergence / Retained Topics

- `_request_progress()` is the SINGLE executor-thread reconcile seam for activation, coordinators, dispatch, and retained runtime projections. Idle retained telemetry must NOT act as its wake clock: telemetry callbacks issue at most one direct wake, only on first samples or real semantic changes. Pose-only arm telemetry relies on geometry-sync follow-up, not a second direct shell wake.
- HC2 ROS architecture separates commands, events, retained snapshots, and continuous streams explicitly. Do NOT use fixed-rate state topics as implicit synchronization or completion barriers; publish snapshots on change with only the minimum heartbeat needed for liveness or late-join behavior.
- `arm_runtime_state` is a semantic measured-TCP / motion-flag / tool-lock surface, NOT an articulation or MoveIt-state stream. `arm_runtime_state_bridge` publishes first valid sample promptly after configuration; immediately on real change; low idle `1 Hz` unchanged heartbeat; sampling cap `50 Hz` (`publish_period_sec = 0.02 s`). Runtime correctness must not depend on a fixed high idle cadence.
- Browser arm articulation is a SEPARATE feed from live-state arm telemetry: `ArmRuntimeState` updates Datacore `LiveArm.live_pose` and motion/tool-lock flags; browser URDF arm joints are driven only by `hive_ui_bridge` `joint_updates` sourced from `/joint_states` for entity ID `kuka_arm`. MoveIt `robot_state.joint_state` is debug-only and must not become the normal browser articulation feed.
- Retained stage telemetry: idle publication is a low-rate heartbeat, not an unconditional 20 Hz loop. `stage_state` still emits immediate transition samples for configure/start/terminal state changes; while actively moving may publish up to 20 Hz.
- KUKA `rsi_only` / real-sim control cadence follows the 12 ms frame-update mode: controller-manager loop and `/joint_states` baseline ≈ 84 Hz, NOT 250 Hz, NEVER an uncapped CPU-limited loop.
- For HC2 TF publication cleanup, three contracts stay separate: `TfProjectionNode`'s `1 Hz` dynamic replay timer is the late-subscriber heartbeat; direct stage/hotel telemetry projection is change-driven and must not republish unchanged local poses just because retained leaf heartbeat fired; stamped `tf_projection/update_frame_pose` convergence publishes still go out even when pose is unchanged (SceneBridge waits on the observed stamp for tracked-root coherence). Robot-link TF from `/joint_states` → `robot_state_publisher` is a separate upstream seam — do not conflate with Datacore semantic-frame TF dedupe.
- SceneBridge should always ingest unchanged TF replay stamps into its latest-frame cache and wake condition waiters, but unchanged replay should not trigger root-sync worker work unless a dirty root/object can actually advance from that updated sync frame.
- SceneBridge root-sync work is driven by its tracked sync-frame reverse index. Unrelated changed TF should update the flat pose cache, but must not enter `_changed_frame_ids`, wake the root-sync guard, or retry dirty objects unless the frame is part of a tracked root sync surface.
- SceneBridge deferred root-sync timers should be lazy one-shot executor objects. Do not leave a canceled timer registered while idle; create the timer only for a pending deferred flush and destroy it when the flush is canceled, completed, paused, or no longer needed.
- SceneBridge is intended as a strict TF-driven materializer of the MoveIt world scene. Runtime-owned action/telemetry code may NAME which tracked root must converge and what publish stamp it expects; sync-surface knowledge (e.g. stage root `live_cs_x` depending on both `live_cs_x` and `live_cs_y`) stays internal to SceneBridge — not duplicated across nodes.
- Out-of-band tracked-root terminal transitions detected from telemetry (notably drawer/deck IO-derived) must still participate in the same pending scene-sync convergence machinery that gates the next `MOVE_ARM` / `PROBE_ARM`. Telemetry is a second producer of the same internal convergence path, not a second public barrier contract.
- If a structure anchor is expected to shift during probing/calibration, do NOT keep it on `/tf_static`. Promote to dynamic TF and make its MoveIt world object self-root-tracked, while keeping late subscribers working through a small replay mechanism in `TfProjectionNode` rather than a new replay service surface.
- For HC2/MoveIt integration, distinguish current-robot-state freshness from world-scene freshness. MoveIt's robot-state validity depends on live `joint_states` / `robot_state_publisher`; SceneBridge keeps moving world collision objects aligned with their real TF-backed poses. Event-driven/coalesced SceneBridge updates are acceptable as long as moving world objects still publish timely pose changes when their source frames actually move.
- Default HC2 KUKA launch graph: if an upstream `ros2_control` topic has zero real subscribers, prefer removing it and any repo-local tooling/docs that depend on it rather than keeping a dormant publisher at zero/low rate. `/joint_states` remains the real joint-state authority — do not conflate with low-level debug or observability surfaces.

---

## 17. KUKA Arm Startup, Root Placement, MoveIt Config Authority

- **Root placement vs startup bend are SEPARATE authorities.** Scene placement = hydrated Datacore live state / `FrameStore` projected as the literal ROS `base_link` frame (not seed YAML, not an extra MoveIt placement service, not an alias frame). Startup bent pose = `ros2_control` joint `initial_value` seeds + `/joint_states`. Do not paper over ghost-placement bugs by introducing a second rooted robot-description variant when the real issue is mixed robot-description sources or missing root pose in trajectory visualization.
- KUKA `world → base_link` is launch/xacro-owned via `x/y/z/roll/pitch/yaw` launch args. If those args stay at defaults, MoveIt's robot sits at the URDF origin regardless of seeded HIVE `base_link_cs`. Seeded `base_link_cs` still matters separately for Datacore runtime pose-root semantics, hydrated frame graphs, and browser URDF rooting.
- HC2 MoveIt/Tesseract root placement is baked into the runtime robot description. ROS TF does not publish `world`, and `/get_planning_scene` snapshots may omit the root transform. Tesseract should preserve the URDF-loaded root joint unless a scene snapshot explicitly provides a new root transform.
- Production HC2 MoveIt config authority is the sibling `../kuka_ws/src/kuka_robot_descriptions/kuka_kr_moveit_config` package included by `production_runtime.launch.py` — NOT the mirrored `hive_3d_model/.../kuka_kr_moveit_config` copy in this repo. SRDF/launch edits meant to affect runtime target the active `kuka_ws` package first.
- For the active KR10 HC2 stack, the sibling `../kuka_ws/.../kr10_r1100_2_joint_limits.yaml` MoveIt override stays aligned with the active `vivo_kr10_r1100_2` URDF joint `<limit>` bounds. The YAML is the effective planner/IK authority; mismatching from the `vivo_` xacro leaves MoveIt and the broader robot model disagreeing about valid joint ranges.
- KUKA cold-start viewer/bootstrap joint angles use the external `kuka_ws` `ros2_control` joint `initial_value` for `vivo_kr10_r1100_2` as startup authority. NOT the MoveIt SRDF `home` named state (planning named pose, currently disagrees on `joint_5`). Current HIVE-D seed uses `joint_5=0.1` rad to avoid wrist singularity; mirror the full seed from `kr_agilus_ros2_control_macro.xacro` into viewer bootstrap defaults and any Tesseract startup IK seed constants.
- Non-hardware KUKA startup: the mock `ros2_control` plugin seeds arm joint state from xacro `initial_value`s and only overwrites when upstream position command interfaces become non-NaN. Six-zero startup is an upstream controller/command-state issue that later reaches `/joint_states` — NOT a browser bootstrap fallback issue, NOT missing xacro seed values.
- RSI-only sim mode: the KUKA startup seed must be applied in two places — the ros2_control xacro must expose joint `state_interface` `initial_value` in the parser-supported nested `<param name="initial_value">...` form; the RSI hardware interface must initialize both `hw_states_` and `hw_commands_` from those parsed values during `on_init()`. Otherwise `joint_state_broadcaster` can publish six zeros while hardware remains `INACTIVE`.
- For HC2 simulation, keep the KUKA arm on the RSI-only stack. Do NOT pull the extended hardware-only KUKA activation path into `real_sim` just to fix startup pose behavior.
- The active runtime exposes ONE canonical unoffset `vivo_` robot description on the shared `/robot_description` path. MoveIt may keep its internal `robot_description` parameter but must not republish a competing shared `/robot_description` topic. RViz ghost/root placement comes from the standard `DisplayTrajectory.trajectory_start.multi_dof_joint_state` root transform derived from TF.

---

## 18. HIVE 3D Viewer Rendering Policy

- The operator-console 3D pane reuses the EXACT `hive_3d_model/viewer_db` renderer/environment setup, not an approximate variant: same ring-light rig, same profile-driven background/lighting values, same HDRI (`wooden_studio_15_4k.hdr`), same post-processing / tone-mapping pipeline defaults.
- The operator-console 3D pane always renders `ObjectMeshModel.PART_OPTICAL_TABLE` at the global/root frame with identity local transform, even if the runtime manifest does not emit it explicitly.
- Runtime viewer background is mode-gated from operation mode: `INACTIVE` uses a 50% desaturated version of the active profile clear color, `MANUAL_OP` / `SEMI_AUTO` / `FULL_AUTO` use the clear color directly, and `FAULT` uses the existing profile-matched crimson. Fault transitions must route through a neutral low-chroma midpoint, not direct RGB/HSL blue-to-red interpolation that visibly passes through violet.
- Runtime viewer material defaults mirror `viewer_db/config/material_overrides.yaml` exactly for shared object classes: robot arm + tools = violet (`#c48ebb`, roughness `0.37`, metalness `0.58`, clearcoat `0.18`); trays = lighter blue (`#6b7ec2`); containers = navy blue (`#3a4f8a`); disks = orange (`#c27a3a`); hotel / stationary decks / sliding-deck carriage = beige (`#c4b8a8`) with viewer_db default roughness/metalness/clearcoat unless the original override specified otherwise.
- HDR-capable WebGL post-processing keeps its custom `EffectComposer` render target in `HalfFloatType`. A custom MSAA target left at default `UnsignedByteType` collapses HDR headroom before final output.
- Viewer post-FX sizing has ONE native output authority: `EffectComposer` owns device-pixel output size; `SelectedOutlinePass` may apply an explicit outline-only supersample scale on its internal mask/outline/SMAA buffers. Outline width is authored in CSS/screen pixels and converted explicitly from `pixelRatio * supersampleScale`, NOT derived from accidental CSS-vs-device-pixel resize mismatches.
- Three.js WebGPU integration resolves `WebGPURenderer` from the package export `three/webgpu`, not the old `examples/jsm/renderers/webgpu/WebGPURenderer.js` path. WebGPU stays explicit opt-in until the viewer's custom material/tone-mapping pipeline is ported off WebGL-specific hooks (`ShaderChunk` patching, `onBeforeCompile`, the current post-processing stack).
- The browser viewer uses an invalidated render loop, NOT a perpetual idle `requestAnimationFrame` loop. Redraw only on scene/state/camera/profile changes; keep RAF alive continuously only while `OrbitControls.update()` is still changing the camera, while async scene/mesh work is in flight, or while new hot-state keeps arriving.
- Viewer part selection persists only on a true click. Orbit/pan drag release must NOT change selection; gate persisted selection by a small screen-space press/release slop, not raw `mouseup` position alone.
- Viewer hover/outline ignores the optical table and treats multi-mesh assemblies as one target where UX requires: sliding deck carriage + frame as one deck target; hotel frame + tilting rack as one hotel target.
- Hover outline AA stays outline-local: generate outline-only RGBA, anti-alias that buffer, composite back over the untouched scene — do NOT run whole-frame AA just for the hover line.
- Viewer hover outlines built from the hovered entity's visible solid screen projection, NOT visible-surface self-occlusion contours. The mask pass renders non-target geometry black and the hovered entity white under normal depth so same-part cavities and overlapping interior surfaces collapse into one solid white region while true through-holes remain black and still outline.
- In the runtime frontend viewer, STL geometry reuse is owned by ONE `ViewerEngine`-level cache shared by scene build and hot visual swaps/overlays. `ObjectMeshModel` stays the semantic authority; geometry byte reuse and disposal are cache-owned. Mesh/object teardown must NOT directly dispose shared cached geometry.
- Async viewer mesh swaps and manifest rebuilds are latest-request-wins by explicit browser-side request/version guards. Any superseded or detached Three.js geometry/material resources must be disposed immediately, not relied on object-identity races.
- Viewer bootstrap ordering belongs inside the imperative viewer layer, NOT `App.tsx` or the React store. `init.viewer_state` may carry current frame/joint/visual bootstrap state, but the browser stages that data until the manifest-backed scene is installed; never apply frame patches against an unbuilt `FrameGraph`.
- In the frontend viewer, debug-overlay chrome stays inside the viewer bounds. Long debug content scrolls inside the overlay itself; overlay height follows the viewer container height rather than duplicating bottom-panel resize state in React props.

---

## 19. Mesh Authority & Loading

- `ObjectMeshModel` is the semantic geometry authority for UI/runtime rendering. STL relative paths are transport artifacts and are NOT guaranteed to uniquely identify a semantic mesh variant — manifest/projection code carries the chosen `ObjectMeshModel` directly rather than reverse-inferring from a path string.
- Installed HIVE mesh assets mirror the canonical runtime mesh catalog in `hive_execution_runtime.mesh_catalog.MESH_FILENAME_MAP`: preserve `consumables/`, `parts/`, `tools/` subpaths under `share/hive_description/mesh_files`. Do NOT treat legacy `old_tools/` meshes as installed runtime authority.
- Runtime Python code resolves installed HIVE mesh files from `get_package_share_directory("hive_description") / "mesh_files"`. Do NOT derive that path relative to `hive_execution_runtime`; those packages install into separate trees.
- Runtime HIVE mesh loads (including Disk Paddle) resolve through `ObjectMeshModel` + `MESH_FILENAME_MAP` into `ui_manifest.mesh_catalog` and the bridge `/assets/meshes` route backed by `hive_description/mesh_files`. Do NOT load operator-console meshes directly from `hive_3d_model`.
- `TipDefinition.object_mesh_model` is the canonical semantic tip-geometry authority. Loaded pipette tip visuals derive from `LivePipetteChannel.tip_definition_id` through `InventoryInfoRegistry.tips`; tip-rack tip type derives through `TipRackInfo.tip_info`. Do NOT copy tip mesh/type onto mutable `TipRackRow` state.
- Per-channel pipette channel meshes and loaded-tip overlays are viewer-only manifest entities rooted at the channel live Z frame (`tip_mount_cs`), NOT SceneBridge startup collision objects.
- Calibrated loaded-tip browser visuals derive overlay pose from `tip_end_cs` expressed in the manifest-rooted channel frame (`tip_mount_cs`). Do NOT make the browser depend on `tip_end_cs` itself being present in the session manifest — tip frames are created dynamically after tip load/eject while manifest rebuild remains topology-only.

---

## 20. Spatial / Quaternion Interop

- Non-orthogonal affine frames in `src/state/spatial` are intentional and stay first-class. They model small real-world skew/scale deviations (e.g. microfluidic wafer chiplet geometry) that materially affect microscopy and precision pipetting at HIVE tolerances.
- Quaternion support is an interoperability surface primarily for HC2/KUKA robotics workflows (e.g. TCP reference orientation exchange), NOT a replacement for the existing basis-matrix frame graph.
- No nested quaternion hierarchy storage as a new canonical spatial representation. Existing parented `LiveCoordinateSystem` affine transforms remain the source of truth.
- API direction: `as_quaternion(root_frame, target_frame)` returns relative orientation between two frames; `from_quaternion(...)` converts quaternion inputs back into coordinate-system-oriented output relative to an optional root context.
- If no parent frame is provided for quaternion import, the output supports a parent-free frame interpretation at origin `(0,0,0)` with axes oriented from the provided quaternion.
- `QuaternionPose` (translation + quaternion) is the desired in-memory robotics interop type.
- SciPy-based conversion is acceptable if efficient and not materially higher overhead than NumPy-only.
- Frames participating in quaternion conversion are expected to be right-handed; tests validate this for quaternion paths.
- Quaternion conversion APIs are on `LiveCoordinateSystem` (NOT `LiveCoordinate`).
- Converting a non-orthonormal frame basis to quaternion raises an error — no silent projection/approximation.
- Focus on `QuaternionPose` as the interop type rather than introducing a separate orientation-only dataclass.
- `MoveArm` scheduler bake outputs include quaternion waypoint DTOs re-expressed in an arm-specific controller pose-root frame (NOT the arm mounting frame), while still carrying existing approach-path DTOs.
- Arm controller pose-root CS propagates Definition → Instance → LiveArm. DB/model default may be nullable during rollout, but hydration/runtime must fail fast if any hydrated arm cannot resolve a pose-root frame.
- Scheduler bake-time access to pose-root/frame graph routes through static session context (NOT direct `HiveLiveState` access in action bake methods); quaternion conversion logic in `MoveArm.bake_command_params` stays factored into focused helpers.

---

## 21. HIVE IO Mapping & Seed Conventions

- Nominal PLC IO-to-function mappings for HIVE hardware (carousel clamps, fridge drawers, sensors) are modeled with explicit typed conventions in HIVE definitions, NOT ad-hoc string-only dictionaries.
- Canonical source-of-truth in the definition layer for wiring/function semantics. Avoid duplicating nominal mappings across instance-level runtime blobs.
- Existing reference IO data is a UUID-keyed component catalog (`references/Temp_IOComponents_with_uuids.json`) with per-component fields (`port`, `io_data_type`, `conversion`); UUID identity is preserved when ingesting/normalizing mappings.
- Links between physical objects and semantic intent are explicit ID-based bindings, NOT inferred from free-text names.
- Define explicit enum-backed mapping names/functions (no name-string matching); validate routing payloads with Pydantic models.
- Maintain both definition-level routing (definition IDs as targets) and instance-level routing (same schema projected to instance IDs) so runtime services can resolve IO by concrete instance objects without definition-ID indirection.
- Production `src/data_model/seed/hive_seed/ref/HIVE-D_PLC_mapping.json` catalog must carry KUKA arm head-valve rows `tool holder actuate`, `tray gripper actuate`, `disk gripper actuate`. `HiveDefinitionService.configure_io_routing_from_uuid_catalog_json(...)` maps robot-arm IO by those canonical component names.
- For selector-style boolean outputs, semantic action state and raw wire polarity are SEPARATE concerns. `HiveIoFunction` identifies the signal; routing metadata decides whether semantic active state is raw high or raw low. For `SLIDING_DECK_DIRECTION`, semantic active state is OPEN; when routing omits an explicit polarity, the baked default is `ACTIVE_LOW` so OPEN resolves to raw false and CLOSED to raw true.
- Suction cup lid pickup/dropoff stays one scheduler IO action. Pickup turns suction on and waits for the suction digital/ready sensor true. Dropoff turns suction off while pulsing the suction drop-off output true for 0.5 s, waits for the suction digital sensor to turn false after about 0.1 s as confirmation, then resets the drop-off output false. The PLC leaf must remain responsive during this pulse; do not model it as a second scheduler action or a blocking `sleep()`.
- Services under `src/data_model/services/hive/` stay context/permission-agnostic for now: prefer direct `session.get(...)` / explicit SQLAlchemy queries over inherited `ScopedCRUDService` accessors that trigger `RequestContext` auth checks. Proper HIVE permission wiring comes in a later dedicated pass aligned with the v2 StudyDesign approach.
- Production HIVE seed configs support two approach-path authoring shapes: legacy top-level `approach_paths` and inline position-level `approach_paths` on explicit tray/tool positions. The production seeder materializes both into definition-time `ApproachPath` rows; hardcoded demo path insertion is NOT the seed authority.
- HIVE-D inline approach paths may use short tool selectors (`Pipette`, `DiskGripper`, `SuctionCup`, `Probe`, tray-gripper family names) instead of the exact seeded `ToolDefinition.name`. The production seeder resolves selectors against the actual seeded tool definitions and raises only on true ambiguity or a real missing tool.
- Sliding-deck tray positions in HIVE seed YAML use explicit `origin` when provided. `z_offset_mm` is the legacy fallback only and must not override authored XY offsets in explicit-position configs.
- HIVE position-orientation convention: approach waypoint frames are parented to the owning position frame; identity waypoint bases mean "align with the position frame." Seeded HIVE configs orient each position CS so X+ points robot-forward at that position. If desired arm-facing differs from the physical resting pose of the tray/disk at that position, encode the difference in the waypoint basis (or other tool/object-specific frames such as pickup centerpoints) — NOT by counter-rotating the labware base CS against the position CS.

---

## 22. Runtime / Hydration Lifecycle

- The production `load_schedule(schedule_id)` path defaults to the database-backed runtime schedule provider. File-backed JSON/pickle schedule providers are explicit dev-only overrides and must NOT be reintroduced as production launch-contract arguments.
- Runtime schedule records are DB-ephemeral. Prebuilt schedule generators must resolve the current HIVE, arm, tools, drawers, tray positions, and labware from names or HIVE associations at generation time, then persist the resolved current IDs. Do not hardcode concrete hardware UUIDs into generator scripts.
- Loading a runtime schedule must validate the persisted action hardware/inventory IDs against the currently hydrated `HiveLiveState` before installing the study lane. A stale schedule row from a previous DB/HIVE seed should fail clearly instead of crashing later during scheduler projection.
- Schedule actions marked "with last" are concurrent siblings of the prior action. The next serial action should wait for the whole concurrent group to finish, not just the immediately previous row, especially before actions that change object parenting such as tray-gripper open/close.
- In the runtime node's provider-selection config, `runtime_schedule_root_dir` is ignored for the DB-backed provider and required for the file-backed JSON/pickle providers.
- `ExecutionManagerNode` can hydrate a new `HiveInstance` into the same long-lived process while the shell is `UNINITIALIZED` or `INACTIVE`, but that does NOT relaunch the separately launched KUKA/MoveIt stack. Any robot-root placement that stays launch/xacro-owned must therefore either pin the process to one `hive_instance_id` or force a full relaunch before switching HIVEs.
- Chute-lane geometry is also instance-level runtime state: `ChuteLaneInstance.coordinate_system_id` and `dropoff_coordinate_system_id` are the authoritative live mounting/dropoff frames for chute operations. Hydration/scheduler snapshot code carries them through `LiveChuteLane`.
- Within one hydrated runtime session, TF hierarchy loading, startup collision-object loading, mounted-tool attachment replay, and the SceneBridge scene-version counter are session/projection state, not active-connectivity state. `MANUAL_OP -> INACTIVE -> MANUAL_OP` must reset leaf/service/telemetry connectivity and Tesseract sync bookkeeping, but must not tell the runtime to reload objects that SceneBridge still owns.
- HC2 runtime action-to-action dispatch must stay immediate and event-driven from leaf completion callbacks. Future progress deadlines are only for time-based wakes such as study temporal releases and coordinator request timeouts; they must not introduce polling latency into normal dispatch.
- Execution-manager dispatch scans must reuse one scheduler-facing `PartStateSnapshot` across all candidate requirement checks in a single scan. Do not rebuild the same snapshot once per candidate action.

---

## 23. Pipette / Dispensing Mechanics (Physical Model)

- The robot arm holds the pipette at a fixed position above the stage when dispensing/aspirating from Tissue Disks. The motion stage moves tissue disks under the pipette while the arm holds still; pipette channels actuate independently in Z to enter/exit ports on chiplets. The microscope images from below through the bottom of the Disk. Pipette channel Z motion offloads repetitive Z motion from the arm to the channels.
- For dispensing, the arm moves to one of N per-channel positions (one per pipette channel), each aligning that channel's tip axis with the microscope's optical axis. During multi-channel dispensing, the arm goes to whichever channel position the operator wants to observe live through the microscope.
- Dispensing position target = 120 mm above the microscope's `optical_axis_coordinate_system` (convention: `DISPENSING_TARGET_HEIGHT_MM` in `pipette_mapper.py`). Each channel gets its own `dispensing_cs` copy at that nominal pose so calibration can adjust channels independently; the arm backsolves per-channel at runtime via `transient_move_live_pose_to_tcp_target`.
- `LivePipette.well_interval(target_container_well_pitch_mm)` is a pitch-only helper based on evenly spaced pipette channels. Returns the touched-well step size (`1` = adjacent wells, `2` = every other well). For multi-channel pipettes it compares nominal pipette pitch against an integer multiple of the target well pitch and returns `None` if the full first-to-last channel span error exceeds `0.25 mm`.
- Channel tip mounting positions are calibrated by the vision cell (NOT nominal channel spacing from the definition).
- `CalibrateTips` is single-tip by definition: one action/request calibrates exactly one channel tip currently placed in the vision cell. Calibrating additional tips requires additional arm moves plus additional `CalibrateTips` actions.
- The probe-search leaf does NOT stay subscribed to TF when no probe search is running. The `probe_search` action server stays available as a retained leaf, but TF ingestion and historical pose buffering scope to the active probe-search window so idle runtime CPU is not burned.

---

## 24. StudyDesign Tests

- StudyDesign integration tests use test-local copied seed modules under `test/integration_tests/studydesign/` rather than calling production `src/data_model/seed` loaders directly, so test seeding stays decoupled from evolving production seed data.
- StudyDesign integration-test `conftest` provides a real auth `RequestContext` fixture (`ctx`) plus seeded-context fixture data per test, with root org established via `data_model.seed.system_entities.ensure_root_organization`.
- Add an integration coverage path that seeds the StudyDesign, materializes Groups, computes Effective Params, compiles one Group timeline, and persists both cache rows (`GroupEffectiveParamsCache`, `GroupTimelineCache`) via a module-scoped cached fixture pattern (mirroring cached-hive integration style).

---

## 25. Standalone Heuristics

- **`command_arm_pose` contract**: one runtime-owned operation owns mode checks, arm selection, low-level proxying, IK/collision via MoveIt (reporting only, never execution), and semantic clear-on-success. The public request is a semantic `live_pose_cs` target. IK failure → do not change raw mock-arm state. IK success → update the raw mock arm directly even when the solved state is colliding; the authoritative result then comes back through the normal `/joint_states` / `arm_runtime_state` / TF path so runtime live state, TF projection, MoveIt, and the browser all converge from the same measured result. Never orchestrate this as multiple side effects from the browser/bridge (e.g. low-level motion first, then a second runtime semantic cleanup call), and never surface it as a UI-specific path with an optimistic pose write.
- **Arm debug collision reporting**: Datacore-projected world/attached objects report their MoveIt object IDs (UUID-backed in normal HC2 projection). Robot-side collisions must report URDF link names because robot links do not have Datacore UUIDs.
- **Manual Arm Move debug UI**: origin fields are world-space draft values; basis rows are read-only values derived from one canonical quaternion. Arrow-key jogging modifies the selected world origin component or rotates the whole basis about the selected local axis; raw component edits must not drift the basis out of an orthonormal frame.
- **Manual Arm Move current-orientation fill**: the draft quaternion is authored in `manifest.root_world_frame`, same as the origin. Populate it from the selected arm `live_pose_cs` orientation expressed in that root frame, not from the raw Three.js scene-world orientation.
- **Browser MoveIt debug overlay shows world + robot collision**: world-object boxes alone are not enough to explain `base_link` / `link_3` / `link_4` start-state collisions.
- **Browser MoveIt robot-link boxes are posed from MoveIt `robot_state.joint_state`** (not the normal live-view joint stream). Robot-link filters are separate from MoveIt object-ID filters. World/attached boxes come straight from `get_planning_scene`.
- **Bridge MoveIt `attached_collision_objects` honor declared `frame_id`** — the overlay must not treat every planning-scene pose as world-relative.
- **Tesseract planning reference scene**: use SceneBridge's flat Tesseract projection as the primary reference scene for sanity checks. MoveIt's debug planning-scene payload can still be used as a comparison because SceneBridge also feeds MoveIt/RViz, but it is no longer Tesseract's production scene source.
- **Planner frames vs TF frames**: live ROS TF projection mostly uses Datacore UUID frame IDs. `base_link` and `ati_tcp` are RobotModel/planner frames, so `tf2_echo base_link ati_tcp` is not the right validation path for MoveArm TCP interpretation. Validate TCP/tool geometry through the baked `MoveitRequest`, MoveIt planning scene/debug payload, and Tesseract `base_link`→`ati_tcp` RobotModel kinematics.
- **Tesseract planning projection direction**: Tesseract should not mirror MoveIt through `/monitored_planning_scene` or block runtime `MoveArm` dispatch on drain/resync services. SceneBridge owns HIVE scene semantics and serves a flat, demand-driven Tesseract planning projection: current world-resident object poses, current arm-carried object poses, cached/versioned geometry only when needed, and active collision allowances. Tesseract pulls this projection before planning; optional prewarm should trigger the same pull/apply path, not introduce a second push data channel.
- **SceneBridge projection pose source**: Tesseract/MoveIt projection poses should be composed from SceneBridge's latest ingested `/tf` + `/tf_static` local-frame cache. SceneBridgeNode should not own/feed a `tf2_ros.Buffer` unless production pose resolution moves back to TF2 APIs. Do not ask TF2 for a latest-common-time lookup during projection assembly; skewed dynamic branches can fail even though a current flat snapshot is available.
- **Tesseract v3 IFOPT tuning direction**: collision-cost evaluation is the primary suspected runtime bottleneck. Target a low hard collision floor with a wider soft clearance band (e.g. about 2.5 mm hard, 30 mm soft), keep hard collision as the safety authority, make the soft clearance pressure cheaper where possible, and remove/disable the velocity smoothness cost because it does not add useful path quality here.
- **Tesseract seed-planning direction**: with the faster IFOPT setup, OMPL should be treated as an expensive topology/fallback tool rather than mandatory work for every MoveArm. Prefer direct/cheap joint-space seeds plus IFOPT and final validation first; run OMPL only for segments or fallback cases where the cheap seed cannot refine into a validated path.
- **Arm-motion diagnostics shape**: smallest useful shape — arm capture explicitly, carry Datacore waypoint provenance through the existing `MoveitRequest` only when armed, keep one last retained capture in the MoveIt leaf, fetch it on demand through debug services / HTTP routes. Do not stream through websocket/bootstrap state, do not rely on normal logs, do not introduce multi-capture retention unless the simple shape proves insufficient.
- **D2 architecture diagrams**: for HC2 runtime/UI cross-node architecture documentation, prefer text-editable D2 sources under `codex_logs/docs/diagrams/`, with each process node capturing its key cached state / params and each edge labeled with transport type, payload shape, and cadence or trigger semantics.
