# Mistakes To Avoid

## Rule

On every user-corrected mistake, add one concise entry as a LESSON LEARNED that avoids this type of mistake in the future. Write lessons so they give useful, specific, actionable guidance that will avoid mistakes of that class in the future. You MUST NOT add lessons that are written as "one-offs" that won't be relevant again; this is not a mistake log. The purpose of this document is to generalize mistakes with reasonably specific scope so they are not repeated again by future agents.

---

## 1. Frame / FrameStore math rules

- `FrameStore` implements `__len__`, so an empty-but-valid store is falsy. Use `if fs is not None:` and explicit `x if x is not None else default` selection — never `if fs:` or `fs or default`.
- `A.as_quat(B)` walks the hierarchy from A to B and the transform depends on any moving frame on that path. A "static" transform from A to B is only static if no moving link sits between them. For arm backsolvers the TCP frame must be the terminal frame of the rigid chain (`live_pose_cs`), never the static parent above it (`dc_kuka_convention_cs`). Tests that leave the arm at its home pose will not catch this because the moving link is then identity — at least one test per backsolver claim must physically displace the arm (non-identity local transform on the moving frame).
- Parent affines are NOT applied to a child's stored local origin/basis. Local `origin`/basis values stay expressed in the parent frame; the parent affine matters only when re-expressing that child into a farther ancestor.
- Prefer `waypoint_frame.as_quat_dto(pose_root_frame)` directly — do not first call `expressed_in(pose_root_frame)` then `as_quat_dto(...)`.
- 2D affine registration flattening: params `[a, b, c, d, tx, ty]` with transform matrix `[[a, c], [b, d]]`. Least-squares seed from `X @ M = Y` must flatten as `[M00, M01, M10, M11, tx, ty]`; any swap of the four linear entries yields a "nearly right" fit that silently distorts calibration frames.
- Vision-guided calibration capture must freeze BOTH the source image and the relevant frame transform at the same moment. Solving from a live "latest image" buffer or transforming offsets through live geometry after motion continues silently captures the wrong marker frame.
- `InitVar` parameters in dataclass `__post_init__` must be read from the local parameter name, not `self.<name>` — dataclasses do not bind `InitVar` values to `self`.
- Never give a dataclass field / `InitVar` the same name as a method. The generated constructor can see the method object as the field default, so omitted booleans like `register` become truthy and silently change behavior. Inspect the generated signature when constructor defaults look suspicious.

## 2. Arm / tool frame triad

Three frames cover arm/tool mating and must never be conflated:

- `ati_tcp` — native ROS/MoveIt TCP and physical coupling point.
- `tool_interface` — wrist/toolchanger collision seam (the robot link that owns attached collision bodies).
- `live_pose_cs` — Datacore semantic moving frame for the same physical seam. May be position-coincident with `ati_tcp` but use a different basis, so measured native `ati_tcp` poses must go through the seeded rectifier chain before mutating `live_pose_cs`.

On a mounted tool, `tool_mount_cs` is the direct mating interface and should snap coincident with the arm coupling seam on latch. There is no "authored native vs authored semantic tool-root" branch — do not invent one.

Before changing attachment, IK, or pose math, confirm which of these roles each frame plays on the current code path. Reconcile older `command_arm_pose` / raw-mock-arm / traversability plan notes against current source — ownership changed over time and stale notes still describe the removed paths.

## 3. Hydration end-to-end propagation

Any change touching DB definitions, DB instances, live-state hydration, or HC2 live state must propagate through the full chain:

DB definition → DB instance → `services/hydration` mapper → `Live*` dataclass → `FrameStore` / `StateManager` → bridge/runtime consumers → UI manifest.

When adding a field: (a) copy it into the instance, (b) propagate into the live info class, (c) include it in `collect_cs_ids()` if it is a frame ID, (d) rebind sub-objects to the final `FrameStore`, (e) consume it in startup collision/UI-manifest builders. When tightening frontend/runtime payload types, update immediate consumer signatures in the same pass. When removing or renaming, grep the whole chain before testing.

## 4. Async SQLAlchemy & lazy-loading test hygiene

- Relationships default to `lazy="raise"` — never downgrade to `selectin` at the mapper level. Add `selectinload(...)` only on the specific query that needs it.
- After creating linked rows in the same session, either `refresh(parent, [relationship])` or `expire_all()` (capture IDs first) before a `selectinload`-based service call. `selectinload` skips relationships already marked loaded on identity-mapped objects.
- In tests, extract IDs before any call that may rollback and use the Savepoint Pattern B from CLAUDE.md.
- FastAPI DB dependencies must be plain async-generator functions. Do not pass an `@asynccontextmanager` helper such as `data_model.model.engine.get_db` to `Depends(...)`; keep HTTP request sessions in `api.dependencies` and override that dependency in route tests.
- When an API transport schema wraps a stricter service DTO, mirror the service DTO's input constraints at the HTTP schema. Otherwise bad client input can bypass FastAPI validation and surface as an internal Pydantic error while constructing the service DTO.
- `BaseCRUDService.list()` returns a `PaginationResult` — access items via `.items`, never the result directly.
- For JTI models, override the base service's bulk `update()` with an ORM get → modify → commit pattern.
- Service tests that exercise `Service.get()` / scoped CRUD must use an authorized org in the installed `RequestContext` (`test_sandbox_org` or an explicitly scoped ctx org).
- Context-enforced service methods can break integration/seed helpers that run without a `RequestContext`. For those paths, load entities via the session directly (or install an explicit scope) while keeping domain validation intact.
- In long-lived ROS/runtime processes, do not call `asyncio.run(...)` repeatedly around `data_model.model.engine.async_session_factory`. SQLAlchemy asyncpg pools are event-loop-bound; keep DB coroutines on one owned event loop or explicitly dispose the engine before closing the loop.

## 5. DB-backed pytest execution

- Never run multiple DB-backed pytest invocations in parallel against the same test DB — truncate/reset fixtures deadlock. Wait for one to finish before starting the next, even for small targeted slices.
- A deadlock during DB-backed pytest usually means another user/agent is running tests. Wait 60 s and rerun before changing code or test setup.
- Never use the parametrize id `request` — pytest reserves it as a built-in fixture name and collection fails before tests run.

## 6. Fail-fast at runtime / hardware boundaries

- Reject unknown message kinds, unknown frame/entity IDs, missing mesh-catalog entries, unknown URDF package URIs, out-of-range numeric motion targets, and unknown dataclass fields at runtime contracts — bridge payloads, manifest specs, motion/control values, hardware-facing action fields. Never warn-and-skip or pass values through unchanged.
- Never use optional / fallback inputs on motion or hardware-facing code. If the real value is missing, raise at the source.
- For robot motion start states, missing manipulator joints are dispatch-level errors, not cache misses. Do not fill missing joints with zero or continue into OMPL/IFOPT with a partial joint map.
- Never fabricate a terminal motion destination from an intermediate waypoint to satisfy a transport field. Never fabricate `start_pose` / `expected-state` inputs from nearby authored waypoints — leave validation disabled if the caller did not provide the value.
- For MoveIt / collision / debug paths, never fall back to non-MoveIt pose or geometry data just to keep overlays visible. If the MoveIt payload is missing, show nothing or raise a producer error.
- Before flagging a defensive-validation issue downstream, trace the value back through real producers and fix at the real source instead of adding bandaids. When only one upstream source is meant to be lossy, do not catch broad validation exceptions at a downstream consumer boundary — fail fast in the consumer; reserve skip-with-warning for the producer-side conversion step.
- Nullable contract fields must use explicit `=== null` / `!== null` checks, not truthiness. An empty-string producer sneaking through a truthy check silently falls back instead of surfacing the contract violation.

## 7. Viewer geometry vs collision geometry

- `CollisionSceneStartupBundle` / `SceneBridge` world specs are for *collision* geometry only. UI-only meshes (pipette channel visuals, tip overlays, drawer flaps, stage clamps, tool overlays) go through `ui_manifest.mesh_catalog` → `/assets/meshes` and render frame-driven under the live parent frame. (Runtime mesh path: `ObjectMeshModel` → `MESH_FILENAME_MAP` → manifest → `/assets/meshes`; `viewer_db` is not the runtime authority.)
- Do not invent synthetic joints when the existing live frame hierarchy already carries the moving subtree — reserve synthetic joints for assets that are explicitly articulated around a local pivot.
- Do not treat cached "open" / live reference frames as alternate geometry assets. For sliding drawers and the sliding deck, the same mesh/entity should move with the live source frame; debug the frame-update path, not a nonexistent "open geometry" swap.
- Do not project browser visuals through runtime-created frames that were never installed in the manifest topology. If a late-created frame (e.g. `tip_end_cs`) only affects a viewer-only mesh under an already-manifested parent, carry that effect as local visual state relative to the installed parent.
- `ObjectMeshModel` + `MESH_FILENAME_MAP` is the single canonical mesh authority for both HC2 runtime geometry and UI viewer meshes. Do not create a parallel mesh registry. The mapping is intentionally not 1:1 — multiple semantic mesh models may share one STL; carry `ObjectMeshModel` forward and never reverse the map from STL paths.
- When comparing MoveIt and Tesseract collision on concave tools or labware, check whether the Tesseract mirror replaced an exact mesh with one convex hull. A single hull can fill open gripper jaws or tray pockets and create invisible false collisions that MoveIt correctly does not report.

## 7a. Frontend overlay controls

- Do not hide form inputs in dev-overlay segmented controls with unanchored `position: absolute`. Focus/change can scroll the overlay to the invisible input and make the menu appear blank. Use visible buttons with `role="radio"` or anchor hidden inputs inside a positioned label with zero size.

## 8. TF / projection convergence (SceneBridge, multi-frame roots)

- When a root depends on multiple dynamic frames (e.g. `live_cs_y` + `live_cs_x` for a stage), treat the full sync surface as one coherent stamped update. Do not flush MOVE updates from a half-applied multi-frame change, and do not accept "any sync frame ≥ S" as convergence — track a per-root coherent stamp advanced only when the whole surface has caught up to one pending required stamp.
- Event-driven SceneBridge is fine, provided `/joint_states` / `robot_state_publisher` freshness stays live, every moving world object emits bounded-latency pose updates on real TF changes, and a dirty-state retry trigger is preserved for residual work after transient TF-lookup or MoveIt-apply failures. Do not delete TF replay or SceneBridge retry timers just because the broader goal is "remove topic heartbeats."
- `SceneBridgeNode` uses direct `/tf` and `/tf_static` subscriptions plus `_latest_local_by_child_frame_id` as its pose authority. Do not reintroduce a `tf2_ros.Buffer` there unless the runtime starts using TF2 APIs for actual pose resolution again.
- Service-owned moving roots (e.g. `LiveFridgeDrawer.drawer_live_cs`) must still be marked dynamic in serialized frame hierarchies and republished on `/tf` when `update_frame_pose` changes them.
- When Datacore already projects a runtime frame into ROS TF, do not reach first for YAML re-reads, duplicate live-state hydration, or one-off MoveIt placement services. First check whether the existing TF projection can own the literal ROS frame name the downstream stack expects.

## 9. Execution-lane & cursor invariants

- HC2 has two action owners: `runtime_session.schedule` (study schedule) and the manual operator lane. Resolve events, feedback, rejection, and status through the owning execution lane via `runtime_session.resolve_action_context(...)` — never hard-wire to one.
- For operational scripts that import scheduler action classes inside the main path, compile/help checks are not enough. Run the real script path after scheduler API changes, because stale dynamic imports and renamed action fields only fail once that branch executes.
- When the user names a specific prebuilt schedule module, do not substitute a generic example schedule script. Search for the CLI wrapper around that module and wire the operational flow to the named schedule source.
- For premature object-parenting or "action ran while arm was moving" bugs, compare execution-event timestamps and schedule dependencies first. The dispatcher does not apply a global "arm active" interlock; "with last" can release the next row early if the next row depends on the shorter sibling instead of the whole concurrent group.
- Do not change authored prebuilt schedule destinations just because a test fixture or expected sequence disagrees. Treat the schedule generator as the source under investigation, ask/verify intended tray positions, and fix brittle tests separately from runtime safety fixes.
- When removing terminal actions from a live `Schedule`, also rewind `ExecutionRun.next_unreleased_index`. `recompute_next_unreleased_index()` scans forward from the old cursor and can otherwise skip a new head action.
- `prepend_many(...)` only rewires the linked list; rewind the cursor explicitly before the next candidate scan.
- `CompletionBridge` tests must seed `runtime_session.resolve_action_context(...)`, not just `_execution_manager` / `_in_flight`. Feedback/result/rejection/dispatched paths now resolve the owning execution lane through `RuntimeSession`.

## 10. Wait-loops, timers, and throttling

- A polled/rechecked state field is a real wait dependency if the next step cannot proceed until another topic updates it. Count it as a wait even without an explicit blocking `wait_for` call.
- Known multi-second lifecycle operations (e.g. `hydrate_hive` ~10 s) have documented windows — wait them out before declaring fault.
- `ps %CPU` in this runtime is lifetime-averaged since process start, not instantaneous. Use interval sampling (`top -b -n 1`, repeated sampling) for pause/resume or idle-vs-active comparisons.
- In `ros2_control` wrappers, explicitly throttle the configured control loop — a `dt` parameter alone does not cap execution. If the configured branch calls `read/update/write` without `sleep_for` / `sleep_until`, `/joint_states` and downstream consumers flood at CPU-limited rates even when `update_rate` is set correctly in YAML.
- Zero-timeout `wait_for_service()` is not an authoritative readiness check in activation code — a background waiter may already have proved the dependency ready while the spot-check returns false and causes livelock. Use a small positive timeout or treat the completed wait itself as authoritative.
- Never call `rclpy.spin_until_future_complete()` from inside a service callback on the same node/executor — it re-enters an already-spinning executor. Use a multithreaded/reentrant executor (≥2 threads) or move the blocking wait off the executor thread.
- After a controller-side action/result monitor is the real gate, inspect its cadence before blaming downstream telemetry timers. A `FollowJointTrajectory` result can quantize at the controller's internal monitor rate even when the permission gate is no longer a topic.
- `ScenePrep drain_ms=0` only proves the planner did not drain queued scene updates at dispatch start. If a `MoveArm` stays active after the controller logs "Goal reached", compare `fjt_wait` against trajectory duration and inspect scene-apply callbacks during that interval; background scene mirror work can delay the action result future and make the next action look queued.

## 11. ROS / MoveIt / OMPL config verification

- For segmented `MoveArm` failures, diagnose by segment. A `bridge_failed` token after successful departure/arrival Descartes means the endpoint ApproachPaths produced seam states; do not blame drawer insertion/extraction or convex-hull approach-path collisions unless logs show the approach segment itself failed. The bridge may still need an explicit lifted/topological route around a mid-path obstacle.
- Source-path HC2 tests that only check a removed symbol or a static source contract should inspect source text instead of importing ROS-dependent modules. Keep macOS/source tests free of ROS package discovery failures unless the test is explicitly ROS-environment-gated.
- When Descartes approach collision allowance is enabled, Descartes may return a departure/arrival seam state that is still in collision. Do not count those approach-owned seam endpoints as bridge collisions; keep the bridge interior strict, and never retry the same invalid seam pair just to fill a preferred OMPL time window.
- For HIVE-D staged seed planning, do not assume many useful seam-pair or wrist-winding branches exist. The practical constraints often leave one reasonable branch on each end: joint_6 is bounded to `[-270 deg, +90 deg]`, joint_4 cannot spin through 360 degrees, and the elbow branch is restricted from jumping to its second IK solution. Optimize the single bridge solve before adding seam-pool orchestration.
- For first-solution OMPL bridge budgets, do not clamp an overrun pre-IFOPT deadline down to the minimum search floor. If the operator wants "up to N seconds until one finishes," remember `max_solutions=1` already returns early; the timeout is a cap, not time that will always be spent.
- Before proposing a new OMPL planner, grep the *installed* library: `strings /opt/ros/jazzy/lib/*/libompl.so.* | grep -E "^<PlannerName>$"`. Jazzy ships OMPL 1.7.0; planners added upstream after 1.7.0 (e.g. AORRTC) require a source build.
- The `planners:` string inside an OMPL `AnytimePathShortening` config is a comma-separated list of OMPL class names (`RRTConnect`, `BiTRRT`, `BITstar`, `RRTstar`, ...), resolved through APS's hardcoded allocator registry — not the general OMPL planner registry. Unknown names are silently logged as `Unknown planner name: X` and that worker is dropped. After editing the APS planners string, tail `/tmp/hive_execution_runtime.log` for `Unknown planner name:` to confirm every worker was accepted. To add a planner as an APS sub-worker edit only that string; do not add a new top-level planner block or extend `manipulator.planner_configs`.
- When adding a custom pipeline via `MoveItConfigsBuilder.planning_pipelines(...)`, the config filename must be exactly `config/<pipeline_name>_planning.yaml`. A mismatched filename (e.g. `stomp_planning_bridge.yaml` for pipeline `stomp_bridge`) will not be auto-loaded.
- In MoveIt 2 STOMP, a seed passed via `MotionPlanRequest.trajectory_constraints` overrides `config.num_timesteps` with the seed trajectory length. Do not assume YAML `num_timesteps` survives seeded planning, and do not invent unsupported STOMP YAML knobs without checking the current planner source.
- Verify `MoveGroupInterface` / `MoveGroupInterface::Plan` fields against the current MoveIt 2 release. Do not assume MoveIt 1-era members like `trajectory_` or `error_code_` exist.
- Verify external ROS overlay package names from `package.xml` or `index.ros.org`, not from repo names or guessed subpackage names.
- When replacing a custom ROS service with a standard one like `std_srvs/SetBool`, re-check every response-field read on the live success path. `SetBool` returns `success` and `message`; leftover custom fields (`response.some_flag`) can pass source-text audits and still 500 at runtime.
- Before switching HC2 runtime planning to an existing sibling MoveIt/OMPL config entry, verify the live planner can instantiate that config end-to-end; a YAML block existing is not proof it is usable.
- `builtin_interfaces/Duration` carries both `sec` and `nanosec`. A downstream service that reads only `timeout.sec` creates a hard 1.0 s cliff — sub-second behavior must be proven end-to-end or moved to a local floating-timeout seam.
- When building a local `planning_scene::PlanningScene` from a `GetPlanningScene` snapshot, do not assume the response is a full scene. Partial/diff responses must be applied with `usePlanningSceneMsg(...)` (or the diff API), not `setPlanningSceneMsg(...)`.
- For Tesseract scene snapshots, do not treat a missing MoveIt root transform as fatal until checking how the runtime URDF owns `world -> base_link`. In HC2, TF does not publish `world`; the fixed root placement is baked into the robot description, so snapshots that omit the root transform should preserve the URDF-loaded joint origin unless they explicitly provide a replacement.
- Debug toggles that write one boolean should use `std_srvs/SetBool`, not a custom `*.srv`. Reserve custom services for contracts that need more than request/response shape.

## 12. RViz / MoveIt live-state debugging

- The browser arm can be injected with nonzero KUKA startup joints on bootstrap when no retained joint state exists. "Browser arm looks right" is not proof `/joint_states` is healthy — inspect `viewer_bootstrap.py` before accepting the visual as evidence.
- `planning_scene.robot_state.joint_state` is a separate authority from browser `/joint_states` rendering. When MoveIt robot collision boxes and the browser arm disagree, trace both feeds before proposing fixes.
- RViz can subscribe to and republish `/monitored_planning_scene` via the hidden `rviz2_private_*` node, making stale scene snapshots look like a history loop even when live joint telemetry is stable.
- `GetStateValidity` requests must copy current `attached_collision_objects` from the live planning scene. A helper that sends solved joints without them silently drops tool collisions even while MoveIt's live planning scene is correct.
- Frontend "MoveIt boxes" debug overlays that only serialize `world.collision_objects` cannot show `base_link` / `link_3` / `link_4` collisions. For arm-vs-world debugging, render robot collision geometry too or compute robot-link extents from the collision URDF and current TF.
- When MoveIt rejects a plan at `CheckStartStateCollision`, map the reported collision object IDs back to their scene entities and source frames first — then verify whether arm links or world objects are mispositioned/misoriented in the projected frame tree.
- Before diagnosing RViz / MoveIt visualization bugs, read the exact launched `.rviz` file and trace every enabled display's topic/producer — do not assume the active config matches nearby reference configs.
- When a startup-pose singularity affects planning, fix the canonical `ros2_control` startup joint seed AND every mirrored startup fallback. Do not paper over it only inside a planner path; grep `kuka_ws` and `hc2_ws` for the joint constants and keep xacro, viewer bootstrap, planner seeds, and docs aligned.
- When KUKA joint limits change, update every startup guard that asserts the expected robot model, especially Tesseract OPW registration checks. A stale guard can crash `tesseract_planning_node` before `/tesseract/prepare_projection` appears even though the URDF itself is correct.

## 13. Handoff / plan reconciliation

- Treat any handoff/plan/AGENTS note as a hypothesis when older than current source. Re-read the current referenced files and trace the live code path before reporting that the note's finding still persists.
- When a handoff/plan file contains stale blocker notes and later verification contradicts them, do not report the older blocker as current status. Reconcile against the latest implementation-log / checklist entries and any direct user clarification before summarizing where integration stands.
- When the user asks for all remaining scoped plan work, do one full reconciliation pass before answering. Cross-check the plan against current code, separate in-scope from deferred/out-of-scope, and write one canonical remaining-work list — do not drip-feed.
- Before planning new work on `command_arm_pose` / manual-arm features, reconcile older plans against current source and tests (`hc2_ws/docs/data_flow_kuka.md`, `test/hc2_ws/test_arm_pose_override_contract.py`). This seam changed quickly; older notes still describe the removed raw-mock override.
- When validating viewer-feature checklist items, trace the whole path (manifest emission, bridge/update production, frontend registration, runtime application) separately. Split mixed-status items (e.g. hotel implemented, stage deferred) instead of bundling them under one checkbox.

## 14. Subagent / research hygiene

- Give subagents narrow file lists, exact purpose, and exact required output sections — no broad forks with meta/status chatter. Wait for requested bounded reads to finish before answering.
- Re-verify subagent findings against the current workspace with `git rev-parse HEAD`, `git status`, and direct reads in the main workspace. Subagents may inspect a forked or clean snapshot rather than the user's current uncommitted state.
- After an interrupted turn or a subagent "missing file" report, verify with `ls` / `find` before telling the user the file does not exist. Stale fork snapshots and interrupted tool output can be wrong.
- For this private repo, inspect code from the local workspace instead of browsing the public web. Use public browsing only for external dependencies (ROS docs/source, upstream projects).
- When the user asks for files from a specific plan / chat-scoped fix, search the transcript first to extract the plan's actual touch points — do not union every file from a broad git-history theme unless the transcript explicitly ties them to the planned fix.

## 15. Scope discipline on narrow fixes

- When the user asks for a narrow change at one concrete seam, keep the patch at that exact seam. Do not generalize to every sibling target type, invent orchestration for multiple containers / takeovers / node-level singletons, pitch "simpler first-pass" shortcuts when the real target is clear, or add helper layers if one short local block is clearer. Prefer the smallest effective guard / smallest readable implementation.
- Before running a formatter on a large existing test/source file, check whether it will reflow unrelated code. Prefer targeted formatting or a smaller file list so behavior changes do not get buried under format-only churn.
- When designing a planner, do not "simplify" by removing cheap search breadth that directly improves feasibility. If candidate extraction or start/goal expansion is already cheap compared with planner setup, preserve it and keep the implementation crisp around that broader search.
- When the user asks about Tesseract planner behavior, inspect `hc2_ws/src/kuka_tesseract_cpp` directly. Do not infer behavior from the older MoveIt leaf just because both consume `MoveitRequest` and use TOTG.
- Before proposing Tesseract scene-sync changes, inspect the current SceneBridge projection path first. Tesseract no longer consumes `/monitored_planning_scene` or `/get_planning_scene` in production; it pulls `scene_bridge/get_tesseract_projection` and applies that flat snapshot before planning.
- When a runtime request only needs one local UUID existence guard, keep the check in the owning service method or one tiny adjacent helper — do not invent a validator table or validator framework.
- When scripting a ROS/HC2 rebuild/launch flow that already works manually, keep commands as close as possible to the proven interactive sequence. Do not add unrelated setup steps or shell strictness inside container command strings unless the manual flow actually needs them. If the user explicitly says to keep a step, preserve it while refactoring a different part.
- For accidental duplicate-launch protection in one container, prefer the smallest fail-fast launch lock on runtime-owning launch surfaces, bridge exempt. Do not build multi-container orchestration.
- For HC2 wait-loop cleanup, prefer the smallest leaf-local change that uses an existing event or exact simulator deadline. Real-hardware confirmation / probe-contact paths stay direct and low-latency unless broader orchestration is explicitly requested.
- When a narrow viewer-only effect needs a few static facts, inventory the existing browser frame tree and pushed semantic state before widening runtime contracts. Prefer the smallest existing viewer payload over adding manifest specs or manual-ops sidecars.
- When the user asks for documentation, do not add docs-only regression tests unless they explicitly request test coverage or the task includes a real behavior change that needs verification. Same rule for cleanup/docs-only sessions: do not add tests out of habit.
- When a user asks for a narrow fix, do not keep reading an unrelated active plan after the required session-start check. Switch immediately to the requested failure seam unless the active plan is directly relevant.

## 16. Docker / HC2 operational rules

- Export `ROS_DOMAIN_ID=80` before any `ros2 ...` probe in the HC2 Docker runtime — service/action calls can hang otherwise.
- When using `rg` across frontend code, exclude generated/vendor trees like `frontend/node_modules` and build outputs unless the search is explicitly about dependencies. Unbounded searches there can flood the tool output and bury the relevant runtime/frontend files.
- Do not inherit host `.env` `DATABASE_URL` / `TEST_DATABASE_URL` values that point at `127.0.0.1` inside the container (loopback targets the container itself). Use `host.docker.internal` or an explicit DB container hostname.
- Before relaunching HC2 in a reused container, explicitly `pkill` existing HC2 / bridge / KUKA bringup processes — do not assume shell job control or partial restarts cleaned up the ROS graph. Prefer the curated cleanup helper or a fresh container.
- After a long Docker/image rebuild, verify HC2 is actually up: `pgrep production_runtime.launch.py`, `pgrep execution_manager_node`, `pgrep hive_ui_bridge`, `curl 127.0.0.1:8080`. Do not claim success from the rebuild completing.
- Live HC2 status/scene probes should use short bounded waits by default. Do not run multi-minute polls unless there is a specific lifecycle operation with a known long window and you tell the user why.
- Do not store the full `/api/ui/bootstrap-snapshot` response in a shell variable during HC2 status polling. It is multi-megabyte JSON and makes polling brittle; write it to `/tmp` with `curl -o` or stream it directly to a parser.
- When polling HC2 HTTP endpoints from Codex, use direct approved `curl -sS ...` commands or container-local probes. A shell-loop command can run under the sandbox and block localhost connects; also never parse a snapshot file after `curl` failed, because it may be stale from an earlier poll.
- Never print full environment arrays or raw `DATABASE_URL` values when inspecting Docker/container overrides. Inspect only the specific keys you need; redact credential-bearing values in command output.
- Always run Python with `uv run python`, never plain `python`. Prefer `uv` / `uv pip` over raw `pip` unless the user explicitly asks for system `pip`.
- When the user says to let them know before using Docker or another live runtime, treat it as a hard boundary — not a soft preference.
- `pkill tesseract_planning_node` before rebuilding `kuka_tesseract_cpp`, or duplicate stale nodes survive the rebuild.
- Avoid `pkill -f <pattern>` inside a shell command whose text contains that same pattern; it can kill its own build shell. Use an exact executable name that fits `pkill -x`, `pgrep` then targeted `kill`, or the bracket pattern form (`[t]esseract_planning_node`) when `-f` is truly needed.
- Strict-mode shell wrappers that source ROS `setup.bash` / `local_setup.bash` must temporarily disable `set -u` — multiple ROS setup scripts reference vars like `AMENT_TRACE_SETUP_FILES` / `AMENT_PYTHON_EXECUTABLE` that may be unset. Pattern: `set +u; source ...; set -u`.
- Repo shell scripts must work on macOS Bash 3.2 — avoid Bash 4-only expansions like `${var,,}`; use `tr` for portable lowercasing.
- For portable repo shell scripts, put `mktemp` random `X` characters at the end of the template basename, e.g. `/tmp/name.log.XXXXXX`. macOS/BSD `mktemp` does not reliably replace `XXXXXX` when a fixed suffix follows it, and can try to create the literal template path.
- ROS `--symlink-install` makes installed package-share mesh files symlink to source files. Path-escape checks for package assets should reject absolute paths and `..` before joining, but should not require the final resolved STL target to remain under the install-space root.

## 17. Test-double rule

- In tests that exercise builders, projections, or helpers that read real `Live*` / DB model properties, use real model/info objects (or the existing factories) — not `SimpleNamespace` / ad-hoc stubs. Schema contract coverage depends on it.
- Do not weaken explicit domain assertions when upgrading stand-in tests to real objects; use real replacement objects with explicit values.
- Do not use `getattr(obj, 'field', None)` defensive lookups in production code just to accommodate loose unit-test doubles. Type the production helper against the actual class/property contract and update the tests.
- For HC2 runtime-shell tests that flush `UiExecutionSummary` / `ManualOpsContext` / `enter_fault()`, hydrate real `Live*` part/tool objects with every field the publish path reads (`info`, `disk_positions`, `temp_c`, mounted-tool IDs, `dc_kuka_convention_cs_id`, `LiveHotel.info`). Those publish paths now read real properties, so fake namespaces rot as the live dataclasses evolve.
- When HC2 source-path tests use hand-built ROS stubs or minimal hydrated `Live*` fixtures, update those helpers alongside new runtime imports and newly required hydrated fields. Stale `hive_control_interfaces.msg` / `.srv` stub lists and too-thin fixtures will fail far downstream and hide the real regression.
- When an HC2 helper is typed against real `Live*` tool classes, use real live-tool objects in tests — class-gated behavior (e.g. active mesh selection) can fail in tests for fixture reasons that do not match runtime.

## 18. HC2 test collection guards

- `hc2_ws` tests that import installed ROS package modules at module scope must guard collection with a Linux + ROS package-availability check and `pytest.skip(..., allow_module_level=True)`. macOS / default dev environments should skip cleanly, not fail collection on missing `rclpy` or package-install paths.
- Do not turn source-level HC2/bridge tests into ROS-only module skips if the path can be exercised with lightweight monkeypatched ROS interface modules. Keep the fast non-ROS coverage; add real-ROS gating only for tests that genuinely need built/generated runtime packages.
- For bridge/UI interface paths that need coverage in non-Linux dev environments, add a source-based test module with lightweight ROS stubs instead of relying only on Linux-only installed-package tests.
- When adding a new module-level `hive_control_interfaces.srv` import in HC2 runtime or bridge code, grep `test/hc2_ws` for every manually stubbed service-name list and update all of them. Source-path tests mirror those imports in multiple files.
- New source-path `test/hc2_ws` modules that import `hive_ui_bridge` directly must add the same repo `sys.path` bootstrapping the other source-path HC2 tests use, or collection fails even when the source path exists.

---

## Standalone heuristics

These are specific enough to keep but do not fit a themed section above.

### Communication / writing

- When asked whether a debug visualization is accurate, answer the fidelity question first: say whether extents, pose, units, and shape match the source data, and only then mention that it is a simplified visualization.
- When the real design question is "commanded target vs measured terminal state," ask it in those exact words. Do not hide the decision behind abstract phrasing like "authority" or "completion semantics."
- When the user states broader ownership semantics for a feature, distinguish "currently wired callers" from "intended contract" before scoping implementation.
- Define optimization jargon immediately with a concrete example when explaining solver design: plain-language meaning first, small numeric example next, equations only if needed.
- When explaining local-vs-parent frame math, do not say a parent affine is "applied" to a child's stored local coordinates — local values stay in the parent frame; the parent affine only matters later when re-expressing or composing farther up the hierarchy.
- In AGENTS / architecture docs, prioritize behavioral meaning and intended usage over lists/constraints/import structure. Include purpose + placement rationale (why the abstraction exists, what belongs in/out) so future readers can reason about boundaries.
- When rewriting an AGENTS / architecture doc, diff against the original afterward and restore any still-valid operational details that were omitted.

### Debugging / diagnostics

- When a robot/planner ghost is displaced but otherwise shaped correctly, check root-placement authority first (launch/xacro `world -> base_link`, TF root transforms, `DisplayTrajectory` root pose) before inventing new debug surfaces.
- When debugging a live frontend after a rebuild/reload, re-check the currently served bundle hash, current websocket session, and current manifest/frame IDs before trusting older bridge logs. Pre-rebuild lines can describe a stale browser bundle or topology.
- When the user says the runtime/container was just restarted, stop diagnosing from older container logs. Re-check uptime against the event timing; if logs are now stale, switch to source/config inspection or ask for a fresh reproduction.
- When viewer geometry is exactly 180° wrong, first verify whether the STL is authored with the opposite local forward axis before changing semantic-frame logic. If the frame is already correct, flip the mesh-basis consumer instead of rotating the source frame.
- Do not conflate MoveIt stale robot-state issues with stale world-collision-object projection. Joint-state freshness drives MoveIt's current-robot-state validity; SceneBridge only keeps moving world objects aligned.
- Before trusting live arm-motion diagnostics, verify the exact reproduced move executed successfully. Logs from a failed `MOVE_ARM` request (e.g. `TimeOptimalTrajectoryGeneration failed`) are not valid evidence for the executed bounce path.
- When a runtime guard or fallback exists only for diagnosis, do not treat it as intended behavior. Separate temporary instrumentation from the real product path and keep digging for the upstream root cause.
- For Tesseract staged-seed Descartes failures, do not assume `allow_descartes_approach_collisions` was ignored just because the token says `arrival_descartes_failed`. Check `DescartesSearch[...]` first: accepted samples/edges plus `search_results=0` can be a graph-search cost issue, such as allowed-collision distance producing negative Dijkstra weights.
- Before proposing a frame-orientation fix, isolate the exact frame that authored the disputed basis and quote its source line. Do not conflate carousel/clamp slot bases, registration frames, and disk chiplet/named-location bases just because they all sit in the same targeting chain.
- Before blaming an HC2 arm/tool collision bug on a frame mismatch, verify the exact intended roles of `live_pose_cs`, `tool_interface`, and `ati_tcp` in the current stack. The right diagnosis can depend more on how attached collision geometry is encoded into MoveIt than on a guessed offset.
- For Tesseract/Descartes IK failures, do not call a TCP pose unreachable until you have separated true reachability from solver sampling. A reachable pose can produce zero Descartes vertices when the configured IK plugin/seed path fails; compare against MoveIt/prior pipeline behavior and inspect the Tesseract IK solver/tip/offset setup first.
- When Descartes diagnostics show accepted vertices on every rung and passing edges between every rung, do not keep tuning IK or collision policy. Inspect the graph search/path-extraction layer, including edge costs and any custom BGL visitor code, before changing planner geometry.
- For staged-seed `MOVE_ARM` failures, keep boundary-stub validation and bridge validation separate. `bridge_failed` means the seam-to-seam connector (direct interpolation or OMPL) was infeasible; do not blame approach-path collision checking unless Descartes/boundary diagnostics independently show the stubs were rejected for collision.
- When adding live planner diagnostics during an active robot-debugging session, keep the touched package warning-clean. New diagnostics are much less useful if compiler warnings bury the planner failure signal in rebuild output.
- When a user asks to root-cause a native crash, do not jump straight to substituting a safer local implementation. First recover prior incident notes, isolate the exact live call path, and get or plan a native backtrace/reproducer for the crashing upstream call.
- Do not diagnose arm-planner failures from one nearby planning-scene bounding box alone. First prove the actual swept route/contact pair from planner diagnostics or state-validity checks; static scene proximity is only a hypothesis.
- Do not infer dispatch-time gripper/tool occupancy from the current live UI state after a failure. Operators often change hardware state while troubleshooting; reconstruct the state from the failed action's command payload, event timing, or planning-scene snapshot at dispatch time.
- For live HC2 planner failures, check the ROS container namespace before declaring logs/snapshots missing. The current staged-seed snapshot is usually inside container `/tmp`, and detailed node diagnostics are under container `/root/.ros/log`; host `/tmp` may be stale from an older run.
- When a `MOVE_ARM` action sits `Queued` for seconds before dispatch, compute the time from runtime events and check the active gate before blaming the planner. In the current demand-projection path, look for pending `GeometrySyncCoordinator` work or a slow Tesseract projection service call; do not chase removed drain/resync services.

### Architecture / ownership

- Do not push runtime-owned sequencing or live-state derivation into the bridge or browser just because the frontend button needs it. Add a first-class runtime contract for it instead of chaining existing commands client-side or rebuilding live truth in the bridge.
- When a runtime/provider seam is meant to be production-ready, verify the launched entrypoint and production launch actually construct it. A provider class that exists only as an injectable option is still an unimplemented feature if `main()` / launch defaults keep instantiating the unavailable stub.
- Do not make a retained global status topic pretend it owns selected-item detail state. Keep the retained topic global (run-state + active item + global error); selected-item completion/revision/counts belong on the selected-item fetch path.
- When a frontend helper consumes engine-owned scene state, strengthen the engine-to-helper contract instead of scattering `Map.get()` / nullable-parent checks through the leaf helper. Centralize weak-library lookups (`Map`, `Object3D.parent`, mesh-catalog maps) in the owner and pass required objects/URIs downstream.
- For repeated-telemetry dedupe, compare to the last published/applied sample in the same representation at the producer or consumer seam. Do not invent new cross-layer helper APIs just to reconstruct equivalent state elsewhere.
- When adding a new `HiveLiveState` chunk, route runtime mutations through `StateManager` methods. Chunks own their local invariants; coordinators and bridge-adjacent helpers must never mutate live state directly.
- Before deleting "unused" retained-interface fields, inspect old producers and the surrounding policy path, not just current direct reads: a field can be dead while the behavior it once carried is still required and must be restored somewhere else.
- Do not introduce per-destination proxy/alignment-frame IDs when the ambiguity is really an arm/tool convention problem. Keep semantic destination IDs pointing at the physical support location; first ask whether the missing transform is universal to the arm/tool stack.

### Data / schema

- Do not edit Alembic migration files for incremental schema updates. If tests/runtime fail due to missing table/column after model changes, ask the user to run migrations rather than patching Alembic.
- Do not create duplicate override-schema classes when a command params dataclass already exists. Use plain mapping kwargs validated against the existing `*CommandParams` fields.
- When a seed/config schema carries a field for a nested definition row, verify the seeder actually writes it into that row — not just that the parent config layer parses it. YAML can look correct while hydration silently sees `None`.
- Loader-validation tests must seed DB-valid rows. If a model has DB `CHECK` constraints, do not insert intentionally invalid rows via helpers — seed constraint-valid values and trigger the invalid condition at load-time through unit combinations or converter monkeypatching.
- When a live frame will later be calibrated per channel/tool, keep per-channel copies as separate frame IDs even when the nominal pose is the same. Collapsing to one shared ID breaks later runtime calibration (all channels move together).
- When updating tests for a behavior change, prefer asserting observable behavior and call flow. Do not add brittle guards around internal import mechanics unless the import path itself is the product requirement.
- For YAML-backed seed regression tests, load the validated config and derive expectations from it instead of copying raw YAML numbers. Hardcoded literals go stale the moment the seed config changes and turn a config update into a fake regression.

### Runtime semantics

- Encode bake-time decisions on the action itself. When runtime behavior depends on a decision made at bake/compile time, encode that decision state on the action/command — not a hidden side-channel invisible to canonical call paths.
- Do not silently clamp retained motion-goal targets at the leaf boundary. Reject out-of-range numeric targets explicitly; otherwise the runtime can mark a motion action successful on a coerced pose and poison downstream semantic state (e.g. the stage's current disk index).
- Before changing telemetry/state-layer sentinel semantics, audit every current producer of that field end-to-end. A producer placeholder like `-1` may already flow on every retained sample — turning `None` from "skip update" into "clear state" can silently convert an ignored placeholder into a destructive clear on every telemetry callback. Flag fragility like this to the user and ask before acting.
- When one telemetry-driven field feeds both `ManualOpsContext` and `UiExecutionSummary`, do not use `elif` between their recheck requests — shared-field changes can require both projections to republish in the same cycle.
- For sim-only features built on the manual-arm route, do not silently make the scan path stricter than `command_arm_pose`. Keep its collision-execution semantics unless the user explicitly requests a stricter policy.
- Do not assume `departure_stub_poses[0]` is the current arm pose unless an upstream producer explicitly prepends it. The current pose reaches `computeCartesianPath(...)` via `setStartState(current_state)`; stripping the first departure stub drops a real authored waypoint.

### Viewer / frontend specifics

- Frontend Vitest auto-discovers `src/**/*.test.ts`. A new `.test.tsx` file will be skipped even if the code is valid — check the repo's test include pattern before adding frontend test files.
- For one-shot frontend debug/catalog fetches, do not depend the request effect on broad heartbeat-driven status objects or on a pending-state toggle that causes the effect to clean itself up immediately. Key off the narrow status fields that actually matter, guard in-flight requests explicitly, and tie invalidation to the real lifecycle transition.
- Imperative async frontend engine setup must be React StrictMode-safe. In Vite dev, mount effects run twice; without cancellation/cleanup guards, a Three.js viewer can hit dev-only gray-screen races even though the production build works.
- Do not let optional viewer rendering extras kill scene startup. If a dev-only or non-topology asset (HDR environment map, etc.) fails to load, log it and keep the 3D viewer running with fallback lighting.
- For async runtime lifecycle commands, never show UI success from the accepted HTTP/ROS response alone. Treat hydrate/load/mode commands as acceptance surfaces and drive completion/failure messaging from pushed runtime state (`active_lifecycle_operation`, loaded IDs, manifest/init refresh, `FAULT`).
- For async Three.js asset replacement, do not use captured object identity as the stale-load guard. Use explicit per-target request versions plus scene-generation checks, and always dispose stale-loaded or detached geometry/material resources when a mesh swap or scene rebuild is superseded.
- For async scene builders that use an engine-lifetime shared asset cache, pass an explicit current-build predicate into the builder and check it before installing late load results. Engine teardown or manifest supersession can happen after a load starts but before it resolves.
- For OrbitControls invalidated render loops, do not keep RAF alive just because interaction started. Use control events only to invalidate; `controls.update()` returning `true` is the authoritative signal that camera motion or damping is still producing visible change.
- For viewer init/load races, do not suppress async scene/mesh render leases just because the renderer stack is not ready yet. `loadManifest()` and async visual work can start before `init()` finishes; keep the lease alive so rendering begins immediately once the renderer/post-FX stack becomes ready.
- If a TypeScript/JS API returns `Promise<T>`, keep disposed/error paths promise-shaped too. A sync throw from a promise-returning loader bypasses contextual error wrapping and leaks cleanup paths like render-lease release callbacks.
- For viewer-only draft previews, do not couple preview visibility to the underlying data-overlay toggle unless the product explicitly says they are the same control.
- Do not key editor-draft reset effects off generic refresh counters when the same entity's local draft must survive successful commands. Scope the reset to identity-change only (e.g. selected tool changes).
- When a manual-ops UI hides a target group, hidden selections must become unselected/invalid immediately — fallback lookups that keep them alive can dispatch an invisible target.
- When browser-topology resets rebuild the same `kuka_arm` entity ID, drop the old retained `kuka_arm` joint group. Replaying stale retained arm joints suppresses startup defaults and carries a pre-reset pose into the next manifest.
- When a browser mount/attach is supposed to represent a physical latch with a canonical local pose, do not use reparent helpers that preserve the child's current world transform on first attach. During reload/reconnect those helpers bake stale cross-stream timing into a persistent local offset — recompute the canonical local pose explicitly or wait for coherent frame state.
- For visible selected-object mask passes, the depth prepass must include the selected object itself. Hiding it during depth prepass breaks self-occlusion and leaves the outline covering random parts of the mesh.
- For Three.js `EffectComposer` pipelines, do not manually resize individual passes back to CSS pixels after `composer.setPixelRatio()` / `setSize()`. Composer already sizes passes in device pixels; overriding changes outline width/softness. Add a pass-local supersample scale if one pass needs higher quality.
- For depth-tested in-scene debug lines, do not force early render. Keep depth testing enabled and render late enough that the scene depth buffer is populated.
- When a bridge/WebSocket `init` payload carries bootstrap state (e.g. `runtime_status`, `recent_events`), update the frontend types and init handler to consume it immediately instead of waiting for later incremental pushes.
- Viewer bootstrap pending-patch merge rules: frame patches by `frame_id` (latest wins), joints by `(entity_id, joint_name)`, visual payloads preserve last non-null. Do not keep only the last whole pending payload — that silently drops arm pose and clamp visuals across reload.
- When normalizing ROS messages into browser-facing event/summary payloads, preserve `header.stamp` as a typed timestamp if the UI renders chronological history or derives durations.
- Before calling an apparent frontend/runtime mismatch a bug, check whether the product intentionally separates destination identity from current measured state (e.g. traversability's selected-tool map vs mounted tool).
- Before delegating HC2 frontend/bridge work from an older prompt, verify the current frontend layout. This checkout may use `frontend/src/net`, `frontend/src/state/useHiveStore.ts`, `frontend/src/viewer/*` instead of older Redux-style paths.

### Runtime / scheduling specifics

- `PartStateSnapshot.probes` is the canonical probe-identity source; do not infer probe identity from generic non-pipette/non-occupancy tool IDs.
- Do not clear one-time projection loaded flags during an active-mode teardown. TF hierarchy load, startup SceneBridge world-object load, startup mounted-tool attach replay, and `scene_version` belong to the hydrated runtime session; mode-cycle teardown should reset live connectivity and Tesseract sync state only.
- Do not add explicit resume cursors for linear voxel scans when completion is at whole-voxel granularity — resume from the first stored voxel with `tested == 0`.
- For traversability subset rescans, do not clear measured bits and reuse the global incomplete-voxel scan loop. The runtime model has one measured bit per voxel; subset rescans need an explicit ordered target list and a selected-channel overwrite path.
- When rebuilding scheduler dataclasses from JSON, ignore `init=False` fields (runtime-only baked metadata, e.g. resolved `MoveArm` approach types). Feeding them back into `__init__` breaks round-trips and confuses authored-vs-baked state.
- For HC2 activation tests with background waiter callbacks, wait for the final observable state transition — not just the intermediate readiness flag. Thread timing can expose an intermediate `ready=True` before the callback-driven mode flip lands.
- When auditing timer-backed HC2 topics, separate four roles before changing code: correctness gating, first-sample readiness, late-join replay, dirty-state retry.
- For Dockerized HC2 ROS test seeding, do not build on the destructive `reseed_hive.py` wrapper or older full-hive builders when the YAML seed path already exists. Reuse the non-destructive YAML seed internals with test-owned copies of the HIVE and labware YAML snapshots, not live production files.
- When hydration/integration tests use the `test/integration_tests/model/full_hive/` builders, update them alongside production seeders for any newly used definition fields — cached test HIVEs will hydrate `None` otherwise.
- When encoding raw boolean output polarity, do not reinterpret an IO-function enum name or hardcode one consumer's inversion rule. Keep the enum as signal identity, model selector-line polarity in routing metadata, and apply the same semantic-active-vs-raw-wire rule in both bake-time command resolution and no-hardware simulators.
- When a runtime test helper targets one subsystem (e.g. geometry/completion bookkeeping), isolate unrelated projection publishing at the helper boundary instead of bloating fixtures until they accidentally satisfy a much larger payload builder.
- For `MoveArm` auto approach-path selection on `ToolPosition` targets, if the arm has a mounted tool the lookup MUST use that mounted `tool_definition_id` and require a real stored path for it — never fall back to `tool_definition_id=None` (the pickup bucket) as a convenience.
- Before calling Tesseract radius-aware approach waypoints "missing", trace all layers: authored `ApproachWaypointDTO.radius_mm`, ROS request fields, runtime converter population, executor pin construction, and IFOPT move-profile registration. Scaffolding can exist while still not being wired into the active MoveArm path.
- When adding Tesseract ApproachPath corner rounding, do not resample straight approach spans at fixed spatial spacing. Under inert approach mode every sampled source state outside the bridge becomes a fixed IFOPT waypoint, so fixed-distance line sampling turns one authored approach segment into dozens of hard timing points. Add samples only where the feature actually changes geometry, such as fillet arcs or explicitly requested terminal rail points.
- For Tesseract planner performance work, do not rank small debug/ghost message emission as a top optimization without timing evidence of middleware backpressure. Focus first on IFOPT collision term count/evaluator type, margins, active collision scope, and NLP size.
- When reviewing wrist-winding expansion, distinguish the helper's general offset list from the robot's actual joint-limit window. A helper that loops `{0, +2π, -2π}` may still produce at most two legal windings on the KR10's roughly ±240° joint_6 range.
- For manual `InjectActions` rejection design, separate request-envelope failures from per-action validation failures. Only per-action failures with a real rejected action belong in Monitoring `REJECTED` events; rejected requests must not churn manual queue projections or progress wakes, and validators must honor action-specific optional IDs.
- For a `MoveArm` debug/manual pose editor that is intentionally world-space, do not "fix" a confusing zero origin by re-rooting the command to a local arm frame. Populate the draft origin from the selected arm's current `live_pose_cs` world position instead.
- When diagnosing the live HC2 Docker runtime, remember `/tmp/hive_execution_runtime.log`, `/tmp/hive_ui_bridge.log`, and trajectory metrics live inside `hc2-live-kuka`. Use `docker exec` for log searches; host `/tmp` is not the runtime log namespace.
- After a DB reseed or HIVE rehydration, do not reuse an old `RuntimeScheduleRecord` UUID just because the schedule name still looks right. Regenerate the schedule against the current DB state and verify schedule load rejects stale hardware/inventory UUIDs before automatic-mode testing.
- When diagnosing Tesseract staged-seed failures, verify the exact latest failure token and snapshot before assuming the bridge stage failed. `/tmp/tesseract_staged_seed_last_failure.json` is overwritten by later failures, and `arrival_descartes_failed` can look like a bridge issue from the UI even though bridge/direct/OMPL were never attempted.
- Descartes Light's BGL SVDE solver builds vertices by mutating one Boost graph inside an OpenMP loop. In HC2 staged seed planning, construct that solver with one build thread unless the upstream graph construction is replaced with a thread-safe implementation.
- Tesseract projection apply should not reintroduce high-rate scene-diff fanout or MoveIt full-scene snapshots. Keep one demand-driven flat projection pull, cache versioned geometry, and refresh world poses from each response.
- Optional planner prewarm must stay optional in the execution path. Do not guard first planning on a prewarmed "ready" flag before calling the same prepare/apply method that can make the planner ready.
- When applying authoritative scene projections, dedupe remove commands before batching. A world-resident object becoming carried can otherwise be queued for removal both as "missing from world" and as "present in carried", making the second remove fail even though the projection is valid.
- Do not claim an HC2 runtime/Tesseract fix is done after unit tests or a clean restart alone. Rebuild, restart/rehydrate ROS, load the target runtime schedule, switch to `FULL_AUTO`, and let the schedule exercise the real action loop before reporting the issue fixed.
- When advising that a schedule can be reloaded to preserve planner caches, explicitly check whether the live inventory still matches the schedule's starting preconditions. `load_schedule` rewinds action statuses; it does not move trays/tools/objects back to their initial layout.
- Do not model whole-schedule restart as a variant of single-action retry. Retry preserves surrounding schedule history and targets one terminal action; restart rewinds the loaded study lane to fresh-load runtime state.
- Do not use TF2 latest-common-time lookups for SceneBridge projection poses. Dynamic branches can have slightly skewed stamps and throw future-extrapolation errors even when the latest local transform cache has a valid current snapshot.
- For tracked-root geometry with multiple dynamic frames, gate root convergence on the whole stamped sync surface, not on whichever frame update arrives first.
- When a browser action form change is meant to remove a dispatch rejection, search the exact rejection text first and patch the runtime guard too; preserving UI state is not enough if the backend still rejects the submitted shape.
- For runtime disk chiplet / named-location geometry, "disk CS" means `registered_coordinate_system_id` only — missing registered geometry is an invariant break, not an excuse to fall back to the disk body's `coordinate_system_id`.
- For stage chiplet alignment, separate "where the point should land" from "what in-plane yaw the disk should have." Projection can determine the goal origin; theta may still come from a different frame (e.g. microscope optical-axis X projected into the target plane). Do not assume the same frame supplies both.
- For hardware-facing ROS leaves, do not implement timed pulses or polling waits with `time.sleep()` in action callbacks. Use deadline/condition waits that do not hold the hardware driver lock, so sibling services, reads, writes, and state publication can keep responding while the action remains active.
- For `ros2_control` mock-arm overrides, writing the hardware plugin's joint state/command vectors while `joint_trajectory_controller` stays active is NOT enough. In `read() -> update() -> write()`, `controller_manager->update()` restores the controller's old desired command and `realistic_joint_following_` drifts the arm back. Any raw override must also quiesce or reseed the controller-side desired target.
- When the user asks for a master plan across multiple named runtime seams, do not narrow to one seam and handwave the rest as "later work." Every named seam needs a concrete decision gate, candidate ownership options, validation criteria, and acceptance conditions — even for deferred seams.
- When diagnosing HC2 mode-change hangs, first prove whether the `runtime/set_operation_mode` service callback actually ran. A timed-out UI request can be the first symptom of an already-wedged rclpy/FastDDS executor, not evidence that the inactive/manual/full-auto handler body hung.
- Do not add "compatibility wrapper" methods in greenfield HC2 runtime code. Rename local call sites to the canonical method and make tests use the canonical API instead of preserving old internal names.

### Reagent / solver specifics

- In the reagent composition solver, do not enforce per-source minimum volume floors or add iterative post-solve source-dropping loops. Use a single-pass tolerance-bounded MILP with a source-minimization objective; handle pipette minimum-transfer checks as a separate downstream executability pass.
- For reagent container planning, keep study-preference and fallback rules in ONE table discriminated by a tier column — not split tables. If pool rows for a role exist but all `available_count` values are 0, produce "no eligible containers" (not unconstrained fallback). The planner must consume/decrement container capacity across demands and deterministically fall back when preferred types are exhausted.

### Packaging / build / shell

- After a failed full `hc2_ws` rebuild, do not treat missing ROS packages in `install/` as the root cause. The full rebuild script deletes `build install log` before compiling, so any compile/link failure leaves the overlay partially missing. Read the first build/link error and fix that.
- When a downstream Tesseract/CMake package reinstalls `libpcl-dev` after the canonical image purges PCL headers, install `libpcap-dev` with it. PCL's `io` component treats Pcap as optional but prints a stderr warning if its headers are missing, which makes clean colcon builds look noisy or failed.
- Do not run `pkill -f <pattern>` in the same shell command that later contains `<pattern>` in a relaunch command. `pkill -f` can match and kill its own shell. Split stop/start into separate commands or use a pattern that cannot match the command line.
- In `hc2_ws`, do not place reusable Python modules at workspace-root `hc2_ws/src/`. Each ROS package `setup.py` installs only code inside that package, so shared helpers must live under a real package directory and be imported through that package path.
- For ROS Python packages that use `setuptools.find_packages()`, every imported subpackage directory needs an `__init__.py`. A source tree can look complete while the installed overlay silently omits subpackages like `hc2_models.ref` or `stage_control.acs`.
- For `ament_python` ROS packages, keep every `data_files` source path relative to the package directory. Absolute repo-root paths fail `colcon` with `'data_files' must be relative`.
- When removing or renaming ROS services/actions, grep operational scripts under `scripts/` as well as launch files and tests. A restart/hydrate helper can still block on a deleted service even after the runtime code and launch tests are clean.
- When packaging runtime mesh assets, install only canonical mesh-catalog directories and preserve their relative subpaths. Flattened STL installs or included legacy mesh folders create basename collisions and break clean rebuilds.
- Runtime code that needs installed ROS assets should resolve the owning package share directory with `ament_index_python.get_package_share_directory(...)`. Do not derive `hive_description` mesh paths from another package's installed `__file__` location.
- When rewriting binary STL files, preserve the full 84-byte header and triangle-count field layout: 80-byte header, uint32 triangle count at offset 80, then 50 bytes per triangle.
- For FastAPI/Starlette static mounts in `colcon --symlink-install` ROS packages, enable `follow_symlink=True` for symlinked mesh/URDF/frontend directories — otherwise the route mounts successfully but asset requests 404.
- When comparing installed vs source frontend bundles under `colcon --symlink-install`, treat broken hashed-asset symlinks in the installed tree as stale data, not fatal errors. Asset-selection code should skip broken entries and prefer the newer source or a freshly rebuilt install tree.
- When splitting a clean `kuka_ws` rebuild around `joint_group_impedance_controller`, skip the dependent metapackages in the earlier pass too. `kuka_controllers exec_depend`s on it, and `kuka_drivers exec_depend`s on `kuka_controllers`, so dependent metapackages and their generated build-prefix scripts fail on the missing installed `package.sh`. After `joint_group_impedance_controller` builds, source the new install overlay before the final pass.
- Do not skip a ROS package from a broad colcon build just because HC2 does not launch it. First check whether a package that must build declares it as an `exec_depend`; if so, colcon can fail on the missing installed `package.sh` before runtime launch matters.
- For upstream plain-CMake packages inside a ROS workspace, use `<build_type>cmake</build_type>` and install `package.xml` under `share/<package_name>`. Do not mark them `ament_cmake` unless their `CMakeLists.txt` actually calls `ament_package()`, or colcon will pass ament-only variables and generate missing `local_setup.*` hooks.
- When adding HIVE tool seed helpers, always copy `object_mesh_model` from the tool config into the definition row for every tool type. If one helper omits it, the tool still seeds and hydrates, but startup collision specs and the UI manifest silently drop that parked tool.
- When cache-busting URDF mesh URLs, check file extensions after stripping query strings/fragments or use a loader callback that does this. `urdf-loader`'s default mesh loader tests the raw path with `/\.stl$/`, so `.stl?v=...` is treated as an unsupported format.
- When passing literal `$skill-name` prompts or other `$`-prefixed values through shell commands, single-quote or escape the dollar sign. Double-quoted shell arguments still expand `$name`, silently corrupting generated metadata.
- The user has explicitly allowed Black formatting churn, so run `uv run black...` on new code modules and do not waste time trying to collapse the diff back to hand-formatted lines. Keep the formatter output and move on to validation.
