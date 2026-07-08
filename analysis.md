# RoboTrainer (RT2) — Architectural, Scientific & Logical Audit

**Scope:** Static analysis only (no build, no execution), strictly along the invocation chain:
`srt` → `robotrainer_bringup/scripts/robotrainer_on_login.bash` → `sr3_on_login.launch` (LED + phidgets + battery + sync) · `roslaunch za_experimental rt2.launch` → `robotrainer_bringup/robot/rt2.launch` → `sr3.launch` → `cob_bringup/ros_control_base.launch` → `canopen_402.launch` (driver `/base/driver`) + controller yamls · `rt2_init`/`rt2_recover` (= `/base/driver/init|recover`) · `roslaunch robotrainer_panel robotrainer.launch` (RViz panel + `data_srv`) · `roslaunch za_experimental rqt_reconfigure.launch` / `rqt_overview.launch`.

**Reference:** Stogl et al., *"Spatial Control Actions for a Physical Training with a Smart Walker"*, ELMAR-2019 — Eq. (1) mass-damper admittance, Eq. (2) discrete recursive velocity, Eq. (3) velocity superposition, Eq. (4) trapezoidal force profile, Eq. (5) velocity compensation.

**Controller path audited (per configuration):** `fts_adaptive_force_controller` (type `robotrainer_controllers/FTSAdaptiveForceController`) as spawned by `rt2.launch`, plus the `FTSBaseController` math it inherits; the base `fts_controller` variant was cross-checked where it differs.

---

## 1. Executive Summary

The implementation is **mathematically faithful to the paper in its core formulas** — the discrete admittance recursion (Eq. 2), the trapezoidal profile (Eq. 4), and the velocity-compensation factor (Eq. 5) are all implemented correctly, symbol for symbol. The launch chain is **structurally sound**: all `<include>`/`<arg>` pairs resolve, all referenced yamls and the `iras` map exist, plugin registrations match the exported classes, the TF tree (map → odom_combined → base_footprint → base_link) is consistent, and `rt2_init`/`rt2_recover` correctly target the canopen node at `/base/driver`.

The problems live one level below the formulas, in four clusters:

1. **A system-wide sampling-rate inconsistency.** The CAN sync (which drives the `ros_control` update loop in `canopen_motor_node`) runs at **50 Hz** (`interval_ms: 20`), while the admittance controller discretizes with `update_rate: 200.0` (the yaml even says `# Check this`) and every modality PT1 assumes `controller_update_rate: 200.0` (the `.params` default is a third value, 100). If the loop is indeed 50 Hz, **every time constant in the system is effectively 4× larger than configured** — the robot is 4× more sluggish than the paper's model predicts, and "identical" parameters are not comparable across configs. The controller ignores the measured `period` handed to `update()`, so the error is silent.

2. **Concurrency defects between the real-time `update()` loop and every reconfiguration path.** Scenario loading from the RViz panel (`configure_modalities`), kinematics updates (`update_kinematics` / `set_state`), and dynamic reconfigure all mutate controller/modality state **without any lock the RT thread respects** — including `std::vector`s being cleared mid-iteration and `geom_` being `reset()` while dereferenced. These can segfault the canopen driver node (which is also the motor controller) while a user leans on the device.

3. **Type/unit mismatches that silently disable or distort documented behavior.** `max_deviation` is written by the editor as `double` but read by path tracking as `int` (→ uninitialized garbage corridor width); the backwards-force clamp compares a normalized force (≤1) against a Newton threshold (≈50) and never fires; the controller-modality "human let go" check compares normalized force against a 1 N threshold and (almost) always fires, permanently throttling virtual forces/walls in `controller_modalities` mode.

4. **Silent-failure tooling.** The scenario data service reports success even when `rosparam load` failed (and crashes/no-ops if `load`/`save` is called before `get_list`), so a trainer can believe a new scenario is active while the robot still runs the previous one.

Nothing in the audited chain prevents bring-up — the robot starts and drives. The findings above concern scientific validity, training correctness, and failure behavior under exactly the interactions the entry points exist for (loading scenarios, reconfiguring during a session, recovering from stops).

---

## 2. Scientific & Mathematical Discrepancies (Physics/Control Theory vs. Code)

### 2.1 Verified-correct implementations (for the record)

| Paper | Code | Verdict |
|---|---|---|
| Eq. (2) `Ẋₙ = Fₙ₋₁·K(1−e^{−1/rT}) + Ẋₙ₋₁·e^{−1/rT}` | `fts_base_controller.cpp:283` `new_vel = b1·force_old + a1·velocity_old` with `a1 = exp(−1/(rate·T))`, `b1 = K(1−a1)` (`discretizeController()`, `fts_base_controller.cpp:1018-1025`) | ✅ Exact, incl. use of **F from the previous cycle** (`force_old_` written after use). Works on normalized force/velocity (÷`max_ft`, ×`max_vel`), consistent with `K` dimensionless and `getScaledLimitedFTSInput()` (`fts_base_controller.cpp:1330-1344`). |
| Eq. (3) superposition | `applyModalities()` cascade (`fts_base_controller.cpp:876-938`): each modality adds its virtual velocity to the running twist | ✅ Equivalent for additive actions; areas manipulate velocity directly, as in the paper (Fig. 4). |
| Eq. (4) trapezoid | `virtual_forces.h:265-271`: `distance_factor = (d − r)/(m − r)` clamped to 1, with `m = trapezoid_max_at_percent_radius · r` | ✅ Exact (also correct in `virtual_walls.h:175-181`). |
| Eq. (5) velocity compensation | `virtual_forces.h:304-311`: `factor = (p_ref + e^{−p_ref} − 1)/(p_act + e^{−p_act} − 1)`, `p = 2r/(T·|ẋ|)`, applied only above the reference velocity | ✅ Exact, incl. the paper's "no reduction below ẋ_ref". |
| Mass/damping ↔ gain/time-const conversions | `discretizeWithNewMassDamping()` (`fts_base_controller.cpp:1042-1051`), `calculatevirtualdamping/mass` (`:1299-1307`) | ✅ `K = 1/D`, `T = M/D` consistent both directions. |
| Wall geometry (Fig. 3 gray/white regions) | `virtual_walls.h:132-153` segment-region test via projections | ✅ Correct endpoint-vs-segment classification; repulsion direction `wall→robot` correct; `d=0` guarded. |

### 2.2 S1 — Sampling-rate inconsistency corrupts *all* PT1 dynamics (Critical, scientific)

- The `ros_control` update loop is driven by the CANopen sync: `robotrainer_config/hardware/rt2_base_driver.yaml:2` (`sync: interval_ms: 20` → **50 Hz**), wired through `cob_bringup/components/ros_control_base.launch:15` and `drivers/canopen_402.launch`.
- The admittance discretization uses `update_rate: 200.0` — `rt2_adaptive_force_controller_za_experimental.yaml:11` (comment literally `# Check this`) — in `discretizeController()` (`fts_base_controller.cpp:1021`).
- Every modality PT1 independently assumes `controller_update_rate: 200.0` (`modalities_za_experimental.yaml`), while the parameter's *declared default* is `100.0` (`robotrainer_modalities/cfg/VirtualForces.params:14`, whose own description warns it must equal the controller rate).
- `FTSBaseController::update(time, period)` **ignores `period`**; nothing validates the assumed rate against reality.

**Consequence:** if the true rate is 50 Hz, `a1 = e^{−1/(200T)}` executed at 50 Hz yields an effective time constant **4× the configured one** (e.g. `x_time_const: 0.5` behaves like ≈2.0 s; a virtual-force `time_const_T: 0.8` behaves like ≈3.2 s). Steady-state gain `K` is unaffected (b1/(1−a1) invariant), so the error is invisible in slow quasi-static tests but wrong in exactly the transient behavior the paper models (Fig. 6's "delayed virtual velocity"). It also breaks Eq. (5): `param = 2r/(T·|ẋ|)` uses the *configured* `time_const_T`, not the effective one.
**Verification on robot (Phase 2 first step):** print `period.toSec()` in `update()` or `rostopic hz /base/joint_states` — then either fix the rate constants or (better) discretize from `period`.

### 2.3 S2 — Backwards-force clamp is dead code (unit mismatch) (High)

`fts_base_controller.cpp:252`:
```cpp
if (force_input_[0] < -max_ft_[0]*backwardsMaxForceScale_) force_input_[0] = -max_ft_[0]*backwardsMaxForceScale_;
```
`force_input_` is **normalized to [−1, 1]** by the time it reaches `update()` (both `FTSController::update()` `fts_controller.cpp:21` and the adaptive path `fts_adaptive_force_controller.cpp:243-257` pass it through `getScaledLimitedFTSInput()`), but the threshold is in **Newtons** (`−100·0.5 = −50`). The condition can never be true, so `backwards_max_force_scale` (yaml: 0.5) has **no effect**. Only the backwards *velocity* clamp (`:300`) works. The same unit confusion appears in the "not started" reconfigure branch. Fix: compare against `−backwardsMaxForceScale_`.

### 2.4 S3 — Velocity-adaption mode switching corrupts the dynamics baseline (High)

`fts_adaptive_force_controller.cpp:1328-1346` (dynamic reconfigure): when the adaption type *changes*, the code snapshots `pre_adaption_base_params_` **if the OLD mode was ≠ NONE** — i.e. it saves the *already-adapted* gain/time-constant as the new "baseline", then "restores" those same values. Net effect:
- Switching NONE → DAMPING_*: baseline is the stale init-time snapshot (misses any base-parameter reconfigure since then — `base_reconfigured_flag_` only refreshes `base_max_ft_`, `fts_adaptive_force_controller.cpp:923-925`).
- Switching DAMPING_* → NONE: the restore is a no-op; **the last adapted damping persists as the permanent controller dynamics**, and the true baseline is lost.
The snapshot condition is inverted: it must be taken when *entering* adaption from NONE (or kept immutable alongside yaml values).

### 2.5 S4 — Controller-modality safety gate uses Newton threshold on normalized force (High)

`modalities_virtual_forces_controller.cpp:104-110`:
```cpp
if ((human_input_force.length() < 1.0)) return true;   // "human let robot go"
```
`data_in.wrench_` is filled with the **normalized** `force_input_` (`fts_base_controller.cpp:919-924`), so `length() < 1.0` is true unless the user pushes at ≥100 % of max force in combined magnitude. Consequently the branch at `modalities_virtual_forces_controller.cpp:67-69` (and its twin in `modalities_virtual_walls_controller.cpp:85-88`) is *always* taken: the virtual velocity is clamped to the human velocity magnitude (`calcVelLimitationFactor`), and the smooth `scaleVelFactorBack` branch is unreachable. In `controller_modalities` mode the spatial actions are therefore **systematically weaker than designed** (a stationary user ⇒ zero virtual velocity ⇒ force areas cannot move the robot at all). If the passivity clamp is intended, the threshold must be expressed in the normalized scale (e.g. `min_ft/max_ft`), and the dead branch removed or reachable.

### 2.6 S5 — Division-by-zero → NaN paths in modality math (Medium)

All are reachable in normal training and produce NaN that propagates into the commanded twist (caught only later by the base controller's NaN guard `fts_base_controller.cpp:393`, which halts the robot with `ROS_FATAL` each cycle):
- **Eq. 5 projection:** `virtual_forces.h:304` divides by `|velocities_on_entry_[i]|²`; entering a force area with ~zero entry velocity (robot repositioned into the area, or first cycle after configure while inside) ⇒ NaN.
- **Areas `keep_direction` / `invert_direction`:** `virtual_areas.h:161,169` divide by `|area_entry_velocity_|²`; entering an area at standstill ⇒ NaN.
- **Path tracking:** intersection denominator `path_tracking.h:294-296` (zero when `direction` degenerates); the NaN *workaround* at `:312-317` then **replaces the entire output velocity (user input included) with the previous virtual velocity** (`data_out.linear = toMsg(previous_velocity_linar_)`) instead of `data_in + prev` — a one-cycle velocity glitch every time it triggers.

### 2.7 S6 — Path-tracking force law: hardcoded dead-zone breaks small corridors (Medium)

`path_tracking.h:335-337`: `d0 = 0.2` (m) hardcoded; `F = max_force/(max_deviation − d0)·(distance − d0)`. For `max_deviation ≤ 0.2` the gain is infinite/negative (repulsion). The paper's evaluation used 0.5 m, but the panel enforces only `MIN_MAX_DEVIATION` (`robotrainer_editor_section.cpp:135`) — see A3 for the type bug that makes this worse. The quadratic branch and end-of-section handling (`pos_prev_segment_ >= size − 6`, `:176`) additionally mean **sections with < 7 points terminate tracking immediately**, and leaving the section zeroes the PT1 state abruptly (`:181-182`, the jerk the paper's outlook mentions).

### 2.8 S7 — Multi-wall velocity limiter uses the wrong wall (Medium)

`virtual_walls.h:155` assigns `current_distance_to_wall` for **every** wall iterated (even those out of range); the "HACK" limiter block (`:242-261`) then uses the distance to the **last wall in the array**, not the nearest active one. `min_distance_to_wall` is computed but initialized to `0.0` and compared with `<` (`:128,168-171`) — never updated, never used. With a single wall the behavior is correct; with several, the human-velocity limiting near walls is driven by an arbitrary wall. Also `last_scaled_virtual_force_ += resulting_force` *inside* the accumulation loop (`:217`) over-counts earlier walls (N·F₁ + (N−1)·F₂ …) in the value exposed to the walls-controller integrals.

### 2.9 S8 — Documented deviations from the paper (accepted design, should be stated)

- **Virtual-force dynamics disabled by configuration:** `modalities_za_experimental.yaml` sets `linear_velocity_from_force_damping: true` and `time_const_T: 0.0` — virtual force velocity is `v = K·F_{n−1}` (`virtual_forces.h:317-318`), *not* the paper's PT1 (Eq. 2). The paper's Fig. 6/7 behavior (delayed yellow line) will not reproduce with this config.
- **One shared PT1 for all walls** (`virtual_walls.h:228`) instead of "individual dynamic parameters per action"; forces do keep per-area recursion state (`prev_virt_forces_[i]`), walls do not.
- **Modalities only act while the user grips** (`fts_base_controller.cpp:313`): a released, still-moving robot gets velocities zeroed (`:346-353`) rather than being repelled by walls — a reasonable safety choice, but different from the paper's pure superposition.
- **Parametrization distance is integrated from *commanded* velocity** (`fts_adaptive_force_controller.cpp:931-957`, `getOldVelocity()`), not measured platform state — slip/limits bias the spring during max-force parametrization (the paper's "momentum instead of deviation" outlook).
- **Area `amplification` is loaded but unimplemented**: `virtual_areas.h:290` reads it; `update()` implements only `double_speed`/`half_speed`/inversions/counterforce (`:153-204`) — editor-set amplification values are silently ignored (paper Fig. 1C "velocity amplification").

### 2.10 S9 — Minor scientific nits (Low)

- Global counterforce passes the configured Newton values through `getScaledLimitedFTSInput()` (`fts_base_controller.cpp:970`), so a counterforce below `min_ft` (3 N x/y) normalizes to **zero** — small counterforces silently do nothing.
- `robotIsMovingForward()` uses `velocity_old_[0] > −1e−5` (`fts_base_controller.cpp:1553-1555`) — "not moving backwards" ≠ "moving forward"; used to select fwd/bwd leg-distance baselines.
- `scaleBetweenValues()` mutates its `scaling_reference` argument in place (aliases `velocity_percent`) — latent trap if reused after the call (`fts_adaptive_force_controller.cpp:893-915`).
- Adaptive `setMaxFt()` every cycle re-normalizes force while `velocity_old_`/`force_old_` keep the old scale for one step — transient inconsistency in the recursion, self-correcting, but worth noting for data analysis.

---

## 3. Architectural & Breaking Bugs (ROS Launch & Node Chain)

### 3.0 Launch-chain map (verified, static)

```
za_experimental/rt2.launch  (args all declared in child ✅)
└─ robotrainer_bringup/robot/rt2.launch
   ├─ [rosparam] /base/{controller yaml}, /base/driver/use_fts=true,
   │             modalities_chain.yaml (→ /robotrainer/modalities_chain_config),
   │             modalities_za_experimental.yaml (→ /modalities/*)
   ├─ components/fts.launch          (param-only in non-standalone mode; FTS lives in the driver process)
   ├─ robot/sr3.launch
   │  ├─ cob_bringup/components/ros_control_base.launch
   │  │  ├─ drivers/canopen_402.launch → node "driver" ns /base  ⇒ /base/driver/init|recover ✅ (rt2_init/rt2_recover)
   │  │  ├─ spawner: joint_state_controller odometry_controller fts_adaptive_force_controller
   │  │  ├─ twist_mux + velocity_smoother (→ /base/twist_controller/command)
   │  │  └─ stuck_detector
   │  ├─ controller_manager load: twist_controller (loaded, not started) ✅
   │  ├─ map_server: cob_default_env_config/iras/map.yaml ✅ exists
   │  └─ cob_navigation_global/amcl_node.xml (system pkg): odom_combined/base_footprint/map ✅
   │     matches odometry_controller (frame_id "/odom_combined" → tf1 strips "/") ✅
   └─ leg_tracker (DISABLED: enable_leg_tracking=false — see A11)

srt → robotrainer_on_login.bash
   ├─ rosservice /robotrainer_hw/set_state  (served by robotrainer_rear_wheels_state_publisher.py
   │   from sr3_on_startup.launch — assumes the boot launch ran; silent failure otherwise)
   └─ sr3_on_login.launch = iirob_led_rectangle + phidgets + battery_voltage + rt2_external_sync ✅
       (sync subscribes /base/virtual_forces/modalities_debug/resulting_force — namespace matches ✅)

robotrainer_panel/robotrainer.launch = data_srv (global ns: /save /load /get_list) + rviz (panel plugin export ✅)
rqt_reconfigure.launch / rqt_overview.launch → rqt_gui with existing perspective files ✅
```
Prerequisite worth documenting: `rt2.launch` and `srt` both assume `sr3_on_startup.launch` already ran at boot — it loads `robot_description`, `/base/wheel_controller/*` (wheel geometry; the x/y positions there are commented out and resolved from the URDF), the lasers, and the `robotrainer_hw` helper node. Running the entry points on a fresh roscore without it fails (controller init: `parseWheelParams` on an empty namespace).

### 3.1 A1 — Scenario (re)configuration races the real-time control loop (Critical)

`configure_modalities` (`fts_base_controller.cpp:869-871`) executes `configureModalities()` **directly in the service thread with no lock and without stopping the controller**. Every modality `configure()` starts with `clear()` on the vectors the RT `update()` is concurrently iterating (`virtual_forces.h:437-445`, `virtual_walls.h:331-338`, `path_tracking.h:439-441`, `virtual_areas.h:236-240`). The panel triggers this exact path from the GUI ("Set Active" `robotrainer_editor_panel.cpp:367`, "Load Active" `:972`) while a session can be running and a user gripping. `applyModalities()` is gated only on `userIsGripping()` — nothing prevents the overlap. Outcome ranges from wrong scenario data mid-cycle to a segfault of `canopen_motor_node` — which is the motor driver; the platform then coasts with a leaning user.

### 3.2 A2 — `update_kinematics` replaces `geom_` under the RT thread (Critical)

`updateWheelParamsCallback` (`fts_base_controller.cpp:848-864`) calls `GeomController::update()` → `geom_.reset(new Controller(...))` (`fts_GeomController.h:66-69`). "Stopping" via `protectedToggleControllerRunning(false)` only sets `running_ = false`; `update()` still calls `geom_->calcDirect()` / `calcControlStep()` **every cycle regardless of `running_`** (`fts_base_controller.cpp:210-214, 409-414`). The scoped-ptr reset in the service thread is a textbook use-after-free against the 50 Hz RT thread. Reachable from `/base/update_kinematics` and from `/robotrainer_hw/set_state` (the `srt` login script calls it with `without_controller_updates: True`, which skips the dangerous part — but the rqt/service path with updates enabled does not).

### 3.3 A3 — `max_deviation` type mismatch: path-tracking corridor is garbage (Critical for path tracking)

- Panel writes `double` (`robotrainer_editor_section.h:29`; `robotrainer_editor_section.cpp:284-287`), scenario files store e.g. `max_deviation: 0.3` (`editor_demo_scenario.yaml:79`).
- `PathTracking::configure()` reads it as **`int`** (`path_tracking.h:511-513`). `ros::param::get(…, int&)` on a double param **fails and leaves the variable uninitialized**; the garbage value is pushed into `max_deviations_` and used as corridor width and force denominator (`:337`).
- Legacy files additionally store `force_distance_function` as `int` (`editor_demo_scenario.yaml:79`), which fails the `std::string` read (`:507`) → empty string → "not implemented! PathTracking is deactivated!" (silent for the trainer; current panel writes strings, so this affects old scenarios).

Same-family bug: sections skipped for `< 2` points still push their `force_distance_function`/`max_deviation` (`path_tracking.h:496-513`), so **all later sections read the wrong parameters** (index misalignment across the parallel vectors).

### 3.4 A4 — Data service silently fails; scenario can stay stale (High, training-safety)

`robotrainer_data_service/nodes/data_srv`:
- `dir` is only assigned inside `get_file_list`. Until then the module-level name resolves to the Python **builtin `dir`**: `save_to_file` does `if dir:` → truthy → `dir + file_name` → `TypeError` (service exception); `load_from_file` does `isinstance(dir, str)` → False → relative paths resolve against `~/.ros` → file not found.
- `subprocess.call([...])` return codes are ignored and both services **always return `success=True`**.

Combined with the panel flow (`dataServiceLoadActive` → `/load` → `configure_modalities`, `robotrainer_editor_panel.cpp:969-973`): a failed load still triggers `configure_modalities`, which re-reads the **previous** `/robotrainer/scenario` parameters — the trainer sees a normal "loaded" flow while the robot runs the old scenario. (Absolute paths from the file dialog bypass the `dir` bug, which is why this partially works in practice; the combobox/relative-name path and `save` before any `get_list` do not.)

### 3.5 A5 — Stale rotation command after controller restart (High)

`WheelControllerBase::starting()` resets `twist_command_.linear.x/y` and **`angular.x`** — not `angular.z` (`fts_wheelControllerBase.h:64-68`). A stale `angular.z` (last joystick/velocity-smoother message, topic `/base/twist_controller/command` hardcoded at `:47`) is replayed as soon as twist input is active after a restart. The adaptive controller's autonomous-return feature toggles `use_twist_input_` at runtime (`fts_adaptive_force_controller.cpp:427,461`) and writes `twist_command_` fields **directly and without `twist_mutex_`** (`:441-457`) — a restart or phase abort in between leaves a nonzero rotation that executes on the next twist-enabled cycle. `topicCallbackTwistCmd` also writes without the mutex the reader holds (`fts_wheelControllerBase.h:127-138`).

### 3.6 A6 — Duplicate dynamic-reconfigure servers per modality namespace (High for the rqt_reconfigure entry point)

Both the base modality *and* its controller-modality wrapper instantiate the same plugin class, and each constructor creates a `dynamic_reconfigure::Server` on the **same fixed namespace**: `VirtualForces` at `/modalities/virtual_forces` (`virtual_forces.h:111`) is constructed twice — once by `loadBaseModalityInstances()` (`fts_base_controller.cpp:673`) and once inside `ModalitiesVFController` (`modalities_virtual_forces_controller.cpp:20`); same for walls and path tracking. The second server's service advertisements (`set_parameters` etc.) collide with the first (roscpp refuses duplicate advertisement within the node). Practical effect for `rqt_reconfigure`: sliders on `/modalities/*` reach **only the first instance** (the `base_modalities` one); the instances actually used in `controller_modalities` mode keep their construction-time values, silently. Related latent defect: the "push yaml → GUI on first callback" mechanism is dead because `params_callback_first_time_` defaults to `false` (`modality_base.h:48`, only checked in `virtual_forces.h:376`); it currently works only because the dyn-reconfigure server itself reads existing values off the param server at startup.

### 3.7 A7 — Realtime-hostile operations inside `init()`/`starting()`/`update()` (High)

Inside the controller (runs in the CANopen sync cycle):
- Blocking **service calls** from RT context: `unsafeRecalculateFTSOffsets()` in `starting()` (`fts_base_controller.cpp:194`), and in the adaptive parametrization loop (`fts_adaptive_force_controller.cpp:225, 323, 460`).
- `led_ac_->waitForServer(ros::Duration(2))` in `init()` (`fts_base_controller.cpp:101`) stalls the controller-manager load service.
- Non-realtime `ros::Publisher::publish()` calls every cycle from modality `update()`s (`virtual_forces.h:223,346-347,363-364`; `virtual_walls.h:123,297-300`; `virtual_areas.h:211-214`), plus per-modality `tf2_ros::TransformListener` instances (7 buffers/subscribers inside the driver process, `modality_base.h:57-58`) and `lookupTransform` per cycle.
- Unthrottled logging in the RT path (`ROS_INFO("[virutal_walls.h] INSIDE")` `virtual_walls.h:172`; `ROS_INFO` in the VF limit path).
- `boost::mutex` locks and `diagnostic_.update()` in `update()` (`fts_base_controller.cpp:248,416`); spin-wait `while(!lock) lock.try_lock();` patterns (`:540-542,565-567`).

At 50 Hz with soft-RT this mostly manifests as jitter/missed sync — but a hung FTS offset service during `starting()` freezes the control loop entirely.

### 3.8 A8 — Dynamic reconfigure ↔ update loop data races (Medium/High)

`reconfigureCallback` (base `fts_base_controller.cpp:1056-1297`; adaptive `fts_adaptive_force_controller.cpp:1256-1411`) writes `gain_`, `time_const_`, `min/max_ft_`, `max_vel_`, `use_controller_`, `modalities_used_`, parametrization state … from the spinner thread. The only "protection" is `running_ = false`, but `update()` continues to execute the full non-command path (incl. `getFTSInput()` normalization using `min/max_ft_`) and the adaptive `update()` reads adaption params unconditionally. These are unsynchronized double/bool/enum writes against a concurrent reader (formally UB; practically torn parameter *sets*). The "controller not started" early-return branch also **applies** some values while claiming read-only (`rotReversed_` `:1113`; `adapt_center_of_rotation_`/`cor_x_` `:1120-1121`). The lock-string machine `protectedToggleControllerRunning` logs `ROS_FATAL` and does nothing on nested/competing locks (`:481-513`) — two overlapping reconfigures (rqt + panel) can leave the controller stopped or double-started; `stopping()` even relies on the WARN path by calling lock(false) twice (`:422-427`).

### 3.9 A9 — `FTSController` double-deletes the (dead) filter chain (Medium)

`chain_ptr_` is deleted in both `~FTSController` (`fts_controller.h:15`) and `~FTSBaseController` (`fts_base_controller.h:187`) → double delete on controller unload/switch of the `fts_controller` variant (heap corruption inside the driver process). The chain is never `configure()`d or `update()`d anywhere — it is dead code — and the config it would load references a **nonexistent plugin class** `robotrainer_modalities/VirtualCounterforceFilter` (`modalities_chain.yaml` vs `modalities_plugin.xml`, which registers only Forces/PathTracking/Areas/Walls filters). The `/modalities/virtual_counterforce` yaml block is likewise read by nothing (the counterforce SCA actually lives in the base controller + `VirtualAreas` `apply_counterforce` publishing `/base/virtual_areas/counterforce/center_dist_percent` — that path is consistent ✅).

### 3.10 A10 — FTS trust chain (Medium, safety posture)

- `getFTSInput()` uses the hardware handle values with **no staleness/timeout check** (`fts_base_controller.cpp:1349-1411`); if the FTS CAN thread stops updating, the last force is integrated indefinitely (the mass-damper will hold a velocity as long as the frozen force persists). Mitigations exist (grip heuristic, NaN guard) but none detect a frozen sensor.
- The grip heuristic itself: any residual offset > `min_ft/3` (= **1 N** for x/y) marks "gripping" and drives the robot (`:1380`). After a bump/lean that shifts the offset (the reason `recalculateFTSOffsets` exists), the robot can self-propel at drift force until someone recalibrates. `rt2_recover` re-enables motors but does **not** trigger offset recalculation or a controller restart — worth an explicit operator step or automation in Phase 2.
- Hardcoded handle name `"ATI_45_Mini"` (`fts_base_controller.cpp:137`) matches `fts.yaml` today; move to a parameter.
- Dead hardcoded service path `/base/fts_adaptive_force_controller/FTSBaseController/set_parameters` (`fts_base_controller.cpp:151`; `untickDynamicReconfigureParam` declared `fts_base_controller.h:498` but never defined/used) — breaks silently if the controller is renamed; remove or implement.

### 3.11 A11 — Adaptive parametrization depends on a disabled subsystem (Medium)

`za_experimental/rt2.launch:23` sets `enable_leg_tracking=false`, but the adaptive controller's `recordFeetDistance`/`adaptX` steps require `/leg_detection/people_msg_stamped` (`fts_adaptive_force_controller.cpp:79`, `leg_tracker_one_person.launch` otherwise started by `robotrainer_bringup/robot/rt2.launch:52-56`). With tracking disabled, "Activate adaptive scale parametrization" (rqt_reconfigure) enters a step that can never complete: `recordBaseFeetDistance` waits for 200 leg samples forever while restricting motion to +x (`:672-716`). No warning is emitted about the missing topic. Guard the reconfigure entry on subscriber liveness, or re-enable the tracker in this config.

### 3.12 A12 — Panel & UX-level defects (Medium)

- Hardcoded default directory `/home/robotrainer/workspace/ros_ws_melodic_robotrainer/...` (`robotrainer_editor_panel.cpp:340`; the portable `homedir` line above it is commented out) — file dialogs break on any other user/machine (e.g. the laptop running the panel against the robot's master).
- All robot interaction via `std::system("rosservice call …")` / `rosparam` (`:362-371, 741-798, 946-998`) — blocks the Qt GUI thread per call (~0.5–1 s each), no error propagation (`/robotrainer_deviation/configure` and `/robotrainer_performance/configure` are absent in this launch set — every "Set Active" pays two failing calls), shell-quoting breaks on scenario names with spaces.
- `dataServiceFetchFileList` removes combobox items with an incrementing index (`:1004-1007`) — clears only every other entry; repeated fetches accumulate duplicates.
- "Load active scenario" (`dataServiceLoadActive` → `dataServiceLoad(ns)` overload `:975-990`) also **clears and replaces the local editor session** as a side effect of what the UI presents as "send file to robot".
- `saveScenarioFile` pollutes the dump with a `scenario: <absolute path>` key (`:768`), which then round-trips into `/robotrainer/scenario/scenario` on load (visible in `4_green_line.yaml`). Harmless but noisy.
- Scenario `4_green_line.yaml` itself: **path-only** (166 points, no forces/walls/sections) — loading it exercises only the display/path pipeline; the force variants (`1_green_line_force_right_40.yaml` etc.) carry `arrow/area/margin` in the exact structure `VirtualForces::configure()` expects ✅.

### 3.13 A13 — Minor/latent (Low)

- `ModalityBase`/`ModalitiesControllerBase` virtual methods declare `bool`/`tf2::Vector3` returns but return nothing (`modality_base.h:22-26`, `modalities_controller_base.h:17-20`), as does `ModalitiesVFController::update` (`modalities_virtual_forces_controller.cpp:35-85`) — formally UB, currently benign because callers ignore the values; fix signatures or return.
- Adaptive `setLEDPhase` switch fall-throughs (`fts_adaptive_force_controller.cpp:1086-1156`): `WAIT_FOR_INPUT` with step `finished` falls into `WALK_FORWARD`; missing `return` at function end (UB) — wrong LED cue in edge phases.
- `unsetForceLimitForModalitie()` sets `setUseLimit(true)` — copy-paste; currently dead code (no callers) (`modalities_virtual_forces_controller.cpp:159-162`).
- `VirtualAreas` reads per-area bool effect params into a map that is never used in `update()` (`virtual_areas.h:283-292`); `current_area_` not reset when a configure yields zero areas (harmless today).
- `sr3.launch` declares `RT2_ip`/`RT2_ip_wired` but never forwards them from `robotrainer_bringup/rt2.launch` (which declares `RT2_ip` and drops it) — the laser IPs actually come from `sr3_on_startup.launch`; the args in this chain are decorative.
- `<arg name=… default=…>` *inside* `<include>` blocks (`za_experimental/rt2.launch:5-23`) — works in roslaunch but is nonstandard; use `value=`.
- `FTSBaseController` is exported as a spawnable controller plugin (`robotrainer_controllers_plugin.xml`) despite "NOT INTENDED FOR STANDALONE USE" (`fts_base_controller.cpp:226`).
- FTS configs are loaded twice (once by `sr3_on_startup.launch:72`, once by `robotrainer_bringup/rt2.launch:31`) — redundant but consistent.
- Data-service `save` sets `<ns>/scenario` *before* dumping (`nodes/data_srv:14`), embedding bookkeeping into every scenario file.
- Panel/`iras` map/AMCL/plugin-XML/perspective files: **all verified present and consistent** — no missing-file breakage found anywhere in the entry chains.

---

## 4. Priority Action Matrix

| # | Priority | Component | Finding | Effort |
|---|----------|-----------|---------|--------|
| S1 | **Critical** (scientific validity) | driver sync ↔ `FTSBaseController::discretizeController` ↔ `/modalities/*` | 50 Hz actual loop vs 200 Hz assumed by every PT1 discretization; `period` ignored | **S** (discretize from `period` / one constant) + **S** verify on robot |
| A1 | **Critical** (crash w/ user contact) | `configure_modalities` ↔ RT `update()` | Modality vectors cleared while iterated; panel triggers at runtime | **M** (stop-controller + swap-under-lock or double-buffer configs) |
| A2 | **Critical** (use-after-free) | `update_kinematics` / `GeomController::update` | `geom_.reset()` under live RT dereference | **M** (same locking pattern as A1) |
| A3 | **Critical** (feature broken) | `PathTracking::configure` ↔ panel/yamls | `max_deviation` int-vs-double → uninitialized corridor; misaligned vectors on skipped sections | **S** |
| A4 | High (silent wrong scenario) | `robotrainer_data_service` | unbound `dir`, ignored return codes, always-success | **S** |
| A5 | High (unintended motion) | `WheelControllerBase` / adaptive return | `angular.z` never reset in `starting()`; unlocked `twist_command_` writes | **S** |
| S2 | High | `FTSBaseController::update` | Backwards-force clamp dead (unit mismatch) | **S** |
| S3 | High | adaptive `reconfigureCallback` | Adaption baseline snapshot inverted → dynamics drift | **S** |
| S4 | High | controller-modalities | 1 N threshold on normalized force → SCAs permanently clamped in `controller_modalities` mode | **S** |
| A6 | High | modalities dyn-reconfigure | Duplicate servers per namespace; rqt reaches only first instance | **M** (share instance or unique ns) |
| A7 | High (RT hygiene) | controller + modalities | Blocking service calls / actionlib / publishers / TF / logging in RT path | **M–L** (staged cleanup) |
| A8 | Medium-High | reconfigure ↔ update | Unsynchronized shared-parameter writes; lock-string machine fragility | **M** (RT-buffer pattern, e.g. `realtime_tools::RealtimeBuffer`) |
| S5 | Medium | forces/areas/path-tracking math | Zero-entry-velocity NaN divisions; NaN workaround drops user velocity | **S** |
| S6 | Medium | `PathTracking` force law | `d0=0.2` hardcoded; `< 7`-point sections unusable; end-of-section jerk | **S** |
| S7 | Medium | `VirtualWalls` | Limiter keyed to last wall, `min_distance_to_wall` dead; debug force over-accumulation | **S** |
| A10 | Medium | FTS chain | No staleness watchdog; 1 N drift ⇒ self-motion; `rt2_recover` lacks recalibration step | **M** |
| A11 | Medium | za_experimental config ↔ adaptive steps | Leg-tracking-dependent parametrization enabled while tracker disabled | **S** |
| A12 | Medium | robotrainer_panel | Hardcoded path, blocking `std::system`, combobox clear bug, side-effecting "load active" | **S–M** |
| A9 | Medium | `FTSController` dtor / chain config | Double delete; dead FilterChain + nonexistent `VirtualCounterforceFilter` | **S** |
| S8 | Medium (documentation) | config vs paper | Linear force→velocity mode, shared wall PT1, grip-gated superposition, commanded-velocity integration, missing amplification | **S** (document) / **M** (implement amplification) |
| A13, S9 | Low | various | UB returns, LED fall-throughs, dead args/params, double param loads, cosmetics | **S** each |

Effort: **S** ≤ ½ day, **M** ≤ 3 days, **L** > 3 days. Priorities weigh (a) user-contact safety, (b) scientific validity of training data, (c) probability of being hit through the audited entry points.

> **Per-finding executable fix plans (work packages for implementation agents): see Section 6.** Each matrix row maps to one WP below with exact steps, risks, and blast radius.

---

## 5. Mitigation Recommendations (Phase 2 plan)

**Step 0 — Measure before touching (half a day).**
On the robot: log `period.toSec()` inside `update()` (or `rostopic hz /base/joint_states`) to settle S1 empirically; record one session bag as a behavioral baseline (admittance step response to a hand push, one force-area traversal) so every later fix can be verified against "before".

**Step 1 — One-line correctness fixes (S-effort batch, no behavior redesign).**
S2 (normalized clamp), A3 (double `max_deviation` + aligned vectors on skip), A5 (`angular.z` reset + mutex), S3 (snapshot on NONE→adaption), A4 (initialize `dir` at node start, check `call()` return codes, propagate failure), S5 (guard `|v_entry| < ε`, fix the NaN fallback to `data_in + prev`), S7 (track nearest active wall), A9 (remove derived-class `delete`, drop dead chain + `VirtualCounterforceFilter` entry), A11 (re-enable leg tracker or gate the reconfigure option). Each is independently testable; land as one reviewed batch.

**Step 2 — Fix the time base (S1).**
Preferred: compute `a1/b1` (and the modality `mds_*`) from the **measured** `period` each cycle (exp per axis at 50 Hz is affordable, or recompute only when `period` deviates > 5 %). Alternative minimum: set a single authoritative `update_rate` param sourced from the sync interval and make all modality configs reference it. Re-run the Step-0 experiments; time constants should now match configuration within tolerance.

**Step 3 — Concurrency architecture (A1, A2, A8; enables safe panel use).**
Adopt one pattern everywhere: non-RT threads never mutate live objects — they build a new config/object and publish it through a `realtime_tools::RealtimeBuffer`/atomic-swap; the RT loop picks it up at cycle boundaries. Apply to: modality scenario data (A1), `geom_` (A2), controller parameter sets from dynamic reconfigure (A8). Replace the lock-string machine with a small explicit state enum guarded by one mutex. This is the largest work item (~2–3 days) and should be its own PR with a stress test (loop `configure_modalities` + reconfigure at 5 Hz while the controller runs in `no_hw_output` mode).

**Step 4 — Controller-modalities scale audit (S4) + decide the mode story.**
Either fix the thresholds to normalized scale and re-tune (`checkHumanLetRobotGo` ≈ `min_ft/max_ft`), or — given `base_modalities` is the paper-validated path — explicitly deprecate `controller_modalities` until its passivity layer is re-validated. Resolve A6 in the same pass by sharing the single plugin instance between base and controller wrappers (or namespacing per instance).

**Step 5 — RT hygiene & safety posture (A7, A10).**
Move FTS offset service calls out of `starting()` (pre-compute before start, or async with a "not ready" state); replace modality debug `ros::Publisher`s with `realtime_tools::RealtimePublisher` behind the existing `debug_` flag; one shared TF buffer for all modalities; throttle/remove RT-path logging. Add an FTS freshness watchdog (timestamp from the sensor thread; zero force + stop on staleness) and fold offset recalibration into the documented `rt2_recover` operator procedure (or auto-trigger on recover when wheels idle).

**Step 6 — Panel/data-service robustness (A12) and paper-alignment documentation (S8).**
Replace `std::system` with proper `ros::ServiceClient`s off the GUI thread (Qt worker) with user-visible error dialogs; parameterize the yaml directory; fix the combobox clearing. Finally, write a short "deviations from ELMAR-2019" note (linear force mode, grip-gated superposition, shared wall dynamics, 50 Hz) into the repo so experiment configs and publications stay honest about what the running system actually computes.

**Regression guard.** After Steps 1–3, re-run the Step-0 bag scenarios plus: load `4_green_line.yaml` and a force scenario via the panel *while gripping* (A1), call `update_kinematics` under load (A2), restart the controller after a joystick rotation (A5), and load a scenario with a deliberately broken filename (A4) — each previously a crash/silent-failure path, each now expected to fail loudly or work.

---

## 6. Work Packages — Per-Finding Fix Plans

One work package (WP) per finding, written so a single agent can execute it in isolation. Template: **Files** (exact touch points) → **Fix steps** (do these, in order) → **Possible problems** (what can go wrong while fixing) → **Scope** (blast radius: what behavior/components/configs the change affects) → **Verify** → **Depends/conflicts** (sequencing against other WPs).

**Shared-file coordination:** WP-S1, S2, A1, A2, A5, A7, A8, A10 all touch `robotrainer_controllers/src/fts_base_controller.cpp` (or its header). Run them **sequentially** (recommended order: A5 → S2 → A3 → A4 → S3 → A9 → S1 → A8 → A2 → A1 → A7 → A10), or assign to one agent per file-cluster. Modality-header WPs (S5, S6, S7, A6) touch `robotrainer_modalities/include/*` and can run in parallel with the controller cluster, but not with each other on the same header.

**Global constraint for every WP:** ROS Melodic / C++11(14), Ubuntu 18.04, Python 2 for scripts. Do not change `.srv`/topic names unless the WP says so. Every WP must keep `catkin build` green for `robotrainer_controllers`, `robotrainer_modalities`, `robotrainer_panel`, `robotrainer_data_service` (hardware-driver packages are not buildable off-robot — do not touch them except where a WP explicitly says so).

---

### WP-S1 — Unify the control-loop time base (fixes S1)
**Priority:** Critical (scientific validity) · **Effort:** S code + on-robot verification
**Files:** `robotrainer_controllers/src/fts_base_controller.cpp` (`discretizeController()` :1018-1025, `update()` :234), `include/robotrainer_controllers/fts_base_controller.h` (`controllerUpdateRate_`), `robotrainer_bringup/configs/rt2_adaptive_force_controller_za_experimental.yaml:11`, `robotrainer_bringup/configs/modalities_za_experimental.yaml` (all `controller_update_rate`), `robotrainer_modalities/include/*.h` (`reconfigureRequest` mds calc: `virtual_forces.h:398`, `virtual_walls.h:316`, `path_tracking.h:413`), `robotrainer_modalities/cfg/*.params` (rate defaults).

**Fix steps:**
1. **Measure first (on robot, mandatory):** add `ROS_INFO_STREAM_THROTTLE(5, "period: " << period.toSec())` in `FTSBaseController::update()` or run `rostopic hz /base/joint_states`. Expected: 20 ms (50 Hz). Abort this WP and re-scope if it is genuinely 200 Hz.
2. In `update()`, keep a `measured_rate_` (filtered `1/period.toSec()`); when it deviates > 5 % from the rate used at last discretization, call `discretizeController()` with the measured rate (do **not** call `exp()` unconditionally every cycle; recompute only on change).
3. Keep the `update_rate` param as the initial value; log one `ROS_WARN` when measured ≠ configured.
4. Modalities cannot see `period` through the `FilterBase::update(in, out)` API. Choose and implement **one**: (a) controller writes the measured rate to a param/latched topic once and modalities read it in `configure()`; or (b) each modality computes `dt` from `ros::Time::now()` deltas internally (jitter-filtered); or (c) minimum fix: set every `controller_update_rate` in the yamls/`.params` to the measured value and add a startup consistency check in the controller. (c) is acceptable for Phase 2; (a) is cleaner.
5. Align all configs: controller yaml `update_rate`, modalities yaml `controller_update_rate` (4 blocks), `VirtualForces.params` default 100 → measured value.
6. **Decision item (with project owner):** matching the rate makes the robot ~4× more responsive than currently felt. Either accept the new (as-configured) dynamics, or multiply all configured time constants by 4 to preserve today's feel. Record the decision in the config comments.

**Possible problems:** external-sync mode (`use_external_sync`) could change the true rate later — that is why step 2 (runtime measurement) is preferred over constant-fixing; per-cycle `exp()` if implemented naively (recompute-on-change avoids it); behavior change surprises users accustomed to the sluggish response — first tests at reduced `max_vel`; old rosbags/studies are not comparable after this change.
**Scope:** transient dynamics of **all three axes** and **all four spatial-action PT1s**; config files in `robotrainer_bringup`; training-data comparability (flag the change date); no topic/service/API changes.
**Verify:** Step-0 baseline bag vs. after — a step force input must now reach 95 % of steady velocity in ≈ 3·T_configured.
**Depends/conflicts:** do after WP-A8 if both are scheduled (A8 moves these params into a struct); otherwise standalone. Conflicts textually with WP-S2/A5 (same file).

---

### WP-S2 — Reactivate the backwards-force clamp (fixes S2)
**Priority:** High · **Effort:** S
**Files:** `robotrainer_controllers/src/fts_base_controller.cpp:252`.

**Fix steps:**
1. Replace the condition/assignment with the normalized threshold: `if (force_input_[0] < -backwardsMaxForceScale_) force_input_[0] = -backwardsMaxForceScale_;`.
2. Grep `update()` for any other comparison of `force_input_`/`force_old_` against `max_ft_`-scaled values (units audit); fix likewise.
3. Add a comment stating `force_input_` is normalized to [−1, 1] at this point.

**Possible problems:** the robot becomes noticeably harder to pull backwards fast (clamp now actually engages at 50 % with the za_experimental yaml) — confirm 0.5/0.7 scales are still the desired training config; none technically.
**Scope:** negative-x direction of every controller deriving from `FTSBaseController` (base, adaptive, integral-compare); no config/API changes.
**Verify:** in `no_hw_output` mode, publish a large backwards force (or replay a bag) and check `debug/force_input_scaled_limited` saturates at −0.5.
**Depends/conflicts:** textual conflict with WP-S1/A5 (same file); otherwise independent.

---

### WP-S3 — Fix the velocity-adaption baseline snapshot (fixes S3)
**Priority:** High · **Effort:** S
**Files:** `robotrainer_controllers/src/fts_adaptive_force_controller.cpp:1328-1346` (`reconfigureCallback`), `:923-925` (`setBaseValues`), header `pre_adaption_base_params_`.

**Fix steps:**
1. Invert the snapshot condition: capture `pre_adaption_base_params_` (gain, time_const, damping, mass) **when the old mode is `NONE` and the new mode is not** (entering adaption).
2. When the new mode is `NONE` (leaving adaption), restore via `discretizeWithNewParameters(pre_adaption_base_params_.time_const, pre_adaption_base_params_.gain)` — restore must run **after** the snapshot logic, using the untouched baseline.
3. Extend `setBaseValues()` (called on `base_reconfigured_flag_`) to also refresh `pre_adaption_base_params_` from the just-applied base parameters, so a base-parameter reconfigure during active adaption updates the baseline.
4. Also restore `max_ft_` via `setBaseValues`-tracked `base_max_ft_` when leaving FORCE_* modes (currently `setMaxFtScale` leaves the last scaled value active).

**Possible problems:** ordering with `apply_base_controller_params` inside the same reconfigure callback — define precedence: base params update the baseline first, then adaption mode switching runs; switching *between* two adaption families (DAMPING→FORCE) must restore dynamics before enabling force scaling.
**Scope:** adaptive controller only; affects anyone toggling `velocity_based_force_adaption_used` in rqt_reconfigure; no config changes.
**Verify:** unit-style test of the callback sequence NONE→DAMPING_LINEAR→NONE→FORCE_LINEAR→NONE asserting `getGain()/getTimeConst()/getMaxFt()` return yaml values at each NONE.
**Depends/conflicts:** if WP-A8 (param-struct refactor) is scheduled, do this first (it is small) and let A8 absorb it.

---

### WP-S4 — Correct the controller-modality force threshold & clamp logic (fixes S4)
**Priority:** High · **Effort:** S–M (incl. re-validation)
**Files:** `robotrainer_modalities/src/modalities_virtual_forces_controller.cpp:104-110` (`checkHumanLetRobotGo`), `:65-76` (branching), `src/modalities_virtual_walls_controller.cpp:83-100`, `robotrainer_modalities/cfg/ModalitiesVFController.params` + `ModalitiesVirtualWallsController.params` (new param).

**Fix steps:**
1. Add a configurable `released_force_threshold` param (default ≈ 0.03, i.e. 3 N at 100 N max) to both controller-modality `.params` files; regenerate configs.
2. Replace `human_input_force.length() < 1.0` with `< released_force_threshold` in both files; comment that the incoming wrench is normalized by the base controller (`fts_base_controller.cpp:919-924`).
3. Re-check the now-reachable `scaleVelFactorBack` branch: verify `forgetting_factor_`/`eps_` defaults give a smooth ramp (log `vel_limit_factor_` at debug level).
4. **Decision item:** if the always-clamp behavior was in fact the intended passivity guarantee, instead keep the clamp but make it explicit (remove the dead branch and the misleading threshold) — document either way.
5. Safety review before subject use: with the threshold fixed, force areas can move the robot while the user holds still — confirm this is wanted in `controller_modalities` mode, or keep `base_modalities` as the validated default (it is the current default).

**Possible problems:** strengthens spatial actions in `controller_modalities` mode — must be re-tested on the robot at low `max_force` first; walls must still be impenetrable after the change (test pushing through a wall slowly and fast).
**Scope:** only `spatial_control_action_type = 2` (controller_modalities) sessions; `base_modalities` path untouched; new dynamic-reconfigure parameter (rqt UI gains one slider).
**Verify:** sim test: constant human force 0.3 normalized inside a force area — virtual velocity must now exceed the human-velocity clamp that previously bounded it.
**Depends/conflicts:** pairs naturally with WP-A6 (same files/instances); do A6 first or together.

---

### WP-S5 — Guard all zero-velocity/degenerate divisions in modalities (fixes S5)
**Priority:** Medium · **Effort:** S
**Files:** `robotrainer_modalities/include/robotrainer_modalities/virtual_forces.h:299-312`, `virtual_areas.h:160-171`, `path_tracking.h:294-317`.

**Fix steps:**
1. `virtual_forces.h` (Eq. 5 projection): before line 304, `if (velocities_on_entry_[i].length2() < 1e-4) { skip compensation for this force; }` (ε = 0.01 m/s, far below the 0.3 m/s reference).
2. `virtual_areas.h` `keep_direction`/`invert_direction`: if `area_entry_velocity_.length2() < 1e-4`, define the fallback — recommended: leave `velocity_out` unchanged for `keep_direction` and only negate (`-velocity_in`) for `invert_direction`; add a `ROS_WARN_THROTTLE(5, …)` naming the area.
3. `path_tracking.h`: guard the intersection when `fabs(denominator) < 1e-9` → reuse the previous cycle's direction/intersection (skip recompute); **fix the NaN fallback** at `:315` to superpose instead of replace: `data_out = data_in;` then add `previous_velocity_linar_` to `data_out.linear` (mirror lines 383-385).
4. Add `ROS_WARN_THROTTLE` on every guard hit so degenerate scenarios are visible in logs.

**Possible problems:** ε too large silently disables compensation at slow elderly walking speeds — keep ε ≤ 0.1·`compensation_reference_velocity`; the areas fallback choice changes behavior for standstill entries (document in the deviations note, WP-S8).
**Scope:** all three modality plugins, both base- and controller-modality paths (shared headers); removes the `ROS_FATAL` NaN-halt failure mode during training; no config changes.
**Verify:** sim scenario with a force area centered on the robot's start pose (zero entry velocity): previously NaN-halt, now clean.
**Depends/conflicts:** same headers as WP-S6/S7 — sequence within the modality cluster.

---

### WP-S6 — Path-tracking force law robustness (fixes S6)
**Priority:** Medium · **Effort:** S
**Files:** `robotrainer_modalities/include/robotrainer_modalities/path_tracking.h:176` (end-of-section), `:323-347` (force law), `robotrainer_modalities/cfg/PathTracking.params` (new `d0` param), optional: `robotrainer_panel/src/robotrainer_editor_section.cpp` (min section length).

**Fix steps:**
1. Promote `d0` (0.2 hardcoded at `:336`) to a `.params` parameter `force_free_corridor` (default 0.2, configurable).
2. Clamp: if `max_deviations_[s] <= d0`, use `d0_eff = 0.5 * max_deviations_[s]` and `ROS_WARN` once per section.
3. Guard the linear-law denominator `> 1e-6`.
4. Replace the end condition `pos_prev_segment_ >= size − 6` with `size − k` where `k = min(5, size/2)` so short sections still track; `ROS_WARN` in `configure()` for sections with < 7 points stating reduced end-margin.
5. Optional (separate commit): on section exit (`:179-183`), keep the PT1 state and let it decay (skip the hard `setZero`) for one branch-controlled release, reducing exit jerk (paper outlook).
6. Optional (panel): enforce a minimum section point count in `RobotrainerEditorSection` at creation.

**Possible problems:** corridor feel changes for tight `max_deviation` scenarios; decay-on-exit means a residual pull for ~3·T after leaving a section — gate it behind a param default-off.
**Scope:** path-tracking modality (both variants); scenario yaml format unchanged; one new dyn-reconfigure param.
**Verify:** unit test of the force law at `distance = {0, d0, max_dev}` for `max_dev ∈ {0.15, 0.5}`; sim run over `0_user_study_green_line.path` sections.
**Depends/conflicts:** touches the same header as WP-A3 (configure) and WP-S5 — coordinate; A3 first (types), then S6.

---

### WP-S7 — Virtual-walls: nearest-wall limiter & debug-force accounting (fixes S7)
**Priority:** Medium · **Effort:** S
**Files:** `robotrainer_modalities/include/robotrainer_modalities/virtual_walls.h:126-171, 215-218, 235-295`.

**Fix steps:**
1. Initialize `min_distance_to_wall = std::numeric_limits<double>::infinity()`; update it (and remember that wall's `effective_radii_[i]`) **inside** the in-range branch (after the `continue`); delete the dead `current_distance_to_wall` per-iteration assignment at `:155`.
2. Drive the limiter block (`:242-261`) with the tracked nearest active wall's distance and radius instead of `current_distance_to_wall`/`min_effective_radius`.
3. Move `last_scaled_virtual_force_` out of the accumulation loop: either assign once after the loop (`last_scaled_virtual_force_ = resulting_force;`) or accumulate the per-wall contribution (`distance_factor * max_force * wall_to_robot/dist`) — pick the latter to keep semantics "sum of wall forces".
4. Remove/throttle the `ROS_INFO("INSIDE")` at `:172` (belongs to WP-A7 but trivial here).

**Possible problems:** the walls-controller integrals consume `getLastscaledForce()` — its magnitude drops after de-duplication, which shifts `humanEndangeredForce()` sensitivity (that function is currently unused in the active branch, but verify before merging); multi-wall behavior changes are the *point* — retest single-wall scenarios for regression.
**Scope:** walls modality + walls-controller diagnostics; no configs.
**Verify:** two-wall sim (robot at 0.3 m from wall A, 5 m from wall B, B last in array): limit factor must derive from A (previously B).
**Depends/conflicts:** same header as WP-S5's walls edits? (S5 does not touch walls) — independent; textual conflict with WP-A7's logging cleanup.

---

### WP-S8 — Document (or implement) the deviations from the paper (fixes S8)
**Priority:** Medium (documentation) · **Effort:** S docs, +M if amplification is implemented
**Files:** new `dependencies_ws/src/robotrainer_control/README_deviations_elmar2019.md` (or repo docs dir); optional: `virtual_areas.h:153-204` (amplification).

**Fix steps:**
1. Write the deviations note covering: (a) `linear_velocity_from_force_damping: true` + `time_const_T: 0.0` replaces Eq. 2 with `v = K·F` for virtual forces in za_experimental; (b) modalities act only while `userIsGripping()`; (c) all walls share one PT1; (d) parametrization distance integrates commanded velocity; (e) area `amplification` unimplemented; (f) actual loop rate vs. paper's implied rate (link WP-S1 result).
2. For each item record: keep-as-is (with rationale) or implement (with WP reference).
3. If implementing amplification: in `virtual_areas.h::update()`, apply `velocity_out *= area_amplification_effects_[j]` (and `rot_out`) when the effect name matches a new `"amplification"` entry; clamp relies on the final `limitValue(max_vel_trans_)` in `WheelControllerBase` — verify that guard covers it; delete the unused `area_effects_` bool-map reads (`:283-292`) in the same pass.

**Possible problems:** amplification > 1 approaches platform velocity limits — test with `max_trans_velocity` guard confirmed; documentation-only items: none.
**Scope:** docs; optionally virtual-areas modality; publications/experiment configs gain an authoritative reference.
**Verify:** review by project owner; amplification sim test if implemented.
**Depends/conflicts:** none (docs); amplification conflicts textually with WP-S5's `virtual_areas.h` edits.

---

### WP-S9 — Minor scientific corrections batch (fixes S9)
**Priority:** Low · **Effort:** S
**Files:** `fts_base_controller.cpp:968-971` (counterforce normalization), `:1553-1555` (`robotIsMovingForward`), `fts_adaptive_force_controller.cpp:893-915` (`scaleBetweenValues`), `:872-883` (`setMaxFtScale`).

**Fix steps:**
1. Counterforce: normalize `staticCounterForce_`/`areaCounterForce_` by plain `value/max_ft_` (skip the `min_ft_` dead-zone branch of `getScaledLimitedFTSInput`) so counterforces < 3 N take effect; small helper `normalizeForceNoDeadzone()`.
2. `robotIsMovingForward()`: rename to `robotIsNotMovingBackwards()` or implement with hysteresis on measured `platform_state_.getVelX()`; update the two call sites (leg-distance baseline selection).
3. `scaleBetweenValues()`: take `scaling_reference` by value (it currently mutates the caller's `velocity_percent`).
4. `setMaxFtScale()`: either rescale `force_old_` by `old_max_ft/new_max_ft` when max force changes, or add a comment accepting the one-cycle transient (decide; the comment is acceptable).

**Possible problems:** (1) changes felt behavior for configs with small counterforces (previously silently zero) — announce; (2) changes which baseline (fwd/bwd) is picked around zero velocity — negligible.
**Scope:** base + adaptive controller internals; no APIs/configs.
**Verify:** unit test of counterforce normalization at 1 N/5 N; compile-time only for (3).
**Depends/conflicts:** same files as the controller cluster — schedule inside it.

---

### WP-A1 — Make scenario (re)configuration safe against the RT loop (fixes A1)
**Priority:** Critical · **Effort:** M
**Files:** `fts_base_controller.cpp:869-871` (`configureModalitiesCallback`), `:876-938` (`applyModalities`), `:655-833` (load/configure), all seven modality plugins' `configure()`/`update()` (`virtual_forces.h`, `virtual_walls.h`, `path_tracking.h`, `virtual_areas.h`, three `modalities_*_controller.cpp`).

**Fix steps:**
1. Pick the pattern: **config-swap under try-lock** (minimal, recommended): each modality gets a `std::mutex config_mutex_`; `configure()` builds *local* vectors first (no member mutation), then locks and swaps them in; `update()` uses `std::unique_lock … std::try_to_lock` — on failure it applies **no modality effect this cycle** (pass-through `data_out = data_in`) and logs at debug.
2. Apply mechanically to all seven plugins (base + controller wrappers; the wrappers mostly delegate — their `configure()` also flips `modalitie_configured_`, guard that with the same lock).
3. In `configureModalitiesCallback`, additionally gate: set `modalities_used_ = none` (via the WP-A8 buffer, or an `std::atomic<modality_type>`) before configuring, restore afterwards — belt-and-braces so half-configured chains are never applied.
4. Stress test (sim, `no_hw_output: true`): loop `rosservice call /base/configure_modalities` at 5 Hz while replaying force input for 10 min under `-fsanitize=thread` (build these two packages locally with TSAN; they have no hardware deps).

**Possible problems:** try-lock pass-through means one cycle without spatial actions during a reload — acceptable (user is mid-scenario-change); header-only template classes → full recompile of dependents; ensure `configure()`'s param reads (slow) stay **outside** the locked region — only the vector swap is locked; deadlock impossible with try-lock but verify no nested locking with WP-A8's structures.
**Scope:** `robotrainer_modalities` package (all plugins) + one controller callback; makes panel "Set Active"/"Load Active" safe during sessions; no message/param format changes.
**Verify:** the stress test above, plus panel round-trip: grip (sim force), Set Active a new scenario, confirm no crash and new areas take effect next cycle.
**Depends/conflicts:** conceptually pairs with WP-A8 (same swap pattern); can land first. Textual conflicts: every modality-header WP.

---

### WP-A2 — Eliminate the `geom_` reset use-after-free (fixes A2)
**Priority:** Critical · **Effort:** M
**Files:** `fts_GeomController.h:45-70` (`GeomController::update/init`), `fts_base_controller.cpp:848-864` (`updateWheelParamsCallback`), `:210-214, 409-414` (RT deref sites); same pattern check in `src/twist_controller.cpp:41` and `src/odometry_controller.cpp:82` callbacks.

**Fix steps:**
1. Change `GeomControllerBase::geom_` from `boost::scoped_ptr` to `std::shared_ptr` with an accompanying `std::mutex geom_swap_mutex_`.
2. `update(wheel_params)` (service thread): construct the new `Controller` fully, then lock and swap the pointer.
3. RT sites: at the top of `FTSBaseController::update()` take a local `std::shared_ptr<Controller> geom = …` under `try_lock` (on failure: reuse last local copy — keep it as a member `geom_rt_`) and use only the local copy throughout the cycle (also pass it to `pos_ctrl_.try_configure(*geom)` and `WheelControllerBase::updateCtrl` — thread the pointer or store `geom_rt_`).
4. Repeat the audit/fix for `WheelController` (twist) and `OdometryController` — both advertise `update_kinematics` and, per the shared base, reset `geom_` the same way.
5. Test: `rosservice call /base/update_kinematics` at 2 Hz for 10 min under TSAN in sim while commanding twists.

**Possible problems:** `shared_ptr` copy in the RT path is lock-free ref-counting (fine); wheel geometry swap mid-motion yields one cycle mixing old state/new kinematics — acceptable (service is meant for standstill; optionally require `!platform_is_moving_` in the callback and return failure otherwise — recommended); `pos_ctrl_.try_configure(*geom_)` currently uses the member directly at `fts_base_controller.cpp:409` — don't miss it.
**Scope:** all three wheel controllers; `/robotrainer_hw/set_state` (rear-wheels helper) and `/base/update_kinematics` become safe; no API changes.
**Verify:** stress test above; kinematics still correct after N swaps (odometry sanity in sim).
**Depends/conflicts:** controller-cluster file conflicts; independent of WP-A1 logically.

---

### WP-A3 — Scenario parameter type compatibility in PathTracking (fixes A3)
**Priority:** Critical (feature-restoring) · **Effort:** S
**Files:** `robotrainer_modalities/include/robotrainer_modalities/path_tracking.h:496-513` (`configure()`).

**Fix steps:**
1. `max_deviation`: declare `double max_deviation = 0.5;` and read with a dual-type helper: try `ros::param::get(path, max_deviation)`; on failure try `int` and cast; on both failing keep the default and `ROS_ERROR` naming the section.
2. `force_distance_function`: try `std::string`; on failure try `int idx` and map through the shared list order (`robotrainer_parameters/cfg/robotrainer_parameters.py:28`: `{0:"trapezoidal",1:"trigonometrical",2:"gaussian"}` — note path tracking itself accepts `linear|trapezoidal|quadratic|trigonometrical`, so map 0→"trapezoidal", 1→"trigonometrical" and warn on 2); on both failing default `"linear"` + warn.
3. **Fix vector alignment:** move the `force_distance_functions_.push_back` and `max_deviations_.push_back` inside the `section.size() >= 2` success branch (`:496-503`), so skipped sections don't shift indices; log skipped section names.
4. Apply the same dual-type read pattern to `VirtualWalls::configure()` (`virtual_walls.h:388-390`) and `VirtualForces::configure()` (`virtual_forces.h:504-506`) `force_distance_function` reads (current panel writes strings, legacy files may not).
5. Test fixture: load `editor_demo_scenario.yaml` (int-typed legacy) and a current panel export; assert parsed values.

**Possible problems:** the legacy int→string mapping is inferred from the current `force_distance_functions` list order — if very old editors used a different order, mapped functions could differ (mitigation: warn loudly on the int path so operators re-save scenarios); none otherwise.
**Scope:** path-tracking (and walls/forces `configure()` hardening); makes the path-tracking SCA functional for panel-authored scenarios — behavior appears where it silently did nothing before (inform trainers!).
**Verify:** the two-fixture test; sim drive along `0_user_study_green_line.path` with a section, confirm corridor force at 0.3/0.5 m deviation.
**Depends/conflicts:** same header as WP-S5/S6 — do A3 first.

---

### WP-A4 — Honest data service (fixes A4)
**Priority:** High · **Effort:** S
**Files:** `robotrainer_data_service/nodes/data_srv` (whole file, ~60 lines).

**Fix steps:**
1. Initialize the yaml directory once in `robotrainer_data_service_server()` (module scope), same computation as in `get_file_list`; keep `get_file_list` re-using it.
2. Replace `call([...])` with checked execution: `ret = call([...]); ok = (ret == 0)`; `return [ok]`. Better: use the `rosparam` Python API (`rosparam.load_file` + `rosparam.upload_params`, `rosparam.dump_params`) inside `try/except`, returning `[False]` with `rospy.logerr` detail on failure.
3. `load_from_file`: verify `os.path.isfile(file_name)` first; return `[False]` if missing.
4. `save_to_file`: only `rospy.set_param(ns + "/scenario", name)` **after** a successful dump (or drop this side effect entirely — see WP-A12 step 5; coordinate).
5. Keep service names/types unchanged (`/save`, `/load`, `/get_list`, `DataOperation`, `FileList`) — the panel depends on them.
6. Tests: call `/load` with a nonexistent relative name (expect `success: False`), `/save` before any `/get_list` (expect success), round-trip a scenario and diff the yaml.

**Possible problems:** panel currently ignores results via `std::system` — failures become visible only after WP-A12; `rosparam` Python API vs subprocess differences in namespace handling (`/` prefix) — keep the existing `"/" + req.nameSpace` convention; Python 2 (Melodic) — no f-strings.
**Scope:** data service node only; scenario save/load reliability for panel and any scripts.
**Verify:** the three tests above.
**Depends/conflicts:** WP-A12 consumes the honest results; coordinate the `scenario` key decision with WP-A12 step 5.

---

### WP-A5 — Twist command reset & synchronization (fixes A5)
**Priority:** High · **Effort:** S
**Files:** `robotrainer_controllers/include/robotrainer_controllers/fts_wheelControllerBase.h:59-68, 127-138`, `fts_base_controller.cpp:355-390`, `fts_adaptive_force_controller.cpp:421-474`.

**Fix steps:**
1. `starting()`: zero `linear.x`, `linear.y`, **`angular.z`** (fix the `angular.x` typo) — keep it under `twist_mutex_`.
2. Replace raw `twist_command_` with `realtime_tools::RealtimeBuffer<geometry_msgs::Twist>`: callback `writeFromNonRT`, RT reader `readFromNonRT` — this removes both the unlocked-callback race and the blocking `scoped_lock` in `update()` (`fts_base_controller.cpp:356`).
3. Adaptive autonomous return (`returnRobotAutonomouslyToStartPosition`): build a complete `Twist` locally (all six fields, others zero) and write it through the same buffer; on completion write an explicit zero twist **before** `setUseTwistInput(false)`.
4. Optional: read the subscribe topic from a param (default `/base/twist_controller/command`) instead of the hardcoded literal (`:47`).

**Possible problems:** `RealtimeBuffer` keeps last value — the timeout logic (`updateCtrl` `:75-79`) still guards staleness via `target_.stamp`; but note `use_twist_input_` path sets `set_new_commands = true` every cycle with the buffered twist — verify the 0.5–1 s timeout still zeroes on publisher silence (it does: `target_.stamp` refreshes each cycle, so **add** a message-age check: store receipt time in the buffer payload and zero if older than `timeout_`). This age check is the one behavioral addition — without it, buffered mode never times out.
**Scope:** all `WheelControllerBase` controllers (fts, adaptive, twist); joystick / velocity-smoother path; parametrization auto-return.
**Verify:** sim: publish `angular.z = 0.3` once, stop/start the controller, enable twist input → robot must not rotate; publisher-silence test with the age check.
**Depends/conflicts:** controller-cluster files; the buffer pattern is the same as WP-A8 — reuse.

---

### WP-A6 — De-duplicate modality dynamic-reconfigure servers (fixes A6)
**Priority:** High · **Effort:** M
**Files:** `fts_base_controller.cpp:655-749` (instance loading), `modalities_virtual_forces_controller.cpp:9-32` (+walls/pathtracking ctors), `robotrainer_modalities/include/*.h` (ctor ns), `robotrainer_parameters/yamls/rqt_params/*.yaml` (preset paths, if option B).

**Fix steps (choose Option A, recommended):**
1. **Option A — share the instance:** add `void setModalityInstance(boost::shared_ptr<ModalityBase<geometry_msgs::Twist>>)` to the three controller-modality classes; in `loadControllerModalityInstances()` pass the already-created base instances (`force_modality_ptr_`, `walls_modality_ptr_`, `pathtracking_modality_ptr_`) instead of letting each wrapper `createInstance` its own; delete the internal loaders in the wrapper ctors.
2. Make wrapper `configure()` idempotent w.r.t. the shared instance (it already just forwards; ensure it is not called twice per `configureModalities()` — it will be: base list + controller list both configure the same object now; that is harmless but log-noisy → configure controller wrappers without re-configuring the shared instance, or accept double-configure).
3. Result: exactly one `dynamic_reconfigure::Server` per `/modalities/*` namespace; rqt sliders now affect the instance used by **both** modes.
4. Remove the dead `params_callback_first_time_` block (`virtual_forces.h:376-380`, `modality_base.h:48`) or implement it properly (set `true` default and push yaml→config on first callback) — pick removal unless GUI-sync is wanted.
5. Verify with `rosservice list | grep set_parameters` in sim: one service per modality ns; check `rosout` has no "already advertised" errors at controller load.

**Possible problems:** shared instance means `getLastscaledForce()` state is shared between modes (they never run simultaneously — `modalities_used_` selects one; safe); Option B (separate namespaces) would instead require moving server construction out of constructors into an `init(ns)` and updating `rqt_params` preset yamls — bigger churn, only choose if modes must be tunable independently.
**Scope:** modality loading in the controller + three wrapper classes; rqt_reconfigure entry point behavior (sliders now reach the active instance in controller_modalities mode — combined with WP-S4 this changes that mode's strength!).
**Verify:** load controller in sim, reconfigure `/modalities/virtual_forces/max_force`, confirm effect in both `spatial_control_action_type` 1 and 2.
**Depends/conflicts:** do together with or before WP-S4; conflicts with WP-A1 edits in the same loader functions.

---

### WP-A7 — Realtime hygiene in controller & modalities (fixes A7)
**Priority:** High · **Effort:** M–L (split into 3 PRs)
**Files:** PR1: `fts_base_controller.cpp:99-106` (LED wait), `:186-198` + `:549-562` (offset call in `starting()`), `fts_adaptive_force_controller.cpp:225, 323, 460` (service calls in update path). PR2: modality debug publishers (`virtual_forces.h:120-127, 223, 338-349, 363-364`; `virtual_walls.h:93-97, 123, 296-300`; `virtual_areas.h:85-89, 211-214`; `path_tracking.h:95-101, 147, 388-397`). PR3: shared TF buffer (`modality_base.h:54-59`, `modalities_controller_base.h:36-40`).

**Fix steps:**
1. **PR1 — no blocking calls in RT/lifecycle:** drop `led_ac_->waitForServer(2s)` from `init()` (check `isServerConnected()` at send time); introduce an async calibration state: `starting()` sets `calibration_pending_` and returns; a non-RT thread (`std::thread` member or `ros::Timer` on the controller's queue) performs `unsafeRecalculateFTSOffsets()` and only then `protectedToggleControllerRunning(true)`; same flag pattern replaces the three adaptive in-update `recalculateFTSOffsets()` calls (request → async → resume step machine on completion).
2. **PR2 — RT-safe debug output:** gate every modality debug publisher behind a `debug` param (default false) and convert to `realtime_tools::RealtimePublisher` `trylock` pattern (or drop publishes to a 10 Hz counter); delete/`ROS_DEBUG_THROTTLE` the per-cycle `ROS_INFO`s (`virtual_walls.h:172`, VF limit prints `modalities_virtual_forces_controller.cpp:290/292` etc.).
3. **PR3 — one TF listener:** replace per-instance `tf2_ros::Buffer/TransformListener` with a process-wide shared instance (static function-local `shared_ptr` getter used by both base classes); keep `lookupTransform` with `ros::Time(0)` (non-blocking on cached data).
4. Instrument before/after: log max `update()` duration over 10 min (simple `ros::WallTime` high-water mark under `debug_`).

**Possible problems:** async calibration delays "controller ready" by the service duration — LED phase must show it (`STOPPED`→`UNLOCKED` on completion; wire into existing phases); a stuck FTS service now leaves the controller cleanly stopped instead of freezing the loop (improvement, but operators must recognize the state via diagnostics); static shared TF buffer lifetime vs. plugin unload — use `shared_ptr` and let last owner release.
**Scope:** controller start/parametrization timing; modality CPU/log footprint inside the canopen driver process; no external interfaces.
**Verify:** loop-duration high-water mark drops; controller load no longer stalls 2 s when LED node absent; parametrization still completes in sim.
**Depends/conflicts:** heavy textual overlap with WP-A1/S5/S7 in modality headers and with the controller cluster — schedule last in each cluster.

---

### WP-A8 — Atomic parameter application & lock-state machine (fixes A8, S3-adjacent)
**Priority:** Medium-High · **Effort:** M
**Files:** `fts_base_controller.{h,cpp}` (reconfigureCallback :1056-1297, protectedToggleControllerRunning :481-513, getters/setters :617-647, update() reads), `fts_adaptive_force_controller.cpp:1256-1411`.

**Fix steps:**
1. Define `struct BaseControllerParams { gain, time_const, min/max_ft, max_vel, use_controller, backwards/reversed scales, yReversed, rotReversed, a1, b1, … }` (everything `update()` reads that reconfigure writes) and hold it in `realtime_tools::RealtimeBuffer<BaseControllerParams>`.
2. `reconfigureCallback` builds a complete new struct (starting from the current one), discretizes (`a1/b1` become struct members), and `writeFromNonRT` — delete the piecewise member writes and the stop/start bracketing for pure parameter changes (keep stop/start only for actions: reset, offsets, wheel orient).
3. `update()` does one `readFromNonRT()` at cycle start into a local reference; replace member reads.
4. Same pattern for the adaptive config block (`force_adaption_params_`, `damping_adaption_params_`, `tanh`, parametrization flags → the flags that trigger *state machine* transitions stay as `std::atomic<bool>` requests consumed by `update()`).
5. Replace the locking-string machine: `enum class RunState {RUNNING, STOPPED}` + `std::string owner` under one `std::mutex`, methods `bool lockStop(owner)` / `bool unlockStart(owner)` returning failure instead of `ROS_FATAL`; make `stopping()` a single call; convert `running_/can_be_running_/userIsGripping_` to `std::atomic<bool>` and delete the shared-mutex spin-wait getters (`:625-647, 1542-1548`).
6. Fix the "not started" reconfigure branch to be strictly read-only (delete writes at `:1113, 1120-1121`).
7. TSAN build + the WP-A1 stress loop with concurrent rqt reconfigures.

**Possible problems:** large mechanical diff (~50 read sites) — do it as one focused PR with no behavior tweaks mixed in; parameters now apply atomically at cycle boundaries (behavioral nuance: a reconfigure mid-cycle no longer half-applies — strictly better, but retune scripts that relied on immediate `set_parameters` readback may see one-cycle lag); interplay with WP-S1 (a1/b1 recompute moves into the struct write + rate-change path) and WP-S3 (baseline logic) — absorb both if sequenced after.
**Scope:** base + adaptive controllers; rqt_reconfigure robustness; groundwork for A1/A2/A5 patterns; no configs/APIs.
**Verify:** TSAN clean; parameter sweep via `dynamic_reconfigure` client script asserting applied == requested.
**Depends/conflicts:** the biggest controller-cluster item — schedule after the S-size controller WPs.

---

### WP-A9 — Remove double delete & dead filter-chain remnants (fixes A9)
**Priority:** Medium · **Effort:** S
**Files:** `fts_controller.h:15`, `fts_base_controller.h:187, 323`, `src/integral_compare_ctrl.cpp:201` (audit), `robotrainer_bringup/robot/rt2.launch:27` (chain yaml load), `robotrainer_bringup/configs/modalities_chain.yaml`, `modalities_za_experimental.yaml` (`virtual_counterforce` block).

**Fix steps:**
1. Delete `delete chain_ptr_;` from `~FTSController` (and from `IntegralCompareCtrl` if it deletes the *base's* pointer — it deletes at `:201`; verify whether that is its own member; if it is the inherited one, remove).
2. In the base: remove the `chain_ptr_` member entirely plus the destructor delete (it is never configured/updated), or if kept for future use, make it `std::unique_ptr` and configure it — **removal recommended**.
3. Remove the `modalities_chain.yaml` `<rosparam>` load from `robotrainer_bringup/robot/rt2.launch:27` and archive/delete the yaml (its `VirtualCounterforceFilter` entry references a class that does not exist in `modalities_plugin.xml`).
4. Decide the `/modalities/virtual_counterforce` yaml block: nothing reads it (the counterforce SCA lives in the controller + `VirtualAreas`) → delete the block and add a comment in the controller yaml pointing at `spatial_control_action/area_counterforce/*` as the real knobs.
5. Grep both workspaces for `modalities_chain_config` consumers before deleting (audit found none — re-confirm at implementation time).

**Possible problems:** none functional (all dead code); if an out-of-tree tool loads the chain config it would break — hence step 5.
**Scope:** controllers package cleanup + bringup config; slightly faster param load; removes a controller-unload crash for the `fts_controller` variant.
**Verify:** build + controller load/unload cycle in sim (`controller_manager unload`) without heap errors (run under ASAN once).
**Depends/conflicts:** trivial; controller-cluster textual conflicts only.

---

### WP-A10 — FTS trust chain: staleness, drift, recover procedure (fixes A10)
**Priority:** Medium (safety posture) · **Effort:** M
**Files:** `fts_base_controller.cpp:1349-1411` (`getFTSInput`), `:137` (handle name), `:151` (dead client), `fts_base_controller.h:498` (dead decl), `force_torque_sensor/src/force_torque_sensor_handle.cpp` + header (timestamp exposure — minimal, coordinated change), ops docs.

**Fix steps:**
1. **Staleness watchdog:** expose a data timestamp from `ForceTorqueSensorHandle` (it already runs its own pull thread; add `ros::Time last_update` alongside the force/torque arrays, or reuse an existing sequence counter if present). In `getFTSInput()`: if `time − last_update > max(5·period, 100 ms)` → force zero, `setLEDPhase(STOPPED)`, diagnostic ERROR "FTS stale".
2. **Drift guard:** in `update()`/diagnostics: if `!userIsGripping()` continuously for > 30 s while `platform_is_moving_ == false` and any `|raw| > min_ft/3`, emit a diagnostic WARN "FTS offset drift — recalibrate"; optional auto-recalc only when additionally wheels idle for the whole window (param-gated, default off).
3. **Recover procedure:** document in the ops README: after `rt2_recover`, release handles and run offset recalculation (rqt `recalculate_FTS_offsets` tick or service); optionally subscribe to the driver's recover event (canopen publishes diagnostics) to prompt via LED.
4. Read the FTS handle name from param (`hw_fts_` lookup `:137`) defaulting to `"ATI_45_Mini"`, sourced from `fts.yaml`'s `FTS/fts_name`.
5. Delete the dead `dysrv_callback_service_` (`:151`) and `untickDynamicReconfigureParam` declaration (`fts_base_controller.h:498`).

**Possible problems:** touching `force_torque_sensor` crosses the interface-only boundary agreed for third-party-ish packages — it *is* in `dependencies_ws` and IPR-owned, but keep the change minimal (one timestamp member + getter) and PR it separately; false-positive staleness during CAN hiccups → threshold generous (100 ms at 50 Hz loop); auto-recalc misfiring while a user hovers hands over sensors — default off, require full idle window.
**Scope:** base controller safety layer; `force_torque_sensor` handle API (+1 getter); operator documentation; LED/diagnostics behavior on sensor failure changes from "keeps last force" to "stops".
**Verify:** sim: freeze the FTS values (kill the pull thread / mock) → robot zeroes velocity within 100 ms and shows STOPPED; drift test with injected 2 N offset.
**Depends/conflicts:** controller cluster; independent of others.

---

### WP-A11 — Gate leg-tracking-dependent parametrization (fixes A11)
**Priority:** Medium · **Effort:** S
**Files:** `ros_ws/src/za_experimental/launch/rt2.launch:23` (config decision) and/or `fts_adaptive_force_controller.cpp:1395-1401` (reconfigure guard), `:672-716` (step timeout).

**Fix steps:**
1. **Decision item:** if adaptive-scale parametrization is part of current experiments → set `enable_leg_tracking` to `true` in `za_experimental/rt2.launch` (tracker + `/base_laser_back/scan` must be live). Otherwise leave disabled and rely on steps 2–3.
2. Reconfigure guard: in the `activate_adaptive_scale_parametrization` branch, check `sub_legtrack_.getNumPublishers() > 0`; if zero → `ROS_ERROR("Leg tracker not running — cannot start adaptive-scale parametrization")`, reset `config.activate_adaptive_scale_parametrization/parameterization_activated = false`, do not enter the step.
3. Runtime timeout: in `recordFeetDistance`/`parametrizeAdaptForceX`, if no `getLegTrackUpdated()` for 10 s while the step is active → abort to `finished`, LED `STOPPED`-style cue, `ROS_ERROR`.

**Possible problems:** `getNumPublishers` is a liveness proxy, not data flow — the timeout (step 3) covers the gap; enabling the tracker adds CPU on the robot PC and depends on the back laser being configured (it is, in `sr3_on_startup.launch`).
**Scope:** za_experimental launch config and/or adaptive controller UX; prevents a mid-session deadlock where the robot only allows +x motion indefinitely.
**Verify:** with tracker absent, toggling the rqt checkbox must refuse with a clear log; with tracker running, the step proceeds.
**Depends/conflicts:** none.

---

### WP-A12 — Panel robustness & UX (fixes A12)
**Priority:** Medium · **Effort:** S–M
**Files:** `robotrainer_panel/src/robotrainer_editor_panel.cpp` (:340, :345-373, :728-799, :924-1000, :1002-1035), `include/robotrainer_panel/robotrainer_editor_panel.h` (members for clients/threads).

**Fix steps:**
1. Default yaml dir: `ros::package::getPath("robotrainer_data_service") + "/yamls/"` with a param override; delete the hardcoded `/home/robotrainer/...` literal (`:340`).
2. Replace `std::system("rosservice call …")` with typed `ros::ServiceClient`s (`std_srvs::Empty` for `/base/configure_modalities`, `robotrainer_data_service::DataOperation` for `/save`+`/load`); run them via `QtConcurrent::run` (or a worker QThread), disable the triggering button while pending, and show a `QMessageBox::critical` on `call() == false || !success`. Keep `/robotrainer_deviation/configure` + `/robotrainer_performance/configure` calls but make them non-fatal (warn-only) since those nodes are optional in this launch set.
3. Keep `rosparam load/dump` for the temp-file transfer but check the `std::system` return code (or use `XmlRpc`); on failure abort the Set Active flow before calling configure.
4. `dataServiceFetchFileList`: replace the index loop with `service_file_combobox->clear();` (`:1004-1007`).
5. Untangle "Load active": split `dataServiceLoad(ns)` so sending a file to the robot does not silently `clearSession()` + overwrite the local editor (`:975-990`) — either prompt the user or make it two buttons.
6. Stop persisting the absolute filename into the scenario ns before dump (`:768`): dump first, set the bookkeeping param afterwards, or store it under a non-dumped ns (coordinate with WP-A4 step 4 so exactly one side owns the `scenario` key).
7. Until WP-A1 lands: before Set Active, warn if `/base/fts_adaptive_force_controller` reports running (query `controller_manager/list_controllers`) — remove the warning after A1.

**Possible problems:** async service calls change GUI flow (buttons disabled during calls) — small UX retraining; `ros::package::getPath` returns the *panel machine's* package path — correct, since `data_srv` also runs there (same launch); message-box spam if robot offline — batch errors per action.
**Scope:** panel package only; trainer-facing reliability (failures become visible); no robot-side changes.
**Verify:** run panel against a sim master: Set Active with controller running (expect warning), load nonexistent file (expect error dialog), fetch list twice (no duplicates).
**Depends/conflicts:** consumes WP-A4 results; step 7 interacts with WP-A1 timing.

---

### WP-A13 — Low-priority cleanup batch (fixes A13 + leftovers)
**Priority:** Low · **Effort:** S (mechanical)
**Files/steps, one commit each:**
1. **UB returns:** add proper `return` values in `modality_base.h:22-26`, `modalities_controller_base.h:17-20`, `ModalitiesVFController::update` (`modalities_virtual_forces_controller.cpp:85` — `return true;`), walls/pathtracking controller `update`s, adaptive `setLEDPhase` end (`fts_adaptive_force_controller.cpp:1158`); then add `-Werror=return-type` to `robotrainer_modalities` and `robotrainer_controllers` `CMakeLists.txt` so it cannot regress. *Risk:* none. *Scope:* recompile only.
2. **LED switch fall-throughs:** add missing `break`/`return` in `setLEDPhase` (`:1086-1156` — `WAIT_FOR_INPUT` inner-switch default, `ROB0T_IN_AUTOMATIC_MOVEMENT` inner-switch default). *Risk:* LED cues change to the intended ones. *Scope:* LED feedback only.
3. **`unsetForceLimitForModalitie`:** currently dead + wrong (`setUseLimit(true)`, `modalities_virtual_forces_controller.cpp:159-162`) — delete both set/unset helpers or fix to `false` if kept. *Scope:* none (unused).
4. **`virtual_areas.h`:** delete the unused `area_effects_` bool-map reads (`:283-292`) unless WP-S8 implements amplification (then they go there); reset `current_area_ = -1` unconditionally in `configure()`. *Scope:* none functional.
5. **Launch arg hygiene (za_experimental + bringup):** change `<arg … default=…>` to `value=` inside the `<include>` in `za_experimental/launch/rt2.launch`; remove the unused `RT2_ip`/`RT2_ip_wired` args from `robotrainer_bringup/robot/rt2.launch` + `sr3.launch` (they are consumed only by `sr3_on_startup.launch`, which has its own). *Risk:* none — verify with `roslaunch --args`/dry inspection that no child declares them as required. *Scope:* launch files only.
6. **Plugin export:** remove the `FTSBaseController` `<class>` entry from `robotrainer_controllers_plugin.xml` (marked "not intended for standalone use"; no config in either workspace spawns it — re-grep first). *Risk:* out-of-tree configs spawning it would break (announce in changelog). *Scope:* plugin registry.
7. **Double FTS param load** (`fts.launch` included by both `sr3_on_startup.launch:72` and `robotrainer_bringup/rt2.launch:31`): keep both (startup covers standalone flows, rt2 refreshes per-session) — **explicit WONTFIX**, add a comment in each include stating the duplication is intentional.
8. **`getFTSInput` future-timestamp hack** (`fts_base_controller.cpp:1402`: `timeSinceReleasing_ = time + 1.0s`): replace with a `bool grip_recently_` + explicit timestamp to make `getTimeSinceReleasingRobot` semantics honest. *Scope:* passive-behavior/adaptive timing paths — verify `use_passive_behavior_ctrlr` behavior unchanged (it is `false` in za_experimental).

---

### Suggested execution waves (for parallel agents)

| Wave | Work packages | Rationale |
|---|---|---|
| 0 | WP-S1 step 1 only (rate measurement) | One fact gates S1's design; needs robot access. |
| 1 (parallel) | WP-A3, WP-A4, WP-A11, WP-A13.1/.5 | Independent files, no cross-conflicts, immediately user-visible value. |
| 2 (parallel) | Controller cluster agent: A5 → S2 → S3 → S9 → A9; Modality cluster agent: S5 → S6 → S7; Panel agent: A12 (after A4) | Clustered by file ownership to avoid merge conflicts. |
| 3 (sequential) | WP-S1 (full) → WP-A8 | Rate fix, then the param-buffer refactor absorbs it cleanly. |
| 4 (sequential) | WP-A1 → WP-A2 → WP-A7 | Concurrency architecture, largest risk, needs the TSAN harness built in Wave 2/3. |
| 5 | WP-S4 + WP-A6 (together) · WP-A10 · WP-S8 docs | Mode-level decisions with owner sign-off; safety posture; documentation. |

Every wave ends by re-running the **Regression guard** list (end of Section 5) plus the WP-specific verifications. Any WP that changes robot *feel* (S1, S2, S4, S9.1, A6) must be flagged to trainers before the next subject session.
