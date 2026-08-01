# Project Deep Dive — Complete Design & Engineering Notes

This is the exhaustive, engineering-level companion to
[`DOCUMENTATION.md`](DOCUMENTATION.md) (the concise ~4-page grading
deliverable). Where that document summarizes, this one explains
**everything**: every algorithm, every parameter and why it has the value
it has, every bug found during development and how it was diagnosed and
fixed, and the reasoning behind every non-obvious design decision. It is
written for a future maintainer (or grader who wants to go deep), not for
a 5-minute skim.

## Table of Contents

1. [Goal & Hard Constraints](#1-goal--hard-constraints)
2. [System Architecture](#2-system-architecture)
3. [Perception](#3-perception)
4. [Planning](#4-planning)
5. [Control](#5-control)
6. [Bug History](#6-bug-history-chronological)
7. [Performance Investigation](#7-performance-investigation)
8. [Testing & Validation Methodology](#8-testing--validation-methodology)
9. [Infrastructure](#9-infrastructure)
10. [Known Limitations, Honestly](#10-known-limitations-honestly)
11. [Topic / Service Glossary](#11-topic--service-glossary)

---

## 1. Goal & Hard Constraints

Drive a fixed ~785 m urban loop (Unity simulation, ROS 2 Jazzy) as fast as
possible without leaving the road, colliding with other vehicles, or
running a red light, while handling two scripted Decision-Making events
(Event I: an NPC merges in after intersection 1; Event II: an NPC crosses
the road ahead then brakes hard, "while returning"). Grading rewards
functionality (50p), code/architecture quality (30p), a written summary
(20p), and a no-semantic-camera bonus (10p); it also imposes a **-30p**
penalty if the code doesn't build or behave as documented.

Standing constraints that shaped every design decision below:

- **Never modify the course-provided `simulation` or `dummy_controller`
  packages.** Everything must be built strictly on top, via our own
  packages and launch/config files.
- **Single launch file.** No "open five terminals" workflows.
- **No semantic camera** (traded off against 10 bonus points, chosen
  deliberately — see Section 3.2).
- Ubuntu 24.04 / ROS 2 Jazzy, verified via an actual pristine Docker
  build (Section 8.3), not just careful reading.

## 2. System Architecture

```
Unity sim  <--TCP/UDP-->  simulation (course-provided bridge, unmodified)
                                |
                                v
                    depth image, RGB image, pose, twist
                                |
        +-----------------------+------------------------+
        v                                                 v
   perception                                          planning
   (obstacle_guard_node,                          (route_planner_node)
    traffic_light_node,
    octomap_server)
        |                                                 |
        +-----------------------+------------------------+
                                v
                            control
                       (pure_pursuit_node)
                                |
                                v
                       /car_command -> simulation -> Unity
```

Seven ROS 2 packages total: `simulation` and `dummy_controller`
(course-provided), `project_interfaces` (our custom messages),
`perception`, `planning`, `control` (our nodes), `bringup` (single launch
file + static TF tree + RViz config). See
[`DOCUMENTATION.md`](DOCUMENTATION.md) Section 3 for the rendered ROS
graph.

The whole system runs as a single-threaded executor per node, driven by
message callbacks — there is no shared control-loop timer across nodes;
`pure_pursuit_node`'s `Control()` runs on every `/OurCar/CoM/pose` (or
`/twist`, whichever arrives) callback, effectively ~50 Hz, matching the
`/car_command` publish rate the task expects.

## 3. Perception

### 3.1 `obstacle_guard_node` — corridor obstacle monitoring

**Purpose**: convert the raw depth-camera point cloud into a small number
of scalar "how far to the nearest obstacle" numbers the controller can act
on every tick, without the controller needing to know anything about
sensors or geometry.

**Pipeline** (`OnCloud`, called on every `/perception/pcl/points` message,
itself produced by the external `depth_image_proc::PointCloudXyzNode`
component from the raw depth image):

1. Look up the `world <- <cloud frame>` transform via `tf2` (cached
   buffer/listener). If unavailable, publish all four corridor distances
   as `+inf` (fail-open — a transform hiccup must never look like "clear
   to detour", so infinite/unknown reads as "no information", and the
   *controller's own* freshness checks are what actually gate driving
   decisions, not this node pretending to know).
2. Build the search window: starting from `nearest_` (see `UpdateNearest`
   below), walk forward along the trajectory's arc-length table until
   `look_ahead` (30 m) is covered. This window is what all the geometry
   below is checked against — points far outside it are cheap to reject
   without any per-point trig.
3. For every point in the cloud (with `stride` decimation, `1/stride²` of
   pixels considered — stride 3 means every 3rd row and column, worked
   out empirically, see Section 7):
   - Reject points with non-finite or too-close-range z (camera-space
     depth `< min_range`, 0.2 m).
   - Reject a specific "own hood" region: points closer than 2.6 m,
     clearly below the camera (`cy > 0.2`), and within ±1.0 m of the
     camera's lateral center. **This exists because of a real early bug**:
     without the lateral bound, the height/range filter alone discarded
     *every* low point across the *full image width* inside 2.6 m —
     including the car's own hood reflection *and* anything genuinely
     alongside the car at the exact distance the detour maneuver needs to
     see (2 m gaps while threading past a parked obstacle). Adding the
     lateral bound fixed this without reopening the original problem
     (seeing the hood as an obstacle).
   - Transform the point to world frame (explicit quaternion-to-rotation-
     matrix math, applied per point — a full TF `doTransform` call per
     point was too slow at cloud resolution, so the transform is computed
     once per frame and applied as a 3×3 matrix multiply per point).
   - Reject points outside world-height `[z_min, z_max] = [0.35, 3.0]` m.
     **`z_min = 0.35` is not an arbitrary round number** — it was
     calibrated after finding that a naive lower z_min classified a
     raised sidewalk fringe at a bend (s≈540) as an obstacle: that fringe
     measured 0.25–0.27 m in world z, flat pavement, not a real hazard.
     0.35 clears it with margin. This same threshold is *why* the
     OctoMap cross-check exists (Section 3.3) — anything between the
     ground and 0.35 m, including a genuine curb, is invisible to this
     path by design.
   - Classify the point against the closest trajectory segment in the
     window, coarse-to-fine: first scan every 4th segment (`kCoarse`) to
     bracket the true minimum, then refine ±4 segments around the best
     coarse hit. **This two-pass search also exists because of a real
     performance bug**: scanning all ~23 segments in the window for
     *every point* of the cloud pushed per-frame processing past 1.5 s
     once `look_ahead` was experimentally raised to 45 m, which starved
     the controller of fresh corridor data and made it crawl the whole
     route. Coarse-to-fine cut that back down without giving up
     correctness (the distance-to-segment profile along a path is smooth,
     so a coarse bracket reliably contains the true minimum).
   - From the closest segment, compute signed lateral offset (`lat`,
     positive = left) and arc-length position (`a`, relative to
     `nearest_`).
   - Update four running minima: `tight_b` (|lat| < 1.1 m, any point),
     `main_b` (|lat| < 1.4 m), `left_b` / `right_b` (|lat ∓ 2.9| < 1.4 m —
     the overtake corridors, offset by the same 2.9 m the detour maneuver
     itself shifts by).
4. Publish all four as `std_msgs/Float32` (`+inf` where nothing qualified).
5. **New**: also call `PublishMapTight(nearest_, end)` — the OctoMap
   cross-check, detailed in Section 3.3.

`UpdateNearest()` is a small windowed search (`nearest_ ± 25/50` points)
around the *previous* nearest-point index, not a global search every
frame. This detail matters a lot: the 785 m route is an out-and-back loop
that passes close to itself in world-space at several points (the outbound
and return legs run near-parallel ~50 m apart in places). A naive global
nearest-point search **flip-flops** between the two legs whenever they're
similarly close — this exact failure was hit and fixed independently
*twice* in this codebase (once here, once in `route_planner_node`'s goal
selection, Section 4) with the identical pattern: lock on with one
heading-filtered global search, then stay locked with a cheap local
window from then on.

Four corridors, one publish call, four different consumers downstream:

| Topic | Half-width | Consumer / purpose |
|---|---|---|
| `/perception/tight_distance` | 1.1 m | ACC following distance (primary safety authority), emergency-stop input, detour-verify gate |
| `/perception/obstacle_distance` | 1.4 m | wide "is anything nearby" cap on cruise speed |
| `/perception/overtake_left_distance` | 1.4 m, +2.9 m offset | is the left detour lane clear |
| `/perception/overtake_right_distance` | 1.4 m, −2.9 m offset | is the right detour lane clear |

### 3.2 `traffic_light_node` — HSV blob detection

**Why not the semantic camera**: it's an explicit 10-point bonus criterion
to solve the whole task without it, and HSV color segmentation on the RGB
camera is sufficient for this simulator's signal heads (bright, saturated
lamp colors against a much less saturated background).

**Algorithm** (`OnImage`):

1. Convert to HSV, threshold three ranges: red (two ranges, since red
   wraps around hue 0/180 in OpenCV's HSV: `[0,140,190]-[6,255,255]` and
   `[174,140,190]-[179,255,255]`), green
   (`[45,120,225]-[70,255,255]`). High saturation/value thresholds (140+,
   190+/225+) are deliberate — they reject dim, desaturated real-world-ish
   clutter (brick facades, dull signage) and keep only genuinely
   lit-lamp-bright pixels.
2. Connected-component analysis on each mask, filtered to blobs that
   *look like a lit circular lamp*: bounding box roughly square (aspect
   ratio close to 1), well-filled (contour area close to bounding-box
   area — rejects hollow rings), area within `[min_area_px, ...]` (4 px
   minimum — the overpass-hung lamps are genuinely tiny in-frame at
   detection range). This shape filter is what rejects the red ring of
   no-U-turn signs (hollow → low fill ratio), red shop banners
   (elongated → bad aspect ratio), and large green highway boards (way
   too large).
3. Only RED and GREEN are ever reported — **amber is deliberately
   skipped**. The signal casing itself is amber-painted metal in this
   simulator's asset, and cannot be reliably separated from a genuinely
   lit amber lamp by color alone; misreading the casing as amber would be
   worse than not detecting amber at all. The controller's traffic-light
   state machine treats "stopped, no fresh green seen yet" as "keep
   waiting", which is legally/behaviorally equivalent to correctly
   detecting amber for the purposes of this task (the car never runs the
   light early either way).
4. **The red↔green misdetection bug (fixed, validated run36)**: the
   original decision logic had two asymmetric special cases without a
   proper dominance/position floor:
   - A "center-facing green demotes an edge red" exception, with no area
     floor — a tiny, barely-visible green sliver near center could
     override a large, clearly-facing red blob.
   - A "green must be 2× the red's area to win" exception, with no
     *position* gate — a huge but off-to-the-side green blob (e.g. a
     distant cross-street's green, visible at a shallow angle) could
     outvote a smaller, dead-center-facing red.

   Fixed by adding both missing gates symmetrically: the center-preference
   exception now also requires the green blob be reasonably large
   relative to the red (`>= ` its area, not just present) **and**
   off-center enough on the red side (`off_center(red) > 0.25`) before
   green is allowed to win; the area-dominance exception now also
   requires the green be facing (`off_center(green) <= kFacingOffCenter`,
   0.20) before its size alone can win. `off_center` is normalized
   horizontal distance from the ROI's center column, `cx_min`/`cx_max`
   (0.15/0.85) additionally bound which raw detections are considered
   "facing" at all before either exception ever runs. Verified via run36:
   TL1/TL2/TL3 all single, clean red→green transitions across the run,
   zero flicker, zero premature green reads.
5. Publishes `project_interfaces/TrafficLight` (RED/GREEN/UNKNOWN) at
   camera frame rate. The controller (Section 5.4) does its own temporal
   debouncing (2-of-3 sliding window) on top of this raw per-frame signal
   — this node itself does not smooth over time, by design (keeps its
   job to "what does this one frame look like", leaving temporal
   judgment calls to the consumer that actually knows the driving state).

### 3.3 OctoMap Cross-Check — supplementary curb detection

Added late in development (2026-08-01) in response to an explicit
decision to close a real gap rather than just document it: `octomap_server`
(course-dependency-available, external package) was wired into the
launch file from the start (point cloud → 3D occupancy map → RViz
visualization) but nothing ever *consumed* it — `obstacle_guard_node`
worked directly off the raw point cloud instead, so the map was purely
decorative.

**Why bother**: Section 3.1's `z_min = 0.35` filter is *specifically*
tuned to ignore anything below that height as road-surface noise — which
means a genuine curb (measured 0.25–0.27 m at one bend) is structurally
invisible to the live per-frame corridor check, by the live check's own
design. An accumulated map can fill exactly that gap without needing to
change `z_min` itself (which is load-bearing for suppressing road noise
everywhere else on the route).

**Mechanism**:

1. `octomap_server`'s launch parameters were changed to project
   `/perception/octomap/projected_map` (a `nav_msgs/OccupancyGrid`) with
   `occupancy_min_z = 0.08`, `occupancy_max_z = 0.35` — exactly the band
   the live check ignores, and nothing more (deliberately *not* the same
   0.35–3.0 m band the live check already covers, to avoid the map signal
   being mostly redundant with the live one).
2. **`filter_ground_plane` was tried and reverted.** `octomap_server`'s
   RANSAC ground-plane filter needs a `base_frame_id` (`base_footprint`
   by default) to transform into before fitting a plane — this project's
   TF tree uses `OurCar/...` frame names, so enabling it just logged
   `Transform error for ground plane filter... quitting callback` on
   *every single cloud frame*, without actually filtering anything.
   Confirmed via `ros2 param get` and by watching the octree's own node
   count keep climbing across those error lines that the point-cloud
   insertion itself was unaffected — it was cosmetic log spam, not a
   silently-broken pipeline, but worth removing regardless. The 0.08–0.35
   m height band alone measured sufficient on its own (a sanity check
   early in a run found only 4 of 13,920 projected cells occupied).
3. `obstacle_guard_node` gained a new subscription to that grid and a new
   method, `PublishMapTight(lo, hi)`, called at the end of every `OnCloud`
   tick with the *same* `[nearest_, end)` trajectory window already
   computed for the live check:
   - Computes the world-space bounding box of that window (padded by the
     tight half-width), converts to grid row/column bounds, and only
     iterates cells inside that box — cost is bounded by the local
     window, not by how large the accumulated map has grown.
   - For each *occupied* cell (`data >= 50`; unknown `-1` and free `0-49`
     are both treated as "not occupied" — deliberately, since treating
     *unknown* as occupied would veto detours through any
     never-yet-observed cell, which is extremely common just ahead of the
     vehicle, and would defeat the whole "must never force extra
     waiting" design goal), reconstructs its world (x, y) as
     `origin + (col+0.5, row+0.5) * resolution` — **no rotation applied**,
     which is only valid because `octomap_server` never sets
     `origin.orientation` (leaves it as the default identity quaternion);
     confirmed against the actual installed `octomap_server` source, not
     assumed.
   - Classifies each occupied cell against the same trajectory-segment
     geometry as the live point classification (same math, no
     coarse-to-fine needed here since only sparse *occupied* cells reach
     this inner loop after ground/speckle filtering).
   - Publishes the nearest hit's arc distance (relative to the *same*
     `nearest_` reference the live `tight_distance` uses, so both are
     directly comparable against the same horizon downstream) to
     `/perception/map_tight_distance`, or `+inf` if none.
4. `pure_pursuit_node`'s `kVerify` state (Section 5.5) ANDs this into its
   detour-clear check as a **strictly additive** veto — full mechanism,
   including two safety bugs found by adversarial review and their fixes,
   is in Section 5.7 (kept there rather than duplicated, since it's really
   a control-side design property).

## 4. Planning

### 4.1 `route_planner_node`

**Base route**: loaded once from a recorded waypoint CSV
(`planning/data/waypoints.csv`), lane-snapped, smoothed
(`smooth_iterations`, a simple 3-pass averaging filter), and converted
into a curvature-limited speed profile: `v_max` (9.5 m/s) on straights,
capped by `sqrt(a_lat_max / |kappa|)` in curves (comfort lateral
acceleration 1.5 m/s², `a_lat_max`), with `sharp_kappa` (0.07 /m)
marking curves sharp enough to need special handling, and the whole
profile then forward/backward-swept against `a_accel`/`a_decel` (2.5 /
3.0 m/s²) so the *commanded* speed profile is itself physically
achievable, not just curvature-legal.

**Lane offset**: `lane_offset_zones` is a per-stretch table (not one flat
value) — a flat 2.9 m offset that was safe on a proven-wide boulevard
wedged the car onto a curb on a narrower street (curbs sit ~0.25 m tall,
below the corridor guard's `z_min = 0.35`, so no live sensor can veto a
bad static offset there — only driven evidence, discovered by actually
hitting the problem, could). `lane_offset_scale` is a global multiplier
on top (a `0.0` A/B-testing knob exposed as a launch argument, drives the
raw recorded centerline unchanged).

**Detour re-splicing** (`OnDetour`, triggered by `/planning/detour` from
`control`): given `[s0, s1, offset, ramp_in, ramp_out]` in true arc
meters, re-splices a laterally-offset segment into the base route between
those arc positions with the given ramp lengths, republishes the full
trajectory. A zero-window request (`[0,0,0,0,0]`) restores the
unmodified base route. This node owns *no* judgment about whether a
detour is safe — that's entirely `control`'s job (Section 5.5); this node
just executes whatever window it's told to.

**Goal selection** (`SelectGoal`, new): the task sheet's Fig. 4 supplies
10 fixed candidate goal poses. `GoalArcs()` projects each onto the base
route's arc length once at startup (`RebuildBaseArc()` builds a cumulative
arc-length table first). Every `/OurCar/CoM/pose` message,
`SelectGoal(px, py, yaw)` finds the nearest goal arc still ahead of the
car's current position and publishes it (on change only, latched) to
`/planning/current_goal`.

**A real oscillation bug was found and fixed here during testing**: the
first implementation did a naive full-scan nearest-point search on every
pose message (no windowing). Because this is the same out-and-back route
geometry described in Section 3.1, the naive search flip-flopped wildly
between two arc positions on opposite legs of the loop (observed
oscillating between s≈642 and s≈263 in one run). Fixed by copying the
*exact* pattern already validated in `obstacle_guard_node`'s
`UpdateNearest`/`pure_pursuit_node`'s equivalent: one heading-filtered
global search to lock on initially (`goal_search_global_` flag), then a
windowed local search (`last_nearest_ ± 25/+50`) from then on. Verified
via run38: goal selection advances monotonically
(s=72→193→263→376→483→504→563→642→669) matching real vehicle progress,
with zero regression to detour/traffic-light behavior.

## 5. Control

`pure_pursuit_node` is the largest and most complex node — roughly 390
lines in `Control()` plus ~150 in `RunObstacleStateMachine()` (extracted
from `Control()` in a later refactor purely for readability, no behavior
change) plus supporting helpers. Every tick (driven by pose/twist
arrival, ~50 Hz):

### 5.1 Pure-pursuit steering

Standard pure-pursuit geometry: lookahead distance
`Ld = lookahead_min + lookahead_gain * v` (3.0 m + 0.5 s of current
speed — `lookahead_min` was raised from an initial 2.4 m after measuring
±0.5 m low-speed tracking weave in run logs at the lower value), find the
trajectory point at that arc distance ahead, compute the classic
pure-pursuit curvature `2*sin(alpha)/Ld`, convert to a steering command
clamped to `max_steer_rad` (0.50 rad, mapped to the `[-1,1]` command
range). `wheelbase` (2.63 m) is taken directly from the task sheet's
vehicle drawing (Fig. 3), not measured/tuned.

Two additional steering caps exist for specific failure modes found in
testing: `maneuver_steer_cap` (0.65, below `max_steer_rad`'s effective
range) avoids a full-lock stall while threading a detour at low speed,
and `standstill_steer_cap` (0.35) keeps the initial launch-from-rest
steering command closer to straight (a large initial pure-pursuit
correction from a cold start could otherwise fishtail before the car has
enough speed for the geometry to behave sensibly).

### 5.2 Throttle / brake law

Base law: `throttle = kp_speed * (v_des - v)` (P-control, gain 0.30) plus
a **feed-forward table**, `throttle_ff_v/t`:
`(0,0), (1.8,0.21), (5.9,0.62), (6.5,0.68), (8.0,0.83), (9.5,0.98)`,
linearly interpolated. **This table exists because of a measured, real
control-law deficiency**: the pure P-law alone parks the car
0.7–2 m/s *below* every commanded speed (e.g. `v_des = 2.5` measured
1.84 actual in logs) — a classic proportional-control steady-state error
against speed-dependent drag/friction the sim itself imposes, worsening
at higher speed. The feed-forward table was built by *reading the
deficit off real run logs* at several speeds and fitting
`T_hold(v) ≈ 0.03 + 0.10*v`, then encoding it as a lookup table rather
than a closed-form formula (easier to re-calibrate a specific breakpoint
without touching the shape everywhere else).

With the steady-state deficit closed, the car rides *at* `v_des`, so
ordinary sensor noise around that point was firing the brake
unnecessarily (brake law has its own base threshold, `kp_brake`=0.60).
Fixed with a **deadband + hysteresis**: inside `brake_deadband` (0.40
m/s) below `v_des`, the car coasts (throttle = 0, not brake, not more
throttle) rather than actively correcting; `brake_release` (0.15 m/s)
gives the exit a different threshold than the entry to avoid limit-cycle
chatter at the boundary. **A real bug was found and fixed in an earlier
iteration of this deadband**: feeding the feed-forward term *inside* the
deadband too (rather than pure coast) held speed artificially constant
even as an approach cap was actively falling, producing a 4 Hz
throttle/brake limit cycle that visibly ate 15 m of an obstacle's
approach distance in one run (nearly causing a stop inside the
minimum detour gap — a real near-livelock). Fixed by making the deadband
a genuine coast (zero throttle command, not feed-forward-held) — the
deadband is bypassed entirely for stop/fail-safe curves via an explicit
`stop_curve` flag, since those need active braking regardless of the
normal-driving deadband logic.

**Brake lag compensation**: the simulator's own brake response has
measured dead time, `brake_lag` (0.3 s). The ACC stopping curve
(Section 5.3) shifts its trigger distance earlier by `speed * brake_lag`
so a fast approach still actually stops at the intended standoff distance
rather than overshooting by the dead-time's worth of travel.

### 5.3 ACC-style gap control

Two layers, deliberately overlapping so the tighter one is always the
active safety authority regardless of what the looser one is doing:

1. **Wide approach cap** (`approach_speed`/`approach_zone`,
   `approach_cap_near/far/vmax`): inside 40 m of *anything* in the wide
   corridor, cruise speed is capped, ramping from 2.5 m/s at 14 m up to
   6.5 m/s at 38 m. **This ramp itself replaced an earlier flat 2.5 m/s
   cap that starved the whole route** — the wide corridor's 1.4 m
   half-width catches roadside furniture (guardrails, mast-arm feet,
   building facades) within 30 m ahead almost continuously in built-up
   sections (measured 60–90% of route time with *some* finite wide-corridor
   reading), so a flat conservative cap meant driving most of the route at
   crawl speed even with nothing actually blocking the lane. The distance-
   scaled ramp reserves genuine caution for genuinely close readings while
   letting the car speed back up as soon as whatever triggered it is
   farther away. `approach_cap_vmax` (6.5, not higher) was chosen
   specifically so the tight-corridor ACC stopping curve (next point)
   retains positive deceleration margin at every point below the far
   anchor — an earlier attempt at 8.0 m/s left *zero* margin for the ACC
   stop (`v²/(2·d) ≈ a_obstacle` exactly at the standoff distance) and
   the car once measurably ate 7 m into the supposed-safe 13 m standoff
   before stopping.
2. **Tight-corridor comfort stop curve**: `sqrt(2 * a_obstacle * (d - obstacle_standoff))`
   (comfort deceleration 1.5 m/s², standoff 13 m) — this is the *real*
   safety-relevant ACC law, always active regardless of the wide cap,
   and it's what determines the actual following/stopping speed against
   a real, close obstacle.

### 5.4 Traffic-light state machine

Per-junction stop-line arc positions are **not** hardcoded constants —
they're stored as world-frame points (`traffic_stop_xy`) and re-projected
onto the route's arc length every time a trajectory is (re)published.
This exists because of a real, painful early bug: hardcoded arc
constants silently pointed at the wrong physical location any time the
route geometry changed even slightly (a lane-offset tweak, a taper
adjustment) — the root cause of what the project's own history calls "the
original TL1 six-run mystery" (multiple runs of confusing
stops-in-the-wrong-place before the arc-vs-world-position bug was
identified).

Each junction also has its own `light_stop_short` distance (not one flat
value) — the overpass-hung signal heads at TL2–TL4 are only visible from
a narrow ~10–25 m window in front of them; stopping exactly at the
official waiting line puts the head almost directly overhead, outside
the camera's field of view (confirmed by capturing a live frame from a
stopped wait and finding empty sky where the lamp should be). Stopping
short enough keeps the lamp in view for the whole wait, so a green onset
is actually *seen* and releases at the true phase rather than always
falling back to the blind timer. 6 m sufficed at TL1 (mast-arm mounted,
visible from the line itself, actually needs less margin) but TL2 needed
more (a 6 m short-stop there still went phase-blind).

**Red-latch anchor bug (found by an 89-agent adversarial review,
confirmed 3/3, fixed)**: the red-hold latch and caution-ramp gate were
originally keyed off distance to the *stop anchor* (which itself sits
6–12 m before the stop line). Once that gap went negative, a fresh red
could never latch even though the physical light was often still visible
and still red 10–20+ m short of the actual junction — the car would
accelerate straight through a visibly red light. Fixed by keying both
gates off distance to the *physical stop line* itself instead
(`to_line = line_s - s_now`), so a red is stoppable all the way up to the
line, not just up to the anchor.

**Blind-release timing — TL4, honestly explained**: TL4's geometry means
the signal head is *only* detectable right at the stop line — there is no
safe short-stop distance that both keeps it in view and gives useful
warning, unlike TL1–TL3. The car therefore sometimes has to release from
a red stop without ever having seen a confirmed-fresh green, using a
timer (`light_blind_release`, 22 s — the junction's own measured red-
phase duration) instead. **This has a hard mathematical limitation that
no timer-only fix can eliminate**: the red phase (22 s) is longer than
the following green+amber window (14 s). For *any* fixed-delay release
timer with zero live visibility into the actual signal phase, there
exists some arrival-time offset within the red phase for which the release
lands *inside the next red phase* — worst case, `R - (C - R) = 22 - 14 =
8` seconds of unavoidable overlap, a straightforward consequence of the
phase arithmetic, not a bug that can be patched away.

**What actually *was* fixed here (a real, scoped bug, not the
fundamental limitation above)**: the "currently blind" freshness check
was originally anchored to `max(last_red_seen, last_green_seen)` — a
live-updating timestamp that a single *late, transient* phase glimpse
deep into a red wait could push forward, delaying the effective start of
the release countdown and increasing the odds of landing in the next red
phase entirely avoidably. Fixed by decoupling "currently blind" (now
gated on a short, fixed freshness window, `light_stale` = 1.0 s) from
"how long have we actually been stopped" (anchored to `red_stop_onset_`,
set once and monotonic, immune to being pushed later by a stray vote).
This narrows the *avoidable* risk without pretending to solve the
*unavoidable* geometric one — documented as such in the code, not
oversold.

**Other fixes bundled into the same red-latch/queue work**: the stall
watchdog now holds the brake (`stall_hold`, 3 s) in *every* controller
mode, not just normal cruising — it previously braked for a single 20 ms
tick then let full throttle resume even mid-detour/mid-verify, which
ground the tires against an invisible curb; `obs_moving_until_` (the
"this looked like it was still moving a moment ago" guard) is now
refreshed every tick during a red wait so the queue-guard settle window
survives the *whole* wait instead of expiring partway through and letting
the car detour around a car stopped ahead of it the instant the light
turns green (a genuinely dangerous move at a signalized junction).

**Approach behavior without a confirmed green** (`light_caution_zone`,
`light_caution_speed`, `light_caution_accel`, `light_prox_cap`): rather
than a hard speed cap stepping in abruptly at the caution zone boundary
(which was itself a full-brake event from 6+ m/s in an earlier version),
approach speed is a graded ramp,
`sqrt(2 * light_caution_accel * (d - light_stop_short))`, floored at 2.2
m/s and ceilinged at 4.0 m/s — the ceiling specifically keeps speed low
enough that the 2-of-3 phase debounce (Section 3.2's consumer-side
smoothing) stays ahead of the stop curve when the phase is still unknown;
a confirmed green lifts the cap entirely.

### 5.5 Obstacle detour state machine

Four states: `kNormal → kVerify → kDetour → kNormal` (loop), with a
`kHold` fallback whenever nothing safe is currently possible.
`RunObstacleStateMachine()` (extracted from `Control()` in a later
readability refactor, no behavior change — see Section 6) returns `true`
if it already published this tick's command via an early-return branch
(kVerify's not-fresh wait, kHold's wait branches), signaling `Control()`
to stop right there rather than falling through to the normal driving
law.

**kNormal — two independent triggers for "this might be a parked
obstacle"**:

1. **Proactive**: while something sits inside `approach_zone` in the wide
   corridor, its *world* arc position (`s_now + lateral-corrected
   distance`) is tracked over a rolling 2.5+ second window
   (`obs_track_`). A **static** obstacle's world position stays constant
   as the car approaches; a **moving** one's drifts. If the tracked
   position varies by ≥1.5 m over the window, it's classified moving and
   remembered (`obs_moving_until_ = now + traffic_settle`, 10 s) — side-road
   traffic that pulls in and pauses briefly before continuing gets
   followed via ordinary ACC, not swerved around. Only once classified
   static, not currently in a settle window, not queued at a light
   (`queue_at_light`, Section 5.4's queue guard), and within the near
   approach band does a detour actually get requested.
2. **Reactive fallback**: if the car ends up stopped behind something the
   proactive path didn't (or couldn't, e.g. a corner refusal) divert
   around, a separate timer (`static_wait`, 8 s) confirms it's genuinely
   parked (the gap must stay essentially constant, not just "stopped for
   a moment") before requesting a detour or, if none is safe, entering
   `kHold`.

**`EngageDetour`** — the actual decision to commit to a detour request:
requires *fresh* data on all four corridors atomically (`< 1.0 s` old —
the guard publishes them together, so one freshness bar covers all),
requires enough gap (`detour_min_gap`, 8.5 m — below this the car would
otherwise livelock since it never reverses, so a short 6 m ramp at crawl
speed is deliberately preferred over waiting forever; `kVerify` still
gates whatever path results), refuses to overtake through a corner
(`corner_heading_deg`, 35° net heading change in the relevant window — an
offset path through a bend risks crossing an inner curb island invisible
to the corridor sensor's height band), and picks a side: **right first**,
specifically because corridor clearance alone cannot see curbs (same
`z_min` limitation as everywhere else) — "more lateral room" is not proof
of a *drivable* pass, and the left/north squeeze past the known static
prop-car at s=317 reads clear on the sensor yet wedges the car on a curb
in practice (measured directly). Passing on the right (the driver's own
side of a wrong-way-parked obstacle) is also simply the correct traffic
convention; left is the fallback only if the right corridor itself reads
blocked.

**kVerify** — confirm the shifted path is actually clear before
committing: waits for a corridor measurement taken strictly *after* the
replan (`tight_stamp_ > state_since_ && waited > 0.6`, up to a 3 s cap
before proceeding on whatever's available regardless), then requires the
tight corridor be clear out to `min(28, detour_end - s_now)` — **and**,
new, the supplementary OctoMap check (Section 5.7). If blocked, tries the
other side once (`tried_other_side_`); if both sides are blocked, gives
up into `kHold`.

**kDetour** — actively threading the shifted path: never restores the
base route mid-thread even if the tight corridor temporarily reads clear
(the car may currently be alongside the obstacle, where "clear ahead"
doesn't mean "safe to cut back in yet") — if blocked, the tight-corridor
ACC below simply holds the car in place; waiting is always legal here.
Exits back to `kNormal` once `s_now > detour_end`.

**kHold** — nothing safe is currently possible: brakes and waits
(`hold_retry`, 5 s) before handing the decision back to `kNormal`, which
owns the re-confirmation logic; `kHold` itself never independently
requests a new detour.

### 5.6 Emergency stop (Event II)

Independent of and *ahead of* every state machine above, every tick:
`a_required = v² / (2 * d_eff)` from the raw, current tight-corridor
distance and speed (`d_eff` additionally subtracts `speed * age` of the
reading to account for how far the car has moved since that
measurement was taken). If `a_required > emergency_decel` (4.0 m/s² —
deliberately above the comfort ACC curve's `a_obstacle`, 1.5, so ordinary
approaches — already slowed by the wide-corridor cap well before gaps get
small — never trip it), commands full brake immediately and returns,
bypassing every other law and state machine for that tick. This is the
direct implementation of the task's Event II requirement ("a vehicle
crosses ahead, then brakes hard — detect the hazard and perform an
emergency stop").

**Empirically confirmed to fire the underlying scenario, not just coded
and hoped for**: instrumented two ways — a throttled "Obstacle entered
corridor" log (rising-edge only, since the ordinary ACC-follow logic
adjusts speed silently by design with zero logging otherwise) and a
small `grab_close_frames.py` script that saved an RGB frame whenever
`/perception/tight_distance` dropped below 20 m. Two independent close-
range clusters showed up per run, not one: the already-known static
prop-car at s≈317 (validated as a sanity check — matches the very next
detour-request log line exactly), and a second, visually distinct vehicle
at s≈531 on the return leg, background changing across a ~25 s window
(confirming it's actively driving through, not a static prop), speed
dipping to 0.5 m/s without ever needing a full stop — the ACC-follow
logic alone handled the closing rate gently enough that trial. Confirms
Event II *does* occur and is survivable, though only one successful trial
was captured — a grader's run could hit a faster-closing encounter that
genuinely needs the dedicated emergency branch, which is exactly why it
exists as a backstop rather than being removed once ACC-follow proved
sufficient once.

### 5.7 OctoMap cross-check integration (bounded-trust design)

The mechanism for producing `/perception/map_tight_distance` is described
in Section 3.3; this section covers how `kVerify` *uses* it and — this is
the important part — how it's kept from ever being able to make things
*worse*.

**The base integration**: `kVerify`'s clear condition gets one more
factor ANDed in — a fresh (`map_tight_stamp_ > state_since_`) map hit
inside the same horizon the live check uses can additionally veto a
detour the live sensor alone would have approved; a stale or absent
reading is always "no objection" (`map_clear = true`).

**Two real bugs, found by an independent adversarial code-review agent
(not self-caught), fixed before shipping**:

1. **Staleness was measured on the wrong clock.** The first version
   tracked only "have we ever received a map message" (`map_received_`,
   a one-shot latch) — `octomap_server` has **no** `respawn=True` (unlike
   the two Unity bridge nodes, which explicitly do), so if it ever died
   or hung, `obstacle_guard_node` would keep republishing a verdict
   against a *frozen* map snapshot forever, and *every* downstream
   staleness check — which only look at *our own* publish timestamp, not
   the underlying map's actual recency — would keep seeing it as
   "fresh". Combined with no ground-plane filtering and no map decay,
   this created a real path to a **permanent, un-clearable detour veto**:
   `kVerify` blocks both sides → `kHold` → `kNormal` → `EngageDetour` →
   `kVerify` → blocked again by the same frozen/stale cell, forever. This
   directly contradicted the explicit design requirement that a
   stale/absent map reading must never force extra waiting. **Fixed** by
   checking `map_.header.stamp` (the scan's *own* timestamp, which
   `octomap_server` genuinely sets — confirmed against its actual
   installed source, not assumed) against a `map_stale_after` threshold
   (3.0 s) inside `PublishMapTight` itself: a dead/hung node now degrades
   this signal to "no data" within a few seconds instead of never.
2. **Even with a live, genuinely fresh map, a single systematic error
   could still seed a persistent false positive** (range-dependent depth
   noise, road camber, vehicle pitch during hard braking shifting where
   "ground" appears in world Z across a spatially-coherent patch of
   points — something `filter_speckles`, which only removes *isolated
   single-voxel* nodes, would not catch), and if the car never
   re-observes that exact spot from a clearing angle again, it would
   never get cleared by the octree's own ray-cast update either. Assessed
   by the reviewer as "plausible but unproven" (not observed in any
   actual test), but still a real gap versus the stated "never causes an
   infinite hang" requirement. **Fixed** with a second, independent bound:
   `map_only_hold_cap` (25 s) — a pure wall-clock timer,
   `map_only_block_since_`, set the first tick the map (and *only* the
   map — the live sensor itself says clear) is blocking, cleared the
   instant that stops being true. Once a map-only block has persisted
   past the cap, the system falls back to the live-sensor-only judgement
   that was the sole check before this feature existed. Deliberately
   implemented as a simple duration timer with no cross-state-machine
   bookkeeping (rather than, say, counting verify attempts, which would
   need to correctly distinguish "same obstacle retried after a kHold
   cycle" from "genuinely different obstacle" — real complexity, real
   risk of a *new* bug, for marginal benefit over the simpler timer) —
   a deliberate scope-minimization choice made under real deadline
   pressure (3 days out at the time).

Both bounds compose: a dead/hung `octomap_server` is caught by (1) within
`map_stale_after` seconds; a live-but-tainted map is caught by (2) within
`map_only_hold_cap` seconds. Together they guarantee the supplementary
check can only ever add a **bounded** delay, never an unbounded hang —
verified by two full-route regression runs after the fixes (run43, run44:
both 785 m routes, `Goal reached`, zero collisions/stalls, identical
detour/traffic-light behavior to every prior baseline run). Honestly:
the map veto itself never actually fired (fresh + finite + within
horizon) in either regression run — this route's one known detour
doesn't happen to sit at a curb the live sensor misses, so the feature's
"catches something the live sensor would've missed" value is
code-reviewed and safety-bounded, not empirically demonstrated the way
Event II was.

## 6. Bug History (chronological)

A consolidated timeline of every non-trivial bug found and fixed across
the project, each with root cause and how it was validated fixed. (Design
decisions that were never actually "broken" — like the coarse-to-fine
search or the windowed nearest-point pattern — are covered in-line in
Sections 3–5 instead of repeated here.)

1. **TL1 six-run mystery** — hardcoded arc-length stop-line constants
   silently pointed at the wrong physical location after route geometry
   changes. *Fixed*: stop lines stored as world-frame points, re-projected
   onto arc length on every trajectory publish.
2. **Red-latch anchor bug** — red-hold/caution gates keyed off distance to
   the stop *anchor* (6–12 m before the line) rather than the line
   itself; once that gap went negative, a fresh red could never latch
   even with the light still visibly red. Found by an 89-agent
   adversarial review, confirmed 3/3. *Fixed*: gates keyed off distance to
   the physical stop line.
3. **Stall-and-grind** — the stall watchdog braked for one 20 ms tick then
   let full throttle resume even mid-detour/mid-verify, grinding against
   an invisible curb. *Fixed*: brake hold (`stall_hold`, 3 s) applies in
   every controller mode.
4. **Queue-guard expiry** — `obs_moving_until_` wasn't refreshed during a
   red wait, so the "this looked like traffic, don't detour" guard could
   expire mid-wait and the car would swerve around a car stopped ahead of
   it the instant the light turned green. *Fixed*: refreshed every tick
   during the wait.
5. **Lane-offset curb wedge** — a flat 2.9 m lane offset (safe on a wide
   boulevard) wedged the car on a curb on a narrower street; the curb sat
   below the corridor guard's height floor so nothing could sensor-veto
   it. *Fixed*: per-stretch `lane_offset_zones` table instead of one flat
   value (found by driving it, not by inspection — the sensor
   structurally couldn't have caught this).
6. **Own-hood blind spot** — the corridor guard's close-range/low-height
   filter, without a lateral bound, discarded every low point across the
   full image width inside 2.6 m, including genuine obstacles at exactly
   the 2 m gaps the detour maneuver threads through. *Fixed*: added a
   lateral bound (`|cx| < 1.0`) so only the car's own hood region is
   excluded.
7. **Look-ahead performance regression** — raising `look_ahead` from 30 to
   45 m (attempting a higher cruise speed) pushed per-frame corridor
   classification past 1.5 s, starving the controller of fresh data and
   causing sustained "stale corridor" braking. *Root-caused* (not just
   reverted blind) by measuring corridor publish rate across several
   look-ahead/stride combinations and finding it *invariant* — the true
   bottleneck was Unity's own single-core render/physics loop pinned at
   99.5% CPU, not this code's per-point cost (see Section 7). *Fixed*:
   reverted to the validated 30 m/stride-3 baseline; separately added
   coarse-to-fine segment search as a genuine, non-regressing performance
   improvement.
8. **Brake-deadband limit cycle** — feeding the throttle feed-forward term
   *inside* the coast deadband held speed artificially constant against a
   falling approach cap, producing a 4 Hz throttle/brake limit cycle that
   ate 15 m of an obstacle's approach distance in one run (nearly a
   livelock). *Fixed*: deadband is a genuine zero-throttle coast, bypassed
   entirely for stop/fail-safe curves via an explicit flag.
9. **Traffic-light red↔green misdetection** — asymmetric override
   exceptions without proper area/position floors let a small, off-center
   green demote a large facing red, or a large off-to-the-side green
   outvote a small facing red. *Fixed*: added the missing dominance and
   facing gates symmetrically to both exceptions. Verified run36.
10. **Goal-selection oscillation** — a naive full-scan nearest-point
    search on every pose message flip-flopped between two arc positions
    on opposite legs of the out-and-back route. *Fixed*: copied the
    already-validated windowed+heading-filtered search pattern from the
    obstacle guard. Verified run38.
11. **TL4 late-vote timer bug** — the blind-release "currently blind"
    check was anchored to a live-updating vote timestamp that a single
    late, transient phase glimpse could push forward, avoidably
    increasing the odds of releasing into the next red phase. *Fixed*:
    decoupled from a fixed, monotonic stop-onset anchor. (The *fundamental*
    R > C−R limitation this junction has is not fixable by any timer —
    documented honestly, not "fixed".)
12. **README install-order bug** — Section 1 told a fresh-machine reader to
    run `install_ros2_jazzy.sh` before Section 2 ever cloned the repo that
    script lives in. Present since the very first commit despite an
    earlier (incorrect) memory note claiming it had already been fixed.
    *Fixed*: swapped section order, verified via an actual dry-run
    clone+move in a scratch directory.
13. **`octomap_server` ground-filter TF error** — `filter_ground_plane`
    needs a `base_footprint` frame this project's TF tree doesn't have;
    errored every frame without filtering anything. *Fixed*: left off,
    relying on the height band alone (measured sufficient).
14. **OctoMap staleness-blindness** — see Section 5.7, bug 1: a dead/hung
    `octomap_server` could look permanently "fresh" from the consumer's
    own republish cadence alone. *Fixed*: check the map's own scan
    timestamp.
15. **OctoMap unbounded trust** — see Section 5.7, bug 2: even a live map
    could theoretically hold a detour forever on a persistent false
    positive. *Fixed*: 25 s wall-clock cap on map-only blocking.
16. **Ghost/orphaned process pile-ups** (recurring operational hazard, not
    a code bug): killing only the `ros2 launch` parent (or an incomplete
    pattern-match kill) repeatedly left individual child node processes
    — and, worse, the whole Unity binary — alive across "clean" restarts,
    causing multiple generations of nodes to fight over the same UDP
    command socket / TCP ports. Hit at least four separate times across
    this project. The only fully reliable fix found: collect every exact
    PID from a fresh `ps aux` and kill each one explicitly, then
    re-verify with a fresh `ps aux` *and* a port check (`ss -tulpn`) —
    never trust a kill command's own "success" or a single grep pass.

## 7. Performance Investigation

**Perception is fundamentally bounded by the Unity simulator itself, not
by this project's code.** Measured by holding `look_ahead`/`stride` at
several different values (45 m/stride 3, 30 m/stride 3, 20 m/stride 3)
and finding the raw corridor-publish rate essentially unchanged across all
of them (~0.7 Hz, 0.731 vs 0.666 Hz measured at the extremes — no
meaningful difference), while `ps` simultaneously showed the Unity
`LinuxBuild.x86_64` process itself pinned at 99.5% CPU on a single core.
The whole depth-image → point-cloud → corridor chain is upstream-rate-
limited by Unity's own render/physics loop; no amount of tuning this
project's own per-frame computation can buy more throughput, because the
frames themselves aren't arriving faster. This directly bounds how high
cruise speed can safely go before stale-corridor braking becomes frequent
— confirmed by *also* separately checking `octomap_server`'s own CPU
usage (measured ~77% of one core on a 24-core dev machine, load average
2.35) after adding it to the pipeline and finding zero measurable impact
on corridor-staleness frequency, consistent with the bottleneck being
Unity's single core, not general system CPU pressure.

**P-controller steady-state deficit**: the base throttle law
(`kp_speed * (v_des - v)`, no integral/feed-forward term) settles
0.7–2 m/s below every commanded speed, worsening at higher speed — a
textbook proportional-control steady-state error against speed-dependent
drag. Measured directly from run logs at multiple operating points (v_des
2.5 → measured 1.84; 2.2 → ~1.5; 2.0 → ~1.4; 8.0 → ~5.9–6.0) and closed
with the feed-forward table in Section 5.2 rather than adding an
integral term (a lookup table calibrated directly against measured
deficits was simpler to reason about and re-tune per breakpoint than
tuning an integrator's windup/gain behavior under the sim's own control
latency).

## 8. Testing & Validation Methodology

**Regression discipline**: every non-trivial change in this project was
followed by at least one full 785 m route run before being considered
done, checking for (a) route completion (`Goal reached`), (b) zero
collisions, (c) the same known events (the s=317 detour, the TL2/TL3
red→green cycles) firing identically to the established baseline, and
(d) no new warning/error classes in the launch log. Several changes
(the OctoMap integration especially) got two full regression runs — once
after the initial implementation, once again after a subsequent
adversarial-review-driven fix — specifically to catch any regression the
fix itself might have introduced.

**Adversarial review as a standing practice, not a one-off**: multiple
points in this project's history used an independent review pass — a
fresh agent given only the diff and told to actively try to find a
failure scenario, not to confirm the implementer's own framing — before
trusting a change as done. This caught real, non-obvious bugs the
implementer's own testing had not surfaced (the TL red-latch anchor bug,
confirmed 3-of-3 by an 89-agent review; the OctoMap staleness and
unbounded-trust bugs, both caught by a single dedicated review agent that
verified its claims against the actual installed `octomap_server` source
rather than trusting in-code comments).

**Ghost-process discipline**: given how many times orphaned processes
from an incomplete cleanup silently corrupted a subsequent "clean" test
run (Section 6, bug 16), every regression run in the later half of this
project's history was preceded by an explicit `ps aux` check, an
explicit-PID kill (never a pattern-match-and-hope), and a re-verification
pass (`ps aux` + `ss -tulpn` port check) before launching.

### 8.1 Fresh-install verification (Docker)

The grading rubric's single largest specific penalty clause is "-30p if
your code does not build as documented" — this project closed that lever
with an actual empirical test, not just careful README proofreading.
`sudo` on the dev machine requires an interactive password unavailable in
this environment; a rootless-Docker install was attempted and got as far
as confirming the kernel supports it (user namespaces, cgroup v2
delegation) before hitting a hard requirement on the `uidmap` package,
which itself needs `sudo` to install — no workaround exists. Once the
user installed real (non-rootless) Docker themselves, a two-agent
workflow (one agent builds, one independently audits the raw build log
without seeing the builder's own conclusions) replicated the README's
clone → `install_ros2_jazzy.sh` → `colcon build` sequence *verbatim* on a
pristine `ubuntu:24.04` image. Result: `docker build` exit 0, all 11
Dockerfile steps completed, 1433 packages installed via the bundled
script with zero `Unable to locate package` errors, and all 7 ROS
packages (`project_interfaces`, `simulation`, `perception`, `planning`,
`dummy_controller`, `control`, `bringup`) finished colcon's build with
zero failures in 24.8 s. The audit step also caught (and this project
then fixed) a real, previously-unfixed README step-ordering bug — Section
1 told a reader to run the install script before Section 2 ever cloned
the repo it lives in, present since the very first commit despite an
earlier project memory note incorrectly claiming it had already been
fixed.

### 8.2 Empirical event confirmation (not just code review)

Two of this project's task-required behaviors were validated with actual
photographic/log evidence that the *scenario itself* occurs in the sim,
not just that the code *would* handle it if it occurred:

- **Event II** (Section 5.6): a throttled corridor-entry log plus a
  frame-grabbing script confirmed a second, visually distinct NPC vehicle
  crossing at s≈531, separate from the known static prop-car, handled
  without collision.
- **Docker build** (Section 8.1): an actual `docker build` run, not a
  manual walkthrough of the README's steps.

## 9. Infrastructure

**Version control**: git repository set up from scratch mid-project
(the workspace root, `~/ros2_ws`, containing `src/`, `README.md`,
`.gitignore`, `install_ros2_jazzy.sh` — *not* `~/ros2_ws/src` itself).
`.gitignore` excludes `build/`/`install/`/`log/` and both Unity build
directories (`unity_sim/Build_Ubuntu/`, `unity_sim/Build_v1.2/`, ~940 MB
combined) — the course-provided simulator binary is identical for every
student and downloadable separately, so committing a second copy would
only waste the repo's storage/bandwidth quota for zero benefit; the
README instead links the course's official download and documents the
exact file layout expected afterward.

**Dependency documentation**: the apt dependency list in the README was
derived by actually surveying what a bare `ros-jazzy-desktop` install is
missing (`python3-colcon-common-extensions`, `ros-jazzy-depth-image-proc`,
`ros-jazzy-octomap-server`, plus `git`/`unzip` for the clone/unzip steps
themselves and `libopencv-dev` — already pulled in transitively, but
listed explicitly since `perception` depends on it directly) — cross-
checked against every package.xml `<depend>` tag across all five custom
packages, not assumed from memory.

**`install_ros2_jazzy.sh`** is a superset convenience script (installs a
few extra genuinely-unused packages — `ros-dev-tools`,
`ros-jazzy-pcl-ros`, `ros-jazzy-tf2-tools`,
`ros-jazzy-octomap-rviz-plugins`, `git-lfs`) alongside the strict minimum
the README's manual apt line documents; this divergence is intentional
(the script is a one-shot "just get everything" path, the manual list is
the audited minimum) rather than an inconsistency.

## 10. Known Limitations, Honestly

(Same substance as `DOCUMENTATION.md` Section 5, expanded with the
reasoning already covered in-line above — repeated here only as a single
consolidated list for quick reference.)

- TL4's blind-release timing has a *mathematically unavoidable* residual
  risk (worst case ~8 s of overlap into the next red phase) given the
  junction's own red-phase-longer-than-green-phase timing — Section 5.4.
- The OctoMap curb cross-check is safety-bounded and code-reviewed but
  never empirically observed to catch a real curb in testing — Section
  5.7.
- Perception/corridor update rate (~0.7–1 Hz) is capped by the Unity
  simulator's own single-core performance, not by anything in this
  codebase — Section 7.
- Group-authorship: the git history is dominated by one contributor's
  commits even where multiple people contributed conceptually/in
  discussion — not fixable via code, only via the written summary /
  presentation honestly addressing the real contribution split (team
  names/contributions in `DOCUMENTATION.md` are still placeholders
  pending the team filling them in).

## 11. Topic / Service Glossary

| Name | Type | Publisher(s) | Subscriber(s) |
|---|---|---|---|
| `/OurCar/CoM/pose` | `geometry_msgs/PoseStamped` | `simulation` | `perception`, `planning`, `control` |
| `/OurCar/CoM/twist` | `geometry_msgs/TwistStamped` | `simulation` | `control` |
| `/OurCar/Sensors/DepthCamera/{image_raw,camera_info}` | `sensor_msgs/{Image,CameraInfo}` | `simulation` | `depth_image_proc` (external) |
| `/OurCar/Sensors/RGBCameraLeft/image_raw` | `sensor_msgs/Image` | `simulation` | `traffic_light_node` |
| `/perception/pcl/points` | `sensor_msgs/PointCloud2` | `depth_image_proc` (external) | `obstacle_guard_node`, `octomap_server` (external) |
| `/perception/octomap/projected_map` | `nav_msgs/OccupancyGrid` | `octomap_server` (external) | `obstacle_guard_node` |
| `/perception/{obstacle,tight,overtake_left,overtake_right}_distance` | `std_msgs/Float32` | `obstacle_guard_node` | `pure_pursuit_node` |
| `/perception/map_tight_distance` | `std_msgs/Float32` | `obstacle_guard_node` | `pure_pursuit_node` |
| `/perception/traffic_light` | `project_interfaces/TrafficLight` | `traffic_light_node` | `pure_pursuit_node` |
| `/planning/trajectory` | `project_interfaces/Trajectory` | `route_planner_node` | `obstacle_guard_node`, `pure_pursuit_node` |
| `/planning/detour` | `std_msgs/Float32MultiArray` | `pure_pursuit_node` | `route_planner_node` |
| `/planning/current_goal` | `geometry_msgs/PoseStamped` | `route_planner_node` | (informational / RViz) |
| `/car_command` | `simulation/VehicleControl` | `pure_pursuit_node` | `simulation` (`ROS_command_transmitter`) |
| `/control/enable` | `std_srvs/SetBool` (service) | — | `pure_pursuit_node` (server) |

---

*This document is meant to converge, not to be finished once and frozen —
if a future change makes any section above inaccurate, update the section
in place rather than leaving stale detail next to new code.*
