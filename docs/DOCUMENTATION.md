# Autonomous Driving Project — Documentation

TUM "Introduction to ROS" 2026 — Autonomous Driving group project.

This document explains the architecture, design decisions, external
dependencies, known limitations, and results of our submission. See the
[repository README](../README.md) for install/build/run instructions, and
[`DEEP_DIVE.md`](DEEP_DIVE.md) for the full engineering-detail version of
everything below (every algorithm, every parameter's exact value and the
reasoning behind it, a complete bug history, and the performance/testing
methodology).

## 1. System Overview

The car drives a fixed ~785 m urban loop published once at startup from a
pre-recorded, lane-snapped set of waypoints. Five ROS 2 packages we wrote
ourselves sit on top of two packages provided by the course (`simulation`,
the Unity TCP/UDP bridge, and `dummy_controller`, an unused reference
example) and several external ROS libraries:

```
Unity sim  <--TCP/UDP-->  simulation (course-provided bridge)
                                |
                                v
                    depth image, RGB image, pose, twist
                                |
        +-----------------------+------------------------+
        v                                                 v
   perception                                          planning
   (obstacle_guard_node,                          (route_planner_node:
    traffic_light_node,                            waypoints -> speed
    + octomap_server for a                          profile -> trajectory,
    supplementary curb check)                        detour replanning)
        |                                                 |
        +-----------------------+------------------------+
                                v
                            control
                    (pure_pursuit_node: path
                     tracking, ACC, traffic-light
                     state machine, detour/verify
                     state machine, emergency stop)
                                |
                                v
                       /car_command -> simulation -> Unity
```

*Perception and planning consume the simulator's sensor/pose streams
independently and run in parallel; `control` is the only node that
combines their outputs and is the sole path to `/car_command` — no other
node talks to the simulator directly.*

A single launch file (`bringup/main.launch.py`) starts every node, the
static TF tree, and RViz. See Section 3 for the full, auto-generated ROS
graph.

## 2. Packages and Nodes

| Package | Owner (EDIT) | Node(s) | Responsibility |
|---|---|---|---|
| `simulation` | *course-provided* | `unity_TCP_stream_receiver`, `ROS_command_transmitter` | TCP/UDP bridge to Unity: publishes ground-truth pose/twist, RGB/depth/semantic camera, IMU; sends `VehicleControl` commands. Ported to ROS 2 Jazzy from the course's original package, not otherwise modified. |
| `dummy_controller` | *course-provided* | — | Minimal example controller from the course. Not used by `bringup`; kept only for reference. |
| `project_interfaces` | Author 1 | — | Our custom message types: `Trajectory`/`TrajectoryPoint` (planned path with per-point speed) and `TrafficLight` (detected signal state). |
| `perception` | Author 2 | `obstacle_guard_node` | Transforms the depth-camera point cloud into the world frame and measures, along the upcoming planned trajectory, the arc-length distance to the nearest obstacle in four corridors (main/tight/overtake-left/overtake-right), plus a supplementary check against an accumulated OctoMap projection for low obstacles (curbs) the live per-frame scan can't see (Section 2.3). |
| `perception` | Author 2 | `traffic_light_node` | RGB HSV blob detection of the facing signal head. Deliberately does **not** use the semantic camera (bonus criterion). |
| `planning` | Author 3 | `route_planner_node` | Builds a lane-snapped, curvature-limited-speed trajectory from the recorded waypoints; reactively replans a lateral detour around a confirmed static obstacle; selects the nearest still-ahead goal from the task's predefined pose list. |
| `control` | Author 1 | `pure_pursuit_node` | Pure-pursuit path tracking, ACC (Adaptive Cruise Control)-style gap control, traffic-light stop/go state machine, detour engage/verify/thread/hold state machine, emergency braking. Implements the required `/control/enable` service. |
| `bringup` | Author 3 | — | Single launch file, static TF tree (sensor extrinsics per the task sheet's vehicle drawing), RViz config. |

### 2.1 Perception

`obstacle_guard_node` receives the depth camera's point cloud (converted to
`sensor_msgs/PointCloud2` by the external `depth_image_proc` component),
transforms every point into the world frame via `tf2`, and classifies each
against the closest segment of the currently-planned trajectory. Points
with world height in `[0.35, 3.0]` m are kept (below that is road-surface
noise and low sidewalk fringes); four corridor distances are published,
including a `tight` corridor used both for ACC following and for verifying
a detour path is actually clear before committing to it.

`traffic_light_node` classifies connected components in an HSV-thresholded
mask of the facing camera image, filtering blobs by shape (compact,
roughly square, well-filled) to reject false positives such as the red
ring of no-U-turn signs or red shop banners. Only red and green are
detected — the amber phase is skipped because the signal casing is itself
amber-painted metal and cannot be reliably separated from a lit amber
lamp; the controller treats "stopped, no fresh green yet" as "keep
waiting," which is compliant behaviour.

### 2.2 Planning

#### Base Path Generation

`route_planner_node` loads the recorded waypoints and, once at startup,
builds a fixed *base path* through three steps: a right-hand lane offset,
iterative smoothing, and a curvature- and acceleration-limited speed
profile.

The lane offset shifts the recorded line sideways so the car drives its
own side of the road rather than straddling the recorded centreline,
using a stretch-by-stretch offset amount tuned to how much room each part
of the road actually has; the offset eases in and out gradually between
stretches and backs off in sharp turns so it never pushes the car toward
a curb or a junction's inside edge. The offset path is then smoothed to
remove the jaggedness inherent in sampled waypoints, since a noisy path
would otherwise be read as artificially tight curvature.

Finally each point is assigned a speed: tighter curves get a lower speed
limit, and separate braking and accelerating passes walk the path in each
direction so that the speed never changes faster than the vehicle can
actually brake or accelerate between two consecutive points. The
resulting base path, with per-point position, heading, curvature, and
speed, is published once as the planner's live trajectory.

#### Detour Replanning and Goal Selection

On top of the static base path, `route_planner_node` handles two forms of
on-line update: obstacle detours requested by `control`, and continuous
short-term goal selection over the task's predefined pose list.

When `control` confirms a static obstacle ahead, it asks the planner for
a local detour: a short stretch of the base path is shifted sideways just
enough to clear the obstacle, then blended smoothly back into the
original line on either side so the car never has to make an abrupt
lateral jump. Rather than recomputing the whole route, the planner
locates the affected stretch directly by binary-searching the path's
arc-length ordering, which keeps replanning cheap and responsive even as
the route grows. The detoured section is re-run through the same
curvature- and acceleration-limited speed profile as the base path, so it
stays just as driveable. Once the obstacle is reported clear, the planner
simply reverts to the unmodified base path.

Independently, the planner continuously tracks the vehicle's progress
along the route and selects the next short-term goal from the task's
predefined pose list, publishing whichever goal is nearest ahead of the
car as it advances. Because the route loops back near itself at a few
points, a plain nearest-point search would be ambiguous between two
nearby but unrelated points on the path; the planner resolves this once,
at startup, with a heading-filtered search that locks onto the correct
pass through the route, then stays locked on with a narrow
position-windowed local search around the last match — cheaper than
repeating the heading check on every update, and sufficient once the
initial ambiguity is resolved.

### 2.3 Control

`pure_pursuit_node` tracks the published trajectory with a standard
pure-pursuit lateral law — steering is chosen geometrically from a
look-ahead point on the path rather than from a PID error term [2] — and
a throttle law combining a feed-forward table with proportional
correction. The feed-forward table exists because a plain proportional
throttle law alone settles noticeably below its commanded speed at
steady state (a textbook proportional-control steady-state error against
the simulator's own speed-dependent drag); the table is calibrated
directly against that measured gap at several speeds rather than added
as an integral term. On top of that:

- **`/control/enable` service** (`std_srvs/SetBool`, the project's
  required self-implemented ROS service) lets an external caller start or
  stop the vehicle on demand: the node launches holding the brake by
  default (`start_enabled:=false`), and a `true` call releases it to
  begin driving. Used, for example, to hold the car at a known
  simulation state before releasing it for a timed test run.
- **ACC-style gap control** slows for anything in the tight/main corridor
  ahead, with a distance-scaled approach cap.
- **Traffic-light state machine** stops at the correct stop line for a red
  phase and releases on a confirmed green; a bounded blind-release timer
  covers the one junction (TL4) whose signal head leaves the camera's
  field of view right at the stop line — see Section 5 for the honest
  limitation here.
- **Obstacle detour state machine** (`kNormal -> kVerify -> kDetour`, with
  a `kHold` fallback) proactively replans around a static obstacle once
  confirmed parked (tracked as static vs. moving by watching its world
  position over a few seconds), verifies the shifted path is clear before
  committing, and threads it.
- **Emergency stop**: every control tick, independent of the above,
  computes the deceleration required to stop before the nearest raw
  corridor obstacle (`a = v^2 / (2 d)`) and immediately commands full
  brake if that exceeds a threshold — this is the task's Event II
  ("NPC crosses then brakes hard") handling.
- **Supplementary OctoMap cross-check**: `obstacle_guard_node` also
  publishes `/perception/map_tight_distance`, sourced from
  `octomap_server`'s accumulated occupancy map (height-banded to
  0.08–0.35 m, exactly the range the live per-frame corridor filter
  ignores) rather than the instantaneous point cloud. `kVerify` ANDs this
  into its detour-clear check: a fresh map hit can additionally veto a
  detour the live sensor alone would have approved (catching a curb the
  live sensor structurally cannot see), but a stale or absent reading
  never blocks anything by itself. Two bounded-trust safeguards keep this
  from ever causing an indefinite hold: the map signal is judged stale
  (and ignored) if the underlying map itself hasn't been updated
  recently, and a map-only block (live sensor says clear) can hold for at
  most 25 s before the system falls back to trusting the live sensor
  alone.

## 3. ROS Graph

The graph below is generated directly from the launch file (not
hand-drawn), so it reflects the actual node/topic wiring: nodes are
ellipses, topics are the labeled edges between them, and edge direction
shows publisher → subscriber. Node color groups each node by which of us
implemented it, matching the ownership column in the package table above.

![ROS computation graph, colored by author](figures/ros_graph.png)

*(Author colors are placeholders — see the table in Section 2 for the
current mapping; replace with real names before submission.)*

## 4. Results

A full run of the finished system (`bringup/main.launch.py`, default
parameters, telemetry logged for these figures): **272 s, zero
collisions, zero stalls, one detour, two red-light stops handled
correctly**. This event pattern — the detour engaging at the same static
obstacle, TL2 and TL3 each requiring a stop-and-release — has repeated
identically across every regression run in this development cycle; we
have not observed it differ.

The route has four signal-controlled stop points in total (TL1–TL4, see
Section 5); TL2/TL3 have consistently been red on arrival throughout this
cycle's testing while TL1/TL4 have consistently been green, so no stop
was needed there. Because phase-on-arrival depends on the real-time sim
clock at spawn, a differently-timed run could see a different subset of
the four require a stop — the stop/wait/release logic (Section 2.3)
applies identically no matter which junction is red, so that would be a
different *subset firing*, not different behavior from what's documented
here.

The route also includes the task's Event II encounter (a second vehicle
crossing, then braking, met on the return leg — Section 2.3): in every
run where it was checked via saved camera frames, it appears at the same
location (~s = 531) and is handled by the ACC gap-following logic alone
(speed easing to ~0.5 m/s, no full stop, no emergency-brake activation) —
this is one of the near-zero dips below, not a second traffic-light stop.
The dedicated emergency-stop branch exists as a backstop for a
closer/faster version of this encounter and has not itself been observed
to fire in testing.

- Average speed **2.7 m/s**, peak **7.5 m/s** on the two longest clear
  straights.

![Driven path colored by instantaneous speed](figures/route_speed_map.png)

- `route_speed_map.png` (above) shows the driven path colored by
  instantaneous speed — the dark (near-zero) points mark the detour, the
  Event II encounter, and the two traffic-light stops; speed climbs
  highest on the long straights on either side of the loop.

![Speed vs. distance travelled](figures/speed_profile.png)

- `speed_profile.png` (above) shows the same data as speed vs. distance
  travelled: five visible near-zero dips — the detour's approach-and-verify
  wait, two ACC-managed slowdowns (one of them the Event II encounter
  above), and the two full traffic-light stops — with the car
  re-accelerating smoothly after each.
- The distance-travelled axis reads **~721 m** rather than the route's
  documented ~785 m: it's a straight-line, point-to-point Euclidean sum
  over 10 Hz pose samples, which undercounts on curves (chord vs. arc) —
  a measurement-method artifact, not a shorter drive.

![Planned route overlaid on the course map](figures/route_on_map.png)

![Traced/driven track over the recorded waypoints](figures/track_traced.png)

- `route_on_map.png` (above) shows the finished planned route (green)
  overlaid on the course's top-down map, with the ten predefined goal
  poses from the task's pose list marked and numbered 0–9 — this is the
  actual input Section 2.2's goal-selection logic runs against, not a
  schematic.
- `track_traced.png` (above) is a closer view of the raw recorded
  waypoints near the start of the route, captured during route-planning
  development to check the recording matched the intended path before
  lane-offset and smoothing were applied.

## 5. Known Limitations / Requirements Not Fully Met

- **TL4 blind-release timing**: this junction's signal head becomes
  detectable only right at the stop line, so the car sometimes has to
  release from the stop without a confirmed-fresh phase reading. The
  release timer's 22 s bound comes from a red-phase duration measured
  directly at TL1 — the one junction whose head stays visible long enough
  to time a full red cycle — and is assumed, not independently confirmed,
  to also bound TL4's own red phase, since TL4's head can't be observed
  early enough to measure its red duration the same way. Even taking that
  transferred bound at face value, the red phase (22 s) is longer than
  the following green+amber window (14 s) — for *any* fixed-delay timer
  with no live visibility, there exists an arrival phase-offset where
  release lands inside the *next* red phase (worst case ~8 s of
  unavoidable overlap).
  We removed one avoidable failure mode (a late transient vote extending
  the wait past the intended point) but this residual risk is a physical
  limitation of the intersection's geometry versus its light timing, not
  something a control-side fix alone can eliminate.
- **OctoMap curb cross-check** (Section 2.3) is code-reviewed and
  bounded-safe (verified not to introduce false blocks or hangs across
  multiple full-route regression runs) but never actually triggered
  during testing — this route's one static obstacle doesn't happen to sit
  at curb height the live sensor misses. It is included as a genuine
  safety net for cases the live corridor structurally can't see, not
  because we observed it catching something in our own test runs.
- **Perception update rate** (~0.7–1 Hz for the point-cloud-derived
  corridor distances) is limited by the Unity simulator's own single-core
  render/physics loop, confirmed by measuring the same corridor rate
  across several different look-ahead/decimation settings in our own
  code — not something fixable from the ROS side. This bounds how much
  cruise speed can be raised before stale-corridor braking becomes
  frequent.

## 6. External Code / Not Written by Us

- `simulation`, `dummy_controller`: provided by the course (maintainers
  Markus Ryll, Jiaming Zhang), ported to ROS 2 Jazzy; not modified beyond
  what porting required.
- `depth_image_proc` (`PointCloudXyzNode`), `octomap_server`,
  `tf2_ros`, `rviz2`, `rclcpp`/`rclcpp_components`, `std_srvs`, `cv_bridge`,
  OpenCV: standard ROS 2 / third-party libraries, used as-is via their
  public APIs and launch-file parameters (not modified); see bibliography.
- Everything in `project_interfaces`, `perception`, `planning`, `control`,
  and `bringup`'s launch file / TF tree is written by us.

## 7. Team & Contributions

*(Placeholder — replace with real names.)*

| Author | Main contributions |
|---|---|
| Author 1 | `control` (`pure_pursuit_node`): path tracking, ACC, traffic-light and detour state machines, emergency stop, `/control/enable` service; `project_interfaces` message design |
| Author 2 | `perception` (`obstacle_guard_node`, `traffic_light_node`): corridor obstacle monitoring, OctoMap cross-check, HSV traffic-light detection |
| Author 3 | `planning` (`route_planner_node`): trajectory generation, detour replanning, goal selection; `bringup` (launch file, TF tree, RViz config) |

## 8. Bibliography

1. Open Robotics, *ROS 2 Documentation — Jazzy Jalisco*, https://docs.ros.org/en/jazzy/, accessed 2026.
2. R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," Carnegie Mellon University Robotics Institute, Tech. Rep. CMU-RI-TR-92-01, 1992.
3. A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees," *Autonomous Robots*, 34(3), pp. 189–206, 2013.
4. `octomap_server` / `octomap_mapping`, OctoMap project, https://github.com/OctoMap/octomap_mapping.
5. `image_pipeline` / `depth_image_proc`, ROS Perception, https://github.com/ros-perception/image_pipeline.
6. R. B. Rusu and S. Cousins, "3D is Here: Point Cloud Library (PCL)," *IEEE International Conference on Robotics and Automation (ICRA)*, 2011.
7. G. Bradski, "The OpenCV Library," *Dr. Dobb's Journal of Software Tools*, 2000.
8. T. Foote, "tf: The Transform Library," *IEEE International Conference on Technologies for Practical Robot Applications (TePRA)*, 2013.
9. Open Robotics, *RViz Documentation*, https://github.com/ros-visualization/rviz.
10. Unity Technologies, *Unity Engine* (course-provided simulation binary).
11. TUM Chair of Autonomous Aerial Systems, *Introduction to ROS 2026 — Autonomous Driving Project* (task sheet), 2026.
