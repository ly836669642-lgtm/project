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

A single launch file (`bringup/main.launch.py`) starts every node, the
static TF tree, and RViz. See Section 3 for the full ROS graph.

## 2. Packages and Nodes

| Package | Owner (EDIT) | Node(s) | Responsibility |
|---|---|---|---|
| `simulation` | *course-provided* | `unity_TCP_stream_receiver`, `ROS_command_transmitter` | TCP/UDP bridge to Unity: publishes ground-truth pose/twist, RGB/depth/semantic camera, IMU; sends `VehicleControl` commands. Ported to ROS 2 Jazzy from the course's original package, not otherwise modified. |
| `dummy_controller` | *course-provided* | — | Minimal example controller from the course. Not used by `bringup`; kept only for reference. |
| `project_interfaces` | Author 1 | — | Our custom message types: `Trajectory`/`TrajectoryPoint` (planned path with per-point speed) and `TrafficLight` (detected signal state). |
| `perception` | Author 2 | `obstacle_guard_node` | Transforms the depth-camera point cloud into the world frame and measures, along the upcoming planned trajectory, the arc-length distance to the nearest obstacle in four corridors (main/tight/overtake-left/overtake-right), plus a supplementary check against an accumulated OctoMap projection for low obstacles (curbs) the live per-frame scan can't see (Section 2.3). |
| `perception` | Author 2 | `traffic_light_node` | RGB HSV blob detection of the facing signal head. Deliberately does **not** use the semantic camera (bonus criterion). |
| `planning` | Author 3 | `route_planner_node` | Builds a lane-snapped, curvature-limited-speed trajectory from the recorded waypoints; reactively replans a lateral detour around a confirmed static obstacle; selects the nearest still-ahead goal from the task's predefined pose list. |
| `control` | Author 1 | `pure_pursuit_node` | Pure-pursuit path tracking, ACC-style gap control, traffic-light stop/go state machine, detour engage/verify/thread/hold state machine, emergency braking. Implements the required `/control/enable` service. |
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

`route_planner_node` loads the recorded waypoints, builds a curvature-
limited speed profile, and publishes a `Trajectory`. On a
`/planning/detour` request from `control` it re-splices a laterally-offset
segment around a confirmed static obstacle, then restores the base route
once past it. It also implements short-term goal selection over the
task's predefined pose list (Fig. 4 in the task sheet): each pose is
projected onto the route's arc length once at startup, and the nearest
goal still ahead of the car is published (on change only) to
`/planning/current_goal`.

### 2.3 Control

`pure_pursuit_node` tracks the published trajectory with a standard
pure-pursuit lateral law and a throttle law combining a feed-forward table
(calibrated against the simulator's own speed-tracking deficit) with
proportional correction. On top of that:

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

![ROS computation graph, colored by author](figures/ros_graph.png)

*(Author colors are placeholders — see the table in Section 2 for the
current mapping; replace with real names before submission.)*

## 4. Results

A full, uninstrumented run of the finished system (`bringup/main.launch.py`,
default parameters): **272 s, zero collisions, zero stalls, one detour, two
red-light stops handled correctly**, matching the same events/timing
observed across every regression run in this development cycle (the
detour always engages at the same static obstacle, both traffic lights
always transition red→green cleanly with no flicker).

- Average speed **2.7 m/s**, peak **7.5 m/s** on the two longest clear
  straights.
- `route_speed_map.png` shows the driven path colored by instantaneous
  speed — the dark (near-zero) points mark the detour and the two
  traffic-light stops; speed climbs highest on the long straights on
  either side of the loop.
- `speed_profile.png` shows the same data as speed vs. distance travelled:
  five visible near-zero dips (the detour's approach-and-verify wait plus
  a couple of ACC-queueing slowdowns, and the two full traffic-light
  stops), with the car re-accelerating smoothly after each.
- The distance-travelled axis reads **~721 m** rather than the route's
  documented ~785 m: it's a straight-line, point-to-point Euclidean sum
  over 10 Hz pose samples, which undercounts on curves (chord vs. arc) —
  a measurement-method artifact, not a shorter drive.
- `route_on_map.png` / `track_traced.png` — the planned route overlaid on
  the course map, generated during route-planning development.

## 5. Known Limitations / Requirements Not Fully Met

- **TL4 blind-release timing**: this junction's signal head becomes
  detectable only right at the stop line, so the car sometimes has to
  release from the stop without a confirmed-fresh phase reading. The
  release timer is bounded by the junction's own measured red-phase
  duration, but the red phase (22 s) is longer than the following
  green+amber window (14 s) — for *any* fixed-delay timer with no live
  visibility, there exists an arrival phase-offset where release lands
  inside the *next* red phase (worst case ~8 s of unavoidable overlap).
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
