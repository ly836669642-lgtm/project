// Pure Pursuit trajectory tracker with obstacle-aware speed control.
//
// Subscribes:  /planning/trajectory              (latched, may be replanned)
//              /OurCar/CoM/pose, /OurCar/CoM/twist (ground truth state)
//              /perception/obstacle_distance      (1.4 m corridor, normal)
//              /perception/tight_distance         (1.1 m corridor, maneuvers)
//              /perception/overtake_{left,right}_distance (side corridors)
//              /perception/traffic_light          (facing signal phase)
// Publishes:   /car_command       (simulation/VehicleControl, 50 Hz)
//              /planning/detour   (std_msgs/Float32MultiArray
//                                  [s_start, s_end, offset, ramp_in, ramp_out]
//                                  in TRUE ARC METERS; zero window = restore)
// Services:    /control/enable    (std_srvs/SetBool; false = hold brake,
//                                  true = drive - the self-implemented ROS
//                                  service required by the task sheet)
//
// Decision design follows normal traffic behaviour, per project owner:
//   * Anything blocking the lane ahead: decelerate early and stop at a
//     full safety distance (obstacle_standoff, 13 m). NEVER reverse.
//   * A suddenly appearing vehicle merging in gets no special case - it
//     is just followed, like any other traffic. One that crosses and
//     brakes hard directly ahead (task Event II) is not a "keep
//     following" case: a dedicated emergency-stop check (top of
//     Control(), ahead of every other decision) commands full brake the
//     instant the required deceleration exceeds a comfort profile.
//   * An obstacle whose WORLD arc position stays constant while we watch
//     it (2.5 s tracked, or a full static_wait once stopped) is treated
//     as parked; then, if there is room, the road is straight, and no
//     signalized stop line is involved (a car waiting at a red is
//     traffic, not parked), request a lateral detour and crawl past it.
//     The 13 m standoff leaves enough ramp room, which is why reversing
//     is never needed.
//   * No overtaking where the route bends sharply (inner curb islands are
//     invisible: curbs sit below the corridor z-band). Bend detection uses
//     the NET HEADING CHANGE of the route, which waypoint noise cannot
//     fake, rather than pointwise curvature.
//   * Corridor data older than a fail-safe age means blocked, never free.
//
// States: NORMAL -> (static obstacle confirmed) -> VERIFY -> DETOUR
//         -> back to NORMAL; HOLD whenever there is nothing safe to do.
//
// All arc positions use the cumulative arc-length table of the published
// trajectory so controller, planner, and guard share exact units.

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <project_interfaces/msg/traffic_light.hpp>
#include <project_interfaces/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simulation/msg/vehicle_control.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

class PurePursuit : public rclcpp::Node {
public:
  PurePursuit() : Node("pure_pursuit") {
    declare_parameter("wheelbase", 2.63);        // m (task sheet Fig. 3)
    declare_parameter("lookahead_min", 3.0);     // m; 2.4 made low-speed
                                                 // tracking weave +-0.5 m
                                                 // (measured in run logs)
    declare_parameter("lookahead_gain", 0.5);    // s (Ld = min + gain * v)
    declare_parameter("max_steer_rad", 0.50);    // rad, maps to command 1.0
    declare_parameter("kp_speed", 0.30);
    declare_parameter("kp_brake", 0.60);
    // Hold-throttle feed-forward, calibrated from run logs (T_hold(v) ~=
    // 0.03 + 0.10*v): without it the pure P law parks 0.7-2 m/s below
    // every commanded speed (v_des 2.5 gave 1.84 measured).
    declare_parameter("throttle_ff_v",
                      std::vector<double>{0.0, 1.8, 5.9, 6.5, 8.0, 9.5});
    declare_parameter("throttle_ff_t",
                      std::vector<double>{0.0, 0.21, 0.62, 0.68, 0.83,
                                          0.98});
    // With the deficit closed v rides AT v_des, so any noise used to fire
    // the 0.15-base brake: feather/coast inside the deadband instead.
    // Bypassed for v_des < 0.5 (stops and fail-safes keep the brake law).
    declare_parameter("brake_deadband", 0.40);   // m/s below v_des
    declare_parameter("brake_release", 0.15);    // hysteresis exit
    declare_parameter("goal_tolerance", 3.0);    // m
    declare_parameter("obstacle_standoff", 13.0);  // m; stopping here leaves
                                                   // room to ramp out later
    declare_parameter("approach_speed", 2.5);    // m/s within approach_zone;
                                                 // brake lag overshoot
                                                 // scales with v^2
    declare_parameter("approach_zone", 40.0);    // m; must exceed both the
                                                 // guard's look_ahead (30)
                                                 // and approach_cap_far, or
                                                 // the cap ramp's far
                                                 // anchor is unreachable
    // Distance-scaled relaxation of the approach cap: flat 2.5 starved
    // the whole route (roadside furniture keeps the wide corridor
    // occupied 60-90% of the time). Ramp from (near, 2.5) up to
    // approach_cap_vmax at the far anchor. The tight-corridor ACC curve
    // sqrt(3*(d-13)) stays above this ramp everywhere below the far
    // anchor, so it remains the safety authority.
    declare_parameter("approach_cap_near", 14.0);  // m; also the detour
                                                   // engagement band top
    declare_parameter("approach_cap_far", 38.0);   // m
    declare_parameter("approach_cap_vmax", 6.5);   // m/s at the far end;
                                                   // 8.0 left zero decel
                                                   // margin for the ACC
                                                   // stop (49/(2*17) ~=
                                                   // a_obstacle) and the
                                                   // car once ate 7 m of
                                                   // the 13 m standoff
    declare_parameter("detour_standoff", 2.0);   // m, gap while threading
    declare_parameter("a_obstacle", 1.5);        // m/s^2, decel curve
    declare_parameter("detour_shift", 2.9);      // m, centers the pass gap
    declare_parameter("detour_margin", 10.0);    // m past the obstacle;
                                                 // short so passes just
                                                 // before corners still fit
    declare_parameter("detour_clearance", 18.0); // m free side corridor
    declare_parameter("detour_speed", 2.0);      // m/s while threading
    declare_parameter("detour_min_gap", 8.5);    // m needed to start a ramp;
                                                 // below this the car would
                                                 // livelock (no reverse), so
                                                 // short 6 m ramps at crawl
                                                 // speed are preferred over
                                                 // waiting forever - kVerify
                                                 // still gates the path
    declare_parameter("static_wait", 8.0);       // s stopped before an
                                                 // obstacle counts as parked
    declare_parameter("traffic_settle", 10.0);   // s an obstacle must stay
                                                 // put after last being SEEN
                                                 // moving before any detour;
                                                 // cars pulling in from side
                                                 // roads pause a few seconds
                                                 // and drive on - follow
                                                 // them, don't swerve
    declare_parameter("corner_heading_deg", 35.0);  // net turn that forbids
                                                    // overtaking
    declare_parameter("corner_zone_heading_deg", 30.0);  // net turn at the
                                                         // obstacle location
    declare_parameter("stale_blocked", 1.5);     // s; older data = blocked
    declare_parameter("brake_lag", 0.3);         // s of sim brake dead time;
                                                 // shifts the ACC curve
                                                 // earlier by speed*lag so
                                                 // fast approaches still
                                                 // stop at the standoff
    declare_parameter("emergency_decel", 4.0);   // m/s^2; task Event II (an
                                                 // NPC crosses, then brakes
                                                 // hard, directly ahead).
                                                 // The ACC ramp below
                                                 // targets a comfortable
                                                 // a_obstacle (1.5) profile
                                                 // and only reaches full
                                                 // brake once the P-law
                                                 // saturates on a large
                                                 // speed error - a genuine
                                                 // surprise needs an
                                                 // immediate, first-
                                                 // priority full-brake
                                                 // command instead, ahead
                                                 // of the comfort curve and
                                                 // every state machine.
                                                 // 4.0 sits above a_obstacle
                                                 // so ordinary approaches
                                                 // (already slowed by the
                                                 // wide-corridor cap before
                                                 // gaps get small) never
                                                 // trip it.
    declare_parameter("stall_time", 4.0);        // s throttle with no motion
    declare_parameter("stall_hold", 3.0);        // s of brake after a stall
                                                 // fires, in every mode: a
                                                 // one-tick brake let the
                                                 // throttle grind straight
                                                 // back into whatever the
                                                 // car was stuck on
    declare_parameter("hold_retry", 5.0);        // s between detour retries
    declare_parameter("maneuver_steer_cap", 0.65);  // avoid full-lock stall
    declare_parameter("standstill_steer_cap", 0.35);  // launch with wheels
                                                      // near straight
    // Stop-line arc positions of the four signalized intersections,
    // measured from RGB frames captured along the route (survey runs).
    // Overpass-hung heads (TL2-TL4) are only visible from a window
    // ~10-25 m in front of them; at the geometric waiting lines the lamp
    // is almost overhead, outside the camera FOV (verified with a live
    // frame from a stopped wait: empty sky where the head should be),
    // and every red wait ended in a phase-blind fallback crossing. The
    // lines therefore sit ~10 m early so the whole wait keeps the lamp
    // in view; the task sheet does not require stopping exactly at the
    // waiting line. TL1's mast-arm head is center-mounted and visible
    // from the line itself.
    // Stop anchors as WORLD points, mapped to route arc length whenever a
    // trajectory arrives. Arc constants had to be recalibrated by hand
    // after every route change (lane offsets, tapers, zones) and silently
    // aimed at the wrong place when someone forgot - the anchors are
    // physical places, so name them that way. Each sits ~10 m ahead of
    // its junction's waiting line, where the overpass-hung head enters
    // the camera's view.
    declare_parameter("traffic_stop_xy",
                      std::vector<double>{-62.4, 1.5,
                                          226.3, -2.0,
                                          129.6, 4.0,
                                          42.7, 4.9});
    declare_parameter("light_zone", 25.0);   // m before the line where a
                                             // red engages the stop
    declare_parameter("light_stale", 1.0);   // s; older phase info ignored
    declare_parameter("light_green_commit", 5.0);  // s after a green
                                                   // release during which a
                                                   // "red" cannot re-latch
                                                   // the same line (phase-
                                                   // impossible; it's the
                                                   // cross-street head)
    declare_parameter("light_stop_short",        // m stopped short of each
                      std::vector<double>{       // line (aligned with
                          6.0, 12.0,             // traffic_stop_s): far
                          9.0, 12.0});           // enough back that THAT
                                                 // head stays inside the
                                                 // camera view, so a green
                                                 // onset is SEEN and
                                                 // releases at the true
                                                 // phase instead of the 22 s
                                                 // blind timer. Per-light:
                                                 // 6 m sufficed at TL1 but
                                                 // TL2 went phase-blind
                                                 // (watched crossing looked
                                                 // like running the red);
                                                 // heads hang differently
                                                 // at every junction. The
                                                 // task sheet explicitly
                                                 // does not require exact
                                                 // stops at the line.
    declare_parameter("light_caution_zone", 40.0);  // m; without a fresh
                                                    // green, approach slowly
    declare_parameter("light_caution_speed", 2.2);  // m/s floor of the
                                                    // graded caution ramp
    declare_parameter("light_caution_accel", 1.2);  // m/s^2 of the graded
                                                    // ramp sqrt(2*a*(d-6)):
                                                    // a step cap at zone
                                                    // entry was a full-brake
                                                    // event from 6+ m/s
    declare_parameter("light_prox_cap", 4.0);    // m/s ceiling of the ramp:
                                                 // keeps the 2-of-3 phase
                                                 // debounce ahead of the
                                                 // stop curve on unknown
                                                 // phase; a confirmed green
                                                 // lifts it entirely
    declare_parameter("light_blind_release", 22.0);  // s BOTH stopped AND
                                                     // phase-blind before
                                                     // proceeding - one full
                                                     // measured red (22 s at
                                                     // TL1) so a lamp that
                                                     // left the FOV can never
                                                     // be crossed while its
                                                     // red is still running;
                                                     // then cross like an
                                                     // inoperative signal.

    declare_parameter("start_enabled", true);
    enabled_ = get_parameter("start_enabled").as_bool();

    cmd_pub_ = create_publisher<simulation::msg::VehicleControl>(
        "/car_command", 1);
    detour_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/planning/detour", 1);

    // Mission enable switch (the self-implemented ROS SERVICE required by
    // the task sheet): false = hold the brake in place, true = drive.
    // Used operationally for supervised starts and phase-shifted test
    // launches; disabling mid-drive is a safe remote stop.
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
        "/control/enable",
        [this](std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
          enabled_ = req->data;
          res->success = true;
          res->message = enabled_ ? "driving enabled" : "holding brake";
          RCLCPP_INFO(get_logger(), "Enable service: %s",
                      res->message.c_str());
        });

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    traj_sub_ = create_subscription<project_interfaces::msg::Trajectory>(
        "/planning/trajectory", latched,
        [this](project_interfaces::msg::Trajectory::SharedPtr m) {
          // detour replans keep the same indexing, so the tracked nearest
          // index stays valid; a global search here could snap onto the
          // opposite leg where the out-and-back route nearly overlaps
          traj_ = *m;
          RebuildArcTable();
          RCLCPP_INFO(get_logger(), "Trajectory update: %zu points",
                      traj_.points.size());
        });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/OurCar/CoM/pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { pose_ = *m; });
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/OurCar/CoM/twist", 10,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
          speed_ = std::hypot(m->twist.linear.x, m->twist.linear.y);
        });
    obstacle_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/perception/obstacle_distance", 10,
        [this](std_msgs::msg::Float32::SharedPtr m) {
          obstacle_dist_ = m->data;
          obstacle_stamp_ = now();
        });
    tight_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/perception/tight_distance", 10,
        [this](std_msgs::msg::Float32::SharedPtr m) {
          tight_dist_ = m->data;
          tight_stamp_ = now();
        });
    left_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/perception/overtake_left_distance", 10,
        [this](std_msgs::msg::Float32::SharedPtr m) {
          left_dist_ = m->data;
          left_stamp_ = now();
        });
    right_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/perception/overtake_right_distance", 10,
        [this](std_msgs::msg::Float32::SharedPtr m) {
          right_dist_ = m->data;
          right_stamp_ = now();
        });
    light_sub_ = create_subscription<project_interfaces::msg::TrafficLight>(
        "/perception/traffic_light", 10,
        [this](project_interfaces::msg::TrafficLight::SharedPtr m) {
          // 2-of-3 sliding window: debounces single-frame misreads AND
          // survives 50% detection flicker of a marginal-size blob, which
          // a consecutive-streak scheme (reset by every NONE) cannot.
          // A stream gap voids the window: two pre-dropout frames plus
          // one fresh one must not complete a phase vote.
          using TL = project_interfaces::msg::TrafficLight;
          if ((now() - last_light_msg_).seconds() > 0.7) light_win_.clear();
          last_light_msg_ = now();
          light_win_.push_back(m->state);
          if (light_win_.size() > 3) light_win_.pop_front();
          int reds = 0, greens = 0;
          for (uint8_t s : light_win_) {
            reds += s == TL::RED;
            greens += s == TL::GREEN;
          }
          if (reds >= 2) last_red_ = now();
          if (greens >= 2) last_green_ = now();
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this] { Control(); });
  }

private:
  enum class Mode { kNormal, kVerify, kDetour, kHold };
  // Moderate brake for every "wait in place, re-evaluate next tick" case
  // (kVerify/kHold pauses, the stall watchdog, the disabled-service hold):
  // firm enough to actually stop, gentle enough not to read as an
  // emergency stop. Deliberately NOT used by the dedicated stop curves
  // (traffic light, ACC, emergency) - those compute their own brake from
  // the actual required deceleration.
  static constexpr float kWaitBrake = 0.6f;

  void RebuildArcTable() {
    const auto& pts = traj_.points;
    arc_.assign(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i)
      arc_[i] = arc_[i - 1] + std::hypot(pts[i].x - pts[i - 1].x,
                                         pts[i].y - pts[i - 1].y);
    RebuildStopArcs();
  }

  // Project the world-frame stop anchors onto the current route. Runs on
  // every trajectory (including detour replans), so lane offsets, tapers
  // and local replans can move the path without ever invalidating them.
  // A global search is safe here: the anchors lie ON the route, while the
  // opposite leg of the out-and-back passes no closer than ~8 m.
  void RebuildStopArcs() {
    const std::vector<double> xy =
        get_parameter("traffic_stop_xy").as_double_array();
    const auto& pts = traj_.points;
    stop_arcs_.clear();
    if (pts.empty()) return;
    for (size_t k = 0; k + 1 < xy.size(); k += 2) {
      size_t best = 0;
      double best_d = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < pts.size(); ++i) {
        const double d = std::hypot(pts[i].x - xy[k], pts[i].y - xy[k + 1]);
        if (d < best_d) {
          best_d = d;
          best = i;
        }
      }
      stop_arcs_.push_back(arc_[best]);
    }
    std::sort(stop_arcs_.begin(), stop_arcs_.end());
  }

  void UpdateNearest(double px, double py, double yaw) {
    const auto& pts = traj_.points;
    size_t lo = 0, hi = pts.size();
    if (!need_global_search_) {
      lo = nearest_ > 25 ? nearest_ - 25 : 0;
      hi = std::min(nearest_ + 50, pts.size());
    }
    // The route starts and ends at the same corner (spawn 6.8 m from the
    // goal), so the one-off global search must not pick by distance
    // alone: snapping onto the route END put s_now at ~785 and silently
    // disarmed the first stop line and its caution zone every run. Only
    // segments roughly facing the car's heading may win; the plain
    // argmin remains as fallback (e.g. restarted mid-route on a U-turn).
    const bool check_heading = need_global_search_;
    double best = 1e18, best_any = 1e18;
    size_t nearest_any = nearest_;
    for (size_t i = lo; i < hi; ++i) {
      const double d = std::hypot(pts[i].x - px, pts[i].y - py);
      if (d < best_any) {
        best_any = d;
        nearest_any = i;
      }
      if (check_heading) {
        const double diff =
            std::remainder(HeadingAt(i) - yaw, 2.0 * M_PI);
        if (std::abs(diff) > M_PI / 2.0) continue;
      }
      if (d < best) {
        best = d;
        nearest_ = i;
      }
    }
    if (best > 1e17) nearest_ = nearest_any;
    need_global_search_ = false;
  }

  size_t IndexAtArcAhead(double dist) const {
    size_t i = nearest_;
    while (i + 1 < traj_.points.size() &&
           arc_[i] - arc_[nearest_] < dist)
      ++i;
    return i;
  }

  double HeadingAt(size_t i) const {
    const auto& pts = traj_.points;
    const size_t a = i > 1 ? i - 1 : 0;
    const size_t b = std::min(i + 1, pts.size() - 1);
    return std::atan2(pts[b].y - pts[a].y, pts[b].x - pts[a].x);
  }

  // net heading change (deg) of the route between lo and hi metres ahead;
  // immune to waypoint jitter, unlike pointwise curvature
  double HeadingChangeDeg(double lo, double hi) const {
    const double h0 = HeadingAt(IndexAtArcAhead(std::max(0.0, lo)));
    const double h1 = HeadingAt(IndexAtArcAhead(hi));
    double d = h1 - h0;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return std::abs(d) * 180.0 / M_PI;
  }

  void SendDetour(double s0, double s1, double offset, double ramp_in,
                  double ramp_out) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {static_cast<float>(s0), static_cast<float>(s1),
                static_cast<float>(offset), static_cast<float>(ramp_in),
                static_cast<float>(ramp_out)};
    detour_pub_->publish(msg);
  }

  // Request a lateral detour around a confirmed-static obstacle, starting
  // rolling or from standstill (never preceded by reversing). The 13 m
  // standoff guarantees ramp room; ramps are floored at 6 m - short
  // ones are only reached when a hot stop ate into the standoff, and
  // the kVerify corridor check still gates the resulting path.
  bool EngageDetour(double s_now, double gap) {
    // committing to a replan needs FRESH data on all four corridors (the
    // guard publishes them atomically, so one freshness bar covers all)
    const rclcpp::Time oldest =
        std::min({obstacle_stamp_, tight_stamp_, left_stamp_, right_stamp_});
    if ((now() - oldest).seconds() > 1.0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Corridor data stale - not committing to detour");
      return false;
    }
    if (gap < get_parameter("detour_min_gap").as_double()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "No room for a ramp (gap %.1f m) - waiting", gap);
      return false;
    }
    s_obs_ = s_now + std::min(gap, 25.0);
    detour_end_ = s_obs_ + get_parameter("detour_margin").as_double();
    const double ramp_in = std::clamp(gap - 2.0, 6.0, 12.0);
    const double ramp_start = s_obs_ - ramp_in - 2.0;
    // never overtake through a corner (traffic rule): a shifted path there
    // crosses the inner curb island, invisible below the corridor z-band.
    // Only the stretch where the car is substantially off-center matters -
    // the return taper's tail (offset fading below ~30%) may brush a
    // gentle bend; the tight-corridor verify still checks that exact path.
    const double turn = HeadingChangeDeg(std::max(0.0, ramp_start - s_now),
                                         s_obs_ + 5.0 - s_now);
    if (turn > get_parameter("corner_heading_deg").as_double()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Refusing detour: road turns %.0f deg in window",
                           turn);
      return false;
    }
    // RIGHT first: corridor clearance cannot see curbs, so "more room" is
    // no proxy for a drivable pass (the left/north squeeze at prop car 1
    // reads clear yet wedges the car on the curb - measured in a clean
    // environment). Passing a wrong-way parked car along its driver side
    // (our right) is also the natural traffic move; left remains the
    // fallback when the right corridor is blocked.
    const double need = get_parameter("detour_clearance").as_double();
    detour_dir_ = 0.0;
    if (right_dist_ > need) detour_dir_ = -1.0;
    else if (left_dist_ > need) detour_dir_ = 1.0;
    if (detour_dir_ == 0.0) return false;
    SendDetour(ramp_start, detour_end_,
               detour_dir_ * get_parameter("detour_shift").as_double(),
               ramp_in, 4.0);
    tried_other_side_ = false;
    mode_ = Mode::kVerify;
    state_since_ = now();
    RCLCPP_INFO(get_logger(),
                "Requested %s detour around parked obstacle at s=%.0f m "
                "(gap %.1f)",
                detour_dir_ > 0 ? "left" : "right", s_obs_, gap);
    return true;
  }

  // Static/queued-obstacle avoidance: kNormal (watch + proactive replan,
  // then a stopped-behind-it fallback) -> kVerify (confirm the shifted
  // path is actually clear) -> kDetour (thread it) -> back to kNormal;
  // kHold whenever nothing safe is currently possible. Returns true if
  // it already published this tick's command (waiting on a fresh
  // corridor read, or holding) - Control() must stop right there.
  bool RunObstacleStateMachine(double s_now, double standoff,
                               double watch_norm, double watch_comp,
                               bool green_fresh,
                               const std::vector<double>& stop_lines,
                               simulation::msg::VehicleControl& cmd) {
    // Queue-traffic guard: an obstacle sitting between us and a
    // signalized stop line (or just past it) while the phase is not a
    // confirmed green is almost certainly a vehicle WAITING at the
    // light. Detouring around it would carry the car through the
    // junction against the phase - wait instead, like everyone else.
    bool queue_at_light = false;
    if (std::isfinite(watch_norm) && !green_fresh) {
      const double q_margin =
          get_parameter("detour_margin").as_double() + 5.0;
      for (double s_stop : stop_lines) {
        if (s_stop > s_now - 2.0 &&
            s_stop < s_now + watch_norm + q_margin) {
          queue_at_light = true;
          break;
        }
      }
    }
    if (mode_ == Mode::kNormal) {
      // Owner rule: the planned line must NEVER keep crossing a known
      // obstacle - divert as soon as one is confirmed static. A static
      // obstacle's WORLD arc position (s_now + distance) stays constant
      // while we approach; a moving vehicle's drifts, so it never
      // triggers a replan.
      if (std::isfinite(watch_norm) &&
          watch_norm < get_parameter("approach_zone").as_double()) {
        obs_track_.emplace_back(now(), s_now + watch_comp);
        while (!obs_track_.empty() &&
               (now() - obs_track_.front().first).seconds() > 3.0)
          obs_track_.pop_front();
        if ((obs_track_.back().first - obs_track_.front().first).seconds() >
            2.5) {
          double mn = 1e18, mx = -1e18;
          for (const auto& e : obs_track_) {
            mn = std::min(mn, e.second);
            mx = std::max(mx, e.second);
          }
          // a drifting world position is a MOVING vehicle (side-road
          // traffic pulling in, crossing NPCs): remember that, and give
          // it traffic_settle seconds of immobility before it can ever
          // be classified as parked - until then ACC just follows it
          if (mx - mn >= 1.5) {
            obs_moving_until_ =
                now() + rclcpp::Duration::from_seconds(
                            get_parameter("traffic_settle").as_double());
          }
          // side ranking from the side corridors is only reliable within
          // ~approach_cap_near meters (at 19 m it once picked the
          // curb-blocked side and the car wedged); still replans a few
          // seconds before arrival
          if (mx - mn < 1.5 && !queue_at_light &&
              now() >= obs_moving_until_ &&
              watch_norm <=
                  get_parameter("approach_cap_near").as_double() &&
              watch_norm >= get_parameter("detour_min_gap").as_double()) {
            if (EngageDetour(s_now, watch_norm)) obs_track_.clear();
          }
        }
      } else {
        obs_track_.clear();
      }
    }
    if (mode_ == Mode::kNormal) {
      // Fallback: stopped behind something the proactive path could not
      // divert around (corner refusals, tight gaps). Traffic moves on
      // within seconds; only a gap that stays constant the whole wait is
      // a parked obstacle worth going around.
      const bool held = std::isfinite(watch_norm) &&
                        watch_norm < standoff + 2.0 && speed_ < 0.6;
      if (held) {
        if (!blocked_since_.has_value() ||
            std::abs(watch_norm - blocked_gap_) > 1.5) {
          blocked_since_ = now();
          blocked_gap_ = watch_norm;
        }
        if ((now() - *blocked_since_).seconds() >
            get_parameter("static_wait").as_double()) {
          blocked_since_.reset();
          if (queue_at_light || now() < obs_moving_until_) {
            // vehicle waiting at a signal, or one that was still MOVING
            // moments ago (side-road traffic settling): not parked,
            // restart the clock
            blocked_since_ = now();
          } else if (!EngageDetour(s_now, watch_norm)) {
            mode_ = Mode::kHold;
            state_since_ = now();
            RCLCPP_INFO(get_logger(),
                        "Parked obstacle, no safe detour - waiting");
          }
        }
      } else {
        blocked_since_.reset();
      }
    } else if (mode_ == Mode::kVerify) {
      // wait for a corridor measurement taken AFTER the replan, then check
      // the tight corridor over the whole detour window
      const double waited = (now() - state_since_).seconds();
      const bool fresh = tight_stamp_ > state_since_ && waited > 0.6;
      if (!fresh && waited < 3.0) {
        cmd.brake = kWaitBrake;
        PublishCmd(cmd);
        return true;
      }
      const double horizon = std::min(28.0, detour_end_ - s_now);
      const bool clear = fresh && (!std::isfinite(tight_dist_) ||
                                   tight_dist_ > horizon);
      if (clear) {
        mode_ = Mode::kDetour;
        RCLCPP_INFO(get_logger(), "Detour path verified clear - going");
      } else if (!tried_other_side_) {
        tried_other_side_ = true;
        detour_dir_ = -detour_dir_;
        const double gap = s_obs_ - s_now;
        const double ramp_in = std::clamp(gap - 2.0, 6.0, 12.0);
        SendDetour(s_obs_ - ramp_in - 2.0, detour_end_,
                   detour_dir_ * get_parameter("detour_shift").as_double(),
                   ramp_in, 4.0);
        state_since_ = now();
        RCLCPP_INFO(get_logger(), "Detour blocked, trying the other side");
      } else {
        SendDetour(0, 0, 0, 0, 0);
        mode_ = Mode::kHold;
        state_since_ = now();
        RCLCPP_INFO(get_logger(),
                    "Both sides blocked - holding and waiting");
      }
    } else if (mode_ == Mode::kDetour) {
      // While threading, NEVER restore the base route (it runs through the
      // parked obstacle and the car may be alongside it). If the detour
      // path is blocked, the tight-corridor ACC below simply holds the car
      // in place until it clears again - waiting is always legal here.
      if (s_now > detour_end_) {
        SendDetour(0, 0, 0, 0, 0);
        mode_ = Mode::kNormal;
        RCLCPP_INFO(get_logger(), "Detour done, back on the base route");
      }
    } else if (mode_ == Mode::kHold) {
      // wait in place, then hand the decision back to kNormal, which owns
      // the 8-s static confirmation - kHold itself never requests detours
      if (now() < hold_min_until_) {
        cmd.brake = kWaitBrake;
        PublishCmd(cmd);
        return true;
      }
      if (!std::isfinite(watch_norm) || watch_norm > standoff + 4.0) {
        mode_ = Mode::kNormal;
        RCLCPP_INFO(get_logger(), "Path ahead cleared - resuming");
      } else if ((now() - state_since_).seconds() >
                 get_parameter("hold_retry").as_double()) {
        mode_ = Mode::kNormal;  // re-confirm the obstacle from scratch
        blocked_since_.reset();
      } else {
        cmd.brake = kWaitBrake;
        PublishCmd(cmd);
        return true;
      }
    }
    return false;
  }

  // single exit point: every command passes the stall watchdog, so a car
  // that is commanded forward but does not move is always detected
  void PublishCmd(simulation::msg::VehicleControl& cmd) {
    // A fired watchdog brakes for stall_hold seconds, in EVERY mode. The
    // old code substituted a brake for one 20 ms tick and, outside
    // kNormal, let full throttle resume immediately: against a curb that
    // is a permanent 4-s-throttle / one-tick-brake grind (observed) which
    // can climb the car over the obstacle it is stuck on.
    if (now() < stall_hold_until_) {
      cmd = simulation::msg::VehicleControl();
      cmd.brake = kWaitBrake;
      braking_ = true;
      cmd_pub_->publish(cmd);
      return;
    }
    if (cmd.throttle >= 0.25f && cmd.brake == 0.0f && speed_ < 0.1) {
      if (!stall_since_.has_value()) stall_since_ = now();
      if ((now() - *stall_since_).seconds() >
          get_parameter("stall_time").as_double()) {
        stall_since_.reset();
        stall_hold_until_ =
            now() + rclcpp::Duration::from_seconds(
                        get_parameter("stall_hold").as_double());
        // Never restore the base route here: mid-detour the car may be
        // alongside the obstacle and the base route runs through it.
        // Brake and wait where we are; only kNormal escalates to kHold.
        if (mode_ == Mode::kNormal) {
          mode_ = Mode::kHold;
          state_since_ = now();
          hold_min_until_ = now() + rclcpp::Duration::from_seconds(10.0);
        }
        RCLCPP_ERROR(get_logger(),
                     "Stalled: commanded throttle %.2f but v=%.2f - "
                     "braking in place (no reverse by design)",
                     cmd.throttle, speed_);
        cmd = simulation::msg::VehicleControl();
        cmd.brake = kWaitBrake;
      }
    } else {
      stall_since_.reset();
    }
    // single exit point: brake-branch hysteresis stays consistent across
    // every early-return publish (kVerify/kHold waits, hard stops, stall)
    braking_ = cmd.brake > 0.0f;
    cmd_pub_->publish(cmd);
  }

  void Control() {
    simulation::msg::VehicleControl cmd;
    if (!enabled_) {
      cmd.brake = kWaitBrake;
      PublishCmd(cmd);
      return;
    }
    if (traj_.points.empty() || !pose_.has_value()) {
      PublishCmd(cmd);  // all zeros until inputs are ready
      return;
    }
    const auto& pts = traj_.points;
    const double px = pose_->pose.position.x;
    const double py = pose_->pose.position.y;
    const auto& q = pose_->pose.orientation;
    const double yaw =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    UpdateNearest(px, py, yaw);
    const double s_now = arc_[nearest_];

    // goal reached -> hold full brake
    const double goal_d =
        std::hypot(pts.back().x - px, pts.back().y - py);
    if (finished_ ||
        (nearest_ + 10 >= pts.size() &&
         goal_d < get_parameter("goal_tolerance").as_double())) {
      finished_ = true;
      cmd.brake = 1.0;
      PublishCmd(cmd);
      RCLCPP_INFO_ONCE(get_logger(), "Goal reached - holding brake.");
      return;
    }

    // --- corridor roles ----------------------------------------------
    // HARD blocking (stop / detour decisions) uses the 1.1 m tight
    // corridor: that is the swept path, and anything wider than it cannot
    // be hit. The 1.4 m corridor over-triggers on roadside furniture
    // (guardrails, arch feet, hydrants at lat 1.1-1.4) and only moderates
    // SPEED below - like a human slowing past close parked objects.
    const double watch_norm = tight_dist_;
    const rclcpp::Time watch_stamp = tight_stamp_;
    // age-compensated gap for the static-obstacle tracker: at the new
    // 4-6 m/s approach speeds, measurement age alone smears the world-arc
    // constancy check by +-0.5 m and masked genuinely parked obstacles
    const double watch_comp =
        std::isfinite(watch_norm)
            ? watch_norm -
                  speed_ * std::clamp((now() - watch_stamp).seconds(), 0.0,
                                      get_parameter("stale_blocked")
                                          .as_double())
            : watch_norm;

    // Diagnostic only: the ACC gap-follow logic below adjusts speed
    // silently (by design - a followed vehicle should not spam logs every
    // tick), so a NEW obstacle entering the tight corridor is otherwise
    // invisible in the console. Log once per distinct encounter (rising
    // edge only) so a run's full obstacle history - static props AND any
    // NPC event (merge/cross) - is visible for review/grading.
    {
      const bool present = std::isfinite(watch_norm) && watch_norm < 35.0;
      if (present && !obstacle_present_) {
        RCLCPP_INFO(get_logger(),
                    "Obstacle entered corridor: %.1f m ahead at s=%.0f m "
                    "(our speed %.1f m/s)", watch_norm, s_now, speed_);
      }
      obstacle_present_ = present;
    }

    // --- emergency stop (task 2.4.3, Event II) ------------------------
    // Fires before anything else this tick, in every mode: a vehicle that
    // crosses and brakes hard directly ahead needs an immediate full-brake
    // command computed straight from raw distance/speed, not one derived
    // through the comfort-tuned ACC curve and P-gain further down. Uses
    // the tight (swept-path) corridor, the same authoritative "would we
    // hit this" measure the rest of the collision-avoidance logic uses.
    {
      const double age_e = (now() - tight_stamp_).seconds();
      if (std::isfinite(tight_dist_) &&
          age_e <= get_parameter("stale_blocked").as_double()) {
        const double d_eff = std::max(0.05, tight_dist_ - speed_ * age_e);
        const double a_required = (speed_ * speed_) / (2.0 * d_eff);
        if (a_required > get_parameter("emergency_decel").as_double()) {
          cmd.throttle = 0.0;
          cmd.brake = 1.0;
          if (!emergency_active_)
            RCLCPP_WARN(get_logger(),
                        "EMERGENCY STOP: %.1f m ahead at %.1f m/s needs "
                        "%.1f m/s^2 - full brake", tight_dist_, speed_,
                        a_required);
          emergency_active_ = true;
          PublishCmd(cmd);
          return;
        }
      }
      emergency_active_ = false;
    }

    // --- static-obstacle state machine -------------------------------
    const double standoff = get_parameter("obstacle_standoff").as_double();
    // light phase freshness, needed both here (queue guard) and by the
    // traffic-light block below
    const double stale = get_parameter("light_stale").as_double();
    const bool red_fresh = (now() - last_red_).seconds() < stale;
    const bool green_fresh = (now() - last_green_).seconds() < stale;
    const std::vector<double>& stop_lines = stop_arcs_;
    if (RunObstacleStateMachine(s_now, standoff, watch_norm, watch_comp,
                                green_fresh, stop_lines, cmd))
      return;

    // --- lookahead + pure pursuit steering ---------------------------
    const double ld = get_parameter("lookahead_min").as_double() +
                      get_parameter("lookahead_gain").as_double() * speed_;
    const size_t tgt = IndexAtArcAhead(ld);
    const double alpha =
        std::atan2(pts[tgt].y - py, pts[tgt].x - px) - yaw;
    const double L = get_parameter("wheelbase").as_double();
    const double steer =
        std::atan2(2.0 * L * std::sin(alpha), std::max(ld, 1.0));
    // simulator convention: positive steering turns right
    double steer_cap = 1.0;
    if (mode_ == Mode::kDetour) {
      steer_cap = get_parameter("maneuver_steer_cap").as_double();
    }
    // the sim car cannot break static friction with wheels turned hard:
    // launch nearly straight, full authority returns once rolling
    if (speed_ < 0.3) {
      steer_cap = std::min(
          steer_cap, get_parameter("standstill_steer_cap").as_double());
    }
    cmd.steering = static_cast<float>(std::clamp(
        -steer / get_parameter("max_steer_rad").as_double(), -steer_cap,
        steer_cap));

    // --- longitudinal: planned speed, capped by the obstacle gap ----
    // Normal driving watches the tight corridor with the 13 m standoff;
    // threading a verified detour watches the 1.1 m corridor with 2 m.
    // Desired speed: while rolling, take the SLOWEST profile point within
    // the braking horizon so corner slowdowns start early enough for the
    // sim's laggy brakes (tracking only the nearest point overshoots
    // corner entry speed and the car understeers off the road). From
    // standstill, take the max over the next few points instead: the
    // profile has v[0]=0 at the spawn anchor and sampling only nearest_
    // would command zero throttle forever (coast = parked).
    double v_des;
    if (speed_ < 0.5) {
      v_des = 0.0;
      for (size_t i = nearest_; i < std::min(nearest_ + 4, pts.size()); ++i)
        v_des = std::max(v_des, pts[i].speed);
    } else {
      v_des = pts[nearest_].speed;
      // preview at least 12 m: a short horizon lets the car accelerate
      // between an obstacle clearing and a corner, then brake too late
      const double brake_horizon =
          std::max(12.0, speed_ * speed_ / 3.6 + 2.0);
      for (size_t i = nearest_; i < pts.size(); ++i) {
        if (arc_[i] - arc_[nearest_] > brake_horizon) break;
        v_des = std::min(v_des, std::max(pts[i].speed, 1.0));
      }
    }
    const double watch = watch_norm;
    const rclcpp::Time stamp = watch_stamp;
    double gap_ref = standoff;
    if (mode_ == Mode::kDetour) {
      // same tight corridor, but threading tolerates a 2 m gap
      v_des = std::min(v_des, get_parameter("detour_speed").as_double());
      gap_ref = get_parameter("detour_standoff").as_double();
    }
    // cautious approach: anything in the WIDE corridor - including
    // roadside furniture that never hard-blocks - caps speed by a ramp
    // over its distance, like a human easing past close objects. The
    // flat 2.5 cap this replaces was active on 60-90% of the route and
    // was the dominant time sink.
    if (mode_ != Mode::kDetour && std::isfinite(obstacle_dist_) &&
        obstacle_dist_ < get_parameter("approach_zone").as_double()) {
      // stale wide data degrades to the conservative flat cap, and the
      // distance is age-compensated exactly like the ACC gap below
      const double wide_age = (now() - obstacle_stamp_).seconds();
      const double d_eff =
          wide_age > get_parameter("stale_blocked").as_double()
              ? 0.0
              : obstacle_dist_ - speed_ * wide_age;
      const double near = get_parameter("approach_cap_near").as_double();
      const double far = get_parameter("approach_cap_far").as_double();
      const double v_lo = get_parameter("approach_speed").as_double();
      double v_hi = get_parameter("approach_cap_vmax").as_double();
      // furniture on a bend: the swept corridor rakes across facades the
      // profile cannot see as curvature - keep the historic flat cap
      if (HeadingChangeDeg(0.0, std::min(d_eff, 30.0)) >
          get_parameter("corner_zone_heading_deg").as_double()) {
        v_hi = v_lo;
      }
      const double v_cap = std::clamp(
          v_lo + (d_eff - near) * (v_hi - v_lo) / std::max(far - near, 1.0),
          v_lo, std::max(v_hi, v_lo));
      v_des = std::min(v_des, v_cap);
    }
    // set whenever a falling gap-based curve binds v_des: those must be
    // TRACKED with the brake, not coasted after - the deadband below
    // once let the car ride the ACC curve 0.4 m/s high from 4 m/s and
    // eat 7 m of the 13 m standoff
    bool stop_curve = false;
    const double age = (now() - stamp).seconds();
    if (age > get_parameter("stale_blocked").as_double()) {
      // fail safe: no fresh corridor data means blocked, never free
      v_des = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Corridor data stale (%.1f s) - stopping", age);
    } else if (std::isfinite(watch)) {
      // the measurement is age seconds old and the car kept moving since;
      // shrink the gap by the distance covered - plus the brake dead time
      // - so latency cannot eat into the standoff
      const double gap =
          watch -
          speed_ * (age + get_parameter("brake_lag").as_double()) - gap_ref;
      const double v_safe =
          gap <= 0.0
              ? 0.0
              : std::sqrt(2.0 * get_parameter("a_obstacle").as_double() *
                          gap);
      if (v_safe < v_des) {
        v_des = v_safe;
        stop_curve = true;
      }
      if (gap <= 0.3) {
        cmd.throttle = 0.0;
        cmd.brake = 1.0;
        PublishCmd(cmd);
        return;
      }
    }

    // --- traffic lights (task 2.4.1: complete stop at red) -----------
    // The detector reports the facing signal's phase; WHERE to stop is
    // route knowledge: fixed stop-line arc positions. Inside the
    // approach zone a fresh RED engages a hold at the line and only a
    // fresh GREEN releases it. With neither seen recently (amber phase,
    // brief dropouts) the current decision stands: driving stays
    // driving, waiting stays waiting - both are compliant.
    // a blind release is void the moment the signal shows red again
    // before we actually crossed (e.g. the occluding vehicle drove off)
    if (red_fresh && s_now < blind_released_line_) {
      blind_released_line_ = -1e9;
    }
    // arm at the WIDER of the hold zone and the caution zone: gating on
    // light_zone alone silently disabled the caution ramp's outer part
    const double arm_zone =
        std::max(get_parameter("light_zone").as_double(),
                 get_parameter("light_caution_zone").as_double());
    double line_s = std::numeric_limits<double>::infinity();
    size_t line_idx = 0;
    for (size_t li = 0; li < stop_lines.size(); ++li) {
      const double s_stop = stop_lines[li];
      if (std::abs(s_stop - blind_released_line_) < 0.1) continue;
      if (s_now < s_stop + 2.0 && s_stop - s_now < arm_zone) {
        line_s = s_stop;
        line_idx = li;
        break;
      }
    }
    if (std::isfinite(line_s)) {
      // per-light stop offset (value copy - see stop_lines above)
      const std::vector<double> shorts =
          get_parameter("light_stop_short").as_double_array();
      const double stop_short =
          shorts.empty() ? 6.0
                         : shorts[std::min(line_idx, shorts.size() - 1)];
      const double gap = line_s - stop_short - s_now;
      // Distance to the ANCHOR is where we would like to stop; distance to
      // the junction is what decides whether stopping is still legal to
      // begin. Those are 6-12 m apart (stop_short), and the anchor itself
      // sits ~10 m before the waiting line - so a red that resolves late
      // is still stoppable long after gap has gone negative. Gating the
      // latch on gap (as before) went red-blind for the last ~16-22 m and
      // drove through plainly red lights.
      const double to_line = line_s - s_now;
      if (red_fresh && last_red_ > last_green_ &&
          (red_hold_ ||
           (to_line > 0.5 && now() >= green_commit_until_))) {
        // never LATCH a new hold once committed: a red first seen while
        // entering the junction is either our phase flipping as we cross
        // (clear it, don't stop in it) or a cross-street head swinging
        // into view during the turn. The green_commit window covers the
        // start of that crossing: seconds after a fresh green released
        // us, the facing head cannot really be red again (shortest green
        // here is 10 s) - a "red" then is the cross-street head resolving
        // from the roll-off viewpoint, and re-latching on it once
        // stranded the car a full extra cycle
        if (!red_hold_)
          RCLCPP_INFO(get_logger(), "Red light - stopping at s=%.0f m",
                      line_s);
        red_hold_ = true;
      } else if (green_fresh && last_green_ > last_red_) {
        if (red_hold_) {
          RCLCPP_INFO(get_logger(), "Green light - proceeding");
          green_commit_until_ =
              now() + rclcpp::Duration::from_seconds(
                          get_parameter("light_green_commit").as_double());
        }
        red_hold_ = false;
      }
      if (!green_fresh && to_line > 0.0 &&
          to_line < get_parameter("light_caution_zone").as_double()) {
        // Phase unknown or red: ease toward the line on a decel ramp,
        // ceilinged at light_prox_cap, so a head that only resolves late
        // (hidden by the overpass) still leaves room to stop and the
        // 2-of-3 phase debounce always outpaces the stop curve. A flat
        // cap here was a full-brake step at zone entry. A confirmed green
        // lifts the cap entirely. It runs to the LINE, not to the stop
        // anchor: between the two the car is still 10+ m short of the
        // junction, and creeping there is exactly what keeps a late red
        // stoppable. Past the line, clearing the junction promptly is the
        // compliant behavior once the lamp leaves view.
        const double v_caution = std::clamp(
            std::sqrt(2.0 * get_parameter("light_caution_accel").as_double() *
                      std::max(gap - 4.0, 0.0)),
            get_parameter("light_caution_speed").as_double(),
            get_parameter("light_prox_cap").as_double());
        if (v_caution < v_des) {
          v_des = v_caution;
          stop_curve = true;
        }
      }
      if (red_hold_) {
        // Anything stopped ahead of us at a red is QUEUE, not scenery: it
        // has been motionless long enough for the static tracker to call
        // it parked, and the queue guard drops the moment the light turns
        // green - which used to fire a detour around the car in front on
        // the very cycle it was about to drive off. Keep the settle
        // window alive through the wait so the release stays a follow.
        obs_moving_until_ =
            now() + rclcpp::Duration::from_seconds(
                        get_parameter("traffic_settle").as_double());
        const double v_safe =
            gap <= 0.0
                ? 0.0
                : std::sqrt(2.0 * get_parameter("a_obstacle").as_double() *
                            gap);
        if (v_safe < v_des) {
          v_des = v_safe;
          stop_curve = true;
        }
        // Fail-safe for an unwinnable wait: stopped AT the line with no
        // phase visible (lamp left the camera FOV overhead). Gated on
        // (a) a full stop of blind_release seconds, anchored to when we
        // physically stopped - a fixed point, immune to being pushed
        // later - and (b) currently having no fresh phase read (age since
        // the last vote exceeds light_stale, the same short freshness
        // window used everywhere else), plus being at the line with the
        // path ahead clear.
        //
        // (b) used to be its OWN blind_release-length timer anchored to
        // max(last_red_, last_green_) instead: a single late, transient
        // vote (plausible exactly here - the head is only briefly
        // glimpse-able before going out of FOV) would re-anchor that
        // timer to a point already deep into the red phase, so the
        // required additional blind_release-second wait from THAT point
        // could overshoot the green+amber window entirely and release
        // into the START of the next red cycle - a confirmed, real defect
        // (89-agent review). Anchoring both timers the same way removes
        // that specific failure mode. It does not make the wait provably
        // safe in every case - blind_release (22s, one measured red
        // phase) is not always long enough to guarantee clearing red
        // regardless of exactly when within the phase we stopped, if the
        // red phase itself is longer than the green+amber window that
        // follows it (true at TL1: 22s red vs 14s green+amber) - a timer
        // with zero visibility fundamentally cannot guarantee this. What
        // this fix removes is the AVOIDABLE extra delay a stray late vote
        // used to add on top of that already-imperfect margin.
        if (speed_ < 0.3) {
          if (red_stop_onset_.nanoseconds() == 0) red_stop_onset_ = now();
        } else {
          red_stop_onset_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        const double vote_age =
            (now() - std::max(last_red_, last_green_)).seconds();
        const double stopped =
            red_stop_onset_.nanoseconds() > 0
                ? (now() - red_stop_onset_).seconds()
                : 0.0;
        const double release =
            get_parameter("light_blind_release").as_double();
        const bool currently_blind =
            vote_age > get_parameter("light_stale").as_double();
        const bool at_line = gap < 3.0;
        const bool path_clear =
            !std::isfinite(tight_dist_) || tight_dist_ > 20.0;
        if (currently_blind && stopped > release && at_line && path_clear) {
          RCLCPP_WARN(get_logger(),
                      "Signal not visible (%.1f s since last read) after a "
                      "%.0f s stop at the line - proceeding across s=%.0f m",
                      vote_age, stopped, line_s);
          blind_released_line_ = line_s;
          red_hold_ = false;
          red_stop_onset_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        } else if (gap <= 0.4) {
          cmd.throttle = 0.0;
          cmd.brake = 1.0;
          PublishCmd(cmd);
          return;
        }
      } else {
        red_stop_onset_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    } else {
      // no stop line armed; observability for the silent path - a hold
      // must end via green or blind release, never by falling out of the
      // arm window (run16 left one unexplained escape at s=495)
      if (red_hold_)
        RCLCPP_WARN(get_logger(),
                    "Red hold dissolved without release - line left the "
                    "arm window at s_now=%.0f m", s_now);
      red_hold_ = false;
      red_stop_onset_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    // P + hold-throttle feed-forward; brake only past a deadband (with
    // hysteresis) so speed can ride AT v_des without a throttle/brake
    // limit cycle. Stops keep the legacy law: v_des < 0.5 bypasses the
    // deadband, so the stale-corridor fail-safe and every stop endgame
    // brake exactly as before.
    const double dv = v_des - speed_;
    const bool brake_now =
        dv < 0.0 &&
        (stop_curve || v_des < 0.5 ||
         dv < -get_parameter("brake_deadband").as_double() ||
         (braking_ && dv < -get_parameter("brake_release").as_double()));
    if (brake_now) {
      cmd.brake = static_cast<float>(std::clamp(
          0.15 + get_parameter("kp_brake").as_double() * (-dv), 0.0, 1.0));
    } else if (dv >= 0.0) {
      cmd.throttle = static_cast<float>(std::clamp(
          get_parameter("kp_speed").as_double() * dv + ThrottleFF(v_des),
          0.0, 1.0));
    }
    // else: inside the deadband, COAST (both zero). Feathering with the
    // feed-forward here held speed constant on falling caps and produced
    // a 4 Hz throttle/brake limit cycle that ate 15 m of an obstacle
    // approach; coasting tracks a falling cap and merely dithers a cruise
    // a tenth below its cap.
    PublishCmd(cmd);
  }

  // Piecewise-linear hold throttle T_hold(v_des), calibrated from run
  // logs; see throttle_ff_v/t. Returns 0 at v_des = 0 (no creep).
  double ThrottleFF(double v) const {
    const auto vs = get_parameter("throttle_ff_v").as_double_array();
    const auto ts = get_parameter("throttle_ff_t").as_double_array();
    if (vs.empty() || vs.size() != ts.size()) return 0.0;
    if (v <= vs.front()) return ts.front();
    for (size_t i = 1; i < vs.size(); ++i) {
      if (v <= vs[i]) {
        return ts[i - 1] +
               (ts[i] - ts[i - 1]) * (v - vs[i - 1]) / (vs[i] - vs[i - 1]);
      }
    }
    return ts.back();
  }

  project_interfaces::msg::Trajectory traj_;
  std::vector<double> arc_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_;
  double speed_{0.0};
  double obstacle_dist_{std::numeric_limits<double>::infinity()};
  double tight_dist_{std::numeric_limits<double>::infinity()};
  double left_dist_{std::numeric_limits<double>::infinity()};
  double right_dist_{std::numeric_limits<double>::infinity()};
  rclcpp::Time obstacle_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tight_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time left_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> blocked_since_;
  double blocked_gap_{0.0};
  std::deque<std::pair<rclcpp::Time, double>> obs_track_;
  // until this time the watched obstacle is treated as settling traffic
  // (recently seen moving), never as a parked obstacle
  rclcpp::Time obs_moving_until_{0, 0, RCL_ROS_TIME};
  // brake-everything window opened by the stall watchdog
  rclcpp::Time stall_hold_until_{0, 0, RCL_ROS_TIME};
  // stop-line arcs projected from traffic_stop_xy onto the live route
  std::vector<double> stop_arcs_;
  std::optional<rclcpp::Time> stall_since_;
  rclcpp::Time hold_min_until_{0, 0, RCL_ROS_TIME};
  Mode mode_{Mode::kNormal};
  rclcpp::Time state_since_{0, 0, RCL_ROS_TIME};
  double detour_dir_{0.0};
  double s_obs_{0.0};
  bool tried_other_side_{false};
  double detour_end_{0.0};
  size_t nearest_{0};
  bool need_global_search_{true};
  bool finished_{false};
  std::deque<uint8_t> light_win_;
  rclcpp::Time last_light_msg_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_red_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_green_{0, 0, RCL_ROS_TIME};
  rclcpp::Time red_stop_onset_{0, 0, RCL_ROS_TIME};
  bool red_hold_{false};
  // crossing-commit window opened by a green release; blocks phase-
  // impossible red re-latches while rolling off the line
  rclcpp::Time green_commit_until_{0, 0, RCL_ROS_TIME};
  double blind_released_line_{-1e9};
  bool braking_{false};
  bool enabled_{true};
  bool emergency_active_{false};
  bool obstacle_present_{false};

  rclcpp::Publisher<simulation::msg::VehicleControl>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr detour_pub_;
  rclcpp::Subscription<project_interfaces::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tight_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_sub_;
  rclcpp::Subscription<project_interfaces::msg::TrafficLight>::SharedPtr
      light_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
