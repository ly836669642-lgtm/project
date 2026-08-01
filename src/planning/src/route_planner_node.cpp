// Route planner: loads the recorded track waypoints, applies a right-hand
// lane offset, smooths them, and attaches a curvature- and acceleration-
// limited speed profile. Also continuously selects the next short-term
// goal from the task's predefined pose list (Fig. 4 spawn positions) as
// the vehicle advances - see SelectGoal(). The result is published
// latched as
//   /planning/trajectory   (project_interfaces/Trajectory, for the controller)
//   /planning/route        (nav_msgs/Path, for RViz)
//   /planning/current_goal (geometry_msgs/PoseStamped, live goal selection)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <project_interfaces/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <limits>

struct Pt {
  double x{0.0};
  double y{0.0};
};

class RoutePlanner : public rclcpp::Node {
public:
  RoutePlanner() : Node("route_planner") {
    const auto default_csv =
        ament_index_cpp::get_package_share_directory("planning") +
        "/data/waypoints.csv";
    declare_parameter("waypoints_file", default_csv);
    // Right-hand traffic. The recorded waypoints trace the LEFT lane, so
    // the route is shifted right - but by how much the road actually
    // allows, measured per stretch by driving it:
    //   [s0, s1, offset_m] triples; anything not listed stays centered.
    //   0-122   wide boulevard, a full 2.9 m lane pitch fits (proven)
    //   252-358 narrower street: 2.9 rode the car onto the right curb
    //           (wedged three times); the drivable half-width allows ~1.5
    //   else    barrier alleys, the s~380 turn (inside edge is unmodeled
    //           void - the car fell off the world), junction-dense tail:
    //           centered, the recorded line IS the drivable lane there
    // Curbs sit ~0.25 m high, BELOW the corridor guard's z_min, so no
    // sensor can veto a bad offset here - only driven evidence can.
    declare_parameter("lane_offset_zones",
                      std::vector<double>{0.0, 122.0, 2.9,
                                          252.0, 358.0, 1.5});
    declare_parameter("lane_offset_scale", 1.0);  // global multiplier;
                                                  // 0 drives the recorded
                                                  // line for A/B runs
    declare_parameter("offset_taper", 20.0);  // m ramp in/out of a zone:
                                              // 8 m demanded a 2.9 m swing
                                              // in ~1.2 s at street speed -
                                              // pure pursuit overshot it
                                              // onto the curb (s~275)
    declare_parameter("v_max", 9.5);          // m/s. 9.5 combined with a
                                              // wider look_ahead/coarser
                                              // stride wedged the car twice
                                              // - but corridor publish rate
                                              // measured IDENTICAL at
                                              // look_ahead 45/30/20 m (the
                                              // Unity sim process itself
                                              // pegs a CPU core, not our
                                              // per-frame compute), and the
                                              // staleness gate is a WALL-
                                              // CLOCK threshold independent
                                              // of speed - so a faster car
                                              // is not exposed to more
                                              // stale data, only to less
                                              // margin per update. Stepping
                                              // v_max up alone, keeping the
                                              // validated look_ahead=30/
                                              // stride=3 fixed, isolates
                                              // that variable. 8.0 was the
                                              // last full-route zero-
                                              // collision run (run32).
    declare_parameter("a_lat_max", 1.5);      // m/s^2, comfort limit in curves
    declare_parameter("sharp_kappa", 0.07);   // 1/m; above this the lane
                                              // offset shrinks to zero so
                                              // tight junction turns stay on
                                              // the drawn line. 0.07 starts
                                              // the fade on the corner
                                              // APPROACH clothoid, not just
                                              // the apex - a partially
                                              // shifted approach is what cut
                                              // the s~380 inside edge
    declare_parameter("a_accel", 2.5);        // m/s^2
    declare_parameter("a_decel", 3.0);        // m/s^2
    declare_parameter("smooth_iterations", 3);
    // Short-term goal selection (task 2.3.1): the only predefined-pose
    // list given anywhere in the task materials is Fig. 4's "Available
    // spawn positions" (world coords, measured at Milestone 2 by spawning
    // at every index and reading /OurCar/CoM/pose). The planner does not
    // hardcode which one is "next" - GoalArcs() below projects each onto
    // the live route's arc length, and SelectGoal() picks the nearest one
    // still ahead of the vehicle's current arc position, re-evaluated on
    // every pose update as the car advances through the list.
    declare_parameter("goal_poses",
                      std::vector<double>{
                          -62.80, -12.29,
                          -49.0,   52.4,
                           70.3,   52.7,
                          141.6,   52.9,
                          229.3,   16.0,
                          141.4,    3.3,
                          122.4,    9.1,
                          103.1,   52.3,
                           46.0,   26.5,
                           39.0,    3.5});

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    traj_pub_ =
        create_publisher<project_interfaces::msg::Trajectory>(
            "/planning/trajectory", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/route", qos);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/planning/current_goal", rclcpp::QoS(1).transient_local().reliable());
    detour_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/planning/detour", 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
          OnDetour(*m);
        });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/OurCar/CoM/pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          const auto& q = m->pose.orientation;
          const double yaw =
              std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          SelectGoal(m->pose.position.x, m->pose.position.y, yaw);
        });

    Plan();
  }

private:
  void Plan() {
    auto pts = LoadWaypoints(
        get_parameter("waypoints_file").as_string());
    if (pts.size() < 5) {
      RCLCPP_FATAL(get_logger(), "Not enough waypoints (%zu)", pts.size());
      return;
    }
    ApplyLaneOffset(&pts);
    Smooth(&pts, get_parameter("smooth_iterations").as_int());
    base_pts_ = pts;
    RebuildBaseArc();
    GoalArcs();
    Publish(base_pts_);
  }

  void RebuildBaseArc() {
    base_arc_.assign(base_pts_.size(), 0.0);
    for (size_t i = 1; i < base_pts_.size(); ++i)
      base_arc_[i] = base_arc_[i - 1] +
                    std::hypot(base_pts_[i].x - base_pts_[i - 1].x,
                              base_pts_[i].y - base_pts_[i - 1].y);
  }

  // Project every predefined pose onto the live base route's arc length.
  // Re-run whenever the base route changes (only at startup today - the
  // base route itself never changes after Plan(), only detour shifts do,
  // and those stay within a short local window that does not move the
  // goal poses' nearest points).
  void GoalArcs() {
    const std::vector<double> xy = get_parameter("goal_poses").as_double_array();
    goal_arcs_.clear();
    if (base_pts_.empty()) return;
    for (size_t k = 0; k + 1 < xy.size(); k += 2) {
      size_t best = 0;
      double best_d = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < base_pts_.size(); ++i) {
        const double d = std::hypot(base_pts_[i].x - xy[k],
                                    base_pts_[i].y - xy[k + 1]);
        if (d < best_d) { best_d = d; best = i; }
      }
      goal_arcs_.push_back(base_arc_[best]);
    }
    std::sort(goal_arcs_.begin(), goal_arcs_.end());
  }

  // Select the next predefined pose still ahead of the vehicle and
  // publish it (only on change, so this is a live topic, not a per-tick
  // log flood). The route is an out-and-back loop that passes close to
  // itself in world-space at several points, so a plain per-message
  // global nearest-point search flip-flops between the two overlapping
  // legs - the same failure mode the controller's UpdateNearest already
  // solved. Same fix here: one heading-filtered global search to lock
  // on, then a narrow local window around the last result.
  void SelectGoal(double px, double py, double yaw) {
    if (base_pts_.empty() || goal_arcs_.empty()) return;
    size_t lo = 0, hi = base_pts_.size();
    if (!goal_search_global_) {
      lo = last_nearest_ > 25 ? last_nearest_ - 25 : 0;
      hi = std::min(last_nearest_ + 50, base_pts_.size());
    }
    size_t nearest = last_nearest_;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = lo; i < hi; ++i) {
      if (goal_search_global_) {
        const size_t a = i > 1 ? i - 1 : 0;
        const size_t b = std::min(i + 1, base_pts_.size() - 1);
        const double heading =
            std::atan2(base_pts_[b].y - base_pts_[a].y,
                      base_pts_[b].x - base_pts_[a].x);
        const double diff = std::remainder(heading - yaw, 2.0 * M_PI);
        if (std::abs(diff) > M_PI / 2.0) continue;
      }
      const double d = std::hypot(base_pts_[i].x - px, base_pts_[i].y - py);
      if (d < best_d) { best_d = d; nearest = i; }
    }
    last_nearest_ = nearest;
    goal_search_global_ = false;
    const double s_now = base_arc_[nearest];
    double goal_s = goal_arcs_.back();
    for (double g : goal_arcs_) {
      if (g > s_now + 1.0) { goal_s = g; break; }
    }
    if (std::abs(goal_s - last_goal_s_) < 0.5) return;
    last_goal_s_ = goal_s;
    size_t gi = 0;
    double gd = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < base_arc_.size(); ++i) {
      const double d = std::abs(base_arc_[i] - goal_s);
      if (d < gd) { gd = d; gi = i; }
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.header.stamp = now();
    ps.pose.position.x = base_pts_[gi].x;
    ps.pose.position.y = base_pts_[gi].y;
    ps.pose.orientation.w = 1.0;
    goal_pub_->publish(ps);
    RCLCPP_INFO(get_logger(),
                "Short-term goal selected: arc s=%.0f m (%.1f, %.1f)",
                goal_s, base_pts_[gi].x, base_pts_[gi].y);
  }

  // Local replan requested by the controller: shift the base route
  // laterally inside [s_start, s_end] (taper lengths from the message,
  // typically 6-12 m in / 4 m out; 10 m is only the fallback) so the
  // published trajectory itself goes around a static obstacle.
  void OnDetour(const std_msgs::msg::Float32MultiArray& m) {
    const auto t_start = std::chrono::steady_clock::now();
    if (base_pts_.empty()) return;
    if (m.data.size() < 3 || m.data[1] - m.data[0] < 1.0f) {
      RCLCPP_INFO(get_logger(), "Detour cleared, restoring base route");
      Publish(base_pts_, "detour-clear");
      return;
    }
    const double s0 = m.data[0], s1 = m.data[1], off = m.data[2];
    const double rin = m.data.size() > 3 ? m.data[3] : 10.0;
    const double rout = m.data.size() > 4 ? m.data[4] : 10.0;
    std::vector<Pt> pts = base_pts_;
    // base_arc_ is sorted (strictly increasing) and fixed, so the window
    // bounds are a binary search into it instead of a linear scan that
    // touches every point in the route to find the ones inside [s0, s1].
    const auto lo = std::lower_bound(base_arc_.begin(), base_arc_.end(), s0);
    const auto hi = std::upper_bound(base_arc_.begin(), base_arc_.end(), s1);
    for (auto it = lo; it != hi; ++it) {
      const size_t i = static_cast<size_t>(it - base_arc_.begin());
      const double ramp = std::clamp(
          std::min({(base_arc_[i] - s0) / std::max(rin, 1.0), 1.0,
                    (s1 - base_arc_[i]) / std::max(rout, 1.0)}),
          0.0, 1.0);
      const Pt& a = base_pts_[i == 0 ? 0 : i - 1];
      const Pt& b = base_pts_[std::min(i + 1, base_pts_.size() - 1)];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double n = std::hypot(dx, dy);
      if (n < 1e-6) continue;
      // left normal of (dx,dy) is (-dy,dx); off > 0 shifts left
      pts[i].x += -dy / n * off * ramp;
      pts[i].y += dx / n * off * ramp;
    }
    // Own cost of this callback before Publish(): binary search + the
    // window shift loop, now O(log n + k) with k = points inside [s0, s1]
    // instead of the old O(n) arc rebuild + full scan (see Publish() for
    // the remaining O(n) curvature/speed-profile/message-build cost).
    const double window_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - t_start)
                                 .count();
    RCLCPP_INFO(get_logger(),
                "Replanned with detour: s=[%.0f, %.0f] offset %+.1f m "
                "(window search + shift: %.3f ms over %ld/%zu points)",
                s0, s1, off, window_ms, static_cast<long>(hi - lo),
                pts.size());
    Publish(pts, "detour");
  }

  std::vector<Pt> LoadWaypoints(const std::string& file) {
    std::vector<Pt> pts;
    std::ifstream in(file);
    if (!in) {
      RCLCPP_FATAL(get_logger(), "Cannot open waypoints file: %s",
                   file.c_str());
      return pts;
    }
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::stringstream ss(line);
      Pt p;
      char comma;
      if (ss >> p.x >> comma >> p.y) pts.push_back(p);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", pts.size(),
                file.c_str());
    return pts;
  }

  // Shift every point perpendicular to the local path direction; positive
  // offset moves it to the right of the driving direction (right-hand
  // lane). The amount comes from lane_offset_zones and ramps in/out over
  // offset_taper so the lateral swing stays inside the tyres' grip at
  // cruise. In sharp turns the offset fades to zero as well, otherwise a
  // right offset tightens right turns into the inside sidewalk.
  void ApplyLaneOffset(std::vector<Pt>* pts) {
    // value copies: iterating the Parameter temporary's array directly is
    // a dangling reference (it cost us the first stop line for six runs)
    const std::vector<double> zones =
        get_parameter("lane_offset_zones").as_double_array();
    const double gain = get_parameter("lane_offset_scale").as_double();
    if (zones.size() < 3 || std::abs(gain) < 1e-6) return;
    const double sharp = get_parameter("sharp_kappa").as_double();
    const double taper =
        std::max(get_parameter("offset_taper").as_double(), 1.0);
    std::vector<double> arc(pts->size(), 0.0);
    for (size_t i = 1; i < pts->size(); ++i)
      arc[i] = arc[i - 1] + std::hypot((*pts)[i].x - (*pts)[i - 1].x,
                                       (*pts)[i].y - (*pts)[i - 1].y);
    std::vector<Pt> out(pts->size());
    const auto& p = *pts;
    for (size_t i = 0; i < p.size(); ++i) {
      const Pt& a = p[i == 0 ? 0 : i - 1];
      const Pt& b = p[std::min(i + 1, p.size() - 1)];
      double dx = b.x - a.x, dy = b.y - a.y;
      double n = std::hypot(dx, dy);
      if (n < 1e-6) { out[i] = p[i]; continue; }
      // strongest applicable zone offset, ramped by distance from its ends
      double off = 0.0;
      for (size_t z = 0; z + 2 < zones.size(); z += 3) {
        if (arc[i] < zones[z] || arc[i] > zones[z + 1]) continue;
        const double edge =
            std::min(arc[i] - zones[z], zones[z + 1] - arc[i]);
        const double ramp = std::clamp(edge / taper, 0.0, 1.0);
        if (std::abs(zones[z + 2] * ramp) > std::abs(off))
          off = zones[z + 2] * ramp;
      }
      if (std::abs(off) < 1e-6) { out[i] = p[i]; continue; }
      double k = 0.0;
      if (i >= 1 && i + 1 < p.size()) k = Curvature(p[i - 1], p[i], p[i + 1]);
      off *= gain * std::clamp(1.0 - k / sharp, 0.0, 1.0);
      // right normal of (dx,dy) is (dy,-dx)
      out[i] = {p[i].x + off * dy / n, p[i].y - off * dx / n};
    }
    *pts = std::move(out);
  }

  void Smooth(std::vector<Pt>* pts, int iterations) {
    for (int it = 0; it < iterations; ++it) {
      std::vector<Pt> out = *pts;
      for (size_t i = 1; i + 1 < pts->size(); ++i) {
        out[i].x = 0.25 * (*pts)[i - 1].x + 0.5 * (*pts)[i].x +
                   0.25 * (*pts)[i + 1].x;
        out[i].y = 0.25 * (*pts)[i - 1].y + 0.5 * (*pts)[i].y +
                   0.25 * (*pts)[i + 1].y;
      }
      *pts = std::move(out);
    }
  }

  // Unsigned Menger curvature through three points.
  static double Curvature(const Pt& a, const Pt& b, const Pt& c) {
    double area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    double la = std::hypot(b.x - a.x, b.y - a.y);
    double lb = std::hypot(c.x - b.x, c.y - b.y);
    double lc = std::hypot(c.x - a.x, c.y - a.y);
    double denom = la * lb * lc;
    return denom < 1e-9 ? 0.0 : 2.0 * std::abs(area2) / denom;
  }

  void Publish(const std::vector<Pt>& pts, const char* reason = "startup") {
    const auto t_start = std::chrono::steady_clock::now();
    const double v_max = get_parameter("v_max").as_double();
    const double a_lat = get_parameter("a_lat_max").as_double();
    const double a_acc = get_parameter("a_accel").as_double();
    const double a_dec = get_parameter("a_decel").as_double();
    const size_t n = pts.size();

    // curvature: +-1 sample span (4 m chord) keeps sharp junction corners
    // sharp instead of smearing them flat; take the max with the +-2 span
    // so both tight corners and long curves register
    std::vector<double> kappa(n, 0.0);
    for (size_t i = 2; i + 2 < n; ++i)
      kappa[i] = std::max(Curvature(pts[i - 1], pts[i], pts[i + 1]),
                          Curvature(pts[i - 2], pts[i], pts[i + 2]));

    // curve speed limit, then backward (braking) and forward (accel) passes
    std::vector<double> v(n, v_max);
    for (size_t i = 0; i < n; ++i)
      if (kappa[i] > 1e-6)
        v[i] = std::min(v_max, std::sqrt(a_lat / kappa[i]));
    v[n - 1] = 0.0;  // stop at the goal
    for (size_t i = n - 1; i > 0; --i) {
      double ds = std::hypot(pts[i].x - pts[i - 1].x,
                             pts[i].y - pts[i - 1].y);
      v[i - 1] = std::min(v[i - 1],
                          std::sqrt(v[i] * v[i] + 2.0 * a_dec * ds));
    }
    v[0] = 0.0;  // start from standstill
    for (size_t i = 0; i + 1 < n; ++i) {
      double ds = std::hypot(pts[i + 1].x - pts[i].x,
                             pts[i + 1].y - pts[i].y);
      v[i + 1] = std::min(v[i + 1],
                          std::sqrt(v[i] * v[i] + 2.0 * a_acc * ds));
    }
    // v[0]=0 exists only to seed the acceleration pass. Publishing it makes
    // the very first trajectory point demand a full stop, which the rolling
    // controller answers with a brake pulse while it is still on index 0 -
    // a throttle/brake limit cycle at every launch. The car starts at rest
    // anyway, so hand it the speed it is allowed to reach.
    if (n > 1) v[0] = v[1];

    project_interfaces::msg::Trajectory traj;
    nav_msgs::msg::Path path;
    traj.header.frame_id = path.header.frame_id = "world";
    traj.header.stamp = path.header.stamp = now();
    for (size_t i = 0; i < n; ++i) {
      const Pt& nb = pts[std::min(i + 1, n - 1)];
      const Pt& pv = pts[i == 0 ? 0 : i - 1];
      project_interfaces::msg::TrajectoryPoint tp;
      tp.x = pts[i].x;
      tp.y = pts[i].y;
      tp.yaw = std::atan2(nb.y - pv.y, nb.x - pv.x);
      tp.curvature = kappa[i];
      tp.speed = v[i];
      traj.points.push_back(tp);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "world";
      ps.pose.position.x = pts[i].x;
      ps.pose.position.y = pts[i].y;
      ps.pose.position.z = 0.3;
      ps.pose.orientation.z = std::sin(tp.yaw / 2.0);
      ps.pose.orientation.w = std::cos(tp.yaw / 2.0);
      path.poses.push_back(ps);
    }
    traj_pub_->publish(traj);
    path_pub_->publish(path);
    const double publish_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t_start)
                                  .count();
    RCLCPP_INFO(get_logger(),
                "Published trajectory (%s): %zu points, v_max %.1f m/s, "
                "%.3f ms",
                reason, n, v_max, publish_ms);
  }

  std::vector<Pt> base_pts_;
  std::vector<double> base_arc_;
  std::vector<double> goal_arcs_;
  double last_goal_s_{-1.0};
  size_t last_nearest_{0};
  bool goal_search_global_{true};
  rclcpp::Publisher<project_interfaces::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      detour_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoutePlanner>());
  rclcpp::shutdown();
  return 0;
}
