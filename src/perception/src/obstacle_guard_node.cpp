// Trajectory-corridor obstacle monitor.
//
// Transforms the depth-camera point cloud into the world frame and measures,
// along the UPCOMING planned trajectory, the arc distance to the nearest
// point inside each of four corridors:
//   /perception/obstacle_distance        main corridor (half-width 1.4 m)
//   /perception/tight_distance           maneuver corridor (1.1 m) - used to
//                                        verify and drive narrow passes
//   /perception/overtake_left_distance   corridor shifted left
//   /perception/overtake_right_distance  corridor shifted right
// Every point is classified against its CLOSEST path segment (no early
// exit), and arc distances use true cumulative segment lengths.
//
// Also cross-checks the accumulated OctoMap projection (height-banded to
// the curb range the live cloud's z_min filter ignores - see PublishMapTight)
// and publishes the nearest hit as /perception/map_tight_distance,
// supplementary to (never a substitute for) the live tight corridor above.

#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <project_interfaces/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ObstacleGuard : public rclcpp::Node {
public:
  ObstacleGuard()
      : Node("obstacle_guard"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    declare_parameter("corridor_half_width", 1.4);  // m, normal driving
    declare_parameter("tight_half_width", 1.1);     // m, narrow maneuvers
    declare_parameter("overtake_shift", 2.9);       // m, side corridors
    declare_parameter("look_ahead", 30.0);          // m along the path.
                                                    // 45 was tried for a
                                                    // higher cruise speed,
                                                    // but the Unity sim
                                                    // process pegs a CPU
                                                    // core at 99.5% and
                                                    // the whole perception
                                                    // chain is sim-frame-
                                                    // rate-limited (30 vs
                                                    // 45 measured the SAME
                                                    // corridor rate) - 30 m
                                                    // is the last validated
                                                    // zero-collision value
    declare_parameter("z_min", 0.35);   // m world height: above road noise
                                        // AND raised sidewalk fringes
                                        // (measured 0.25-0.27 in the bend
                                        // at s~540 - flat pavement, not an
                                        // obstacle)
    declare_parameter("z_max", 3.0);    // m world height: below overpasses
    declare_parameter("min_range", 0.2);  // m, absolute minimum
    declare_parameter("stride", 3);     // point decimation (1/stride^2)
    declare_parameter("map_stale_after", 3.0);  // s - see PublishMapTight:
                                        // octomap_server has no respawn=True
                                        // (unlike the two bridge nodes), and
                                        // a frozen map_ would otherwise keep
                                        // looking "fresh" forever just from
                                        // this node's own republish cadence

    wide_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/perception/obstacle_distance", 10);
    tight_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/perception/tight_distance", 10);
    ot_left_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/perception/overtake_left_distance", 10);
    ot_right_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/perception/overtake_right_distance", 10);
    map_tight_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/perception/map_tight_distance", 10);

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    traj_sub_ = create_subscription<project_interfaces::msg::Trajectory>(
        "/planning/trajectory", latched,
        [this](project_interfaces::msg::Trajectory::SharedPtr m) {
          // keep the tracked index across detour replans (same indexing);
          // resetting to 0 would trap the windowed search at route start
          const bool first = traj_.points.empty();
          traj_ = *m;
          RebuildArcTable();
          if (first) nearest_ = 0;
        });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/OurCar/CoM/pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { pose_ = *m; });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/pcl/points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          OnCloud(*msg);
        });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/perception/octomap/projected_map", latched,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
          map_ = *m;
          map_received_ = true;
        });
  }

private:
  void RebuildArcTable() {
    const auto& pts = traj_.points;
    arc_.assign(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i)
      arc_[i] = arc_[i - 1] + std::hypot(pts[i].x - pts[i - 1].x,
                                         pts[i].y - pts[i - 1].y);
  }

  void PublishAll(float main_d, float tight_d, float left_d, float right_d) {
    std_msgs::msg::Float32 m;
    m.data = main_d;
    wide_pub_->publish(m);
    std_msgs::msg::Float32 t;
    t.data = tight_d;
    tight_pub_->publish(t);
    std_msgs::msg::Float32 l;
    l.data = left_d;
    ot_left_pub_->publish(l);
    std_msgs::msg::Float32 r;
    r.data = right_d;
    ot_right_pub_->publish(r);
  }

  void OnCloud(const sensor_msgs::msg::PointCloud2& cloud) {
    const float inf = std::numeric_limits<float>::infinity();
    if (traj_.points.empty() || !pose_.has_value()) {
      PublishAll(inf, inf, inf, inf);
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform("world", cloud.header.frame_id,
                                      tf2::TimePointZero);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "TF world<-%s unavailable: %s",
                           cloud.header.frame_id.c_str(), e.what());
      PublishAll(inf, inf, inf, inf);
      return;
    }
    const auto& q = tf.transform.rotation;
    const double r00 = 1 - 2 * (q.y * q.y + q.z * q.z);
    const double r01 = 2 * (q.x * q.y - q.z * q.w);
    const double r02 = 2 * (q.x * q.z + q.y * q.w);
    const double r10 = 2 * (q.x * q.y + q.z * q.w);
    const double r11 = 1 - 2 * (q.x * q.x + q.z * q.z);
    const double r12 = 2 * (q.y * q.z - q.x * q.w);
    const double r20 = 2 * (q.x * q.z - q.y * q.w);
    const double r21 = 2 * (q.y * q.z + q.x * q.w);
    const double r22 = 1 - 2 * (q.x * q.x + q.y * q.y);
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const double tz = tf.transform.translation.z;

    const auto& pts = traj_.points;
    UpdateNearest();
    const double look = get_parameter("look_ahead").as_double();
    size_t end = nearest_;
    while (end + 1 < pts.size() && arc_[end] - arc_[nearest_] < look) ++end;
    if (end - nearest_ < 2) {
      PublishAll(inf, inf, inf, inf);
      return;
    }

    const double half_w = get_parameter("corridor_half_width").as_double();
    const double tight_w = get_parameter("tight_half_width").as_double();
    const double shift = get_parameter("overtake_shift").as_double();
    const double z_lo = get_parameter("z_min").as_double();
    const double z_hi = get_parameter("z_max").as_double();
    const double rng_min = get_parameter("min_range").as_double();
    const int stride = static_cast<int>(get_parameter("stride").as_int());

    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"),
        iz(cloud, "z");
    float main_b = inf, tight_b = inf, left_b = inf, right_b = inf;
    const int row_px = static_cast<int>(cloud.width);
    int n = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz, ++n) {
      if (stride > 1) {
        const int r = n / row_px, c = n % row_px;
        if (r % stride || c % stride) continue;
      }
      const float cx = *ix, cy = *iy, cz = *iz;
      if (!std::isfinite(cz) || cz < rng_min) continue;
      // own hood: close ahead, clearly below the camera, AND within the
      // car's half width - without the lateral bound this discarded every
      // low point across the full image width inside 2.6 m, blinding the
      // corridors exactly where detour threading operates (2 m gaps)
      if (cz < 2.6f && cy > 0.2f && std::abs(cx) < 1.0f) continue;
      const double wx = r00 * cx + r01 * cy + r02 * cz + tx;
      const double wy = r10 * cx + r11 * cy + r12 * cz + ty;
      const double wz = r20 * cx + r21 * cy + r22 * cz + tz;
      if (wz < z_lo || wz > z_hi) continue;

      // Classify against the CLOSEST segment in the window. Coarse-to-fine:
      // the distance-to-segment profile along the path is smooth, so a
      // scan every kCoarse segments brackets the true minimum, and a
      // +-kCoarse refinement finds it. Scanning all ~23 segments per point
      // for every point of the cloud pushed the node past 1.5 s per frame
      // once look_ahead grew to 45 m - the controller then saw nothing but
      // stale corridors and crawled the whole route.
      constexpr size_t kCoarse = 4;
      double best_d2 = 1e18, best_lat = 0.0, best_arc = 0.0;
      size_t best_i = end;
      for (size_t i = nearest_; i < end; i += kCoarse) {
        const double ax = pts[i].x, ay = pts[i].y;
        const double sx = pts[i + 1].x - ax, sy = pts[i + 1].y - ay;
        const double L2 = sx * sx + sy * sy;
        if (L2 < 1e-9) continue;
        double t = std::clamp(((wx - ax) * sx + (wy - ay) * sy) / L2, 0.0,
                              1.0);
        const double rx = wx - (ax + t * sx), ry = wy - (ay + t * sy);
        const double d2 = rx * rx + ry * ry;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_i = i;
        }
      }
      if (best_i == end) continue;
      const size_t lo =
          best_i > nearest_ + kCoarse ? best_i - kCoarse : nearest_;
      const size_t hi = std::min(best_i + kCoarse + 1, end);
      best_d2 = 1e18;
      for (size_t i = lo; i < hi; ++i) {
        const double ax = pts[i].x, ay = pts[i].y;
        const double sx = pts[i + 1].x - ax, sy = pts[i + 1].y - ay;
        const double L2 = sx * sx + sy * sy;
        if (L2 < 1e-9) continue;
        const double L = std::sqrt(L2);
        double t = std::clamp(((wx - ax) * sx + (wy - ay) * sy) / L2, 0.0,
                              1.0);
        const double rx = wx - (ax + t * sx), ry = wy - (ay + t * sy);
        const double d2 = rx * rx + ry * ry;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_lat = (sx * ry - sy * rx) / L;  // signed, + = left
          best_arc = (arc_[i] - arc_[nearest_]) + t * L;
        }
      }
      if (best_d2 > 1e17) continue;
      const float a = static_cast<float>(best_arc);
      const double lat = best_lat;
      if (std::abs(lat) < tight_w && a < tight_b) tight_b = a;
      if (std::abs(lat) < half_w) {
        if (a < main_b) main_b = a;
      } else if (std::abs(lat - shift) < half_w) {
        if (a < left_b) left_b = a;
      } else if (std::abs(lat + shift) < half_w) {
        if (a < right_b) right_b = a;
      }
    }
    PublishAll(main_b, tight_b, left_b, right_b);
    PublishMapTight(nearest_, end);
  }

  // Cross-check the accumulated OctoMap projection (height-banded to
  // 0.08-0.35 m in the launch file - see the comment there) for low
  // obstacles the live corridor's z_min=0.35 filter deliberately ignores
  // (curbs measured 0.25-0.27 m, see the z_min comment above). This is
  // SUPPLEMENTARY ONLY: an infinite or unavailable result must never by
  // itself block driving - it can only ADD caution on top of the live
  // tight-corridor check (see kVerify in control/pure_pursuit_node.cpp,
  // which ANDs this in, never ORs it out).
  //
  // Freshness is checked against map_.header.stamp (the underlying scan
  // time octomap_server itself sets), NOT just "have we ever received a
  // message" - octomap_server has no respawn=True, so if it dies or hangs,
  // map_received_ alone would stay true forever and this node would keep
  // republishing a verdict against a frozen snapshot every tick, making a
  // permanently-stale map look "fresh" to every downstream staleness check
  // that only looks at OUR publish timestamp. That would defeat the whole
  // fail-open design and could turn one stale/false-positive occupied cell
  // into a permanent, un-clearable detour veto (kVerify -> both sides
  // blocked -> kHold -> kNormal -> EngageDetour -> kVerify -> blocked again
  // by the same frozen cell, forever). Checking the scan's OWN timestamp
  // instead closes that: a dead/hung octomap_server degrades this signal
  // to "no data" (+inf) like any other absent input.
  void PublishMapTight(size_t lo, size_t hi) {
    std_msgs::msg::Float32 out;
    out.data = std::numeric_limits<float>::infinity();
    const double stale_after = get_parameter("map_stale_after").as_double();
    const bool map_data_fresh =
        map_received_ &&
        (now() - rclcpp::Time(map_.header.stamp)).seconds() <= stale_after;
    if (map_data_fresh && hi > lo + 1 && hi <= traj_.points.size() &&
        map_.info.resolution > 1e-6) {
      const auto& pts = traj_.points;
      const double tight_w = get_parameter("tight_half_width").as_double();
      const double res = map_.info.resolution;
      const double ox = map_.info.origin.position.x;
      const double oy = map_.info.origin.position.y;
      const int w = static_cast<int>(map_.info.width);
      const int h = static_cast<int>(map_.info.height);

      // Bounding box of the trajectory window, padded by the tight
      // half-width - scanning only this box (not the whole accumulated
      // map) keeps this cheap regardless of how large the map has grown.
      double bx_lo = pts[lo].x, bx_hi = pts[lo].x;
      double by_lo = pts[lo].y, by_hi = pts[lo].y;
      for (size_t i = lo; i <= hi; ++i) {
        bx_lo = std::min(bx_lo, pts[i].x);
        bx_hi = std::max(bx_hi, pts[i].x);
        by_lo = std::min(by_lo, pts[i].y);
        by_hi = std::max(by_hi, pts[i].y);
      }
      bx_lo -= tight_w + res;
      bx_hi += tight_w + res;
      by_lo -= tight_w + res;
      by_hi += tight_w + res;
      const int col_lo = std::max(0, static_cast<int>((bx_lo - ox) / res));
      const int col_hi =
          std::min(w - 1, static_cast<int>((bx_hi - ox) / res));
      const int row_lo = std::max(0, static_cast<int>((by_lo - oy) / res));
      const int row_hi =
          std::min(h - 1, static_cast<int>((by_hi - oy) / res));

      double best_arc = 1e18;
      for (int row = row_lo; row <= row_hi; ++row) {
        for (int col = col_lo; col <= col_hi; ++col) {
          const size_t idx = static_cast<size_t>(row) * w + col;
          if (idx >= map_.data.size()) continue;
          if (map_.data[idx] < 50) continue;  // free or unknown (-1)
          const double wx = ox + (col + 0.5) * res;
          const double wy = oy + (row + 0.5) * res;

          // Same closest-segment classification as the live cloud check,
          // over the same [lo, hi) window - no coarse-to-fine needed here
          // since only OCCUPIED cells reach this inner loop, and
          // ground/speckle filtering keeps that count small.
          double best_d2 = 1e18, best_lat = 0.0, best_seg_arc = 0.0;
          for (size_t i = lo; i < hi; ++i) {
            const double ax = pts[i].x, ay = pts[i].y;
            const double sx = pts[i + 1].x - ax, sy = pts[i + 1].y - ay;
            const double L2 = sx * sx + sy * sy;
            if (L2 < 1e-9) continue;
            const double L = std::sqrt(L2);
            double t = std::clamp(
                ((wx - ax) * sx + (wy - ay) * sy) / L2, 0.0, 1.0);
            const double rx = wx - (ax + t * sx), ry = wy - (ay + t * sy);
            const double d2 = rx * rx + ry * ry;
            if (d2 < best_d2) {
              best_d2 = d2;
              best_lat = (sx * ry - sy * rx) / L;
              best_seg_arc = (arc_[i] - arc_[lo]) + t * L;
            }
          }
          if (best_d2 > 1e17) continue;
          if (std::abs(best_lat) < tight_w && best_seg_arc < best_arc)
            best_arc = best_seg_arc;
        }
      }
      if (best_arc < 1e17) out.data = static_cast<float>(best_arc);
    }
    map_tight_pub_->publish(out);
  }

  void UpdateNearest() {
    const auto& pts = traj_.points;
    const double px = pose_->pose.position.x;
    const double py = pose_->pose.position.y;
    size_t lo = nearest_ > 25 ? nearest_ - 25 : 0;
    size_t hi = std::min(nearest_ + 50, pts.size());
    double best = 1e18;
    for (size_t i = lo; i < hi; ++i) {
      const double d = std::hypot(pts[i].x - px, pts[i].y - py);
      if (d < best) {
        best = d;
        nearest_ = i;
      }
    }
  }

  project_interfaces::msg::Trajectory traj_;
  std::vector<double> arc_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_;
  size_t nearest_{0};
  nav_msgs::msg::OccupancyGrid map_;
  bool map_received_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wide_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tight_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ot_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ot_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_tight_pub_;
  rclcpp::Subscription<project_interfaces::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleGuard>());
  rclcpp::shutdown();
  return 0;
}
