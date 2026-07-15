// Traffic light detection from the front RGB camera, HSV blob analysis.
//
// The task requires a complete stop at red lights (task sheet 2.4.1). This
// node classifies the lit lamp of the facing signal head and publishes a
// project_interfaces/TrafficLight state; the controller decides where to
// stop (it knows the stop-line arc positions along the route).
//
// Deliberately NOT using the semantic camera (bonus criterion): thresholds
// below were calibrated from RGB frames captured along the track.
//
// Only RED and GREEN are detected. The amber phase is skipped on purpose:
// the signal casings are amber-painted metal and cannot be separated from
// a lit amber lamp reliably. The controller treats "no fresh green" as
// "keep waiting" while stopped, which is compliant behavior.

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "project_interfaces/msg/traffic_light.hpp"

namespace {

struct Blob {
  int area{0};
  cv::Rect box;
  cv::Point2d center;
};

// Connected components filtered to look like a lit circular lamp:
// compact (bounding box roughly square, well filled), small but not tiny.
// Rejects the red ring of no-U-turn signs (hollow -> low fill), red shop
// banners (elongated), and green highway boards (too large).
std::vector<Blob> LampBlobs(const cv::Mat &mask, int min_area) {
  std::vector<Blob> out;
  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(mask, labels, stats,
                                                 centroids, 8, CV_32S);
  for (int i = 1; i < n; ++i) {
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < min_area || area > 800) continue;
    const cv::Rect box(stats.at<int>(i, cv::CC_STAT_LEFT),
                       stats.at<int>(i, cv::CC_STAT_TOP),
                       stats.at<int>(i, cv::CC_STAT_WIDTH),
                       stats.at<int>(i, cv::CC_STAT_HEIGHT));
    const double aspect = static_cast<double>(box.width) / box.height;
    if (aspect < 0.4 || aspect > 2.5) continue;
    const double fill = static_cast<double>(area) / (box.width * box.height);
    if (fill < 0.45) continue;
    out.push_back({area, box,
                   {centroids.at<double>(i, 0), centroids.at<double>(i, 1)}});
  }
  return out;
}

}  // namespace

class TrafficLightNode : public rclcpp::Node {
public:
  TrafficLightNode() : Node("traffic_light") {
    // Signal heads hang 4-6 m high and stay above the horizon in the
    // image; cars, banners and shopfronts stay below. Calibrated from
    // captured frames (lamps observed at y 1..134 over 8..30 m range).
    declare_parameter("roi_height_px", 170);
    declare_parameter("min_area_px", 4);  // the overpass-hung lamps are
                                          // only ~5 px from their stop
                                          // poses; 6 rejected them and
                                          // every red wait went phase-
                                          // blind. Ring-sign fragments
                                          // (4-6 px) are outvoted by the
                                          // 2x green-dominance rule.
    // Facing signals appear at cx 0.18..0.65 on approach; CROSS-street
    // heads seen at a shallow angle sit at the image edges (measured
    // 0.13 at the first intersection, red while ours was green).
    declare_parameter("cx_min", 0.15);
    declare_parameter("cx_max", 0.85);

    pub_ = create_publisher<project_interfaces::msg::TrafficLight>(
        "/perception/traffic_light", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/OurCar/Sensors/RGBCameraLeft/image_raw",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnImage(msg); });
    RCLCPP_INFO(get_logger(), "Traffic light detector up (RGB HSV)");
  }

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImageConstPtr cv;
    try {
      cv = cv_bridge::toCvShare(msg, "rgb8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cv_bridge: %s", e.what());
      return;
    }
    const int roi_h = std::min<int>(
        get_parameter("roi_height_px").as_int(), cv->image.rows);
    const cv::Mat roi = cv->image(cv::Range(0, roi_h), cv::Range::all());
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_RGB2HSV);

    // Lit-lamp thresholds measured from track frames:
    //   red lamp   H 0..7 (wraps at 174+), S>=170, V>=229 observed
    //   green lamp H 51..63, S>=123, V>=252 observed
    //   red ring signs S 101..134 -> S floor 140 rejects them
    //   green road boards V<=206 mostly -> V floor 225 rejects them
    cv::Mat red_lo, red_hi, red, green;
    cv::inRange(hsv, cv::Scalar(0, 140, 190), cv::Scalar(6, 255, 255),
                red_lo);
    cv::inRange(hsv, cv::Scalar(174, 140, 190), cv::Scalar(179, 255, 255),
                red_hi);
    cv::bitwise_or(red_lo, red_hi, red);
    cv::inRange(hsv, cv::Scalar(45, 120, 225), cv::Scalar(70, 255, 255),
                green);

    const int min_area =
        static_cast<int>(get_parameter("min_area_px").as_int());
    const double cx_min = get_parameter("cx_min").as_double() * msg->width;
    const double cx_max = get_parameter("cx_max").as_double() * msg->width;
    auto pick = [&](const std::vector<Blob> &blobs) -> const Blob * {
      const Blob *best = nullptr;
      for (const auto &b : blobs) {
        if (b.center.x < cx_min || b.center.x > cx_max) continue;
        if (!best || b.area > best->area) best = &b;
      }
      return best;
    };
    // named locals: pick() returns a pointer into the vector, which must
    // outlive every use below (a temporary here left best_* dangling and
    // produced garbage phases until the controller's debounce ate them)
    const auto red_blobs = LampBlobs(red, min_area);
    const auto green_blobs = LampBlobs(green, min_area);
    const Blob *best_red = pick(red_blobs);
    const Blob *best_green = pick(green_blobs);

    project_interfaces::msg::TrafficLight out;
    out.header = msg->header;
    out.state = project_interfaces::msg::TrafficLight::NONE;

    // Prefer RED unless the green blob clearly dominates (2x) AND sits in
    // the facing position: mistaking leftover red-ring sign fragments for
    // the phase costs a needless stop; missing a red lamp runs the light;
    // a big green blob picked up from a cross-street head must not flip
    // the phase just because it out-sizes a facing red somewhere else in
    // frame (this let a cross-street green re-latch our own red as green
    // once - the position gate below closes that).
    //
    // One narrow exception: the facing head sits near the image center,
    // cross-street heads near the edges (measured: facing cx 0.54, cross
    // phantoms 0.13-0.21). An EDGE red coexisting with a CENTERED green
    // is the cross signal seen at an angle - trusting it once re-latched
    // a hold seconds after a genuine green release and stranded the car
    // an extra light cycle. Only that geometry demotes red, and only when
    // the centered green is not a stray fragment (leaf, reflection, sign
    // sliver at min_area_px) dwarfed by the red it would override.
    const auto off_center = [&](const Blob &b) {
      return std::abs(b.center.x / msg->width - 0.5);
    };
    constexpr double kFacingOffCenter = 0.20;
    const Blob *best = nullptr;
    if (best_red && best_green && off_center(*best_red) > 0.25 &&
        off_center(*best_green) <= kFacingOffCenter &&
        best_green->area >= best_red->area) {
      best = best_green;
      out.state = project_interfaces::msg::TrafficLight::GREEN;
    } else if (best_red &&
               (!best_green || best_green->area < 2 * best_red->area ||
                off_center(*best_green) > kFacingOffCenter)) {
      best = best_red;
      out.state = project_interfaces::msg::TrafficLight::RED;
    } else if (best_green) {
      best = best_green;
      out.state = project_interfaces::msg::TrafficLight::GREEN;
    }
    if (best) {
      out.pixel_count = static_cast<uint32_t>(best->area);
      out.cx = static_cast<float>(best->center.x / msg->width);
      out.cy = static_cast<float>(best->center.y / msg->height);
    }
    pub_->publish(out);
  }

  rclcpp::Publisher<project_interfaces::msg::TrafficLight>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficLightNode>());
  rclcpp::shutdown();
  return 0;
}
