#include <memory>
#include <string>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class EmergencyDetector : public rclcpp::Node {
public:
  EmergencyDetector()
  : Node("emergency_detector")
  {
    // Parameters
    declare_parameter<std::string>("markers_topic", "/center_markers");
    declare_parameter<std::string>("emergency_topic", "/emergency");
    declare_parameter<double>("mindist", 0.5);

    get_parameter("markers_topic", markers_topic_);
    get_parameter("emergency_topic", emergency_topic_);
    get_parameter("mindist", mindist_);

    sub_markers_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      markers_topic_, 10,
      std::bind(&EmergencyDetector::markers_cb, this, std::placeholders::_1)
    );
    pub_emergency_ = create_publisher<std_msgs::msg::Int32>(emergency_topic_, 10);
  }

private:
  std::string markers_topic_, emergency_topic_;
  double mindist_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_emergency_;

  void markers_cb(const visualization_msgs::msg::MarkerArray::ConstSharedPtr &msg) {
    bool emergency = false;
    for (const auto &marker : msg->markers) {
      double x = marker.pose.position.x;
      double y = marker.pose.position.y;
      double dist = std::hypot(x, y);
      if (dist <= mindist_) {
        emergency = true;
        break;
      }
    }
    std_msgs::msg::Int32 out;
    out.data = emergency ? 1 : 0;
    pub_emergency_->publish(out);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

