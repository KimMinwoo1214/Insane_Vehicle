#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cfloat>  // DBL_MAX

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int32.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class EmergencyDetector : public rclcpp::Node {
public:
  EmergencyDetector()
  : Node("emergency_detector")
  {
    declare_parameter<std::string>("cloud_topic", "/clustered_points");
    declare_parameter<std::string>("emergency_topic", "/emergency");
    declare_parameter<float>("roi_min_x", 0.3f);
    declare_parameter<float>("roi_max_x",  3.0f);
    declare_parameter<float>("roi_min_y", -0.9f); //실전 상황에서 동적 장애물 등장 방향보고 비대칭적 ROI 적용하기
    declare_parameter<float>("roi_max_y",  0.9f);
    declare_parameter<double>("min_dist", 1.0);

    get_parameter("cloud_topic", cloud_topic_);
    get_parameter("emergency_topic", emergency_topic_);
    get_parameter("roi_min_x", roi_min_x_);
    get_parameter("roi_max_x", roi_max_x_);
    get_parameter("roi_min_y", roi_min_y_);
    get_parameter("roi_max_y", roi_max_y_);
    get_parameter("min_dist", min_dist_);

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 10,
      std::bind(&EmergencyDetector::cloud_cb, this, std::placeholders::_1)
    );
    pub_emergency_ = create_publisher<std_msgs::msg::Int32>(emergency_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "EmergencyDetector node started with clustered_points ROI + min_dist check");
  }

private:
  std::string cloud_topic_, emergency_topic_;
  float roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_;
  double min_dist_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_emergency_;

  void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    double closest = DBL_MAX;

    for (const auto &pt : cloud.points) {
      if (pt.x >= roi_min_x_ && pt.x <= roi_max_x_ &&
          pt.y >= roi_min_y_ && pt.y <= roi_max_y_) {
        double d = std::hypot(pt.x, pt.y);
        if (d < closest) closest = d;
      }
    }

    bool emergency = (closest < min_dist_);

    std_msgs::msg::Int32 out;
    out.data = emergency ? 1 : 0;
    pub_emergency_->publish(out);

    if (closest < DBL_MAX) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10,
        "Closest: %.2f m  -> Emergency: %d", closest, out.data);
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10,
        "Closest: N/A (no ROI points) -> Emergency: %d", out.data);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyDetector>());
  rclcpp::shutdown();
  return 0;
}

