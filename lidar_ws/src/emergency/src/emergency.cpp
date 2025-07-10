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
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class EmergencyDetector : public rclcpp::Node {
public:
  EmergencyDetector()
  : Node("emergency_detector")
  {
    // 기본 ROI 파라미터 선언
    declare_parameter<std::string>("cloud_topic", "/clustered_points");
    declare_parameter<std::string>("emergency_topic", "/emergency");
    declare_parameter<float>("roi_min_x", 0.3f);
    declare_parameter<float>("roi_max_x",  1.0f);
    declare_parameter<float>("roi_min_y", -0.3f);
    declare_parameter<float>("roi_max_y",  0.3f);
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

    RCLCPP_INFO(this->get_logger(), "EmergencyDetector node started with optimizations: tunnel override + cropbox ROI");
  }

private:
  std::string cloud_topic_, emergency_topic_;
  float roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_;
  double min_dist_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_emergency_;

  void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // PCL 클라우드 변환
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // 1) 터널 ROI 검사 (수동 루프 + 임계치 도달 시 조기 탈출)
    const size_t TUNNEL_THRESHOLD = 5000;
    size_t tunnel_count = 0;
    for (const auto &pt : cloud->points) {
      if (pt.x >= 0.0f && pt.x <= 1.5f &&
          pt.y >= -1.0f && pt.y <= 1.0f &&
          pt.z >= 0.3f && pt.z <= 1.0f) {
        if (++tunnel_count >= TUNNEL_THRESHOLD) {
          break;  // 임계치 이상이면 루프 즉시 탈출
        }
      }
    }

    std_msgs::msg::Int32 out;
    if (tunnel_count >= TUNNEL_THRESHOLD) {
      // 터널 오버라이드: Emergency OFF
      out.data = 0;
      pub_emergency_->publish(out);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10,
        "Tunnel override active: %zu points (>=5000) -> Emergency force OFF", tunnel_count);
      return;
    }

    // 2) 기본 ROI에 대해 PCL CropBox 필터 적용
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, -FLT_MAX, 1.0f));
    crop.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_,  FLT_MAX, 1.0f));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop.filter(*roi_cloud);

    // 3) ROI 내 최단 거리 계산
    double closest = DBL_MAX;
    for (const auto &pt : roi_cloud->points) {
      double d = std::hypot(pt.x, pt.y);
      if (d < closest) {
        closest = d;
      }
    }

    bool emergency = (closest < min_dist_);
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

