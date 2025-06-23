#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_msgs/msg/bounding_box2_d_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

class LidarYoloFusionNode : public rclcpp::Node {
public:
  LidarYoloFusionNode() : Node("lidar_yolo_fusion_node") {
    using std::placeholders::_1;

    declare_parameter<std::vector<double>>("camera_intrinsics", {703.379, 0, 330.37, 0, 750.72, 226.50, 0, 0, 1});
    declare_parameter<std::vector<double>>("extrinsic_translation", {0.0, 0.0, -0.1});

    auto intrinsics = get_parameter("camera_intrinsics").as_double_array();
    auto translation = get_parameter("extrinsic_translation").as_double_array();

    K_ = Eigen::Matrix3f();
    K_ << intrinsics[0], intrinsics[1], intrinsics[2],
          intrinsics[3], intrinsics[4], intrinsics[5],
          intrinsics[6], intrinsics[7], intrinsics[8];

    R_ = Eigen::Matrix3f::Identity();
    T_ = Eigen::Vector3f(translation[0], translation[1], translation[2]);

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 10, std::bind(&LidarYoloFusionNode::lidarCallback, this, _1));

    bbox_sub_ = create_subscription<custom_msgs::msg::BoundingBox2DArray>(
        "/yolo_bounding_boxes", 10, std::bind(&LidarYoloFusionNode::yoloCallback, this, _1));

    fused_pub_ = create_publisher<geometry_msgs::msg::Point>("/fused_cone_position", 10);
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *latest_cloud_);
    tryFuse();
  }

  void yoloCallback(const custom_msgs::msg::BoundingBox2DArray::SharedPtr msg) {
    latest_boxes_ = *msg;
    tryFuse();
  }

  void tryFuse() {
    if (!latest_cloud_ || latest_boxes_.boxes.empty()) return;

    for (const auto& box : latest_boxes_.boxes) {
      float u_min = box.x;
      float u_max = box.x + box.width;
      float v_min = box.y;
      float v_max = box.y + box.height;

      for (const auto& pt : latest_cloud_->points) {
        Eigen::Vector3f pt_lidar(pt.x, pt.y, pt.z);
        Eigen::Vector3f pt_cam = R_ * pt_lidar + T_;

        if (pt_cam.z() <= 0.1f || !std::isfinite(pt_cam.z())) continue;

        Eigen::Vector3f pt_img = K_ * pt_cam;
        float u = pt_img(0) / pt_img(2);
        float v = pt_img(1) / pt_img(2);

        if (u >= u_min && u <= u_max && v >= v_min && v <= v_max) {
          geometry_msgs::msg::Point p;
          p.x = pt.x;
          p.y = pt.y;
          p.z = pt.z;
          fused_pub_->publish(p);
          break;  // Publish only one point per box
        }
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<custom_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr fused_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
  custom_msgs::msg::BoundingBox2DArray latest_boxes_;

  Eigen::Matrix3f K_;
  Eigen::Matrix3f R_;
  Eigen::Vector3f T_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarYoloFusionNode>());
  rclcpp::shutdown();
  return 0;
}

