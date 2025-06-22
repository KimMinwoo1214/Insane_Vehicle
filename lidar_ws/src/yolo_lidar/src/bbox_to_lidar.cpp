// lidar_yolo_fusion_optimized.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_msgs/msg/bounding_box2d_array.hpp>  // YOLO bbox 메시지
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>

#include <limits>
#include <vector>

class LidarYoloFusionNode : public rclcpp::Node {
public:
  LidarYoloFusionNode() : Node("lidar_yolo_fusion_node") {
    yolo_sub_ = this->create_subscription<custom_msgs::msg::BoundingBox2DArray>(
      "/yolo_bboxes", 10, std::bind(&LidarYoloFusionNode::yoloCallback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", rclcpp::SensorDataQoS(), std::bind(&LidarYoloFusionNode::lidarCallback, this, std::placeholders::_1));

    fused_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/yolo_lidar_fused", 10);

    // 카메라 내부행렬 (예시값, 실제값으로 교체)
    K_ << 600, 0, 320,
          0, 600, 240,
          0, 0, 1;

    // Extrinsics (Lidar to Camera) 초기값
    R_ = Eigen::Matrix3f::Identity();
    T_ = Eigen::Vector3f::Zero();
  }

private:
  rclcpp::Subscription<custom_msgs::msg::BoundingBox2DArray>::SharedPtr yolo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr fused_pub_;

  custom_msgs::msg::BoundingBox2DArray last_bboxes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  std::vector<Eigen::Vector3f> cluster_centers_lidar_; // 라이다 좌표계 클러스터 중심점

  Eigen::Matrix3f K_;
  Eigen::Matrix3f R_;
  Eigen::Vector3f T_;

  void yoloCallback(const custom_msgs::msg::BoundingBox2DArray::SharedPtr msg) {
    last_bboxes_ = *msg;
    tryFuse();
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *latest_cloud_);
    cluster_centers_lidar_.clear();

    if (latest_cloud_->empty()) return;

    // 1. 라이다 좌표계에서 클러스터링 수행
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(latest_cloud_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.3);   // 클러스터 간 거리 임계값 (조정 가능)
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(500);
    ec.setSearchMethod(tree);
    ec.setInputCloud(latest_cloud_);
    ec.extract(cluster_indices);

    // 각 클러스터 중심점 계산 (라이다 좌표계)
    for (const auto& indices : cluster_indices) {
      float x_sum = 0, y_sum = 0, z_sum = 0;
      for (auto idx : indices.indices) {
        const auto& pt = latest_cloud_->points[idx];
        x_sum += pt.x; y_sum += pt.y; z_sum += pt.z;
      }
      float n = static_cast<float>(indices.indices.size());
      cluster_centers_lidar_.push_back(Eigen::Vector3f(x_sum / n, y_sum / n, z_sum / n));
    }

    tryFuse();
  }

  void tryFuse() {
    if (last_bboxes_.boxes.empty() || cluster_centers_lidar_.empty()) return;

    // bbox 당 매칭된 클러스터 중심점이 있을 수 있으므로 순회
    for (const auto& box : last_bboxes_.boxes) {
      float u_min = box.x;
      float u_max = box.x + box.w;
      float v_min = box.y;
      float v_max = box.y + box.h;

      float best_dist = std::numeric_limits<float>::max();
      Eigen::Vector3f best_center_lidar;
      bool found = false;

      // 2. 각 클러스터 중심점을 카메라 좌표계로 변환 → 이미지 투영
      for (const auto& center_lidar : cluster_centers_lidar_) {
        Eigen::Vector3f pt_cam = R_ * center_lidar + T_;

        if (pt_cam.z() <= 0.1f) continue;

        Eigen::Vector3f img_pt = K_ * pt_cam;
        float u = img_pt.x() / img_pt.z();
        float v = img_pt.y() / img_pt.z();

        // bbox 내부에 중심점이 있는지 확인
        if (u >= u_min && u <= u_max && v >= v_min && v <= v_max) {
          float dist = pt_cam.norm();
          if (dist < best_dist) {
            best_dist = dist;
            best_center_lidar = center_lidar;
            found = true;
          }
        }
      }

      if (found) {
        geometry_msgs::msg::Point p;
        p.x = best_center_lidar.x();
        p.y = best_center_lidar.y();
        p.z = best_center_lidar.z();
        fused_pub_->publish(p);
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarYoloFusionNode>());
  rclcpp::shutdown();
  return 0;
}
