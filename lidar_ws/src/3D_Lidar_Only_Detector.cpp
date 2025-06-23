#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>

class ConeDetector : public rclcpp::Node {
public:
  ConeDetector() : Node("cone_detector") {
    declare_parameter("z_min", 0.05);
    declare_parameter("z_max", 1.5);
    declare_parameter("cluster_tolerance", 0.3);
    declare_parameter("min_cluster_size", 10);
    declare_parameter("max_cluster_size", 200);
    declare_parameter("distance_min", 0.5);
    declare_parameter("distance_max", 30.0);

    get_parameter("z_min", z_min_);
    get_parameter("z_max", z_max_);
    get_parameter("cluster_tolerance", cluster_tolerance_);
    get_parameter("min_cluster_size", min_cluster_size_);
    get_parameter("max_cluster_size", max_cluster_size_);
    get_parameter("distance_min", distance_min_);
    get_parameter("distance_max", distance_max_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", rclcpp::SensorDataQoS(),
      std::bind(&ConeDetector::pointcloud_callback, this, std::placeholders::_1));

    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_cones", 10);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    geometry_msgs::msg::PoseArray cones;
    cones.header = msg->header;

    for (const auto& indices : cluster_indices) {
      float x = 0, y = 0, z = 0;
      for (int idx : indices.indices) {
        x += cloud->points[idx].x;
        y += cloud->points[idx].y;
        z += cloud->points[idx].z;
      }
      x /= indices.indices.size();
      y /= indices.indices.size();
      z /= indices.indices.size();

      float distance = std::sqrt(x*x + y*y + z*z);
      if (distance < distance_min_ || distance > distance_max_) continue;

      geometry_msgs::msg::Pose p;
      p.position.x = x;
      p.position.y = y;
      p.position.z = z;
      cones.poses.push_back(p);
    }

    pub_->publish(cones);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;

  double z_min_, z_max_, cluster_tolerance_;
  int min_cluster_size_, max_cluster_size_;
  double distance_min_, distance_max_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetector>());
  rclcpp::shutdown();
  return 0;
}

