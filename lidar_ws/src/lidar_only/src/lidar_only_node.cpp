#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <custom_msgs/msg/object_info_array.hpp>
#include <custom_msgs/msg/object_info.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <cmath>

using std::placeholders::_1;
using PointT = pcl::PointXYZI;

class LidarObstacleDetector : public rclcpp::Node {
public:
    LidarObstacleDetector() : Node("lidar_obstacle_detector") {
        // Declare hyperparameters
        min_distance_ = this->declare_parameter("min_distance", 0.5);
        max_distance_ = this->declare_parameter("max_distance", 30.0);
        ground_z_threshold_ = this->declare_parameter("ground_z_threshold", -1.5);
        cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.5);
        min_cluster_size_ = this->declare_parameter("min_cluster_size", 5);
        max_cluster_size_ = this->declare_parameter("max_cluster_size", 5000);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", rclcpp::SensorDataQoS(),
            std::bind(&LidarObstacleDetector::pointcloud_callback, this, _1));

        pub_ = this->create_publisher<custom_msgs::msg::ObjectInfoArray>(
            "/lidar_only_object_info", 10);

        RCLCPP_INFO(this->get_logger(), "LidarObstacleDetector initialized.");
    }

private:
    double min_distance_, max_distance_;
    double ground_z_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<custom_msgs::msg::ObjectInfoArray>::SharedPtr pub_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);

        // 지면 제거 및 거리 필터링
        pcl::PointCloud<PointT>::Ptr valid(new pcl::PointCloud<PointT>());
        for (const auto& pt : filtered->points) {
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (pt.z > ground_z_threshold_ && dist >= min_distance_ && dist <= max_distance_) {
                valid->points.push_back(pt);
            }
        }

        if (valid->empty()) return;

        // 클러스터링
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        tree->setInputCloud(valid);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(valid);
        ec.extract(cluster_indices);

        custom_msgs::msg::ObjectInfoArray info_array;
        info_array.header = msg->header;

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            for (int idx : indices.indices) {
                cluster->points.push_back(valid->points[idx]);
            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            float x = centroid[0];
            float y = centroid[1];
            float z = centroid[2];
            float distance = std::sqrt(x * x + y * y + z * z);

            custom_msgs::msg::ObjectInfo info;
            info.x = x;
            info.y = y;
            info.distance = distance;
            info_array.object_infos.push_back(info);
        }

        pub_->publish(info_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}

