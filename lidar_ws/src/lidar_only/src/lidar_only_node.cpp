#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "custom_msgs/msg/object_info_array.hpp"
#include "custom_msgs/msg/object_info.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using std::placeholders::_1;
using PointT = pcl::PointXYZI;

class LidarOnlyNode : public rclcpp::Node
{
public:
    LidarOnlyNode() : Node("lidar_only_node")
    {
        // Declare parameters with units
        declare_parameter("cluster_tolerance", 0.1);   // meter
        declare_parameter("min_cluster_size", 15);      // point count
        declare_parameter("max_cluster_size", 250);   // point count
        declare_parameter("min_z", -0.4);              // meter
        declare_parameter("max_z", 0.3);               // meter
        declare_parameter("min_range", 0.5);           // meter
        declare_parameter("max_range", 10.0);          // meter

        // Get parameters
        get_parameter("cluster_tolerance", cluster_tolerance_);
        get_parameter("min_cluster_size", min_cluster_size_);
        get_parameter("max_cluster_size", max_cluster_size_);
        get_parameter("min_z", min_z_);
        get_parameter("max_z", max_z_);
        get_parameter("min_range", min_range_);
        get_parameter("max_range", max_range_);

        // Sub & Pub
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10, std::bind(&LidarOnlyNode::pointcloud_callback, this, _1));

        pub_info_ = create_publisher<custom_msgs::msg::ObjectInfoArray>(
            "/lidar_only_object_info", 10);

        pub_clustered_pc_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/clustered_points", 10);

        RCLCPP_INFO(this->get_logger(), "LidarOnlyNode initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<custom_msgs::msg::ObjectInfoArray>::SharedPtr pub_info_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered_pc_;

    // Parameters
    double cluster_tolerance_;  // meter
    int min_cluster_size_;      // point count
    int max_cluster_size_;      // point count
    double min_z_, max_z_;      // meter
    double min_range_, max_range_;  // meter

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*msg, *cloud);

        // Remove NaNs
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);

        // Z + range filtering
        pcl::PointCloud<PointT>::Ptr valid(new pcl::PointCloud<PointT>());
        for (const auto &pt : filtered->points)
        {
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (pt.z > min_z_ && pt.z < max_z_ && dist > min_range_ && dist < max_range_)
                valid->points.push_back(pt);
        }

        // Clustering
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

        // Output messages
        custom_msgs::msg::ObjectInfoArray info_array;
        pcl::PointCloud<PointT>::Ptr all_clusters(new pcl::PointCloud<PointT>());

        int cluster_id = 0;
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            Eigen::Vector4f centroid;
            for (int idx : indices.indices)
                cluster->points.push_back(valid->points[idx]);

            pcl::compute3DCentroid(*cluster, centroid);

            custom_msgs::msg::ObjectInfo info;
            info.x = centroid[0];
            info.y = centroid[1];
            info.distance = centroid.head<3>().norm();
            info_array.object_infos.push_back(info);

            // RGB 색상 부여
            uint8_t r = (cluster_id * 53) % 256;
            uint8_t g = (cluster_id * 97) % 256;
            uint8_t b = (cluster_id * 201) % 256;
            uint32_t rgb = (r << 16) | (g << 8) | b;
            float rgb_float = *reinterpret_cast<float *>(&rgb);

            for (auto &pt : cluster->points)
                pt.intensity = rgb_float;

            *all_clusters += *cluster;
            cluster_id++;
        }

        pub_info_->publish(info_array);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*all_clusters, output);
        output.header = msg->header;
        pub_clustered_pc_->publish(output);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOnlyNode>());
    rclcpp::shutdown();
    return 0;
}

