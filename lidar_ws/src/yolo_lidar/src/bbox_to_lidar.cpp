#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "custom_msgs/msg/bounding_box2_d_array.hpp"
#include "custom_msgs/msg/bounding_box2_d.hpp"
#include "custom_msgs/msg/object_info_array.hpp"
#include "custom_msgs/msg/object_info.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>

using std::placeholders::_1;
using PointT = pcl::PointXYZ;

class BBoxToLidarNode : public rclcpp::Node
{
public:
    BBoxToLidarNode()
    : Node("bbox_to_lidar_node")
    {
        this->declare_parameter("fx", 600.0);
        this->declare_parameter("fy", 600.0);
        this->declare_parameter("cx", 320.0);
        this->declare_parameter("cy", 240.0);

        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();

        this->declare_parameter("cluster_tolerance", 0.5);
        this->declare_parameter("min_cluster_size", 5);
        this->declare_parameter("max_cluster_size", 2500);

        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

        T_lidar_to_cam_ << 
             0.9047731,  -0.42203779, -0.05718171, -2.72159267,
             -0.16147892, -0.21570338, -0.96301434, -0.12413885,
             0.39409415,  0.8805431,  -0.26331282, -1.0815486,
             0,  0, 0,  1;

        subscription_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10, std::bind(&BBoxToLidarNode::pointCloudCallback, this, _1));
        subscription_bbox_ = this->create_subscription<custom_msgs::msg::BoundingBox2DArray>(
            "/yolo_bounding_boxes", 10, std::bind(&BBoxToLidarNode::bboxCallback, this, _1));

        publisher_info_ = this->create_publisher<custom_msgs::msg::ObjectInfoArray>(
            "/object_info", 10);

        RCLCPP_INFO(this->get_logger(), "BBoxToLidarNode (Object Info Mode) initialized.");
    }

private:
    double fx_, fy_, cx_, cy_;
    double cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;
    Eigen::Matrix4f T_lidar_to_cam_;

    custom_msgs::msg::BoundingBox2DArray::SharedPtr last_bbox_array_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pc_;
    rclcpp::Subscription<custom_msgs::msg::BoundingBox2DArray>::SharedPtr subscription_bbox_;
    rclcpp::Publisher<custom_msgs::msg::ObjectInfoArray>::SharedPtr publisher_info_;

    void bboxCallback(const custom_msgs::msg::BoundingBox2DArray::SharedPtr msg)
    {
        last_bbox_array_ = msg;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!last_bbox_array_) return;

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        tree->setInputCloud(filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered);
        ec.extract(cluster_indices);
        RCLCPP_INFO(this->get_logger(), "Total clusters: %zu", cluster_indices.size());
        
        custom_msgs::msg::ObjectInfoArray info_array;

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            for (int idx : indices.indices)
                cluster->points.push_back(filtered->points[idx]);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            Eigen::Vector4f pt_cam = T_lidar_to_cam_ * centroid;

            if (pt_cam(2) <= 0) continue;

            float u = fx_ * pt_cam(0) / pt_cam(2) + cx_;
            float v = fy_ * pt_cam(1) / pt_cam(2) + cy_;

            for (const auto &bbox : last_bbox_array_->boxes) {
                if (u >= bbox.xmin && u <= bbox.xmax &&
                    v >= bbox.ymin && v <= bbox.ymax)
                {
                    custom_msgs::msg::ObjectInfo info;
                    info.x = (bbox.xmin + bbox.xmax) / 2.0;
                    info.y = (bbox.ymin + bbox.ymax) / 2.0;
                    info.distance = std::sqrt(
                        centroid[0]*centroid[0] +
                        centroid[1]*centroid[1] +
                        centroid[2]*centroid[2]);
                    info_array.object_infos.push_back(info);
                    break;
                }
            }
        }

        publisher_info_->publish(info_array);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBoxToLidarNode>());
    rclcpp::shutdown();
    return 0;
}

