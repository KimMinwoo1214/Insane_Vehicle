#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

class ReactiveFollowGapClustered : public rclcpp::Node {
public:
    ReactiveFollowGapClustered() : Node("reactive_follow_gap_clustered") {
        // === 차량 파라미터 ===
        declare_parameter("car.width", 0.7);
        declare_parameter("car.wheelbase", 0.735);
        declare_parameter("car.max_steering_deg", 22.5);
        declare_parameter("speed.maximum", 300);

        // === 필터링 ===
        declare_parameter("filter.z_min", -0.4);
        declare_parameter("filter.z_max", 0.2);
        declare_parameter("filter.x_min", 1.0);
        declare_parameter("filter.x_max", 10.0);
        declare_parameter("filter.y_limit", 1.5);
        declare_parameter("filter.min_range", 0.3);
        declare_parameter("filter.max_range", 20.0);
        declare_parameter("filter.voxel_leaf_size", 0.1);

        // === Clustering ===
        declare_parameter("cluster.eps", 0.5);
        declare_parameter("cluster.min_points", 3);
        declare_parameter("cluster.max_points", 1000);

        // === Gap ===
        declare_parameter("gap.min_width", 0.8);

        // === Marker ===
        declare_parameter("marker.frame_id", "base_link");
        declare_parameter("marker.scale", 0.2);

        // === Topics ===
        declare_parameter("topics.lidar", "/rslidar_points");
        declare_parameter("topics.pwm_cmd", "/pwm_cmd");
        declare_parameter("topics.marker", "/gap_markers");

        // === 파라미터 로드 ===
        get_parameter("car.width", car_width_);
        get_parameter("car.wheelbase", wheelbase_);
        get_parameter("car.max_steering_deg", max_steering_deg_);
        get_parameter("speed.maximum", max_speed_);

        get_parameter("filter.z_min", z_min_);
        get_parameter("filter.z_max", z_max_);
        get_parameter("filter.x_min", x_min_);
        get_parameter("filter.x_max", x_max_);
        get_parameter("filter.y_limit", y_limit_);
        get_parameter("filter.min_range", min_range_);
        get_parameter("filter.max_range", max_range_);
        get_parameter("filter.voxel_leaf_size", voxel_leaf_size_);

        get_parameter("cluster.eps", cluster_eps_);
        get_parameter("cluster.min_points", cluster_min_points_);
        get_parameter("cluster.max_points", cluster_max_points_);

        get_parameter("gap.min_width", min_gap_width_);

        get_parameter("marker.frame_id", marker_frame_);
        get_parameter("marker.scale", marker_scale_);

        get_parameter("topics.lidar", lidar_topic_);
        get_parameter("topics.pwm_cmd", pwm_topic_);
        get_parameter("topics.marker", marker_topic_);

        // === 구독 & 퍼블리셔 ===
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, 10, std::bind(&ReactiveFollowGapClustered::pointcloud_callback, this, std::placeholders::_1));

        pwm_pub_ = create_publisher<std_msgs::msg::String>(pwm_topic_, 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pwm_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // === 파라미터 ===
    double car_width_, wheelbase_, max_steering_deg_;
    double z_min_, z_max_, x_min_, x_max_, y_limit_;
    double min_range_, max_range_;
    double voxel_leaf_size_;
    double cluster_eps_;
    int cluster_min_points_, cluster_max_points_;
    double min_gap_width_;
    double marker_scale_;
    std::string marker_frame_;
    int max_speed_;
    std::string lidar_topic_, pwm_topic_, marker_topic_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // === PointCloud 변환 ===
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // === ROI 필터링 ===
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : cloud->points) {
            if (pt.z < z_min_ || pt.z > z_max_) continue;
            if (pt.x < x_min_ || pt.x > x_max_) continue;
            if (std::abs(pt.y) > y_limit_) continue;

            double r = std::hypot(pt.x, pt.y);
            if (r < min_range_ || r > max_range_) continue;

            filtered->points.push_back(pt);
        }

        // === VoxelGrid 다운샘플 ===
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(filtered);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        vg.filter(*downsampled);

        // === Euclidean Clustering ===
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(downsampled);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_eps_);
        ec.setMinClusterSize(cluster_min_points_);
        ec.setMaxClusterSize(cluster_max_points_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(downsampled);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No clusters found!");
            return;
        }

        // === Cluster centers ===
        struct ClusterCenter {
            double x, y, angle, range;
        };

        std::vector<ClusterCenter> clusters;
        int id = 0;
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto& indices : cluster_indices) {
            double cx = 0, cy = 0;
            for (auto idx : indices.indices) {
                cx += downsampled->points[idx].x;
                cy += downsampled->points[idx].y;
            }
            cx /= indices.indices.size();
            cy /= indices.indices.size();

            double range = std::hypot(cx, cy);
            double angle = std::atan2(cy, cx);
            clusters.push_back({cx, cy, angle, range});

            // === RViz Marker: cluster center ===
            visualization_msgs::msg::Marker m;
            m.header.frame_id = marker_frame_;
            m.header.stamp = now();
            m.ns = "clusters";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = cx;
            m.pose.position.y = cy;
            m.pose.position.z = 0.0;
            m.scale.x = marker_scale_;
            m.scale.y = marker_scale_;
            m.scale.z = marker_scale_;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 0.8;
            marker_array.markers.push_back(m);
        }

        // === Cluster 각도순 정렬 ===
        std::sort(clusters.begin(), clusters.end(),
                  [](const ClusterCenter& a, const ClusterCenter& b) { return a.angle < b.angle; });

        // === 가장 큰 Gap 찾기 ===
        double max_gap = 0.0;
        double best_angle = 0.0;

        for (size_t i = 0; i < clusters.size(); ++i) {
            size_t j = (i + 1) % clusters.size();
            double da = clusters[j].angle - clusters[i].angle;
            if (da < 0) da += 2 * M_PI;

            double avg_range = (clusters[i].range + clusters[j].range) / 2.0;
            double arc = 2 * std::sin(da / 2.0) * avg_range;

            if (arc > max_gap && arc > min_gap_width_) {
                max_gap = arc;
                best_angle = clusters[i].angle + da / 2.0;
                if (best_angle > M_PI) best_angle -= 2 * M_PI;
            }
        }

        if (max_gap < min_gap_width_) {
            RCLCPP_WARN(this->get_logger(), "No wide enough gap! Going straight.");
            best_angle = 0.0;
        }

        // === RViz Marker: Gap 방향 ===
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = marker_frame_;
        arrow.header.stamp = now();
        arrow.ns = "gap_direction";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point p0, p1;
        p0.x = 0.0;
        p0.y = 0.0;
        p0.z = 0.0;
        p1.x = 5.0 * std::cos(best_angle);
        p1.y = 5.0 * std::sin(best_angle);
        p1.z = 0.0;
        arrow.points.push_back(p0);
        arrow.points.push_back(p1);
        arrow.scale.x = 0.05;  // shaft diameter
        arrow.scale.y = 0.1;   // head diameter
        arrow.scale.z = 0.1;   // head length
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        marker_array.markers.push_back(arrow);

        marker_pub_->publish(marker_array);

        // === Pure Pursuit ===
        double lookahead = 5.0;
        double x = lookahead * std::cos(best_angle);
        double y = lookahead * std::sin(best_angle);
        double la_angle = std::atan2(y, x + wheelbase_);
        double steer_rad = std::atan2(2.0 * wheelbase_ * std::sin(la_angle),
                                      std::sqrt((x + wheelbase_) * (x + wheelbase_) + y * y));

        double max_rad = max_steering_deg_ * M_PI / 180.0;
        steer_rad = std::clamp(steer_rad, -max_rad, max_rad);

        int pwm_angle = static_cast<int>(180 - (90 + steer_rad * (90.0 / max_rad)));
        int speed_pwm = max_speed_;

        std_msgs::msg::String pwm_msg;
        pwm_msg.data = std::to_string(pwm_angle) + "," + std::to_string(speed_pwm);
        pwm_pub_->publish(pwm_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGapClustered>());
    rclcpp::shutdown();
    return 0;
}

