#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/object_info_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <vector>
#include <cmath>
#include <limits>

using std::placeholders::_1;

class LatticePlanner : public rclcpp::Node
{
public:
    LatticePlanner() : Node("lattice_planner")
    {
        sub_ = create_subscription<custom_msgs::msg::ObjectInfoArray>(
            "/lidar_only_object_info", 10, std::bind(&LatticePlanner::callback, this, _1));

        pub_path_ = create_publisher<nav_msgs::msg::Path>("/lidar_only_path", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_markers", 10);
    }

private:
    rclcpp::Subscription<custom_msgs::msg::ObjectInfoArray>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

    struct PathCandidate
    {
        std::vector<geometry_msgs::msg::PoseStamped> points;
        double cost;
    };

    void callback(const custom_msgs::msg::ObjectInfoArray::SharedPtr msg)
    {
        std::vector<PathCandidate> candidates;
        rclcpp::Time now = this->now();

        // 여러 개의 곡선 경로 후보 생성 (좌, 정면, 우)
        for (int i = -2; i <= 2; ++i)
        {
            PathCandidate pc;
            double offset = i * 1.0; // 좌우 offset (meter)

            for (int step = 1; step <= 20; ++step)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.header.stamp = now;

                double t = step / 20.0;
                pose.pose.position.x = t * 8.0;
                pose.pose.position.y = offset * std::pow(t, 2);
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;

                pc.points.push_back(pose);
            }

            pc.cost = compute_cost(pc.points, msg->object_infos);
            candidates.push_back(pc);
        }

        // 가장 안전한 경로 선택
        auto best = std::min_element(candidates.begin(), candidates.end(),
                                     [](const PathCandidate &a, const PathCandidate &b)
                                     { return a.cost < b.cost; });

        // 퍼블리시
        nav_msgs::msg::Path path;
        path.header.frame_id = "base_link";
        path.header.stamp = now;
        path.poses = best->points;
        pub_path_->publish(path);

        // RViz 시각화
        visualization_msgs::msg::MarkerArray markers;
        for (size_t i = 0; i < candidates.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = path.header;
            marker.ns = "lattice";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.05;
            marker.color.a = 1.0;
            marker.color.r = (i == best - candidates.begin()) ? 0.0 : 1.0;
            marker.color.g = (i == best - candidates.begin()) ? 1.0 : 0.0;
            marker.color.b = 0.0;

            for (auto &p : candidates[i].points)
                marker.points.push_back(p.pose.position);

            markers.markers.push_back(marker);
        }
        pub_markers_->publish(markers);
    }

    double compute_cost(const std::vector<geometry_msgs::msg::PoseStamped> &points,
                        const std::vector<custom_msgs::msg::ObjectInfo> &obstacles)
    {
        double cost = 0.0;
        for (const auto &p : points)
        {
            for (const auto &o : obstacles)
            {
                double dx = p.pose.position.x - o.x;
                double dy = p.pose.position.y - o.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < 1.0)
                    cost += 1000.0 / dist;
            }
        }
        return cost;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatticePlanner>());
    rclcpp::shutdown();
    return 0;
}

