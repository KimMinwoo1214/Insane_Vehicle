//빌드 전 할 일
//CMakeLists.txt에 : 
// add_executable(lattice_planner_node src/lattice_planner_node.cpp)
// ament_target_dependencies(lattice_planner_node rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs)
// install(TARGETS lattice_planner_node DESTINATION lib/${PROJECT_NAME})
//package.xml에 : 
// <depend>rclcpp</depend>
// <depend>nav_msgs</depend>
// <depend>geometry_msgs</depend>
// <depend>std_msgs</depend>
// <depend>visualization_msgs</depend>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>

using std::placeholders::_1;

class LatticePlanner : public rclcpp::Node {
public:
    LatticePlanner() : Node("lattice_planner") {
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10, std::bind(&LatticePlanner::localPathCallback, this, _1));

        obstacle_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/object_info", 10, std::bind(&LatticePlanner::obstacleCallback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&LatticePlanner::planAndPublish, this));
    }

private:
    void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        local_path_ = *msg;
    }

    void obstacleCallback(const std_msgs::msg::String::SharedPtr msg) {
        obstacles_.clear();
        std::stringstream ss(msg->data);
        std::string type;
        std::getline(ss, type, ',');
        if (type == "obstacle") {
            std::string sx, sy;
            std::getline(ss, sx, ',');
            std::getline(ss, sy, ',');
            double raw_x = std::stod(sx);
            double raw_y = std::stod(sy);
            double x = -raw_x;  // 뒤 → 앞
            double y = -raw_y;  // 오른쪽 → 왼쪽
            obstacles_.emplace_back(std::make_pair(x, y));
        }
    }

    void planAndPublish() {
        if (local_path_.poses.size() < 2) return;

        auto goal = local_path_.poses.back().pose.position;
        auto lattice_paths = generateLatticePaths(goal.x, goal.y);
        int best_index = selectSafePath(lattice_paths);

        path_pub_->publish(lattice_paths[best_index]);
        publishObstacleMarkers();
        publishPathMarkers(lattice_paths[best_index]);
    }

    std::vector<nav_msgs::msg::Path> generateLatticePaths(double xf, double yf_center) {
        std::vector<double> offsets = {-0.5, 0.0, 0.5};
        double step = 0.5;
        std::vector<nav_msgs::msg::Path> paths;

        for (double offset : offsets) {
            double yf = yf_center + offset;
            nav_msgs::msg::Path path;
            path.header.frame_id = "base_link";

            for (double x = 0.0; x <= xf; x += step) {
                double y = cubic(x, 0.0, yf, xf);
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            paths.push_back(path);
        }
        return paths;
    }

    double cubic(double x, double y0, double yf, double xf) {
        double a0 = y0;
        double a1 = 0;
        double a2 = 3 * (yf - y0) / (xf * xf);
        double a3 = -2 * (yf - y0) / (xf * xf * xf);
        return a0 + a1 * x + a2 * x * x + a3 * x * x * x;
    }

    int selectSafePath(const std::vector<nav_msgs::msg::Path> &paths) {
        std::vector<int> weights(paths.size(), 0);
        for (size_t i = 0; i < paths.size(); ++i) {
            for (const auto &pose : paths[i].poses) {
                for (const auto &[ox, oy] : obstacles_) {
                    double dx = ox - pose.pose.position.x;
                    double dy = oy - pose.pose.position.y;
                    if (std::hypot(dx, dy) < 1.0) {
                        weights[i] += 100;
                    }
                }
            }
        }
        int best = 1;  // 기본값 : 직진
        for (size_t i = 0; i < weights.size(); ++i) {
            if (weights[i] < weights[best]) best = i;
        }
        return best;

    }

    void publishObstacleMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < obstacles_.size(); ++i) {
            const auto &[x, y] = obstacles_[i];
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);
    }

    void publishPathMarkers(const nav_msgs::msg::Path &path) {
        visualization_msgs::msg::MarkerArray marker_array;
        int base_id = 1000;  // 장애물과 구분을 위해 id offset

        for (size_t i = 0; i < path.poses.size(); ++i) {
            const auto &pose = path.poses[i];
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = now();
            marker.id = base_id + i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose.pose;
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color.g = 1.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    // 멤버 변수
    nav_msgs::msg::Path local_path_;
    std::vector<std::pair<double, double>> obstacles_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatticePlanner>());
    rclcpp::shutdown();
    return 0;
}
