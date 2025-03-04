#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <queue>
#include <vector>
#include <cmath>

// 장애물 회피 및 경로 탐색을 위한 노드 클래스 정의
struct Node {
    double x, y;  // 위치
    double g_cost; // 시작 지점부터 현재까지의 비용
    double h_cost; // 목표 지점까지의 추정 비용 (휴리스틱)
    double f_cost() const { return g_cost + h_cost; }  // f = g + h
    Node* parent; // 부모 노드

    bool operator>(const Node& other) const { return f_cost() > other.f_cost(); } // 우선순위 큐에 필요
};

// LatticePlanner 클래스
class LatticePlanner {
public:
    LatticePlanner(ros::NodeHandle& nh) {
        path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1);
        laser_sub_ = nh.subscribe("/scan", 1, &LatticePlanner::laserCallback, this);
        ros::spin();
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int i = 0; i < msg->ranges.size(); ++i) {
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * cos(i * msg->angle_increment);
            point.y = msg->ranges[i] * sin(i * msg->angle_increment);
            cloud->points.push_back(point);
        }

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-10.0, 10.0);
        pass.filter(*cloud);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(-10.0, 10.0);
        pass.filter(*cloud);

        planPath(cloud);
    }

    // Simple lattice planner to generate a path
    void planPath(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "base_link";

        // Define a simple start and goal pose
        geometry_msgs::PoseStamped start_pose, goal_pose;
        start_pose.pose.position.x = 0.0;
        start_pose.pose.position.y = 0.0;
        start_pose.pose.position.z = 0.0;
        start_pose.pose.orientation.w = 1.0;

        goal_pose.pose.position.x = 5.0;  // Target x position
        goal_pose.pose.position.y = 5.0;  // Target y position
        goal_pose.pose.position.z = 0.0;
        // 여기 다음 goal을 설정하도록 수정하기

        // A* 알고리즘을 통해 경로 생성
        std::vector<geometry_msgs::PoseStamped> feasible_path = generateFeasiblePath(start_pose, goal_pose, cloud);
        for (const auto& pose : feasible_path) {
            path.poses.push_back(pose);
        }

        path_pub_.publish(path);
    }

    // A* 경로 생성 함수
    std::vector<geometry_msgs::PoseStamped> generateFeasiblePath(
        const geometry_msgs::PoseStamped& start_pose,
        const geometry_msgs::PoseStamped& goal_pose,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

        std::vector<geometry_msgs::PoseStamped> path;

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
        std::vector<std::vector<bool>> closed_set(100, std::vector<bool>(100, false)); // 예시로 100x100 그리드
        Node start_node = {start_pose.pose.position.x, start_pose.pose.position.y, 0.0, calculateHeuristic(start_pose, goal_pose), nullptr};
        open_set.push(start_node);

        while (!open_set.empty()) {
            Node current_node = open_set.top();
            open_set.pop();

            if (isGoalReached(current_node, goal_pose)) {
                Node* node = &current_node;
                while (node != nullptr) {
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = node->x;
                    pose.pose.position.y = node->y;
                    path.push_back(pose);
                    node = node->parent;
                }
                std::reverse(path.begin(), path.end());
                break;
            }

            closed_set[static_cast<int>(current_node.x)][static_cast<int>(current_node.y)] = true;

            // 인접 노드 탐색
            for (const auto& direction : directions) {
                double new_x = current_node.x + direction.first;
                double new_y = current_node.y + direction.second;

                if (isValidMove(new_x, new_y, closed_set, cloud)) {
                    double new_g_cost = current_node.g_cost + 1.0; // 이동 비용 (1단계로 설정)
                    double new_h_cost = calculateHeuristic({new_x, new_y, 0.0, 0.0, nullptr}, goal_pose);
                    Node new_node = {new_x, new_y, new_g_cost, new_h_cost, nullptr};
                    new_node.parent = new Node(current_node);
                    open_set.push(new_node);
                }
            }
        }

        return path;
    }

    bool isGoalReached(const Node& current_node, const geometry_msgs::PoseStamped& goal_pose) {
        return std::abs(current_node.x - goal_pose.pose.position.x) < 0.1 &&
               std::abs(current_node.y - goal_pose.pose.position.y) < 0.1;
    }

    double calculateHeuristic(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::PoseStamped& goal_pose) {
        return std::sqrt(std::pow(current_pose.pose.position.x - goal_pose.pose.position.x, 2) +
                         std::pow(current_pose.pose.position.y - goal_pose.pose.position.y, 2));
    }

    bool isValidMove(double x, double y, const std::vector<std::vector<bool>>& closed_set, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 그리드 내에서 유효한지 체크하고, 장애물이 있는지 확인
        if (closed_set[static_cast<int>(x)][static_cast<int>(y)]) {
            return false;
        }

        for (const auto& point : cloud->points) {
            double distance = std::sqrt(std::pow(x - point.x, 2) + std::pow(y - point.y, 2));
            if (distance < 0.5) {  // 장애물이 0.5m 이내에 있으면 이동 불가
                return false;
            }
        }
        return true;
    }
private:
    ros::Publisher path_pub_;
    ros::Subscriber laser_sub_;
    std::vector<std::pair<double, double>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}}; // 상, 하, 좌, 우
};

int main(int argc, char** argv) {
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planner");
    ros::NodeHandle nh;

    // Instantiate the LatticePlanner class
    LatticePlanner lattice_planner(nh);

    return 0;
}
