// perception_with_bbox_and_lidar.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
        bbox_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/bbox", 10, std::bind(&PerceptionNode::bboxCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10, std::bind(&PerceptionNode::lidarCallback, this, std::placeholders::_1));

        obstacle_pub_ = this->create_publisher<std_msgs::msg::String>("/object_info", 10);

        // 카메라 내부 파라미터
        fx_ = 640.0;
        fy_ = 640.0;
        cx_ = 320.0;
        cy_ = 320.0;

        // 외부 파라미터 (Camera -> LiDAR 변환)
        R_ = {
            {-1, 0, 0},
            {0, 0, 1},
            {0, 1, 0}
        };
        T_ = {0.04, 0.0, -0.06};
    }

private:
    void bboxCallback(const std_msgs::msg::String::SharedPtr msg) {
        bbox_uv_.clear();
        std::stringstream ss(msg->data);
        std::string token;
        std::vector<float> coords;

        while (std::getline(ss, token, ',')) {
            coords.push_back(std::stof(token));
        }

        if (coords.size() % 2 != 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid bbox center input");
            return;
        }

        for (size_t i = 0; i < coords.size(); i += 2) {
            bbox_uv_.emplace_back(coords[i], coords[i + 1]);
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (bbox_uv_.empty()) return;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (size_t i = 0; i < msg->width; ++i, ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // LiDAR → Camera 좌표계로 변환
            float pt_lidar[3] = {x, y, z};
            float pt_cam[3];
            for (int j = 0; j < 3; ++j) {
                pt_cam[j] = R_[j][0] * pt_lidar[0] + R_[j][1] * pt_lidar[1] + R_[j][2] * pt_lidar[2] + T_[j];
            }

            if (pt_cam[2] <= 0.1) continue;  // 뒤에 있는 점은 무시

            float u = fx_ * pt_cam[0] / pt_cam[2] + cx_;
            float v = fy_ * pt_cam[1] / pt_cam[2] + cy_;

            for (const auto &center : bbox_uv_) {
                float du = u - center.first;
                float dv = v - center.second;
                if (std::sqrt(du * du + dv * dv) < 20.0) {  // 20픽셀 이내
                    // 장애물 퍼블리시
                    std::ostringstream oss;
                    oss << "obstacle," << x << "," << y << "," << std::sqrt(x*x + y*y + z*z);
                    std_msgs::msg::String output;
                    output.data = oss.str();
                    obstacle_pub_->publish(output);
                    break;  // 하나의 bbox에 하나의 점만 매칭
                }
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bbox_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_pub_;

    std::vector<std::pair<float, float>> bbox_uv_;  // 중심점 (u,v)

    double fx_, fy_, cx_, cy_;
    std::vector<std::vector<double>> R_;
    std::vector<double> T_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
