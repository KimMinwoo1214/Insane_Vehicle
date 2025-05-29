#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

class PerceptionFusionNode : public rclcpp::Node {
public:
    PerceptionFusionNode() : Node("perception_fusion_node") {
        bbox_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/bbox", 10, std::bind(&PerceptionFusionNode::bboxCallback, this, std::placeholders::_1));
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10, std::bind(&PerceptionFusionNode::lidarCallback, this, std::placeholders::_1));
        obstacle_pub_ = this->create_publisher<std_msgs::msg::String>("/object_info", 10);

        // Calibration: Camera to LiDAR (R, T)
        R_ = {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}};
        T_ = {0.04, 0.0, -0.06};  // lidar 기준
    }

private:
    std::vector<std::pair<float, float>> bboxes_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_;

    void bboxCallback(const std_msgs::msg::String::SharedPtr msg) {
        bboxes_.clear();
        std::stringstream ss(msg->data);
        std::string token;
        std::vector<float> coords;
        while (std::getline(ss, token, ',')) {
            coords.push_back(std::stof(token));
        }
        for (size_t i = 0; i + 1 < coords.size(); i += 2) {
            bboxes_.emplace_back(coords[i], coords[i + 1]);  // (u, v)
        }

        if (latest_lidar_) {
            processFusion();
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_lidar_ = msg;
    }

    void processFusion() {
        std::vector<std::vector<float>> points;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_lidar_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_lidar_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_lidar_, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float lx = *iter_x;
            float ly = *iter_y;
            float lz = *iter_z;

            // Convert to camera coordinate: P_cam = R * P_lidar + T
            float cx = R_[0][0] * lx + R_[0][1] * ly + R_[0][2] * lz + T_[0];
            float cy = R_[1][0] * lx + R_[1][1] * ly + R_[1][2] * lz + T_[1];
            float cz = R_[2][0] * lx + R_[2][1] * ly + R_[2][2] * lz + T_[2];

            // Project to image plane
            float fx = 640.0, fy = 640.0, cx_ = 320.0, cy_ = 320.0;
            int u = static_cast<int>(fx * cx / cz + cx_);
            int v = static_cast<int>(fy * cy / cz + cy_);

            for (const auto &[bbox_u, bbox_v] : bboxes_) {
                if (std::abs(u - bbox_u) < 15 && std::abs(v - bbox_v) < 15) {
                    points.push_back({lx, ly, lz});
                    break;
                }
            }
        }

        if (!points.empty()) {
            float x_sum = 0, y_sum = 0, z_sum = 0;
            for (const auto &pt : points) {
                x_sum += pt[0];
                y_sum += pt[1];
                z_sum += pt[2];
            }
            float x_mean = x_sum / points.size();
            float y_mean = y_sum / points.size();
            float z_mean = z_sum / points.size();

            std::ostringstream oss;
            oss << "obstacle," << x_mean << "," << y_mean << "," << std::sqrt(x_mean * x_mean + y_mean * y_mean);

            std_msgs::msg::String out_msg;
            out_msg.data = oss.str();
            obstacle_pub_->publish(out_msg);
        }
    }

    std::vector<std::vector<double>> R_;
    std::vector<double> T_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bbox_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionFusionNode>());
    rclcpp::shutdown();
    return 0;
}
