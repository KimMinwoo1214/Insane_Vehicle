#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <custom_msgs/msg/bbox_array.hpp>  // 가정한 메시지 타입, 필요시 수정
#include <sstream>
#include <cmath>

class ObstaclePublisher : public rclcpp::Node {
public:
    ObstaclePublisher() : Node("obstacle_publisher") {
        bbox_sub_ = this->create_subscription<custom_msgs::msg::BboxArray>(
            "/bbox", 10, std::bind(&ObstaclePublisher::bboxCallback, this, std::placeholders::_1));

        obstacle_pub_ = this->create_publisher<std_msgs::msg::String>("/object_info", 10);

        // 카메라 내부 파라미터 (fx, fy, cx, cy)
        fx_ = 640.0;
        fy_ = 640.0;
        cx_ = 320.0;
        cy_ = 320.0;

        // 외부 파라미터: 카메라 → LiDAR
        // 카메라 기준: x-오른쪽, y-아래, z-앞
        // LiDAR 기준: x-뒤, y-왼쪽, z-위
        R_ = {
            {-1, 0, 0},
            {0, 0, 1},
            {0, 1, 0}
        };
        T_ = {0.04, 0.0, -0.06};  // 라이다 기준
    }

private:
    void bboxCallback(const custom_msgs::msg::BboxArray::SharedPtr msg) {
        for (const auto &box : msg->boxes) {
            if (box.class_id != 0 && box.class_id != 1) continue;

            // 중심 좌표 계산
            double u = (box.xmin + box.xmax) / 2.0;
            double v = (box.ymin + box.ymax) / 2.0;

            // 깊이 임의 설정 (YOLO는 깊이 알 수 없음 → 후처리 필요 시 Lidar 연동)
            double Z_cam = 5.0;

            // 카메라 기준 3D 좌표 (전방 z)
            double X_cam = (u - cx_) * Z_cam / fx_;
            double Y_cam = (v - cy_) * Z_cam / fy_;
            double cam_point[3] = {X_cam, Y_cam, Z_cam};

            // 회전 및 변환 → LiDAR 기준 (base_link와 동일)
            double lidar_point[3];
            for (int i = 0; i < 3; ++i) {
                lidar_point[i] = R_[i][0] * cam_point[0] + R_[i][1] * cam_point[1] + R_[i][2] * cam_point[2] + T_[i];
            }

            // lattice용 문자열 포맷으로 퍼블리시
            std::ostringstream oss;
            oss << "obstacle," << lidar_point[0] << "," << lidar_point[1] << "," << lidar_point[2];

            std_msgs::msg::String msg_out;
            msg_out.data = oss.str();
            obstacle_pub_->publish(msg_out);
        }
    }

    rclcpp::Subscription<custom_msgs::msg::BboxArray>::SharedPtr bbox_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_pub_;

    double fx_, fy_, cx_, cy_;
    std::vector<std::vector<double>> R_;
    std::vector<double> T_;
};
