#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <eigen3/Eigen/Dense>

class GPSKalmanFilter : public rclcpp::Node {
public:
    GPSKalmanFilter() : Node("gps_kalman_filter") {
        // GPS 데이터 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&GPSKalmanFilter::gps_callback, this, std::placeholders::_1));

        // 필터링된 GPS 데이터 퍼블리시
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/filtered_fix", 10);

        // 초기 칼만 필터 상태 설정
        X << 0.0, 0.0;  // 초기 위치 (위도, 경도)
        P << 1, 0, 0, 1;  // 초기 공분산
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

    Eigen::Vector2d X;  // 상태 벡터 (위도, 경도)
    Eigen::Matrix2d P;  // 공분산 행렬
    Eigen::Matrix2d F;  // 상태 전이 행렬
    Eigen::Matrix2d Q;  // 프로세스 노이즈 행렬
    Eigen::Matrix2d H;  // 관측 행렬
    Eigen::Matrix2d R;  // 관측 노이즈 행렬

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            RCLCPP_WARN(this->get_logger(), "No GPS Fix, skipping measurement...");
            return;
        }

        Eigen::Vector2d Z(msg->latitude, msg->longitude);  // 관측값

        // 예측 단계
        X = F * X;  // 상태 예측
        P = F * P * F.transpose() + Q;  // 공분산 예측

        // 측정 업데이트 단계
        Eigen::Vector2d Y = Z - H * X;  // 측정 잔차
        Eigen::Matrix2d S = H * P * H.transpose() + R;  // 측정 불확실성
        Eigen::Matrix2d K = P * H.transpose() * S.inverse();  // 칼만 이득

        X = X + K * Y;  // 상태 업데이트
        P = (Eigen::Matrix2d::Identity() - K * H) * P;  // 공분산 업데이트

        // 필터링된 데이터 퍼블리시
        auto filtered_msg = sensor_msgs::msg::NavSatFix();
        filtered_msg.header = msg->header;
        filtered_msg.latitude = X(0);
        filtered_msg.longitude = X(1);
        filtered_msg.altitude = msg->altitude;  // 고도는 그대로 유지

        publisher_->publish(filtered_msg);

        RCLCPP_INFO(this->get_logger(), "Filtered GPS: lat=%.6f, lon=%.6f", X(0), X(1));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSKalmanFilter>());
    rclcpp::shutdown();
    return 0;
}
