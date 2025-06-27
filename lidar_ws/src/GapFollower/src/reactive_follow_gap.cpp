#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include "std_msgs/msg/string.hpp"

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_follow_gap") {
        declare_parameter("car.width", 0.7);
        declare_parameter("car.wheelbase", 0.735);
        declare_parameter("car.max_steering_deg", 22.5);
        declare_parameter("speed.maximum", 150);
        declare_parameter("filter.z_min", 0.2);
        declare_parameter("filter.z_max", 1.4);
        declare_parameter("filter.x_min", 1.0);
        declare_parameter("filter.x_max", 20.0);
        declare_parameter("filter.y_limit", 3.0);
        declare_parameter("topics.lidar", "/rslidar_points");
        declare_parameter("topics.pwm_cmd", "/pwm_cmd");

        get_parameter("car.width", car_width_);
        get_parameter("car.wheelbase", wheelbase_);
        get_parameter("car.max_steering_deg", max_steering_deg_);
        get_parameter("speed.maximum", max_speed_);
        get_parameter("filter.z_min", z_min_);
        get_parameter("filter.z_max", z_max_);
        get_parameter("filter.x_min", x_min_);
        get_parameter("filter.x_max", x_max_);
        get_parameter("filter.y_limit", y_limit_);
        get_parameter("topics.lidar", lidar_topic_);
        get_parameter("topics.pwm_cmd", pwm_topic_);

        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, 10, std::bind(&ReactiveFollowGap::pointcloud_callback, this, std::placeholders::_1));

        pwm_pub_ = create_publisher<std_msgs::msg::String>(pwm_topic_, 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pwm_pub_;

    double car_width_;
    double wheelbase_;
    double max_steering_deg_;
    double z_min_, z_max_;
    double x_min_, x_max_;
    double y_limit_;
    int max_speed_;
    std::string lidar_topic_, pwm_topic_;

    std::vector<float> extract2DScan(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        std::vector<float> ranges(1080, 30.0);  // max range = 30m

        for (const auto& pt : cloud.points) {
            if (pt.z < z_min_ || pt.z > z_max_) continue;
            if (pt.x < x_min_ || pt.x > x_max_) continue;
            if (std::abs(pt.y) > y_limit_) continue;

            double range = std::hypot(pt.x, pt.y);
            double angle = std::atan2(pt.y, pt.x);
            int index = static_cast<int>((angle + M_PI) * 1080 / (2 * M_PI));
            if (index >= 0 && index < 1080) {
                ranges[index] = std::min(ranges[index], static_cast<float>(range));
            }
        }

        return ranges;
    }

    std::vector<float> preprocess(const std::vector<float>& ranges) {
        std::vector<float> filtered = ranges;
        float max_range = 20.0;

        for (auto& r : filtered)
            if (r < 0.1 || r > max_range) r = max_range;

        int window = 3;
        std::vector<float> smooth(filtered.size(), max_range);
        for (size_t i = 0; i < filtered.size(); ++i) {
            float sum = 0.0;
            int count = 0;
            for (int j = -window / 2; j <= window / 2; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < static_cast<int>(filtered.size())) {
                    sum += filtered[idx];
                    count++;
                }
            }
            smooth[i] = sum / count;
        }

        return smooth;
    }

    int find_max_gap(const std::vector<float>& ranges) {
        float max_val = *std::max_element(ranges.begin(), ranges.end());
        std::vector<int> max_indices;
        for (size_t i = 0; i < ranges.size(); ++i)
            if (ranges[i] == max_val) max_indices.push_back(i);

        int best_start = max_indices[0];
        int max_len = 1, current_len = 1, current_start = best_start;

        for (size_t i = 1; i < max_indices.size(); ++i) {
            if (max_indices[i] == max_indices[i - 1] + 1) {
                current_len++;
            } else {
                if (current_len > max_len) {
                    max_len = current_len;
                    best_start = current_start;
                }
                current_start = max_indices[i];
                current_len = 1;
            }
        }

        if (current_len > max_len) {
            max_len = current_len;
            best_start = current_start;
        }

        return best_start + max_len / 2;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto ranges = extract2DScan(msg);
        auto filtered = preprocess(ranges);

        int best_index = find_max_gap(filtered);
        double angle = -M_PI + best_index * (2 * M_PI / 1080.0);

        double lookahead = filtered[best_index];
        double x = lookahead * std::cos(angle);
        double y = lookahead * std::sin(angle);
        double la_angle = std::atan2(y, x + wheelbase_);
        double steer_rad = std::atan2(2.0 * wheelbase_ * std::sin(la_angle),
                                      std::sqrt((x + wheelbase_) * (x + wheelbase_) + y * y));

        // 제한 각도 클리핑
        double max_rad = max_steering_deg_ * M_PI / 180.0;
        steer_rad = std::clamp(steer_rad, -max_rad, max_rad);

        int pwm_angle = static_cast<int>(90 + steer_rad * (90.0 / max_rad));
        int speed_pwm = max_speed_;

        std_msgs::msg::String pwm_msg;
        pwm_msg.data = std::to_string(pwm_angle) + "," + std::to_string(speed_pwm);
        pwm_pub_->publish(pwm_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
