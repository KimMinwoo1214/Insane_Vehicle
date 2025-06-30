#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

class LanePathNode : public rclcpp::Node {
public:
    LanePathNode() : Node("lane_path_node") {
        // 카메라 설정
        int camera_index = 0;
        cap_.open(camera_index, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // 이미지 처리 파라미터
        canny_inf = 50;
        canny_sup = 150;
        hough_threshold = 50;
        slope_threshold = 0.3;

        // 퍼블리셔 설정 (차선 중심 경로)
        lane_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("lane_path", 10);

        // 타이머 설정 (주기적으로 실행)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(15),
            std::bind(&LanePathNode::timer_callback, this));

        cv::namedWindow("Lane Path", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "LanePathNode started.");
    }

    ~LanePathNode() {
        if (cap_.isOpened()) cap_.release();
        cv::destroyAllWindows();
    }

private:
    void timer_callback() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame!");
            return;
        }

        int width = frame.cols;
        int height = frame.rows;

        // ROI 설정 (영상의 하단 절반만 사용)
        cv::Rect roi_rect(0, height / 2, width, height / 2);
        cv::Mat roi_frame = frame(roi_rect);

        // 그레이스케일 변환
        cv::Mat gray;
        cv::cvtColor(roi_frame, gray, cv::COLOR_BGR2GRAY);

        // 가우시안 블러 적용
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        // Canny Edge 검출
        cv::Mat edges;
        cv::Canny(blurred, edges, canny_inf, canny_sup);

        // 허프 변환으로 직선 검출
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, hough_threshold, 50, 10);

        // 왼쪽/오른쪽 차선 분리
        auto line_pair = separateLine(lines, slope_threshold);
        auto left_lines = line_pair.first;
        auto right_lines = line_pair.second;

        // 차선 중앙 계산
        auto left_avg = weighted_average_line(left_lines);
        auto right_avg = weighted_average_line(right_lines);

        // 로컬 경로를 구성할 여러 개의 점 생성
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map"; // 차선 경로를 퍼블리시할 좌표계

        for (int i = 0; i < 10; i++) { // 10개의 차선 중심 점 생성
            int y = height - i * 30; // y 좌표를 일정 간격으로 배치 (30픽셀 간격)

            int lane_center_x = width / 2; // 기본값은 이미지 중앙
            if (!left_lines.empty() && !right_lines.empty()) {
                lane_center_x = (compute_x(left_avg, y) + compute_x(right_avg, y)) / 2;
            } else if (!left_lines.empty()) {
                lane_center_x = compute_x(left_avg, y) + width / 2;
            } else if (!right_lines.empty()) {
                lane_center_x = compute_x(right_avg, y) - width / 2;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = static_cast<double>(lane_center_x);
            pose.pose.position.y = static_cast<double>(y);
            pose.pose.position.z = 0.0;
            path_msg.poses.push_back(pose);
        }

        // ROS2 메시지 퍼블리시
        lane_path_pub_->publish(path_msg);

        // 시각화
        cv::Mat laneVis = frame.clone();
        for (const auto& pose : path_msg.poses) {
            cv::circle(laneVis, cv::Point(static_cast<int>(pose.pose.position.x), static_cast<int>(pose.pose.position.y)), 5, cv::Scalar(255, 0, 0), -1);
        }
        cv::imshow("Lane Path", laneVis);
        cv::waitKey(1);
    }

    // 차선을 왼쪽과 오른쪽으로 분리하는 함수
    std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> separateLine(const std::vector<cv::Vec4i>& lines, double slope_threshold) {
        std::vector<cv::Vec4i> left_lines, right_lines;
        for (const auto &line : lines) {
            int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            double slope = static_cast<double>(y2 - y1) / (x2 - x1 + 1e-6);
            if (std::abs(slope) < slope_threshold) continue;
            if (slope < 0) left_lines.push_back(line);
            else right_lines.push_back(line);
        }
        return std::make_pair(left_lines, right_lines);
    }

    // 검출된 선들의 평균을 계산하는 함수
    std::pair<double, double> weighted_average_line(const std::vector<cv::Vec4i>& lines_vec) {
        double slope_sum = 0.0, intercept_sum = 0.0, length_sum = 0.0;
        for (const auto &l : lines_vec) {
            double x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];
            double slope = (y2 - y1) / (x2 - x1 + 1e-6);
            double intercept = y1 - slope * x1;
            double length = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            slope_sum += slope * length;
            intercept_sum += intercept * length;
            length_sum += length;
        }
        if (length_sum == 0.0) return std::make_pair(0.0, 0.0);
        return std::make_pair(slope_sum / length_sum, intercept_sum / length_sum);
    }

    int compute_x(std::pair<double, double> line, int y) {
        return static_cast<int>((y - line.second) / (line.first + 1e-6));
    }

    // 멤버 변수
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lane_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    int canny_inf, canny_sup, hough_threshold;
    double slope_threshold;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LanePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
