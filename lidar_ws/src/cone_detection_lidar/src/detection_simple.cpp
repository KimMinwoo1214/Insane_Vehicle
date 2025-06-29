#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>  // for numeric_limits

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

// 콘 중심 마커 초기화 유틸  
#include "cone_detection_lidar/marker.hpp"

using namespace std::chrono_literals;

// === DBSCAN 기반 ===
struct Point2D {
  double x, y;
  int neighbor_pts{};
  bool core{};
  int cluster_id{-1};
};

static double point_distance(const Point2D &a, const Point2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

static void find_neighbors(std::vector<Point2D> &pts, double eps) {
  for (auto &p : pts) {
    p.neighbor_pts = 0;
    for (auto &q : pts) {
      if (point_distance(p, q) <= eps) {
        p.neighbor_pts++;
      }
    }
    p.core = (p.neighbor_pts >= 2);
  }
}

static int find_clusters(std::vector<Point2D> &pts, double eps) {
  int cid = 0;
  const int MAX_ITER = 10;
  for (int iter = 0; iter < MAX_ITER; ++iter) {
    for (auto &p : pts) {
      if (p.core && p.cluster_id < 0) {
        p.cluster_id = cid;
        break;
      }
    }
    for (auto &p : pts) {
      if (p.cluster_id == cid) {
        for (auto &q : pts) {
          if (q.core && q.cluster_id < 0 && point_distance(p, q) <= eps) {
            q.cluster_id = cid;
          }
        }
      }
    }
    cid++;
  }
  for (auto &p : pts) {
    if (!p.core) {
      for (auto &q : pts) {
        if (q.core && q.cluster_id >= 0 && point_distance(p, q) <= eps) {
          p.cluster_id = q.cluster_id;
        }
      }
    }
  }
  return cid;
}

class DetectionWithSideAndCenter : public rclcpp::Node {
public:
  DetectionWithSideAndCenter()
  : Node("detection_with_side_and_center")
  {
    // 파라미터 선언
    declare_parameter<std::string>("cloud_in_topic", "/points");
    declare_parameter<double>("eps", 0.5);
    declare_parameter<int>("cluster_points_min", 2);
    declare_parameter<int>("cluster_points_max", 50);
    declare_parameter<float>("minX", -80.0f);
    declare_parameter<float>("maxX",  80.0f);
    declare_parameter<float>("minY", -25.0f);
    declare_parameter<float>("maxY",  25.0f);
    declare_parameter<float>("minZ",  -2.0f);
    declare_parameter<float>("maxZ",  -0.15f);

    // 파라미터 가져오기
    get_parameter("cloud_in_topic", cloud_in_topic_);
    get_parameter("eps", eps_);
    get_parameter("cluster_points_min", cp_min_);
    get_parameter("cluster_points_max", cp_max_);
    get_parameter("minX", minX_); get_parameter("maxX", maxX_);
    get_parameter("minY", minY_); get_parameter("maxY", maxY_);
    get_parameter("minZ", minZ_); get_parameter("maxZ", maxZ_);

    // 퍼블리셔 설정
    pub_lidar_       = create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10);
    pub_marker_      = create_publisher<visualization_msgs::msg::MarkerArray>("clustered_marker", 10);
    pub_simple_path_ = create_publisher<nav_msgs::msg::Path>("simple_path", 10);
    pub_angle_       = create_publisher<std_msgs::msg::Float64>("steering_angle", 10);

    // 구독 설정
    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_topic_, 10,
      std::bind(&DetectionWithSideAndCenter::lidar_cb, this, std::placeholders::_1)
    );
  }

private:
  void lidar_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // PointCloud 변환 및 필터링
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl::CropBox<pcl::PointCloud<pcl::PointXYZI>::PointType> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(minX_, minY_, minZ_, 1.0f));
    crop.setMax(Eigen::Vector4f(maxX_, maxY_, maxZ_, 1.0f));
    auto filt = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    crop.filter(*filt);

    // 2D 포인트로 변환 및 클러스터링
    std::vector<Point2D> pts;
    pts.reserve(filt->size());
    for (auto &p : filt->points) {
      pts.push_back({p.x, p.y});
    }
    find_neighbors(pts, eps_);
    int ncl = find_clusters(pts, eps_);

    // 클러스터 중심 계산
    std::vector<double> sx(ncl), sy(ncl);
    std::vector<int> cnt(ncl);
    for (auto &p : pts) {
      if (p.cluster_id >= 0) {
        sx[p.cluster_id] += p.x;
        sy[p.cluster_id] += p.y;
        cnt[p.cluster_id]++;
      }
    }

    std::vector<std::pair<double,double>> centers;
    visualization_msgs::msg::MarkerArray marr;
    for (int i = 0; i < ncl; ++i) {
      if (cnt[i] >= cp_min_ && cnt[i] <= cp_max_) {
        double cx = sx[i] / cnt[i];
        double cy = sy[i] / cnt[i];
        centers.emplace_back(cx, cy);
        visualization_msgs::msg::Marker m;
        init_center_marker(m, cx, cy, i);
        m.header.frame_id = msg->header.frame_id;
        m.header.stamp = now();
        marr.markers.push_back(m);
      }
    }

    // 퍼블리시: 클러스터링 포인트와 마커
    sensor_msgs::msg::PointCloud2 out_pc;
    pcl::toROSMsg(*filt, out_pc);
    out_pc.header = msg->header;
    pub_lidar_->publish(out_pc);
    pub_marker_->publish(marr);

    // --- 여기부터 수정된 부분: y 양/음 그룹에서 x 최소값으로 선택 ---
    std::pair<double,double> neg_pt, pos_pt;
    bool has_neg = false, has_pos = false;
    double min_x_neg = std::numeric_limits<double>::max();
    double min_x_pos = std::numeric_limits<double>::max();

    for (auto &pt : centers) {
      double x = pt.first;
      double y = pt.second;
      // y < 0 그룹에서 x 최소
      if (y < 0.0 && x < min_x_neg) {
        min_x_neg = x;
        neg_pt     = pt;
        has_neg    = true;
      }
      // y > 0 그룹에서 x 최소
      if (y > 0.0 && x < min_x_pos) {
        min_x_pos = x;
        pos_pt     = pt;
        has_pos    = true;
      }
    }

    if (has_neg && has_pos) {
      double cx2 = 0.5 * (neg_pt.first + pos_pt.first);
      double cy2 = 0.5 * (neg_pt.second + pos_pt.second);

      // 경로(Path) 생성
      nav_msgs::msg::Path simple_path;
      simple_path.header.frame_id = msg->header.frame_id;
      simple_path.header.stamp    = now();

      // 시작 원점
      geometry_msgs::msg::PoseStamped ps0;
      ps0.header = simple_path.header;
      ps0.pose.position.x = 0.0;
      ps0.pose.position.y = 0.0;
      ps0.pose.orientation.w = 1.0;
      simple_path.poses.push_back(ps0);

      // 선택된 중간점
      geometry_msgs::msg::PoseStamped ps1;
      ps1.header = simple_path.header;
      ps1.pose.position.x = cx2;
      ps1.pose.position.y = cy2;
      ps1.pose.orientation.w = 1.0;
      simple_path.poses.push_back(ps1);

      pub_simple_path_->publish(simple_path);

      // 스티어링 앵글 계산
      double angle_rad = std::atan2(cy2, cx2);
      double angle_deg = angle_rad * 180.0 / M_PI;
      double steering = 90.0 - angle_deg;
      steering = std::clamp(steering, 67.5, 112.5);

      std_msgs::msg::Float64 ang;
      ang.data = steering;
      pub_angle_->publish(ang);

    } else {
      RCLCPP_WARN(get_logger(), "유효한 간단 경로를 생성할 수 없습니다.");
    }
  }

  // 퍼블리셔·서브스크립션 멤버
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pub_simple_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_angle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

  // 파라미터
  std::string cloud_in_topic_;
  double eps_;
  int    cp_min_, cp_max_;
  float  minX_, maxX_, minY_, maxY_, minZ_, maxZ_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionWithSideAndCenter>());
  rclcpp::shutdown();
  return 0;
}

