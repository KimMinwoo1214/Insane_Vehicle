#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>    // for std::sort, std::clamp
#include <utility>      // for std::pair
#include <cfloat>       // for FLT_MAX

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

// Utility to initialize center markers
#include "cone_detection_lidar/marker.hpp"

using namespace std::chrono_literals;

// 2D point struct for clustering
struct Point2D {
  double x, y;
  bool core;
  int cluster_id;
  Point2D(double _x=0, double _y=0)
    : x(_x), y(_y), core(false), cluster_id(-1) {}
};
static double point_distance(const Point2D &a, const Point2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
static void find_neighbors(std::vector<Point2D> &pts, double eps) {
  for (auto &p : pts) {
    int cnt = 0;
    for (auto &q : pts) if (point_distance(p, q) <= eps) ++cnt;
    p.core = (cnt >= 2);
  }
}
static int find_clusters(std::vector<Point2D> &pts, double eps) {
  int cid = 0;
  const int MAX_ITER = 10;
  for (int iter = 0; iter < MAX_ITER; ++iter) {
    for (auto &p : pts) if (p.core && p.cluster_id < 0) { p.cluster_id = cid; break; }
    for (auto &p : pts) if (p.cluster_id == cid) {
      for (auto &q : pts) if (q.core && q.cluster_id < 0 && point_distance(p, q) <= eps)
        q.cluster_id = cid;
    }
    ++cid;
  }
  for (auto &p : pts) if (!p.core) {
    for (auto &q : pts) if (q.core && q.cluster_id >= 0 && point_distance(p, q) <= eps) {
      p.cluster_id = q.cluster_id;
      break;
    }
  }
  return cid;
}

class DetectionWithSideAndCenter : public rclcpp::Node {
public:
  DetectionWithSideAndCenter()
    : Node("detection_simple")
  {
    // --- parameters ---
    declare_parameter<std::string>("cloud_in_topic", "/points");
    declare_parameter<double>("eps", 0.5);
    declare_parameter<int>("cluster_points_min", 2);
    declare_parameter<int>("cluster_points_max", 50);
    declare_parameter<float>("minX", -80.0f);
    declare_parameter<float>("maxX", 80.0f);
    declare_parameter<float>("minY", -40.0f);
    declare_parameter<float>("maxY", 40.0f);
    declare_parameter<float>("minZ", -2.0f);
    declare_parameter<float>("maxZ", -0.15f);
    declare_parameter<int>("path_samples", 50);

    get_parameter("cloud_in_topic", cloud_in_topic_);
    get_parameter("eps", eps_);
    get_parameter("cluster_points_min", cp_min_);
    get_parameter("cluster_points_max", cp_max_);
    get_parameter("minX", minX_);
    get_parameter("maxX", maxX_);
    get_parameter("minY", minY_);
    get_parameter("maxY", maxY_);
    get_parameter("minZ", minZ_);
    get_parameter("maxZ", maxZ_);
    get_parameter("path_samples", samples_);

    pub_lidar_ = create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10);
    pub_cluster_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", 10);
    pub_center_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("center_markers", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("simple_path", 10);
    pub_angle_ = create_publisher<std_msgs::msg::Float64>("steering_angle", 10);

    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_topic_, 10,
      std::bind(&DetectionWithSideAndCenter::lidar_cb, this, std::placeholders::_1)
    );
  }

private:
  // Parameters
  std::string cloud_in_topic_;
  double eps_;
  int cp_min_, cp_max_;
  float minX_, maxX_, minY_, maxY_, minZ_, maxZ_;
  int samples_;

  // Publishers & subscription
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_center_markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_angle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

  void lidar_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // 1) Crop & convert
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl::CropBox<pcl::PointCloud<pcl::PointXYZI>::PointType> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(minX_, minY_, minZ_, 1.0f));
    crop.setMax(Eigen::Vector4f(maxX_, maxY_, maxZ_, 1.0f));
    auto filt = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    crop.filter(*filt);

    // Publish filtered cloud
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*filt, out);
    out.header = msg->header;
    pub_lidar_->publish(out);

    // 2) DBSCAN clustering
    std::vector<Point2D> pts;
    pts.reserve(filt->size());
    for (auto &p : filt->points) pts.emplace_back(p.x, p.y);
    find_neighbors(pts, eps_);
    int ncl = find_clusters(pts, eps_);

    // 3) Compute cluster centers & markers
    std::vector<std::pair<double, double>> centers;
    visualization_msgs::msg::MarkerArray cm;
    std::vector<double> sx(ncl, 0), sy(ncl, 0);
    std::vector<int> cnt(ncl, 0);
    for (size_t i = 0; i < pts.size(); ++i) {
      int cid = pts[i].cluster_id;
      if (cid < 0) continue;
      sx[cid] += pts[i].x;
      sy[cid] += pts[i].y;
      cnt[cid]++;
    }
    for (int cid = 0; cid < ncl; ++cid) {
      if (cnt[cid] < cp_min_ || cnt[cid] > cp_max_) continue;
      double cx = sx[cid] / cnt[cid];
      double cy = sy[cid] / cnt[cid];
      centers.emplace_back(cx, cy);
      visualization_msgs::msg::Marker m;
      init_center_marker(m, cx, cy, cid);
      m.header = msg->header;
      m.color.a = 1.0;
      cm.markers.push_back(m);
    }
    pub_center_markers_->publish(cm);
    if (centers.size() < 2) return;

    // 4) K-NN chaining: find two seeds with smallest x
    int s1 = 0, s2 = 1;
    if (centers[s2].first < centers[s1].first) std::swap(s1, s2);
    for (size_t i = 2; i < centers.size(); ++i) {
      if (centers[i].first < centers[s1].first) {
        s2 = s1;
        s1 = i;
      } else if (centers[i].first < centers[s2].first) {
        s2 = i;
      }
    }
    auto build_chain = [&](int seed)->std::vector<Eigen::Vector2f> {
      std::vector<bool> used(centers.size(), false);
      std::vector<Eigen::Vector2f> chain;
      int cur = seed;
      chain.emplace_back(centers[cur].first, centers[cur].second);
      used[cur] = true;
      while (true) {
        double best_d = DBL_MAX;
        int next = -1;
        for (size_t j = 0; j < centers.size(); ++j) {
          if (used[j]) continue;
          double dx = centers[j].first - centers[cur].first;
          if (dx <= 0) continue;
          double d = std::hypot(dx, centers[j].second - centers[cur].second);
          if (d < best_d) { best_d = d; next = j; }
        }
        if (next < 0) break;
        used[next] = true;
        cur = next;
        chain.emplace_back(centers[cur].first, centers[cur].second);
      }
      return chain;
    };
    auto chain1 = build_chain(s1);
    auto chain2 = build_chain(s2);
    size_t num = std::min(chain1.size(), chain2.size());
    if (num < 2) return;

    // 5) Build path midpoints, prepend origin
    nav_msgs::msg::Path path;
    path.header = msg->header;
    // start at vehicle origin
    geometry_msgs::msg::PoseStamped origin;
    origin.header = path.header;
    origin.pose.position.x = 0.0;
    origin.pose.position.y = 0.0;
    origin.pose.orientation.w = 1.0;
    path.poses.push_back(origin);
    // midpoints
    for (size_t i = 0; i < num; ++i) {
      Eigen::Vector2f mid = 0.5f * (chain1[i] + chain2[i]);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = mid.x();
      ps.pose.position.y = mid.y();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);

    // 6) Steering angle based on first two points
    if (path.poses.size() >= 2) {
      auto &p0 = path.poses[0].pose.position;
      auto &p1p = path.poses[1].pose.position;
      double ang = std::atan2(p1p.y - p0.y, p1p.x - p0.x);
      double deg = ang * 180.0 / M_PI;
      double st = std::clamp(90.0 - deg, 67.5, 112.5);
      std_msgs::msg::Float64 a; a.data = st;
      pub_angle_->publish(a);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionWithSideAndCenter>());
  rclcpp::shutdown();
  return 0;
}

