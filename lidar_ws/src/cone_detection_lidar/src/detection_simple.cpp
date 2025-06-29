#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>    // for std::sort, std::clamp
#include <cfloat>       // for DBL_MAX

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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

using namespace std::chrono_literals;

struct ClusterInfo {
  double x, y, radius;
};

class DetectionWithSideAndCenter : public rclcpp::Node {
public:
  DetectionWithSideAndCenter()
    : Node("detection_simple_euclid_size")
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
    // Hyperparameters for offset behavior
    declare_parameter<double>("offset_gain", 1.0);      // 1보다 커질수록 큰 장애물에서 멀리 떨어짐. 1보다 작을수록 순수 중앙선에 가깝게 피팅됨
    declare_parameter<double>("offset_limit", 0.5);     // offset gain의 영향력 설정 (클수록 효과가 커짐)
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
    get_parameter("offset_gain", offset_gain_);
    get_parameter("offset_limit", offset_limit_);
    get_parameter("path_samples", samples_);

    pub_lidar_ = create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10);
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
  double offset_gain_, offset_limit_;
  int samples_;

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_center_markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_angle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

  void lidar_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // 1) Crop & convert
    auto cloud_i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *cloud_i);
    pcl::CropBox<pcl::PointCloud<pcl::PointXYZI>::PointType> crop;
    crop.setInputCloud(cloud_i);
    crop.setMin(Eigen::Vector4f(minX_, minY_, minZ_, 1.0f));
    crop.setMax(Eigen::Vector4f(maxX_, maxY_, maxZ_, 1.0f));
    auto filt_i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    crop.filter(*filt_i);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*filt_i, out);
    out.header = msg->header;
    pub_lidar_->publish(out);

    // 2) Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(filt_i);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(eps_);
    ec.setMinClusterSize(cp_min_);
    ec.setMaxClusterSize(cp_max_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filt_i);
    ec.extract(cluster_indices);

    // 3) Compute cluster center & radius
    std::vector<ClusterInfo> clusters;
    visualization_msgs::msg::MarkerArray cm;
    for (size_t cid = 0; cid < cluster_indices.size(); ++cid) {
      const auto &indices = cluster_indices[cid];
      double sx = 0.0, sy = 0.0;
      for (auto idx : indices.indices) {
        const auto &pt = filt_i->points[idx]; sx += pt.x; sy += pt.y;
      }
      double cx = sx / indices.indices.size();
      double cy = sy / indices.indices.size();
      double max_r = 0.0;
      for (auto idx : indices.indices) {
        const auto &pt = filt_i->points[idx];
        double dx = pt.x - cx, dy = pt.y - cy;
        max_r = std::max(max_r, std::hypot(dx, dy));
      }
      clusters.push_back({cx, cy, max_r});

      // visualize center
      visualization_msgs::msg::Marker m;
      m.header = msg->header;
      m.ns = "center";
      m.id = cid;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = cx;
      m.pose.position.y = cy;
      m.pose.position.z = 0.0;
      m.scale.x = 0.2;
      m.scale.y = 0.2;
      m.scale.z = 0.2;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      cm.markers.push_back(m);
    }
    pub_center_markers_->publish(cm);
    if (clusters.size() < 2) return;

    // 4) K-NN chaining to select seeds by smallest x
    int s1 = 0, s2 = 1;
    if (clusters[1].x < clusters[0].x) std::swap(s1, s2);
    for (size_t i = 2; i < clusters.size(); ++i) {
      if (clusters[i].x < clusters[s1].x) { s2 = s1; s1 = i; }
      else if (clusters[i].x < clusters[s2].x) { s2 = i; }
    }
    auto build_chain = [&](int seed){
      std::vector<bool> used(clusters.size(), false);
      std::vector<ClusterInfo> chain;
      int cur = seed; used[cur] = true;
      chain.push_back(clusters[cur]);
      while (true) {
        double best_d = DBL_MAX; int next = -1;
        for (size_t j = 0; j < clusters.size(); ++j) {
          if (used[j]) continue;
          double dx = clusters[j].x - clusters[cur].x;
          if (dx <= 0) continue;
          double d = std::hypot(dx, clusters[j].y - clusters[cur].y);
          if (d < best_d) { best_d = d; next = j; }
        }
        if (next < 0) break;
        used[next] = true; cur = next; chain.push_back(clusters[cur]);
      }
      return chain;
    };
    auto chain1 = build_chain(s1);
    auto chain2 = build_chain(s2);
    size_t num = std::min(chain1.size(), chain2.size());
    if (num < 2) return;

    // 5) Build path midpoints with radius offset and hyperparameters
    nav_msgs::msg::Path path;
    path.header = msg->header;
    geometry_msgs::msg::PoseStamped origin;
    origin.header = path.header;
    origin.pose.position.x = 0.0;
    origin.pose.position.y = 0.0;
    origin.pose.orientation.w = 1.0;
    path.poses.push_back(origin);
    for (size_t i = 0; i < num; ++i) {
      auto &L = chain1[i], &R = chain2[i];
      Eigen::Vector2f pL(L.x, L.y), pR(R.x, R.y);
      Eigen::Vector2f dir = (pR - pL).normalized();
      double rL = L.radius, rR = R.radius;
      // Compute base midpoint
      Eigen::Vector2f base_mid = 0.5f * (pL + pR);
      // Hyperparameterized offset
      double raw_offset = (rL - rR) * 0.5;                          // half difference
      double offset = raw_offset * offset_gain_;                     // apply gain
      offset = std::clamp(offset, -offset_limit_, offset_limit_);    // clamp to limit
      /*
        - offset_gain_ > 1.0 : increases push away from larger cluster
        - offset_gain_ < 1.0 : decreases push, closer to pure midpoint
        - offset_limit_ smaller : restricts maximum shift
        - offset_limit_ larger  : allows larger safe offset
      */
      Eigen::Vector2f mid = base_mid + dir * float(offset);

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
      const auto &p0 = path.poses[0].pose.position;
      const auto &p1 = path.poses[1].pose.position;
      double ang = std::atan2(p1.y - p0.y, p1.x - p0.x);
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

