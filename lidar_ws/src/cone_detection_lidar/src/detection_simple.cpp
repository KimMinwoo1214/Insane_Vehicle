#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cfloat>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Eigen for PCA
#include <Eigen/Dense>

// Utility to initialize center markers
#include "cone_detection_lidar/marker.hpp"

using namespace std::chrono_literals;

class DetectionWithSideAndCenter : public rclcpp::Node {
public:
  DetectionWithSideAndCenter()
    : Node("detection_simple")
  {
    declare_parameter<std::string>("cloud_in_topic", "/points");
    declare_parameter<double>("eps", 0.25);
    declare_parameter<float>("voxel_leaf", 0.05f);
    declare_parameter<int>("cluster_points_min", 1);
    declare_parameter<int>("cluster_points_max", 50000);
    declare_parameter<float>("minX", -80.0f);
    declare_parameter<float>("maxX",  80.0f);
    declare_parameter<float>("minY", -100.0f);
    declare_parameter<float>("maxY",  100.0f);
    declare_parameter<float>("minZ", -1.0f);
    declare_parameter<float>("maxZ",  2.0f);
    declare_parameter<double>("gap_threshold", 0.3);

    get_parameter("cloud_in_topic", cloud_in_topic_);
    get_parameter("eps", eps_);
    get_parameter("voxel_leaf", voxel_leaf_);
    get_parameter("cluster_points_min", cp_min_);
    get_parameter("cluster_points_max", cp_max_);
    get_parameter("minX", minX_);
    get_parameter("maxX", maxX_);
    get_parameter("minY", minY_);
    get_parameter("maxY", maxY_);
    get_parameter("minZ", minZ_);
    get_parameter("maxZ", maxZ_);
    get_parameter("gap_threshold", gap_th_);

    pub_cropped_   = create_publisher<sensor_msgs::msg::PointCloud2>("cropped_points",   10);
    pub_clustered_ = create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10);
    pub_centers_   = create_publisher<visualization_msgs::msg::MarkerArray>("center_markers",  10);
    pub_path_      = create_publisher<nav_msgs::msg::Path>("simple_path",       10);
    pub_angle_     = create_publisher<std_msgs::msg::Float32>("cone_steering_angle",   10);

    sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_topic_, 10,
      std::bind(&DetectionWithSideAndCenter::lidar_cb, this, std::placeholders::_1)
    );
  }

private:
  std::string cloud_in_topic_;
  double eps_, gap_th_;
  float voxel_leaf_, minX_, maxX_, minY_, maxY_, minZ_, maxZ_;
  int cp_min_, cp_max_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cropped_, pub_clustered_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_centers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_angle_;

  void lidar_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // 1) Crop box and downsample
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *cloud);
    pcl::CropBox<pcl::PointCloud<pcl::PointXYZI>::PointType> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(minX_,minY_,minZ_,1.0f));
    crop.setMax(Eigen::Vector4f(maxX_,maxY_,maxZ_,1.0f));
    auto roi = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    crop.filter(*roi);

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(roi);
    vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    auto ds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    vg.filter(*ds);

    sensor_msgs::msg::PointCloud2 out1;
    pcl::toROSMsg(*ds, out1);
    out1.header = msg->header;
    pub_cropped_->publish(out1);

    // 2) Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(ds);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(eps_);
    ec.setMinClusterSize(cp_min_);
    ec.setMaxClusterSize(cp_max_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(ds);
    std::vector<pcl::PointIndices> clusters;
    ec.extract(clusters);

    // 3) PCA + gap splitting for center detection
    auto cloud_rgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_rgb->header   = ds->header;
    cloud_rgb->is_dense = ds->is_dense;
    visualization_msgs::msg::MarkerArray markers;
    std::vector<Eigen::Vector2f> centers;

    int cid = 0;
    for (auto &c : clusters) {
      std::vector<Eigen::Vector2f> pts;
      pts.reserve(c.indices.size());
      for (int idx : c.indices) pts.emplace_back(ds->points[idx].x, ds->points[idx].y);

      Eigen::Vector2f mean = Eigen::Vector2f::Zero();
      for (auto &p : pts) mean += p;
      mean /= float(pts.size());
      Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
      for (auto &p : pts) {
        auto d = p - mean;
        cov += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
      Eigen::Vector2f axis = es.eigenvectors().col(1).normalized();

      std::vector<std::pair<float,int>> proj;
      proj.reserve(pts.size());
      for (int i = 0; i < pts.size(); ++i) proj.emplace_back(axis.dot(pts[i] - mean), i);
      std::sort(proj.begin(), proj.end(), [](auto &a, auto &b){ return a.first < b.first; });

      std::vector<std::vector<int>> subs;
      std::vector<int> cur = { proj[0].second };
      for (int i = 0; i + 1 < proj.size(); ++i) {
        if (proj[i+1].first - proj[i].first > gap_th_) {
          subs.push_back(cur);
          cur.clear();
        }
        cur.push_back(proj[i+1].second);
      }
      subs.push_back(cur);

      for (auto &sub : subs) {
        Eigen::Vector2f sc = Eigen::Vector2f::Zero();
        for (int i : sub) sc += pts[i];
        sc /= float(sub.size());
        centers.push_back(sc);

        visualization_msgs::msg::Marker m;
        init_center_marker(m, sc.x(), sc.y(), cid);
        m.header = msg->header; m.color.a = 1.0;
        markers.markers.push_back(m);

        uint8_t r = (cid*37)%255, g=(cid*91)%255, b=(cid*53)%255;
        for (int i : sub) {
          auto &pt = pts[i];
          pcl::PointXYZRGB pr{pt.x(), pt.y(), 0.0f, r, g, b};
          cloud_rgb->points.push_back(pr);
        }
        ++cid;
      }
    }
    pub_centers_->publish(markers);
    sensor_msgs::msg::PointCloud2 out2;
    pcl::toROSMsg(*cloud_rgb, out2);
    out2.header = msg->header;
    pub_clustered_->publish(out2);

    // 4) Path generation
    if (centers.empty()) {
      std_msgs::msg::Float32 fallback;
      fallback.data = 0.0;
      pub_angle_->publish(fallback);
      RCLCPP_WARN(this->get_logger(), "⚠️ No clusters found — fallback steering angle 0° published.");
      return;
    }

    std::vector<Eigen::Vector2f> left, right;
    for (auto &c : centers) {
      if (c.y() >= 0) left.push_back(c);
      else            right.push_back(c);
    }

    nav_msgs::msg::Path path; path.header = msg->header;
    geometry_msgs::msg::PoseStamped ori; ori.header = path.header; ori.pose.orientation.w = 1.0;
    path.poses.push_back(ori);
    Eigen::Vector2f mid;
    if (!left.empty() && !right.empty()) {
      // ✅ 바꾼 부분: 진행 방향(Y축) 기준으로 가장 가까운 거 선택
      Eigen::Vector2f bestL; double minL = DBL_MAX;
      for (auto &c : left) {
        double d = std::abs(c.y());
        if (d < minL) { minL = d; bestL = c; }
      }
      Eigen::Vector2f bestR; double minR = DBL_MAX;
      for (auto &c : right) {
        double d = std::abs(c.y());
        if (d < minR) { minR = d; bestR = c; }
      }
      mid = 0.5f * (bestL + bestR);
    } else {
      auto &side = (!left.empty() ? left : right);
      std::sort(side.begin(), side.end(), [](auto &a, auto &b){
        return std::abs(a.y()) < std::abs(b.y());
      });
      if (side.size() >= 3) mid = 0.5f * (side[0] + side[1]);
      else mid = side[0];
    }
    geometry_msgs::msg::PoseStamped ps; ps.header = path.header;
    ps.pose.position.x = mid.x(); ps.pose.position.y = mid.y(); ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
    pub_path_->publish(path);

    if (path.poses.size() >= 2) {
      auto &p0 = path.poses[0].pose.position;
      auto &p1 = path.poses[1].pose.position;
      double ang = std::atan2(p1.y - p0.y, p1.x - p0.x);
      double deg = ang * 180.0 / M_PI;
      std_msgs::msg::Float32 a; a.data = std::clamp(90.0 - deg, 67.5, 112.5);
      pub_angle_->publish(a);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionWithSideAndCenter>());
  rclcpp::shutdown();
  return 0;
}

