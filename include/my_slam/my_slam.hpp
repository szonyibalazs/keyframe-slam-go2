#ifndef MY_SLAM__MY_SLAM_HPP_
#define MY_SLAM__MY_SLAM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace my_slam
{

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

// ---------------------------------------------------------------------------
// Globalis terkep: ritka voxel hash (hatarolatlan terulet, duplikatum-mentes)
// ---------------------------------------------------------------------------
struct VoxelKey
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const VoxelKey & o) const
  {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const
  {
    const std::size_t h1 = static_cast<std::size_t>(k.x) * 73856093u;
    const std::size_t h2 = static_cast<std::size_t>(k.y) * 19349663u;
    const std::size_t h3 = static_cast<std::size_t>(k.z) * 83492791u;
    return h1 ^ h2 ^ h3;
  }
};

struct VoxelData
{
  float sx{0.f};
  float sy{0.f};
  float sz{0.f};
  float si{0.f};
  uint32_t n{0};
};

// ---------------------------------------------------------------------------
// Kulcskep (keyframe): mar map frame-be transzformalt, ritkitott felho
// ---------------------------------------------------------------------------
struct KeyFrame
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  CloudT::Ptr cloud;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ---------------------------------------------------------------------------
// Egy ICP korrespondencia (point-to-plane)
// ---------------------------------------------------------------------------
struct Correspondence
{
  Eigen::Matrix<double, 6, 1> J{Eigen::Matrix<double, 6, 1>::Zero()};
  double r{0.0};
  double w{0.0};
  bool valid{false};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MySlamNode : public rclcpp::Node
{
public:
  explicit MySlamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- fo feldolgozas ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // --- TF ---
  bool lookupTf(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, Eigen::Isometry3d & out);

  // --- felho elokeszites ---
  void preprocess(
    const CloudT::Ptr & in_sensor,
    const Eigen::Isometry3d & T_base_sensor,
    CloudT::Ptr & out_icp,
    CloudT::Ptr & out_map);

  // --- scan-to-map illesztes (point-to-plane Gauss-Newton) ---
  bool alignToLocalMap(
    const CloudT::Ptr & src_base,
    const Eigen::Isometry3d & init,
    Eigen::Isometry3d & result,
    double & fitness);

  // --- terkep kezeles ---
  void addKeyFrame(const Eigen::Isometry3d & pose, const CloudT::Ptr & cloud_map);
  void rebuildLocalMap(const Eigen::Isometry3d & pose);
  void insertIntoGlobalMap(const CloudT::Ptr & cloud_map);
  CloudT::Ptr buildGlobalCloud();

  // --- kimenetek ---
  void publishGlobalMapTimer();
  void broadcastMapToOdom(const rclcpp::Time & stamp);
  void publishOdomAndPath(const rclcpp::Time & stamp);
  void handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // --- segedfuggvenyek ---
  static Eigen::Matrix3d expSO3(const Eigen::Vector3d & w);
  static CloudT::Ptr voxelDownsample(const CloudT::Ptr & in, double leaf);

  // ------------------------------------------------------------------
  // parameterek
  // ------------------------------------------------------------------
  std::string cloud_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string fallback_sensor_frame_;
  std::string map_save_path_;

  double min_range_{0.4};
  double max_range_{50.0};
  double min_z_base_{-1.5};
  double max_z_base_{3.0};

  double icp_leaf_{0.25};
  double map_leaf_{0.10};
  double local_map_leaf_{0.20};
  double global_map_leaf_{0.10};

  double keyframe_dist_{0.5};
  double keyframe_angle_{0.26};
  double local_map_radius_{40.0};
  int local_map_max_keyframes_{50};

  int max_iterations_{20};
  int min_correspondences_{60};
  int max_icp_points_{15000};
  double max_correspondence_dist_{1.0};
  double plane_threshold_{0.10};
  double huber_delta_{0.10};
  double converge_trans_{1e-4};
  double converge_rot_{1e-4};
  double max_position_correction_{1.0};
  double max_angle_correction_{0.35};

  double tf_timeout_{0.05};
  double map_publish_period_{2.0};
  int num_threads_{4};
  bool publish_registered_scan_{true};

  // ------------------------------------------------------------------
  // allapot
  // ------------------------------------------------------------------
  std::mutex mtx_;

  Eigen::Isometry3d T_map_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_map_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_keyframe_pose_{Eigen::Isometry3d::Identity()};
  bool initialized_{false};
  bool has_keyframe_{false};

  std::deque<KeyFrame> keyframes_;
  CloudT::Ptr local_map_;
  pcl::KdTreeFLANN<PointT>::Ptr local_kdtree_;
  bool local_map_valid_{false};

  std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> global_map_;

  nav_msgs::msg::Path path_;
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_tf_stamp_{false};

  // ------------------------------------------------------------------
  // ROS interfesz
  // ------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::CallbackGroup::SharedPtr cb_group_scan_;
  rclcpp::CallbackGroup::SharedPtr cb_group_slow_;
};

}  // namespace my_slam

#endif  // MY_SLAM__MY_SLAM_HPP_