#include "my_slam/my_slam.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace my_slam
{

using namespace std::chrono_literals;

// ===========================================================================
// ctor
// ===========================================================================
MySlamNode::MySlamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("my_slam", options)
{
  // ---------------- parameterek ----------------
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  fallback_sensor_frame_ =
    declare_parameter<std::string>("fallback_sensor_frame", "utlidar_lidar");
  map_save_path_ = declare_parameter<std::string>("map_save_path", "/tmp/my_slam_map.pcd");

  min_range_ = declare_parameter<double>("min_range", 0.4);
  max_range_ = declare_parameter<double>("max_range", 50.0);
  min_z_base_ = declare_parameter<double>("min_z_base", -1.5);
  max_z_base_ = declare_parameter<double>("max_z_base", 3.0);

  icp_leaf_ = declare_parameter<double>("icp_leaf", 0.25);
  map_leaf_ = declare_parameter<double>("map_leaf", 0.10);
  local_map_leaf_ = declare_parameter<double>("local_map_leaf", 0.20);
  global_map_leaf_ = declare_parameter<double>("global_map_leaf", 0.10);

  keyframe_dist_ = declare_parameter<double>("keyframe_dist", 0.5);
  keyframe_angle_ = declare_parameter<double>("keyframe_angle", 0.26);
  local_map_radius_ = declare_parameter<double>("local_map_radius", 40.0);
  local_map_max_keyframes_ = declare_parameter<int>("local_map_max_keyframes", 50);

  max_iterations_ = declare_parameter<int>("max_iterations", 20);
  min_correspondences_ = declare_parameter<int>("min_correspondences", 60);
  max_icp_points_ = declare_parameter<int>("max_icp_points", 15000);
  max_correspondence_dist_ = declare_parameter<double>("max_correspondence_dist", 1.0);
  plane_threshold_ = declare_parameter<double>("plane_threshold", 0.10);
  huber_delta_ = declare_parameter<double>("huber_delta", 0.10);
  max_position_correction_ = declare_parameter<double>("max_position_correction", 1.0);
  max_angle_correction_ = declare_parameter<double>("max_angle_correction", 0.35);

  tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);
  map_publish_period_ = declare_parameter<double>("map_publish_period", 2.0);
  num_threads_ = declare_parameter<int>("num_threads", 4);
  publish_registered_scan_ = declare_parameter<bool>("publish_registered_scan", true);

  local_map_.reset(new CloudT());
  local_kdtree_.reset(new pcl::KdTreeFLANN<PointT>());
  path_.header.frame_id = map_frame_;

  // ---------------- TF ----------------
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock(), 10s);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // ---------------- callback csoportok ----------------
  cb_group_scan_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_slow_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_group_scan_;

  // A Unitree lidar topicjai BEST_EFFORT QoS-szel mennek -> SensorDataQoS kell,
  // kulonben soha nem jon letre a kapcsolat.
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
    std::bind(&MySlamNode::cloudCallback, this, std::placeholders::_1),
    sub_opt);

  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/my_slam/map", rclcpp::QoS(1).transient_local());
  local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/my_slam/local_map", 1);
  scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/my_slam/scan_registered", 2);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/my_slam/odometry", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/my_slam/path", 1);

  save_srv_ = create_service<std_srvs::srv::Trigger>(
    "/my_slam/save_map",
    std::bind(&MySlamNode::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_slow_);

  map_timer_ = create_wall_timer(
    std::chrono::duration<double>(map_publish_period_),
    std::bind(&MySlamNode::publishGlobalMapTimer, this),
    cb_group_slow_);

  RCLCPP_INFO(
    get_logger(),
    "my_slam elindult | felho: %s | frame-ek: %s -> %s -> %s | IMU: NEM hasznalt",
    cloud_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
}

// ===========================================================================
// segedfuggvenyek
// ===========================================================================
Eigen::Matrix3d MySlamNode::expSO3(const Eigen::Vector3d & w)
{
  const double theta = w.norm();
  if (theta < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

CloudT::Ptr MySlamNode::voxelDownsample(const CloudT::Ptr & in, double leaf)
{
  CloudT::Ptr out(new CloudT());
  if (in->empty() || leaf <= 0.0) {
    *out = *in;
    return out;
  }
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(
    static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  vg.filter(*out);
  return out;
}

bool MySlamNode::lookupTf(
  const std::string & target, const std::string & source,
  const rclcpp::Time & stamp, Eigen::Isometry3d & out)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      target, source, stamp, tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException & e1) {
    // Fallback: legfrissebb elerheto transzform. Identity-re SOHA nem esunk vissza,
    // mert az torz felhot / szetugralo pozt okoz.
    try {
      tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
    } catch (const tf2::TransformException & e2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF hiba %s <- %s : %s", target.c_str(), source.c_str(), e2.what());
      return false;
    }
  }
  out = tf2::transformToEigen(tf);
  return true;
}

// ===========================================================================
// felho elokeszites: sensor frame -> base_link frame, szures, ritkitas
// ===========================================================================
void MySlamNode::preprocess(
  const CloudT::Ptr & in_sensor,
  const Eigen::Isometry3d & T_base_sensor,
  CloudT::Ptr & out_icp,
  CloudT::Ptr & out_map)
{
  CloudT::Ptr filtered(new CloudT());
  filtered->reserve(in_sensor->size());

  const double min_r2 = min_range_ * min_range_;
  const double max_r2 = max_range_ * max_range_;
  const Eigen::Matrix3d R = T_base_sensor.linear();
  const Eigen::Vector3d t = T_base_sensor.translation();

  for (const auto & p : in_sensor->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double r2 =
      static_cast<double>(p.x) * p.x +
      static_cast<double>(p.y) * p.y +
      static_cast<double>(p.z) * p.z;
    if (r2 < min_r2 || r2 > max_r2) {
      continue;
    }
    const Eigen::Vector3d pb =
      R * Eigen::Vector3d(p.x, p.y, p.z) + t;
    if (pb.z() < min_z_base_ || pb.z() > max_z_base_) {
      continue;
    }
    PointT q;
    q.x = static_cast<float>(pb.x());
    q.y = static_cast<float>(pb.y());
    q.z = static_cast<float>(pb.z());
    q.intensity = p.intensity;
    filtered->push_back(q);
  }
  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = true;

  out_icp = voxelDownsample(filtered, icp_leaf_);
  out_map = voxelDownsample(filtered, map_leaf_);
}

// ===========================================================================
// scan-to-map point-to-plane ICP (Gauss-Newton, SE(3))
//   T = T_map_base, forras pontok base_link frame-ben
// ===========================================================================
bool MySlamNode::alignToLocalMap(
  const CloudT::Ptr & src_base,
  const Eigen::Isometry3d & init,
  Eigen::Isometry3d & result,
  double & fitness)
{
  fitness = 0.0;
  result = init;

  if (!local_map_valid_ || local_map_->size() < 100 || src_base->empty()) {
    return false;
  }

  // ritkitas ha tul sok pont (valos ideju mukodes miatt)
  std::vector<int> idx;
  const int n_src = static_cast<int>(src_base->size());
  if (n_src > max_icp_points_) {
    const int stride = (n_src + max_icp_points_ - 1) / max_icp_points_;
    idx.reserve(max_icp_points_ + 1);
    for (int i = 0; i < n_src; i += stride) {
      idx.push_back(i);
    }
  } else {
    idx.resize(n_src);
    for (int i = 0; i < n_src; ++i) {
      idx[i] = i;
    }
  }

  const int N = static_cast<int>(idx.size());
  std::vector<Correspondence, Eigen::aligned_allocator<Correspondence>> corrs(N);

  Eigen::Isometry3d T = init;
  const double max_corr2 = max_correspondence_dist_ * max_correspondence_dist_;

  int last_valid = 0;
  double last_err = 0.0;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d tr = T.translation();

#ifdef _OPENMP
    #pragma omp parallel for num_threads(num_threads_) schedule(static)
#endif
    for (int i = 0; i < N; ++i) {
      corrs[i].valid = false;

      const PointT & sp = src_base->points[idx[i]];
      const Eigen::Vector3d pb(sp.x, sp.y, sp.z);
      const Eigen::Vector3d pw = R * pb + tr;

      PointT query;
      query.x = static_cast<float>(pw.x());
      query.y = static_cast<float>(pw.y());
      query.z = static_cast<float>(pw.z());

      std::vector<int> nn_idx(5);
      std::vector<float> nn_d2(5);
      if (local_kdtree_->nearestKSearch(query, 5, nn_idx, nn_d2) < 5) {
        continue;
      }
      if (nn_d2[4] > max_corr2) {
        continue;
      }

      // sikillesztes: A * n0 = -1  ->  n = n0/|n0|, d = 1/|n0|
      Eigen::Matrix<double, 5, 3> A;
      const Eigen::Matrix<double, 5, 1> ones = -Eigen::Matrix<double, 5, 1>::Ones();
      for (int k = 0; k < 5; ++k) {
        const PointT & q = local_map_->points[nn_idx[k]];
        A(k, 0) = q.x;
        A(k, 1) = q.y;
        A(k, 2) = q.z;
      }
      Eigen::Vector3d n0 = A.colPivHouseholderQr().solve(ones);
      const double nn = n0.norm();
      if (!std::isfinite(nn) || nn < 1e-9) {
        continue;
      }
      const double d = 1.0 / nn;
      const Eigen::Vector3d n = n0 / nn;

      // planaritas ellenorzes
      bool planar = true;
      for (int k = 0; k < 5; ++k) {
        const PointT & q = local_map_->points[nn_idx[k]];
        if (std::fabs(n.x() * q.x + n.y() * q.y + n.z() * q.z + d) > plane_threshold_) {
          planar = false;
          break;
        }
      }
      if (!planar) {
        continue;
      }

      const double r = n.dot(pw) + d;
      if (std::fabs(r) > max_correspondence_dist_) {
        continue;
      }

      // Huber sulyozas + tavolsag alapu skalazas
      const double ar = std::fabs(r);
      double w = (ar <= huber_delta_) ? 1.0 : (huber_delta_ / ar);
      const double range_w = 1.0 - 0.9 * ar / std::max(0.1, std::sqrt(pb.norm()));
      if (range_w <= 0.1) {
        continue;
      }
      w *= range_w;

      // Jacobi: J_t = n^T ; J_phi = -(R^T n) x p
      const Eigen::Vector3d m = R.transpose() * n;
      const Eigen::Vector3d Jphi = -m.cross(pb);

      corrs[i].J.head<3>() = n;
      corrs[i].J.tail<3>() = Jphi;
      corrs[i].r = r;
      corrs[i].w = w;
      corrs[i].valid = true;
    }

    // --- normalegyenletek osszegyujtese ---
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    int valid = 0;
    double err = 0.0;

    for (int i = 0; i < N; ++i) {
      if (!corrs[i].valid) {
        continue;
      }
      H.noalias() += corrs[i].w * corrs[i].J * corrs[i].J.transpose();
      b.noalias() -= corrs[i].w * corrs[i].r * corrs[i].J;
      err += corrs[i].w * corrs[i].r * corrs[i].r;
      ++valid;
    }

    last_valid = valid;
    if (valid < min_correspondences_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Keves ICP korrespondencia (%d) - odom prediktciora esunk vissza.", valid);
      return false;
    }
    last_err = err / static_cast<double>(valid);

    // Levenberg-szeru csillapitas a degeneralt (folyoso, sik fal) esetekre
    H.diagonal().array() += 1e-6;

    const Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);
    if (!dx.allFinite()) {
      return false;
    }

    const Eigen::Vector3d dt = dx.head<3>();
    const Eigen::Vector3d dphi = dx.tail<3>();

    T.translation() += dt;
    T.linear() = T.linear() * expSO3(dphi);

    // ortonormalizalas (numerikus drift ellen)
    Eigen::Quaterniond q(T.linear());
    q.normalize();
    T.linear() = q.toRotationMatrix();

    if (dt.norm() < 1e-4 && dphi.norm() < 1e-4) {
      break;
    }
  }

  // --- korrekcio ellenorzese: tul nagy ugrast nem fogadunk el ---
  const Eigen::Isometry3d delta = init.inverse() * T;
  const double dpos = delta.translation().norm();
  const double dang = Eigen::AngleAxisd(delta.linear()).angle();
  if (dpos > max_position_correction_ || std::fabs(dang) > max_angle_correction_) {
    RCLCPP_WARN(
      get_logger(),
      "ICP korrekcio elutasitva (dpos=%.2f m, dang=%.1f deg) - odom prediktciot hasznalunk.",
      dpos, dang * 180.0 / M_PI);
    return false;
  }

  fitness = last_err;
  (void)last_valid;
  result = T;
  return true;
}

// ===========================================================================
// terkep kezeles
// ===========================================================================
void MySlamNode::addKeyFrame(const Eigen::Isometry3d & pose, const CloudT::Ptr & cloud_map)
{
  KeyFrame kf;
  kf.pose = pose;
  kf.cloud = voxelDownsample(cloud_map, local_map_leaf_);
  keyframes_.push_back(kf);

  // memoria vedelem: nagyon hosszu bejarasnal a regi keyframe felhoket eldobjuk
  // (a globalis voxel-terkepben mar bennuk vannak)
  const std::size_t hard_limit =
    static_cast<std::size_t>(std::max(200, local_map_max_keyframes_ * 8));
  while (keyframes_.size() > hard_limit) {
    keyframes_.pop_front();
  }

  last_keyframe_pose_ = pose;
  has_keyframe_ = true;
}

void MySlamNode::rebuildLocalMap(const Eigen::Isometry3d & pose)
{
  CloudT::Ptr acc(new CloudT());
  const Eigen::Vector3d p = pose.translation();
  const double r2 = local_map_radius_ * local_map_radius_;

  int used = 0;
  for (auto it = keyframes_.rbegin(); it != keyframes_.rend(); ++it) {
    if ((it->pose.translation() - p).squaredNorm() > r2) {
      continue;
    }
    *acc += *(it->cloud);
    if (++used >= local_map_max_keyframes_) {
      break;
    }
  }

  local_map_ = voxelDownsample(acc, local_map_leaf_);
  if (local_map_->size() >= 10) {
    local_kdtree_->setInputCloud(local_map_);
    local_map_valid_ = true;
  } else {
    local_map_valid_ = false;
  }
}

void MySlamNode::insertIntoGlobalMap(const CloudT::Ptr & cloud_map)
{
  const double inv = 1.0 / global_map_leaf_;
  for (const auto & p : cloud_map->points) {
    VoxelKey k;
    k.x = static_cast<int32_t>(std::floor(p.x * inv));
    k.y = static_cast<int32_t>(std::floor(p.y * inv));
    k.z = static_cast<int32_t>(std::floor(p.z * inv));
    auto & v = global_map_[k];
    if (v.n < 20u) {   // atlagolas, de korlatozva -> nem "elmosodik" a terkep
      v.sx += p.x;
      v.sy += p.y;
      v.sz += p.z;
      v.si += p.intensity;
      ++v.n;
    }
  }
}

CloudT::Ptr MySlamNode::buildGlobalCloud()
{
  CloudT::Ptr out(new CloudT());
  out->reserve(global_map_.size());
  for (const auto & kv : global_map_) {
    const VoxelData & v = kv.second;
    if (v.n == 0) {
      continue;
    }
    const float inv_n = 1.0f / static_cast<float>(v.n);
    PointT p;
    p.x = v.sx * inv_n;
    p.y = v.sy * inv_n;
    p.z = v.sz * inv_n;
    p.intensity = v.si * inv_n;
    out->push_back(p);
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

// ===========================================================================
// fo callback
// ===========================================================================
void MySlamNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto t_start = std::chrono::steady_clock::now();

  rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = now();
  }

  std::string sensor_frame = msg->header.frame_id;
  if (sensor_frame.empty()) {
    sensor_frame = fallback_sensor_frame_;
  }

  // --- TF-ek (IMU NELKUL: a mozgas-prior kizarolag a go2_base odom-jabol jon) ---
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odom_sensor = Eigen::Isometry3d::Identity();
  if (!lookupTf(odom_frame_, base_frame_, stamp, T_odom_base)) {
    return;
  }
  if (!lookupTf(odom_frame_, sensor_frame, stamp, T_odom_sensor)) {
    return;
  }
  const Eigen::Isometry3d T_base_sensor = T_odom_base.inverse() * T_odom_sensor;

  // --- felho konverzio + elokeszites ---
  CloudT::Ptr raw(new CloudT());
  pcl::fromROSMsg(*msg, *raw);
  if (raw->empty()) {
    return;
  }

  CloudT::Ptr cloud_icp, cloud_map_res;
  preprocess(raw, T_base_sensor, cloud_icp, cloud_map_res);
  if (cloud_icp->size() < 50) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Tul keves pont a szures utan.");
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);

  // --- pozicio prediktcio az odometriabol ---
  const Eigen::Isometry3d T_pred = T_map_odom_ * T_odom_base;
  Eigen::Isometry3d T_opt = T_pred;

  if (!initialized_) {
    T_opt = T_pred;
    initialized_ = true;
  } else {
    double fitness = 0.0;
    if (!alignToLocalMap(cloud_icp, T_pred, T_opt, fitness)) {
      T_opt = T_pred;   // biztonsagos visszaeses: tiszta odometria
    }
  }

  T_map_base_ = T_opt;
  T_map_odom_ = T_map_base_ * T_odom_base.inverse();

  // --- keyframe / terkep frissites ---
  bool need_local_rebuild = false;
  if (!has_keyframe_) {
    need_local_rebuild = true;
  } else {
    const Eigen::Isometry3d d = last_keyframe_pose_.inverse() * T_map_base_;
    const double dpos = d.translation().norm();
    const double dang = std::fabs(Eigen::AngleAxisd(d.linear()).angle());
    if (dpos > keyframe_dist_ || dang > keyframe_angle_) {
      need_local_rebuild = true;
    }
  }

  CloudT::Ptr cloud_in_map(new CloudT());
  pcl::transformPointCloud(*cloud_map_res, *cloud_in_map, T_map_base_.matrix().cast<float>());

  insertIntoGlobalMap(cloud_in_map);

  if (need_local_rebuild) {
    addKeyFrame(T_map_base_, cloud_in_map);
    rebuildLocalMap(T_map_base_);

    sensor_msgs::msg::PointCloud2 lm_msg;
    pcl::toROSMsg(*local_map_, lm_msg);
    lm_msg.header.frame_id = map_frame_;
    lm_msg.header.stamp = stamp;
    local_map_pub_->publish(lm_msg);
  }

  // --- kimenetek ---
  broadcastMapToOdom(stamp);
  publishOdomAndPath(stamp);

  if (publish_registered_scan_ && scan_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 sc;
    pcl::toROSMsg(*cloud_in_map, sc);
    sc.header.frame_id = map_frame_;
    sc.header.stamp = stamp;
    scan_pub_->publish(sc);
  }

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_DEBUG(
    get_logger(), "scan feldolgozva %.1f ms | icp pontok: %zu | terkep voxelek: %zu",
    ms, cloud_icp->size(), global_map_.size());
  if (ms > 100.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Lassu feldolgozas: %.1f ms (novelt icp_leaf / kisebb max_icp_points segit).", ms);
  }
}

// ===========================================================================
// kimenetek
// ===========================================================================
void MySlamNode::broadcastMapToOdom(const rclcpp::Time & stamp)
{
  if (has_last_tf_stamp_ && stamp <= last_tf_stamp_) {
    return;   // nem monoton idobelyeg -> a TF buffer eldobna / warningolna
  }
  last_tf_stamp_ = stamp;
  has_last_tf_stamp_ = true;

  geometry_msgs::msg::TransformStamped t = tf2::eigenToTransform(T_map_odom_);
  t.header.stamp = stamp;
  t.header.frame_id = map_frame_;
  t.child_frame_id = odom_frame_;
  tf_broadcaster_->sendTransform(t);
}

void MySlamNode::publishOdomAndPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;

  const Eigen::Vector3d p = T_map_base_.translation();
  const Eigen::Quaterniond q(T_map_base_.linear());
  odom.pose.pose.position.x = p.x();
  odom.pose.pose.position.y = p.y();
  odom.pose.pose.position.z = p.z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom_pub_->publish(odom);

  geometry_msgs::msg::PoseStamped ps;
  ps.header = odom.header;
  ps.pose = odom.pose.pose;
  path_.header.stamp = stamp;
  path_.header.frame_id = map_frame_;
  if (path_.poses.empty() ||
    (Eigen::Vector3d(
      path_.poses.back().pose.position.x,
      path_.poses.back().pose.position.y,
      path_.poses.back().pose.position.z) - p).norm() > 0.1)
  {
    path_.poses.push_back(ps);
    if (path_.poses.size() > 20000) {
      path_.poses.erase(path_.poses.begin());
    }
    path_pub_->publish(path_);
  }
}

void MySlamNode::publishGlobalMapTimer()
{
  CloudT::Ptr cloud;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (global_map_.empty()) {
      return;
    }
    cloud = buildGlobalCloud();
  }

  if (map_pub_->get_subscription_count() == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = map_frame_;
  msg.header.stamp = now();
  map_pub_->publish(msg);
}

void MySlamNode::handleSaveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  CloudT::Ptr cloud;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cloud = buildGlobalCloud();
  }

  if (cloud->empty()) {
    res->success = false;
    res->message = "A terkep ures.";
    return;
  }

  if (pcl::io::savePCDFileBinary(map_save_path_, *cloud) == 0) {
    res->success = true;
    res->message = "Mentve: " + map_save_path_ + " (" +
      std::to_string(cloud->size()) + " pont)";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } else {
    res->success = false;
    res->message = "Nem sikerult menteni ide: " + map_save_path_;
  }
}

}  // namespace my_slam

// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_slam::MySlamNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}