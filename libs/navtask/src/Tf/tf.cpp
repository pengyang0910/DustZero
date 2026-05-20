/*
 * tf.cpp
 * Frame_t / TF_t 实现
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include "navtask/Tf/tf.h"
#include <cmath>

// ─────────────────────────────────────────────────────────
// Frame_t
// ─────────────────────────────────────────────────────────
Frame_t::Frame_t() {
  r = Eigen::Matrix3d::Identity();
  v = Eigen::Vector3d::Zero();
}

Frame_t::Frame_t(const Eigen::Matrix3d& r_, const Eigen::Vector3d& v_)
    : r(r_), v(v_) {
  yaw_ = std::atan2(r(1, 0), r(0, 0));
}

Frame_t::~Frame_t() = default;

void Frame_t::SetRotationFromRPY(double roll_, double pitch_, double yaw_) {
  this->roll_  = roll_;
  this->pitch_ = pitch_;
  this->yaw_   = yaw_;
  r = GetMatrixFromRPY(roll_, pitch_, yaw_);
}

Eigen::Matrix3d Frame_t::GetRotation() const {
  return r;
}

void Frame_t::SetTransVector(double tx_, double ty_, double tz_) {
  v << tx_, ty_, tz_;
}

Eigen::Vector3d Frame_t::GetTransVector() const {
  return v;
}

void Frame_t::SetRTParams(double tx_, double ty_, double tz_,
                          double roll_, double pitch_, double yaw_) {
  SetTransVector(tx_, ty_, tz_);
  SetRotationFromRPY(roll_, pitch_, yaw_);
}

Eigen::Vector3d Frame_t::GetTFVector(const Eigen::Vector3d& v_) const {
  return r * v_ + v;
}

Eigen::Vector3d Frame_t::GetInvTFVector(const Eigen::Vector3d& v_) const {
  return r.transpose() * (v_ - v);
}

void Frame_t::GetTFPose(double x, double y, double angle,
                        double& out_x, double& out_y, double& out_angle) const {
  Eigen::Vector3d p(x, y, 0.0);
  Eigen::Vector3d p_out = GetTFVector(p);
  out_x     = p_out.x();
  out_y     = p_out.y();
  out_angle = angle + yaw_;
}

void Frame_t::GetInvTFPose(double x, double y, double angle,
                           double& out_x, double& out_y, double& out_angle) const {
  Eigen::Vector3d p(x, y, 0.0);
  Eigen::Vector3d p_out = GetInvTFVector(p);
  out_x     = p_out.x();
  out_y     = p_out.y();
  out_angle = angle - yaw_;
}

Eigen::Matrix3d Frame_t::GetMatrixFromRPY(double roll_, double pitch_, double yaw_) {
  Eigen::Matrix3d R;
  double cr = std::cos(roll_), sr = std::sin(roll_);
  double cp = std::cos(pitch_), sp = std::sin(pitch_);
  double cy = std::cos(yaw_), sy = std::sin(yaw_);

  R(0, 0) = cy * cp;
  R(0, 1) = cy * sp * sr - sy * cr;
  R(0, 2) = cy * sp * cr + sy * sr;
  R(1, 0) = sy * cp;
  R(1, 1) = sy * sp * sr + cy * cr;
  R(1, 2) = sy * sp * cr - cy * sr;
  R(2, 0) = -sp;
  R(2, 1) = cp * sr;
  R(2, 2) = cp * cr;
  return R;
}

// ─────────────────────────────────────────────────────────
// TF_t
// ─────────────────────────────────────────────────────────
TF_t::TF_t() {
  loadStaticParam();
}

TF_t::~TF_t() = default;

void TF_t::loadStaticParam() {
  // 默认：Odom 与 Map 重合
  Frame_t map_to_odom;
  map_to_odom.SetRTParams(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  frames_[std::string(FRAME_Map) + "->" + FRAME_Odom] = map_to_odom;
}

void TF_t::UpdateDynamicFrame(const std::string& frame_parent,
                              const std::string& frame_child,
                              double x, double y, double angle) {
  Frame_t frame;
  frame.SetRotationFromRPY(0.0, 0.0, angle);
  frame.SetTransVector(x, y, 0.0);
  frames_[frame_parent + "->" + frame_child] = frame;
}

TF_Err_t TF_t::LookUp(const std::string& frame_parent,
                      const std::string& frame_child,
                      double in_x, double in_y, double in_angle,
                      double& out_x, double& out_y, double& out_angle) const {
  auto it = frames_.find(frame_parent + "->" + frame_child);
  if (it != frames_.end()) {
    it->second.GetTFPose(in_x, in_y, in_angle, out_x, out_y, out_angle);
    return TF_Err_t::OK;
  }

  // 尝试查找逆变换
  auto it_inv = frames_.find(frame_child + "->" + frame_parent);
  if (it_inv != frames_.end()) {
    it_inv->second.GetInvTFPose(in_x, in_y, in_angle, out_x, out_y, out_angle);
    return TF_Err_t::OK;
  }

  return TF_Err_t::InValidChild;
}
#pragma GCC diagnostic pop
