/*
 * tf.h
 * 坐标变换库（TF Tree）
 * 迁移自 history/212alpha/NavUtils/Tf/tf.h
 * 基于 Eigen3 实现 2D/3D 坐标帧变换。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

// ── 坐标系名称常量 ────────────────────────────────────────
#define FRAME_Map  "Map"
#define FRAME_Odom "Odom"
#define FRAME_X1C  "X1C"
#define FRAME_XD1Q "XD1Q"
#define FRAME_XD1Y "XD1Y"

enum TF_Err_t {
  OK = 0,
  InValidParent,
  InValidChild,
  Error,
};

// ── 单帧变换 ──────────────────────────────────────────────
// 描述 parent → child 的刚体变换：
//   p_parent = R * p_child + t
class Frame_t {
public:
  Frame_t();
  Frame_t(const Eigen::Matrix3d& r_, const Eigen::Vector3d& v_);
  ~Frame_t();

  void SetRotationFromRPY(double roll_, double pitch_, double yaw_);
  Eigen::Matrix3d GetRotation() const;
  void SetTransVector(double tx_, double ty_, double tz_);
  Eigen::Vector3d GetTransVector() const;
  void SetRTParams(double tx_, double ty_, double tz_,
                   double roll_, double pitch_, double yaw_);

  // child frame 中的向量 → parent frame
  Eigen::Vector3d GetTFVector(const Eigen::Vector3d& v_) const;
  // parent frame 中的向量 → child frame
  Eigen::Vector3d GetInvTFVector(const Eigen::Vector3d& v_) const;

  // 位姿变换（x, y 为平面坐标，angle 为绕 Z 轴弧度）
  // GetTFPose:   child → parent
  // GetInvTFPose: parent → child
  void GetTFPose(double x, double y, double angle,
                 double& out_x, double& out_y, double& out_angle) const;
  void GetInvTFPose(double x, double y, double angle,
                    double& out_x, double& out_y, double& out_angle) const;

  static Eigen::Matrix3d GetMatrixFromRPY(double roll_, double pitch_, double yaw_);

private:
  Eigen::Matrix3d r;  // 旋转矩阵 (parent = R * child)
  Eigen::Vector3d v;  // 平移向量 (parent = R * child + t)
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  std::string name;
};

// ── TF 树 ─────────────────────────────────────────────────
// 简化实现：维护 <parent, child> → Frame_t 的映射，
// 支持直接查找，暂不支持多跳链式查询。
/*      TF Tree
 *   Map---->Odom---->X1C
 *                 |-->XD1Q
 *                 |-->XD1Y
 */
class TF_t {
public:
  TF_t();
  ~TF_t();

  // 更新/添加一条动态变换（parent → child）
  void UpdateDynamicFrame(const std::string& frame_parent,
                          const std::string& frame_child,
                          double x, double y, double angle);

  // 查询：将 child frame 中的位姿转换到 parent frame
  TF_Err_t LookUp(const std::string& frame_parent,
                  const std::string& frame_child,
                  double in_x, double in_y, double in_angle,
                  double& out_x, double& out_y, double& out_angle) const;

private:
  std::map<std::string, Frame_t> frames_;  // key = "parent->child"
  void loadStaticParam();
};
