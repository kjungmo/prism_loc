#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc_fusion {
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Quaterniond so3Exp(const Eigen::Vector3d& phi);
Eigen::Vector3d so3Log(const Eigen::Quaterniond& q);
}  // namespace prism_loc_fusion
