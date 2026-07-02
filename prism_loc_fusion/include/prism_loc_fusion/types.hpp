#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc_fusion {
struct NominalState {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
};
struct EskfParams {
  double sigma_acc{1e-2};
  double sigma_gyro{1e-3};
  double sigma_acc_bias{1e-4};
  double sigma_gyro_bias{1e-5};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.81)};
};
}  // namespace prism_loc_fusion
