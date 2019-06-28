/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/


#ifndef ROBORTS_SLAM_POSE_GRAPH_CERES_TYPES_H
#define ROBORTS_SLAM_POSE_GRAPH_CERES_TYPES_H

#include "ceres/ceres.h"
#include <Eigen/Core>

namespace roborts_slam {
namespace solver {

struct Pose2d {
  double x;
  double y;
  double yaw_radians;

};

struct Constraint2d {
  int id_begin;
  int id_end;

  double x = 0;
  double y = 0;
  double yaw_radians = 0;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, and yaw.
  Eigen::Matrix3d information;

};

// Normalizes the angle in radians between [-pi and pi).
template<typename T>
inline T NormalizeAngle(const T &angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
      two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}


class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

class PoseGraph2dErrorTerm {
 public:
  PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians,
                       const Eigen::Matrix3d& sqrt_information)
      : p_ab_(x_ab, y_ab),
        yaw_ab_radians_(yaw_ab_radians),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                  const T* const x_b, const T* const y_b, const T* const yaw_b,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
    const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals_ptr);

    residuals_map.template head<2>() =
        RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) -
            p_ab_.cast<T>();
    residuals_map(2) = solver::NormalizeAngle(
        (*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));

    // Scale the residuals by the square root information matrix to account for
    // the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction* Create(double x_ab, double y_ab,
                                     double yaw_ab_radians,
                                     const Eigen::Matrix3d& sqrt_information) {
    return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1,
                                            1, 1>(new PoseGraph2dErrorTerm(
        x_ab, y_ab, yaw_ab_radians, sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

} // namespace solver
} // namespace roborts_slam

#endif // ROBORTS_SLAM_POSE_GRAPH_CERES_TYPES_H
