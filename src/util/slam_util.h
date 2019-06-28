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

#ifndef ROBORTS_SLAM_UTIL_SLAM_UTIL_H
#define ROBORTS_SLAM_UTIL_SLAM_UTIL_H


#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <limits>
#include <cmath>
#include <memory>
#include <float.h>

#include <Eigen/Core>

#include <boost/timer.hpp>

#include "param_config.h"
#include "boundbox.h"

#include <boost/timer.hpp>
#include <glog/logging.h>


namespace roborts_slam {

using Pose2d   = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Vec4d    = Eigen::Vector4d;
using PoseWithIdVector = std::vector<std::pair<int, Pose2d>>;

//map
const std::string kPubMapName("pub_map");
const std::string kCoarseScanMatchMapName("front_end_coarse_scan_match_map");
const std::string kFineScanMatchMapName("front_end_fine_scan_match_map");
const std::string kBackEndCoarseScanMatchMapName("back_end_coarse_scan_match_map");
const std::string kBackEndFineScanMatchMapName("back_end_fine_scan_match_map");

const double kMaxVariance = 500.0;

const double kDoubleTolerance = 1e-06;

//#define SLAM_TIME_DEBUG

#define TIMER_START(FUNC) boost::timer t_##FUNC;
#define TIMER_END(FUNC) LOG(WARNING) << "[" << #FUNC << "]" << "cost time: " << ((t_##FUNC.elapsed()) * 1000.0) << std::endl;
#define TIMER_END_OUTPUT(FUNC) std::cout << "[" << #FUNC << "]" << "cost time: " << ((t_##FUNC.elapsed()) * 1000.0) << "ms" << std::endl;

namespace util {


inline bool DoubleEqual(double a, double b, double tolerance = kDoubleTolerance) {
  double delta = a - b;
  return delta < 0.0 ? delta >= -fabs(tolerance) : delta <= fabs(tolerance);
}

inline double Round(double value) {
  return value >= 0.0 ? std::floor(value + 0.5) : std::ceil(value - 0.5);
}

inline double MaxAbxLimit(double value, double limit) {
  if (value > fabs(limit)) {
    value = fabs(limit);
  } else if (value < -fabs(limit)) {
    value = -fabs(limit);
  }

  return value;
}

inline double SquareDistance2d(const Eigen::Vector2d &position1, const Eigen::Vector2d &position2) {
  Eigen::Vector2d diff = position1 - position2;
  return (diff.x() * diff.x() + diff.y() * diff.y());
}

inline double EuclideanDistance2d(const Eigen::Vector2d &position1, const Eigen::Vector2d &position2) {
  return sqrt(SquareDistance2d(position1, position2));
}

/**
 * Normalizes angle to be in the range of [-pi, pi]
 * @param angle to be normalized
 * @return normalized angle
 */
inline double NormalizeAngle(double angle) {

  double normal_angle = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  if (normal_angle > M_PI) {
    normal_angle -= 2.0 * M_PI;
  }

  return normal_angle;
}

inline bool PoseChangeEnough(const Eigen::Vector3d &pose1, const Eigen::Vector3d &pose2,
                             double distance_threshold, double angle_threshold) {
  //check distance
  if (((pose1.head<2>() - pose2.head<2>()).norm()) >= distance_threshold) {
    return true;
  }

  double angle_diff = NormalizeAngle(pose1.z() - pose2.z());
  if (fabs(angle_diff) >= angle_threshold) {
    return true;
  }

  return false;
}

inline double SquaredDistance(const Pose2d &pose_1, const Pose2d &pose_2) {
  return (std::pow((pose_1(0) - pose_2(0)), 2) + std::pow((pose_1(1) - pose_2(1)), 2));
}

} // namespace util

} // namespace roborts_slam



#endif //   ROBORTS_SLAM_UTIL_SLAM_UTIL_H_
