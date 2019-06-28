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

#ifndef ROBORTS_SLAM_SCAN_MATCH_OPTIMIZE_SCAN_MATCHER_H
#define ROBORTS_SLAM_SCAN_MATCH_OPTIMIZE_SCAN_MATCHER_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "map/map_manager.h"
#include "slam/sensor_data_manager.h"
#include "util/slam_util.h"


namespace roborts_slam {

class OptimizeScanMatchParam {
 public:
  double cost_decrease_threshold() const { return cost_decrease_threshold_; }
  void set_cost_decrease_threshold(double value) { cost_decrease_threshold_ = value; }

  double cost_min_threshold() const { return cost_min_threshold_; }
  void set_cost_min_threshold(double value) { cost_min_threshold_ = value; }

  int iterate_max_times() const { return iterate_max_times_; }
  void set_iterate_max_times(int value) { iterate_max_times_ = value; }

  double max_update_distance() const { return max_update_distance_; }
  void set_max_update_distance(double value) { max_update_distance_ = value; }

  double max_update_angle() const { return max_update_angle_; }
  void set_max_update_angle_(double value) { max_update_angle_ = value; }

 private:
  int iterate_max_times_;

  double cost_decrease_threshold_;
  double cost_min_threshold_;

  double max_update_distance_;
  double max_update_angle_;
};

class BasedOptimizeScanMatch {
 public:
  BasedOptimizeScanMatch() :
      H_(Eigen::Matrix3d::Zero()),
      b_(Eigen::Vector3d::Zero()) {

  }

  double ScanMatch(std::shared_ptr <ScanMatchMap> map,
                   std::shared_ptr <RangeDataContainer2d> range_data,
                   std::shared_ptr <OptimizeScanMatchParam> optimize_scan_match_param,
                   Eigen::Vector3d &best_pose) {

    if(!map->IsMapInit() || range_data->GetSize() == 0){
      LOG(WARNING) << "Invalid scan match input !";
      return kMaxCost;
    }


    Eigen::Vector3d init_pose_world = best_pose;
    Eigen::Vector3d estimate_pose = map->GetMapCoordsPose(init_pose_world);

    map_resolution_ = map->GetCellLength();


    int iterate_max_times = optimize_scan_match_param->iterate_max_times();
    for (int iter = 0; iter < iterate_max_times; ++iter) {

      last_cost_ = cost_;
      cost_ = 0.0;

      H_ = Eigen::Matrix3d::Zero();
      b_ = Eigen::Vector3d::Zero();

      Eigen::Vector2d translation(estimate_pose[0], estimate_pose[1]);
      Eigen::Matrix2d rotation;
      rotation << cos(estimate_pose[2]), -sin(estimate_pose[2]),
                  sin(estimate_pose[2]), cos(estimate_pose[2]);

      UpdateCost(estimate_pose, translation, rotation, map, range_data);

      Eigen::Vector3d det = CalculateDet();

      if (std::isnan(det[0]) || std::isnan(det[1]) || std::isnan(det[2])) {
        LOG(INFO) << "result is nan !";
        return kMaxCost;
      }

      if (iter == 0) {
        LOG(INFO) << "init cost: " << cost_;
      }

      if (iter > 0 &&
          (last_cost_ - cost_ < optimize_scan_match_param->cost_decrease_threshold() ||
              cost_ < optimize_scan_match_param->cost_min_threshold())) {
        LOG(INFO) << "optimize times: " << iter
                  << "    current cost: " << cost_;
        break;
      }

      UpdatePose(estimate_pose, det,
                 optimize_scan_match_param->max_update_distance(),
                 optimize_scan_match_param->max_update_angle());

    }

    estimate_pose[2] = util::NormalizeAngle(estimate_pose[2]);

    best_pose = map->GetWorldCoordsPose(estimate_pose);

    return cost_;

  }

 private:

  inline Eigen::Vector3d CalculateDet(void){

    Eigen::Vector3d det;
    det = H_.ldlt().solve(b_);

    return det;
  }

  inline void UpdatePose(Eigen::Vector3d& estimate_pose, Eigen::Vector3d det,
                         double max_update_distance, double max_update_angle){
    estimate_pose[0] +=
        util::MaxAbxLimit(det[0], max_update_distance / map_resolution_);
    estimate_pose[1] +=
        util::MaxAbxLimit(det[1], max_update_distance / map_resolution_);
    estimate_pose[2] +=
        util::MaxAbxLimit(det[2], max_update_angle);
  }

  void UpdateCost(const Eigen::Vector3d& estimate_pose,
                  const Eigen::Vector2d& translation,
                  const Eigen::Matrix2d& rotation,
                  std::shared_ptr <ScanMatchMap> map,
                  std::shared_ptr <RangeDataContainer2d> range_data){

    int valid_point = 1;
    int size = range_data->GetSize();
    for (int point_index = 0; point_index < size; ++point_index) {

      double error = 0.0;

      Eigen::Vector2d local_point = range_data->GetDataPoint(point_index);
      Eigen::Vector2d map_point = rotation * local_point + translation;

      if (map->PointInMap(map_point)) {

        double cell_length = map->GetCellLength();

        double x = map_point[0];
        double y = map_point[1];

        double x0 = std::floor(x);
        double y0 = std::floor(y);
        double x1 = std::ceil(x);
        double y1 = std::ceil(y);

        double m_p00 = map->GetGridProbValue(static_cast<int>(x0), static_cast<int>(y0));
        double m_p01 = map->GetGridProbValue(static_cast<int>(x0), static_cast<int>(y1));
        double m_p10 = map->GetGridProbValue(static_cast<int>(x1), static_cast<int>(y0));
        double m_p11 = map->GetGridProbValue(static_cast<int>(x1), static_cast<int>(y1));

        //linear interpolation for grid map
        double scan_match_response = ((y - y0) * (m_p11 * (x - x0) + m_p01 * (x1 - x)) +
            (y1 - y) * (m_p10 * (x - x0) + m_p00 * (x1 - x)));

        //calculate the optimize cost of this iteration
        scan_match_response =
            (scan_match_response >= 0) ? ((scan_match_response <= 1) ? (scan_match_response) : (1)) : (0);
        error = 1 - scan_match_response;
        cost_ += (error * error);

        Eigen::Matrix<double, 1, 3> J;
        Eigen::Matrix<double, 2, 3> de_s;
        Eigen::Matrix<double, 1, 2> de_m;

        de_s << 1, 0, (-sin(estimate_pose[2]) * local_point[0] - cos(estimate_pose[2]) * local_point[1]),
            0, 1, (cos(estimate_pose[2]) * local_point[0] - sin(estimate_pose[2]) * local_point[1]);

        de_m(0, 0) = (((y - y0)) * (m_p11 - m_p01) + ((y1 - y)) * (m_p10 - m_p00));
        de_m(0, 1) = (((x - x0)) * (m_p11 - m_p10) + ((x1 - x)) * (m_p01 - m_p00));

        //calculate the jacobi
        J = -de_m * de_s;

        H_ += J.transpose() * J;
        b_ += -J.transpose() * error;

        valid_point++;

      } else {
//            cost += 0.5;
      }
    }

    //normalize the cost
    cost_ *= (kCostPointSize / valid_point);
  }


 private:

  Eigen::Matrix3d H_;
  Eigen::Vector3d b_;

  double cost_;
  double last_cost_;

  double map_resolution_;

  const double kCostPointSize = 1000;
  const double kMaxCost = (1.0 * kCostPointSize);

};

} // namespace roborts_slam

#endif //ROBORTS_SLAM_SCAN_MATCH_OPTIMIZE_SCAN_MATCHER_H
