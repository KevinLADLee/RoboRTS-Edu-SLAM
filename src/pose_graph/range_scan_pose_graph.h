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


#ifndef ROBORTS_SLAM_POSE_GRAPH_RANGE_SCAN_POSE_GRAPH_H
#define ROBORTS_SLAM_POSE_GRAPH_RANGE_SCAN_POSE_GRAPH_H

#include "pose_graph.h"
#include "slam/sensor_data_manager.h"

#include "ceres_pose_graph_solver.h"

namespace roborts_slam{

using RangeDataContainer2dPtr = std::shared_ptr<RangeDataContainer2d>;
using ScanMatchFunc = std::function<double(const std::shared_ptr<RangeDataContainer2d>&,
                             int,
                             const std::vector<int>&,
                             Eigen::Vector3d&,
                             Eigen::Matrix3d&,
                             bool, bool)>;

using CorrectPosesFunc = std::function<void(const std::vector<std::pair<int, Pose2d>>&)>;

class RangeScanPoseGraph : public PoseGraph<RangeDataContainer2d> {

 public:
  RangeScanPoseGraph(std::shared_ptr<ParamConfig> param,
                     std::shared_ptr<SensorDataManager> sensor_data_manager_ptr,
                     const ScanMatchFunc &scan_match_function,
                     const CorrectPosesFunc &correct_poses_function);

  void UpdateGraph(const int &range_data_container_id, const Matrix3d &covariance);

  bool TryCloseLoop(const int &current_range_data_id);

  void GetGraphFromSolver(std::vector<Eigen::Vector2d> &poses, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &constraints);

  void ForceComputeByCeres();

 private:

  void AddVertex(const int &range_data_container_id);

  void AddEdge(const int &range_data_container_id, const Matrix3d& covariance);

  void AddEdge(const int &source_range_data_container_id,
               const int &target_range_data_container_id,
               bool &is_new_edge,
               std::shared_ptr<Edge<RangeDataContainer2d>> &edge_ptr);

  void LinkScans(const int &source_range_scan_id,
                 const int &target_range_scan_id,
                 const Pose2d &mean,
                 const Matrix3d &covariance);

  void LinkNearChains(const int &range_data_container_id);

  int LinkChainToScan(const std::vector<int> &chain_ids, const int &range_data_container_id, const Pose2d& mean, const Matrix3d& covariance);

  std::vector<std::vector<int>> FindNearChainsIds(const int &range_data_container_id);

  std::vector<int> FindNearLinkedScans(const int &range_data_container_id);

  bool NearScanVisitor(Vertex<RangeDataContainer2d>::Ptr vertex_ptr);

  std::vector<int> FindPossibleLoopClosure(const int &range_data_container_id,
                                           int &start_id);

  bool ComparePoseDistanceToLinkScanMaxDistance(const Pose2d &pose_1, const Pose2d &pose_2);

  int FindClosestRangeScanId(const std::vector<int> &chain_ids,
                             const int &range_data_container_id);

 private:
  std::shared_ptr<SensorDataManager> sensor_data_manager_ptr_;

  //Callback function, passed by slam processor
  ScanMatchFunc scan_match_function_;
  CorrectPosesFunc correct_poses_function_;

  //Parameters
  int loop_match_min_chain_size_;
  double link_match_min_response_;
  double link_scan_max_distance_;
  double loop_match_min_response_coarse_;
  double loop_match_max_variance_coarse_;
  double loop_match_min_response_fine_;

  //Data
  double link_scan_max_distance_square_;
  int current_range_data_id_;
  int current_center_id_;
  Pose2d current_center_pose_;

};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_POSE_GRAPH_RANGE_SCAN_POSE_GRAPH_H