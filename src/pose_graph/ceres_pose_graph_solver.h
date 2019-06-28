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

#ifndef ROBORTS_SLAM_POSE_GRAPH_CERES_POSE_GRAPH_SOLVER_H
#define ROBORTS_SLAM_POSE_GRAPH_CERES_POSE_GRAPH_SOLVER_H

#include <vector>

#include <Eigen/Dense>

#include "ceres_types.h"
#include "pose_graph.h"
#include "slam/sensor_data_manager.h"

namespace roborts_slam {

class CeresSolver : public PoseGraphSolver<roborts_slam::RangeDataContainer2d> {
 public:
  CeresSolver();

  ~CeresSolver();

  void Clear();

  void Compute();

  const PoseWithIdVector &GetCorrections() const;

  void AddNode(Vertex<roborts_slam::RangeDataContainer2d>::Ptr vertex_ptr);

  void AddConstraint(Edge<roborts_slam::RangeDataContainer2d>::Ptr edge_ptr);

  void GetGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges);

 private:
  void BuildOptimizationProblem();

 private:
  PoseWithIdVector corrections_;

  std::shared_ptr<ceres::Problem> problem_ptr_;

  std::map<int, solver::Pose2d, std::less<int>> poses_;
  std::vector<solver::Constraint2d> constraints_;

};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_POSE_GRAPH_CERES_POSE_GRAPH_SOLVER_H
