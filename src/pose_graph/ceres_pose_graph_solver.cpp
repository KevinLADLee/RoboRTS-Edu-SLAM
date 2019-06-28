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


#include "ceres_pose_graph_solver.h"

namespace roborts_slam{

CeresSolver::CeresSolver() {

}

CeresSolver::~CeresSolver() {

}

void CeresSolver::Clear() {
  corrections_.clear();
}

void CeresSolver::Compute() {

  BuildOptimizationProblem();

  CHECK(problem_ptr_ != nullptr);
  ceres::Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_ptr_.get(), &summary);

  LOG(INFO) << summary.FullReport();

  for (std::map<int, solver::Pose2d>::const_iterator poses_iter = poses_.begin();
       poses_iter != poses_.end();
       ++poses_iter) {
    const std::map<int, solver::Pose2d>::value_type &pair = *poses_iter;
    Pose2d pose(pair.second.x, pair.second.y, pair.second.yaw_radians);
    corrections_.push_back(std::make_pair(pair.first, pose));
  }

}

const PoseWithIdVector &CeresSolver::GetCorrections() const {
  return corrections_;
}

void CeresSolver::AddNode(Vertex<roborts_slam::RangeDataContainer2d>::Ptr vertex_ptr) {
  Pose2d sensor_pose = vertex_ptr->GetDataContainer()->sensor_pose();

  int id;
  solver::Pose2d pose;

  id = vertex_ptr->GetDataContainer()->id();

  pose.x = sensor_pose[0];
  pose.y = sensor_pose[1];
  pose.yaw_radians = sensor_pose[2];
  pose.yaw_radians = solver::NormalizeAngle(pose.yaw_radians);
  if (poses_.find(id) != poses_.end()) {
    LOG(ERROR) << "Duplicate vertex with ID: " << id;
    return;
  }
  (poses_)[id] = pose;

  DLOG(INFO) << "Adding node by RangeScan " << id;

}

void CeresSolver::AddConstraint(Edge<roborts_slam::RangeDataContainer2d>::Ptr edge_ptr) {

  // Create a new edge
  solver::Constraint2d constraint2d;

  constraint2d.id_begin = edge_ptr->GetSource()->GetDataContainer()->id();

  constraint2d.id_end = edge_ptr->GetTarget()->GetDataContainer()->id();

  if (poses_.find(constraint2d.id_begin) == poses_.end()) {

    LOG(ERROR) << "Constrain2d with id_begin " << constraint2d.id_begin << " does not exist!";

    return;
  }

  if (poses_.find(constraint2d.id_end) == poses_.end()) {

    LOG(ERROR) << "Constrain2d with id_end " << constraint2d.id_end << " does not exist!";

    return;
  }

  auto edge_link_info_ptr = edge_ptr->GetLinkInfo();

  Pose2d diff = edge_link_info_ptr->GetPoseDifference();

  constraint2d.x = diff(0);
  constraint2d.y = diff(1);
  constraint2d.yaw_radians = diff(2);

  // Set the covariance of the measurement
  Matrix3d precision_matrix = edge_link_info_ptr->GetCovariance().inverse();

  Eigen::Matrix<double, 3, 3> info = Eigen::Matrix3d::Identity();

  info(0, 0) = precision_matrix(0, 0);

  info(0, 1) = info(1, 0) = precision_matrix(0, 1);

  info(0, 2) = info(2, 0) = precision_matrix(0, 2);

  info(1, 1) = precision_matrix(1, 1);

  info(1, 2) = info(2, 1) = precision_matrix(1, 2);

  info(2, 2) = precision_matrix(2, 2);

  constraint2d.information = info;

  LOG(INFO) << "Add constraint: (" << constraint2d.id_begin << " <-> " << constraint2d.id_end << ")";
  LOG(INFO) << "Pose diff: (" << constraint2d.x << ", " << constraint2d.y << ", " << constraint2d.yaw_radians << ")";
  LOG(INFO) << "Covariance: " << info.inverse();

  constraints_.push_back(constraint2d);

}

void CeresSolver::GetGraph(std::vector<Eigen::Vector2d> &nodes,
                           std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges) {
  for (const auto &it : poses_) {
    Eigen::Vector2d pose(it.second.x, it.second.y);
    nodes.push_back(pose);
  }

  for (const auto &it : constraints_) {
    Eigen::Vector2d pose_0, pose_1;
    pose_0 << poses_[it.id_begin].x, poses_[it.id_begin].y;
    pose_1 << poses_[it.id_end].x, poses_[it.id_end].y;
    edges.emplace_back(pose_0, pose_1);
  }

}

void CeresSolver::BuildOptimizationProblem() {
  CHECK(!poses_.empty());
  problem_ptr_.reset();
  problem_ptr_ = std::make_shared<ceres::Problem>();

  if (constraints_.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = nullptr;
  ceres::LocalParameterization *angle_local_parameterization = solver::AngleLocalParameterization::Create();

  for (std::vector<solver::Constraint2d>::const_iterator constraints_iter = constraints_.begin();
       constraints_iter != constraints_.end();
       ++constraints_iter) {
    const solver::Constraint2d &constraint = *constraints_iter;

    std::map<int, solver::Pose2d>::iterator pose_begin_iter = poses_.find(constraint.id_begin);
    CHECK(pose_begin_iter != poses_.end())
    << "Pose with ID: " << constraint.id_begin << " not found.";
    std::map<int, solver::Pose2d>::iterator pose_end_iter =
        poses_.find(constraint.id_end);
    CHECK(pose_end_iter != poses_.end())
    << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix3d sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        solver::PoseGraph2dErrorTerm::Create(constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);
    problem_ptr_->AddResidualBlock(cost_function,
                                   loss_function,
                                   &pose_begin_iter->second.x,
                                   &pose_begin_iter->second.y,
                                   &pose_begin_iter->second.yaw_radians,
                                   &pose_end_iter->second.x,
                                   &pose_end_iter->second.y,
                                   &pose_end_iter->second.yaw_radians);

    problem_ptr_->SetParameterization(&pose_begin_iter->second.yaw_radians,
                                      angle_local_parameterization);
    problem_ptr_->SetParameterization(&pose_end_iter->second.yaw_radians,
                                      angle_local_parameterization);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  auto pose_start_iter = poses_.begin();
  CHECK(pose_start_iter != poses_.end()) << "There are no poses.";
  problem_ptr_->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem_ptr_->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem_ptr_->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);

}

} // namespace roborts_slam