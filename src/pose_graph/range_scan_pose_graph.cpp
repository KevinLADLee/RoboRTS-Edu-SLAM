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


#include "range_scan_pose_graph.h"

namespace roborts_slam {

RangeScanPoseGraph::RangeScanPoseGraph(std::shared_ptr<ParamConfig> param,
                                       std::shared_ptr<SensorDataManager> sensor_data_manager_ptr,
                                       const ScanMatchFunc &scan_match_function,
                                       const CorrectPosesFunc &correct_poses_function) {

  solver_ptr_ = std::make_unique<CeresSolver>();

  loop_match_min_chain_size_ = param->loop_match_min_chain_size();
  link_match_min_response_ = param->link_match_min_response();
  link_scan_max_distance_ = param->link_scan_max_distance();
  loop_match_min_response_coarse_ = param->loop_match_min_response_coarse();
  loop_match_max_variance_coarse_ = param->loop_match_max_variance_coarse();
  loop_match_min_response_fine_ = param->loop_match_min_response_fine();

  link_scan_max_distance_square_ = std::pow(link_scan_max_distance_, 2);
  sensor_data_manager_ptr_ = sensor_data_manager_ptr;
  scan_match_function_ = scan_match_function;
  correct_poses_function_ = correct_poses_function;

}

void RangeScanPoseGraph::UpdateGraph(const int &range_data_container_id, const Matrix3d &covariance) {
  current_range_data_id_ = range_data_container_id;
  AddVertex(current_range_data_id_);
  AddEdge(current_range_data_id_, covariance);
}

void RangeScanPoseGraph::AddVertex(const int &range_data_container_id) {
  //TODO Mutex
  auto range_data_container_ptr = sensor_data_manager_ptr_->GetRangeData(range_data_container_id);
  auto vertex_ptr = std::make_shared<Vertex<RangeDataContainer2d>>(range_data_container_ptr);
  PoseGraph::AddVertex(vertex_ptr);
}

void RangeScanPoseGraph::AddEdge(const int &range_data_container_id, const Matrix3d &covariance) {

  // link to previous scan
  int previous_scan_id = range_data_container_id - 1;

  if(range_data_container_id > 0) {
    CHECK_GE(previous_scan_id, 0) << "RangeScanPoseGraph::AddEdge: previous_scan_id MUST >0!";
    LinkScans(previous_scan_id,
              range_data_container_id,
              sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->sensor_pose(),
              covariance);
  }

//  std::vector<Pose2d> means;
//  std::vector<Matrix3d> covariances;

  // link to other near chains
//  TIMER_START(AddEdge_LinkNearChains);
  LinkNearChains(range_data_container_id);
//  TIMER_END(AddEdge_LinkNearChains);

}

void RangeScanPoseGraph::AddEdge(const int &source_range_scan_id,
                                 const int &target_range_scan_id,
                                 bool &is_new_edge,
                                 std::shared_ptr<Edge<RangeDataContainer2d>> &edge_ptr) {

  auto vertex_source = vertex_vec_[source_range_scan_id];
  auto vertex_target = vertex_vec_[target_range_scan_id];

  for (const Edge<RangeDataContainer2d>::Ptr &it : vertex_source->GetEdges()) {
    if (it->GetTarget() == vertex_target) {
      is_new_edge = false;
      edge_ptr = it;
      return;
    }
  };

  edge_ptr = std::make_shared<Edge<RangeDataContainer2d>>();
  edge_ptr->AddVertex(vertex_source, vertex_target);
  is_new_edge = true;
  PoseGraph::AddEdge(edge_ptr);
}

void RangeScanPoseGraph::LinkScans(const int &source_range_scan_id,
                                   const int &target_range_scan_id,
                                   const Pose2d &mean,
                                   const Matrix3d &covariance) {
  bool is_new_edge = true;
  std::shared_ptr<Edge<RangeDataContainer2d>> edge_ptr = nullptr;
  AddEdge(source_range_scan_id, target_range_scan_id, is_new_edge, edge_ptr);

  // only attach link information if the edge is new
  if (is_new_edge) {
    auto scan_pose = sensor_data_manager_ptr_->GetRangeData(source_range_scan_id)->sensor_pose();
    edge_ptr->SetLinkInfo(std::make_shared<EdgeLinkInfo>(scan_pose, mean, covariance));
    if (solver_ptr_ != nullptr) {
      solver_ptr_->AddConstraint(edge_ptr);
    }
  }
}

void RangeScanPoseGraph::LinkNearChains(const int &range_data_container_id) {
//  TIMER_START(FindNearChainsIds);
  const auto near_chains = FindNearChainsIds(range_data_container_id);
//  TIMER_END(FindNearChainsIds);

  for (auto &it : near_chains) {
    if (it.size() < loop_match_min_chain_size_) {
      continue;
    }

    std::vector<int> sparse_chain_ids;
    int i = 0;
    if(it.size() > 10){
      for(auto &id : it){
        if(i%2 == 0){
          sparse_chain_ids.push_back(id);
        }
        if(sparse_chain_ids.size() > 10){
          break;
        };
        i++;
      }
    } else{
      sparse_chain_ids = it;
    }

    //TODO Mutex
    Pose2d mean = sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->sensor_pose();
    Matrix3d covariance;
    // match scan against "near" chain
//    TIMER_START(LinkNearChains_scan_match_function);
    LOG(WARNING) << "Near chain size: " << (int)(sparse_chain_ids.size());
    auto closest_range_scan_id = FindClosestRangeScanId(sparse_chain_ids, range_data_container_id);
    auto response = scan_match_function_(sensor_data_manager_ptr_->GetRangeData(range_data_container_id), closest_range_scan_id, sparse_chain_ids, mean, covariance, true, true);
    CHECK(!std::isnan(covariance(0,0))) << "Covariance NAN!";
    LOG(INFO) << "Link near chain response: " << response;
    LOG(INFO) << "Link near chain covariance: " << covariance;
//    TIMER_END(LinkNearChains_scan_match_function);

    if (response > link_match_min_response_) {
//      TIMER_START(LinkNearChains_LinkChainToScan);
      LinkChainToScan(it, range_data_container_id, mean, covariance);
//      TIMER_END(LinkNearChains_LinkChainToScan);
    }
  }


}

int RangeScanPoseGraph::LinkChainToScan(const std::vector<int> &chain_ids,
                                        const int &range_data_container_id,
                                        const Pose2d &mean,
                                        const Matrix3d &covariance) {

  int closest_scan_id = FindClosestRangeScanId(chain_ids, range_data_container_id);
  Pose2d scan_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->barycenter_pose();

  auto closest_scan_ptr = sensor_data_manager_ptr_->GetRangeData(closest_scan_id);
  CHECK(closest_scan_id >= 0) << "LinkChainToScan: ERROR! closest_scan_id < 0!";
  Pose2d closest_scan_barycenter_pose = closest_scan_ptr->barycenter_pose();

  if(ComparePoseDistanceToLinkScanMaxDistance(scan_barycenter_pose, closest_scan_barycenter_pose)){
    LinkScans(closest_scan_id, range_data_container_id, mean, covariance);
  }

//  double squaredDistance = util::SquaredDistance(scan_barycenter_pose, closest_scan_barycenter_pose);
//  if (squaredDistance < std::pow(link_scan_max_distance_, 2)) {
//    LinkScans(closest_scan_id, range_data_container_id, mean, covariance);
//  }
  return closest_scan_id;
}

int RangeScanPoseGraph::FindClosestRangeScanId(const std::vector<int> &chain_ids, const int &range_data_container_id) {
  Pose2d scan_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->barycenter_pose();
  double best_squared_distance = DBL_MAX;
  int closest_scan_id = -1;
  for (const auto &it : chain_ids) {
    Pose2d it_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(it)->barycenter_pose();
    double squared_distance = util::SquaredDistance(scan_barycenter_pose, it_barycenter_pose);
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      closest_scan_id = it;
    }
  }
  return closest_scan_id;
}

std::vector<std::vector<int>> RangeScanPoseGraph::FindNearChainsIds(const int &range_data_container_id) {
  std::vector<std::vector<int>> near_chains_ids;
  Pose2d scan_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->barycenter_pose();

  std::vector<int> scan_id_processed;

  const std::vector<int> near_linked_scan_ids = FindNearLinkedScans(range_data_container_id);


  for (const auto &it : near_linked_scan_ids) {
    auto near_scan_id = it;
    if (it == range_data_container_id) {
      continue;
    }

    if (std::find(scan_id_processed.begin(), scan_id_processed.end(), near_scan_id) != scan_id_processed.end()) {
      continue;
    }

    scan_id_processed.push_back(near_scan_id);

    bool is_valid_chain = true;
    std::list<int> chain_ids;
    for (int candidate_scan_num = near_scan_id - 1; candidate_scan_num >= 0; candidate_scan_num--) {

      if (candidate_scan_num == range_data_container_id) {
        is_valid_chain = false;
      }

      Pose2d candidate_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(candidate_scan_num)->barycenter_pose();
      if(ComparePoseDistanceToLinkScanMaxDistance(scan_barycenter_pose, candidate_barycenter_pose)){
        chain_ids.push_front(candidate_scan_num);
        scan_id_processed.push_back(candidate_scan_num);
      } else {
        break;
      }
    }

    chain_ids.push_back(near_scan_id);
    int end = current_range_data_id_;
    for (int candidate_scan_id = near_scan_id + 1; candidate_scan_id < end; candidate_scan_id++) {

      if (candidate_scan_id == range_data_container_id) {
        is_valid_chain = false;
      }

      Pose2d candidate_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(candidate_scan_id)->barycenter_pose();
      if(ComparePoseDistanceToLinkScanMaxDistance(scan_barycenter_pose, candidate_barycenter_pose)){
        chain_ids.push_back(candidate_scan_id);
        scan_id_processed.push_back(candidate_scan_id);
      } else {
        break;
      }
    }

    if (is_valid_chain) {
      std::vector<int> tmp_chain;
      std::copy(chain_ids.begin(), chain_ids.end(), std::inserter(tmp_chain, tmp_chain.begin()));
      near_chains_ids.push_back(tmp_chain);
    }
  }

  return near_chains_ids;
}

std::vector<int> RangeScanPoseGraph::FindNearLinkedScans(const int &range_data_container_id) {
  current_center_pose_ = sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->barycenter_pose();
  std::vector<RangeDataContainer2dPtr> range_scan_ptr_vec = BreadthFirstTraverse(GetVertex(range_data_container_id),
                                                                                 std::bind(&RangeScanPoseGraph::NearScanVisitor,
                                                                                           this,
                                                                                           std::placeholders::_1));
  std::vector<int> range_scan_ids;

  std::stringstream id_str;
  id_str << "ID<" << range_data_container_id << ">" << "Near Linked Scans id: ";
  for(const auto &it : range_scan_ptr_vec){
    range_scan_ids.emplace_back(it->id());
    id_str << " " << it->id();
  }
  LOG(INFO) << id_str.str();

  return range_scan_ids;

}

bool RangeScanPoseGraph::NearScanVisitor(Vertex<RangeDataContainer2d>::Ptr vertex_ptr) {
  auto range_scan_ptr = vertex_ptr->GetDataContainer();
  Pose2d pose = range_scan_ptr->barycenter_pose();
  double squaredDistance = util::SquaredDistance(pose, current_center_pose_);
  return (squaredDistance <= std::pow(link_scan_max_distance_, 2));
}

bool RangeScanPoseGraph::TryCloseLoop(const int &range_data_container_id){
  bool loop_closed = false;

  int start_range_data_container_id = 0;

  std::vector<int> candidate_chain_ids = FindPossibleLoopClosure(range_data_container_id, start_range_data_container_id);

  while (!candidate_chain_ids.empty()) {
    auto range_data_ptr = sensor_data_manager_ptr_->GetRangeData(range_data_container_id);
    Pose2d best_pose = range_data_ptr->sensor_pose();
    Matrix3d covariance;

    auto closest_range_scan_id = FindClosestRangeScanId(candidate_chain_ids, range_data_container_id);
    double coarse_response = scan_match_function_(range_data_ptr,
                                                  closest_range_scan_id,
                                                  candidate_chain_ids,
                                                  best_pose,
                                                  covariance,
                                                  true,
                                                  true);
    LOG(INFO) << "Loop closure response (coarse): " << coarse_response;
    LOG(INFO) << "Covariance: " << covariance;

    if ((coarse_response > loop_match_min_response_coarse_ &&
        (covariance(0, 0) < loop_match_max_variance_coarse_ &&
            (covariance(1, 1) < loop_match_max_variance_coarse_)))) {

      auto tmp_scan_ptr = std::make_shared<RangeDataContainer2d>();
      tmp_scan_ptr->CreateFrom(range_data_ptr, 1.0);
      tmp_scan_ptr->set_sensor_pose(best_pose);
      double fine_response = scan_match_function_(tmp_scan_ptr, closest_range_scan_id, candidate_chain_ids, best_pose, covariance, true, true);
      LOG(INFO) << "Loop closure response (fine): " << fine_response;
      LOG(INFO) << "Best pose: " << best_pose.transpose();

      if (fine_response < loop_match_min_response_fine_) {
        LOG(WARNING) << "REJECTED loop closure!";
      } else {

        range_data_ptr->set_sensor_pose(best_pose);

        auto closest_loop_scan_id =  LinkChainToScan(candidate_chain_ids, range_data_container_id, best_pose, covariance);

        if (solver_ptr_ != nullptr) {
          LOG(INFO) << "SUCCSSED loop closure! " << "(" << range_data_container_id << ", " << closest_loop_scan_id << ")";
          solver_ptr_->Compute();
          correct_poses_function_(solver_ptr_->GetCorrections());
          solver_ptr_->Clear();
        }

        loop_closed = true;
      }
    }
    candidate_chain_ids = FindPossibleLoopClosure(range_data_container_id, start_range_data_container_id);
  }

  return loop_closed;
}

std::vector<int> RangeScanPoseGraph::FindPossibleLoopClosure(const int &range_data_container_id, int &start_id) {

  std::vector<int> chain_ids;  // return value

  Pose2d barycenter_pose = sensor_data_manager_ptr_->GetRangeData(range_data_container_id)->barycenter_pose();

  // possible loop closure chain should not include close scans that have a
  // path of links to the scan of interest
  const std::vector<int> near_linked_scan_ids = FindNearLinkedScans(range_data_container_id);

  int scans_num = sensor_data_manager_ptr_->current_data_index();

  for (; start_id < scans_num; start_id++) {
    auto candidate_scan_id = start_id;
    Pose2d candidate_scan_barycenter_pose = sensor_data_manager_ptr_->GetRangeData(start_id)->barycenter_pose();

    if(ComparePoseDistanceToLinkScanMaxDistance(candidate_scan_barycenter_pose, barycenter_pose)){
      // a linked scan cannot be in the chain
      if (find(near_linked_scan_ids.begin(), near_linked_scan_ids.end(), candidate_scan_id)
          != near_linked_scan_ids.end()) {
        chain_ids.clear();
      } else {
        chain_ids.push_back(candidate_scan_id);
      }
    } else {
      // return chain if it is long "enough"
      if (chain_ids.size() >= loop_match_min_chain_size_) {
        return chain_ids;
      } else {
        chain_ids.clear();
      }
    }
  }

  return chain_ids;
}

void RangeScanPoseGraph::GetGraphFromSolver(std::vector<Eigen::Vector2d> &poses,
                                            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &constraints)
{
  solver_ptr_->GetGraph(poses, constraints);
}

void RangeScanPoseGraph::ForceComputeByCeres() {
  if (solver_ptr_ != nullptr) {
    LOG(INFO) << "Force graph optimization!";
    solver_ptr_->Compute();
    correct_poses_function_(solver_ptr_->GetCorrections());
    solver_ptr_->Clear();
  }
}

bool RangeScanPoseGraph::ComparePoseDistanceToLinkScanMaxDistance(const Pose2d &pose_1, const Pose2d &pose_2) {
  return util::SquaredDistance(pose_1, pose_2) < link_scan_max_distance_square_;
}

} // namespace roborts_slam
