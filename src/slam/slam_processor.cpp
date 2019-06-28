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

#include "slam_processor.h"

namespace roborts_slam{

SlamProcessor::SlamProcessor(std::shared_ptr<ParamConfig> param,
                             std::shared_ptr<SensorDataManager> sensor_data_manager, double map_resolution):
    param_(param),
    sensor_data_manager_(sensor_data_manager),
    map_resolution_(map_resolution),
    pub_map_created_(false),
    map_manager_(std::make_shared<MapManager>()),
    last_map_update_pose_(Eigen::Vector3d(FLT_MAX, FLT_MAX, FLT_MAX)),
    current_sensor_pose_(Eigen::Vector3d::Zero()){

  CHECK_GT(map_resolution, 0.0) << "Map resolution must greater than 0!";
  LOG(INFO) << "map_resolution " << map_resolution_;

  SlamParamInit();

  scan_matchers_ = std::make_unique<ScanMatchers>(front_end_scan_match_param_, sensor_data_manager_);

  //create backend pose_graph object
  range_data_pose_graph_ = std::make_unique<RangeScanPoseGraph>(param_,
                                                                sensor_data_manager_,
                                                                std::bind(&SlamProcessor::ScanMatchInterface,
                                                                          this,
                                                                          std::placeholders::_1,
                                                                          std::placeholders::_2,
                                                                          std::placeholders::_3,
                                                                          std::placeholders::_4,
                                                                          std::placeholders::_5,
                                                                          std::placeholders::_6,
                                                                          std::placeholders::_7),
                                                                std::bind(&SlamProcessor::CorrectPoseAndMap,
                                                                          this,
                                                                          std::placeholders::_1));

  map_penalize_times_ = 0;
  last_process_time_ = std::chrono::steady_clock::now();

}

SlamProcessor::~SlamProcessor(){
  back_end_running_ = false;
  back_end_thread_->join();
};

bool SlamProcessor::process(void){

  bool res = false;

  current_data_index_ = sensor_data_manager_->current_data_index();
  is_first_range_data_ = sensor_data_manager_->IsFirstRangeData();

//    close_loop_ = false;

  if(is_first_range_data_){

    //the first frame, init all map
    CreateAllMap();

    //start backend process thread
    new_vertex_ = false;
    back_end_running_ = true;
    back_end_thread_ = std::make_unique<std::thread>(std::bind(&SlamProcessor::BackEndProcessThread, this));

    current_sensor_pose_ = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d cur_odom_pose = sensor_data_manager_->GetOdomPose();
  Eigen::Vector3d last_odom_pose = sensor_data_manager_->GetOdomPose(current_data_index_ - 1);
  Eigen::Vector3d predict_sensor_pose(current_sensor_pose_);

  //check the robot if move enough, avoid accumulating too many invalid sensor data
  if((MoveEnough(cur_odom_pose, last_odom_pose) || !use_move_check_) ||
      is_first_range_data_){

    std::cout << "current_data_index_ " << current_data_index_ << std::endl << std::endl;

    LOG(INFO) << "SlamProcess : " << current_data_index_ << "  *********************************************************************";

//      std::unique_lock<std::mutex> front_end_lock(front_end_mutex_);

#ifdef SLAM_TIME_DEBUG
    TIMER_START(SlamFrontEndProcess);
#endif

    //create range datas of different resolutions for each map

    std::shared_ptr<RangeDataContainer2d> pub_map_range_data = std::make_shared<RangeDataContainer2d>();
    pub_map_range_data->CreateFrom(sensor_data_manager_->GetRangeData(), 1 / map_resolution_);

    std::shared_ptr<RangeDataContainer2d> coarse_map_range_data = std::make_shared<RangeDataContainer2d>();
    coarse_map_range_data->CreateFrom(sensor_data_manager_->GetRangeData(), 1 / param_->coarse_map_resolution());

    std::shared_ptr<RangeDataContainer2d> fine_map_range_data = std::make_shared<RangeDataContainer2d>();
    fine_map_range_data->CreateFrom(sensor_data_manager_->GetRangeData(), 1 / param_->fine_map_resolution());


    Eigen::Matrix3d process_cov_matrix = Eigen::Matrix3d::Identity();

    //do not scanmatch for first frame
    if(!is_first_range_data_) {

      if(use_odometry_){
        predict_sensor_pose = PredictPoseByOdom(current_sensor_pose_,
                                                last_odom_pose, cur_odom_pose);
        LOG(INFO) << "predict_sensor_pose " << predict_sensor_pose.transpose();
      }


#ifdef SLAM_TIME_DEBUG
      TIMER_START(FrontEndScanMatch);
#endif

      Eigen::Vector3d process_sensor_pose(predict_sensor_pose);
      if(!kUseRunningRangeScanMatch){

        {
          std::unique_lock<std::mutex> map_lock(map_mutex_);

          std::unique_lock<std::mutex> scan_match_lock(scan_matchers_->GetScanMatchMutex());

          //Do scanmatch for range data and map ,and get the response score
          scan_matchers_->SetScanMatchParam(front_end_scan_match_param_);
          scan_match_score_ = scan_matchers_->ScanMatch(coarse_map_range_data,
                                                        fine_map_range_data,
                                                        map_manager_->GetScanMatchMap(kCoarseScanMatchMapName),
                                                        map_manager_->GetScanMatchMap(kFineScanMatchMapName),
                                                        process_sensor_pose,
                                                        process_cov_matrix);
        }

      }else{
        //debug test, do not use
        std::shared_ptr<RangeDataContainer2d> range_data = std::make_shared<RangeDataContainer2d>();
        range_data->CreateFrom(sensor_data_manager_->GetRangeData(), 1.0);
        scan_match_score_ = this->ScanMatchInterface(range_data, 0,
                                                     sensor_data_manager_->GetRunningRangeIdVec(),
                                                     process_sensor_pose,
                                                     process_cov_matrix);
      }

#ifdef SLAM_TIME_DEBUG
      TIMER_END_OUTPUT(FrontEndScanMatch);
#endif


      //check the scanmatch result by remapping the range data to map
      double map_penalty = MapCheckPenalize(pub_map_range_data, process_sensor_pose);
      if(map_penalize_times_ < 5){
        scan_match_score_ *= map_penalty;
        scan_match_score_ = (scan_match_score_ > 1.0) ? (1.0) : (scan_match_score_);
        if(map_penalty < 0.7){
          map_penalize_times_++;
        }else{
          map_penalize_times_ = 0;
        }
      }else{
        map_penalize_times_ = 0;
      }

      LOG(INFO) << "scan_match_score_ : " << scan_match_score_;
      //check scanmatch score, if score is low,do not update sensor pose
      if(scan_match_score_ > std::max(0.5, map_update_score_threshold_)){
        current_sensor_pose_ = process_sensor_pose;
        res = true;
      }

    }


    LOG(INFO) << "current_sensor_pose_ : " << current_sensor_pose_.transpose();
//      LOG(INFO) << "cur_odom_pose : " << cur_odom_pose.transpose();
//      LOG(INFO) << "last_odom_pose : " << last_odom_pose.transpose();

    // update sensor pose and id for all range data

    sensor_data_manager_->GetRangeData()->set_sensor_pose(current_sensor_pose_);
    sensor_data_manager_->GetRangeData()->set_id(current_data_index_);

    coarse_map_range_data->set_sensor_pose(current_sensor_pose_);
    coarse_map_range_data->set_id(current_data_index_);

    fine_map_range_data->set_sensor_pose(current_sensor_pose_);
    fine_map_range_data->set_id(current_data_index_);

    pub_map_range_data->set_sensor_pose(current_sensor_pose_);
    pub_map_range_data->set_id(current_data_index_);

//      LOG(INFO) << "update_range_data !";


#ifdef SLAM_TIME_DEBUG
    TIMER_START(UpdateAllMap);
#endif

    //update map with range data
    if(UpdateMap(pub_map_range_data, coarse_map_range_data, fine_map_range_data)){

      sensor_data_manager_->AddMultiresolutionRangeData(kPubMapName, pub_map_range_data);
      sensor_data_manager_->AddMultiresolutionRangeData(kCoarseScanMatchMapName, coarse_map_range_data);
      sensor_data_manager_->AddMultiresolutionRangeData(kFineScanMatchMapName, fine_map_range_data);
      sensor_data_manager_->UpdateRunningRange(current_data_index_);

      //send back_end process signal
      BackEndDataUpdataSignal(current_data_index_, process_cov_matrix);

    }else {
      //if map does not update, clear current range data from sensor data manager
      sensor_data_manager_->ClearCurrentData();
    }

#ifdef SLAM_TIME_DEBUG
    TIMER_END_OUTPUT(UpdateAllMap);
#endif

#ifdef SLAM_TIME_DEBUG
    TIMER_END_OUTPUT(SlamFrontEndProcess);
#endif

//      return res;

  }else {
    //if robot did not move enough, clear current range data from sensor data manager
    sensor_data_manager_->ClearCurrentData();
  }

  return res;
}


double SlamProcessor::ScanMatchInterface(const std::shared_ptr<RangeDataContainer2d>& range_data,
                                         int closest_id,
                                         const std::vector<int>& range_id,
                                         Eigen::Vector3d& best_pose,
                                         Eigen::Matrix3d& cov_matrix,
                                         bool use_fine_scan_match,
                                         bool use_front_end_scan_match_param){

  double scan_match_score = 0.0;

//    CHECK_LE(closest_id, current_data_index_) << "Find a error closest id!";

#ifdef SLAM_TIME_DEBUG
  TIMER_START(BackEndScanMatch);
#endif

  //create range datas of different resolutions for each map

  std::shared_ptr<RangeDataContainer2d> coarse_map_range_data = std::make_shared<RangeDataContainer2d>();
  coarse_map_range_data->CreateFrom(range_data, 1 / param_->coarse_map_resolution());

  std::shared_ptr<RangeDataContainer2d> fine_map_range_data = std::make_shared<RangeDataContainer2d>();
  fine_map_range_data->CreateFrom(range_data, 1 / param_->fine_map_resolution());

  std::shared_ptr<ScanMatchMap> coarse_map = map_manager_->GetScanMatchMap(kBackEndCoarseScanMatchMapName);
  std::shared_ptr<ScanMatchMap> fine_map = map_manager_->GetScanMatchMap(kBackEndFineScanMatchMapName);

#ifdef SLAM_TIME_DEBUG
  TIMER_START(BackEndResetMap);
#endif

  //update map with given range data
  ResetScanMatchMapWithRangeVec(sensor_data_manager_->GetMultiresolutionRangeDataVecWithId(kCoarseScanMatchMapName, range_id),
                                coarse_map, param_->coarse_map_use_blur());
  ResetScanMatchMapWithRangeVec(sensor_data_manager_->GetMultiresolutionRangeDataVecWithId(kFineScanMatchMapName, range_id),
                                fine_map, param_->fine_map_use_blur());

#ifdef SLAM_TIME_DEBUG
  TIMER_END_OUTPUT(BackEndResetMap);
#endif

  {
    std::unique_lock<std::mutex> scan_match_lock(scan_matchers_->GetScanMatchMutex());

    if(use_front_end_scan_match_param){
      scan_matchers_->SetScanMatchParam(front_end_scan_match_param_);
    }else{
      scan_matchers_->SetScanMatchParam(back_end_scan_match_param_);
    }

    //scan match
    scan_match_score = scan_matchers_->ScanMatch(coarse_map_range_data,
                                                 fine_map_range_data,
                                                 coarse_map,
                                                 fine_map,
                                                 best_pose,
                                                 cov_matrix,
                                                 use_fine_scan_match);
  }

  LOG(INFO) << "back_end_scan_match_pose " << best_pose.transpose();

  //check scanmatch result with pub map
  std::shared_ptr<RangeDataContainer2d> pub_map_range_data = std::make_shared<RangeDataContainer2d>();
  pub_map_range_data->CreateFrom(range_data, 1 / map_resolution_);
  double map_penalty = MapCheckPenalize(pub_map_range_data, best_pose, true);
  scan_match_score *= map_penalty;
  scan_match_score = (scan_match_score > 1.0) ? (1.0) : (scan_match_score);

  LOG(INFO) << "scan_match_score " << scan_match_score;

#ifdef SLAM_TIME_DEBUG
  TIMER_END_OUTPUT(BackEndScanMatch);
#endif

  return scan_match_score;
}


void SlamProcessor::CorrectPoseAndMap(const std::vector<std::pair<int, Eigen::Vector3d>>& corrected_pose){

  LOG(INFO) << "Start correct pose and map !";

//    std::unique_lock<std::mutex> front_end_lock(front_end_mutex_);

  for(auto pose_with_id : corrected_pose){
    int id = pose_with_id.first;
    Eigen::Vector3d sensor_pose = pose_with_id.second;

    CHECK_LE(id, current_data_index_) << "Invalid correct id !";

    //update range data with corrected pose
    UpdateRangeData(id, sensor_pose);
  }

  {
    std::unique_lock<std::mutex> map_lock(map_mutex_);

    //update all maps with corrected range data

    std::vector<int> pub_map_range_id_vec = sensor_data_manager_->GetRangeIdVec();
    for(int i = 0; i < map_min_passthrough_; ++i){
      pub_map_range_id_vec.push_back(0);
    }
    std::vector<std::shared_ptr<RangeDataContainer2d>> pub_map_range_data_vec =
        sensor_data_manager_->GetMultiresolutionRangeDataVecWithId(kPubMapName, pub_map_range_id_vec);
    map_manager_->GetPubMap(kPubMapName)->InitMapWithRangeVec(pub_map_range_data_vec);

    std::vector<std::shared_ptr<RangeDataContainer2d>> coarse_map_range_data_vec =
        sensor_data_manager_->GetMultiresolutionRangeDataVec(kCoarseScanMatchMapName);
    map_manager_->GetScanMatchMap(kCoarseScanMatchMapName)->InitMapWithRangeVec(coarse_map_range_data_vec,
                                                                                param_->coarse_map_use_blur());

    std::vector<std::shared_ptr<RangeDataContainer2d>> fine_map_range_data_vec =
        sensor_data_manager_->GetMultiresolutionRangeDataVec(kFineScanMatchMapName);
    map_manager_->GetScanMatchMap(kFineScanMatchMapName)->InitMapWithRangeVec(fine_map_range_data_vec,
                                                                              param_->fine_map_use_blur());
  }

  close_loop_ = true;
}


void SlamProcessor::BackEndDataUpdataSignal(int data_index,
                                            const Eigen::Matrix3d& data_cov){

  std::unique_lock<std::mutex> back_end_lock(back_end_mutex_);

  //add new range data to backend process buffer and notify backend thread
  back_end_data_buffer_.push_back(std::make_pair(data_index, data_cov));
  new_vertex_ = true;
  back_end_process_condition_.notify_one();
}

void SlamProcessor::BackEndProcessThread(void){

  int last_process_index = -1;
  std::chrono::microseconds sleep_time = std::chrono::microseconds(1000);

  while(back_end_running_){

    {
      std::unique_lock<std::mutex> back_end_lock(back_end_mutex_);

//        std::unique_lock<std::mutex> back_end_data_lock(back_end_data_mutex_);

      while(!new_vertex_){
        back_end_process_condition_.wait_for(back_end_lock, sleep_time);
      }

#ifdef SLAM_TIME_DEBUG
      TIMER_START(BackEndProcess);
#endif

      //add verties and edge to backend pose graph
      last_process_index = back_end_data_buffer_.back().first;
      while(!back_end_data_buffer_.empty()){
        auto current_back_end_data = back_end_data_buffer_.front();

        range_data_pose_graph_->UpdateGraph(current_back_end_data.first, current_back_end_data.second);
        back_end_data_buffer_.pop_front();
      }
      new_vertex_ = false;

      //check closeloop, optimize(if closeloop succeed)
      range_data_pose_graph_->TryCloseLoop(last_process_index);

#ifdef SLAM_TIME_DEBUG
      TIMER_END_OUTPUT(BackEndProcess);
#endif

      LOG(INFO) << "back_end_process : " << last_process_index;
    }

//      std::this_thread::sleep_for(sleep_time);
  }
}

std::shared_ptr<ScanMatchMap> SlamProcessor::CreateScanMatchMapWithRangeVec(
    const std::vector<std::shared_ptr<RangeDataContainer2d>>& range_data_vec,
    double resolution, double deviation, bool use_blur,
    double gaussian_blur_offset){

  double range_max = this->sensor_data_manager_->GetRangeFinder()->range_max();
  double init_map_size = (range_max + kMinScanMatchMapBound) * 2;
  double map_offset_x = -(current_sensor_pose_.x() - 0.5 * init_map_size);
  double map_offset_y = -(current_sensor_pose_.y() - 0.5 * init_map_size);
  Eigen::Vector2d map_offset(map_offset_x, map_offset_y);
  Eigen::Vector2i map_size(static_cast<int>(init_map_size / resolution),
                           static_cast<int>(init_map_size / resolution));
  std::shared_ptr<ScanMatchMap> map = std::make_shared<ScanMatchMap>(resolution, map_size, map_offset, deviation, kMapUnknownCellProb);
  map->set_cell_occu_prob_offset(gaussian_blur_offset);

  map->InitMapWithRangeVec(range_data_vec, use_blur);

  return map;
}

void SlamProcessor::ResetScanMatchMapWithRangeVec(const std::vector<std::shared_ptr<RangeDataContainer2d>>& range_data_vec,
                                                  std::shared_ptr<ScanMatchMap> map, bool use_blur) {
  double resolution = map->GetCellLength();

  double map_offset_x = -(current_sensor_pose_.x() - 0.5 * map->GetSizeX() * resolution);
  double map_offset_y = -(current_sensor_pose_.y() - 0.5 * map->GetSizeY() * resolution);
  Eigen::Vector2d map_offset(map_offset_x, map_offset_y);
  map->set_map_offset(map_offset);

  map->set_use_auto_map_resize(false);
  map->set_just_update_occu(true);

  map->InitMapWithRangeVec(range_data_vec, use_blur, true);

}

void SlamProcessor::CreateAllMap(void){

  std::unique_lock<std::mutex> map_lock(map_mutex_);

  double range_max = sensor_data_manager_->GetRangeFinder()->range_max();
  double init_map_size = (init_map_size_ < kMinMapSize) ? (kMinMapSize * range_max) : (init_map_size_ * range_max);
  Eigen::Vector2d map_offset(init_map_size * map_offset_x_, init_map_size * map_offset_y_);

  Eigen::Vector2i pub_map_size(static_cast<int>(init_map_size / map_resolution_),
                               static_cast<int>(init_map_size / map_resolution_));
  map_manager_->AddPubMap(kPubMapName, std::make_shared<PubMap>(map_resolution_, pub_map_size, map_offset));
  std::shared_ptr<PubMap> pub_map = map_manager_->GetPubMap(kPubMapName);
  CHECK(pub_map != nullptr) << "Cannot get " << kPubMapName << "!";
  pub_map->set_extend_factor(param_->map_extend_factor());
  pub_map->set_use_auto_map_resize(true);
  pub_map_created_ = true;


  double coarse_map_deviation = param_->coarse_map_deviation();
  double coarse_map_resolution = param_->coarse_map_resolution();
  Eigen::Vector2i coarse_map_size(static_cast<int>(init_map_size / coarse_map_resolution),
                                  static_cast<int>(init_map_size / coarse_map_resolution));
  map_manager_->AddScanMatchMap(kCoarseScanMatchMapName, std::make_shared<ScanMatchMap>(coarse_map_resolution,
                                                                                        coarse_map_size,
                                                                                        map_offset,
                                                                                        coarse_map_deviation, kMapUnknownCellProb));
  std::shared_ptr<ScanMatchMap> coarse_scanmatch_map = map_manager_->GetScanMatchMap(kCoarseScanMatchMapName);
  CHECK(coarse_scanmatch_map != nullptr) << "Cannot get " << kCoarseScanMatchMapName << "!";
  coarse_scanmatch_map->set_extend_factor(param_->map_extend_factor());
  coarse_scanmatch_map->set_cell_occu_prob_offset(param_->gaussian_blur_offset());
  coarse_scanmatch_map->set_use_auto_map_resize(true);
  coarse_scanmatch_map->set_just_update_occu(true);

  double fine_map_deviation = param_->fine_map_deviation();
  double fine_map_resolution = param_->fine_map_resolution();
  Eigen::Vector2i fine_map_size(static_cast<int>(init_map_size / fine_map_resolution),
                                static_cast<int>(init_map_size / fine_map_resolution));
  map_manager_->AddScanMatchMap(kFineScanMatchMapName, std::make_shared<ScanMatchMap>(fine_map_resolution,
                                                                                      fine_map_size,
                                                                                      map_offset,
                                                                                      fine_map_deviation, kMapUnknownCellProb));
  std::shared_ptr<ScanMatchMap> fine_scanmatch_map = map_manager_->GetScanMatchMap(kFineScanMatchMapName);
  CHECK(fine_scanmatch_map != nullptr) << "Cannot get " << kFineScanMatchMapName << "!";
  fine_scanmatch_map->set_extend_factor(param_->map_extend_factor());
  fine_scanmatch_map->set_cell_occu_prob_offset(param_->gaussian_blur_offset());
  fine_scanmatch_map->set_use_auto_map_resize(true);
  fine_scanmatch_map->set_just_update_occu(true);

  map_manager_->AddScanMatchMap(kBackEndCoarseScanMatchMapName,
                                CreateScanMatchMapWithRangeVec(sensor_data_manager_->GetMultiresolutionRangeDataVec(kCoarseScanMatchMapName),
                                                               param_->coarse_map_resolution(),
                                                               param_->coarse_map_deviation(),
                                                               param_->coarse_map_use_blur(),
                                                               param_->gaussian_blur_offset()));

  map_manager_->AddScanMatchMap(kBackEndFineScanMatchMapName,
                                CreateScanMatchMapWithRangeVec(sensor_data_manager_->GetMultiresolutionRangeDataVec(kFineScanMatchMapName),
                                                               param_->fine_map_resolution(),
                                                               param_->fine_map_deviation(),
                                                               param_->fine_map_use_blur(),
                                                               param_->gaussian_blur_offset()));

}


bool SlamProcessor::UpdateMap(std::shared_ptr<RangeDataContainer2d> pub_map_range_data,
                              std::shared_ptr<RangeDataContainer2d> coarse_map_range_data,
                              std::shared_ptr<RangeDataContainer2d> fine_map_range_data){

  std::unique_lock<std::mutex> map_lock(map_mutex_);

  if((scan_match_score_ > map_update_score_threshold_ && //scan_match_score_ <= 1.0 &&
      (util::PoseChangeEnough(current_sensor_pose_, last_map_update_pose_, map_update_distance_threshold_, map_update_angle_threshold_) ||
          !use_map_update_move_check_) ) ||
      current_data_index_ < 1){

    if(is_first_range_data_){
      //assume the first frame is accurate
      map_manager_->GetPubMap(kPubMapName)->SetOccuThreshold(0.5);
      map_manager_->GetPubMap(kPubMapName)->SetMinPassThrough(1);
      map_manager_->GetPubMap(kPubMapName)->SetUpdateFreeFactor(map_min_passthrough_);
      map_manager_->GetPubMap(kPubMapName)->SetUpdateOccupiedFactor(map_min_passthrough_ * 2);
    }
    else{
      map_manager_->GetPubMap(kPubMapName)->SetOccuThreshold(map_occu_threshold_);
      map_manager_->GetPubMap(kPubMapName)->SetMinPassThrough(map_min_passthrough_);
      map_manager_->GetPubMap(kPubMapName)->SetUpdateFreeFactor(map_update_free_factor_);
      map_manager_->GetPubMap(kPubMapName)->SetUpdateOccupiedFactor(map_update_occu_factor_);
    }

    map_manager_->GetPubMap(kPubMapName)->UpdateMapByRange(pub_map_range_data);

    map_manager_->GetScanMatchMap(kCoarseScanMatchMapName)->UpdateMapByRange(coarse_map_range_data, param_->coarse_map_use_blur());

    map_manager_->GetScanMatchMap(kFineScanMatchMapName)->UpdateMapByRange(fine_map_range_data, param_->fine_map_use_blur());

//      LOG(INFO) << "is_transformed " << fine_map_range_data->is_transformed();

    last_map_update_pose_ = current_sensor_pose_;

//      LOG(INFO) << "update map !";

    return true;

  }

  return false;
}

double SlamProcessor::MapCheckPenalize(std::shared_ptr<RangeDataContainer2d> map_range_data ,
                                       const Eigen::Vector3d& candidate_pose,
                                       bool use_logistic){
  std::unique_lock<std::mutex> map_lock(map_mutex_);

  double penalty = 1.0;

  if(param_->use_map_check_feedback()){
    penalty = map_manager_->GetPubMap(kPubMapName)->
        MapFeedbackResponsePenalty(map_range_data,
                                   candidate_pose,
                                   param_->map_check_point_num(),
                                   param_->map_check_bound_tolerance(),
                                   param_->map_check_penalty_gain(),
                                   false);

    if(use_logistic){
      penalty = (1 / (1 + exp(-10 * (penalty - 0.4))));
    }
  }

  return penalty;
}

void SlamProcessor::UpdateRangeData(int id, const Eigen::Vector3d& sensor_pose){
  sensor_data_manager_->GetRangeData(id)->set_sensor_pose(sensor_pose);
  sensor_data_manager_->GetMultiresolutionRangeData(kPubMapName, id)->set_sensor_pose(sensor_pose);
  sensor_data_manager_->GetMultiresolutionRangeData(kCoarseScanMatchMapName, id)->set_sensor_pose(sensor_pose);
  sensor_data_manager_->GetMultiresolutionRangeData(kFineScanMatchMapName, id)->set_sensor_pose(sensor_pose);
}

bool SlamProcessor::MoveEnough(Eigen::Vector3d cur_odom_pose,
                               Eigen::Vector3d last_odom_pose){

  auto duration = std::chrono::steady_clock::now() - last_process_time_;
  if(std::chrono::duration_cast<std::chrono::seconds>(duration).count() > move_time_threshold_){
    last_process_time_ = std::chrono::steady_clock::now();
    return true;
  }

  return util::PoseChangeEnough(cur_odom_pose, last_odom_pose,
                                move_distance_threshold_, move_angle_threshold_);

}

Eigen::Vector3d SlamProcessor::PredictPoseByOdom(const Eigen::Vector3d& last_pose,
                                                 const Eigen::Vector3d& last_odom,
                                                 const Eigen::Vector3d& cur_odom){
  Eigen::Vector3d predict_pose(last_pose);
  Eigen::Vector3d odom_to_map;
  Eigen::Matrix2d odom_to_map_r;

  odom_to_map[2] = last_pose[2] - last_odom[2];
  odom_to_map_r << cos(odom_to_map[2]), -sin(odom_to_map[2]),
      sin(odom_to_map[2]), cos(odom_to_map[2]);
  odom_to_map.head<2>() = last_pose.head<2>() - odom_to_map_r * last_odom.head<2>();
  predict_pose.head<2>() = odom_to_map_r * cur_odom.head<2>() + odom_to_map.head<2>();

  predict_pose[2] = odom_to_map[2] + cur_odom[2];

  return predict_pose;
}


void SlamProcessor::set_map_resolution(double map_resolution){
  if(map_resolution > 0){
    map_resolution_ = map_resolution;
  }else{
    LOG(WARNING) << "Invalid map resolution!";
  }
}

double SlamProcessor::map_resolution(void) const {
  return map_resolution_;
}

void SlamProcessor::set_current_sensor_pose(const Eigen::Vector3d& current_sensor_pose){
  current_sensor_pose_ = current_sensor_pose;
}

Eigen::Vector3d SlamProcessor::current_sensor_pose(void) const {
  return current_sensor_pose_;
}

std::shared_ptr<PubMap> SlamProcessor::GetPubMap(void){
  if(pub_map_created_){
    return map_manager_->GetPubMap(kPubMapName);
  }else{
    LOG(WARNING) << "Pub map has not been created!";
    return nullptr;
  }
}

std::mutex& SlamProcessor::GetMapMutex(void){
  return map_mutex_;
}

void SlamProcessor::GetGraphInfo(std::vector<Eigen::Vector2d> &nodes,
                                 std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > & edges){
  std::unique_lock<std::mutex> back_end_lock(back_end_mutex_);
  range_data_pose_graph_->GetGraphFromSolver(nodes, edges);
}

void SlamProcessor::ForceGraphOptimize(void){
  std::unique_lock<std::mutex> back_end_lock(back_end_mutex_);
  range_data_pose_graph_->ForceComputeByCeres();
}

void SlamProcessor::SlamParamInit(void){

  init_map_size_ = param_->init_map_size();
  map_offset_x_ = param_->map_offset_x();
  map_offset_y_ = param_->map_offset_y();

  use_odometry_ = param_->use_odometry();

  if(!use_odometry_){
    use_move_check_ = false;
  }else{
    use_move_check_ = param_->use_move_check();
    move_distance_threshold_ = param_->move_distance_threshold();
    move_angle_threshold_ = param_->move_angle_threshold();
    move_time_threshold_ = param_->move_time_threshold();
  }

  use_map_update_move_check_ = param_->use_map_update_move_check();
  map_update_score_threshold_ = param_->map_update_score_threshold();
  map_update_distance_threshold_ = param_->map_update_distance_threshold();
  map_update_angle_threshold_ = param_->map_update_angle_threshold();

  map_update_free_factor_ = param_->map_update_free_factor();
  map_update_occu_factor_ = param_->map_update_occu_factor();
  map_occu_threshold_ = param_->map_occu_threshold();
  map_min_passthrough_ = param_->map_min_passthrough();

  front_end_scan_match_param_ = std::make_shared<ScanMatchParam>();

  front_end_scan_match_param_->set_use_optimize_scan_match(param_->use_optimize_scan_match());
  front_end_scan_match_param_->set_iterate_times(param_->iterate_times());
  front_end_scan_match_param_->set_cost_decrease_threshold(param_->cost_decrease_threshold());
  front_end_scan_match_param_->set_cost_min_threshold(param_->cost_min_threshold());
  front_end_scan_match_param_->set_max_update_distance(param_->max_update_distance());
  front_end_scan_match_param_->set_max_update_angle(param_->max_update_angle());
  front_end_scan_match_param_->set_optimize_failed_cost(param_->optimize_failed_cost());

  front_end_scan_match_param_->set_coarse_search_space_size(param_->coarse_search_space_size());
  front_end_scan_match_param_->set_coarse_search_space_resolution(param_->coarse_search_space_resolution());
  front_end_scan_match_param_->set_coarse_search_angle_offset(param_->coarse_search_angle_offset());
  front_end_scan_match_param_->set_coarse_search_angle_resolution(param_->coarse_search_angle_resolution());
  front_end_scan_match_param_->set_coarse_response_threshold(param_->coarse_response_threshold());
  front_end_scan_match_param_->set_coarse_use_point_size(param_->coarse_use_point_size());

  front_end_scan_match_param_->set_fine_search_space_size(param_->fine_search_space_size());
  front_end_scan_match_param_->set_fine_search_space_resolution(param_->fine_search_space_resolution());
  front_end_scan_match_param_->set_fine_search_angle_offset(param_->fine_search_angle_offset());
  front_end_scan_match_param_->set_fine_search_angle_resolution(param_->fine_search_angle_resolution());
  front_end_scan_match_param_->set_fine_response_threshold(param_->fine_response_threshold());
  front_end_scan_match_param_->set_fine_use_point_size(param_->fine_use_point_size());

  front_end_scan_match_param_->set_super_fine_search_space_size(param_->super_fine_search_space_size());
  front_end_scan_match_param_->set_super_fine_search_space_resolution(param_->super_fine_search_space_resolution());
  front_end_scan_match_param_->set_super_fine_search_angle_offset(param_->super_fine_search_angle_offset());
  front_end_scan_match_param_->set_super_fine_search_angle_resolution(param_->super_fine_search_angle_resolution());
  front_end_scan_match_param_->set_super_fine_response_threshold(param_->super_fine_response_threshold());
  front_end_scan_match_param_->set_super_fine_use_point_size(param_->super_fine_use_point_size());

  if(!use_odometry_){
    front_end_scan_match_param_->set_use_center_penalty(false);
  }


  back_end_scan_match_param_ = std::make_shared<ScanMatchParam>();
}





}
