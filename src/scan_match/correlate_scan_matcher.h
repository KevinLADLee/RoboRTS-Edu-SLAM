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


#ifndef ROBORTS_SLAM_SCAN_MATCH_CORRELATE_SCAN_MATCHER_H
#define ROBORTS_SLAM_SCAN_MATCH_CORRELATE_SCAN_MATCHER_H

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "map/map_manager.h"
#include "slam/sensor_data_manager.h"
#include "util/slam_util.h"

namespace roborts_slam {

enum CorrelationScanMatchType {
  COARSE_CORRELATION_SCAN_MATCH = 0,
  FINE_CORRELATION_SCAN_MATCH,
  SUPER_CORRELATION_SCAN_MATCH,
  FAST_CORRELATION_SCAN_MATCH,
};

class CorrelationScanMatchParam{
 public:
  double search_space_size() const { return search_space_size_; }
  void set_search_space_size(double value) { search_space_size_ = value; }

  double search_space_resolution() const { return search_space_resolution_; }
  void set_search_space_resolution(double value) { search_space_resolution_ = value; }

  double search_angle_offset() const { return search_angle_offset_; }
  void set_search_angle_offset(double value) { search_angle_offset_ = value; }

  double search_angle_resolution() const { return search_angle_resolution_; }
  void set_search_angle_resolution(double value) { search_angle_resolution_ = value; }

  double response_threshold() const { return response_threshold_; }
  void set_response_threshold(double value) { response_threshold_ = value; }

  int use_point_size() const { return use_point_size_; }
  void set_use_point_size(int value) { use_point_size_ = value; }

  int max_depth() const { return max_depth_; }
  void set_max_depth(int value) { max_depth_ = value; }

  bool use_center_penalty() const { return use_center_penalty_; }
  void set_use_center_penalty(bool value) { use_center_penalty_ = value; }

  CorrelationScanMatchType correlation_scan_match_type() const { return correlation_scan_match_type_; }
  void set_correlation_scan_match_type(CorrelationScanMatchType value) { correlation_scan_match_type_ = value; }

 private:
  double search_space_size_;
  double search_space_resolution_;

  double search_angle_offset_;
  double search_angle_resolution_;

  double response_threshold_;

  int use_point_size_;

  int max_depth_;

  bool use_center_penalty_;

  CorrelationScanMatchType correlation_scan_match_type_;
};


class LookUpArray{
 public:
  LookUpArray():
      capacity_(0),
      size_(0),
      array_(nullptr){}

  ~LookUpArray(){
    delete[] array_;
    array_ = nullptr;
  }

  void SetSize(int size){
    if (size > capacity_)
    {
      if (array_ != nullptr)
      {
        delete [] array_;
      }
      capacity_ = size;
      array_ = new std::pair<double, double>[capacity_];
    }

    size_ = size;
  }

  int GetSize() const {
    return size_;
  }

  inline std::pair<double, double>& operator [] (int index) {
    assert(index < size_);
    return array_[index];
  }

  inline std::pair<double, double>* GetArrayPointer() {
    return array_;
  }

 private:
  std::pair<double, double>* array_;
  int size_;
  int capacity_;
};

class AngleSearchLookUpTable{
 public:
  AngleSearchLookUpTable(){}

  ~AngleSearchLookUpTable(){
    DeleteArray();
  }

  /**
   * @brief store range point position on grid map to lookup table
   * @param map : map for match
   * @param range_data : range data for match
   * @param base_pose : initial center pose for search
   * @param angle_offset : angle offset
   * @param angle_resolution : angle resolution
   */
  void UpdateLookUpTable(std::shared_ptr<ScanMatchMap> map,
                         std::shared_ptr<RangeDataContainer2d> range_data,
                         const Eigen::Vector3d& base_pose, double angle_offset, double angle_resolution){

    int n_angles = static_cast<int>(std::floor(angle_offset * 2 / angle_resolution) + 1);
    Resize(n_angles);

    int map_size_x = map->GetSizeX();

    //search angle interval : [center_angle - angle_offset, center_angle + angle_offset]
    double base_angle = base_pose[2];
    double start_angle = base_angle - angle_offset;
    for(int angle_index = 0; angle_index < n_angles; ++angle_index){
      //iterate for every angle
      double angle = start_angle + angle_index * angle_resolution;

      lookup_array_[angle_index]->SetSize(range_data->GetSize());
      angles_.at(angle_index) = angle;

      std::pair<double, double>* array_pointer = lookup_array_[angle_index]->GetArrayPointer();

      double cosine = cos(angle);
      double sine = sin(angle);
      Eigen::Vector2d point_map;
      Eigen::Vector2d point_local;
      for(int point_index = 0; point_index < range_data->GetSize(); ++point_index){

        //calculate point position in map coordinate system
        point_local = range_data->GetDataPoint(point_index);
        point_map[0] = cosine * point_local.x() - sine * point_local.y();// + base_pose[0];
        point_map[1] = sine * point_local.x() + cosine * point_local.y();// + base_pose[1];

        //store the position for lookup later
        array_pointer[point_index] = std::pair<double, double>(point_map[0],point_map[1]);
      }
    }
  }

  int GetAnglesNum() const {
    return angles_.size();
  }

  double GetAngle(int index) const {
    return angles_[index];
  }

  LookUpArray* GetLookUpArray(int index) const {
    return lookup_array_[index];
  }

 private:

  void Resize(double n_angles){
    angles_.resize(n_angles);

    if(lookup_array_.size() != n_angles){
      DeleteArray();
      lookup_array_.resize(n_angles);
      for(int i = 0; i < n_angles; ++i){
        lookup_array_[i] = new LookUpArray();
      }
    }
  }

  void DeleteArray(){
    for(auto array : lookup_array_){
      delete array;
    }
    lookup_array_.clear();
  }

  std::vector<double> angles_;
  std::vector<LookUpArray *> lookup_array_;
};



class Candidate2D {
 public:

  Candidate2D() = default;

  Candidate2D(const double x,
              const double y,
              const double angle,
              const int angle_index)
      : x_(x),
        y_(y),
        angle_(angle),
        angle_index_(angle_index){}

  bool operator<(const Candidate2D& other) const { return this->score() < other.score(); }
  bool operator>(const Candidate2D& other) const { return this->score() > other.score(); }

  double x(void) const{ return x_; }
  void set_x(double value){ x_ = value; }

  double y(void) const{ return y_; }
  void set_y(double value){ y_ = value; }

  double angle(void) const { return angle_; }
  void set_angle(double value){ angle_ = value; }

  double angle_index(void) const { return angle_index_; }
  void set_angle_index(double value){ angle_index_ = value; }

  double score(void) const { return score_; }
  void set_score(double value){ score_ = value; }


 private:

  double x_ = 0.;
  double y_ = 0.;
  double angle_ = 0.;
  int angle_index_ = 0;

  double score_ = 0.0;
};


class BranchAndBoundCorrelateScanMatcher{
 public:

  double ScanMatch(std::shared_ptr<ScanMatchMap> map,
                   std::shared_ptr<RangeDataContainer2d> range_data,
                   std::shared_ptr<CorrelationScanMatchParam> scan_match_param,
                   std::shared_ptr<AngleSearchLookUpTable> angle_lookup_table,
                   Eigen::Vector3d& current_pose,
                   Eigen::Matrix3d& cov_matrix){

    map_ = map;
    map_resolution_ = map->GetCellLength();

    use_point_size_ = scan_match_param->use_point_size();

    search_spcae_resolution_ = scan_match_param->search_space_resolution();
    search_space_size_ = scan_match_param->search_space_size();

    search_angle_resolution_ = scan_match_param->search_angle_resolution();
    serach_angle_size_ = scan_match_param->search_angle_offset() * 2;

    Eigen::Vector3d init_pose_world = current_pose;
    Eigen::Vector3d center_pose = map->GetMapCoordsPose(init_pose_world);
//    LOG(INFO) << "center_pose : " <<  center_pose.transpose();

    double min_score = scan_match_param->response_threshold();

    TIMER_START(UpdateLookUpTable);
    angle_lookup_table_ = angle_lookup_table;
    angle_lookup_table_->UpdateLookUpTable(map, range_data, center_pose, serach_angle_size_ / 2, search_angle_resolution_);
    TIMER_END_OUTPUT(UpdateLookUpTable);

    const double highest_resolution = search_spcae_resolution_;
    const int max_depth = scan_match_param->max_depth();
    const double lowest_resolution = (1 << max_depth) * highest_resolution;
    TIMER_START(ComputeLowestResolutionCandidates);
    const std::vector<Candidate2D> lowest_resolution_candidates =
        ComputeLowestResolutionCandidates(center_pose, scan_match_param, lowest_resolution);
    TIMER_END_OUTPUT(ComputeLowestResolutionCandidates);
    candidate_pose_ = lowest_resolution_candidates;

    double lowest_resolution_best_score = lowest_resolution_candidates.front().score();
//    const Candidate2D best_candidate = lowest_resolution_candidates.front();

    TIMER_START(BranchAndBound);
    best_candidate_pose_ = BranchAndBound(lowest_resolution_candidates,
                                          max_depth, lowest_resolution_best_score - 0.1);
    TIMER_END_OUTPUT(BranchAndBound);

    double best_response = lowest_resolution_candidates.front().score();
    Eigen::Vector3d best_pose(center_pose);
    if (best_candidate_pose_.score() > min_score) {
      best_response = best_candidate_pose_.score();
      best_pose = Eigen::Vector3d(best_candidate_pose_.x(), best_candidate_pose_.y(), best_candidate_pose_.angle());
    }

//    LOG(INFO) << "best response : " <<  best_response;

    return best_response;

  }

  std::vector<Candidate2D> ComputeLowestResolutionCandidates(
      const Eigen::Vector3d& center_pose,
      std::shared_ptr<CorrelationScanMatchParam> scan_match_param,
      double lowest_resolution){

    std::vector<Candidate2D> lowest_resolution_candidates;
    int search_spcae_num = static_cast<int>(util::Round(search_space_size_ / lowest_resolution) + 1);
    int search_angle_num = angle_lookup_table_->GetAnglesNum();
    int num_candidates = search_angle_num * search_spcae_num * search_spcae_num;

    int point_step_size = 1;
    int use_point_size = use_point_size_;
    double search_space_start_x = center_pose[0] - (search_space_size_ / map_resolution_) * 0.5;
    double search_space_start_y = center_pose[1] - (search_space_size_ / map_resolution_) * 0.5;
    double space_step_factor = lowest_resolution / map_resolution_;

    lowest_resolution_candidates.reserve(num_candidates);
    for(int angle_index = 0; angle_index < search_angle_num; angle_index++){
      double angle = angle_lookup_table_->GetAngle(angle_index);

      LookUpArray* lookup_array = angle_lookup_table_->GetLookUpArray(angle_index);
      std::pair<double, double>* array_pointer = lookup_array->GetArrayPointer();
      int point_size = lookup_array->GetSize();
      if(point_size <= 0)continue;
      if(point_size < 2 * use_point_size){
        use_point_size = point_size;
        point_step_size = 1;
      }else{
        point_step_size = point_size / (use_point_size - 1);
      }

      for(int x_index = 0; x_index < search_spcae_num; ++x_index){
        double x = search_space_start_x + x_index * space_step_factor;

        for(int y_index = 0; y_index < search_spcae_num; ++y_index){
          double y = search_space_start_y + y_index * space_step_factor;

          double score = 0.0;
          for(int point_index = 0; point_index < point_size; point_index += point_step_size){

            int map_grid_x = static_cast<int>(array_pointer[point_index].first + x + 0.5);
            int map_grid_y = static_cast<int>(array_pointer[point_index].second + y + 0.5);

            score += map_->GetGridProbValue(map_grid_x, map_grid_y);
          }

          score /= use_point_size;

          lowest_resolution_candidates.emplace_back(x, y, angle, angle_index);
          lowest_resolution_candidates.back().set_score(score);
        }
      }
    }

    std::sort(lowest_resolution_candidates.begin(), lowest_resolution_candidates.end(),
              std::greater<Candidate2D>());

//    ScoreCandidates(lowest_resolution_candidates);

    return lowest_resolution_candidates;
  }




  void ScoreCandidates(std::vector<Candidate2D>& candidates ){

    int point_step_size = 1;
    int use_point_size = use_point_size_;

    for (Candidate2D& candidate : candidates) {

      LookUpArray* lookup_array = angle_lookup_table_->GetLookUpArray(candidate.angle_index());
      std::pair<double, double>* array_pointer = lookup_array->GetArrayPointer();
      int point_size = lookup_array->GetSize();
      if(point_size <= 0)continue;
      if(point_size < 2 * use_point_size){
        use_point_size = point_size;
        point_step_size = 1;
      }else{
        point_step_size = point_size / (use_point_size - 1);
      }

      double score = 0.0;
      for(int point_index = 0; point_index < point_size; point_index += point_step_size){

        int map_grid_x = static_cast<int>(array_pointer[point_index].first + candidate.x() + 0.5);
        int map_grid_y = static_cast<int>(array_pointer[point_index].second + candidate.y() + 0.5);

        score += map_->GetGridProbValue(map_grid_x, map_grid_y);
      }

      score /= use_point_size;
      candidate.set_score(score);
    }

    std::sort(candidates.begin(), candidates.end(),
              std::greater<Candidate2D>());
  }


  Candidate2D BranchAndBound(const std::vector<Candidate2D>& candidates,
                             const int candidate_depth,
                             double min_score) {

//    LOG(INFO) << "candidate_depth " << candidate_depth;

    if (candidate_depth == 0) {
      // Return the best candidate.
      return *candidates.begin();
    }

    Candidate2D best_high_resolution_candidate(0, 0.0, 0.0, 0.0);
    best_high_resolution_candidate.set_score(min_score);
    for (const Candidate2D& candidate : candidates) {
      if (candidate.score() <= min_score) {
        break;
      }

      std::vector<Candidate2D> higher_resolution_candidates;
      const double half_depth_resolution = (1 << (candidate_depth - 1)) * search_spcae_resolution_;
      const double half_width = half_depth_resolution / map_resolution_;
      for (double x_offset : {0.0, half_width}) {

        for (double y_offset : {0.0, half_width}) {

          higher_resolution_candidates.emplace_back(
              candidate.x() + x_offset, candidate.y() + y_offset,
              angle_lookup_table_->GetAngle(candidate.angle_index()),
              candidate.angle_index());
        }
      }

      ScoreCandidates(higher_resolution_candidates);

      best_high_resolution_candidate = std::max(
          best_high_resolution_candidate,
          BranchAndBound(higher_resolution_candidates,
                         candidate_depth - 1,
                         best_high_resolution_candidate.score()));
    }

    return best_high_resolution_candidate;
  }

  Candidate2D best_candidate_pose(void) const {
    return best_candidate_pose_;
  }

  std::vector<Candidate2D> candidate_pose(void) const {
    return candidate_pose_;
  }

 private:
  std::shared_ptr<AngleSearchLookUpTable> angle_lookup_table_;

  std::vector<Candidate2D> candidate_pose_;
  Candidate2D best_candidate_pose_;

  double search_spcae_resolution_;
  double search_space_size_;

  double search_angle_resolution_;
  double serach_angle_size_;

  std::shared_ptr<ScanMatchMap> map_;
  double map_resolution_;

  int use_point_size_;
};


class MultiResolutionCorrelateScanMatcher{
 public:

  /**
   * @brief scan match interface of multi resolution search method
   * @param map : map for match
   * @param range_data : scan data for match
   * @param scan_match_param : param of algorithm
   * @param init_pose : init pose for scan match(important)
   * @param angle_lookup_table : store the map grids of every angles (accelerate calculate efficiency)
   */
  double ScanMatch(std::shared_ptr<ScanMatchMap> map,
                   std::shared_ptr<RangeDataContainer2d> range_data,
                   std::shared_ptr<CorrelationScanMatchParam> scan_match_param,
                   std::shared_ptr<AngleSearchLookUpTable> angle_lookup_table,
                   Eigen::Vector3d& init_pose){

    search_spcae_resolution_ = scan_match_param->search_space_resolution();
    search_angle_resolution_ = scan_match_param->search_angle_resolution();

    search_space_size_ = scan_match_param->search_space_size();
    serach_angle_size_ = scan_match_param->search_angle_offset() * 2;

    candidate_pose_.clear();

//    LOG(INFO) << "center_pose " << center_pose.transpose();

    double map_resolution = map->GetCellLength();

    //update angle lookup table, store  all of the range point position on the grid map of every search pose
    angle_lookup_table_ = angle_lookup_table;
    angle_lookup_table_->UpdateLookUpTable(map, range_data, init_pose, serach_angle_size_ / 2, search_angle_resolution_);

    int search_spcae_num = static_cast<int>(util::Round(search_space_size_ / search_spcae_resolution_) + 1);
    int search_angle_num = angle_lookup_table_->GetAnglesNum();

//    LOG(INFO) << "search_spcae_num  " << search_spcae_num << "    search_angle_num " << search_angle_num;

    double response;
    int use_point_size = scan_match_param->use_point_size();
    int point_step_num = 1;
    double search_space_start_x = init_pose[0] - (search_space_size_ / map_resolution) * 0.5;
    double search_space_start_y = init_pose[1] - (search_space_size_ / map_resolution) * 0.5;
    double space_step_factor = search_spcae_resolution_ / map_resolution;

    //visit every x,y,angle of search range
    Candidate2D cur_candidate_pose;
    for(int angle_index = 0; angle_index < search_angle_num; ++angle_index){
      cur_candidate_pose.set_angle_index(angle_index);
      cur_candidate_pose.set_angle(angle_lookup_table_->GetAngle(angle_index));

      //get map grid index of all of the range point in current pose from angle lookup table
      LookUpArray* lookup_array = angle_lookup_table_->GetLookUpArray(angle_index);
      std::pair<double, double>* array_pointer = lookup_array->GetArrayPointer();
      //use a part of points to calculate response for reduce the amount of calculation
      int point_size = lookup_array->GetSize();
      if(point_size < 2 * use_point_size){
        use_point_size = point_size;
        point_step_num = 1;
      }else{
        point_step_num = point_size / (use_point_size - 1);
      }

      for(int x_index = 0; x_index < search_spcae_num; ++x_index){
        cur_candidate_pose.set_x(search_space_start_x + x_index * space_step_factor);

        for(int y_index = 0; y_index < search_spcae_num; ++y_index){
          cur_candidate_pose.set_y(search_space_start_y + y_index * space_step_factor);

          //calculate the response of current search pose
          response = GetResponse(map, cur_candidate_pose, array_pointer,
                                 point_size, point_step_num, use_point_size);

          //record response
          cur_candidate_pose.set_score(response);
          candidate_pose_.emplace_back(cur_candidate_pose);

        }
      }
    }

    //penalize the response of every candidate pose
    if(scan_match_param->use_center_penalty()){
      if(scan_match_param->correlation_scan_match_type() == COARSE_CORRELATION_SCAN_MATCH){
        PenalizeResponse(init_pose,
                         candidate_pose_,
                         map_resolution,
                         search_space_size_,
                         kDistancePenaltyGainCoarse,
                         kAngularPenaltyGain);
      }else{
        PenalizeResponse(init_pose,
                         candidate_pose_,
                         map_resolution,
                         search_space_size_,
                         kDistancePenaltyGainFine,
                         kAngularPenaltyGain);
      }
    }


    //sort candidate pose according to response score
    std::sort(candidate_pose_.begin(), candidate_pose_.end(),
              std::greater<Candidate2D>());

    //find the highest response candidate
    best_candidate_pose_ = FindBestCandidate(candidate_pose_, kResponseFilterTolerance);

    return best_candidate_pose_.score();
  }


  Candidate2D best_candidate_pose(void) const {
    return best_candidate_pose_;
  }

  std::vector<Candidate2D> candidate_pose(void) const {
    return candidate_pose_;
  }


 private:
  /**
   * @brief statistic the response of range points on the grid map
   * @param map : map
   * @param cur_candidate_pose : current candidate pose
   * @param array_pointer : point positin on grid map stored in the lookup table
   * @param point_size : the current frame range point number
   * @param point_step_num : point sample intrval
   * @param use_point_size : point_size / point_step_num
   * @return the response
   */
  inline double GetResponse(std::shared_ptr<ScanMatchMap> map,
                            const Candidate2D& cur_candidate_pose,
                            std::pair<double, double>* array_pointer,
                            int point_size, int point_step_num, int use_point_size) const {
    double response = 0.0;
    int invalid_point_size = 0;
    int map_grid_x,map_grid_y;

    for(int point_index = 0; point_index < point_size; point_index += point_step_num){

      map_grid_x = static_cast<int>(array_pointer[point_index].first + cur_candidate_pose.x() + 0.5);
      map_grid_y = static_cast<int>(array_pointer[point_index].second + cur_candidate_pose.y() + 0.5);

      //get the probability of current map grid position from map
      double tmp_res = map->GetGridProbValue(map_grid_x, map_grid_y);
      response += tmp_res;

    }

//    LOG(INFO) << "current pose response sum " << response;
//    LOG(INFO) << "point_size " << point_size << "  use_point_size " << use_point_size;
    //calculate the average response
    response /= (use_point_size - invalid_point_size);

    return response;
  }

  /**
   * @brief find the best candidate pose
   * @param sorted_candidates : sorted candidate pose vecter according to response score
   * @param score_tolerance : the score thershold for calculate the best candidate
   * @return the best candidate pose
   */
  Candidate2D FindBestCandidate(std::vector<Candidate2D>& sorted_candidates,
                               double score_tolerance){

    Candidate2D best_candidate_pose = sorted_candidates.front();

    Eigen::Vector2d average_position = Eigen::Vector2d::Zero();
    double theta_x = 0.0;
    double theta_y = 0.0;
    double best_score_sum = 0.0;
    int best_score_counts = 0;

    //statistic the candidate pose which is similar to the best candidate pose
    for(auto candidate : sorted_candidates){
      double cur_score = candidate.score();
      Eigen::Vector2d cur_position(candidate.x(), candidate.y());
      if(util::DoubleEqual(cur_score, best_candidate_pose.score(), score_tolerance)){
        average_position += (cur_position * cur_score);

        theta_x += cos(candidate.angle()) * cur_score;
        theta_y += sin(candidate.angle()) * cur_score;

        best_score_sum += cur_score;
        best_score_counts++;
      }else
      {
        break;
      }
    }

    //average
    if(best_score_counts > 1){
      average_position /= best_score_sum;
      theta_x /= best_score_sum;
      theta_y /= best_score_sum;
      best_candidate_pose.set_x(average_position[0]);
      best_candidate_pose.set_y(average_position[1]);
      best_candidate_pose.set_angle(atan2(theta_y, theta_x));
    }

    return best_candidate_pose;
  }

  /**
   * @brief penalize the response of every candidate pose,
   *        farther away from center pose, more serious for penalty
   * @param center_pose : the init pose for search
   * @param candidates : all of the candidate pose
   */
  void PenalizeResponse(const Eigen::Vector3d& center_pose,
                        std::vector<Candidate2D>& candidates,
                        double scale_factor, double max_distance_bound,
                        double distance_penalty_gain, double angle_penalty_gain){

    Eigen::Vector3d cur_pose;
    double cur_pose_distance;
    double cur_pose_angle_diff;

    for(auto & candidate : candidates){
      if(!util::DoubleEqual(candidate.score(), 0.0)){

        Eigen::Vector3d cur_pose(candidate.x(), candidate.y(), candidate.angle());

        cur_pose_distance = util::SquareDistance2d(cur_pose.head<2>(), center_pose.head<2>());
        cur_pose_distance *= (scale_factor * scale_factor);
        double distance_penalty = 1.0 - (distance_penalty_gain * cur_pose_distance / (max_distance_bound / 2));
        distance_penalty = std::max(distance_penalty, 0.5);

        cur_pose_angle_diff = pow((cur_pose[2] - center_pose[2]), 2);
        double angle_penalty = 1.0 - (angle_penalty_gain * cur_pose_angle_diff / 0.349);
        angle_penalty = std::max(angle_penalty, 0.9);

        candidate.set_score(candidate.score() * (distance_penalty * angle_penalty));
      }
    }

  }

 private:
  std::shared_ptr<AngleSearchLookUpTable> angle_lookup_table_;

  std::vector<Candidate2D> candidate_pose_;
  Candidate2D best_candidate_pose_;

  double search_spcae_resolution_;
  double search_angle_resolution_;

  double search_space_size_;
  double serach_angle_size_;

  const double kAngularPenaltyGain = 0.25;
  const double kDistancePenaltyGainCoarse = 0.4;
  const double kDistancePenaltyGainFine = 0.2;

  const double kResponseFilterTolerance = 1e-2;
};

class BasedCorrelationScanMatch{
 public:
  BasedCorrelationScanMatch():
      branch_and_bound_scan_matcher_(std::make_unique<BranchAndBoundCorrelateScanMatcher>()),
      multi_resolution_scan_matcher_(std::make_unique<MultiResolutionCorrelateScanMatcher>()),
      angle_lookup_table_(std::make_shared<AngleSearchLookUpTable>()){

  }


  /**
   * @brief scan match interface
   * @param map : map for match
   * @param range_data : scan data for match
   * @param scan_match_param : param of algorithm
   * @param current_pose : init pose for scan match(important)
   * @param cov_matrix : store the calculate result
   */
  double ScanMatch(std::shared_ptr<ScanMatchMap> map,
                   std::shared_ptr<RangeDataContainer2d> range_data,
                   std::shared_ptr<CorrelationScanMatchParam> scan_match_param,
                   Eigen::Vector3d& current_pose,
                   Eigen::Matrix3d& cov_matrix){

    double response = kMinResponse;

    if(!map->IsMapInit() || range_data->GetSize() == 0){
      LOG(WARNING) << "Invalid scan match input !";
      return response;
    }

    //load search resolution from param
    search_spcae_resolution_ = scan_match_param->search_space_resolution();
    search_angle_resolution_ = scan_match_param->search_angle_resolution();

    max_angular_variance_ = 4 * pow(search_angle_resolution_, 2);

//    search_space_size_ = scan_match_param->search_space_size();
//    serach_angle_size_ = scan_match_param->search_angle_offset() * 2;

    double map_resolution = map->GetCellLength();

    //transform sensor pose in world to pose in map
    Eigen::Vector3d init_pose_world = current_pose;
    Eigen::Vector3d center_pose = map->GetMapCoordsPose(init_pose_world);

    std::vector<Candidate2D> candidate_poses;
    Candidate2D best_candidate_pose;

    if(scan_match_param->correlation_scan_match_type() == FAST_CORRELATION_SCAN_MATCH){
      branch_and_bound_scan_matcher_->ScanMatch(map, range_data, scan_match_param,
                                                angle_lookup_table_,
                                                current_pose, cov_matrix);
      candidate_poses = branch_and_bound_scan_matcher_->candidate_pose();
      best_candidate_pose = branch_and_bound_scan_matcher_->best_candidate_pose();

    }else{  //use this commonly
      //do scanmatch with multi resolution search method
      multi_resolution_scan_matcher_->ScanMatch(map, range_data, scan_match_param,
                                                angle_lookup_table_,
                                                center_pose);

      //get the scan match result
      candidate_poses = multi_resolution_scan_matcher_->candidate_pose();
      best_candidate_pose = multi_resolution_scan_matcher_->best_candidate_pose();
    }


    //statistic the covrance of x,y,angle with candidate poses
    switch(scan_match_param->correlation_scan_match_type()){
      case FAST_CORRELATION_SCAN_MATCH:
      case COARSE_CORRELATION_SCAN_MATCH:
      {
        ComputePositionalCovariance(candidate_poses, best_candidate_pose, map_resolution, cov_matrix);
        ComputeAngularCovariance(candidate_poses, best_candidate_pose, search_spcae_resolution_ / map_resolution, cov_matrix);
        break;
      }

      case FINE_CORRELATION_SCAN_MATCH:
      {
        ComputePositionalCovariance(candidate_poses, best_candidate_pose, map_resolution, cov_matrix);
        break;
      }

      case SUPER_CORRELATION_SCAN_MATCH:
      {
        ComputeAngularCovariance(candidate_poses, best_candidate_pose, search_spcae_resolution_ / map_resolution, cov_matrix);
        break;
      }

      default:
        break;
    }

    //check the response score
    double best_score = best_candidate_pose.score();
    best_score = (best_score > 1.0) ? (1.0) : (best_score);
    response = best_score;

    //if the response is high enough, update the sensor pose
    if(response > scan_match_param->response_threshold()){
      Eigen::Vector3d best_pose(best_candidate_pose.x(), best_candidate_pose.y(), best_candidate_pose.angle());
      current_pose = map->GetWorldCoordsPose(best_pose);
    }

//    LOG(INFO) << "best response : " <<  response;

    return response;

  }


 private:

  /**
   * @brief calculate the covariance of the x,y
   * @param candidates : all of the candidate search pose
   * @param best_candidate_pose : the pose of the highest response
   * @param map_resolution : map resolution
   * @param cov_matrix : store the calculate result
   */
  void ComputePositionalCovariance(const std::vector<Candidate2D>& candidates,
                                   const Candidate2D& best_candidate_pose,
                                   double map_resolution,
                                   Eigen::Matrix3d& cov_matrix){
    cov_matrix.setIdentity();

    double best_score = best_candidate_pose.score();

    if(best_score < kDoubleTolerance){
      cov_matrix(0, 0) = kMaxVariance;
      cov_matrix(1, 1) = kMaxVariance;
      cov_matrix(2, 2) = max_angular_variance_;

      return;
    }

    double accumulated_variance_xx = 0.0;
    double accumulated_variance_xy = 0.0;
    double accumulated_variance_yy = 0.0;
    double norm = 0.0;

    double best_pose_x = best_candidate_pose.x();
    double best_pose_y = best_candidate_pose.y();

    //use the sample in the score interval of [best_score - 0.1, best_score]
    double score_bound = std::min(best_score - 0.1, 0.5);
    int point_counter = 0;

    for(const auto & candidate : candidates){
      double cur_score = candidate.score();
      if(cur_score > score_bound && point_counter < kMaxVarianceUsePointSize){
        norm += cur_score;
        accumulated_variance_xx += (pow((candidate.x() - best_pose_x), 2) * cur_score);
        accumulated_variance_xy += ((candidate.x() - best_pose_x) * (candidate.y() - best_pose_y) * cur_score);
        accumulated_variance_yy += (pow((candidate.y() - best_pose_y), 2) * cur_score);

        point_counter++;

      }else{
        break;
      }
    }

    if(norm > kDoubleTolerance){
      double variance_xx = accumulated_variance_xx / norm;
      double variance_xy = accumulated_variance_xy / norm;
      double variance_yy = accumulated_variance_yy / norm;
      double variance_aa = max_angular_variance_;

      double min_variance = 0.1 * pow((search_spcae_resolution_ / map_resolution), 2);
      variance_xx = std::max<double>(variance_xx, min_variance);
      variance_yy = std::max<double>(variance_yy, min_variance);

      //calculate the covariance of x,y and transform the scale from map to real world
      cov_matrix(0, 0) = (variance_xx * pow(map_resolution, 2)) / best_score;
      cov_matrix(0, 1) = (variance_xy * pow(map_resolution, 2)) / best_score;
      cov_matrix(1, 0) = (variance_xy * pow(map_resolution, 2)) / best_score;
      cov_matrix(1, 1) = (variance_yy * pow(map_resolution, 2)) / best_score;
      cov_matrix(2, 2) = variance_aa;
    }

    if(util::DoubleEqual(cov_matrix(0, 0), 0.0)){
      cov_matrix(0, 0) = kMaxVariance;
    }

    if(util::DoubleEqual(cov_matrix(1, 1), 0.0)){
      cov_matrix(1, 1) = kMaxVariance;
    }

  }

  /**
   * @brief calculate the covariance of the angle
   * @param candidates : all of the candidate search pose
   * @param best_candidate_pose : the pose of the highest response
   * @param linear_tolerance : the tolerance of the x y
   * @param cov_matrix : store the calculate result
   */
  void ComputeAngularCovariance(const std::vector<Candidate2D>& candidates,
                                const Candidate2D& best_candidate_pose,
                                double linear_tolerance,
                                Eigen::Matrix3d& cov_matrix) {

    double best_score = best_candidate_pose.score();

//    LOG(INFO) << "  linear_tolerance " << linear_tolerance;

    if(best_score < kDoubleTolerance){
      cov_matrix(2, 2) = max_angular_variance_;
      return;
    }

    double best_angle = best_candidate_pose.angle();

    double angle = 0.0;
    double norm = 0.0;
    double accumulated_variance_aa = 0.0;

    //use the sample in the score interval of [best_score - 0.1, best_score]
    double score_bound = std::min(best_score - 0.1, 0.5);
    int point_counter = 0;

    //statistic the candidate of having the "same" x,y with best_candidate
    for(const auto & candidate : candidates){
      double cur_score = candidate.score();
      if(cur_score >= score_bound && point_counter < kMaxVarianceUsePointSize){

        if(util::DoubleEqual(candidate.x(), best_candidate_pose.x(), linear_tolerance) &&
           util::DoubleEqual(candidate.y(), best_candidate_pose.y(), linear_tolerance)){
          angle = candidate.angle();
          norm += cur_score;
          accumulated_variance_aa += (pow((angle - best_angle), 2) * cur_score);

          point_counter++;
        }
      }
    }

    //calculate the variance, and assume the angle have nothing to do with the x,y
    double variance_aa = max_angular_variance_;
    if (norm > kDoubleTolerance) {
      if (accumulated_variance_aa < kDoubleTolerance) {
        variance_aa = max_angular_variance_ / 4;
      }

      variance_aa = accumulated_variance_aa / norm;
    }
    else {
      variance_aa = 200 * max_angular_variance_;
    }

    cov_matrix(2, 2) = variance_aa;
  }

 private:

  std::unique_ptr<BranchAndBoundCorrelateScanMatcher> branch_and_bound_scan_matcher_;
  std::unique_ptr<MultiResolutionCorrelateScanMatcher> multi_resolution_scan_matcher_;

  //record the map grid index of every angle resolution
  std::shared_ptr<AngleSearchLookUpTable> angle_lookup_table_;

  double search_spcae_resolution_;
  double search_angle_resolution_;

  double max_angular_variance_ = kMaxVariance;
  const int kMaxVarianceUsePointSize = 20;
  const double kMinResponse = 0.0;

};


} // namespace roborts_slam

#endif //ROBORTS_SLAM_SCAN_MATCH_CORRELATE_SCAN_MATCHER_H
