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

#ifndef ROBORTS_SLAM_SCAN_MATCH_SCAN_MATCHERS_H
#define ROBORTS_SLAM_SCAN_MATCH_SCAN_MATCHERS_H

#include <iostream>
#include <vector>
#include <map>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "map/map_manager.h"
#include "slam/sensor_data_manager.h"
#include "util/slam_util.h"

#include "optimize_scan_matcher.h"
#include "correlate_scan_matcher.h"

namespace roborts_slam{


class ScanMatchParam{
 public:
  
  const bool use_optimize_scan_match(void) const { return use_optimize_scan_match_; }
  void set_use_optimize_scan_match(double value) { use_optimize_scan_match_ = value; }
  
  const int iterate_times(void) const { return iterate_times_; }
  void set_iterate_times(int value) { iterate_times_ = value; }
  
  const double cost_decrease_threshold(void) const { return cost_decrease_threshold_; }
  void set_cost_decrease_threshold(double value) { cost_decrease_threshold_ = value; }
  
  const double cost_min_threshold(void) const { return cost_min_threshold_; }
  void set_cost_min_threshold(double value) { cost_min_threshold_ = value; }
  
  const double max_update_distance(void) const { return max_update_distance_; }
  void set_max_update_distance(double value) { max_update_distance_ = value; }

  const double max_update_angle(void) const { return max_update_angle_; }
  void set_max_update_angle(double value) { max_update_angle_ = value; }

  const double optimize_failed_cost(void) const { return optimize_failed_cost_; }
  void set_optimize_failed_cost(double value) { optimize_failed_cost_ = value; }


  const double coarse_search_space_size(void) const { return coarse_search_space_size_; }
  void set_coarse_search_space_size(double value) { coarse_search_space_size_ = value; }
  
  const double coarse_search_space_resolution(void) const { return coarse_search_space_resolution_; }
  void set_coarse_search_space_resolution(double value) { coarse_search_space_resolution_ = value; }

  const double coarse_search_angle_offset(void) const { return coarse_search_angle_offset_; }
  void set_coarse_search_angle_offset(double value) { coarse_search_angle_offset_ = value; }

  const double coarse_search_angle_resolution(void) const { return coarse_search_angle_resolution_; }
  void set_coarse_search_angle_resolution(double value) { coarse_search_angle_resolution_ = value; }

  const double coarse_response_threshold(void) const { return coarse_response_threshold_; }
  void set_coarse_response_threshold(double value) { coarse_response_threshold_ = value; }

  const int coarse_use_point_size(void) const { return coarse_use_point_size_; }
  void set_coarse_use_point_size(int value) { coarse_use_point_size_ = value; }


  const double fine_search_space_size(void) const { return fine_search_space_size_; }
  void set_fine_search_space_size(double value) { fine_search_space_size_ = value; }

  const double fine_search_space_resolution(void) const { return fine_search_space_resolution_; }
  void set_fine_search_space_resolution(double value) { fine_search_space_resolution_ = value; }

  const double fine_search_angle_offset(void) const { return fine_search_angle_offset_; }
  void set_fine_search_angle_offset(double value) { fine_search_angle_offset_ = value; }

  const double fine_search_angle_resolution(void) const { return fine_search_angle_resolution_; }
  void set_fine_search_angle_resolution(double value) { fine_search_angle_resolution_ = value; }

  const double fine_response_threshold(void) const { return fine_response_threshold_; }
  void set_fine_response_threshold(double value) { fine_response_threshold_ = value; }

  const int fine_use_point_size(void) const { return fine_use_point_size_; }
  void set_fine_use_point_size(int value) { fine_use_point_size_ = value; }
  

  const double super_fine_search_space_size(void) const { return super_fine_search_space_size_; }
  void set_super_fine_search_space_size(double value) { super_fine_search_space_size_ = value; }

  const double super_fine_search_space_resolution(void) const { return super_fine_search_space_resolution_; }
  void set_super_fine_search_space_resolution(double value) { super_fine_search_space_resolution_ = value; }

  const double super_fine_search_angle_offset(void) const { return super_fine_search_angle_offset_; }
  void set_super_fine_search_angle_offset(double value) { super_fine_search_angle_offset_ = value; }

  const double super_fine_search_angle_resolution(void) const { return super_fine_search_angle_resolution_; }
  void set_super_fine_search_angle_resolution(double value) { super_fine_search_angle_resolution_ = value; }

  const double super_fine_response_threshold(void) const { return super_fine_response_threshold_; }
  void set_super_fine_response_threshold(double value) { super_fine_response_threshold_ = value; }

  const int super_fine_use_point_size(void) const { return super_fine_use_point_size_; }
  void set_super_fine_use_point_size(int value) { super_fine_use_point_size_ = value; }

  bool use_center_penalty() const { return use_center_penalty_; }
  void set_use_center_penalty(bool value) { use_center_penalty_ = value; }

 private:
  
//optimize
  bool   use_optimize_scan_match_ = false;
  int    iterate_times_           = 10;
  double cost_decrease_threshold_ = 1;
  double cost_min_threshold_      = 2;
  double max_update_distance_     = 0.5;
  double max_update_angle_        = 0.2;
  double optimize_failed_cost_    = 10;


//correlattion
  double coarse_search_space_size_       = 0.8;
  double coarse_search_space_resolution_ = 0.1;
  double coarse_search_angle_offset_     = 0.01745 * 80;
  double coarse_search_angle_resolution_ = 0.01745 * 2;
  double coarse_response_threshold_      = 0.6;
  int    coarse_use_point_size_          = 100;

  double fine_search_space_size_       = 0.2;
  double fine_search_space_resolution_ = 0.02;
  double fine_search_angle_offset_     = 0.01745 * 20;
  double fine_search_angle_resolution_ = 0.01745 * 2;
  double fine_response_threshold_      = 0.7;
  int    fine_use_point_size_          = 100;

  double super_fine_search_space_size_       = 0.02;
  double super_fine_search_space_resolution_ = 0.01;
  double super_fine_search_angle_offset_     = 0.01745 * 2;
  double super_fine_search_angle_resolution_ = 0.01745 * 0.2;
  double super_fine_response_threshold_      = 0.7;
  int    super_fine_use_point_size_          = 200;

  bool use_center_penalty_ = true;
};

class ScanMatchers
{
 public:
  ScanMatchers(std::shared_ptr<ScanMatchParam> param,
               std::shared_ptr<SensorDataManager> sensor_data_manager):
      scan_match_param_(param),
      sensor_data_manager_(sensor_data_manager),
      optimize_scan_matcher_(std::make_unique<BasedOptimizeScanMatch>()),
      correlate_scan_matcher_(std::make_unique<BasedCorrelationScanMatch>()),
      fine_correlate_scan_match_param_(std::make_shared<CorrelationScanMatchParam>()),
      super_fine_correlate_scan_match_param_(std::make_shared<CorrelationScanMatchParam>()),
      coarse_correlate_scan_match_param_(std::make_shared<CorrelationScanMatchParam>()),
      fast_correlate_scan_match_param_(std::make_shared<CorrelationScanMatchParam>()),
      optimize_scan_match_param_(std::make_shared<OptimizeScanMatchParam>()){

    ScanMatchParamInit();

  }

  double ScanMatch(std::shared_ptr<RangeDataContainer2d> coarse_map_range_data,
                   std::shared_ptr<RangeDataContainer2d> fine_map_range_data,
                   std::shared_ptr<ScanMatchMap> coarse_map,
                   std::shared_ptr<ScanMatchMap> fine_map,
                   Eigen::Vector3d& best_pose,
                   Eigen::Matrix3d& cov_matrix,
                   bool use_fine_scan_match = true){

//    std::unique_lock<std::mutex> scan_match_lock(scan_match_mutex_);

    double scan_match_score = 0.0;
    int scan_match_times = 0; //record the scanmatch times to calculate the average score at last
    double optimize_cost;
    Eigen::Vector3d process_pose(best_pose);

    //check map size to avoid overstep the boundary during calculating
    MapSizeCheck(coarse_map, coarse_map_range_data, process_pose,
                 coarse_correlate_scan_match_param_->search_space_size());

    MapSizeCheck(fine_map, fine_map_range_data, process_pose,
                 coarse_correlate_scan_match_param_->search_space_size());

#ifdef SLAM_TIME_DEBUG
    TIMER_START(OptimizeScanMatch);
#endif

    if(use_optimize_scan_match_){
      //scan match with optimize based method
      optimize_cost = optimize_scan_matcher_->ScanMatch(coarse_map, coarse_map_range_data,
                                                        optimize_scan_match_param_, process_pose);

      //translate the optimize cost to scan match score
      scan_match_score = optimize_failed_cost_ / (optimize_cost + optimize_failed_cost_);
      scan_match_times++;
    }

#ifdef SLAM_TIME_DEBUG
    TIMER_END_OUTPUT(OptimizeScanMatch);
#endif

#ifdef SLAM_TIME_DEBUG
    TIMER_START(CorrelationScanMatch);
#endif

    // if not use fine resolution scanmatch, must execute the coarse resolution correlate based scanmatch to calculate the covariance
    if(!use_optimize_scan_match_ ||
       !use_fine_scan_match ||
       (use_optimize_scan_match_ && optimize_cost > optimize_failed_cost_)){

      if(use_optimize_scan_match_){
        //if the program running here, that indicates optimization failed
        scan_match_score = 0.0;
        scan_match_times--;
        process_pose = best_pose;

        LOG(WARNING) << " Optimize scan match failed ! ";
      }

      //coarse resolution correlate based scanmatch
      scan_match_score += correlate_scan_matcher_->ScanMatch(fine_map, fine_map_range_data, \
                                                             coarse_correlate_scan_match_param_,
                                                             process_pose, cov_matrix);
      scan_match_times++;
    }

    best_pose = process_pose;


    if(use_fine_scan_match){
      //fine resolution correlate based scanmatch
      scan_match_score += correlate_scan_matcher_->ScanMatch(fine_map, \
                                                             fine_map_range_data, \
                                                             fine_correlate_scan_match_param_,
                                                             process_pose, cov_matrix);
      scan_match_times++;

      //super fine resolution correlate based scanmatch
      scan_match_score += correlate_scan_matcher_->ScanMatch(fine_map, \
                                                             fine_map_range_data, \
                                                             super_fine_correlate_scan_match_param_,
                                                             process_pose, cov_matrix);
      scan_match_times++;
    }

    best_pose = process_pose;


    //test correlate based scanmatch with branch and bound algorithm
//    {
//      scan_match_score += correlate_scan_matcher_->ScanMatch(fine_map, \
//                                                             fine_map_range_data, \
//                                                             fast_correlate_scan_match_param_,
//                                                             process_pose, cov_matrix);
//      scan_match_times++;
//    }


#ifdef SLAM_TIME_DEBUG
    TIMER_END_OUTPUT(CorrelationScanMatch);
#endif

    //statistic the scanmatch score
    scan_match_score /= scan_match_times;

    LOG(INFO) <<  " BEST RESPONSE = " << scan_match_score
              << ",  VARIANCE = "
              << cov_matrix(0, 0) << ", " << cov_matrix(1, 1) << ", " << cov_matrix(2, 2);


    return scan_match_score;
  }

  void SetScanMatchParam(std::shared_ptr<ScanMatchParam> param){
//    std::unique_lock<std::mutex> scan_match_lock(scan_match_mutex_);

    scan_match_param_ = param;
    ScanMatchParamInit();
  }

  std::mutex& GetScanMatchMutex(void){
    return scan_match_mutex_;
  }

 private:

  /**
   * @brief init scan match param
   */
  void ScanMatchParamInit(void){

    coarse_correlate_scan_match_param_->set_search_space_size(scan_match_param_->coarse_search_space_size());
    coarse_correlate_scan_match_param_->set_search_space_resolution(scan_match_param_->coarse_search_space_resolution());
    coarse_correlate_scan_match_param_->set_search_angle_offset(scan_match_param_->coarse_search_angle_offset());
    coarse_correlate_scan_match_param_->set_search_angle_resolution(scan_match_param_->coarse_search_angle_resolution());
    coarse_correlate_scan_match_param_->set_response_threshold(scan_match_param_->coarse_response_threshold());
    coarse_correlate_scan_match_param_->set_use_point_size(scan_match_param_->coarse_use_point_size());
    coarse_correlate_scan_match_param_->set_correlation_scan_match_type(COARSE_CORRELATION_SCAN_MATCH);
    coarse_correlate_scan_match_param_->set_use_center_penalty(scan_match_param_->use_center_penalty());

    fine_correlate_scan_match_param_->set_search_space_size(scan_match_param_->fine_search_space_size());
    fine_correlate_scan_match_param_->set_search_space_resolution(scan_match_param_->fine_search_space_resolution());
    fine_correlate_scan_match_param_->set_search_angle_offset(scan_match_param_->fine_search_angle_offset());
    fine_correlate_scan_match_param_->set_search_angle_resolution(scan_match_param_->fine_search_angle_resolution());
    fine_correlate_scan_match_param_->set_response_threshold(scan_match_param_->fine_response_threshold());
    fine_correlate_scan_match_param_->set_use_point_size(scan_match_param_->fine_use_point_size());
    fine_correlate_scan_match_param_->set_correlation_scan_match_type(FINE_CORRELATION_SCAN_MATCH);
    fine_correlate_scan_match_param_->set_use_center_penalty(scan_match_param_->use_center_penalty());

    super_fine_correlate_scan_match_param_->set_search_space_size(scan_match_param_->super_fine_search_space_size());
    super_fine_correlate_scan_match_param_->set_search_space_resolution(scan_match_param_->super_fine_search_space_resolution());
    super_fine_correlate_scan_match_param_->set_search_angle_offset(scan_match_param_->super_fine_search_angle_offset());
    super_fine_correlate_scan_match_param_->set_search_angle_resolution(scan_match_param_->super_fine_search_angle_resolution());
    super_fine_correlate_scan_match_param_->set_response_threshold(scan_match_param_->super_fine_response_threshold());
    super_fine_correlate_scan_match_param_->set_use_point_size(scan_match_param_->super_fine_use_point_size());
    super_fine_correlate_scan_match_param_->set_correlation_scan_match_type(SUPER_CORRELATION_SCAN_MATCH);
    super_fine_correlate_scan_match_param_->set_use_center_penalty(scan_match_param_->use_center_penalty());

    //branch_and_bound test
    fast_correlate_scan_match_param_->set_search_space_size(0.8);
    fast_correlate_scan_match_param_->set_search_space_resolution(0.01);
    fast_correlate_scan_match_param_->set_search_angle_offset(0.523);
    fast_correlate_scan_match_param_->set_search_angle_resolution(0.00349);
    fast_correlate_scan_match_param_->set_response_threshold(0.5);
    fast_correlate_scan_match_param_->set_use_point_size(100);
    fast_correlate_scan_match_param_->set_max_depth(4);
    fast_correlate_scan_match_param_->set_correlation_scan_match_type(FAST_CORRELATION_SCAN_MATCH);

    optimize_scan_match_param_->set_iterate_max_times(scan_match_param_->iterate_times());
    optimize_scan_match_param_->set_cost_decrease_threshold(scan_match_param_->cost_decrease_threshold());
    optimize_scan_match_param_->set_cost_min_threshold(scan_match_param_->cost_min_threshold());
    optimize_scan_match_param_->set_max_update_distance(scan_match_param_->max_update_distance());
    optimize_scan_match_param_->set_max_update_angle_(scan_match_param_->max_update_angle());


    use_optimize_scan_match_ = scan_match_param_->use_optimize_scan_match();
    optimize_failed_cost_ = scan_match_param_->optimize_failed_cost();
  }

  /**
   * @brief check that the current range points is in the map
   * @param map : current scanmatch map
   * @param range_data : current frame range data
   * @param init_pose : predicted current sensor pose
   * @param offset : reserved search space size
   * @return if the farthest range point is in the map extent, return true
   */
  bool MapSizeCheck(std::shared_ptr<ScanMatchMap> map,
                    std::shared_ptr<RangeDataContainer2d> range_data,
                    Eigen::Vector3d& init_pose,
                    double offset){

    Eigen::Vector3d init_pose_world = init_pose;
    Eigen::Vector3d center_pose = map->GetMapCoordsPose(init_pose_world);
//    LOG(INFO) << "center_pose " << center_pose.transpose() << std::endl;

    double map_resolution = map->GetCellLength();

    if(sensor_data_manager_->GetRangeFinder() == nullptr) return false;
    double range_max = sensor_data_manager_->GetRangeFinder()->range_max();
    double max_size = (range_max + offset) / map_resolution;
    BoundBox2d range_point_boundbox(Eigen::Vector2d((center_pose.x() - max_size), (center_pose.y() - max_size)),
                                    Eigen::Vector2d((center_pose.x() + max_size), (center_pose.y() + max_size)));

    if(!map->UpdateBound(range_point_boundbox)){
      LOG(INFO) << "range_max : " << range_max
                << "   offset : " << offset;
      LOG(INFO) << "ScanMatch update map size !";
      return false;
    }

    return true;
  }


 private:

//  std::shared_ptr<MapManager> map_manager_;
  std::shared_ptr<SensorDataManager> sensor_data_manager_;

  //scan matcher
  std::unique_ptr<BasedOptimizeScanMatch> optimize_scan_matcher_;
  std::unique_ptr<BasedCorrelationScanMatch> correlate_scan_matcher_;

  //param
  std::shared_ptr<ScanMatchParam> scan_match_param_;
  std::shared_ptr<CorrelationScanMatchParam> coarse_correlate_scan_match_param_;
  std::shared_ptr<CorrelationScanMatchParam> fine_correlate_scan_match_param_;
  std::shared_ptr<CorrelationScanMatchParam> super_fine_correlate_scan_match_param_;
  std::shared_ptr<CorrelationScanMatchParam> fast_correlate_scan_match_param_;
  std::shared_ptr<OptimizeScanMatchParam> optimize_scan_match_param_;

  bool use_optimize_scan_match_;
  double optimize_failed_cost_;

  std::mutex scan_match_mutex_;

  //debug
};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_SCAN_MATCH_SCAN_MATCHERS_H



