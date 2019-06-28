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

#ifndef ROBORTS_SLAM_SLAM_SLAM_PROCESSOR_H
#define ROBORTS_SLAM_SLAM_SLAM_PROCESSOR_H

#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <float.h>

#include "sensor_data_manager.h"
#include "map/map_manager.h"
#include "scan_match/scan_matchers.h"
#include "pose_graph/range_scan_pose_graph.h"
#include "util/slam_util.h"

namespace roborts_slam{


class SlamProcessor{
 public:
  SlamProcessor(std::shared_ptr<ParamConfig> param,
                std::shared_ptr<SensorDataManager> sensor_data_manager,
                double map_resolution);

  ~SlamProcessor();

  /**
   * @brief slam process main thread.
   * @return The slam process result(true is succeed)
   */
  bool process(void);

  /**
   * @brief Scan match interface for backend
   * @param range_data : current laser range data
   * @param closest_id : not use now
   * @param range_id : getting the range datas from the sensor data manager by these ids
   * @param best_pose : return the match result pose
   * @param cov_matrix : return the match result covariance
   * @param use_fine_scan_match : use fine resolution scanmatch if this is true
   * @param cov_matrix : use default scanmatch patam if this is true
   * @return the scanmatch response score
   */
  double ScanMatchInterface(const std::shared_ptr<RangeDataContainer2d>& range_data,
                            int closest_id,
                            const std::vector<int>& range_id,
                            Eigen::Vector3d& best_pose,
                            Eigen::Matrix3d& cov_matrix,
                            bool use_fine_scan_match = true,
                            bool use_front_end_scan_match_param = true);


  /**
   * @brief correct the poses and maps (after backend optimization)
   * @param corrected_pose : the optimized poses
   */
  void CorrectPoseAndMap(const std::vector<std::pair<int, Eigen::Vector3d>>& corrected_pose);

  /**
   * @brief set or get map_resolution
   */
  void set_map_resolution(double map_resolution);
  double map_resolution(void) const ;

  /**
   * @brief set or get current_sensor_pose
   */
  void set_current_sensor_pose(const Eigen::Vector3d& current_sensor_pose);
  Eigen::Vector3d current_sensor_pose(void) const ;

  /**
   * @brief get the occupying grid map
   */
  std::shared_ptr<PubMap> GetPubMap(void);

  std::mutex& GetMapMutex(void);

  /**
   * @brief get vertexes and edges of pose graph for visualization
   */
  void GetGraphInfo(std::vector<Eigen::Vector2d> &nodes,
                    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > & edges);

  /**
   * @brief force to running optimization once
   */
  void ForceGraphOptimize(void);


 private:

  /**
   * @brief send the signal to backend thread for update
   * @param data_index : current processing range data id
   * @param data_cov : current processing range data covariance
   */
  void BackEndDataUpdataSignal(int data_index,
                               const Eigen::Matrix3d& data_cov);

  void BackEndProcessThread(void);

  /**
   * @brief create the map for scanmatch with range data(for backend scanmatch)
   * @param range_data_vec : used range data vector
   * @param resolution : the map resolution
   * @param deviation : sigma of gaussian blur
   * @param use_blur : if use gaussian blur for map
   * @param gaussian_blur_offset : the occupied probobility of map grid if use gaussian blur
   * @return the created map
   */
  std::shared_ptr<ScanMatchMap> CreateScanMatchMapWithRangeVec(
      const std::vector<std::shared_ptr<RangeDataContainer2d>>& range_data_vec,
      double resolution, double deviation, bool use_blur,
      double gaussian_blur_offset);

  /**
   * @brief reset and update map with given range datas(for backend scanmatch)
   * @param range_data_vec : given range datas for updating scanmatch map
   * @param map : given scanmatch map
   * @param use_blur : if use gaussian blur for map
   */
  void ResetScanMatchMapWithRangeVec(const std::vector<std::shared_ptr<RangeDataContainer2d>>& range_data_vec,
                                     std::shared_ptr<ScanMatchMap> map, bool use_blur);

  /**
   * @brief create all the maps of different resolutions
   */
  void CreateAllMap(void);

  /**
   * @brief update all the maps with current range data
   * @param pub_map_range_data : range data for pub map resolution
   * @param coarse_map_range_data : range data for coarse map resolution
   * @param fine_map_range_data : range data for fine map resolution
   * @return if update succeed, return true
   */
  bool UpdateMap(std::shared_ptr<RangeDataContainer2d> pub_map_range_data,
                 std::shared_ptr<RangeDataContainer2d> coarse_map_range_data,
                 std::shared_ptr<RangeDataContainer2d> fine_map_range_data);


  double MapCheckPenalize(std::shared_ptr<RangeDataContainer2d> map_range_data ,
                          const Eigen::Vector3d& candidate_pose,
                          bool use_logistic = false);

  /**
   * @brief set the sensor pose of range data
   * @param id : range data id
   * @param sensor_pose : the pose of this range data
   */
  void UpdateRangeData(int id, const Eigen::Vector3d& sensor_pose);

  /**
   * @brief check the robot if having a enough motion by odometry information
   * @param last_odom_pose last frame odometry pose
   * @param cur_odom_pose current frame odometry pose
   * @return if the robot has move enough, return true
   */
  bool MoveEnough(Eigen::Vector3d cur_odom_pose,
                  Eigen::Vector3d last_odom_pose);

  /**
   * @brief predict the current sensor pose by odometry,
   *        get the better initial value for scan match
   * @param last_pose last frame sensor pose
   * @param last_odom last frame odometry pose
   * @param cur_odom current frame odometry pose
   * @return predicted current sensor pose
   */
  Eigen::Vector3d PredictPoseByOdom(const Eigen::Vector3d& last_pose,
                                    const Eigen::Vector3d& last_odom,
                                    const Eigen::Vector3d& cur_odom);

  /**
   * @brief init system slam param
   */
  void SlamParamInit(void);


 private:
  //param
  std::shared_ptr<ParamConfig> param_;
  std::shared_ptr<ScanMatchParam> front_end_scan_match_param_;
  std::shared_ptr<ScanMatchParam> back_end_scan_match_param_;

  //sensor data manager
  std::shared_ptr<SensorDataManager> sensor_data_manager_;

//scan matcher
  std::unique_ptr<ScanMatchers> scan_matchers_;

  //map
  std::shared_ptr<MapManager> map_manager_;
  bool pub_map_created_;
  float map_resolution_;
  std::mutex map_mutex_;

  //backend
  std::unique_ptr<RangeScanPoseGraph> range_data_pose_graph_;
  bool back_end_running_;
  std::unique_ptr<std::thread> back_end_thread_;
  std::mutex back_end_mutex_;
  std::mutex back_end_data_mutex_;
  std::condition_variable back_end_process_condition_;
  std::deque<std::pair<int, Eigen::Matrix3d>> back_end_data_buffer_;
  bool new_vertex_;

  //pose for update
  Eigen::Vector3d current_sensor_pose_;
  Eigen::Vector3d last_map_update_pose_;

  std::chrono::steady_clock::time_point last_process_time_;

  //slam param
  bool use_odometry_;
  bool is_first_range_data_;
  int current_data_index_;
  double scan_match_score_;
  bool close_loop_;
  int map_penalize_times_;

  double init_map_size_;
  double map_offset_x_;
  double map_offset_y_;

  bool   use_move_check_;
  double move_distance_threshold_;
  double move_angle_threshold_;
  double move_time_threshold_;

  bool   use_map_update_move_check_;
  double map_update_score_threshold_;
  double map_update_distance_threshold_;
  double map_update_angle_threshold_;
  double map_update_free_factor_;
  double map_update_occu_factor_;
  double map_occu_threshold_;
  double map_min_passthrough_;

  //preset factor
  const double kMinMapSize = 3;
  const double kMinScanMatchMapBound = 2.0;
  const float kMapUnknownCellProb = 0.3f;
  const bool kUseRunningRangeScanMatch = false;

};

} // namespace roborts_slam


#endif // ROBORTS_SLAM_SLAM_SLAM_PROCESSOR_H
