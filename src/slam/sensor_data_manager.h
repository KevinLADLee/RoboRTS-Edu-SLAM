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

#ifndef ROBORTS_SLAM_SLAM_SENSOR_DATA_MANAGER_H
#define ROBORTS_SLAM_SLAM_SENSOR_DATA_MANAGER_H

#include <vector>
#include <map>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "util/slam_util.h"

namespace roborts_slam{

class LaserRangeFinder{
 public:
  LaserRangeFinder(){}

  LaserRangeFinder(double min_angle, double max_angle, double angle_resolution, double min_range, double max_range):
      min_angle_(min_angle),
      max_angle_(max_angle),
      angle_resolution_(angle_resolution),
      min_range_(min_range),
      max_range_(max_range){}

  void set_range_threshold_scale(double range_threshold_scale){
    range_threshold_scale_ = range_threshold_scale;

    if(min_range_ < max_range_){
      range_threshold_= min_range_ + range_threshold_scale_ * (max_range_ - min_range_);
    }
  }

  void set_range_threshold(double range_threshold){
    if(range_threshold > min_angle_ && range_threshold <= max_range_){
      range_threshold_ = range_threshold;
    }
  }

  double range_threshold(void){
    return range_threshold_;
  }

  double range_max(void){
    return max_range_;
  }


 private:

  double min_angle_;
  double max_angle_;
  double angle_resolution_;

  double min_range_;
  double max_range_;
  double range_threshold_;

  double range_threshold_scale_;

};

enum CurrentCoordinate
{
  kCoordinateLocal = 0,
  kCoordinateWorld,
  kCoordinateMap,
} ;

template<typename T>
class RangeDataContainer
{
 public:

  RangeDataContainer(int size = 1000) {
    data_point_.reserve(size);

    cur_range_point_frame_ = kCoordinateLocal;
    scale_factor_ = 1.0;
  }

  void CreateFrom(const std::shared_ptr<RangeDataContainer> other, double factor) {

    scale_factor_ = factor;

    sensor_pose_ = other->sensor_pose();
    sensor_origin_ = other->sensor_origin() * scale_factor_;

    data_point_ = other->data_point_;

    unsigned int size = data_point_.size();

    for (unsigned int i = 0; i < size; ++i){
      data_point_[i] *= scale_factor_;
    }

    bound_box_.Reset();
  }

  void AddDataPoint(const Eigen::Matrix<T, 2, 1>& point) {
    data_point_.push_back(point);
  }

  void TransformPoint(const Eigen::Affine2d& transform,
                      std::vector<Eigen::Matrix<T, 2, 1>>& data_point){
    for(auto & point : data_point){
      point = transform * point;
    }
  }

  void TransformUpdate(const Eigen::Affine2d& transform){
    switch(cur_range_point_frame_){
      case kCoordinateLocal:
      {
        TransformPoint(transform, data_point_);
        break;
      }
      case kCoordinateWorld:
      {
        TransformPoint(transform * local_to_world_.inverse(), data_point_);
        break;
      }
      case kCoordinateMap:
      {
        TransformPoint(transform * local_to_map_.inverse(), data_point_);
        break;
      }
      default:
        break;
    }
  }

  void TransformLocalToMap(const Eigen::Affine2d& transform){
    TransformUpdate(transform);

    local_to_map_ = transform;
    cur_range_point_frame_ = kCoordinateMap;
  }

  void TransformLocalToWorld(const Eigen::Affine2d& transform){
    TransformUpdate(transform);

    local_to_world_ = transform;
    cur_range_point_frame_ = kCoordinateWorld;
  }

  void TransformToLocal(void){
    switch(cur_range_point_frame_){
      case kCoordinateLocal:
      {
        break;
      }
      case kCoordinateWorld:
      {
        TransformPoint(local_to_world_.inverse(), data_point_);
        break;
      }
      case kCoordinateMap:
      {
        TransformPoint(local_to_map_.inverse(), data_point_);
        break;
      }
      default:
        break;
    }

    cur_range_point_frame_ = kCoordinateLocal;
  }

  std::vector<Eigen::Matrix<T, 2, 1>> GetLocalPointVec(void){

    std::vector<Eigen::Matrix<T, 2, 1>> local_point_vec;

    switch(cur_range_point_frame_){
      case kCoordinateLocal:
      {
        local_point_vec =  data_point_;
        break;
      }
      case kCoordinateWorld:
      {
        TransformPoint(local_to_world_.inverse(), local_point_vec);
        break;
      }
      case kCoordinateMap:
      {
        TransformPoint(local_to_map_.inverse(), local_point_vec);
        break;
      }
      default:
        break;
    }

    return local_point_vec;
  }

  void UpdateBarycenterPose(void){

    std::vector<Eigen::Matrix<T, 2, 1>> data_point_vec = GetLocalPointVec();
    double transform_scale_factor = 1.0 / scale_factor_;

    Eigen::Affine2d pose_transform((Eigen::Translation2d(this->sensor_pose()[0], this->sensor_pose()[1]) *
                                    Eigen::Rotation2Dd(this->sensor_pose()[2]) *
                                    Eigen::AlignedScaling2d(transform_scale_factor, transform_scale_factor)));

    TransformPoint(pose_transform, data_point_vec);

    Eigen::Matrix<T, 2, 1> point_sum = Eigen::Matrix<T, 2, 1>::Zero();
    for(auto point : data_point_vec){
      point_sum += point;
    }

    int point_size = data_point_vec.size();
    if(point_size > 0){
      point_sum.array() /= point_size;
    }

    barycenter_pose_[0] = static_cast<double>(point_sum[0]);
    barycenter_pose_[1] = static_cast<double>(point_sum[1]);
    barycenter_pose_[2] = this->sensor_pose()[2];
  }


  void UpdateBound(void){
    bound_box_.Reset();

    for(auto point : data_point_){
      bound_box_.AddPoint(point);
    }
  }

  const BoundBox2d GetBoundBox(void) const{
    return bound_box_;
  }

  void Clear() {
    data_point_.clear();
  }

  int GetSize() const {
    return data_point_.size();
  }

  const Eigen::Matrix<T, 2, 1>& GetDataPoint(int index) const {
    CHECK(index < data_point_.size()) << "Invalid data point index";
    return data_point_[index];
  }

  inline Eigen::Matrix<T, 2, 1>& operator [] (int index) {
    CHECK(index < data_point_.size()) << "Invalid data point index";
    return data_point_[index];
  }

  Eigen::Matrix<T, 2, 1> sensor_origin() const {
    return sensor_origin_;
  }

  void set_sensor_origin(const Eigen::Matrix<T, 2, 1>& origin) {
    sensor_origin_ = origin;
  }

  void set_sensor_pose(const Eigen::Vector3d& sensor_pose){
    std::unique_lock<std::mutex> sensor_pose_lock(sensor_pose_mutex_);
    sensor_pose_ = sensor_pose;
  }

  Eigen::Vector3d sensor_pose(void) {
    std::unique_lock<std::mutex> sensor_pose_lock(sensor_pose_mutex_);
    return sensor_pose_;
  }

  void set_barycenter_pose(const Eigen::Vector3d& barycenter_pose){
    std::unique_lock<std::mutex> barycenter_pose_lock(barycenter_pose_mutex_);
    barycenter_pose_ = barycenter_pose;
  }

  Eigen::Vector3d barycenter_pose(void){
    std::unique_lock<std::mutex> barycenter_pose_lock(barycenter_pose_mutex_);

    UpdateBarycenterPose();

    return barycenter_pose_;
  }

  int id(void) {
    std::lock_guard<std::mutex> id_lock(id_mutex_);
    return id_;
  }

  void set_id(int id) {
    std::lock_guard<std::mutex> id_lock(id_mutex_);
    id_ = id;
  }

  int scale_factor(void) const {
    return scale_factor_;
  }

  void set_scale_factor(int scale_factor) {
    scale_factor_ = scale_factor;
  }

  CurrentCoordinate cur_range_point_frame(void) const {
    return cur_range_point_frame_;
  }

 protected:

  std::vector<Eigen::Matrix<T, 2, 1>> data_point_;
  Eigen::Matrix<T, 2, 1> sensor_origin_;
  BoundBox2d bound_box_;

  Eigen::Vector3d sensor_pose_;
  Eigen::Vector3d barycenter_pose_;

  int id_;
  double scale_factor_;

  Eigen::Affine2d local_to_world_;
  Eigen::Affine2d local_to_map_;
  CurrentCoordinate cur_range_point_frame_;

  std::mutex id_mutex_;
  std::mutex sensor_pose_mutex_;
  std::mutex barycenter_pose_mutex_;
};

using RangeDataContainer2f = RangeDataContainer<float>;
using RangeDataContainer2d = RangeDataContainer<double>;



class OdometryData{
 public:
  OdometryData(Eigen::Vector3d odom_pose): odom_pose_(odom_pose){}

  Eigen::Vector3d odom_pose(void){
    return odom_pose_;
  }

  void set_odom_pose(const Eigen::Vector3d& odom_pose){
    odom_pose_ = odom_pose;
  }

 private:
  Eigen::Vector3d odom_pose_;
};

class SensorDataManager{
 public:
  SensorDataManager():
      range_finder_(nullptr),
      current_data_index_(-1){}

  ~SensorDataManager(){

    if(sensor_range_.size() != 0){
      sensor_range_.clear();
    }

    sensor_odom_.clear();
  }

  int current_data_index(void){
    return current_data_index_;
  }

  std::unique_ptr<LaserRangeFinder>& GetRangeFinder(void){
    return range_finder_;
  }

  void SetRangeFinder(std::unique_ptr<LaserRangeFinder> range_finder){
    range_finder_ = std::move(range_finder);
  }

  void AddSensorData(std::shared_ptr<RangeDataContainer2d> range_data, OdometryData odom){
    std::unique_lock<std::mutex> sensor_range_lock(sensor_range_mutex_);

    if(range_data == nullptr){
      return;
    }

    sensor_range_.push_back(range_data);
    sensor_odom_.push_back(odom);

    current_data_index_++;
  }

  std::vector<std::shared_ptr<RangeDataContainer2d>> GetRangeDataVec(void){
    std::unique_lock<std::mutex> sensor_range_lock(sensor_range_mutex_);
    return sensor_range_;
  }

  std::vector<int> GetRangeIdVec(void){
    std::vector<int> range_id_vec;
    for(int i = 0; i <= current_data_index_; ++i){
      range_id_vec.push_back(i);
    }

    return range_id_vec;
  }

  std::shared_ptr<RangeDataContainer2d> GetRangeData(int index = -1){
    std::unique_lock<std::mutex> sensor_range_lock(sensor_range_mutex_);

    int res_index = current_data_index_;
    if(index >= 0 && index < sensor_range_.size()){
      res_index = index;
    }

    return sensor_range_[res_index];
  }

  std::shared_ptr<RangeDataContainer2d> GetLastRangeData(void){
    int last_index = current_data_index_;
    if(current_data_index_ > 0){
      last_index = current_data_index_ - 1;
    }
    return GetRangeData(last_index);
  }

  Eigen::Vector3d GetSensorPose(int index = -1){
    return GetRangeData(index)->sensor_pose();
  }

  OdometryData GetOdomData(int index = -1){
    int res_index = current_data_index_;
    if(index >= 0 && index < sensor_odom_.size()){
      res_index = index;
    }
    return sensor_odom_[res_index];
  }

  Eigen::Vector3d GetOdomPose(int index = -1){
    int res_index = current_data_index_;
    if(index >= 0 && index < sensor_odom_.size()){
      res_index = index;
    }
    return sensor_odom_[res_index].odom_pose();
  }

  Eigen::Vector3d SetOdomPose(const Eigen::Vector3d& odom_pose, int index = -1){
    int res_index = current_data_index_;
    if(index >= 0 && index < sensor_odom_.size()){
      res_index = index;
    }

    sensor_odom_[res_index].set_odom_pose(odom_pose);
  }

  void ClearCurrentData(void){
    std::unique_lock<std::mutex> sensor_range_lock(sensor_range_mutex_);

    if(current_data_index_ >= 0){
      sensor_range_.pop_back();
      sensor_odom_.pop_back();

      current_data_index_--;
    }
  }

  bool IsFirstRangeData(void){
    if(current_data_index_ == 0){
      return true;
    }
    return false;
  }

  std::shared_ptr<RangeDataContainer2d> GetMultiresolutionRangeData(const std::string& name,
                                                                    int index = -1){
    std::unique_lock<std::mutex> multiresolution_range_data_lock(multiresolution_range_data_mutex_);

    if(multiresolution_range_data_.find(name) != multiresolution_range_data_.end()){
      int res_index = current_data_index_;
      if(index >= 0 && index < multiresolution_range_data_[name].size()){
        res_index = index;
      }
      return multiresolution_range_data_[name][res_index];
    }else{
      return nullptr;
    }

  }

  void AddMultiresolutionRangeData(const std::string& name,
                                   std::shared_ptr<RangeDataContainer2d> range_data){
    std::unique_lock<std::mutex> multiresolution_range_data_lock(multiresolution_range_data_mutex_);
    multiresolution_range_data_[name].push_back(range_data);
  }


  std::vector<std::shared_ptr<RangeDataContainer2d>> GetMultiresolutionRangeDataVec(const std::string& name){
    std::unique_lock<std::mutex> multiresolution_range_data_lock(multiresolution_range_data_mutex_);
    return multiresolution_range_data_[name];
  }

  std::vector<std::shared_ptr<RangeDataContainer2d>> GetMultiresolutionRangeDataVecWithId(const std::string& name,
                                                                                          const std::vector<int>& range_id){
    std::unique_lock<std::mutex> multiresolution_range_data_lock(multiresolution_range_data_mutex_);

    std::vector<std::shared_ptr<RangeDataContainer2d>> range_data_vec;

    for(auto id : range_id){
      range_data_vec.push_back(multiresolution_range_data_[name][id]);
    }

    return range_data_vec;
  }


  std::vector<std::shared_ptr<RangeDataContainer2d>> GetRunningRangeDataVec(const std::string& name){
    std::unique_lock<std::mutex> running_range_lock(running_range_mutex);

    return GetMultiresolutionRangeDataVecWithId(name, running_range_id_);
  }

  std::vector<int> GetRunningRangeIdVec(void){
    std::unique_lock<std::mutex> running_range_lock(running_range_mutex);

    return running_range_id_;
  }

  void UpdateRunningRange(int current_id){

    std::unique_lock<std::mutex> running_range_lock(running_range_mutex);

    running_range_id_.push_back(current_id);

    std::vector<std::shared_ptr<RangeDataContainer2d>>& range_data_vec = sensor_range_;
    Eigen::Vector3d front_pose = range_data_vec[running_range_id_.front()]->sensor_pose();
    Eigen::Vector3d back_pose = range_data_vec[running_range_id_.back()]->sensor_pose();

    double distance = util::EuclideanDistance2d(front_pose.head<2>(), back_pose.head<2>());
    while(running_range_id_.size() > running_range_size_ || distance > running_range_max_distance_){
      running_range_id_.erase(running_range_id_.begin());

      front_pose = range_data_vec[running_range_id_.front()]->sensor_pose();
      back_pose = range_data_vec[running_range_id_.back()]->sensor_pose();
      distance = util::EuclideanDistance2d(front_pose.head<2>(), back_pose.head<2>());
    }
  }

  void set_running_range_size(int value){
    if(value > 0){
      running_range_size_ = value;
    }
  }

  void set_running_range_max_distance(double value){
    if(value > 0){
      running_range_max_distance_ = value;
    }
  }


 private:

  std::unique_ptr<LaserRangeFinder> range_finder_;
  std::vector<std::shared_ptr<RangeDataContainer2d>> sensor_range_;
  std::vector<OdometryData> sensor_odom_;

  std::map<std::string, std::vector<std::shared_ptr<RangeDataContainer2d>>> multiresolution_range_data_;

  std::vector<int> running_range_id_;
  int running_range_size_ = 70;
  double running_range_max_distance_ = 5.0;

  int current_data_index_;

  std::mutex sensor_range_mutex_;
  std::mutex multiresolution_range_data_mutex_;
  std::mutex running_range_mutex;


};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_SLAM_SENSOR_DATA_MANAGER_H