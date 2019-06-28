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

#ifndef ROBORTS_SLAM_MAP_OCCU_GRID_MAP_H
#define ROBORTS_SLAM_MAP_OCCU_GRID_MAP_H

#include <iostream>
#include <vector>
#include <array>
#include "slam/sensor_data_manager.h"

#include "grid_map_base.h"
#include "grid_map_cell.h"


namespace roborts_slam{

enum CellUpdateType {
  SET_CELL_FREE = 0,
  SET_CELL_OCCUPIED,
  SET_CELL_OCCUPIED_BLUR,
};

class GaussianBlur{
 public:
  GaussianBlur(double sigma, double map_resolution) :
      sigma_(sigma),
      resolution_(map_resolution){

    const double min_distance_deviation = 0.5 * map_resolution;
    const double max_distance_deviation = 10 * map_resolution;
    if(sigma > min_distance_deviation &&
        sigma < max_distance_deviation &&
        map_resolution > 0){

      blur_states_ = true;
      InitKernel();

    }else{
      half_kernel_size_ = 0;
      kernel_size_ = 0;
      blur_states_ = false;
      LOG(WARNING) << "Invalid gaussian blur param !";
    }
  }

  bool GetBlurStates() const {
    return blur_states_;
  }

  double GetKernelHalfSize(){
    return half_kernel_size_ * resolution_;
  }

  int GetKernelGridHalfSize() const{
    return half_kernel_size_;
  }

  int GetKernelGridSize() const{
    return kernel_size_;
  }

  double* GetKernelValuePointer() const{
    return kernel_val_;
  }


 private:
  void InitKernel(){
    CalculateKernelSize();

    kernel_val_ = new double[kernel_size_ * kernel_size_];

    for(int i = -half_kernel_size_;  i <= half_kernel_size_; ++i ){
      for(int j = -half_kernel_size_; j <= half_kernel_size_; ++j){
        double distance_to_mean = hypot(i * resolution_, j * resolution_);
        double z = exp(-0.5 * pow(distance_to_mean / sigma_, 2));
        int kernel_index = (i + half_kernel_size_) + kernel_size_ * (j + half_kernel_size_);

        kernel_val_[kernel_index] = z;
//        LOG(INFO) << z << "  ";
      }
//      LOG(INFO);
    }
  }

  void CalculateKernelSize(){
//    half_kernel_size_ = static_cast<int>(std::floor((2.0 * sigma_ / resolution_) + 0.5));
    half_kernel_size_ = static_cast<int>((sigma_ / resolution_) * sqrt(log(2)));
    kernel_size_ = half_kernel_size_ * 2 + 1;
  }

  bool blur_states_ = false;
  double sigma_;
  double resolution_;
  int kernel_size_;
  int half_kernel_size_;

  double* kernel_val_;

};


using LineVisitorCallbackFunc = std::function<void(const Eigen::Vector2i&, double&)>;
class LineVisitor{
 public:
  LineVisitor(const LineVisitorCallbackFunc& line_visitor_callback){
    line_visitor_callback_ = line_visitor_callback;
  }

  inline double ErgodLineBresenhami(const Eigen::Vector2i& start_point,
                                    const Eigen::Vector2i& end_point){
    double res = 0.0;

    int x0 = start_point[0];
    int y0 = start_point[1];

    int x1 = end_point[0];
    int y1 = end_point[1];

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    if (x0 > x1)
    {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }

    int delta_x = x1 - x0;
    int delta_y = abs(y1 - y0);
    int error = 0;
    int y_step;
    int y = y0;

    if (y0 < y1)
    {
      y_step = 1;
    }
    else
    {
      y_step = -1;
    }

    Eigen::Vector2i point;
    for (int x = x0; x <= x1; x++)
    {
      if (steep)
      {
        point.x() = y;
        point.y() = x;
      }
      else
      {
        point.x() = x;
        point.y() = y;
      }

      error += delta_y;

      if (2 * error >= delta_x)
      {
        y += y_step;
        error -= delta_x;
      }

      this->line_visitor_callback_(point, res);
    }

    return res;
  }

 private:
  LineVisitorCallbackFunc line_visitor_callback_;
};

template<typename CellType, typename CellFunctionsType>
class OccuGridMap : public GridMapBase<CellType>{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OccuGridMap(double resolution,  const Eigen::Vector2i& size, const Eigen::Vector2d& offset,
              double deviation = 0.0, float default_cell_prob = kDefaultCellProb)
      : GridMapBase<CellType>(resolution, size, offset, default_cell_prob),
        gaussian_blur_(std::make_unique<GaussianBlur>(deviation, resolution)),
        use_auto_map_resize_(true),
        just_update_occu_(false),
        cur_update_index(0),
        cur_mark_occu_index(-1),
        cur_mark_free_index(-1){

    half_kernel_size_ = gaussian_blur_->GetKernelGridHalfSize();
    use_blur_ = gaussian_blur_->GetBlurStates();

    line_visitor_ = std::make_unique<LineVisitor>(std::bind(&OccuGridMap::BeamLineVisitorCallback, this,
                                                            std::placeholders::_1,
                                                            std::placeholders::_2));
  }

  virtual ~OccuGridMap() {}


  void InitMapWithRangeVec(const std::vector<std::shared_ptr<RangeDataContainer2d>>& range_data_vec,
                           bool use_blur = false, bool use_reset_speedup = false){

//    TIMER_START(InitMapWithRangeVec);

    if(use_reset_speedup){
      this->ResetValueSpeedup(map_update_point_);
    }else{
      this->Reset();
    }

    cur_update_index = 0;
    cur_mark_occu_index = -1;
    cur_mark_free_index = -1;

    map_update_point_.clear();

    for(auto range_data_raw : range_data_vec){
      int try_update_map_times = 5;
      while(!UpdateMapByRange(range_data_raw, use_blur) && try_update_map_times){
        try_update_map_times--;
        LOG(WARNING) << "try_update_map_times " << try_update_map_times;
      }
//      LOG(INFO) << "  " << range_data_raw->id() ;
//      LOG(INFO) << "range_sensor_pose " << range_data_raw->sensor_pose().transpose();
    }

    if(!use_auto_map_resize_){
      this->UpdateBoundAdaptMap();
    }

//    TIMER_END_OUTPUT(InitMapWithRangeVec);

  }


  bool UpdateMapByRange(std::shared_ptr<RangeDataContainer2d> range_data_raw,
                        bool use_blur = false){

    if(range_data_raw->GetSize() == 0){
      LOG(WARNING) << "Range data is null !";
    }

    if(!gaussian_blur_->GetBlurStates()){
      use_blur = false;
    }
    use_blur_ = use_blur;

//    LOG(INFO) << "use_blur " << use_blur;

    cur_mark_free_index = cur_update_index + 1;
    cur_mark_occu_index = cur_update_index + 2;

    std::shared_ptr<RangeDataContainer2d> range_data = std::make_shared<RangeDataContainer2d>();
    range_data->CreateFrom(range_data_raw, 1.0);

    Eigen::Vector3d pose_in_map(this->GetMapCoordsPose(range_data->sensor_pose()));
    Eigen::Affine2d pose_transform((Eigen::Translation2d(pose_in_map[0], pose_in_map[1]) * Eigen::Rotation2Dd(pose_in_map[2])));
//    LOG(INFO) << "pose_in_world : " << range_data->get_sensor_pose().transpose();
//    LOG(INFO) << "pose_in_map : " << pose_in_map.transpose();

    range_data->TransformLocalToMap(pose_transform);

    if(use_auto_map_resize_){
      range_data->UpdateBound();
      BoundBox2d range_point_boundbox = range_data->GetBoundBox();
      if(use_blur){
        range_point_boundbox.ExtendBoundBox(gaussian_blur_->GetKernelGridHalfSize());
      }

//    LOG(INFO) << "range_point_boundbox    min_bound : " << range_point_boundbox.get_min_bound().transpose()
//              << "  max_bound : " << range_point_boundbox.get_max_bound().transpose();
      if(!this->UpdateBound(range_point_boundbox)){
        //update map size
        cur_update_index += 3;
        return false;
      }
    }

    Eigen::Vector2d beam_start_point_d(pose_transform * range_data->sensor_origin());
    beam_start_point_d.array() += (0.5);
    Eigen::Vector2i beam_start_point_i(beam_start_point_d.cast<int>());

    int valid_range_points = range_data->GetSize();

    for (int i = 0; i < valid_range_points; ++i) {
      Eigen::Vector2d beam_end_point_d(range_data->GetDataPoint(i));
      beam_end_point_d.array() += (0.5);
      Eigen::Vector2i beam_end_point_i(beam_end_point_d.cast<int>());

      if (beam_start_point_i != beam_end_point_i){
        if(!just_update_occu_){
          this->line_visitor_->ErgodLineBresenhami(beam_start_point_i, beam_end_point_i);
        }

        if(use_blur){
          this->CellUpdate(beam_end_point_i[0], beam_end_point_i[1], SET_CELL_OCCUPIED_BLUR);
        }else {
          this->CellUpdate(beam_end_point_i[0], beam_end_point_i[1], SET_CELL_OCCUPIED);
        }
      }
    }

    this->SetUpdated();
    cur_update_index += 3;

    return true;
  }

  double MapFeedbackResponsePenalty(std::shared_ptr<RangeDataContainer2d> range_data_raw ,
                                    const Eigen::Vector3d& best_pose,
                                    int check_point_num, double bound_tolerance, double penalty_gain,
                                    bool use_blur){
    double response_coeff = 1.0;

    if(bound_tolerance < 0 ||
       check_point_num <= 0 ||
       penalty_gain <= 0.0 || penalty_gain >= 1.0){
      return 1.0;
    }

    use_blur_ = use_blur;
    bound_tolerance_ = bound_tolerance;

    std::unique_ptr<LineVisitor> line_visitor = std::make_unique<LineVisitor>
                                   (std::bind(&OccuGridMap::CheckOccuLineVisitorCallback, this,
                                              std::placeholders::_1,
                                              std::placeholders::_2));

    Eigen::Vector3d best_pose_in_map(this->GetMapCoordsPose(best_pose));
    if(!this->PointInMap(best_pose_in_map.x(), best_pose_in_map.y())){
      return 0.0;
    }
    Eigen::Affine2d pose_transform((Eigen::Translation2d(best_pose_in_map[0], best_pose_in_map[1]) *
                                    Eigen::Rotation2Dd(best_pose_in_map[2])));

    Eigen::Vector2d beam_start_point_d(pose_transform * range_data_raw->sensor_origin());
    beam_start_point_d.array() += (0.5);
    Eigen::Vector2i beam_start_point_i(beam_start_point_d.cast<int>());

    int all_point_num = range_data_raw->GetSize();
    int point_step_num = 1;
    if(all_point_num < 2 * check_point_num){
      check_point_num = all_point_num;
      point_step_num = 1;
    }else{
      point_step_num = all_point_num / (check_point_num - 1);
    }

    double penalty = 0;
    for (int point_index = 0; point_index < all_point_num; point_index += point_step_num) {

      Eigen::Vector2d beam_end_point_d(range_data_raw->GetDataPoint(point_index));
      beam_end_point_d = pose_transform * beam_end_point_d;
      beam_end_point_d.array() += (0.5);
      Eigen::Vector2i beam_end_point_i(beam_end_point_d.cast<int>());

      if (beam_start_point_i == beam_end_point_i || !this->PointInMap(beam_end_point_i.x(), beam_end_point_i.y())){
        continue;
      }else{
        current_end_point_ = beam_end_point_i;
        penalty += line_visitor->ErgodLineBresenhami(beam_start_point_i, beam_end_point_i);
      }
    }

//    LOG(INFO) << "penalty " << penalty;
    penalty *= penalty_gain;
    response_coeff = std::max((1.0 + 2 * penalty_gain - penalty), 0.1);
//    LOG(INFO) << "response_coeff " << response_coeff;
    return response_coeff;
  }


  inline double GetGridProbValue(int x, int y){
    return cell_func_.GetGridProbability(this->GetCell(x, y));
  }

  inline double GetGridProbValue(int index){
    return cell_func_.GetGridProbability(this->GetCell(index));
  }

  inline GridStates GetGridStates(int x, int y){
    CellType& cell = this->GetCell(x, y);
    return cell_func_.GetGridStates(cell);
  }

  inline GridStates GetGridStates(int index){
    CellType& cell = this->GetCell(index);
    return cell_func_.GetGridStates(cell);
  }

  inline void SetUpdateFreeFactor(float factor) {
    cell_func_.SetUpdateFreeFactor(factor);
  }

  inline void SetUpdateOccupiedFactor(float factor) {
    cell_func_.SetUpdateOccupiedFactor(factor);
  }

  inline void SetOccuThreshold(float value) {
    cell_func_.SetOccuThreshold(value);
  }

  inline void SetMinPassThrough(float value) {
    cell_func_.SetMinPassThrough(value);
  }

  inline void set_use_auto_map_resize(bool use_auto_map_resize) {
    use_auto_map_resize_ = use_auto_map_resize;
  }

  inline void set_just_update_occu(bool just_update_occu) {
    just_update_occu_ = just_update_occu;
  }

  inline void set_cell_occu_prob_offset(double cell_occu_prob_offset){
    cell_occu_prob_offset_ = cell_occu_prob_offset;
  }

 private:

  void BeamLineVisitorCallback(const Eigen::Vector2i& point, double& reserve){
    this->CellUpdate(point.x(), point.y(), SET_CELL_FREE);
  }

  void CheckOccuLineVisitorCallback(const Eigen::Vector2i& point, double& result){
    double penalty_sum = 0.0;
    CellType& cell = this->GetCell(point.x(), point.y());

    if(use_blur_){
      if(cell.GetValue() > cell_occu_prob_offset_){
        double distance = util::EuclideanDistance2d(current_end_point_.cast<double>(), point.cast<double>());
        if(distance > bound_tolerance_){
          penalty_sum += 1.0;
        }
      }
    }else{
      if(cell_func_.GetGridStates(cell) == GridStates_Occupied){
        double distance = util::EuclideanDistance2d(current_end_point_.cast<double>(), point.cast<double>());
        if(distance > bound_tolerance_){
          penalty_sum += 1.0;
        }
      }
    }

    if(result < 1.0){
      result += penalty_sum;
    }

  }


  inline int CellUpdate(int x, int y,
                        CellUpdateType cell_update_type){
    if(!this->PointInMap(x, y, half_kernel_size_ + 1)){
      return 0;
    }

//    CellType& cell = this->GetCell(x, y);
    switch (cell_update_type){
      case SET_CELL_FREE:
        SetCellFree(x, y);
        break;
      case SET_CELL_OCCUPIED:
        SetCellOccu(x, y);
        break;
      case SET_CELL_OCCUPIED_BLUR:
        SetCellOccuBlur(x, y);
        break;
      default:
        break;
    }

    return 1;

  }

  inline void SetCellFree(int x, int y)
  {
    CellType& cell = this->GetCell(x, y);

    if (cell.update_index_ < cur_mark_free_index) {
      cell_func_.UpdateSetFree(cell);
      cell.update_index_ = cur_mark_free_index;
    }
//        cell_func_.UpdateSetFree(cell);

    map_update_point_.push_back(this->GetGridIndex(x, y));
  }

  inline void SetCellOccu(int x, int y)
  {
    CellType& cell = this->GetCell(x, y);

    if (cell.update_index_ < cur_mark_occu_index) {

      if (cell.update_index_ == cur_mark_free_index) {
        cell_func_.UpdateUnsetFree(cell);
      }

      cell_func_.UpdateSetOccupied(cell);
      cell.update_index_ = cur_mark_occu_index;
    }

//        cell_func_.UpdateSetOccupied(cell);

    map_update_point_.push_back(this->GetGridIndex(x, y));
  }

  inline void SetCellOccuBlur(int x, int y){
    CellType& center_cell = this->GetCell(x, y);

    if (center_cell.update_index_ < cur_mark_occu_index) {

      if(!just_update_occu_){
        if (center_cell.update_index_ == cur_mark_free_index) {
          cell_func_.UpdateUnsetFree(center_cell);
        }

        cell_func_.UpdateSetOccupied(center_cell);
        center_cell.update_index_ = cur_mark_occu_index;
      }else{
        cell_func_.SetGridProbability(center_cell, 1.0f);
//        center_cell.update_index_ = cur_mark_occu_index;
      }


//      float center_cell_prob = cell_func_.GetGridProbability(center_cell);
//      LOG(INFO) << "center_cell_prob "<< center_cell_prob;

      int half_kernel_size = gaussian_blur_->GetKernelGridHalfSize();
      int kernel_size = gaussian_blur_->GetKernelGridSize();
      int grid_index_x = x;
      int grid_index_y = y;

      double* kernel_value = gaussian_blur_->GetKernelValuePointer();
      int kernel_index = 0;

      for(int j = -half_kernel_size; j <= half_kernel_size; ++j){
        for(int i = -half_kernel_size;  i <= half_kernel_size; ++i ){

          int kernel_index = (i + half_kernel_size) + kernel_size * (j + half_kernel_size);

          CellType& cell = this->GetCell(x + i, y + j);
//              cell_func_.UpdateSetOccupiedWithFactor(cell, kernel_value[kernel_index]);
          cell_func_.SetGridProbability(cell, kernel_value[kernel_index] * cell_occu_prob_offset_);
//          cell.update_index_ = cur_mark_occu_index;
//          LOG(INFO) << "cell_value " << cell.GetValue();

          map_update_point_.push_back(this->GetGridIndex(x + i, y + j));
        }
      }

    }
  }


 private:

  CellFunctionsType cell_func_;

  std::unique_ptr<GaussianBlur> gaussian_blur_;
  int half_kernel_size_;
  bool use_blur_;

  std::unique_ptr<LineVisitor> line_visitor_;

  std::vector<int> map_update_point_;

  double bound_tolerance_;
  Eigen::Vector2i current_end_point_;

  bool use_auto_map_resize_;
  bool just_update_occu_;

  int cur_update_index;
  int cur_mark_occu_index;
  int cur_mark_free_index;

  double cell_occu_prob_offset_ = 0.72;

};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_MAP_OCCU_GRID_MAP_H
