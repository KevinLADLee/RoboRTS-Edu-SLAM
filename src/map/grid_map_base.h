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

#ifndef ROBORTS_SLAM_MAP_GRID_MAP_BASE_H
#define ROBORTS_SLAM_MAP_GRID_MAP_BASE_H


#include <map>
#include <utility>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "grid_map_cell.h"
#include "util/slam_util.h"

#define OMP_NUM_THREADS 4

namespace roborts_slam{

enum MapExtendType{
  EXTEND_INTEGRALLY = 0,
  EXTEND_PARTLY,
};

template<typename CellType>
class GridMapBase{

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  GridMapBase(double resolution, const Eigen::Vector2i& size, const Eigen::Vector2d& offset,
              float default_cell_prob = kDefaultCellProb):
      grid_cell_(nullptr),
      scale_factor_(1.0 / resolution),
      map_offset_(offset),
      grid_cell_num_(0),
      map_update_index_(-1),
      extend_factor_(1),
      default_cell_prob_(default_cell_prob){

    this->Resize(size);
    size_x_ = map_size_[0];

    this->SetMapTransform();
  }

  virtual ~GridMapBase()
  {
    DeleteGridCell();
  }

  void SetMapTransform(void){
    world_to_map_ = Eigen::AlignedScaling2d(scale_factor_, scale_factor_) * Eigen::Translation2d(map_offset_[0], map_offset_[1]);
    map_to_world_ = world_to_map_.inverse();
  }

  inline Eigen::Vector2d GetWorldCoords(const Eigen::Vector2d& map_coords) const
  {
    return map_to_world_ * map_coords;
  }

  inline Eigen::Vector2d GetMapCoords(const Eigen::Vector2d& world_coords) const
  {
    return world_to_map_ * world_coords;
  }

  inline Eigen::Vector3d GetWorldCoordsPose(const Eigen::Vector3d& pose_in_map) const
  {
    Eigen::Vector2d pose_in_world(map_to_world_ * pose_in_map.head<2>());
    return Eigen::Vector3d(pose_in_world[0], pose_in_world[1], pose_in_map[2]);
  }

  inline Eigen::Vector3d GetMapCoordsPose(const Eigen::Vector3d& pose_in_world) const
  {
    Eigen::Vector2d pose_in_map(world_to_map_ * pose_in_world.head<2>());
    return Eigen::Vector3d(pose_in_map[0], pose_in_map[1], pose_in_world[2]);
  }

  void Reset(void){
//    TIMER_START(GRID_MAP_BASE_RESET);
//    LOG(WARNING) << "Reset num: " << grid_cell_num_;
//    #pragma omp parallel for
    for(int i = 0; i < grid_cell_num_; ++i){
      grid_cell_[i].ResetGridCell(default_cell_prob_);
    }
//    TIMER_END(GRID_MAP_BASE_RESET);
  }

  void ResetThread (std::vector<int> reset_points) {

    int reset_point_size = reset_points.size();
    for(int i = 0; i < reset_point_size; ++i){
      grid_cell_[reset_points[i]].ResetGridCell(default_cell_prob_);
    }

  }

  void ResetValueSpeedup(const std::vector<int>& reset_points){

    int reset_point_size = reset_points.size();
    for(int i = 0; i < reset_point_size; ++i){
      grid_cell_[reset_points[i]].ResetGridCell(default_cell_prob_);
    }

//    const int kThreadNum = 4;
//    int id_step = reset_point_size / kThreadNum;
//    std::vector<std::thread> t(kThreadNum);
//
//    for (int i = 0 ; i < kThreadNum ; i++){
//
//      int start_id = i * id_step;
//      int end_id = start_id + id_step;
//      if(i == kThreadNum - 1){
//        end_id = reset_point_size;
//      }
//
//      std::vector<int>::const_iterator start_iter = reset_points.begin() + start_id;
//      std::vector<int>::const_iterator end_iter = reset_points.begin() + end_id;
//      std::vector<int> cut_vector(start_iter, end_iter);
//
//      t[i] = std::thread(std::bind(&GridMapBase::ResetThread, this, cut_vector)) ;
//    }
//
//    for (int i = 0 ; i < kThreadNum ; i++){
//      t[i].join();
//    }

//    LOG(INFO) << "grid_cell_num_ " << grid_cell_num_
//              << "    reset_point_size " << reset_point_size;
  }

  void DeleteGridCell()
  {
    if (grid_cell_ != nullptr){

      delete[] grid_cell_;
      grid_cell_ = nullptr;

      map_size_.array() = -1;
      grid_cell_num_ = 0;
    }
  }

  void AllocateGridCell(const Eigen::Vector2i& new_size)
  {
    DeleteGridCell();

    grid_cell_num_ = new_size.x() * new_size.y();
    if(grid_cell_num_ > 0){
      grid_cell_ = new CellType[grid_cell_num_]{default_cell_prob_};
      grid_cell_reset_ = new CellType[grid_cell_num_]{default_cell_prob_};
      CHECK(grid_cell_ != nullptr && grid_cell_reset_ != nullptr)  << "allocate map failed !";
    }else{
      LOG(WARNING) << "invalid init map size !";
    }

    map_size_ = new_size;
  }

  void Resize(const Eigen::Vector2i& new_size){
    AllocateGridCell(new_size);
//      Reset();
  }

  void set_extend_factor(double extend_factor){
    if(extend_factor > 0){
      extend_factor_ = extend_factor;
    }
  }


  void ExtendSize(MapExtendType extend_type){

    BoundBox2d tmp_bound_box;
    tmp_bound_box.AddBoundBox(bound_box_);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = map_size_.cast<double>();
    tmp_bound_box.AddPoint(map_min);
    tmp_bound_box.AddPoint(map_max);

    Eigen::Vector2d min_bound = tmp_bound_box.get_min_bound();
    Eigen::Vector2d max_bound = tmp_bound_box.get_max_bound();

    if(extend_type == EXTEND_INTEGRALLY){

      min_bound -= (tmp_bound_box.GetBoxSize()).cast<double>() * extend_factor_;
      max_bound += (tmp_bound_box.GetBoxSize()).cast<double>() * extend_factor_;

    }else if(extend_type == EXTEND_PARTLY){

      if(bound_box_.get_min_bound().x() <= map_min.x()){
        min_bound.x() -= (tmp_bound_box.GetBoxSize()).cast<double>().x() * extend_factor_;
      }
      if(bound_box_.get_min_bound().y() <= map_min.y()){
        min_bound.y() -= (tmp_bound_box.GetBoxSize()).cast<double>().y() * extend_factor_;
      }
      if(bound_box_.get_max_bound().x() >= map_max.x()){
        max_bound.x() += (tmp_bound_box.GetBoxSize()).cast<double>().x() * extend_factor_;
      }
      if(bound_box_.get_max_bound().y() >= map_max.y()){
        max_bound.y() += (tmp_bound_box.GetBoxSize()).cast<double>().y() * extend_factor_;
      }
    }

    tmp_bound_box.AddPoint(min_bound);
    tmp_bound_box.AddPoint(max_bound);

    map_offset_ -= tmp_bound_box.GetFloorMin() / scale_factor_;

    Eigen::Vector2i pre_grid_offset = (-tmp_bound_box.GetFloorMin().cast<int>());
    int pre_size_x = size_x_;
    int pre_size_y = map_size_.y();

    Eigen::Vector2i map_size_tmp = tmp_bound_box.GetBoxSize();
    size_x_ = map_size_tmp.x();
    grid_cell_num_ = map_size_tmp.x() * map_size_tmp.y();

    CellType* grid_cell_tmp = new CellType[grid_cell_num_]{default_cell_prob_};

    CellType* cell_pointer_tmp = grid_cell_tmp + (pre_grid_offset.y() * size_x_ + pre_grid_offset.x());
    for(int row = 0; row < pre_size_y; ++row){
      memcpy(cell_pointer_tmp, grid_cell_ + pre_size_x * row, pre_size_x * sizeof(CellType));
      cell_pointer_tmp += size_x_;
    }

    map_size_ = map_size_tmp;

    delete[] grid_cell_;
    grid_cell_ = grid_cell_tmp;
    grid_cell_tmp = nullptr;
    grid_cell_reset_ = new CellType[grid_cell_num_]{default_cell_prob_};

    Eigen::Vector2d new_bound_box_min = bound_box_.get_min_bound() - tmp_bound_box.get_min_bound();
    Eigen::Vector2d new_bound_box_max = bound_box_.get_max_bound() - tmp_bound_box.get_min_bound();
    bound_box_.ResetWithBound(new_bound_box_min, new_bound_box_max);

    this->SetMapTransform();
  }


  bool UpdateBound(const BoundBox2d& bound_box){
    if(bound_box_.IsInBounds(bound_box.get_min_bound()) &&
        bound_box_.IsInBounds(bound_box.get_max_bound())){
//      LOG(INFO) << "Do not need to resize ! ";
      return true;
    }

    bound_box_.AddBoundBox(bound_box);
    if(!PointInMap(bound_box_.get_min_bound()) || !PointInMap(bound_box_.get_max_bound()) ){

      ExtendSize(EXTEND_PARTLY);

      LOG(INFO) << "Map Resize ! /************************************/";
      return false;
    }

    return true;
  }

  void UpdateBoundAdaptMap(void) {
    Eigen::Vector2d min_bound(0.0, 0.0);
    Eigen::Vector2d max_bound(static_cast<double>(map_size_[0] + 1),
                              static_cast<double>(map_size_[1] + 1));

    bound_box_.set_min_bound(min_bound);
    bound_box_.set_max_bound(max_bound);
  }

  void set_map_offset(const Eigen::Vector2d& map_offset) {
    map_offset_ = map_offset;

    this->SetMapTransform();
  }

  void set_default_cell_prob(float value){
    if(value >= 0.0f && value <= 1.0f){
      default_cell_prob_ = value;
    }
  }

  const double get_scale_factor() const{
    return scale_factor_;
  }

  void set_scale_factor(float factor){
    scale_factor_ = factor;

    this->SetMapTransform();
  }

  const double GetCellLength(void){
    return (1 / scale_factor_);
  }

  int GetSizeX() const { return map_size_[0]; }
  int GetSizeY() const { return map_size_[1]; }
  int GetGridCellNum() const { return grid_cell_num_; }

  int GetGridIndex(int x, int y){
    return (y * size_x_ + x);
  }

  int GetBoundSizeX() const { return bound_box_.GetSizeX(); }
  int GetBoundSizeY() const { return bound_box_.GetSizeY(); }

  Eigen::Vector2d GetStartGrid() const {
    return (Eigen::Vector2d(std::floor(bound_box_.get_min_bound()[0]), std::floor(bound_box_.get_min_bound()[1])));
  }

  Eigen::Vector2d GetEndGrid() const {
    return (Eigen::Vector2d(std::ceil(bound_box_.get_max_bound()[0]), std::ceil(bound_box_.get_max_bound()[1])));
  }

  bool PointInMap(const Eigen::Vector2d& point_map) const{
    // return bound_box_.IsInBounds(point_map);
    if(point_map[0] > 0 && point_map[0] < map_size_[0] &&
        point_map[1] > 0 && point_map[1] < map_size_[1]){
      return true;
    }
    return false;
  }

  bool PointInMap(double x, double y, double bound_tolerance = 0.0) const{
    // return bound_box_.IsInBounds(point_map);
    if(x > bound_tolerance && x < map_size_[0] - bound_tolerance &&
        y > bound_tolerance && y < map_size_[1] - bound_tolerance){
      return true;
    }
    return false;
  }

  bool PointInMap(int index) const {
    return (index > 0 && index < grid_cell_num_);
  }

  CellType& GetCell(int x, int y){
    return grid_cell_[y * size_x_ + x];
  }

  CellType& GetCell(int index)
  {
    return grid_cell_[index];
  }

  float GetCellValue(int x, int y){
    return grid_cell_[y * size_x_ + x].GetValue();
  }

  float GetCellValue(int index){
    return grid_cell_[index].GetValue();
  }

  void SetUpdated() { map_update_index_++; }

  int map_update_index() const { return map_update_index_; }

  bool IsMapInit(void){
    if(map_update_index_ < 0){
      return false;
    }
    return true;
  }


 private:

  CellType* grid_cell_;
  CellType* grid_cell_reset_;

  double scale_factor_;
  Eigen::Vector2i map_size_;
  Eigen::Vector2d map_offset_;
  BoundBox2d bound_box_;
  int grid_cell_num_;
  int size_x_;

  double extend_factor_;
  float default_cell_prob_;

  Eigen::Affine2d map_to_world_;
  Eigen::Affine2d world_to_map_;

  int map_update_index_;
};


} // namespace roborts_slam


#endif // ROBORTS_SLAM_MAP_GRID_MAP_BASE_H