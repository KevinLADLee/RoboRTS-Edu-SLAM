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

#ifndef ROBORTS_SLAM_UTIL_BOUNDBOX_H
#define ROBORTS_SLAM_UTIL_BOUNDBOX_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <limits>
#include <cmath>
#include <float.h>

#include <Eigen/Core>

namespace roborts_slam {

template<typename T>
class BoundBox{
 public:

  const T value_max = static_cast<T>(FLT_MAX);//std::numeric_limits<T>::max();
  const T value_min = static_cast<T>(FLT_MIN);//std::numeric_limits<T>::min();

  BoundBox():max_bound_(value_min,value_min),
             min_bound_(value_max,value_max){}

  BoundBox(const Eigen::Matrix<T,2,1>& min_bound, const Eigen::Matrix<T,2,1>& max_bound) :
      min_bound_(min_bound),
      max_bound_(max_bound){}

  BoundBox(const BoundBox& other_box) {
    this->min_bound_ = other_box.min_bound_;
    this->max_bound_ = other_box.max_bound_;
  }

  BoundBox& operator = (const BoundBox& other_box){
    this->min_bound_ = other_box.min_bound_;
    this->max_bound_ = other_box.max_bound_;
  }

  inline void Reset(void){
    min_bound_ = Eigen::Matrix<T,2,1>(value_max,value_max);
    max_bound_ = Eigen::Matrix<T,2,1>(value_min,value_min);
  }

  inline void ResetWithBound(const Eigen::Matrix<T,2,1>& min_bound, const Eigen::Matrix<T,2,1>& max_bound){
    min_bound_ = min_bound;
    max_bound_ = max_bound;
  }

  inline const Eigen::Matrix<T,2,1> get_min_bound(void) const{
    return min_bound_;
  }

  inline void set_min_bound(const Eigen::Matrix<T,2,1>& min_bound){
    min_bound_ = min_bound;
  }

  inline const Eigen::Matrix<T,2,1> get_max_bound(void) const{
    return max_bound_;
  }

  inline void set_max_bound(const Eigen::Matrix<T,2,1>& max_bound){
    max_bound_ = max_bound;
  }

  inline const Eigen::Matrix<T,2,1> GetFloorMin(void) const{
    return Eigen::Matrix<T,2,1>(std::floor(min_bound_[0]), std::floor(min_bound_[1]));
  }

  inline const Eigen::Matrix<T,2,1> GetCeilMax(void) const{
    return Eigen::Matrix<T,2,1>(std::ceil(max_bound_[0]), std::ceil(max_bound_[1]));
  }

  inline Eigen::Vector2i GetBoxSize() const
  {
    int box_x = std::ceil(max_bound_[0]) - std::floor(min_bound_[0]);
    int box_y = std::ceil(max_bound_[1]) - std::floor(min_bound_[1]);
    return Eigen::Vector2i(box_x, box_y);
  }

  inline int GetSizeX() const{
    if(std::ceil(max_bound_[0]) - std::floor(min_bound_[0]) < 0){
      return 0;
    }
    return static_cast<int>(std::ceil(max_bound_[0]) - std::floor(min_bound_[0]) + 1);
  }

  inline int GetSizeY() const{
    if(std::ceil(max_bound_[1]) - std::floor(min_bound_[1]) < 0){
      return 0;
    }
    return static_cast<int>(std::ceil(max_bound_[1]) - std::floor(min_bound_[1]) + 1);
  }

  inline void AddPoint(const Eigen::Matrix<T,2,1>& point)
  {
    if(point[0] < min_bound_[0]){min_bound_[0] = point[0];}
    if(point[1] < min_bound_[1]){min_bound_[1] = point[1];}
    if(point[0] > max_bound_[0]){max_bound_[0] = point[0];}
    if(point[1] > max_bound_[1]){max_bound_[1] = point[1];}
  }

  inline void AddBoundBox(const BoundBox& box)
  {
    AddPoint(box.get_min_bound());
    AddPoint(box.get_max_bound());
  }

  inline void ExtendBoundBox(T extend_size){
    min_bound_.array() -= extend_size;
    max_bound_.array() += extend_size;
  }

  inline bool IsInBounds(const Eigen::Matrix<T,2,1>& point) const
  {
    return (point[0] > min_bound_[0] && point[0] < max_bound_[0] &&
        point[1] > min_bound_[1] && point[1] < max_bound_[1]);
  }


 private:
  Eigen::Matrix<T,2,1> min_bound_;
  Eigen::Matrix<T,2,1> max_bound_;

};

using BoundBox2f = BoundBox<float>;
using BoundBox2d = BoundBox<double>;


} // namespace roborts_slam

#endif // ROBORTS_SLAM_UTIL_BOUNDBOX_H
