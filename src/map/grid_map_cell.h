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

#ifndef ROBORTS_SLAM_MAP_GRID_MAP_CELL_H
#define ROBORTS_SLAM_MAP_GRID_MAP_CELL_H

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>



namespace roborts_slam{

const float kDefaultCellProb = 0.5f;

typedef enum
{
  GridStates_Unknown = -1,
  GridStates_Occupied = 100,
  GridStates_Free = 0
} GridStates;



const float kUnknownGridProb = 0.5f;
class CountCell{
 public:
  CountCell(float val = kDefaultCellProb){
    ResetGridCell(val);
  }

  void SetValue(float val) {
    prob_value_ = val;
  }

  float GetValue() const {
    return prob_value_;
  }

  bool IsOccupied() const {
    return prob_value_ > 0.5f;
  }

  bool IsFree() const{
    return prob_value_ < 0.5f;
  }

  void ResetGridCell(float val) {
    prob_value_ = val;
    pass_count_ = 0.0f;
    hit_count_ = 0.0f;
    update_index_ = -1;
  }

  float pass_count(void) {
    return pass_count_;
  }

 public:
  float pass_count_;
  float hit_count_;
  float prob_value_;
  int update_index_;
};

class CountCellFunctions
{
 public:

  CountCellFunctions(){
    this->SetUpdateFreeFactor(0.0f);
    this->SetUpdateOccupiedFactor(0.0f);

    this->SetOccuThreshold(0.5f);
    this->SetMinPassThrough(2.0f);
  }

  void UpdateSetOccupied(CountCell& cell) const {
    cell.hit_count_ += (1.0f + update_occu_factor_);
    cell.pass_count_ += (1.0f + update_free_factor_);
    cell.prob_value_ = cell.hit_count_ / cell.pass_count_;
    if(cell.prob_value_ > 1.0f){
      cell.prob_value_ = 1.0f;
    }
  }

  void UpdateSetFree(CountCell& cell) const {
    cell.pass_count_ += (1.0f + update_free_factor_);
    cell.prob_value_ = cell.hit_count_ / cell.pass_count_;
  }

  void UpdateUnsetFree(CountCell& cell) const {
    cell.pass_count_ -= (1.0f + update_free_factor_);
    cell.prob_value_ = cell.hit_count_ / cell.pass_count_;
  }

  float GetGridProbability(const CountCell& cell) const {
    return cell.prob_value_;
  }

  void SetGridProbability(CountCell& cell, float prob) {
    if(cell.GetValue() < prob){
      cell.SetValue(prob);

      cell.hit_count_ = prob * cell.pass_count_;
    }
  }

  inline GridStates GetGridStates(const CountCell& cell) const{

    if(cell.pass_count_ >= min_pass_through_){
      if(cell.GetValue() < occu_threshold_){
        return GridStates_Free;
      }else{
        return GridStates_Occupied;
      }
    }

    return GridStates_Unknown;
  }

  void SetUpdateFreeFactor(float factor){
    update_free_factor_ = factor;
  }

  void SetUpdateOccupiedFactor(float factor){
    update_occu_factor_ = factor;
  }

  void SetOccuThreshold(float value) {
    occu_threshold_ = value;
  }

  void SetMinPassThrough(float value) {
    min_pass_through_ = value;
  }

 private:
  float update_occu_factor_;
  float update_free_factor_;

  float occu_threshold_;
  float min_pass_through_;

};




class LogOddsCell
{
 public:
  LogOddsCell(float val = kDefaultCellProb){
    ResetGridCell(val);
  }

  void SetValue(float val)
  {
    logodds_val_ = val;
  }

  float GetValue() const
  {
    return logodds_val_;
  }

  bool IsOccupied() const
  {
    return logodds_val_ > 0.0f;
  }

  bool IsFree() const
  {
    return logodds_val_ < 0.0f;
  }

  void ResetGridCell(float val)
  {
    logodds_val_ = 0.0f;
    prob_value_ = val;
    update_index_ = -1;
  }

 public:

  float logodds_val_;
  float prob_value_;
  int update_index_;
};

class LogOddsCellFunctions
{
 public:

  LogOddsCellFunctions() {
    this->SetUpdateFreeFactor(0.3f);
    this->SetUpdateOccupiedFactor(0.9f);
  }

  void UpdateSetOccupied(LogOddsCell& cell) const {
    if (cell.logodds_val_ < 50.0f){
      cell.logodds_val_ += logodds_occu_;

      float odds = exp(cell.logodds_val_);
      cell.prob_value_ = odds / (odds + 1.0f);
    }

  }

  void UpdateSetOccupiedWithFactor(LogOddsCell& cell, double occu_factor) const {
    if (cell.logodds_val_ < 50.0f && occu_factor <= 1.0){
      cell.logodds_val_ += logodds_occu_ * occu_factor;

      float odds = exp(cell.logodds_val_);
      cell.prob_value_ = odds / (odds + 1.0f);
    }
  }

  void UpdateSetFree(LogOddsCell& cell) const {
    cell.logodds_val_ += logodds_free_;

    float odds = exp(cell.logodds_val_);
    cell.prob_value_ = odds / (odds + 1.0f);
  }

  void UpdateUnsetFree(LogOddsCell& cell) const {
    cell.logodds_val_ -= logodds_free_;

    float odds = exp(cell.logodds_val_);
    cell.prob_value_ = odds / (odds + 1.0f);
  }

  float GetGridProbability(const LogOddsCell& cell) const {
//    float odds = exp(cell.logodds_val_);
//    return odds / (odds + 1.0f);
    return cell.prob_value_;

  }

  void SetGridProbability(LogOddsCell& cell, float prob) {
    float logodds_val = ProbToLogOdds(prob);
    if(cell.GetValue() < logodds_val){
      cell.logodds_val_ = logodds_val;

      cell.prob_value_ = prob;
    }
  }

  inline GridStates GetGridStates(const LogOddsCell& cell) const{
    if(cell.IsFree()){
      return GridStates_Free;
    }else if(cell.IsOccupied()){
      return GridStates_Occupied;
    }

    return GridStates_Unknown;
  }

  void SetUpdateFreeFactor(float factor) {
    logodds_free_ = ProbToLogOdds(factor);
  }

  void SetUpdateOccupiedFactor(float factor) {
    logodds_occu_ = ProbToLogOdds(factor);
  }

  void SetOccuThreshold(float value) {}

  void SetMinPassThrough(float value) {}

 protected:

  float ProbToLogOdds(float prob) {
    float odds = prob / (1.0f - prob);
    return log(odds);
  }

  float logodds_occu_;
  float logodds_free_;
};




class ProbabilityCell
{
 public:
  ProbabilityCell(float val = kDefaultCellProb){
    ResetGridCell(val);
  }

  void SetValue(float val)
  {
    prob_value_ = val;
  }

  float GetValue() const
  {
    return prob_value_;
  }

  void ResetGridCell(float val)
  {
    prob_value_ = val;
    update_index_ = -1;
  }

 public:

  float prob_value_;
  int update_index_;
};

class ProbabilityCellFunctions
{
 public:

  ProbabilityCellFunctions() {
    this->SetUpdateFreeFactor(0.2f);
    this->SetUpdateOccupiedFactor(0.5f);
  }

  void UpdateSetOccupied(ProbabilityCell& cell) const {
    float new_prob_value = cell.GetValue() + update_occu_factor_;
    if(new_prob_value > 1.0f) new_prob_value = 1.0f;
    cell.SetValue(new_prob_value);
  }

  void UpdateSetFree(ProbabilityCell& cell) const {
    float new_prob_value = cell.GetValue() - update_free_factor_;
    if(new_prob_value < 0.0f) new_prob_value = 0.0f;
    cell.SetValue(new_prob_value);
  }

  void UpdateUnsetFree(ProbabilityCell& cell) const {
    float new_prob_value = cell.GetValue() + update_free_factor_;
    if(new_prob_value > 1.0f) new_prob_value = 1.0f;
    cell.SetValue(new_prob_value);
  }

  float GetGridProbability(const ProbabilityCell& cell) const {
    return cell.GetValue();
  }

  void SetGridProbability(ProbabilityCell& cell, float prob) {
    if(cell.GetValue() < prob && prob <= 1.0f){
      cell.SetValue(prob);
    }
  }

  inline GridStates GetGridStates(const ProbabilityCell& cell) const{
    if(cell.GetValue() < 0.5f){
      return GridStates_Free;
    }else if(cell.GetValue() > 0.5f){
      return GridStates_Occupied;
    }

    return GridStates_Unknown;
  }

  void SetUpdateFreeFactor(float factor) { update_free_factor_ = factor; }

  void SetUpdateOccupiedFactor(float factor) { update_occu_factor_ = factor; }

  void SetOccuThreshold(float value) {}
  void SetMinPassThrough(float value) {}

 protected:

  float update_occu_factor_;
  float update_free_factor_;
};


} // namespace roborts_slam


#endif // ROBORTS_SLAM_MAP_GRID_MAP_CELL_H