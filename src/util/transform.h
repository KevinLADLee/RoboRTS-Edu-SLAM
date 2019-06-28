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

#ifndef ROBORTS_SLAM_UTIL_TRANSFORM_H
#define ROBORTS_SLAM_UTIL_TRANSFORM_H

#include <Eigen/Dense>
#include "util/slam_util.h"
#include <iostream>


namespace roborts_slam {

class Translation2d {
 public:
  Translation2d(const Pose2d &pose_from, const Pose2d &pose_to){
    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
    delta_pose_ = (pose_to - pose_from);
    T.pretranslate(Eigen::Vector2d(delta_pose_(0), delta_pose_(1)));
    T_ = T;
  };

  Pose2d Transform(const Pose2d &pose){
    Eigen::Vector2d pose_vec;
    pose_vec << pose(0), pose(1);
    auto pose_new = T_ * pose_vec;
    Pose2d pose2d_new;
    pose2d_new << pose_new(0), pose_new(1), 0;
    pose2d_new(2) = pose(2) + delta_pose_(2);
    return pose2d_new;
  }

 private:
  Pose2d delta_pose_;
  Eigen::Isometry2d T_;
};

class Transform2d{
 public:
  Transform2d(const Pose2d &pose_1, const Pose2d &pose_2) {
    SetTranform(pose_1, pose_2);
  };

  Pose2d Transform(const Pose2d &pose){
    Pose2d new_position = transform_ + rotation_ * pose;
    double angle = util::NormalizeAngle(pose(2) + transform_(2));
    return Pose2d(new_position(0), new_position(1), angle);
  };

  Pose2d InverseTransformPose(const Pose2d &pose){
    Pose2d new_position = inverse_rotation_ * (pose - transform_);
    double angle = util::NormalizeAngle(pose(2) - transform_(2));
    return Pose2d(new_position(0), new_position(1), angle);
  };

 private:
  void SetTranform(const Pose2d &pose_1, const Pose2d &pose_2){
    if (pose_1 == pose_2)
    {
      rotation_.setIdentity();
      transform_ = Pose2d(0,0,0);
      return;
    }

    auto rotate = Eigen::AngleAxisd(pose_2(2) - pose_1(2), Eigen::Vector3d(0,0,1));
    rotation_ = rotate.toRotationMatrix();

    inverse_rotation_ = Eigen::AngleAxisd(pose_1(2) - pose_2(2), Eigen::Vector3d(0,0,1)).toRotationMatrix();

    Pose2d new_position;
    if (pose_1(0) != 0.0 || pose_1(1) != 0.0)
    {
      new_position = pose_2 - rotation_ * pose_1;
    }
    else
    {
      new_position = pose_2;
    }

    transform_ = Pose2d(new_position(0), new_position(1), pose_2(2)-pose_1(2));
  }


 private:
  Eigen::Matrix3d rotation_;
  Eigen::Matrix3d inverse_rotation_;
  Pose2d transform_;

};

class TransformByMidFrame{
 public:
  TransformByMidFrame(const Pose2d &target_frame_in_mid_frame, const Pose2d &source_frame_in_mid_frame){
    source_frame_to_mid_ =  std::make_unique<Transform2d>(Pose2d(0,0,0), source_frame_in_mid_frame);
    mid_frame_to_target_ = std::make_unique<Transform2d>(target_frame_in_mid_frame, Pose2d(0,0,0));
  };

  inline Pose2d Transform(const Pose2d &pose_in_source_frame){
    return mid_frame_to_target_->Transform(source_frame_to_mid_->Transform(pose_in_source_frame));
  };

 private:
  std::unique_ptr<Transform2d> mid_frame_to_target_ = nullptr;
  std::unique_ptr<Transform2d> source_frame_to_mid_ = nullptr;
};

} // namespace roborts_slam


#endif // ROBORTS_SLAM_UTIL_TRANSFORM_H
