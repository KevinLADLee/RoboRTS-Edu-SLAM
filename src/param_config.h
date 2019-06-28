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

#include <ros/ros.h>

#ifndef ROBORTS_SLAM_PARAM_CONFIG_H
#define ROBORTS_SLAM_PARAM_CONFIG_H

namespace roborts_slam{

class ParamConfig {
 public:
  ParamConfig(const ros::NodeHandle& nh) {
    nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    nh.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    nh.param<std::string>("laser_frame_id", laser_frame_id_, "");
    nh.param<std::string>("global_frame_id", global_frame_id_, "odom");

    nh.param<std::string>("odom_topic_name", odom_topic_name_, "odom");
    nh.param<std::string>("map_topic_name", map_topic_name_, "map");

    nh.param<bool>("use_odom_correct", use_odom_correct_, false);
    nh.param<double>("odom_interpolation_time", odom_interpolation_time_, 0.005);
    nh.param<double>("range_threshold_scale", range_threshold_scale_, 0.95);
    nh.param<bool>("publish_visualize", publish_visualize_, true);

    nh.param<double>("init_map_size", init_map_size_, 5);
    nh.param<double>("map_offset_x", map_offset_x_, 0.5);
    nh.param<double>("map_offset_y", map_offset_y_, 0.5);
    nh.param<double>("bound_tolerance", bound_tolerance_, 1);
    nh.param<double>("map_extend_factor", map_extend_factor_, 0.03);

    nh.param<double>("map_resolution", map_resolution_, 0.05);
    nh.param<double>("map_update_free_factor", map_update_free_factor_, 0.3);
    nh.param<double>("map_update_occu_factor", map_update_occu_factor_, 0.7);
    nh.param<double>("map_occu_threshold", map_occu_threshold_, 0.2);
    nh.param<double>("map_min_passthrough", map_min_passthrough_, 3.0);

    nh.param<double>("coarse_map_resolution", coarse_map_resolution_, 0.1);
    nh.param<double>("coarse_map_deviation", coarse_map_deviation_, 0.4);
    nh.param<bool>("coarse_map_use_blur", coarse_map_use_blur_, true);

    nh.param<double>("fine_map_resolution", fine_map_resolution_, 0.01);
    nh.param<double>("fine_map_deviation", fine_map_deviation_, 0.03);
    nh.param<bool>("fine_map_use_blur", fine_map_use_blur_, true);

    nh.param<double>("gaussian_blur_offset", gaussian_blur_offset_, 0.72);

    nh.param<bool>("use_optimize_scan_match", use_optimize_scan_match_, true);
    nh.param<int>("iterate_times", iterate_times_, 10);
    nh.param<double>("cost_decrease_threshold", cost_decrease_threshold_, 1);
    nh.param<double>("cost_min_threshold", cost_min_threshold_, 2);
    nh.param<double>("max_update_distance", max_update_distance_, 0.5);
    nh.param<double>("max_update_angle", max_update_angle_, 0.2);
    nh.param<double>("optimize_failed_cost", optimize_failed_cost_, 20);

    nh.param<double>("coarse_search_space_size", coarse_search_space_size_, 0.8);
    nh.param<double>("coarse_search_space_resolution", coarse_search_space_resolution_, 0.1);
    nh.param<double>("coarse_search_angle_offset", coarse_search_angle_offset_, 0.01745 * 100);
    nh.param<double>("coarse_search_angle_resolution", coarse_search_angle_resolution_, 0.01745 * 2);
    nh.param<double>("coarse_response_threshold", coarse_response_threshold_, 0.6);
    nh.param<int>("coarse_use_point_size", coarse_use_point_size_, 100);

    nh.param<double>("fine_search_space_size", fine_search_space_size_, 0.2);
    nh.param<double>("fine_search_space_resolution", fine_search_space_resolution_, 0.02);
    nh.param<double>("fine_search_angle_offset", fine_search_angle_offset_, 0.01745 * 20);
    nh.param<double>("fine_search_angle_resolution", fine_search_angle_resolution_, 0.01745 * 2);
    nh.param<double>("fine_response_threshold", fine_response_threshold_, 0.7);
    nh.param<int>("fine_use_point_size", fine_use_point_size_, 100);

    nh.param<double>("super_fine_search_space_size", super_fine_search_space_size_, 0.02);
    nh.param<double>("super_fine_search_space_resolution", super_fine_search_space_resolution_, 0.01);
    nh.param<double>("super_fine_search_angle_offset", super_fine_search_angle_offset_, 0.01745 * 2);
    nh.param<double>("super_fine_search_angle_resolution", super_fine_search_angle_resolution_, 0.01745 * 0.2);
    nh.param<double>("super_fine_response_threshold", super_fine_response_threshold_, 0.7);
    nh.param<int>("super_fine_use_point_size", super_fine_use_point_size_, 200);

    nh.param<bool>("use_odometry", use_odometry_, true);

    nh.param<bool>("use_map_check_feedback", use_map_check_feedback_, true);
    nh.param<int>("map_check_point_num", map_check_point_num_, 50);
    nh.param<double>("map_check_bound_tolerance", map_check_bound_tolerance_, 3.0);
    nh.param<double>("map_check_penalty_gain", map_check_penalty_gain_, 0.05);

    nh.param<bool>("use_map_update_move_check", use_map_update_move_check_, false);
    nh.param<double>("map_update_score_threshold", map_update_score_threshold_, 0.48);
    nh.param<double>("map_update_distance_threshold", map_update_distance_threshold_, 0.1);
    nh.param<double>("map_update_angle_threshold", map_update_angle_threshold_, 0.01745 * 1);

    nh.param<bool>("use_move_check", use_move_check_, false);
    nh.param<double>("move_distance_threshold", move_distance_threshold_, 0.05);
    nh.param<double>("move_angle_threshold", move_angle_threshold_, 0.01745 * 0.5);
    nh.param<double>("move_time_threshold", move_time_threshold_, 5);

    nh.param<double>("move_max_linear_vel", move_max_linear_vel_, 3.0);
    nh.param<double>("move_max_angular_vel", move_max_angular_vel_, 3.0);

    nh.param<double>("running_range_max_distance", running_range_max_distance_, 5.0);
    nh.param<int>("running_range_size", running_range_size_, 70);

    nh.param<int>("loop_match_min_chain_size", loop_match_min_chain_size_, 8);
    nh.param<double>("link_match_min_response", link_match_min_response_, 0.8);
    nh.param<double>("link_scan_max_distance", link_scan_max_distance_, 7.0);
    nh.param<double>("loop_match_min_response_coarse", loop_match_min_response_coarse_, 0.58);
    nh.param<double>("loop_match_max_variance_coarse", loop_match_max_variance_coarse_, 0.4);
    nh.param<double>("loop_match_min_response_fine", loop_match_min_response_fine_, 0.55);

  }

  const std::string odom_frame_id(void) const { return odom_frame_id_; }
  const std::string base_frame_id(void) const { return base_frame_id_; }
  const std::string laser_frame_id(void) const { return laser_frame_id_; }
  const std::string global_frame_id(void) const { return global_frame_id_; }

  const std::string odom_topic_name(void) const { return odom_topic_name_; }
  const std::string map_topic_name(void) const { return map_topic_name_; }

  const bool publish_visualize(void) const { return publish_visualize_; }

  const bool use_odom_correct(void) const { return use_odom_correct_; }
  const double odom_interpolation_time(void) const { return odom_interpolation_time_; }
  const double range_threshold_scale(void) const { return range_threshold_scale_; }


  const double init_map_size(void) const { return init_map_size_; }
  const double map_offset_x(void) const { return map_offset_x_; }
  const double map_offset_y(void) const { return map_offset_y_; }
  const double bound_tolerance(void) const { return bound_tolerance_; }
  const double map_extend_factor(void) const { return map_extend_factor_; }

  const double map_resolution(void) const { return map_resolution_; }
  const double map_update_free_factor(void) const { return map_update_free_factor_; }
  const double map_update_occu_factor(void) const { return map_update_occu_factor_; }
  const double map_occu_threshold(void) const { return map_occu_threshold_; }
  const double map_min_passthrough(void) const { return map_min_passthrough_; }

  const double coarse_map_resolution(void) const { return coarse_map_resolution_; }
  const double coarse_map_deviation(void) const { return coarse_map_deviation_; }
  const bool coarse_map_use_blur(void) const { return coarse_map_use_blur_; }

  const double fine_map_resolution(void) const { return fine_map_resolution_; }
  const double fine_map_deviation(void) const { return fine_map_deviation_; }
  const bool fine_map_use_blur(void) const { return fine_map_use_blur_; }

  const double gaussian_blur_offset(void) const { return gaussian_blur_offset_; }


  const bool use_optimize_scan_match(void) const { return use_optimize_scan_match_; }
  const int iterate_times(void) const { return iterate_times_; }
  const double cost_decrease_threshold(void) const { return cost_decrease_threshold_; }
  const double cost_min_threshold(void) const { return cost_min_threshold_; }
  const double max_update_distance(void) const { return max_update_distance_; }
  const double max_update_angle(void) const { return max_update_angle_; }
  const double optimize_failed_cost(void) const { return optimize_failed_cost_; }

  const double coarse_search_space_size(void) const { return coarse_search_space_size_; }
  const double coarse_search_space_resolution(void) const { return coarse_search_space_resolution_; }
  const double coarse_search_angle_offset(void) const { return coarse_search_angle_offset_; }
  const double coarse_search_angle_resolution(void) const { return coarse_search_angle_resolution_; }
  const double coarse_response_threshold(void) const { return coarse_response_threshold_; }
  const int coarse_use_point_size(void) const { return coarse_use_point_size_; }

  const double fine_search_space_size(void) const { return fine_search_space_size_; }
  const double fine_search_space_resolution(void) const { return fine_search_space_resolution_; }
  const double fine_search_angle_offset(void) const { return fine_search_angle_offset_; }
  const double fine_search_angle_resolution(void) const { return fine_search_angle_resolution_; }
  const double fine_response_threshold(void) const { return fine_response_threshold_; }
  const int fine_use_point_size(void) const { return fine_use_point_size_; }

  const double super_fine_search_space_size(void) const { return super_fine_search_space_size_; }
  const double super_fine_search_space_resolution(void) const { return super_fine_search_space_resolution_; }
  const double super_fine_search_angle_offset(void) const { return super_fine_search_angle_offset_; }
  const double super_fine_search_angle_resolution(void) const { return super_fine_search_angle_resolution_; }
  const double super_fine_response_threshold(void) const { return super_fine_response_threshold_; }
  const int super_fine_use_point_size(void) const { return super_fine_use_point_size_; }


  const bool use_map_check_feedback(void) const { return use_map_check_feedback_; }
  const int map_check_point_num(void) const { return map_check_point_num_; }
  const double map_check_bound_tolerance(void) const { return map_check_bound_tolerance_; }
  const double map_check_penalty_gain(void) const { return map_check_penalty_gain_; }

  const bool use_map_update_move_check(void) const { return use_map_update_move_check_; }
  const double map_update_score_threshold(void) const { return map_update_score_threshold_; }
  const double map_update_distance_threshold(void) const { return map_update_distance_threshold_; }
  const double map_update_angle_threshold(void) const { return map_update_angle_threshold_; }

  const bool use_odometry(void) const { return use_odometry_; }

  const bool use_move_check(void) const { return use_move_check_; }
  const double move_distance_threshold(void) const { return move_distance_threshold_; }
  const double move_angle_threshold(void) const { return move_angle_threshold_; }
  const double move_time_threshold(void) const { return move_time_threshold_; }

  const double move_max_linear_vel(void) const { return move_max_linear_vel_; }
  const double move_max_angular_vel(void) const { return move_max_angular_vel_; }

  const double running_range_max_distance(void) const { return running_range_max_distance_; }
  const int running_range_size(void) const { return running_range_size_; }

  int loop_match_min_chain_size() const {
    return loop_match_min_chain_size_;
  }
  double link_match_min_response() const {
    return link_match_min_response_;
  }
  double link_scan_max_distance() const {
    return link_scan_max_distance_;
  }
  double loop_match_min_response_coarse() const {
    return loop_match_min_response_coarse_;
  }
  double loop_match_max_variance_coarse() const {
    return loop_match_max_variance_coarse_;
  }
  double loop_match_min_response_fine() const {
    return loop_match_min_response_fine_;
  }

 private:

  std::string odom_frame_id_;
  std::string laser_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;

  std::string odom_topic_name_;
  std::string laser_topic_name_;
  std::string map_topic_name_;

  bool publish_visualize_;

  //map
  double init_map_size_     = 5;
  double map_offset_x_      = 0.5;
  double map_offset_y_      = 0.5;
  double bound_tolerance_   = 10;
  double map_extend_factor_ = 0.5;

  double map_resolution_ = 0.05;
  double map_update_free_factor_ = 0.3;
  double map_update_occu_factor_ = 0.7;
  double map_occu_threshold_     = 0.2;
  double map_min_passthrough_    = 3;

  double coarse_map_resolution_ = 0.1;
  double coarse_map_deviation_  = 0.4;
  bool   coarse_map_use_blur_   = true;

  double fine_map_resolution_ = 0.01;
  double fine_map_deviation_  = 0.03;
  bool   fine_map_use_blur_   = true;

  double gaussian_blur_offset_    = 0.7;


  //sensor
  bool use_odom_correct_;
  double odom_interpolation_time_ = 0.005;
  double range_threshold_scale_ = 0.95;


//optimize
  bool   use_optimize_scan_match_ = true;
  int    iterate_times_           = 10;
  double cost_decrease_threshold_ = 1;
  double cost_min_threshold_      = 2;
  double max_update_distance_     = 0.5;
  double max_update_angle_        = 0.2;
  double optimize_failed_cost_    = 20;


//correlattion
  double coarse_search_space_size_    = 0.8;
  double coarse_search_space_resolution_ = 0.1;
  double coarse_search_angle_offset_     = 0.01745 * 80;
  double coarse_search_angle_resolution_ = 0.01745 * 2;
  double coarse_response_threshold_     = 0.6;
  int    coarse_use_point_size_          = 100;

  double fine_search_space_size_       = 0.2;
  double fine_search_space_resolution_ = 0.02;
  double fine_search_angle_offset_     = 0.01745 * 20;
  double fine_search_angle_resolution_ = 0.01745 * 2;
  double fine_response_threshold_     = 0.7;
  int    fine_use_point_size_          = 100;

  double super_fine_search_space_size_       = 0.02;
  double super_fine_search_space_resolution_ = 0.01;
  double super_fine_search_angle_offset_     = 0.01745 * 2;
  double super_fine_search_angle_resolution_ = 0.01745 * 0.2;
  double super_fine_response_threshold_     = 0.7;
  int    super_fine_use_point_size_          = 200;


//slam
  bool   use_odometry_;

  bool   use_map_check_feedback_;
  int    map_check_point_num_;
  double map_check_bound_tolerance_;
  double map_check_penalty_gain_;

  bool   use_map_update_move_check_     = false;
  double map_update_score_threshold_    = 0.48;
  double map_update_distance_threshold_ = 0.1;
  double map_update_angle_threshold_    = 0.01745 * 1;

  bool   use_move_check_          = false;
  double move_distance_threshold_ = 0.05;
  double move_angle_threshold_    = 0.01745 * 0.5;
  double move_time_threshold_     = 5;

  double move_max_linear_vel_     = 3.0;
  double move_max_angular_vel_    = 3.0;

  double running_range_max_distance_ = 5.0;
  int    running_range_size_         = 70;

//Pose Graph and Loop Closure
  int    loop_match_min_chain_size_;
  double link_match_min_response_;
  double link_scan_max_distance_;
  double loop_match_min_response_coarse_;
  double loop_match_max_variance_coarse_;
  double loop_match_min_response_fine_;


};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_PARAM_CONFIG_H

