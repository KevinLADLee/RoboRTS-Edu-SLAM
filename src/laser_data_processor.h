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

#ifndef ROBORTS_SLAM_LASER_DATA_PROCESSOR_H
#define ROBORTS_SLAM_LASER_DATA_PROCESSOR_H



#include "param_config.h"

//#define USE_PCL_VISUALIZATION
#ifdef USE_PCL_VISUALIZATION
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#endif

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

namespace roborts_slam {


const double kCorrectTimeResolution = 0.005; //s

class LaserDataProcessor {
 public:

  LaserDataProcessor(ros::NodeHandle *nh, std::shared_ptr<ParamConfig> param);

  ~LaserDataProcessor() {}

  /**
   * @brief laser data process interface
   * @param laser scan message
   */
  void Process(sensor_msgs::LaserScan &scan_msg);

 private:
  /**
   * @brief Get odometry information of specify timepoint
   */
  bool GetLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt);

  /**
   * @brief Correct distorted laser data by odomtry
   * @param laser point vector and start and end time of this lidar frame
   */
  void DataCorrect(std::vector<double> &ranges,
                   std::vector<double> &angles,
                   ros::Time frame_start_time, ros::Time frame_end_time);

  /**
   * @brief interpolation for every subdivded beams
   */
  void BeamsUpdate(tf::Stamped<tf::Pose> frame_base_pose,
                   tf::Stamped<tf::Pose> frame_start_pose,
                   tf::Stamped<tf::Pose> frame_end_pose,
                   std::vector<double> &ranges,
                   std::vector<double> &angles,
                   int start_index, const size_t beam_number);

 private:

  std::unique_ptr<tf::TransformListener> tf_listener_ptr_;
  ros::NodeHandle *nh_;
  ros::Publisher scan_pub_;

  std::shared_ptr<ParamConfig> param_;

#ifdef USE_PCL_VISUALIZATION
  pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
  std::unique_ptr<pcl::visualization::CloudViewer> laser_data_viewer_;
#endif

};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_LASER_DATA_PROCESSOR_H
