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

#include "laser_data_processor.h"

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <array>

#include "util/transform.h"

namespace roborts_slam{

LaserDataProcessor::LaserDataProcessor(ros::NodeHandle* nh, std::shared_ptr<ParamConfig> param):
    nh_(nh),
    param_(param) {

  tf_listener_ptr_ = std::make_unique<tf::TransformListener>();
  scan_pub_ = nh_->advertise<sensor_msgs::LaserScan>("/scan_processed", 10);

#ifdef USE_PCL_VISUALIZATION
  laser_data_viewer_ = std::make_unique<pcl::visualization::CloudViewer>("laser_data_viewer");
#endif

}

void LaserDataProcessor::Process(sensor_msgs::LaserScan & scan_msg) {

  ros::Time start_time;
  ros::Time end_time;
  start_time = scan_msg.header.stamp;

  sensor_msgs::LaserScan scan_msg_processed(scan_msg);

  int beam_num = scan_msg.ranges.size();
  end_time = start_time + ros::Duration(scan_msg.time_increment * beam_num);


  std::vector<double> angles,ranges;
  for(int i = 0; i < beam_num; ++i) {
    double laser_range = scan_msg.ranges[i];
    double laser_angle = scan_msg.angle_min + scan_msg.angle_increment * i;

    //eliminate the error data
    if(laser_range < 0.05 || std::isnan(laser_range) || std::isinf(laser_range)){
      laser_range = 0.0;
    }

    ranges.push_back(laser_range);
    angles.push_back(laser_angle);
  }

#ifdef USE_PCL_VISUALIZATION
  tf::Stamped<tf::Pose> visual_pose;
  if(!GetLaserPose(visual_pose, start_time))
  {
    ROS_WARN("Not visualPose,Can not Calib");
    return ;
  }
  double visual_yaw = tf::getYaw(visual_pose.getRotation());

  visual_cloud_.clear();
  for(int i = 0; i < ranges.size();i++)
  {

    if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
      continue;

    double x = ranges[i] * cos(angles[i]);
    double y = ranges[i] * sin(angles[i]);

    pcl::PointXYZRGB pt;
    pt.x = x * cos(visual_yaw) - y * sin(visual_yaw) + visual_pose.getOrigin().getX();
    pt.y = x * sin(visual_yaw) + y * cos(visual_yaw) + visual_pose.getOrigin().getY();
    pt.z = 1.0;

    // pack r/g/b into rgb
    unsigned char r = 255, g = 0, b = 0;    //red color
    unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
    pt.rgb = *reinterpret_cast<float*>(&rgb);

    visual_cloud_.push_back(pt);
  }
#endif

  ros::WallTime startTime = ros::WallTime::now();

  if(start_time < end_time && ranges.size() > 0){
//    std::cout << "DataCorrect !" << std::endl;
    DataCorrect(ranges, angles, start_time, end_time);

    //Recovery the corrected laser data
    for(int i = 0; i < ranges.size(); i++) {
      scan_msg_processed.ranges[i] = 0.0f;
    }
    for(int i = 0; i < ranges.size(); i++) {
      int j = (angles[i] - scan_msg_processed.angle_min) / scan_msg_processed.angle_increment;
      if(j < 0)continue;
      scan_msg_processed.ranges[j] = ranges[i];
    }

    scan_pub_.publish(scan_msg_processed);
    scan_msg = scan_msg_processed;
  }

  ros::WallDuration duration = ros::WallTime::now() - startTime;
//  ROS_INFO("LaserDataProcess took: %f milliseconds", duration.toSec()*1000.0f );

#ifdef USE_PCL_VISUALIZATION
  for(int i = 0; i < ranges.size();i++)
  {

    if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
      continue;

    double x = ranges[i] * cos(angles[i]);
    double y = ranges[i] * sin(angles[i]);


    pcl::PointXYZRGB pt;
    pt.x = x * cos(visual_yaw) - y * sin(visual_yaw) + visual_pose.getOrigin().getX();
    pt.y = x * sin(visual_yaw) + y * cos(visual_yaw) + visual_pose.getOrigin().getY();
    pt.z = 1.0;

    unsigned char r = 0, g = 255, b = 0;    // green color
    unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
    pt.rgb = *reinterpret_cast<float*>(&rgb);

    visual_cloud_.push_back(pt);
  }

  laser_data_viewer_->showCloud(visual_cloud_.makeShared());
#endif

}


void LaserDataProcessor::DataCorrect(std::vector<double>& ranges,
                                     std::vector<double>& angles,
                                     ros::Time frame_start_time, ros::Time frame_end_time) {

  int beam_number = ranges.size();
  if(beam_number != angles.size()) {
    ROS_WARN("Error:ranges not match to the angles");
    return ;
  }
//  std::cout << "beam_number " << beam_number << std::endl;
//  std::cout << "frame_start_time " << frame_start_time << std::endl;
//  std::cout << "frame_end_time " << frame_end_time << std::endl;

  int interpolation_time_duration = static_cast<int>(param_->odom_interpolation_time() * 1e6);
//  std::cout << "interpolation_time_duration " << interpolation_time_duration << std::endl;

  tf::Stamped<tf::Pose> frame_start_pose;
  tf::Stamped<tf::Pose> frame_mid_pose;
  tf::Stamped<tf::Pose> frame_base_pose;
  tf::Stamped<tf::Pose> frame_end_pose;

  //time resolutin is us
  double start_time = frame_start_time.toSec() * 1e6;
  double end_time = frame_end_time.toSec() * 1e6;
  double time_inc = (end_time - start_time) / beam_number;

  int start_index = 0;

  if(!GetLaserPose(frame_start_pose, frame_start_time)) {
    ROS_WARN("Not Start Pose,Can not Calib");
    return ;
  }

  if(!GetLaserPose(frame_end_pose, frame_end_time)) {
    ROS_WARN("Not End Pose, Can not Calib");
    return ;
  }

//  std::cout << "frame_start_pose " << frame_start_pose.getOrigin().getX()
//            << ", " << frame_start_pose.getOrigin().getY()
//            << ", " << tf::getYaw(frame_start_pose.getRotation())
//            << std::endl;
//  std::cout << "frame_end_pose " << frame_end_pose.getOrigin().getX()
//            << ", " << frame_end_pose.getOrigin().getY()
//            << ", " << tf::getYaw(frame_end_pose.getRotation())
//            << std::endl;

  //split the laser data into several beams and handle separately
  frame_base_pose = frame_start_pose;
  for(int i = 0; i < beam_number; i++) {
    double mid_time = start_time + time_inc * (i - start_index);
    if(mid_time - start_time > interpolation_time_duration || (i == beam_number - 1)) {

      if(!GetLaserPose(frame_mid_pose, ros::Time(mid_time/1e6))) {
        return ;
      }

      size_t interp_count = i - start_index + 1;

      BeamsUpdate(frame_base_pose,
                  frame_start_pose,
                  frame_mid_pose,
                  ranges,
                  angles,
                  start_index,
                  interp_count);

      start_time = mid_time;
      start_index = i;
      frame_start_pose = frame_mid_pose;
    }
  }
}




void LaserDataProcessor::BeamsUpdate(
    tf::Stamped<tf::Pose> frame_base_pose,
    tf::Stamped<tf::Pose> frame_start_pose,
    tf::Stamped<tf::Pose> frame_end_pose,
    std::vector<double>& ranges,
    std::vector<double>& angles,
    int start_index, const size_t beam_number) {

//  double frame_mid_yaw[beam_number];
//  double frame_mid_x[beam_number];
//  double frame_mid_y[beam_number];

  std::unique_ptr<double []> frame_mid_yaw(new double[beam_number]);
  std::unique_ptr<double []> frame_mid_x(new double[beam_number]);
  std::unique_ptr<double []> frame_mid_y(new double[beam_number]);

  double start_pose_angle = -tf::getYaw(frame_start_pose.getRotation());
  double end_pose_angle = -tf::getYaw(frame_end_pose.getRotation());
  double angle_diff = end_pose_angle - start_pose_angle;
  if(angle_diff > M_PI){
    start_pose_angle += 2 * M_PI;
  }else if(angle_diff < -M_PI){
    end_pose_angle += 2 * M_PI;
  }
  frame_mid_yaw[0] = start_pose_angle;
  frame_mid_yaw[beam_number-1] = end_pose_angle;
  double det_yaw = (frame_mid_yaw[beam_number-1] - frame_mid_yaw[0]) / (beam_number - 1);

  frame_mid_x[0] = frame_start_pose.getOrigin().getX();
  frame_mid_x[beam_number-1] = frame_end_pose.getOrigin().getX();
  double det_x = (frame_mid_x[beam_number-1] - frame_mid_x[0]) / (beam_number - 1);

  frame_mid_y[0] = frame_start_pose.getOrigin().getY();
  frame_mid_y[beam_number-1] = frame_end_pose.getOrigin().getY();
  double det_y = (frame_mid_y[beam_number-1] - frame_mid_y[0]) / (beam_number - 1);


  for(int i = 0; i < beam_number; i++) {
    frame_mid_yaw[i] = frame_mid_yaw[0] + det_yaw * (i);
    frame_mid_x[i] = frame_mid_x[0] + det_x * (i);
    frame_mid_y[i] = frame_mid_y[0] + det_y * (i);
  }

  double frame_base_yaw = -tf::getYaw(frame_base_pose.getRotation());
  double frame_base_x = frame_base_pose.getOrigin().getX();
  double frame_base_y = frame_base_pose.getOrigin().getY();

  for(int i = start_index; i < start_index + beam_number; i++) {
    int mid_index = i - start_index;

    double x = ranges[i] * cos(angles[i]);
    double y = ranges[i] * sin(angles[i]);

    Pose2d mid_frame(frame_mid_x[mid_index], frame_mid_y[mid_index], frame_mid_yaw[mid_index]);
    Pose2d base_frame(frame_base_x, frame_base_y, frame_base_yaw);

    //transform mid_frame points to base_frame point coordinate
    roborts_slam::Transform2d trans_mid_to_odom(Pose2d(0,0,0), mid_frame);
    roborts_slam::Transform2d trans_odom_to_base(base_frame, Pose2d(0,0,0));
    Pose2d base_point = trans_odom_to_base.Transform(trans_mid_to_odom.Transform(Pose2d(x, y, 0)));

//    TransformByMidFrame trans_mid_to_base(base_frame, mid_frame);
//    Pose2d base_point = trans_mid_to_base.Transform(Pose2d(x, y, 0));

    ranges[i] = sqrt(base_point.x() * base_point.x() +
                     base_point.y() * base_point.y());
    angles[i] = atan2(base_point.y(), base_point.x());


//    double x_odom = x * cos(frame_mid_yaw[mid_index]) - y * sin(frame_mid_yaw[mid_index]) + frame_mid_x[mid_index];
//    double y_odom = x * sin(frame_mid_yaw[mid_index]) + y * cos(frame_mid_yaw[mid_index]) + frame_mid_y[mid_index];
//
//    double tmp_x = x_odom - frame_base_x;
//    double tmp_y = y_odom - frame_base_y;
//
//    frame_mid_x[mid_index] = tmp_x * cos(-frame_base_yaw) - tmp_y * sin(-frame_base_yaw);
//    frame_mid_y[mid_index] = tmp_x * sin(-frame_base_yaw) + tmp_y * cos(-frame_base_yaw);
//
//    ranges[i] = sqrt(frame_mid_x[mid_index] * frame_mid_x[mid_index] +
//        frame_mid_y[mid_index] * frame_mid_y[mid_index]);
//    angles[i] = atan2(frame_mid_y[mid_index], frame_mid_x[mid_index]);
  }

}

bool LaserDataProcessor::GetLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time tf_time) {

  odom_pose.setIdentity();

  tf::Stamped < tf::Pose > laser_pose;
  laser_pose.setIdentity();
  laser_pose.frame_id_ = param_->laser_frame_id();
  laser_pose.stamp_ = tf_time;

  try
  {
    if(!tf_listener_ptr_->waitForTransform(param_->odom_frame_id(), param_->laser_frame_id(), tf_time, ros::Duration(0.1))) {
      std::cout << "Can not Wait Transform" << std::endl;
      return false;
    }
    tf_listener_ptr_->transformPose(param_->odom_frame_id(), laser_pose, odom_pose);
  }
  catch (tf::LookupException& ex)
  {
    std::cout << "No Transform available" << std::endl;
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    std::cout << "Connectivity Error" << std::endl;
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    std::cout << "Extrapolation Error" << std::endl;
    return false;
  }

  return true;
}

} // namespace roborts_slam