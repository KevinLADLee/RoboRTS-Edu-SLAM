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

#ifndef ROBORTS_SLAM_ROBORTS_SLAM_NODE_H
#define ROBORTS_SLAM_ROBORTS_SLAM_NODE_H

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMapRequest.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>

#include "laser_data_processor.h"
#include "param_config.h"
#include "util/slam_util.h"
#include "util/transform.h"
#include "slam/slam_processor.h"
#include "slam/sensor_data_manager.h"

namespace roborts_slam {

const double kMapResolution = 0.05;
const double kMapPubPeriod = 1;

const double kTransformPubPeriod = 0.01;

class SlamNode {
 public:
  SlamNode();

  /**
    * @brief Laser scan messages callback function (as main loop of slam process)
    * @param laser_scan_msg Laser scan data
    */
  void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr);

  /**
   * @brief map server callback
   * @return
   */
  bool MapServiceCallback(nav_msgs::GetMap::Request &request,
                          nav_msgs::GetMap::Response &response);

  /**
   * @brief Force to initiate a backend posegraph optimization
   * @return
   */
  bool GraphOptimizeCallback(std_srvs::Empty::Request &request,
                             std_srvs::Empty::Response &response);


 private:

  /**
   * @brief Get odometry information of specify timepoint
   */
  bool GetOdomFromTf(Pose2d &pose_2d, const ros::Time &timestamp);

  std::shared_ptr<roborts_slam::RangeDataContainer2d> BuildRangeDataContainer(const sensor_msgs::LaserScan &scan_msg);

  /**
   * @brief Publish transform information between odom_frame and global_frame(or laser_frame and global_frame)
   * @param frame_id and time_stamp
   */
  void PublishTransform(std::string laser_frame_id, ros::Time time_stamp);

  /**
   * @brief Publish transform information thread
   */
  void PublishTransformThread();

  /**
   * @brief Publish builded map of slam processer periodically
   * @param publish cycle
   */
  void PublishMapThread(double map_pub_period);

  /**
   * @brief Publish vertexes and edges of pose graph for visualization
   */
  void PublishVisualization();

 private:
  using LaserScanMsgFilter = message_filters::Subscriber<sensor_msgs::LaserScan>;
  using TfLaserScanFilter = tf::MessageFilter<sensor_msgs::LaserScan>;

  //ros subcriber publisher server
  ros::NodeHandle nh_;
  std::unique_ptr<tf::TransformListener> tf_listener_ptr_;
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
  ros::Subscriber laser_scan_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_filter_ptr_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_ptr_;
  ros::Publisher map_pub_, markers_pub_;
  ros::ServiceServer map_srv_server_;
  ros::ServiceServer graph_optimize_srv_server_;

  //Parameters
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string base_laser_frame_;
  double transform_publish_period_;


  //Mutex
  std::mutex map_mutex_;
  std::mutex map_to_odom_mutex_;

  //Status
  bool first_map_;
  bool map_to_odom_tramsform_getted_;
  bool map_to_laser_tramsform_getted_;
  bool publish_graph_ = true;

  //Slam
  std::shared_ptr<ParamConfig> slam_param_ptr_;
  std::shared_ptr<roborts_slam::SlamProcessor> slam_processor_ptr_;
  std::shared_ptr<LaserDataProcessor> laser_data_processor_;
  std::shared_ptr<roborts_slam::SensorDataManager> sensor_data_manager_ptr_;
  std::unique_ptr<std::thread> transform_pub_thread_;
  std::unique_ptr<std::thread> map_pub_thread_;

  tf::Transform map_to_odom_;
  tf::Transform map_to_laser_;
  unsigned marker_count_ = 0;

  bool use_odometry_;

  Pose2d last_scan_best_pose_;
  nav_msgs::OccupancyGrid map_;
  int map_update_index_;

};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_ROBORTS_SLAM_NODE_H
