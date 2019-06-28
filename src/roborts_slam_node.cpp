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

#include "roborts_slam_node.h"

namespace roborts_slam{

SlamNode::SlamNode() {

  //load ros param
  slam_param_ptr_ = std::make_shared<ParamConfig>(nh_);

  //frame
  odom_frame_ = slam_param_ptr_->odom_frame_id();
  base_frame_ = slam_param_ptr_->base_frame_id();
  base_laser_frame_ = slam_param_ptr_->laser_frame_id();
  map_frame_ = slam_param_ptr_->global_frame_id();

  // flag that indicates whether or not use odometry in slam system
  use_odometry_ = slam_param_ptr_->use_odometry();

  publish_graph_ = slam_param_ptr_->publish_visualize();
  first_map_ = true;
  map_update_index_ = -1;

  //ros tf
  tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
  tf_listener_ptr_ = std::make_unique<tf::TransformListener>();

  if(use_odometry_){
    //synchronization laser scan msg and odom tf
    laser_scan_sub_filter_ptr_ = std::make_shared<LaserScanMsgFilter>(nh_, "scan", 15);
    laser_scan_filter_ptr_ = std::make_shared<TfLaserScanFilter>(*laser_scan_sub_filter_ptr_, *tf_listener_ptr_, odom_frame_, 15);
    laser_scan_filter_ptr_->registerCallback(boost::bind(&SlamNode::LaserScanCallback, this, _1));
    std::cout << "Use odometry !" << std::endl;
  }else{
    //just subscribe the laser scan msg
    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 15, &SlamNode::LaserScanCallback, this);
    std::cout << "Do not use odometry !" << std::endl;
  }

  //correct the laser data by odom information
  laser_data_processor_ = std::make_shared<LaserDataProcessor>(&nh_, slam_param_ptr_);
  std::cout << "Laser data processor Init!" << std::endl;

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(slam_param_ptr_->map_topic_name(), 1, true);
  map_srv_server_ = nh_.advertiseService("dynamic_map", &SlamNode::MapServiceCallback, this);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("graph_visualization", 1);
  graph_optimize_srv_server_ = nh_.advertiseService("force_graph_optimize", &SlamNode::GraphOptimizeCallback, this);

  //sensor data container included the laser data and odometry data
  sensor_data_manager_ptr_ = std::make_shared<roborts_slam::SensorDataManager>();
  sensor_data_manager_ptr_->set_running_range_size(slam_param_ptr_->running_range_size());
  sensor_data_manager_ptr_->set_running_range_max_distance(slam_param_ptr_->running_range_max_distance());

  std::cout << "Sensor data manager Init!" << std::endl;

  //slam processor
  slam_processor_ptr_ = std::make_shared<roborts_slam::SlamProcessor>(slam_param_ptr_, sensor_data_manager_ptr_,
                                                                      slam_param_ptr_->map_resolution());

  std::cout << "SLAM processor Init!" << std::endl;

  //tf transform and ros tf publish thread
  map_to_odom_tramsform_getted_ = false;
  map_to_laser_tramsform_getted_ = false;
  transform_publish_period_ = kTransformPubPeriod;
  transform_pub_thread_ = std::make_unique<std::thread>(std::bind(&SlamNode::PublishTransformThread, this));

  //map pub thread
  map_pub_thread_ = std::make_unique<std::thread>(std::bind(&SlamNode::PublishMapThread, this, kMapPubPeriod));

}

void SlamNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr) {

  Pose2d odom_pose = Eigen::Vector3d::Zero();
  sensor_msgs::LaserScan laser_scan_msg = *laser_scan_msg_ptr;

  if(base_laser_frame_.empty()){
    base_laser_frame_ = laser_scan_msg_ptr->header.frame_id;
  }

  if(use_odometry_){
    if(slam_param_ptr_->use_odom_correct()){
      laser_data_processor_->Process(laser_scan_msg);
    }

    //get odometry information from ros tf tree
    if(!GetOdomFromTf(odom_pose, laser_scan_msg.header.stamp)){
      ROS_ERROR("Could not determine robot's odom pose!");
      return;
    }
  }

  //get and save the lidar information
  if(sensor_data_manager_ptr_->GetRangeFinder() == nullptr){
    std::unique_ptr<roborts_slam::LaserRangeFinder> range_finder = std::make_unique<roborts_slam::LaserRangeFinder>(laser_scan_msg.angle_min,
                                                                                                                    laser_scan_msg.angle_max,
                                                                                                                    laser_scan_msg.angle_increment,
                                                                                                                    laser_scan_msg.range_min,
                                                                                                                    laser_scan_msg.range_max);
    sensor_data_manager_ptr_->SetRangeFinder(std::move(range_finder));
    sensor_data_manager_ptr_->GetRangeFinder()->set_range_threshold_scale(slam_param_ptr_->range_threshold_scale());
  }

  //save the laser range data and odometry data to sensor_data_manager
  auto range_data_container = BuildRangeDataContainer(laser_scan_msg);
  sensor_data_manager_ptr_->AddSensorData(range_data_container, roborts_slam::OdometryData(odom_pose));

  //start an slam processing
  if(slam_processor_ptr_->process()){

    last_scan_best_pose_ = slam_processor_ptr_->current_sensor_pose();

    //if slam processing succeed, update tf
    PublishTransform(laser_scan_msg.header.frame_id, laser_scan_msg.header.stamp);

    PublishVisualization();

  }

}

bool SlamNode::MapServiceCallback(nav_msgs::GetMap::Request &request,
                                  nav_msgs::GetMap::Response &response) {
  {
    std::lock_guard<std::mutex> lock(slam_processor_ptr_->GetMapMutex());
    response.map = map_;
  }
  return true;
}

bool SlamNode::GraphOptimizeCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  slam_processor_ptr_->ForceGraphOptimize();
  return true;
}

bool SlamNode::GetOdomFromTf(Pose2d& pose_2d, const ros::Time & timestamp) {

  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
                              timestamp,
                              base_laser_frame_);
  tf::Stamped<tf::Pose> pose_stamp;
  try {
    this->tf_listener_ptr_->transformPose(odom_frame_,
                                          ident,
                                          pose_stamp);
  } catch (tf::TransformException &e) {
    ROS_ERROR("Couldn't transform from %s to %s", base_laser_frame_.c_str(), odom_frame_.c_str());
    return false;
  }

  pose_2d(0) = pose_stamp.getOrigin().x();
  pose_2d(1) = pose_stamp.getOrigin().y();
  double yaw, pitch, roll;
  pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
  pose_2d(2) = yaw;

  return true;
}

void SlamNode::PublishTransformThread() {
  if(transform_publish_period_ == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period_);
  while(ros::ok())
  {
    {
      std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
      if(map_to_odom_tramsform_getted_){
        tf_broadcaster_ptr_->sendTransform(tf::StampedTransform(map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
      }
      if(map_to_laser_tramsform_getted_){
        tf_broadcaster_ptr_->sendTransform( tf::StampedTransform (map_to_laser_, ros::Time::now(), map_frame_, base_laser_frame_));
      }
    }
    r.sleep();
  }
}

void SlamNode::PublishVisualization() {
  if(publish_graph_){
    std::vector<Eigen::Vector2d> graph_nodes;

    std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d> > graph_edges;

    slam_processor_ptr_->GetGraphInfo(graph_nodes, graph_edges);

    visualization_msgs::MarkerArray marray;

    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.lifetime = ros::Duration(0);

    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.02;
    edge.scale.y = 0.02;
    edge.scale.z = 0.02;
    edge.color.a = 1.0;
    edge.color.r = 0.0;
    edge.color.g = 0.0;
    edge.color.b = 1.0;

    m.action = visualization_msgs::Marker::ADD;

    uint id = 0;

    for (uint i=0; i < graph_nodes.size(); i++)
    {
      m.id = id;
      m.pose.position.x = graph_nodes[i](0);
      m.pose.position.y = graph_nodes[i](1);
      marray.markers.push_back(visualization_msgs::Marker(m));
      id++;
    }

    if(!graph_edges.empty()) {
      for (uint i = 0; i < graph_edges.size(); i++) {

        geometry_msgs::Point p;

        p.x = graph_edges[i].first(0);

        p.y = graph_edges[i].first(1);

        edge.points.push_back(p);

        p.x = graph_edges[i].second(0);

        p.y = graph_edges[i].second(1);

        edge.points.push_back(p);

        edge.id = id;

        marray.markers.push_back(visualization_msgs::Marker(edge));

        id++;
      }
    }

    m.action = visualization_msgs::Marker::DELETE;

    for (; id < marker_count_; id++)
    {
      m.id = id;
      marray.markers.push_back(visualization_msgs::Marker(m));
    }

    marker_count_ = marray.markers.size();
    markers_pub_.publish(marray);
  }
}

std::shared_ptr<roborts_slam::RangeDataContainer2d> SlamNode::BuildRangeDataContainer(const sensor_msgs::LaserScan &scan_msg) {
  size_t size = scan_msg.ranges.size();
  std::shared_ptr<roborts_slam::RangeDataContainer2d> range_data_container = std::make_shared<roborts_slam::RangeDataContainer2d>(size);
  range_data_container->set_sensor_origin(Eigen::Vector2d::Zero());
  double current_index_angle = scan_msg.angle_min;
  double range_threshold = sensor_data_manager_ptr_->GetRangeFinder()->range_threshold();
//  double resolution = slam_processor_ptr_->get_map_resolution();
  for(size_t i = 0; i < size; ++i){
    double dist = scan_msg.ranges[i];

    if((dist > scan_msg.range_min) && (dist < range_threshold))
    {
//      dist /= resolution;
      range_data_container->AddDataPoint(Eigen::Vector2d(cos(current_index_angle) * dist, sin(current_index_angle) * dist));
    }

    current_index_angle += scan_msg.angle_increment;
  }

  range_data_container->set_sensor_pose(last_scan_best_pose_);
  return range_data_container;
}



void SlamNode::PublishTransform(std::string laser_frame_id, ros::Time time_stamp) {

  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);

  if(use_odometry_){
    tf::StampedTransform odom_to_laser;
    try
    {
      tf_listener_ptr_->waitForTransform(odom_frame_, laser_frame_id, time_stamp, ros::Duration(0.5));
      tf_listener_ptr_->lookupTransform(odom_frame_, laser_frame_id, time_stamp, odom_to_laser);
    }
    catch(tf::TransformException &e)
    {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
      odom_to_laser.setIdentity();
    }


    map_to_odom_ =  tf::Transform(tf::Transform(tf::createQuaternionFromRPY(0, 0, last_scan_best_pose_[2]),
                                                tf::Vector3(last_scan_best_pose_[0], last_scan_best_pose_[1], 0.0))
                                      * odom_to_laser.inverse());

    tf_broadcaster_ptr_->sendTransform( tf::StampedTransform (map_to_odom_, time_stamp, map_frame_, odom_frame_));
    map_to_odom_tramsform_getted_ = true;

  }else{
    map_to_laser_ = tf::Transform(tf::Transform(tf::createQuaternionFromRPY(0, 0, last_scan_best_pose_[2]),
                                                tf::Vector3(last_scan_best_pose_[0], last_scan_best_pose_[1], 0.0)));

    tf_broadcaster_ptr_->sendTransform( tf::StampedTransform (map_to_laser_, time_stamp, map_frame_, base_laser_frame_));
    map_to_laser_tramsform_getted_ = true;
  }






}

void SlamNode::PublishMapThread(double map_pub_period) {
  ros::Rate r(1.0 / map_pub_period);
  ROS_INFO("map pub period : %f", map_pub_period);

  while(ros::ok())
  {
    {
      ros::WallTime t1 = ros::WallTime::now();

      std::lock_guard<std::mutex> lock(slam_processor_ptr_->GetMapMutex());

      if (first_map_) {
        //init map object
        map_.info.resolution = slam_processor_ptr_->map_resolution();
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        first_map_ = false;
      }

      //get map from the slam_processor
      std::shared_ptr<roborts_slam::PubMap> occu_grid_map = slam_processor_ptr_->GetPubMap();
      if (occu_grid_map != nullptr
          && occu_grid_map->map_update_index() != map_update_index_){
        // if map is updated,then update publish map information

        map_update_index_ = occu_grid_map->map_update_index();

        double resolution = occu_grid_map->GetCellLength();

        const bool kDisplayFullMap = false;
        if (kDisplayFullMap) {
          Eigen::Vector2d map_origin(occu_grid_map->GetWorldCoords(Eigen::Vector2d::Zero()));
          map_origin.array() -= occu_grid_map->GetCellLength() * 0.5;
          map_.info.origin.position.x = map_origin.x();
          map_.info.origin.position.y = map_origin.y();
          map_.info.origin.orientation.w = 1.0;

          map_.info.resolution = resolution;

          map_.info.width = occu_grid_map->GetSizeX();
          map_.info.height = occu_grid_map->GetSizeY();

          int size = map_.info.width * map_.info.height;
          map_.data.resize(size);
          std::vector<int8_t> &data = map_.data;
          memset(&data[0], -1, sizeof(int8_t) * size);
          // int size = occu_grid_map->GetGridCellNum();
          for (int i = 0; i < size; ++i) {
            switch (occu_grid_map->GetGridStates(i)) {
              case roborts_slam::GridStates_Occupied: {
                data[i] = 100;
                break;
              }
              case roborts_slam::GridStates_Free: {
                data[i] = 0;
                break;
              }
              case roborts_slam::GridStates_Unknown: {
                data[i] = -1;
                break;
              }
              default:break;
            }
          }
        } else {
          //just load the processed map of boundbox size
          Eigen::Vector2d map_origin(occu_grid_map->GetWorldCoords(occu_grid_map->GetStartGrid()));
          map_origin.array() -= occu_grid_map->GetCellLength() * 0.5;
          map_.info.origin.position.x = map_origin.x();
          map_.info.origin.position.y = map_origin.y();
          map_.info.origin.orientation.w = 1.0;

          map_.info.resolution = resolution;

          map_.info.width = (occu_grid_map->GetBoundSizeX());
          map_.info.height = (occu_grid_map->GetBoundSizeY());

          int size = map_.info.width * map_.info.height;
          map_.data.resize(size, static_cast<int8_t>(-1));
          std::vector<int8_t> &data = map_.data;

          int data_index = 0;
          int start_x = occu_grid_map->GetStartGrid()[0];
          int start_y = occu_grid_map->GetStartGrid()[1];
          int end_x = occu_grid_map->GetEndGrid()[0];
          int end_y = occu_grid_map->GetEndGrid()[1];

          for (int j = start_y; j <= end_y; ++j) {
            for (int i = start_x; i <= end_x; ++i) {

              switch (occu_grid_map->GetGridStates(i, j)) {
                case roborts_slam::GridStates_Occupied: {
                  data.at(data_index) = 100;
                  break;
                }
                case roborts_slam::GridStates_Free: {
                  data.at(data_index) = 0;
                  break;
                }
                case roborts_slam::GridStates_Unknown: {
                  data.at(data_index) = -1;
                  break;
                }
                default:break;
              }
              data_index++;

            }
          }
        }

      }

      //publish the map whatever
      map_.header.stamp = ros::Time::now();
      map_.header.frame_id = map_frame_;
      map_pub_.publish(map_);

      DLOG(INFO) << "Map width: " << map_.info.width << " height: " << map_.info.height;

//      ros::WallDuration t2 = ros::WallTime::now() - t1;
//      std::cout << "map pub time s: " << t2.toSec() << std::endl;
    }

    r.sleep();
  }

}

} // namespace roborts_slam



int main(int argc, char** argv){
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "roborts_slam_node");
  roborts_slam::SlamNode slam_node;
  ros::spin();
}
