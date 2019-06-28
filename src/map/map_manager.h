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

#ifndef ROBORTS_SLAM_MAP_MAP_MANAGER_H
#define ROBORTS_SLAM_MAP_MAP_MANAGER_H

#include <iostream>
#include <vector>
#include <string>

#include "slam_map.h"

namespace roborts_slam{


class MapManager{
 public:
  MapManager() {}

  virtual ~MapManager() {}

  std::shared_ptr<ScanMatchMap> GetScanMatchMap(const std::string& map_name){
    if(scan_match_map_container_.find(map_name) != scan_match_map_container_.end()){
      return scan_match_map_container_[map_name];
    }else{
      LOG(WARNING) << "MapManager have no " << map_name << "!";
      return nullptr;
    }
  }

  std::shared_ptr<PubMap> GetPubMap(const std::string& map_name){
    if(pub_map_container_.find(map_name) != pub_map_container_.end()){
      return pub_map_container_[map_name];
    }else{
      LOG(WARNING) << "MapManager have no " << map_name << "!";
      return nullptr;
    }
  }

  bool AddScanMatchMap(const std::string& map_name, std::shared_ptr<ScanMatchMap> map){
    CHECK(map != nullptr) << "Cannot create " << map_name << "!";
    scan_match_map_container_[map_name] = map;
    return true;
  }

  bool AddPubMap(const std::string& map_name, std::shared_ptr<PubMap> map){
    CHECK(map != nullptr) << "Cannot create " << map_name << "!";
    pub_map_container_[map_name] = map;
    return true;
  }


 private:

  std::map<std::string, std::shared_ptr<ScanMatchMap>> scan_match_map_container_;
  std::map<std::string, std::shared_ptr<PubMap>> pub_map_container_;
};






} // namespace roborts_slam

#endif // ROBORTS_SLAM_MAP_MAP_MANAGER_H