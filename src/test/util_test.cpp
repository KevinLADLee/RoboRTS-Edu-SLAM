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

#include "util/transform.h"
#include "util/slam_util.h"

int main(int argc, char** argv){

  using namespace roborts_slam;
  Pose2d base_link(1,1,M_PI / -4);
  Pose2d map(0,0,0);

  Transform2d trans(base_link, map);

  {
    Pose2d pose(0, 0, 0);
    std::cout << trans.Transform(pose).transpose() << std::endl;
  }

  {
    Pose2d pose(1, 1, M_PI / 2);
    std::cout << trans.Transform(pose).transpose() << std::endl;
  }

  {
    Pose2d pose1(1,2,0);
    Transform2d trans1(Pose2d(1,1,M_PI / 2), Pose2d(0, 0, 0));
    std::cout << "Tran1: \n"<< trans1.Transform(pose1).transpose() << std::endl;
  }

  {
    Pose2d pose1(1,1,0);
    Transform2d trans2(Pose2d(1,1,M_PI / 2), Pose2d(2, 1, M_PI / 2));
    std::cout << "Tran2: \n"<< trans2.Transform(pose1).transpose() << std::endl;
  }

  {
    Pose2d pose1(1,1,0);
    Transform2d transA( Pose2d(0, 0, 0),Pose2d(2, 2, 0));
    Transform2d transB(Pose2d(1,1, M_PI / 2), Pose2d(0, 0, 0));

    std::cout << "Tran2: \n"<< transB.Transform(transA.Transform(Pose2d(1, 1, 0))) << std::endl;

    TransformByMidFrame tran(Pose2d(1,1,M_PI/2), Pose2d(2,2,0));

    std::cout << "Tran2: \n"<< tran.Transform(Pose2d(1, 1, 0)) << std::endl;

  }

  std::cout << "Normalize angle: " << util::NormalizeAngle(4.0 * M_PI) << std::endl;


}