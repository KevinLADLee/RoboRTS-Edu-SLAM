# RoboRTS-Edu
This project is an education version of RoboRTS-SLAM (2d-version).

## Feature

Front-End: correlative scan matching and optimization based scan matching.

Back-End: Graph optimazation wrappered from SLAM-Karto.

## Build and Run

```bash
mkdir -p roborts_edu_ws/src
cd roborts_edu_ws/src
git clone https://github.com/SUSTech-Robotics/RoboRTS-Edu.git
cd ..

sudo rosdep init
#The command ‘sudo rosdep init’ will print an error if you have already executed it since installing ROS. 
#This error can be ignored.

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make

```

## Tutorial

For more information about RoboMaster AI Robot platform and RoboRTS framework, please refer to [RoboRTS-Edu Tutorial](https://sustech-robotics.github.io/RoboRTS-Edu-Tutorial/#/)

## Acknowledgements

* [slam_karto](http://wiki.ros.org/slam_karto)
* Olson, Edwin B. "Real-time correlative scan matching." 2009 IEEE International Conference on Robotics and Automation. IEEE, 2009.
* Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2D mapping." 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2010.


## Developers

This project is developed by @KevinLADLee and fengyu in RoboMaster. 

Special thanks for all RoboRTS project members: 
* Noah Guo
* Charles Yang
* Dunkem Wen

## Copyright and License

RoboRTS-SLAM is provided under the [GPL-v3](COPYING).


