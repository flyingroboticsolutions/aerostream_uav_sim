#!/bin/bash

export PX4_HOME_LAT=43.857251
export PX4_HOME_LON=18.398316
export PX4_HOME_ALT=416

cd ~/codes/PX4-Autopilot/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

roslaunch px4 mavros_posix_sitl.launch sdf:=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_downward_depth_camera/iris_downward_depth_camera.sdf
