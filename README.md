# AeroSTREAM UAV Simulation Research Training

## ROS Workspace
[RTFM](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Create and source your catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
sudo apt install ros-noetic-joy
sudo pip install pynput
```

## PX4 SITL + Gazebo
[RTFM](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo-classic)

Clone and setup PX4 source code:
```
mkdir -p ~/codes
cd ~/codes

git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.0
git submodule update --init --recursive

bash ./Tools/setup/ubuntu.sh --no-nuttx
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
```

Reboot the system (?) and compile PX4 source code:
```
cd ~/codes/PX4-Autopilot/
make
make px4_sitl jmavsim
make px4_sitl gazebo-classic
```

Now you can command to take off directly from the PX4 console:
```
commander takeoff
```

Or from a Ground Control Station (GCS). For example, [these](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu) are the instructions to install [QGroundControl](http://qgroundcontrol.com/):
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
chmod +x ./QGroundControl.AppImage
```

Let's bring this drone closer!
```
export PX4_HOME_LAT=43.857251
export PX4_HOME_LON=18.398316
export PX4_HOME_ALT=416
```

## ROS integration
[Here](https://docs.px4.io/main/en/ros/mavros_installation.html) how to install MAVROS:
```
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

cd ~/codes
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

[Here](https://docs.px4.io/main/en/simulation/ros_interface.html) how to launch PX4 SITL from ROS:
```
cd ~/codes/PX4-Autopilot/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
roslaunch px4 px4.launch

cd ~/codes/PX4-Autopilot/
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world

cd ~/codes/PX4-Autopilot/
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
rosrun gazebo_ros spawn_model -sdf -file $(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf -model iris -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0
```

Or the *tl;dr* version:
```
roslaunch px4 posix_sitl.launch
```

---

Until now, we have PX4 SITL and a simulation of a UAV, but we cannot communicate with the drone through ROS... yet!
Introducing [**MAVROS**](http://wiki.ros.org/mavros)!
```
cd ~/codes/PX4-Autopilot/
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch px4 mavros_posix_sitl.launch
```

Want a camera?
```
roslaunch px4 mavros_posix_sitl.launch sdf:=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_downward_depth_camera/iris_downward_depth_camera.sdf
```

Let's check it's working! Try [this](https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html).
```
cd ~/catkin_ws
catkin_create_pkg aerostream_uav_sim roscpp
catkin_make
source devel/setup.bash
```

Copy the node into a file, compile and test:
```
catkin_make
rosrun aerostream_uav_sim offb_node
```
This node is a fantastic staring point to start editing. Have fun!

Bonus: Want PX4 to publish map to base_link tf?
```
sudo nano $(rospack find mavros)/launch/px4_config.yaml
```

...and just change send param to true:
```
# local_position
local_position:
  frame_id: "map"
  tf:
    send: true
    frame_id: "map"
    child_frame_id: "base_link"
    send_fcu: false
```

More bonus: Want to insert models?
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find aerostream_uav_sim)/models
```

## AirSim
For the simulator, let's use [precompiled](https://github.com/Microsoft/AirSim/releases), wi will test v1.8.1 for Linux. Just execute the bash file for the selected scenario.

But, again, this is only the simulator. In order to communicate it with [ROS](https://microsoft.github.io/AirSim/airsim_ros_pkgs):
```
sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-mavros*
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
./setup.sh
./build.sh
source ~/.bashrc
catkin_make
```

**NOTE**: the ros2 folder should be CATKIN_IGNOREd

Now we can launch the *bridge* between the simulator and ROS:
```
roslaunch airsim_ros_pkgs airsim_node.launch
```

And communicate with it though topics and services:
```
rosservice call /airsim_node/SimpleFlight/takeoff "waitOnLastTask: false"
rosservice call /airsim_node/SimpleFlight/land "waitOnLastTask: false"
```

AirSim [settings](https://microsoft.github.io/AirSim/settings/) file should be something like:
```
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor"
}
```

Why don't we make another node to play with the joystick here?!

