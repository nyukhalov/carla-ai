# carla-ai

AI for Carla Simulator

## Installation

- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Follow [the official guide](https://carla.readthedocs.io/en/latest/start_quickstart/) to install Carla
- Install [Carla ROS Bridge](https://github.com/carla-simulator/ros-bridge). The following changes should be made in order to compile it on Ubuntu 20.04
  ```diff
  diff --git a/carla_ros_bridge/src/carla_ros_bridge/bridge.py b/carla_ros_bridge/src/carla_ros_bridge/bridge.py
  index dd7838b..0a7683d 100755
  --- a/carla_ros_bridge/src/carla_ros_bridge/bridge.py
  +++ b/carla_ros_bridge/src/carla_ros_bridge/bridge.py
  @@ -531,7 +531,7 @@ def main():
                  CarlaRosBridge.CARLA_VERSION, dist.version))
              sys.exit(1)
  
  -        if LooseVersion(carla_client.get_server_version()) < \
  +        if False and LooseVersion(carla_client.get_server_version()) < \
              LooseVersion(CarlaRosBridge.CARLA_VERSION):
              rospy.logfatal("CARLA Server version {} required. Found: {}".format(
                  CarlaRosBridge.CARLA_VERSION, carla_client.get_server_version()))
  diff --git a/carla_ros_bridge/src/carla_ros_bridge/lidar.py b/carla_ros_bridge/src/carla_ros_bridge/lidar.py
  index cd920d1..0201f88 100644
  --- a/carla_ros_bridge/src/carla_ros_bridge/lidar.py
  +++ b/carla_ros_bridge/src/carla_ros_bridge/lidar.py
  @@ -57,7 +57,7 @@ class Lidar(Sensor):
          ]
  
          lidar_data = numpy.fromstring(
  -            carla_lidar_measurement.raw_data, dtype=numpy.float32)
  +            carla_lidar_measurement.raw_data.tobytes(), dtype=numpy.float32)
          lidar_data = numpy.reshape(
              lidar_data, (int(lidar_data.shape[0] / 4), 4))
          # we take the oposite of y axis
  diff --git a/pcl_recorder/CMakeLists.txt b/pcl_recorder/CMakeLists.txt
  index 068ed8b..f6a8fc1 100644
  --- a/pcl_recorder/CMakeLists.txt
  +++ b/pcl_recorder/CMakeLists.txt
  @@ -1,7 +1,7 @@
  cmake_minimum_required(VERSION 2.8.3)
  project(pcl_recorder)
  
  -add_compile_options(-std=c++11)
  +add_compile_options(-std=c++14)
  
  find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp
                                          sensor_msgs roslaunch)
  ```
- Install the following libraries
  ```bash
  sudo apt install python-is-python3 python3-rosdep libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev
  ```
- Install [libxerces-c v3.1](https://github.com/apache/xerces-c/tree/v3.1.4)
  ```bash
  git clone git@github.com:apache/xerces-c.git -v v3.1.4
  cd xerces-c
  ./reconf
  ./configure
  make
  cp src/.libs/libxerces-c-3.1.so /usr/lib/
  ```
- Create a conda environment from the `environment.yaml` file in this repository and activate it
  ```bash
  conda env create -f environment.yaml
  conda activate carla-ai
  ```
- Create a catkin workspace
  ```bash
  mkdir catkin_ws/src
  cd catkin_ws/src
  ln -s /path/to/ros-bridge
  cd ..
  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src -r
  catkin_make
  ```
- Source the ROS scripts
  ```bash
  source /opt/ros/noetic/setup.zsh
  source devel/setup.zsh
  ```
- Add the path to the carla egg to PYTHONPATH env variable
  ```bash
  # for carla installed as Ubuntu package
  export PYTHONPATH="/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg:$PYTHONPATH"
  # for carla distr downloaded from github
  export PYTHONPATH="/opt/carla-nightly/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:$PYTHONPATH"
  ```
- Add the path to the system python to PYTHONPATH env variable
  ```bash
  export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"
  ```

## Running

1. Run Carla
   For headless mode execute
   ```bash
   DISPLAY= /opt/carla/bin/CarlaUE4.sh -opengl
   ```

   or for normal mode execute
   ```bash
   /opt/carla/CarlaUE4.sh
   ```
2. Run ros-bridge
   ```bash
   roslaunch carla_ros_bridge carla_ros_bridge.launch
   ```
3. Run the Carla AI module
   ```bash
   roslaunch carla_ai carla_ai.launch
   ```
