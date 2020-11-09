#!/usr/bin/env zsh

set -e

export PYTHONPATH="/opt/carla-0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:$PYTHONPATH"
export PYTHONPATH="/opt/carla-0.9.10.1/PythonAPI/carla:$PYTHONPATH"
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
