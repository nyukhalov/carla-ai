#!/usr/bin/env zsh

set -e

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"${ACADOS_HOME}/lib"
export PYTHONPATH="${CARLA_HOME}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:$PYTHONPATH"
export PYTHONPATH="${CARLA_HOME}/PythonAPI/carla:$PYTHONPATH"
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"
source /opt/ros/noetic/setup.zsh
source "${CARLA_CATKIN_WS_PATH}/devel/setup.zsh"
