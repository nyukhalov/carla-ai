cmake_minimum_required(VERSION 2.8.3)
project(carla_ai)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

# Install the python package
catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)

# Install executables
catkin_install_python(PROGRAMS
                        nodes/viewer_node
                        nodes/ego_node
                        nodes/control_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
