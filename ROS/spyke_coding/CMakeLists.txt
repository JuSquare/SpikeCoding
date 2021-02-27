cmake_minimum_required(VERSION 3.0.2)
project(spyke_coding)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package(
  LIBRARIES spyke_coding spyke_coding_schemes
  CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(PROGRAMS src/generate_spikes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

