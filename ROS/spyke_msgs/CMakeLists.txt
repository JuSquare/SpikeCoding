cmake_minimum_required(VERSION 3.0.2)
project(spyke_msgs)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs)

FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

add_message_files(
    FILES
    ${messages_to_build}
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
