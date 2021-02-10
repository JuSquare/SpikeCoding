cmake_minimum_required(VERSION 2.8.3)
project(spike_encoder)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(spike_encoder
  src/encoder.cpp
  src/encoder_node.cpp
)

# make the nodelet into a library
cs_add_library(spike_encoder_nodelet
  src/encoder_nodelet.cpp
  src/encoder.cpp
)

# link the executable to the necesarry libs
target_link_libraries(spike_encoder
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(spike_encoder_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
install(FILES spike_encoder_nodelet.xml test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
