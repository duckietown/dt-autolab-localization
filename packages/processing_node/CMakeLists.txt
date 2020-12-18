cmake_minimum_required(VERSION 2.9)
project(processing_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  duckietown_msgs)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

catkin_package()

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

add_executable(apriltag_processor_node src/apriltag_processor_node.cpp)

add_dependencies(apriltag_processor_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(apriltag_processor_node ${catkin_LIBRARIES} ${OPENCV_LIBRARIES} aruco)

target_include_directories(apriltag_processor_node PUBLIC ${OPENCV_INCLUDE_DIRS})