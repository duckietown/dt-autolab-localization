cmake_minimum_required(VERSION 2.8.3)
project(distributed_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  autolab_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
