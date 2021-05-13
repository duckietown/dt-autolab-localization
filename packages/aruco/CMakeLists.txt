cmake_minimum_required(VERSION 3.0)
project(aruco)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages should use this.
)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonInterp)
find_package(PythonLibs REQUIRED)
find_package(aruco REQUIRED)
find_package(boost_numpy REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

python_add_module(aruco_python lib/aruco_python.cpp)
target_include_directories(aruco_python PUBLIC
        ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(aruco_python
        ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${OPENCV_LIBRARIES}
        boost_numpy38 aruco opencv_highgui)
