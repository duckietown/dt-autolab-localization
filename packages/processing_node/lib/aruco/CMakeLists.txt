cmake_minimum_required(VERSION 3.0)
project(aruco_caller)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonInterp)
find_package(PythonLibs REQUIRED)
find_package(aruco REQUIRED)
find_package(boost_numpy REQUIRED)

python_add_module(aruco_caller aruco_caller.cpp)
target_include_directories(aruco_caller PUBLIC
        ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(aruco_caller
        ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${OPENCV_LIBRARIES}
        boost_numpy aruco opencv_highgui)
