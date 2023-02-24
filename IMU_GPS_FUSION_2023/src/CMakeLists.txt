cmake_minimum_required(VERSION 3.14)
# project(IMU_GPS_EKF_CPP)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# find_package(Eigen3  REQUIRED)
# include_directories(/usr/include/eigen3)
add_executable(IMU_GPS_FUSION main.cpp mat.cpp navi.cpp)