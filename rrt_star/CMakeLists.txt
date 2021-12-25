cmake_minimum_required(VERSION 3.0)
project(rrt_star)

#set(CMAKE_CXX_STANDARD 14)
set(COL ../collision_checks)
include_directories(${COL}/include)


add_executable(rrt_star main.cpp ${COL}/collision_check.cpp ${COL}/load_pointcloud.cpp ${COL}/3d_coordinate.cpp)
