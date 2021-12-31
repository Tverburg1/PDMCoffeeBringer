cmake_minimum_required(VERSION 3.0)
project(rrt_star)

#set(CMAKE_CXX_STANDARD 14)
set(COL ../collision_checks)
include_directories(${COL}/include)


#look for SFML and add it
set(PATH_TO_FINDSFML /usr/share/SFML/cmake/Modules)# the argument should be the filepath to FindSFML.cmake
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}/" ${PATH_TO_FINDSFML})
find_package(SFML 2.3 COMPONENTS system window graphics REQUIRED)

add_executable(rrt_star main.cpp ${COL}/collision_check.cpp ${COL}/load_pointcloud.cpp ${COL}/3d_coordinate.cpp)

target_link_libraries(rrt_star sfml-graphics sfml-window sfml-system)