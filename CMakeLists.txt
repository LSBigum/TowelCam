cmake_minimum_required(VERSION 3.10)
project(TowelGripper)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
set(DEPENDENCIES realsense2 boost_system ${OpenCV_LIBS})
add_executable(TowelGripper src/camData.cpp)
target_link_libraries (TowelGripper ${DEPENDENCIES})