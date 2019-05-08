##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.10)
project(TowelGripper)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

##############################################################################
# Dependencies
##############################################################################

find_package(OpenCV REQUIRED)
include_directories(
  include/cam
  LIBRARIES cam
)
set(DEPENDENCIES realsense2 boost_system ${OpenCV_LIBS})

##############################################################################
# Binaries
##############################################################################

add_executable(TowelGripper src/camData.cpp)
target_link_libraries (TowelGripper ${DEPENDENCIES})
