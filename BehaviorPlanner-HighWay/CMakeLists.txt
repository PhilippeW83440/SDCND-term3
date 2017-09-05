project(BEHAVIOR_PLANNNER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources road.h vehicle.h main.cpp road.cpp vehicle.cpp)


add_executable(behavior_planner ${sources})
