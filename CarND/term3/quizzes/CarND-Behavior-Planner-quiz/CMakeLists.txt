project(BehaviorPlanner)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/cost.cpp src/main.cpp src/road.cpp src/vehicle.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(BehaviorPlanner ${sources})
