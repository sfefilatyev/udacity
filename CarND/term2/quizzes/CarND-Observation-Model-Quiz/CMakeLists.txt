project(ObservationModel)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/observation_model.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(ObservationModel ${sources})
