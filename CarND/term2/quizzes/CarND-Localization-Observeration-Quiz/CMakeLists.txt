project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources1 src/observation1.cpp)
set(sources2 src/observation2.cpp)
set(sources3 src/observation3.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)


add_executable(observation1 ${sources1})
add_executable(observation2 ${sources2})
add_executable(observation3 ${sources3})
