cmake_minimum_required(VERSION 3.10)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(hybrid_astar_planner
        LICENSE src/main.cpp)
