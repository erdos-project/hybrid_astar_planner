cmake_minimum_required(VERSION 3.10)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(hybrid_astar_planner
        LICENSE src/main.cpp src/HybridAStar/AStar.cpp src/HybridAStar/AStar.h
        src/HybridAStar/HybridAStar.cpp src/HybridAStar/HybridAStar.h
        src/HybridAStar/HybridAStarWrapper.cpp src/DubinsPath/DubinsPath.cpp
        src/DubinsPath/DubinsPath.h src/MapInfo/MapInfo.cpp
        src/MapInfo/MapInfo.h src/Car/Car.cpp src/Car/Car.h
        )
