cmake_minimum_required(VERSION 3.10)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS "-march=native -O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Eigen3)

if(Eigen3_FOUND)
    set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen)
    set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(.)
include_directories(
        ${CUSTOM_EIGEN}
)

add_executable(hybrid_astar_planner
        LICENSE src/main.cpp src/utils.h src/HybridAStar/AStar.cpp
        src/HybridAStar/AStar.h
        src/HybridAStar/HybridAStar.cpp src/HybridAStar/HybridAStar.h
        src/HybridAStar/HybridAStarWrapper.cpp src/DubinsPath/Dubins.cpp
        src/DubinsPath/Dubins.h src/MapInfo/MapInfo.cpp
        src/MapInfo/MapInfo.h src/Car/Car.cpp src/Car/Car.h
        src/Obstacle/Obstacle.cpp src/Obstacle/Obstacle.h )
target_link_libraries(hybrid_astar_planner Qt5::Widgets Qt5::Gui)
