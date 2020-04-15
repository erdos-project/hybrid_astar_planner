cmake_minimum_required(VERSION 3.10)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
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

add_library(HybridAStar
        SHARED LICENSE include/utils.h
        src/HybridAStar.cpp include/HybridAStar.h
        src/HybridAStarWrapper.cpp src/Dubins.cpp
        include/Dubins.h src/MapInfo.cpp
        include/MapInfo.h src/Car.cpp include/Car.h
        src/Obstacle.cpp include/Obstacle.h include/py_cpp_struct.h)

target_link_libraries(HybridAStar Qt5::Widgets Qt5::Gui)
