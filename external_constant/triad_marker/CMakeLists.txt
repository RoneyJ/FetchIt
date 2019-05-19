cmake_minimum_required(VERSION 2.8.3)
project(triad_marker)

find_package(catkin_simple REQUIRED)

catkin_simple()
#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Executables
cs_add_executable(triad_display src/triad_display.cpp)

cs_install()
cs_export()

