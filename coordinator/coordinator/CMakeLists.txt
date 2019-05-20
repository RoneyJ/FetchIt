cmake_minimum_required(VERSION 2.8.3)
project(coordinator)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# Executables: 
cs_add_executable(visit_all src/visit_all.cpp)
cs_add_executable(grab_all src/grab_all_v2.cpp)
cs_add_executable(vision_demo src/vision_demo.cpp)

cs_install()
cs_export()
