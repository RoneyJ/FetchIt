cmake_minimum_required(VERSION 2.8.3)
project(test_fetch_arm_ac)

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

#Executable:
# General start
cs_add_executable(fetch_arm_pre_pose src/fetch_arm_pre_pose.cpp)
# For tucked position only
cs_add_executable(fetch_arm_safe_start src/fetch_arm_safe_start.cpp)
# Head Preset
cs_add_executable(fetch_head_tilt_preset src/fetch_head_tilt_preset.cpp)
# Torso PReset
cs_add_executable(fetch_torso_lift_preset src/fetch_torso_lift_preset.cpp)

cs_install()
cs_export()
    
