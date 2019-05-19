cmake_minimum_required(VERSION 2.8.3)
project(mobot_pub_des_state)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
find_package(Boost REQUIRED COMPONENTS system thread)


catkin_simple()

# Executables: 
cs_add_executable(mobot_pub_des_state src/pub_des_state_main.cpp src/pub_des_state.cpp)
cs_add_executable(open_loop_controller src/open_loop_controller.cpp)
cs_add_executable(pub_des_state_path_client_amcl_correction src/pub_des_state_path_client_amcl_correction.cpp)

cs_install()
cs_export()
    
