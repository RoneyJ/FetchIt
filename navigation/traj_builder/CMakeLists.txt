cmake_minimum_required(VERSION 2.8.3)
project(traj_builder)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Libraries: 
cs_add_library(traj_builder src/traj_builder.cpp)   

cs_install()
cs_export()
    