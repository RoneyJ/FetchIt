cmake_minimum_required(VERSION 2.8.3)
project(object_finder_lib)

find_package(catkin_simple REQUIRED)
catkin_simple()

# Libraries: 
cs_add_library(object_finder_lib src/object_finder_lib.cpp)   

cs_install()
cs_export()
    
