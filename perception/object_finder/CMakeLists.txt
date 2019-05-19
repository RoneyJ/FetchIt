cmake_minimum_required(VERSION 2.8.3)
project(object_finder)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
#use for display image
find_package(cv_bridge)
find_package(image_transport)
find_package(message_generation)
find_package(sensor_msgs)
find_package(std_msgs)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# Libraries: 
cs_add_library(object_finder_as src/object_finder_as.cpp)   

# Executables: 
cs_add_executable(object_finder_main src/object_finder_main.cpp)
cs_add_executable(example_object_finder_action_client src/example_object_finder_action_client.cpp)
# link a node in this package with a library created in this same package

target_link_libraries(object_finder_main object_finder_as ${OpenCV_LIBS})

cs_install()
cs_export()
    
