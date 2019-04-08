# teamCaseFetch
This repository hosts TeamCase's Fetchit Challenge.

For Fetchit Challenge, refer:
https://opensource.fetchrobotics.com/competition

This code is organized in three major areas: navigation, perception and manipulation. See READMEs in these directories regarding their use and how to run subsystem tests.

The coordinator package is responsible for integrating the above.  This is aided by libraries in
each subsystem that simplify the interfaces to them, including move_base_lib, move_part_lib and object_finder_lib.

A typical sequence would consist of: invoking navigation (e.g. to a pre-coded zone), invoking object finding 
(using object codes) and object manipulation (with part codes and location  codes or specified poses).

Folders pre-fixed with "test" are designated for different testings.

## Environment Configuration:
### For Catkin Workspace:
You will need **Two other repository** from Fetch along side this repository to make it work:

Fetch ROS Repository: https://github.com/fetchrobotics/fetch_ros

Fetch Gazebo Repository: https://github.com/fetchrobotics/fetch_gazebo

### Required packages:
`ros-melodic-costmap-2d`
`ros-melodic-moveit*`
`ros-melodic-simple-grasping`
`ros-melodic-robot-calibration`
`ros-melodic-slam-karto`
`ros-melodic-rotate-recovery`
`ros-melodic-graft`
`ros-melodic-navfn`
`ros-melodic-clear-costmap-recovery` 
`ros-melodic-joy` 
`ros-melodic-map-server`
`ros-melodic-sick-tim`
`ros-melodic-teleop-twist-keyboard`
`ros-melodic-move-base-msgs`
`ros-melodic-openni2-launch`
`ros-melodic-rgbd-launch`
`ros-melodic-move-base`
`ros-melodic-base-local-planner`
`ros-melodic-amcl`

## Running the Code:
To start the challenge in Gazebo:
`roslaunch fetchit_challenge main.launch`

Start the perception action server:
`roslaunch  object_finder_launch object_finder.launch`

Start the navigation nodes: 
`roslaunch navigation_launch navigation.launch`

Start the manipulation nodes: 
`roslaunch manipulation_launch manipulation.launch`

If desired, place kit on pedestal with:
`rosrun gazebo_set_state set_kit_service`
then "glue" the kit to the pedestal with:
`rosservice call sticky_finger/base_link  true`

Start the coordinator:
`roslaunch coordinator_launch coordinator.launch`

