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
### Using Docker Provided by Fetch:
Please refer to: https://github.com/fetchrobotics/fetch_gazebo/issues/75

*NOTE: Fetch uses catkin overlay, and have a repository named stable and active.*

### CWRU students:
**Make sure you git pull the above mentioned repositories alongside Team Case's repository.**

Fetch ROS Repository: https://github.com/fetchrobotics-gbp/fetch_ros-release.git

Fetch Gazebo Repository: https://github.com/fetchrobotics/fetch_gazebo.git

Fetch Robot Controller Repository (0.6.0-0): https://github.com/fetchrobotics/robot_controllers-release.git

RGBD Launch Repository: https://github.com/ros-gbp/rgbd_launch-release.git

ROS Controllers Repository: https://github.com/ros-gbp/ros_controllers-release.git



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

