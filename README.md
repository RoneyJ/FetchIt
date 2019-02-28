# teamCaseFetch
code for TeamCase Fetch competition
see docs from Fetch here:
https://github.com/fetchrobotics/docs/tree/master/source

This code is organized in three major areas: navigation, perception and manipulation.
See READMEs in these directories regarding their use and how to run subsystem tests.

The coordinator package is responsible for integrating the above.  This is aided by libraries in
each subsystem that simplify the interfaces to them, including move_base_lib, move_part_lib and object_finder_lib.

A typical sequence would consist of: invoking navigation (e.g. to a pre-coded zone), invoking object finding 
(using object codes) and object manipulation (with part codes and location  codes or specified poses).

## running the code:
To start up the code, launch the Gazebo simulation with:
`roslaunch worlds Fetch_kit.launch`

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
