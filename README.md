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

To start up the code, launch the Gazebo simulation with:
`roslaunch worlds Fetch_kit.launch`

Start the perception action server:
`rosrun object_finder object_finder_as`


Start the manipulation action server: (move these to a single launch file)
`roslaunch manipulation_launch manipulation.launch`

should include:
`roslaunch fetch_arm_behavior_server fetch_static_transforms.launch`
`rosrun fetch_arm_behavior_server fetch_cart_move_as`

Start the navigation nodes: (CREAT ME!)
`roslaunch perception_launch perception.launch`

Start the coordinator:
`roslaunch coordinator_launch coordinator`




see package test_fetch_arm_ac for an example of how to populate and send trajectory goals to the arm.

Can teleoperate the base with:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/base_controller/command
note: topic for base motion is /base_controller/command (not cmd_vel)

see README under navigation folder for how to control the robot's base motion

Initializations:  likely want to tilt head down, so can view a table top.
likely want to elevate the torso to max
likely want to pre-position the arm it is out of way of camera, but is easy to swing
into position to grasp from above.
Can do this with:
`rosrun test_fetch_arm_ac fetch_head_tilt_preset` 
`rosrun test_fetch_arm_ac fetch_torso_lift_preset`
`rosrun test_fetch_arm_ac fetch_arm_pre_pose`
with last node, can interactively move the arm joints as well, and monitor IK solutions



