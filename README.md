# teamCaseFetch
code for TeamCase Fetch competition
see docs from Fetch here:
https://github.com/fetchrobotics/docs/tree/master/source

NOTE:  modified fetch/fetch_gazebo/robots/fetch.gazebo.xacro to add sticky-fingers plugin
saved this in: teamCase_fetch_model/fetch.launch.xml
modified our launch file to refer to this model:
roslaunch worlds Fetch_kit.launch


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



