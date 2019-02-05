# teamCaseFetch
code for TeamCase Fetch competition
see docs from Fetch here:
https://github.com/fetchrobotics/docs/tree/master/source

see package test_fetch_arm_ac for an example of how to populate and send trajectory goals to the arm.

Can teleoperate the base with:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/base_controller/command
note: topic for base motion is /base_controller/command (not cmd_vel)

