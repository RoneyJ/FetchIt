# fetch_move_arm

Simple examples to send a trajectory goal to the Fetch arm.

## Example usage
start up fetch simulation, e.g. with:
`roslaunch fetchit_challenge main.launch` better:


run the test arm-motion node with:
`rosrun fetch_move_arm fetch_arm_test_action_client2`

Can also test pan/tilt of head by running:
`rosrun fetch_move_arm fetch_head_test_action_client`

also, 
`rosrun fetch_move_arm fetch_torso_test_action_client`

To tilt the head to 1.0 rad, run:
`rosrun fetch_move_arm fetch_head_tilt_preset` 

To raise up torso lift joint to max, run:
`rosrun fetch_move_arm fetch_torso_lift_preset`

To move the arm around interactively (one joint at a time), run:
`rosrun fetch_move_arm fetch_arm_test_interactive`

to move the arm to a grasp pre-pose, run:
`rosrun fetch_move_arm fetch_arm_pre_pose`

to move the arm to a pre-pose without hitting itself, run:
`rosrun fetch_move_arm fetch_arm_safe_start`



## Running tests/demos
For simulation:
`roslaunch fetchit_challenge main.launch`  

see also:
`rosrun cartesian_motion_commander fetch_cartesian_interactive_ac`  
to interactively command cartesian motions
    
