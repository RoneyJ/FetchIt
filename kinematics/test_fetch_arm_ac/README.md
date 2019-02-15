# test_fetch_arm_ac

Simple examples to send a trajectory goal to the Fetch arm.

## Example usage
start up fetch simulation, e.g. with:
`roslaunch fetch_gazebo pickplace_playground.launch`
run the test arm-motion node with:
`rosrun test_fetch_arm_ac fetch_arm_test_action_client2`

Can also test pan/tilt of head by running:
`rosrun test_fetch_arm_ac fetch_head_test_action_client`

also, 
`rosrun test_fetch_arm_ac fetch_torso_test_action_client`

To tilt the head to 1.0 rad, run:
`rosrun test_fetch_arm_ac fetch_head_tilt_preset` 

To raise up torso lift joint to max, run:
`rosrun test_fetch_arm_ac fetch_torso_lift_preset`

To move the arm around interactively (one joint at a time), run:
`rosrun test_fetch_arm_ac fetch_arm_test_interactive`

to move the arm to a grasp pre-pose, run:
`rosrun test_fetch_arm_ac fetch_arm_pre_pose`



## Running tests/demos
    
