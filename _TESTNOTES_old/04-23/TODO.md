# TODO for next TEST:
*Updated 2014-04-21:*

**For Navigation & Coordinator (Jason and Tyler):**
1. load a map (Frank)
2. Startup With AMCL (Frank)
3. must start up the open-loop steering node: `rosrun mobot_pub_des_state open_loop_controller`
4. must start up the pub_des_state node: `rosrun mobot_pub_des_state mobot_pub_des_state`
5. must start up a new node: `rosrun mobot_pub_des_state pub_des_state_path_client_amcl_correction`
6. make a client of this service to send the robot to the different key poses (Tyler)
7. you should fill in lines 60-82 of the function set_hardcoded_poses(Jason) refer: https://github.com/cwru-robotics/teamCaseFetch/blob/master/navigation/mobot_pub_des_state/src/pub_des_state_path_client_amcl_correction.cpp
8. create another service to get the value of: `g_got_new_key_pose` which is "false" when the move is done.  OR, we could turn this into an action server instead.
9. have the coordinator send the robot to its sequence of key poses
10. Jason define short moves near each table to coerce the robot to change its heading to point towards the table. This extra move could be  built into the new node, so Tyler does not have to command this.

*To command robot to go to key pose, do:* `rosservice call set_key_pose_index 2`

*To find key pose defination, refer here:* https://github.com/cwru-robotics/teamCaseFetch/blob/master/navigation/mobot_pub_des_state/srv/key_pose_move.srv

**For Manipulation: (Josh)**
1. Test Cartesian jog program.  
2. Use Catesian jog to establish grasp heights.

**For Image Processing: (Chris, Devansh)**
1. Make sure we get some images of totes.  
2. Test Grasp on Friday

**Package Maintenance (Frank):**
1. Need a launch file template bring up Map, AMCL, open_loop_controller, mobot_pub_des_state, pub_des_state_path_client_amcl_correction
