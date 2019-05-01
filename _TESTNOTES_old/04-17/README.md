# teamCaseFetchRemoteExperiments
This repository is used to store the minimal code to be run at Fetch Robotics Remote Experimentation Sessions. Each (branch/release) includes the different test conducted.

# Test of 2019-04-17
The folllowing information can also be found in the designated (branch/release).

## Goals:
**Test Mapping**
1. Robot is centered in the arena and facing the Shunk table.
2. Robot is rebooted from this pose so that the odometry is 0,0,0
3. Do a slow spin-in-place

**Test Arm Motion**
1. Do new arm move tests...slower, more carefully

**Test tune linear steering**
1. do K_phi lin-steering feedback tests

## Executable:
#### 1. ROS BAG and its variants: 
This will run the rosbag and bag the desired topics for later testing and development.

`roslaunch teamcasefetch_launchers bagger.launch`

#### 2. test_generate_map: 
This command will command the robot to spin in place and then use slam to generate a map and save the map. 

`roslaunch teamcasefetch_launchers test_generate_map.launch`

### 3. Linear steering testing:

roslaunch teamcasefetch_launchers test_back_and_forth_w_odom_feedback.launch

## Test Summary:
*Updated 2014-04-15:*

**Our Current Test Code**
1. Odom seems reliable than gazebo.
2. accel_max, speed_max tuned.
3. Arm motion seems contained.

**pub_des_state.h Result**
const double accel_max = 0.1; // 0.2m/sec^2
const double speed_max = 0.2; // m/sec

**ROSBAG Location**
https://drive.google.com/drive/folders/1DJ5A_mY5UUGDql5SSb_GPcLSB_NfJDea