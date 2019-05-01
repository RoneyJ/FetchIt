# teamCaseFetchRemoteExperiments
This repository is used to store the minimal code to be run at Fetch Robotics Remote Experimentation Sessions. Each (branch/release) includes the different test conducted.

# Test of 2019-04-17
The folllowing information can also be found in the designated (branch/release).

## Goals:
**Navigation & Coordination**
1. Coordinator sends robot to various stations repeatably (Jason and Tyler). This may well require some tuning/iterations.
2. Tune k_phi, K_DISP, K_TRIP_DIST

**Test Arm Motion**
1. Have arm go through blind grasping motions, approach from above (Josh). 
2. Tune the grasp height to the table heights. 
3. Establish the grasping height of the tote handle.

**Test Vision Processing**
1. Get some images of totes

## Executable:
#### 1. ROS BAG and its variants: 
This will run the rosbag and bag the desired topics for later testing and development.
`roslaunch teamcasefetch_launchers bagger.launch`

#### 2. Coordinated Navigation Test:
This will run the robot across all station repeatedly.
`roslaunch teamcasefetch_launchers test_navigation_coordinated.launch`




## Test Summary:
*Updated 2014-04-15:*

**Our Current Test Code**

**Navigation Tuning Result**
const double accel_max = 0.1; // 0.2m/sec^2
const double speed_max = 0.2; // m/sec

**ROSBAG Location**
https://drive.google.com/drive/folders/1DJ5A_mY5UUGDql5SSb_GPcLSB_NfJDea