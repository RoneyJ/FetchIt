# teamCaseFetchRemoteExperiments
This repository is used to store the minimal code to be run at Fetch Robotics Remote Experimentation Sessions. Each (branch/release) includes the different test conducted.

# Test of 2019-04-15
The folllowing information can also be found in the designated (branch/release).
## Goals:
1. Obtain (ROSBAG) real robot Lidar, Odometry data
2. Try running the robot around with elevated torso and extended (pre-pose) arm
3. Send the robot through a sequence of moves to approach the various tables
4. Try some type of manipulation, starting with "mime" of pick/place

## Executable:
#### 1. ROS BAG: 
This will run the rosbag and bag the desired topics for later testing and development.

`roslaunch teamcasefetch_lauchers bagger.launch`

#### 2. Navigation Test (*BASE MOTION INVOLVED*): 
This command will send the robot go around each stations point by point. 

`roslaunch teamcasefetch_lauchers test_navigation.launch`

#### 3. Torso Test (*TORSO MOTION INVOLVED*)
This command will command the robot to raise the torso.

`roslaunch teamcasefetch_lauchers test_torso.launch`

#### 4. Head Tilt Test (*HEAD MOTION INVOLVED*)
This command will command the head to tilt.

`roslaunch teamcasefetch_lauchers test_head.launch`

#### 5. Arm Motion Test (*ARM MOTION INVOLVED*)
This command will move the arm to extended position.

`roslaunch teamcasefetch_lauchers test_manipulation.launch`

## Test Summary:
*Updated 2014-04-15:*

**Our Current Test Code**
1. Bagger launch file did not works. Blame Frank for that (He derped).
2. Navigation somewhat works, but the arena is slightly different than gazebo, some what success. Odometry seems working much more better than gazebo. 
3. Arm test concluded with some wired extra motion, it is a bit dangerous. Need more investiagtion into ROS bag.

**Others**
1. Camera correspondence: 
    
    `Camera 0: Above the Screw Bin`
    
    `Camera 1: Above Gear Box bin`

    `Camera 2: Above Shower Bins`

    `Camera 3: Above Shunk Machine`

**Regarding Arm Motion** (Not for CWRU but general knowledge)
1. Moveit is unpredictable but it performs collision checking
2. Use moveit to record motion, do collision checking
3. Do not use the default moveit, as its planner sucks
4. Use trajectory to do finner tuning

**ROSBAG Location**
https://drive.google.com/drive/folders/1DJ5A_mY5UUGDql5SSb_GPcLSB_NfJDea


## TODO for next TEST:
*Updated 2014-04-15:*

**Package Maintainance:**
1. Rosinstall Update, need AMCL, Gmapping, Need Seperate packages
2. Update the Fetch Simulation we have

**For Navigation:**
1. Need to create a map from what we have
2. Prepare for SLAM on remote test (Need launch file, dependency)
3. Test AMCL workability. (Need add dependency.)
4. Need to have a lin_steering_w_odom test
5. Need to have a lin_steering_w_amcl test

**For Manipulation:**
1. Slow down the current joint_state command speed
2. Test moving joint angle one by one
3. Prep rqt_plot for joint angle bag

**For Coordinator**
1. Test the concept of moving the robot base while the torso is lifted
2. Test the robot driving with arm in default position

