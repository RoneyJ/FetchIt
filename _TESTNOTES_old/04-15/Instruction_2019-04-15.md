# Instructions/ROS Command to run:
Please execute the following commands:

## 0. To get our repository:

Add the following lines to the active.rosinstall:

    - git:
        local-name: teamCaseFetchExperiments
        uri: https://github.com/cwru-robotics/teamCaseFetchRemoteExperiments.git

## 1. Test for navigation:

Execute: `roslaunch teamcasefetch_lauchers bagger.launch`

Execute: `roslaunch teamcasefetch_lauchers test_navigation.launch`

*Retrive rosbag at: `~/teamcasefetch_lauchers/bags`*

## 2. Test for manipulation:

Execute: `roslaunch teamcasefetch_lauchers bagger.launch`

Execute: `roslaunch teamcasefetch_lauchers test_manipulation.launch`

*Retrive rosbag at: `~/teamcasefetch_lauchers/bags`*

## 3. Test for lift torso:

Execute: `roslaunch teamcasefetch_lauchers bagger.launch`

Execute: `roslaunch teamcasefetch_launchers test_head.launch`

*Retrive rosbag at: `~/teamcasefetch_lauchers/bags`*

## 4. Test for tilt head:

Execute: `roslaunch teamcasefetch_lauchers bagger.launch`

Execute: `roslaunch teamcasefetch_launchers test_torso.launch`

*Retrive rosbag at: `~/teamcasefetch_lauchers/bags`*

## 5. Test for grasp:

Execute: `roslaunch teamcasefetch_lauchers bagger.launch`

Execute: `roslaunch teamcasefetch_lauchers test_manipulation.launch`

*Retrive rosbag at: `~/teamcasefetch_lauchers/bags`*