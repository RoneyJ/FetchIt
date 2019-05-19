# The mobot_pub_des_state Package

This package illustrates a desired-state publisher that creates and publishes sequences of states that are dynamically feasible and which lead a robot through a sequence of subgoals, treated as a polyline.
The publisher exploits the traj_builder library to construct dynamically-feasible trajectories.
It accepts subgoals via the service "append_path_queue_service", which appends subgoals to the current queue of subgoals.

The system can also perform e-stop utilizing service call:

* The service *estop_service* invokes an e-stop state, causing the robot to come to a halt with a dynamically-feasible trajectory.
  * `rosservice call estop_service`
* The service *clear_estop_service* allows the robot to resume visiting subgoals.
  * `rosservice call flush_path_queue_service`
* The current path queue can be flushed via the service *flush_path_queue_service*.
  * `rosservice call clear_estop_service`

When there are no subgoals left in the queue, the robot halts at its lasts subgoal.
It will resume motion if/when new subgoals are added to the queue.

## Most Up to Date Node

The following node will be actively used, as of `2019-05-19`.

**open_loop_controller** `rosrun mobot_pub_des_state open_loop_controller`
This node is responsible for controlling the system openloop. 

**mobot_pub_des_state** `rosrun mobot_pub_des_state mobot_pub_des_state`
This node is responsible publishing the desired state (main file). It is modified with a new service to reset the current pose. Subsequent motion commands are trajectories with respect to the new start pose

**pub_des_state_path_client_amcl_correction** `rosrun mobot_pub_des_state pub_des_state_path_client_amcl_correction`
This node is responsible for corrections based on AMCL base_link with respect to map, so one can clear accumulated errors and drive the robot using cmd_vel (via open_loop_controller in this package) 

## Included Nodes

Then can command motions to key poses a client, or from command line with:
`rosservice call set_key_pose_index 2`  (pick a number of code to send for key-pose index)  

## Structure

    mobot_pub_des_State Folder
    ├── srv                                           # Service Message structure
    │   └── integer_query.srv                       # Action Server Message Definition
    ├── src                                           # source code folder
    │   ├── object_finder_main.cpp                    # Main file for starting the object finder action server
    │   *****MIX Compile*****                         # The following file are actually considered one file.
    │   ├── object_finder_as.cpp                      # Main file for Object Finder Action Server
    │   ├── object_finder_helper_fncs.cpp             # Helper function for object_finder_as
    │   ├── bolt_finder_fncs.cpp                      # Seperation file for finding bolt part. Combined towards Object_finder_as
    │   ├── gear_finder_fncs.cpp                      # Seperation file for finding gear part. Combined towards Object_finder_as
    │   ├── bolt_finder_fncs.cpp                      # Seperation file for finding bolt. Combined towards Object_finder_as
    │   ├── gearbox_finder_fncs.cpp                   # Seperation file for finding gearbox. Combined towards Object_finder_as
    │   ├── template_tote_finder.cpp                  # Seperation file for finding tote. Combined towards Object_finder_as
    │   *****END MIX COMPILE*****
    │   └── example_object_finder_action_client.cpp   # A test file for object finder
    ├── package.xml                                   # Package XML
    ├── CMakeList.txt                                 # CMake List
    └── README.md                                     # This File!

## Dependencies

This package generates and recieves drive commands
