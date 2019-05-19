# The Manipulation Folder

This is the Maniputlation folder area. This folder hosts all the manipulation related objects:

    Manipulation Folder
    ├── _Notes                                    # Used for storing notes throughout dev
    ├── arm_motion_action                         # Define the action message used by arm_motion_interface and generic_cartesian_planner
    ├── arm_motion_interface                      # Counterpart to the cartesian_motion_commander library
    ├── **bringup_manipulation**                  # Manipulation Starter
    ├── cartesian_motion_commander                # Used by action clients of a cartMoveActionServer
    ├── fetch_arm_behavior_server                 # This node presents a cartesian action server interface
    ├── generic_cartesian_panner                  # General-purpose Cartesian-interpolation routines
    ├── gripper_interface                         # simplified gripper interface for use
    ├── joint_space_planner                       # Dynamic Library for a feedforward network to obtain the min-cost path through
    ├── Kinematics                                # Overall Container for arm kinematics
    |   ├── fetch_fk_ik                           # forward and inverse kinematics library for Fetch robot arm
    |   └── fk_ik_virtual                         # specifies a virtual fk/ik library  to generalize different FK/IK specialized functions
    ├── **move_part_lib**                         # Library to simplify interactions with arm behavior server
    ├── **fetch_move_arm**                        # four nodes of: Safe_start, pre_pose_only, Lift Torso, Tilt Head
    └── README.md                                 # This File!

## Command Related to Manipulation

### Latest Command

1. **bringup_manipulation** `roslaunch bringup_manipulation bringup_manipulation.launch`

### Unused Command

The following command are compiled with the main code, but might not be used during competition.

### Unimplemented Command