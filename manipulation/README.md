# The Manipulation Folder

This is the Maniputlation folder area. This folder hosts all the manipulation related objects:

    Manipulation Folder
    ├── _Notes                                          # Used for storing notes throughout dev
    ├── arm_motion_action                               # Define the action message used by arm_motion_interface and generic_cartesian_planner
    ├── arm_motion_interface                            # Counterpart to the cartesian_motion_commander library
    ├── bringup_manipulation                            # Manipulation Starter
    ├── cartesian_motion_commander                      # Used by action clients of a cartMoveActionServer
    ├── fetch_arm_behavior_server                       # This node presents a cartesian action server interface
    ├── generic_cartesian_panner                        # General-purpose Cartesian-interpolation routines
    ├── gripper_interface                               # simplified gripper interface for use
    ├── joint_space_planner                             # Dynamic Library for a feedforward network to obtain the min-cost path through 
    ├── Kinematics                                      # Overall Container for arm kinematics

    ├── move_part_lib                                   # Library to simplify interactions with arm behavior server
    ├── arm_motion_interface                            # Counterpart to the cartesian_motion_commander library
    ├── arm_motion_interface                            # Counterpart to the cartesian_motion_commander library
    
    └── README.md                                       # This File!

## Command Related to Navigation

### Latest Command

1. **bringup_navigation** `roslaunch bringup_navigation bringup_navigation.launch`

### Unused Command

The following command are compiled with the main code, but might not be used during competition.

### Unimplemented Command