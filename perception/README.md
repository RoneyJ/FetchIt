# The Perception Folder

This is the Perception folder area. This folder hosts all the vision related objects:

    Coordinator Folder
    ├── bringup_vision                                  # Vision Startup Command
    ├── triad_marker                             # Package used for creating marker for display
    ├── object_finder                                   # The object finder folder for all object finder code
    ├── object_finder_lib                               # The obejct finder library for used by coordinator
    └── README.md                                       # This File!

## Command Related to Coordinator

### Latest Command

1. **bringup_vision** `roslaunch bringup_vision bringup_vision.launch`
2. **object_finder_main** `rosrun object_finder object_finder_main`

### Unused Command

The following command are compiled with the main code, but might not be used during competition.

**example_object_finder_action_client:** `rosrun object_finder example_object_finder_action_client`

### Unimplemented Command
