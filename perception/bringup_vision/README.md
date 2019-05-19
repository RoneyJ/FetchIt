# The bringup_vision launcher Package

This is the package for bringing up the launcher necessity for vision. It includes all the node needed for vision pre-start.

## Most Up to Date Node

The following node will be actively used, as of `2019-05-19`.

**bringup_vision:** `rosrun bringup_vision bringup_vision.launch`  
This node is responsible for: initiate all the vision processing related packages. It includes:

1. **Object Finder Main:** `rosrun obejct_finder object_finder_main`
2. **Triad Display BGTASK:** `rosrun triad_marker triad_display`

## Included Nodes

Nodes here are included, but they might not be actively used for competition.

## Dependencies

Uses libraries: object_finder, Object_finder_lib, triad_marker, respectively.