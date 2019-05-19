# The Coordinator Code Package

This is the coordinator code package. It includes all the acutal code for coordinator.

## Most Up to Date Node

The following node will be actively used, as of `2019-04-30`.

**grabe_all:** `rosrun coordinator grab_all`  
This node is responsible for: interactively proceed through all check points and proced all action.

**visit_all:** `rosrun coordinator visit_all`  
This node is responsible for: command the robot around the arena once, with user interaction for starting a navigation

## Included Nodes

Nodes here are included, but they might not be actively used for competition.

## Dependencies

Uses libraries: find_part_lib, move_base_lib, move_part_lib for perception, navigation and manipulation, respectively.