# The bringup_manipulation launcher Package

This is the package for bringing up the launcher necessity for manipulation. It includes all the node needed for manipulation pre-start.

## Most Up to Date Node

The following node will be actively used, as of `2019-05-19`.

**bringup_navigation:** `rosrun bringup_manipulation bringup_manipulation.launch`  
This node is responsible for: initiate all the manipulation processing related packages. It includes:

1. **SET torso lift:** `rosrun fetch_move_arm fetch_torso_lift_preset`
2. **SET head tilt:** `rosrun fetch_move_arm fetch_head_tilt_preset`
3. **SET arm start:** `rosrun fetch_move_arm fetch_arm_safe_start`
4. 
## Included Nodes

Nodes here are included, but they might not be actively used for competition.

## Dependencies

Uses libraries:  respectively.
Provides necessity for coordinator