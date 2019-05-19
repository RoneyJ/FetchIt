# The bringup_navigation launcher Package

This is the package for bringing up the launcher necessity for navigation. It includes all the node needed for navigation pre-start.

## Most Up to Date Node

The following node will be actively used, as of `2019-05-19`.

**bringup_navigation:** `rosrun bringup_navigation bringup_navigation.launch`  
This node is responsible for: initiate all the navigation processing related packages. It includes:

1. **AMCL:** `rosrun amcl amcl scan:=/base_scan`
2. **Map Server:** `rosrun map_server map_server $(find mapper)/real_map/map_main.yaml`
3. **Open Loop Controller:** `rosrun mobot_pub_des_state open_loop_controller`
4. **mobot_pub_des_state:** `rosrun mobot_pub_des_state mobot_pub_des_state`
5. **pub_des_state_path_client_amcl_correction:** `rosrun mobot_pub_des_state pub_des_state_path_client_amcl_correction`

## Included Nodes

Nodes here are included, but they might not be actively used for competition.

## Dependencies

Uses libraries: map, amcl, pubdes state respectively.
Provides necessity for coordinator