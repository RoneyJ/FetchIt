# coordinator_launch

package to launch the coordinator

## Example usage
Should first start the simulator (or robot):
`roslaunch worlds Fetch_kit.launch`

Start the manipulation subsystem:
`roslaunch manipulation_launch manipulation.launch`

Start the perception subsystem:
`roslaunch  object_finder_launch object_finder.launch`

Start the navigation nodes: 
`roslaunch navigation_launch navigation.launch`

Start the coordinator:
`roslaunch coordinator_launch coordinator.launch`

## Running tests/demos
see subsystem folders--manipulation, perception, navigation--for how to run subsystem tests
