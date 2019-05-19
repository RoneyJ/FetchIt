# triad_marker
This node illustrates how to display markers in rviz. 
The service rviz_marker_svc expects a floating-point number, which is uses to 
set the height of a horizontal plane of markers.

The node triad_display subscribes to topic "triad_display_pose" to receive stamped poses, then
constructs a triad of markers to display 3 axes corresponding to the received pose.
To view, add a Marker item in rviz and set the topic to /triad_display

## Example usage
For the triad display:
`rosrun rviz rviz`
add marker item, topic /triad_display
`rosrun triad_marker triad_display`
