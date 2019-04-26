/** grab_tote.cpp
 * twa16
 * 4/24/19
 * 
 * File for FetchIt! to test:
 * - navigating the robot to the tote table,
 * - seeing a tote,
 * - and picking it up.
 */

#include <ros/ros.h>
#include <move_part_lib/move_part.h>                // Manipulation
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation
#include <object_finder_lib/object_finder.h>        // Perception

using namespace std;

int main(int argc, char** argv) {
    /* Node Setup */
    // Initialize this node with a standard node handle
    ros::init(argc, argv, "grab_tote");
    ros::NodeHandle nh;

    // Create Manipulation and Perception objects
    MovePart movePart;
    FindPart findPart;

    // Connect to Navigation service
    ros::ServiceClient set_key_pose_client = nh.serviceClient<mobot_pub_des_state::key_pose_move>("set_key_pose_index");
    while (!set_key_pose_client.exists()) {
        ROS_INFO("Waiting for service from set_key_pose_index ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Connected client to set_key_pose_index service");


    /* Navigation */
    // Manual pause for testing
    int ans;
    cout << "Enter 1 to move to TOTE_TABLE" << endl;
    cin >> ans;

    // Create Nagivation service message
    mobot_pub_des_state::key_pose_move key_pose_move_srv;

    // Set desination to the tote table
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::TOTE_TABLE;

    ROS_INFO("Attempting navigation to tote table");
    set_key_pose_client.call(key_pose_move_srv);


    /* Perception */
    // Manual pause for testing
    cout << "Enter 1 to look for totes on the talbe" << endl;
    cin >> ans;

    // Vector to contain Perceived parts
    std::vector <geometry_msgs::PoseStamped> part_poses;

    // Set Tote part code
    int partCode = 0;

    ROS_INFO("Attempting to locate tote(s) on table");
    bool success = findPart.find_part(partCode, part_poses);

    int num_parts = part_poses.size();
    if (num_parts < 1) {
        ROS_WARN("DID NOT FIND ANY PARTS; QUITTING");
        return 0;
    }

    ROS_INFO("Found %d parts", num_parts);
    for (int i=0; i < num_parts; i++) {
        ROS_INFO_STREAM(part_poses[i] << endl);
    }

    // Choose the first part
    geometry_msgs::PoseStamped source_pose = part_poses[0];
    ROS_INFO_STREAM("Chosen part pose " << source_pose << endl);
    ROS_INFO("Triad pose: ");
    findPart.display_triad(source_pose);


    /* Manipulation */
    // Manual pause for testing
    // TODO uncomment following lines once tote pose is found
/*    cout<<"Enter 1 to attempt grasp: ";
    cin>>ans;

    ROS_INFO("Attempting to grasp chosen part");
    success = movePart.get_part(partCode, source_pose);

    ROS_INFO("Attempting to move arm to preset");
    success = movePart.preset_arm();
*/
    // TODO uncomment following lines once arm function exists
    // ROS_INFO("Attempting to place kit");
    // success = movePart.place_kit()? function to place kit


    /* END */
    ROS_INFO("done");
    return 0;
}
