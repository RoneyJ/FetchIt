//pub_des_state_path_client_amcl_correction
//wsn, 4/20/19:


#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>
#include <mobot_pub_des_state/key_pose_move.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <xform_utils/xform_utils.h>

using namespace std;

//XformUtils xform_utils;

int main(int argc, char **argv) {
    ros::init(argc, argv, "visit_all");
    ros::NodeHandle n;

    ros::ServiceClient set_key_pose_client = n.serviceClient<mobot_pub_des_state::key_pose_move>("set_key_pose_index");
    while (!set_key_pose_client.exists()) {
        ROS_INFO("waiting for service from set_key_pose_index ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");

    mobot_pub_des_state::key_pose_move key_pose_move_srv;

    /**
     *  Using cin to break up operations until we have a way to determine if the operation finished successfully. 
     */

    //Start at home
    int ans;
    cout << "enter 1 to move to HOME_POSE" << endl;
    cin >> ans;

    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::HOME_POSE;
    set_key_pose_client.call(key_pose_move_srv);


    //Move to tote table
    cout << "enter 1 to move to TOTE_TABLE" << endl;
    cin >> ans;

    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::TOTE_TABLE;
    set_key_pose_client.call(key_pose_move_srv);
    
    
    //Then to gearbox table
    cout << "enter 1 to move to GEARBOX_TABLE" << endl;
    cin >> ans;

    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::GEARBOX_TABLE;
    set_key_pose_client.call(key_pose_move_srv);


    //Then to shunk table
    cout << "enter 1 to move to SHUNK_TABLE" << endl;
    cin >> ans;

    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::SHUNK_TABLE;
    set_key_pose_client.call(key_pose_move_srv);


    //Then to bolt bin
    cout << "enter 1 to move to BOLT_BIN" << endl;
    cin >> ans;

    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::BOLT_BIN;
    set_key_pose_client.call(key_pose_move_srv);


    //Then to kit dropoff
    cout << "enter 1 to move to KIT_DROPOFF" << endl;
    cin >> ans;

    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::KIT_DROPOFF;
    set_key_pose_client.call(key_pose_move_srv);

    
    ROS_INFO("done");
    return 0;
}
