//pub_des_state_path_client:
// this node loops from table 1->2->3->4->5->1->...

#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

int table_num = 0;


geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceClient queue_client = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");

    geometry_msgs::Quaternion quat;

    while (!client.exists()) {
        ROS_INFO("waiting for service...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    mobot_pub_des_state::integer_query integer_query_srv;
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    while (true) { //repeat forever
        path_srv.request.path.poses.clear();
	cout << "Input a table number to move to: " << endl;
	cin >> table_num;

	if(table_num == 1){
		//go to table 1
		pose.position.x = 6.98; 
		pose.position.y = -2.35;
		pose.position.z = 0.0;
		quat = convertPlanarPhi2Quaternion(0);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
	}
	else if(table_num == 2){
		//go to table 2
		pose.position.x = 7.0;
		pose.position.y = -2.74;
		quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
	}
	else if(table_num == 3){
		//go to table 3
		pose.position.x = 6.03;
		pose.position.y = -2.74;
		quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
	}
	else if(table_num == 4){
		//go to table 4
		pose.position.x = 6.01;
		pose.position.y = -2.18;
		quat = convertPlanarPhi2Quaternion(-3.1416);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
	}
	else if(table_num == 5){
		//go to table 5
		pose.position.x = 6.67;
		pose.position.y = -2.18;
		quat = convertPlanarPhi2Quaternion(1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
	}

        client.call(path_srv);

        int npts = 1;
        while (npts > 0) {
            queue_client.call(integer_query_srv);
            npts = integer_query_srv.response.int_val;
            ROS_INFO("waiting... %d points left in path queue", npts);
            ros::Duration(1.0).sleep();
        }
    }
    return 0;
}
