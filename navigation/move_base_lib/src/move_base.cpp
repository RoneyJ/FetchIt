


#include<move_base_lib/move_base.h>


#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;


geometry_msgs::Quaternion MoveBase::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

MoveBase::MoveBase() { //constructor
    path_client_ = nh_.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    queue_client_= nh_.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");
    //srv_stick.request.data = true;
    //srv_release.request.data = false;
    gearbox_table_approach_pose_.header.frame_id = "world";
    gearbox_table_approach_pose_.pose.position.x = 0.42;
    gearbox_table_approach_pose_.pose.position.y = -1.1;
    gearbox_table_approach_pose_.pose.position.z = 0;
    gearbox_table_approach_pose_.pose.orientation.x = 0;
    gearbox_table_approach_pose_.pose.orientation.y = 0;
    gearbox_table_approach_pose_.pose.orientation.z = -0.707;    
    gearbox_table_approach_pose_.pose.orientation.w = 0.707;    
    
    
    startup_pose_.header.frame_id = "world";
    startup_pose_.pose.position.x = 0;
    startup_pose_.pose.position.y = 0;
    startup_pose_.pose.position.z = 0;
    startup_pose_.pose.orientation.x = 0;
    startup_pose_.pose.orientation.y = 0;
    startup_pose_.pose.orientation.z = 0;    
    startup_pose_.pose.orientation.w = 1;    
    
    while (!path_client_.exists()) {
        ROS_INFO("waiting for path service...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected path_client_ to service");
    gazebo_state_subscriber_ = nh_.subscribe("/gazebo_fetch_pose", 1, &MoveBase::gazeboStateCallback, this); // for gazebo state 

}

void MoveBase::gazeboStateCallback(const geometry_msgs::Pose gazebo_pose) {
    //gazebo_x_ = gazebo_pose.position.x;
    //gazebo_y_= gazebo_pose.position.y;
    gazebo_pose_ = gazebo_pose;
    //double qz = gazebo_pose.orientation.z;
    //double qw = gazebo_pose.orientation.w;
    
    //gazebo_phi_= 2.0 * atan2(qz, qw);
}

bool MoveBase::move_to_location_code(int location_code, geometry_msgs::Pose &result_pose)  {
    //geometry_msgs/PoseStamped[] poses;
    geometry_msgs/PoseStamped pose;

    
    if (location_code == GEARBOX_TABLE)  {  //navigate to gearbox table
        path_srv_msg_.request.path.poses.clear();
        //poses.clear();
        pose = gearbox_table_approach_pose_;
        pose.pose.position.y+= 0.1; // back up 0.1m from table approach;
        path_srv_msg_.request.path.poses.push_back(pose);
        pose = gearbox_table_approach_pose_;
        path_srv_msg_.request.path.poses.push_back(pose);
        //path_srv_msg_.request.poses = poses;
        //send the request:
        path_client_.call(path_srv_msg_);
        wait_for_path_done();
        result_pose = gazebo_pose_;
        return true;
    }
    else {
        ROS_WARN("location code %d not recognized!",location_code);
        return false;
    }
}

void MoveBase::wait_for_path_done() {
        int npts = 1;
        while (npts > 0) {
            queue_client_.call(int_query_srv_msg_);
            ros::spinOnce();
            npts = int_query_srv_msg_.response.int_val;
            ROS_INFO("waiting... %d points left in path queue", npts);
            ros::Duration(1.0).sleep();
        }
}

/*  //Jason's code:
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
	//go to table 1
		 pose.position.x = 0.72; 
        	pose.position.y = 0.38;
       	 	pose.position.z = 0.0;
		quat = convertPlanarPhi2Quaternion(0);
        	pose.orientation = quat;
        	pose_stamped.pose = pose;
        	path_srv.request.path.poses.push_back(pose_stamped);
		// pose 1.1
 		pose.position.x = 0.721; 
 		pose.position.y = 0.38;
 		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);

	
       //go to table 2
                quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose.position.x = 0.61;
		pose.position.y = -1.13;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
		//pose 2.1
		pose.position.y = -1.131;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);

	//go to table 3
                quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose.position.x = -0.72;
		pose.position.y = -1.0;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
		//pose 3.1
                pose.position.y = -1.01;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
       //go to table 4
                quat = convertPlanarPhi2Quaternion(-3.1415);
		pose.orientation = quat;
		pose.position.x = -0.98;
		pose.position.y = 0.27;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
		//pose 4.1
		pose.position.x = -0.981;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
       //go to table 5
		quat = convertPlanarPhi2Quaternion(-3.1415);
		pose.orientation = quat;
		pose.position.x = -0.24;
		pose.position.y = 0.26;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
		//pose 5.1
		pose.position.y = 0.261;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);

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
*/