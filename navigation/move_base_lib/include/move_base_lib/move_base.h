#ifndef MOVE_BASE_H_
#define MOVE_BASE_H_
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

const int GEARBOX_TABLE = 1;
const int BOLT_TABLE = 2;
const int KIT_PICKUP_TABLE= 3;
const int KIT_DROPOFF_TABLE = 4;
const int GEAR_TABLE = 5;

class MoveBase 
{
public:
	MoveBase();
        geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);


        ros::ServiceClient sticky_finger_client;// = n.serviceClient<std_srvs::SetBool>("/sticky_finger/r_gripper_finger_link");
        std_srvs::SetBool srv_stick,srv_release;
private:
        ros::NodeHandle nh_; 
    ros::ServiceClient path_client;// = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceClient queue_client;// = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");
    
	std::map<std::string,double> part_width_map_ = 
	{
		{"dummy_part", 0.01}
	}; //more parts go here as we find out about them

	const double MAX_EFFORT_ = 1.0, OPEN_POSITION_ = 0.1, OPEN_EFFORT_ = 1.0, DEFAULT_GRASP_WIDTH_ = 0.1 ;
	
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac_;
	control_msgs::GripperCommandGoal goal_;
	control_msgs::GripperCommandResult result_;
	
	bool waitForGrasp(double timeout);
	bool isGrasping();
	bool waitForRelease(double timeout);


};
#endif
