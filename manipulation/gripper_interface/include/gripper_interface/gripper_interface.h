#ifndef GRIPPER_INTERFACE_H_
#define GRIPPER_INTERFACE_H_
#include <map>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <std_srvs/SetBool.h>




class GripperInterface 
{
public:
	GripperInterface();
	bool graspObject(std::string, double);
	bool graspObject(std::string);
	bool graspObject(); 
	bool releaseObject(std::string, double); 
	bool releaseObject(double);
	bool releaseObject();
        ros::ServiceClient sticky_finger_client;// = n.serviceClient<std_srvs::SetBool>("/sticky_finger/r_gripper_finger_link");
        std_srvs::SetBool srv_stick,srv_release;
private:
        ros::NodeHandle nh_; 
    
	std::map<std::string,double> part_width_map_ = 
	{
		{"dummy_part", 0.01}
	}; //more parts go here as we find out about them

<<<<<<< HEAD
<<<<<<< HEAD
	const double MAX_EFFORT_ = 10.0, OPEN_POSITION_ = 0.115, OPEN_EFFORT_ = 1.0, DEFAULT_GRASP_WIDTH_ = 0.1, CLOSE_POSITION_ = 0.0 ;
=======
	const double MAX_EFFORT_ = 100, OPEN_POSITION_ = 0.115, OPEN_EFFORT_ = 100, DEFAULT_GRASP_WIDTH_ = 0.0 ;
>>>>>>> bc65c4c1d0319a4bdbfb872b5d58357f946880b0
=======
	const double MAX_EFFORT_ = 1.0, OPEN_POSITION_ = 0.1, OPEN_EFFORT_ = 1.0, DEFAULT_GRASP_WIDTH_ = 0.1 ;
>>>>>>> parent of dad0a35... functions for tote dropoff/pickup/recovery
	
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac_;
	control_msgs::GripperCommandGoal goal_;
	control_msgs::GripperCommandResult result_;
	
	bool waitForGrasp(double timeout);
	bool isGrasping();
	bool waitForRelease(double timeout);


};
#endif

