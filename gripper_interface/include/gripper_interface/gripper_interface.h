#ifndef GRIPPER_INTERFACE_H_
#define GRIPPER_INTERFACE_H_
#include <map>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>




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

private:
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

