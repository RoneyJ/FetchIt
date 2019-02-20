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
	bool isStalled();
	bool graspObject(std::string);
	bool graspObject();
	bool releaseObject(std::string);
	bool releaseObject();
private:
	std::map<std::string,double> part_width_mappings_;
	const double MAX_EFFORT_ = 1.0, MAX_OPEN_ = 0.1, OPEN_EFFORT_ = 1.0, DEFAULT_GRASP_WIDTH_ = 0.1 ;
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac_;
	control_msgs::GripperCommandGoal goal_;
	control_msgs::GripperCommandResult result_;


};
#endif

