//Implementing open loop control of gripper control using a simple action client
//Eventually, would want an action server based approach which would be accessed by calling the member functions of a GripperController library
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "teamCase_gripper_control_test");
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", true);
	//Recheck the name of the action server
	
	control_msgs::GripperCommandGoal goal;
	ROS_INFO("Waiting for server"); //Take this out after testing
	ac.waitForServer(); 
	ROS_INFO("Connected");

	//For test purposes, just open and close
	while(ros::ok()) {
		ROS_INFO("Open");
		goal.command.position = 0.05; //Open by just 5cm
		goal.command.max_effort = 1;
		ac.sendGoal(goal);
		//Doesnt seem like the fetch gripper action server gives feedback :( 
		
		ac.waitForResult();
		
		ROS_INFO("Zero");
		goal.command.position = 0.05;
		goal.command.max_effort = 1;
		//ac.sendGoal(goal);
		//ac.waitForResult();
		ros::Duration(0.5).sleep();

	}

return 0;

}