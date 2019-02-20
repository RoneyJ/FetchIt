#include <gripper_interface/gripper_interface.h>

GripperInterface::GripperInterface() : ac_("gripper_controller/gripper_action", true) {
	ROS_INFO("Connecting to gripper action server");
	ac_.waitForServer();
	//open the gripper and be ready
	//serves a second purpose of getting gripper status through "result" continously to check gripper status
	//"result will only be sent after a first goal request "
	goal_.command.position = MAX_OPEN_;
	goal_.command.max_effort = OPEN_EFFORT_;
	ac_.sendGoal(goal_);
	ROS_INFO("Connected!");
}

bool GripperInterface::graspObject() {
	result_ = *ac_.getResult();
	//check if already grasping something
	goal_.command.position = DEFAULT_GRASP_WIDTH_;
	goal_.command.max_effort = MAX_EFFORT_;
	ac_.sendGoal(goal_);
}