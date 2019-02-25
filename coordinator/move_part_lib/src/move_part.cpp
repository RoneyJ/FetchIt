#include <move_part_lib/move_part.h>

MovePart::MovePart() :
robot_arm_motion_action_client_("/arm_controller/follow_joint_trajectory",true),
torso_action_client_("/torso_controller/follow_joint_trajectory", true),
head_action_client_("/head_controller/follow_joint_trajectory", true),
object_finder_ac_("object_finder_action_service", true)
{ //constructor
  ROS_INFO("MovePart constructor");

}

void MovePart::initializeSubscribers() {

}

void MovePart::initializePublishers() {

}


void MovePart::objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
            const object_finder::objectFinderResultConstPtr& result) {

}

void MovePart::armMotionCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {

}

void MovePart::torsoDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {

}

void MovePart::headDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {

}

bool MovePart::get_part(int part_code) {
return false;
}

bool MovePart::place_grasped_part(int part_code, geometry_msgs::PoseStamped destination_pose){
return false;
}


bool MovePart::stow_grasped_part(int part_code) {
 return false;
}


