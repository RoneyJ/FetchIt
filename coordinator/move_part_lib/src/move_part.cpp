#include <move_part_lib/move_part.h>

MovePart::MovePart() { //constructor

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


