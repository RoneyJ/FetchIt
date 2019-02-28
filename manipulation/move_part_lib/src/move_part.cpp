#include <move_part_lib/move_part.h>

MovePart::MovePart() :
robot_arm_motion_action_client_("/arm_controller/follow_joint_trajectory",true),
torso_action_client_("/torso_controller/follow_joint_trajectory", true),
head_action_client_("/head_controller/follow_joint_trajectory", true)
//object_finder_ac_("/object_finder_action_service", true)
{ //constructor
  ROS_INFO("MovePart constructor");

    b_des_ << 0, 0, -1;
    n_des_ << 1, 0, 0;
    t_des_ = b_des_.cross(n_des_);

    R_gripper_down_.col(0) = n_des_;
    R_gripper_down_.col(1) = t_des_;
    R_gripper_down_.col(2) = b_des_;            
}

void MovePart::initializeSubscribers() {

}

void MovePart::initializePublishers() {

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

bool MovePart::preset_arm() {
    cart_motion_commander_.plan_jspace_traj_current_to_waiting_pose(2, 2.0); 
    cart_motion_commander_.execute_planned_traj();
    ros::Duration(2.0).sleep();// wait for execution
}

bool MovePart::move_to_dropoff_kit1() {
    cart_motion_commander_.plan_jspace_traj_current_to_kit_dropoff1(2, 2.0); 
    cart_motion_commander_.execute_planned_traj();
    ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

bool MovePart::move_to_dropoff_kit2() {
    cart_motion_commander_.plan_jspace_traj_current_to_kit_dropoff2(2, 2.0); 
    cart_motion_commander_.execute_planned_traj();
    ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

bool MovePart::move_to_dropoff_kit3() {
    cart_motion_commander_.plan_jspace_traj_current_to_kit_dropoff3(2, 2.0); 
    cart_motion_commander_.execute_planned_traj();
    ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

//specified source_pose should use torso_lift_link frame to specify part pose
//THIS FNC SHOULD TRANSFORM from  source reference to torso_lift_link reference frame...
// NOT YET IMPLEMENTED, SO USER BEWARE
// part major axis should be x-axis; gripper will be oriented s.t. line between fingers
// is perpendicular to part major axis
//ALSO, derive desired gripper approach orientation from part orientation...NOT DONE YET!!!
bool MovePart::get_part(int part_code, geometry_msgs::PoseStamped source_pose) {

    //O_des_ << PART_X_VAL, PART_Y_VAL, APPROACH_HT; //0.5, 0.4, 0.2; //0.3,-0.1,0.0;
    //O_des_ << perceived_object_pose.pose.position.x,perceived_object_pose.pose.position.y,APPROACH_HT;
    O_des_ << source_pose.pose.position.x,source_pose.pose.position.y,source_pose.pose.position.z+APPROACH_CLEARANCE;
 
    //FIX ME!!!  need correct orientation
    tool_affine_.linear() = R_gripper_down_;
    tool_affine_.translation() = O_des_;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    //tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");
    tool_pose_ = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine_, "torso_lift_link");  
    bool traj_is_valid = false;
    int rtn_code;

    int nsteps = 10;
    double arrival_time = 2.0;
    int rtn_val;

    ROS_INFO("requesting plan to approach pose:");
    rtn_val = cart_motion_commander_.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose_);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander_.execute_planned_traj();
        //ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    tool_pose_.pose.position.z = GRASP_HEIGHT; //descend to grasp pose
    ROS_INFO("requesting plan descend to grasp pose:");
    xformUtils.printPose(tool_pose_);
    rtn_val = cart_motion_commander_.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose_);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander_.execute_planned_traj();
        //ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        return false;
    }    
    gripper_interface_.graspObject();
    ros::Duration(3.0).sleep();  //have to wait on gripper; get some feedback??
    
       tool_pose_.pose.position.z = source_pose.pose.position.z+APPROACH_CLEARANCE; //descend to grasp pose
    ROS_INFO("requesting plan depart with grasped object:");
    xformUtils.printPose(tool_pose_);
    rtn_val = cart_motion_commander_.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose_);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander_.execute_planned_traj();
        //ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }  
        
        
return true;
}

bool MovePart::place_grasped_part(int part_code, geometry_msgs::PoseStamped destination_pose){
return false;
}


bool MovePart::stow_grasped_part(int part_code) {
 return false;
}


