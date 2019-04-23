// fetch_cartesian_interactive_ac.cpp: 
// wsn, April, 2019

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
using namespace std;

const double GRASP_HEIGHT = 0.055;
const double PART_X_VAL = 0.625; //0.64;
const double PART_Y_VAL = 0.2; //0.059; //minus-sign error w/ object-finder?
const double APPROACH_HT = 0.15;

int main(int argc, char** argv) {
    ros::init(argc, argv, "fetch_cartesian_interactive_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    nsteps = 10;
    arrival_time = 2.0;


    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;

    //pre-pose: rosrun tf tf_echo system_ref_frame generic_gripper_frame
    //  0.435, 0.414, 0.604
    //rosrun tf tf_echo torso_lift_link generic_gripper_frame:  0.522, 0.414, 0.226
    O_des << PART_X_VAL, PART_Y_VAL, APPROACH_HT; //0.5, 0.4, 0.2; //0.3,-0.1,0.0;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation() = O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    //tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");
    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "torso_lift_link");

    ROS_INFO("requesting plan to approach pose:");
    xformUtils.printPose(tool_pose);
    ROS_INFO_STREAM("Rdes = " << endl << tool_affine.linear() << endl);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }
    /*
    int nsegs = 0;
    vector<double> arrival_times;

    tool_pose.pose.position.z = GRASP_HEIGHT; //descend to grasp pose
    ROS_INFO("requesting plan descend to grasp pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    ROS_INFO("requesting plan move depart:");

    tool_pose.pose.position.z = APPROACH_HT;
    rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }
   */
    string xyz("x");
    double ds;
    bool valid_code;
    int cart_code;
    while(true) {
        cout << "enter x, y or z code for desired Cartesian perturbation: " << endl;
        cin>>xyz;
        valid_code = true;
        if (xyz == "x") cart_code = 0;
        else if (xyz == "y") cart_code = 1;
        else if (xyz == "z") cart_code = 2;
        else {
            ROS_WARN("not a valid input: %s", xyz.c_str());
            valid_code = false;
        }
        if (valid_code) {
            cout << "enter desired displacement value (in m):  ";
            cin>>ds;

        if (cart_code == 0) tool_pose.pose.position.x += ds;
        if (cart_code == 1) tool_pose.pose.position.y += ds;
        if (cart_code == 2) tool_pose.pose.position.z += ds;
        ROS_INFO("desired tool pose: ");
        xformUtils.printPose(tool_pose);
        //arrival_times.clear();
        //note: bug in plan_cartesian_traj_qprev_to_des_tool_pose(); if no valid IK soln along path, action server crashes!
        //...first call only???
        // for single move (not multisegment), use plan_cartesian_traj_current_to_des_tool_pose
        // and TODO: find/fix this bug
        rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            //ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
    }
    }
    return 0;
}

