#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fetch_fk_ik/fetch_kinematics.h>

#include <object_finder/objectFinderAction.h>
#include <gripper_interface/gripper_interface.h>


using namespace std;

class MovePart 
{
public:
	MovePart(); //constructor
        //public member fncs
        GripperInterface gripperInterface_;
	bool get_part(int part_code);
        bool place_grasped_part(int part_code, geometry_msgs::PoseStamped destination_pose);
        bool stow_grasped_part(int part_code);

private:
        //private fncs and data
        ros::NodeHandle nh_;
    	void initializeSubscribers();
    	void initializePublishers();

       //object finder:
        actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac_;
        void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
            const object_finder::objectFinderResultConstPtr& result);
        object_finder::objectFinderGoal object_finder_goal_;
        bool g_found_object_code_;
        std::vector <geometry_msgs::PoseStamped> perceived_object_poses_;

    //for arm control:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robot_arm_motion_action_client_;
    void armMotionCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result); 

    Fetch_fwd_solver fwd_solver_;
    Fetch_IK_solver ik_solver_;
    std::vector<Eigen::VectorXd> q_solns_;
    Eigen::Vector3d O_7_,O_7_des_;
    
    Eigen::Affine3d A_fwd_DH_;
    Eigen::VectorXd q_ARM_INIT_,q_ARM_INIT2_; //
    control_msgs::FollowJointTrajectoryGoal robot_arm_goal_;
    //goal message compatible with robot action server
    trajectory_msgs::JointTrajectory arm_des_trajectory_;


    //for torso:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_action_client_;
   void torsoDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result); 
    control_msgs::FollowJointTrajectoryGoal robot_torso_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory torso_des_trajectory;



    //for head:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_action_client_;
    void headDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    control_msgs::FollowJointTrajectoryGoal head_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory head_des_trajectory;


};
