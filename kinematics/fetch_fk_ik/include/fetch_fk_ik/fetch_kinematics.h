/* 
 * File:   fetch_kinematics.h
 * Author: wsn
 *
 * Created Feb, 2019
 */

/*
fetch params:
link	a	d	alpha	q_offset
1	0.117	0.06	pi/2	-pi/2 	lower=\"-1.6056\" upper=\"1.6056\
2	0	0	-pi/2	0	lower=\"-1.221\" upper=\"1.518\
3	0	d3	-pi/2	0	continuous
4	0	0	pi/2	0	lower=\"-2.251\" upper=\"2.251
5	0	d5	-pi/2	0	continuous
6	0	0	pi/2	0	lower= -2.16\" upper=\"2.16
7	0	d7	pi	0	continuous


vel limits, 1.256, 1.454, 1.571, 1.521, 1.571, 2.268, 2.268

<joint name=\"shoulder_pan_joint\"\
  \ type=\"revolute\">\n    <origin rpy=\"0 0 0\" xyz=\"0.119525 0 0.34858\"/>\n \
  \   <parent link=\"torso_lift_link\"/>\n    <child link=\"shoulder_pan_link
<joint name=\"shoulder_lift_joint\"\
  \ type=\"revolute\">\n    <origin rpy=\"0 0 0\" xyz=\"0.117 0 0.0599999999999999\"\
  />\n    <parent link=\"shoulder_pan_link\"/>\n    <child link=\"shoulder_lift_link

 choose base frame consistent with shoulder_pan_link
 make fk consistent with:
 rosrun tf tf_echo shoulder_pan_link generic_gripper_frame (w/ origin same as gripper_link, but reoriented)


*/

#ifndef FETCH_IK_H
#define	FETCH_IK_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>


const int NJNTS=7;
const int MAX_SOLNS_RETURNED=100; //tune me!
                                

// a fwd kin solver...
// list DH params here

const double DH_a1=0.117;
const double DH_a2=0.0;
const double DH_a3=0.0;
const double DH_a4=0.0;
const double DH_a5=0.0;
const double DH_a6=0.0;
const double DH_a7=0.0;


const double DH_d1 = 0.06; //
const double DH_d2 = 0.0;
const double DH_d3 = 0.352;
const double DH_d4 = 0.0;
const double DH_d5 = 0.3215;
const double DH_d6 = 0.0;
const double DH_d7 = 0.30495;

//robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
const double DH_alpha1 = M_PI/2.0;
const double DH_alpha2 = -M_PI/2.0;
const double DH_alpha3 = -M_PI/2.0;
const double DH_alpha4 = M_PI/2.0;
const double DH_alpha5 = -M_PI/2.0;
const double DH_alpha6 = M_PI/2.0;
const double DH_alpha7 = M_PI;

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
const double DH_q_offset1 = -M_PI/2.0;
const double DH_q_offset2 = 0;
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = 0.0;
const double DH_q_offset5 = 0.0;
const double DH_q_offset6 = 0;
const double DH_q_offset7 = 0;

const double deg2rad = M_PI/180.0;

const double DH_q_max1 = 1.6056;
const double DH_q_max2 = 1.518;
const double DH_q_max3 = M_PI; //continuous
const double DH_q_max4 = 2.251;
const double DH_q_max5 = M_PI; //continuous
const double DH_q_max6 = 2.16; 
const double DH_q_max7 = M_PI; //continuous

const double DH_q_min1 = -1.6056;
const double DH_q_min2 = -1.221;
const double DH_q_min3 = -M_PI; //
const double DH_q_min4 = -2.251;
const double DH_q_min5 = -M_PI;
const double DH_q_min6 = -2.16; 
const double DH_q_min7 = -M_PI; 


const double DH_a_params[]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
const double DH_d_params[] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6, DH_d7};
const double DH_alpha_params[] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6, DH_alpha7};
const double DH_q_offsets[] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6,DH_q_offset7};
const double q_lower_limits[] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6, DH_q_min7};
const double q_upper_limits[] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6, DH_q_max7};
const double g_qdot_max_vec[] = {1.256, 1.454, 1.571, 1.521, 1.571, 2.268, 2.268}; //values per URDF 
const double g_q_home_pose[] = {0,0,0,0,0,0,0};
//put these in planner_joint_weights.h
//const double jspace_planner_weights[] = {5,5,3,0.5,0.2,0.2}; //default weights for jspace planner (changeable in planner)


class Fetch_fwd_solver {
public:
    Fetch_fwd_solver(); //constructor; //const hand_s& hs, const atlas_frame& base_frame, double rot_ang);
    //atlas_hand_fwd_solver(const hand_s& hs, const atlas_frame& base_frame);
    //Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); // given vector of q angles, compute fwd kin
    Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); 
    Eigen::Matrix4d get_wrist_frame();
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec);
    //Eigen::MatrixXd get_Jacobian(const Eigen::VectorXd& q_vec);
    void  test_q123(std::vector<Eigen::VectorXd> q_solns);
private:
    Eigen::Matrix4d fwd_kin_solve_(const Eigen::VectorXd& q_vec);
    Eigen::Matrix4d A_mats[NJNTS], A_mat_products[NJNTS], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    Eigen::MatrixXd Jacobian;


};

class Fetch_IK_solver: public Fetch_fwd_solver {
public:
    Fetch_IK_solver(); //constructor; 

    // return the number of valid solutions; actual vector of solutions will require an accessor function
    int ik_solve(Eigen::Affine3d const& desired_hand_pose); // given desired pose, compute IK
    int ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns);
    void get_solns(std::vector<Eigen::VectorXd> &q_solns);
    bool fit_joints_to_range(Eigen::VectorXd &qvec);
    //Eigen::MatrixXd get_Jacobian(const Eigen::VectorXd& q_vec);
private:
    bool fit_q_to_range(double q_min, double q_max, double &q);    

    Eigen::Matrix4d A_mats[NJNTS], A_mat_products[NJNTS], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    double L_humerus_;
    double L_forearm_;
    double phi_elbow_;
    std::vector<Eigen::VectorXd> q_solns_fit_,q_solns_;
    //given desired flange pose, fill up solns for q1, q2, q3 based on wrist position
    bool compute_q123_solns(Eigen::Affine3d const& desired_hand_pose, std::vector<Eigen::VectorXd> &q_solns);
    //double solve_for_theta2(double q1,Eigen::Vector3d w_des);
    bool solve_for_theta2(Eigen::Vector3d w_wrt_1,double r_goal, double q2_solns[2]);    
    bool solve_for_theta3(Eigen::Vector3d w_wrt_1,double r_goal, double q3_solns[2]); 

    bool solve_spherical_wrist(Eigen::VectorXd q_in,Eigen::Matrix3d R_des, std::vector<Eigen::VectorXd> &q_solns);    
    //Eigen::MatrixXd Jacobian;
};


#endif	/* FETCH_IK_H */

