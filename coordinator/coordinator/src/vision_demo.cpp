// Vision Based Manipulation Demonstration
// CXQ41@case.edu
// Used for demo at fetch's show booth ICRA 2019

#include <ros/ros.h>
#include <move_part_lib/move_part.h>                // Manipulation
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation
#include <mobot_pub_des_state/integer_query.h>      // Navigation
#include <object_finder_lib/object_finder.h>        // Perception
#include <part_codes/part_codes.h>                  // Part Code

//! Quick Configuration:
int partCode = part_codes::part_codes::GEARBOX_BOTTOM;

using namespace std;

int user_input;
bool manipulation_success;
bool vision_return;

int main(int argc, char** argv){
    //! Initial ROS Setup:
    ros::init(argc,argv,"vision_demo");
    ros::NodeHandle nh;

    //! Initial Vision and Manipulation Setup:
    MovePart movePart;
    FindPart findPart;

    //! Prepose (should not affect but just in case)
    ROS_WARN(">>>Arm Reset");
    manipulation_success = movePart.preset_arm();
    if (!manipulation_success){
        //error catcher
        ROS_FATAL("ERROR! Initiate PLAN FAILED...");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(5.0).sleep();                
        return 0;
    }            
    ros::Duration(1.0).sleep();

    //! Select Object to detect:
    ROS_WARN("Choose a perception operation:\n1 :: Gearbox top\n2 :: Gearbox bottom\n3 :: Bolt\n4 :: Small gear\n5 :: Large gear\n6 :: Tote");
    cin >> user_input; 

    //! input guard
    while(!(user_input>0 && user_input<7)){
        ROS_FATAL("USER INPUT NOT SUPPORTED! TRY AGAIN>>>");
        cin >> user_input;
    }
    
    //! passing through object code
    switch(user_input){
        case 1:
            partCode = part_codes::part_codes::GEARBOX_TOP;
            break;
        case 2:
            partCode = part_codes::part_codes::GEARBOX_BOTTOM;
            break;
        case 3:
            partCode = part_codes::part_codes::BOLT;
            break;
        case 4:
            partCode = part_codes::part_codes::SMALL_GEAR;
            break;
        case 5:
            partCode = part_codes::part_codes::LARGE_GEAR;
            break;
        case 6:
            partCode = part_codes::part_codes::TOTE;
            break;
        default:
            ROS_WARN("Unrecognized perception operation...");
            break;
    }

    //! Initiate a pose array to store the pose
    std::vector <geometry_msgs::PoseStamped> part_poses;

    //! Hand Over to Object Finder
    ROS_WARN("Attempt to call object finder...");
    vision_return = findPart.find_part(partCode,part_poses);

    //! Object Finder Return Check
    int num_parts = part_poses.size();
    if (num_parts<1){
        ROS_FATAL(">>>>>OBJECT FINDER FAILED<<<<<");
        ROS_FATAL("Please Wait for auto-recovery!!!");
        ros::Duration(5.0).sleep();        
        return 0;
    } else {
        ROS_WARN("Found >>>%d<<< blobs total.",num_parts);
        //! Stick a triad on each detected component
        for (int i=0; i < num_parts; i++) {
            ROS_WARN("Blob %i detected pose:",i);
            geometry_msgs::PoseStamped temporary_pose = part_poses[i];
            ROS_INFO_STREAM(part_poses[i] << endl);
            findPart.display_triad(temporary_pose);
        }
    }

    //! The selected one
    geometry_msgs::PoseStamped selected_object_pose = part_poses[0];
    
    //! Prompt for manipulation
    ROS_FATAL("PLEASE MAKE SURE SCENE IS CLEAR and CHECK RVIZ!\n1 :: continue\n2 :: manually restart");
    cin>>user_input;
    
    if (user_input==2){
        ROS_FATAL("USER ENTERED RESTART...");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(1.0).sleep();        
        return 0;
    }

    //! Grasp the object
    ROS_WARN(">>>Grasp Object");
    manipulation_success = movePart.get_part(partCode,selected_object_pose);
    if (!manipulation_success){
        //error catcher
        ROS_FATAL("ERROR! GRASP OBJECT PLAN FAILED...");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(5.0).sleep();                
        return 0;
    }
    ros::Duration(1.0).sleep();

    //! Intermediate Clearance
    ROS_WARN(">>>Arm intermediate step");
    manipulation_success = movePart.preset_arm();
    if (!manipulation_success){
        //error catcher
        ROS_FATAL("ERROR! Reset Arm after grasp PLAN FAILED...");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(5.0).sleep();                
        return 0;
    }
    ros::Duration(1.0).sleep();

    //! Store the part
    switch(partCode){
        case part_codes::part_codes::GEARBOX_TOP:
        case part_codes::part_codes::GEARBOX_BOTTOM:
            ROS_WARN(">>>Attempting to place gearbox part in kit zone 1");
            manipulation_success = movePart.move_to_dropoff_kit1();
            break;
        case part_codes::part_codes::BOLT:
            ROS_WARN(">>>Attempting to place bolt part in kit zone 2");
            manipulation_success = movePart.move_to_dropoff_kit2();
            break;
        case part_codes::part_codes::LARGE_GEAR:
        case part_codes::part_codes::SMALL_GEAR:
            ROS_WARN(">>>Attempting to place gear part in kit zone 3");
            manipulation_success = movePart.move_to_dropoff_kit3();
            break;
        case part_codes::part_codes::TOTE:
            ROS_WARN(">>>Attempting to leave tote on pedastal");
            manipulation_success = movePart.move_to_dropoff_tote();                    
            break;
        default:
            ROS_FATAL("no dropoff case for this part! ");
            movePart.release_grasped_part(); //drop the part back on the table
            manipulation_success = false;
            break;
    }
    if (!manipulation_success){
        //error catcher
        ROS_FATAL("ERROR! Drop component PLAN FAILED...");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(5.0).sleep();                
        return 0;
    }
    ros::Duration(1.0).sleep();

    //! Recover to initial pose
    if(partCode != part_codes::part_codes::TOTE){
        ROS_INFO("Attempting to recover from dropoff");
        manipulation_success = movePart.recover_from_dropoff();
    }
    else{
        ROS_INFO("Attempting to move arm to preset");
        manipulation_success = movePart.recover_from_tote();
    }
    if (!manipulation_success){
        //error catcher
        ROS_FATAL("ERROR! Reset Arm after drop-off to tote PLAN FAILED...");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(5.0).sleep();                
        return 0;
    }

    //! Check if user wants the tote back...
    ROS_WARN("Do you want the tote back?\n1 :: Yes\n2 :: No");
    cin >> user_input;
    
    while((user_input != 1) && (user_input != 2)){
        ROS_FATAL("PLEASE INPUT:\n1 :: Yes\n2 :: No");
        cin>>user_input;
    }

    if (user_input == 1){
        manipulation_success = movePart.grab_tote_from_pedestal_and_place_on_table();
        if (!manipulation_success){
            //error catcher
            ROS_FATAL("ERROR! Return Tote to table PLAN FAILED...");
            ROS_FATAL("Please Wait for auto-restart!!!");
            ros::Duration(5.0).sleep();                    
            return 0;
        }                
    } else {
        ROS_FATAL("Program is gonna loop through....");
        ROS_FATAL("Please Wait for auto-restart!!!");
        ros::Duration(2.0).sleep();
    }
}