/** grab_all.cpp...TEST VERSION--just does arm motions
 * twa16 5/1/19
 * Edited twa16 5/3 to place parts instead of releasing them
 */

#include <ros/ros.h>
#include <move_part_lib/move_part.h>                // Manipulation
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation
#include <mobot_pub_des_state/integer_query.h>      // Navigation
#include <object_finder_lib/object_finder.h>        // Perception
#include <part_codes/part_codes.h>

using namespace std;

int ans;

int main(int argc, char** argv) {
    /* Node Setup */
    // Initialize this node with a standard node handle
    ros::init(argc, argv, "grab_tote");
    ros::NodeHandle nh;

    // Create Manipulation and Perception objects
    MovePart movePart;
    FindPart findPart;



    /* Perception */
    cout << "Choose a perception operation: 0 for Fake part \n1 for Gearbox top\n2 Gearbox bottom\n3 for Bolt\n4 for Small gear\n5 for Large gear\n6 for Tote" << endl;
    cin >> ans;  
    //ans=0; // Fake part

    // Vector to contain Perceived parts
    std::vector <geometry_msgs::PoseStamped> part_poses;

    // Create part code
    int partCode;

    // Set Part code as noted by ans
    switch(ans){
        case 0:
            partCode = part_codes::part_codes::FAKE_PART;
            break;
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
            ROS_WARN("Unrecognized perception operation");
            break;
    }

    ROS_INFO("Attempting to locate part(s) on table");
    bool success = findPart.find_part(partCode, part_poses);

    int num_parts = part_poses.size();
    if (num_parts < 1) {
        ROS_WARN("DID NOT FIND ANY PARTS; QUITTING");
        return 0;
    }

    ROS_INFO("Found %d parts", num_parts);
    for (int i=0; i < num_parts; i++) {
        ROS_INFO_STREAM(part_poses[i] << endl);
    }
    if (num_parts<1) {
        ROS_WARN("giving up");
        return 0;
    }

    // Choose the first part
    geometry_msgs::PoseStamped source_pose = part_poses[0];
    ROS_INFO_STREAM("Chosen part pose " << source_pose << endl);
    ROS_INFO("Triad pose: ");
    findPart.display_triad(source_pose);



    /* Manipulation */
    // Manual pause for testing
    // sleep statements are for testing purposes
    cout<<"Enter 1 to attempt grasp: ";
    cin>>ans;

    ROS_INFO("Attempting to grasp chosen part");
    success = movePart.get_part(partCode, source_pose);
  
    if(!success){
        ROS_ERROR("Failed to get part");
        return 0;
    }
    //ros::Duration(4.0).sleep();

    // movePart.release_grasped_part(); //drop the part back on the table
    // ros::Duration(3.0).sleep();

    ROS_INFO("Attempting to move arm to preset");
    success = movePart.preset_arm();
    if(!success){
        ROS_ERROR("Failed to move to preset");
        return 0; //this will kill the coordinator.  Do something else
    }
    //ros::Duration(3.0).sleep();


  
    // Place the part in the kit
    switch(partCode){
        case part_codes::part_codes::GEARBOX_TOP:
        case part_codes::part_codes::GEARBOX_BOTTOM:
            ROS_INFO("Attempting to place gearbox part in kit zone 1");
            cout<<"enter 1: ";
            cin>>ans;
            success = movePart.move_to_dropoff_kit1();
            if(!success){
                ROS_ERROR("Failed to place gearbox part in tote; quitting!");
                return 0;
            }
            ROS_INFO("Attempting to recover from dropoff");
            cout<<"enter  1: ";
            cin>>ans;
            success = movePart.recover_from_dropoff();
            break;
        case part_codes::part_codes::BOLT:
            ROS_INFO("Attempting to place bolt part in kit zone 2");
            cout<<"enter 1: ";
            cin>>ans;            
            success = movePart.move_to_dropoff_kit2();
            if(!success){
                ROS_ERROR("Failed to place bolt part in tote; quitting!");
                return 0;
            }          
            ROS_INFO("Attempting to recover from dropoff");
            cout<<"enter  1: ";
            cin>>ans;
            success = movePart.recover_from_dropoff();            
            break;
        case part_codes::part_codes::LARGE_GEAR:
        case part_codes::part_codes::SMALL_GEAR:
            ROS_INFO("Attempting to place gear part in kit zone 3");
            cout<<"enter 1: ";
            cin>>ans;            
            success = movePart.move_to_dropoff_kit3();
            if(!success){
                ROS_ERROR("Failed to place gear part in tote; quitting!");
                return 0;
            }            
            ROS_INFO("Attempting to recover from dropoff");
            cout<<"enter  1: ";
            cin>>ans;
            success = movePart.recover_from_dropoff();            
            break;
        case part_codes::part_codes::TOTE:
            //ROS_INFO("Attempting to pickup kit");
            //success = movePart.move_to_pickup_tote(); //TO GET TOTE FROM PEDESTAL AND GO TO MANTIS POSE
            //if(!success){
            //    ROS_ERROR("Failed to pickup tote part");
            //    return 0;
            //}

            //ros::Duration(3.0).sleep();

            ROS_INFO("Attempting to dropoff kit");
            cout<<"enter 1: ";
            cin>>ans;            
            success = movePart.move_to_dropoff_tote(); //TO PLACE TOTE ON PEDESTAL
            ROS_INFO("Attempting to recover from tote on pedestal dropoff");
            cout<<"enter  1: ";
            cin>>ans;
            success = movePart.recover_from_tote();
            break;
        default:
            ROS_WARN("no dropoff case for this part! ");
            movePart.release_grasped_part(); //drop the part back on the table
            ros::Duration(3.0).sleep();
            return 0;
    }
    

    if(!success){
        ROS_ERROR("action failed");
        return 0;
    }


    /* END */
    ROS_INFO("done");
    return 0;
}
