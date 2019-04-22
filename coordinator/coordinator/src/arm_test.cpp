//arm_test.cpp
// jpr87, April 2019
#include<ros/ros.h>
#include<move_part_lib/move_part.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_test"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
   MovePart movePart;

   geometry_msgs::PoseStamped part_pose;
   int partCode = part_codes::part_codes::GEARBOX_TOP;
   geometry_msgs::PoseStamped source_pose; //for part acquisition

   //populate a pose to mimic picking up a part
   part_pose.pose.position.x = 0.640851;
   part_pose.pose.position.y = 0.058637;
   part_pose.pose.position.z = 0.025;
   part_pose.pose.orientation.x = 0.0;
   part_pose.pose.orientation.y = 0.0;
   part_pose.pose.orientation.z = 0.0;
   part_pose.pose.orientation.w = 1.0;

   ROS_INFO("attempting to acquire a perceived part using pose: ");

   source_pose= part_pose;
   ROS_INFO_STREAM(source_pose<<endl);
   
   int ans;
   cout<<"enter 1 to attempt grasp: ";
   cin>>ans;
   //invoke the  "get_part()" function to acquire specified part from specified  pose
   bool success = movePart.get_part(partCode,source_pose);
   
   //Move to drop off imaginary gearbox in bin and recover
   success = movePart.preset_arm();
   ros::Duration(1.0).sleep();
   success = movePart.move_to_dropoff_kit1();
   ros::Duration(1.0).sleep();
   success = movePart.recover_from_dropoff();

   cout<<"enter 1 to attempt grasp: ";
   cin>>ans;

   //Test other 2 dropoff poses
   ros::Duration(1.0).sleep();
   success = movePart.move_to_dropoff_kit2();
   ros::Duration(1.0).sleep();
   success = movePart.recover_from_dropoff();

   cout<<"enter 1 to attempt grasp: ";
   cin>>ans;

   ros::Duration(1.0).sleep();
   success = movePart.move_to_dropoff_kit3();
   ros::Duration(1.0).sleep();
   success = movePart.recover_from_dropoff();
   

   ROS_INFO("done");
   return 0;
}