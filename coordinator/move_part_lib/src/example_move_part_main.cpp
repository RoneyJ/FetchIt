//example main fnc for using move_part library
#include<ros/ros.h>
#include <move_part_lib/move_part.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_move_part_main"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
   MovePart movePart;
   movePart.preset_arm();
   int partCode = object_finder::objectFinderGoal::GEARBOX_TOP;
   bool success = movePart.get_part(partCode);
   
       

}

