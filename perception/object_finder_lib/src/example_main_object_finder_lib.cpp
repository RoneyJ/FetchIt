#include<ros/ros.h>
#include<object_finder_lib/object_finder.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_find_part_main"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
   FindPart findPart;
   std::vector <geometry_msgs::PoseStamped> part_poses;
   int partCode = part_codes::part_codes::GEARBOX_TOP;
   bool success = findPart.find_part(partCode,part_poses);
   int nparts = part_poses.size();
   ROS_INFO("found %d parts",nparts);
   for  (int i=0;i<nparts;i++) {
     ROS_INFO_STREAM(part_poses[i]<<endl);
   }

}
