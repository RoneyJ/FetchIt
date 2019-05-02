//source code for gear-finder fncs


//Note: may want to use functions defined in object_helper_fncs.cpp, including:
// void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
//        Eigen::Vector4f box_pt_max, vector<int> &indices)
// float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)

// void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 

//  void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)

//void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <math.h>
#include <stdio.h> 
using namespace std;
using namespace cv;
const float MIN_X_g = 0.4; //include points starting 0.4m in front of robot
const float MAX_X_g = 0.7; //include points out to 0.9m in front of robot
const float MIN_Y_g = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y_g = 0.7; //include points up to 0.5m to right of robot
const float MIN_DZ_g = 0.02; //box filter from this height above the table top
const float MAX_DZ_g = 0.2; //consider points up to this height above table top
const float MIN_DZ_GEAR = 0.01;
const float MAX_DZ_GEAR = 0.1;

const float min_small_gear_pts = 95.0;
const float max_small_gear_pts = 250.0;
const float min_large_gear_pts = 285.0;
const float max_large_gear_pts = 900.0;


bool ObjectFinder::find_small_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {
	geometry_msgs::PoseStamped object_pose;
	object_poses.clear();
	Eigen::Vector4f box_pt_min, box_pt_max;
	vector<float> countours_area;
	vector<float> countours_orientation;
    box_pt_min << MIN_X_g, MIN_Y_g, table_height + MIN_DZ_GEAR ,0; //1cm above table top
    box_pt_max << MAX_X_g, MAX_Y_g, table_height+ MAX_DZ_GEAR,0;
    ROS_INFO("Finding blobs");
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    float npts_cloud = indices_.size();



    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs,viable_labels_);

    // Doing PCA analysis to find the orientation fo each blob


    imwrite("gearimage.png",g_dst);


	XformUtils transform;
    int nlabels = viable_labels_.size();
    if ( nlabels>0)
    {
    	
    for (int label = 0; label < nlabels; ++label) {
        ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);
       // if (avg_z_heights[label] <= 91) 
        
            if (npts_blobs[label] >=  min_small_gear_pts && npts_blobs[label] <=  max_small_gear_pts ) {
                cout<<"inside thje blolb.....\n";

            	object_pose.pose.position.x = x_centroids_wrt_robot[label];
			    object_pose.pose.position.y = y_centroids_wrt_robot[label];
			    object_pose.pose.position.z = 0.005;
			    object_pose.pose.orientation = g_vec_of_quat[label];
			    object_poses.push_back(object_pose);

            	return true;
            }
            else continue;
        
    	
    }
}
else return false;
}

//placeholder for new code:
bool ObjectFinder::find_large_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {
	geometry_msgs::PoseStamped object_pose;
	object_poses.clear();
	Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X_g, MIN_Y_g, table_height + MIN_DZ_GEAR ,0; //1cm above table top
    box_pt_max << MAX_X_g, MAX_Y_g, table_height+ MAX_DZ_GEAR,0;
    ROS_INFO("Finding blobs");
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    float npts_cloud = indices_.size();



    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs,viable_labels_);





    int nlabels = viable_labels_.size();
    imwrite("gearimage.png",g_dst);

    if ( nlabels>0)
    {
    	
    	for (int label = 0; label < nlabels; ++label) {
        ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);
        //if (avg_z_heights[label] > 92) 
        { 
            if (npts_blobs[label] >=  min_large_gear_pts && npts_blobs[label] <=  max_large_gear_pts ) {

            	object_pose.pose.position.x = x_centroids_wrt_robot[label];
			    object_pose.pose.position.y = y_centroids_wrt_robot[label];
			    object_pose.pose.position.z = 0.005;
			    object_pose.pose.orientation = g_vec_of_quat[label];
			    object_poses.push_back(object_pose);

            	return true;
            }
            else continue;
        }
    	
    }
}
else return false;

        vector<Vec3b> 
        
        for (int r = 0; r < g_dst.rows; ++r) {
                for (int c = 0; c < g_dst.cols; ++c) {
                    int label = g_labelImage.at<int>(r, c);
                    Vec3b &pixel = g_dst.at<Vec3b>(r, c);
                    pixel = colors[label];
                    temp_y_centroids[label] += c; //robot y-direction corresponds to columns--will negate later
                    temp_x_centroids[label] += r; //robot x-direction corresponds to rows--will negate later
                    temp_npts_blobs[label] += 1.0;
                    double zval = (float) g_bw_img(r, c);
                    temp_avg_z_heights[label] += zval; //check the  order of this
                }
            }

}

