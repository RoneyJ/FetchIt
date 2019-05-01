//source code for gear-finder fncs


//Note: may want to use functions defined in object_helper_fncs.cpp, including:
// void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
//        Eigen::Vector4f box_pt_max, vector<int> &indices)
// float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)

// void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 

//  void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)

//void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)

const float MIN_X_g = 0.4; //include points starting 0.4m in front of robot
const float MAX_X_g = 0.7; //include points out to 0.9m in front of robot
const float MIN_Y_g = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y_g = 0.7; //include points up to 0.5m to right of robot
const float MIN_DZ_g = 0.02; //box filter from this height above the table top
const float MAX_DZ_g = 0.2; //consider points up to this height above table top
const float MIN_DZ_GEAR = 0.01;
const float MAX_DZ_GEAR = 0.1;

//here is old, non-functional code:

bool ObjectFinder::find_large_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {

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
    if (viable_labels_.size() >0){
    	imwrite( "large_gearImage.png", g_dst );
    	return true;
    }
    else return false; //fix me
}

//placeholder for new code:
bool ObjectFinder::find_small_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {
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
    if (viable_labels_.size() >0){
    	imwrite( "small_gearImage.png", g_dst );
    	return true;
    }
    else return false; 
}


