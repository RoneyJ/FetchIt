//source code for find_gearbox_top() and find_gearbox_bottom() goes here


//Note: may want to use functions defined in object_helper_fncs.cpp, including:
// void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
//        Eigen::Vector4f box_pt_max, vector<int> &indices)
// float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)

// void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 

//  void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)

//void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)



//here is old, non-functional code:  does not compute orientations

// use table_height_ member var
const float MIN_X_GEARBOX = 0.25; //include points starting 0.4m in front of robot
const float MAX_X_GEARBOX = 0.8; //include points out to 0.9m in front of robot
const float MIN_Y_GEARBOX = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y_GEARBOX = 0.7; //include points up to 0.5m to right of robot
const float MIN_DZ_GEARBOX = 0.01; //box filter from this height above the table top
const float MAX_DZ_GEARBOX = 0.1; //consider points up to this height above table top
const double TABLE_GRASP_CLEARANCE_GEARBOX = 0.01;

bool ObjectFinder::find_gearbox_tops(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights, vector<float> &npts_blobs, 
        vector<geometry_msgs::PoseStamped> &object_poses) {

    geometry_msgs::PoseStamped object_pose;

    Eigen::Vector4f box_pt_min, box_pt_max;

    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    object_poses.clear();


    box_pt_min << MIN_X_GEARBOX, MIN_Y_GEARBOX, table_height + MIN_DZ_GEARBOX, 0; //from MIN_DZ above table top
    box_pt_max << MAX_X_GEARBOX, MAX_Y_GEARBOX, table_height + MAX_DZ_GEARBOX, 0;

    //find which points in the transformed cloud are within the specified box
    //result is in "indices"
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    int npts_cloud = indices_.size();

   //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]
    //
    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs,viable_labels_);


    object_poses.clear();  //THIS IS THE WHOLE POINT.  FUNCTION MUST POPULATE THIS VECTOR!!

    int n_objects = x_centroids_wrt_robot.size();
    if (n_objects < 2) return false; //background is object 0
    object_pose.header.frame_id = "torso_lift_link";
    for (int i_object = 1; i_object < n_objects; i_object++) {
        object_pose.pose.position.x = x_centroids_wrt_robot[i_object];
        object_pose.pose.position.y = y_centroids_wrt_robot[i_object];
        object_pose.pose.position.z = table_height + TABLE_GRASP_CLEARANCE_GEARBOX;
        //FIX ME!  do not yet have value orientation info
        object_pose.pose.orientation.x = 0;
        object_pose.pose.orientation.y = 0;
        object_pose.pose.orientation.z = 0;
        object_pose.pose.orientation.w = 1;
        object_poses.push_back(object_pose);
    }
    return true;
}



//placeholder for new code:
bool ObjectFinder::find_gearbox_bottoms(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights, vector<float> &npts_blobs, 
        vector<geometry_msgs::PoseStamped> &object_poses) {
    return false;
}


