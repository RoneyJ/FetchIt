//! This is the source code for finding the gearbox bottom and gearbox top

/*
*Note: may want to use functions defined in object_helper_fncs.cpp, including:
void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min, Eigen::Vector4f box_pt_max, vector<int> &indices)
float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)
void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 
void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)
void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)
*/

//! Magic Numbers:
const float MIN_X_GEARBOX = 0.0; //include points starting 0.4m in front of robot
const float MAX_X_GEARBOX = 0.8; //include points out to 0.9m in front of robot
const float MIN_Y_GEARBOX = -1.0; //include points starting -0.5m to left of robot
const float MAX_Y_GEARBOX = 1.0; //include points up to 0.5m to right of robot
const float MIN_DZ_GEARBOX = 0.02; //box filter from this height above the table top
const float MAX_DZ_GEARBOX = 0.1; //consider points up to this height above table top
const double TABLE_GRASP_CLEARANCE_GEARBOX = 0.01;


//! Local Tool Kit:
//? findOrientation utilizing linear regression
double findOrientation(int labelNum) {
    std::vector<double> x;
    std::vector<double> y;

    //Loops through the entire image and 
    for (int r = 0; r < g_dst.rows; ++r) {
        for (int c = 0; c < g_dst.cols; ++c) {
            int label = g_labelImage.at<int>(r, c);
            if(label == labelNum) {
                x.push_back(r);
                y.push_back(Nu - c);
            }
        }
    }
    double numPoints = x.size();

    double EX    = std::accumulate(x.begin(), x.end(), 0.0)/numPoints;
    double EY    = std::accumulate(y.begin(), y.end(), 0.0)/numPoints;
    double EXX   = std::inner_product(x.begin(), x.end(), x.begin(), 0.0)/numPoints;
    double EXY   = std::inner_product(x.begin(), x.end(), y.begin(), 0.0)/numPoints;
    double slope = (EXY-(EX*EY)) / (EXX-(EX*EX));
    
    return atan2((EXY-(EX*EY)), (EXX-(EX*EX)));
}

void splitGearboxTopBottom(vector <int> &lookup_table, int part_id){
    if (part_id == part_codes::part_codes::GEARBOX_BOTTOM){
        ROS_INFO("[gearbox_finder_fnc=seperation] Seperating for bottom gearbox part now...");
        //TODO Implement Logic for finding component!
        lookup_table.clear();
        lookup_table.push_back(0);//! Temporary work around:
    } else if (part_id ==part_codes::part_codes::GEARBOX_TOP)
    {
        ROS_INFO("[gearbox_finder_fnc=seperation] Seperating for top gearbox part now...");
        //TODO Implement Logic for finding component!
    }else
    {
        ROS_ERROR("[gearbox_finder_fnc=seperation] COMPONENT ID NOT KNOWN!");
    }
}

//! Find Gearbox Top MAIN
bool ObjectFinder::find_gearbox_tops
                                    (
                                        float table_height,
                                        vector<float> &x_centroids_wrt_robot,
                                        vector<float> &y_centroids_wrt_robot,
                                        vector<float> &avg_z_heights,
                                        vector<float> &npts_blobs,
                                        vector<geometry_msgs::PoseStamped> &object_poses
                                    )
{
    //! Variable Initialization            
    geometry_msgs::PoseStamped object_pose; //* Used to populate the final message

    //! These are the value will get passed back... Initialize them now...
    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    object_poses.clear();
    viable_labels_.clear();

    //! Initialize parameter used for box filter
    Eigen::Vector4f box_pt_min, box_pt_max;
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


    //! Blob Finder (Segmentation)
    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]
    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs, viable_labels_);
    ROS_INFO("[gearbox_finder_fnc=bottom]Totoal Found Blobs: %d.",x_centroids_wrt_robot.size());


    //! Call seperation here to seperate gearbox bottom and gearbox top
    vector <int> valid_component_list;
    splitGearboxTopBottom(valid_component_list,part_codes::part_codes::GEARBOX_BOTTOM);

    //*** From here, in order to refer to object, we need to do: valid_component_list[i_object] as our original counter.
    int valid_components_count = valid_component_list.size(); // This gets how many valid gearbox component are there

    //! Generate angle matrix
    vector <float> orientation_wrt_robot; //* Note that this is the orientation for all blob, not just the valid one.
    for (int orientation_counter = 0; orientation_counter <x_centroids_wrt_robot.size(); orientation_counter++){
        orientation_wrt_robot.push_back((float) findOrientation(orientation_counter));
    }

    //! Update the object_pose for robot to grasp
    object_poses.clear();

    //* Fault Handling
    if (valid_components_count < 1){
        ROS_ERROR("[gearbox_finder_fnc=bottom]OBJECT NOT FOUND!");
        return false; //background is object 0    
    } else
    {
        ROS_WARN("[gearbox_finder_fnc=bottom]CURRENT VALID COMPONENT COUNT IS: %d",valid_components_count);

    }
    
    
    //* Pre Publishing
    object_pose.header.frame_id = "torso_lift_link";
    for (int i_object = 1; i_object < valid_components_count; i_object++) {
        //? Position
        object_pose.pose.position.x = x_centroids_wrt_robot[valid_component_list[i_object]];
        object_pose.pose.position.y = y_centroids_wrt_robot[valid_component_list[i_object]];
        object_pose.pose.position.z = table_height + TABLE_GRASP_CLEARANCE_GEARBOX;
        //? Orientation
        object_pose.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(orientation_wrt_robot[valid_component_list[i_object]]);
        //? Push Back
        object_poses.push_back(object_pose);
    }
    return true;
}

//! Find Gearbox Bottom MAIN
bool ObjectFinder::find_gearbox_bottoms(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights, vector<float> &npts_blobs, 
        vector<geometry_msgs::PoseStamped> &object_poses) {
    return false;
}

