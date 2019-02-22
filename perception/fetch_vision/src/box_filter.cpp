//find_filter_and_segment.cpp
// try to segment objects.  Start w/ filtering

// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn Feb 2019

#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs


using namespace std;
PclUtils *g_pcl_utils_ptr; 


void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,Eigen::Vector3f box_pt_min,  
       Eigen::Vector3f box_pt_max,vector<int> &indices) {
    int npts = input_cloud_ptr->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    cout<<"box min: "<<box_pt_min.transpose()<<endl;
    cout<<"box max: "<<box_pt_max.transpose()<<endl;
    for (int i = 0; i < npts; ++i) {
        pt = input_cloud_ptr->points[i].getVector3fMap();
        
        //check if in the box:
        if ((pt[0]>box_pt_min[0])&&(pt[0]<box_pt_max[0])&&(pt[1]>box_pt_min[1])&&(pt[1]<box_pt_max[1])&&(pt[2]>box_pt_min[2])&&(pt[2]<box_pt_max[2])) { 
            //passed box-crop test; include this point
            //ROS_INFO("point passes test");
            cout<<"pt passed test: "<<pt.transpose()<<endl;
               indices.push_back(i);
        }
    }
    int n_extracted = indices.size();
    cout << " number of points within box = " << n_extracted << endl;            
}

void find_indices_box_filtered2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, vector<int> &indices) {
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    //pcl::PointXYZRGB point;
    int ans;
    double z = -0.06;
    double dz = 0.1;
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();
        /*
        if (npts>0) cout<<"num indices: "<<npts<<endl;
        //cout<<"enter 1: ";
        //cin>>ans;
        int i_index;
        for (int i=0;i<npts;i++) {
            i_index = indices[i];
            point = input_cloud_ptr->points[i_index];
            cout<<"point "<<i_index<<": x,y,z = "<<point.x<<", "<<point.y<<", "<<point.z<<endl;
        }
         * */

    int n_extracted = indices.size();
    cout << " number of points within box = " << n_extracted << endl;            
}


double find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,double z_min, double z_max, double dz) {
    vector<int> indices;
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    pcl::PointXYZRGB point;
    int ans;
    for (double z = z_min; z < z_max; z += dz) {
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();
        /*
        if (npts>0) cout<<"num indices: "<<npts<<endl;
        //cout<<"enter 1: ";
        //cin>>ans;
        int i_index;
        for (int i=0;i<npts;i++) {
            i_index = indices[i];
            point = input_cloud_ptr->points[i_index];
            cout<<"point "<<i_index<<": x,y,z = "<<point.x<<", "<<point.y<<", "<<point.z<<endl;
        }
         * */
        if (npts>0) ROS_INFO("z=%f; npts = %d",z,npts);
        if (npts > npts_max) {
            npts_max = npts;
            z_table = z + 0.5 * dz;
        }
    }
    ROS_INFO("max pts %d at height z= %f",npts_max,z_table);
    return z_table;
}

//use this for kinect simu:
Eigen::Affine3f compute_affine_cam_wrt_floor(double tilt_ang) {
     //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_floor;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam<<0,0,1.810;
    affine_cam_wrt_floor.translation() = O_cam;
    double cos_tilt = cos(tilt_ang);
    double sin_tilt = sin(tilt_ang);
    Eigen::Vector3f nvec,tvec,bvec;
    //[c 0 s
    // 0 1 0
    //-s 0 c]  or, try:
    //[s 0 c
    // 0 1 0
    // c 0 -s]
    // or rot about x-axis:
    //[1  0  0
    // 0  c -s
    // 0  s  c]     
    nvec<<1,0,0;
    tvec<<0,cos_tilt,sin_tilt;
    bvec<<0,-sin_tilt,cos_tilt;
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;
    affine_cam_wrt_floor.linear() = R_cam;
    return affine_cam_wrt_floor;
}

//use this for Fetch simu:
Eigen::Affine3f compute_affine_cam_wrt_torso_lift_link(void) {
     //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_torso;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam<<0.244,0.02, 0.627;
    affine_cam_wrt_torso.translation() = O_cam;
    //double cos_tilt = cos(tilt_ang);
    //double sin_tilt = sin(tilt_ang);
    Eigen::Vector3f nvec,tvec,bvec;
 
    nvec<<0,-1, 0;
    tvec<<-0.8442,0,-0.5377;
    bvec<<0.5377,0,-0.8442;
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;
    affine_cam_wrt_torso.linear() = R_cam;
    return affine_cam_wrt_torso;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<int> indices;
    Eigen::Affine3f A_plane_wrt_camera;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "head_camera_rgb_optical_frame";
    ROS_INFO("view frame head_camera_rgb_optical_frame on topic pcd");
    //<origin xyz="0.027 0.0 1.810" rpy="0.0 1.2 0"/>
    //define pose of the camera for transforming points:
    Eigen::Affine3f affine_cam_wrt_floor; //pose of cam w/rt floor

    
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud, ros_box_filtered_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too

        A_plane_wrt_camera = compute_affine_cam_wrt_torso_lift_link();
        cout<<"R_plane_wrt_cam: "<<endl<<A_plane_wrt_camera.linear()<<endl;

       pcl::transformPointCloud(*pclKinect_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera);
       //pcl::transformPointCloud(*downsampled_kinect_ptr, *transformed_cloud_ptr, A_plane_wrt_camera);

    //pclTransformed_ptr_
    int npts = transformed_cloud_ptr->points.size();
    cout<<"npts transformed = "<<npts<<endl;


    double table_height = find_table_height(transformed_cloud_ptr,-2.0, 2.0, 0.01);
    
    Eigen::Vector3f box_pt_min,box_pt_max;
    box_pt_min<<0.5,-0.5,0.02;
    box_pt_max<<2.0,0.5,0.5;
    int dim;

        dim=0;

        box_pt_min(0)=-0.5; //min x-val search
        box_pt_max(0)= 0.5; //max x-val
        box_pt_min(1)= -0.5; //min y-val search
        box_pt_max(1)= 0.5; //max y-val
        box_pt_min(2)= -0.06; //min z-val search...just above table height
        box_pt_max(2)= 0.05; //max z-val
       


        
            //find_indices_box_filtered(transformed_cloud_ptr,box_pt_min,box_pt_max,indices);
            find_indices_box_filtered2(transformed_cloud_ptr,indices);
            pcl::copyPointCloud(*pclKinect_clr_ptr, indices, *box_filtered_cloud_ptr); //extract these pts into new cloud
            //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
            pcl::toROSMsg(*box_filtered_cloud_ptr, ros_box_filtered_cloud); //convert to ros message for publication and display

          while (ros::ok()) {
            pubBoxFilt.publish(ros_box_filtered_cloud);
            pubCloud.publish(ros_cloud);
            ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
            ros::Duration(0.3).sleep();
        }
        //pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        //pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
        //pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        //pubBoxFilt.publish(ros_box_filtered_cloud);
        //ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        //ros::Duration(0.3).sleep();


    return 0;
}
