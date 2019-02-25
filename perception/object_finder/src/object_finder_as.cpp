// action server to respond to perception requests
// Wyatt Newman, 2/2019

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<pcl_utils/pcl_utils.h>
#include<object_finder/objectFinderAction.h>
#include <xform_utils/xform_utils.h>

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Eigen::Affine3f g_affine_headcam_wrt_base;
//magic numbers for filtering pointcloud points:
//specify the min and max values, x,y, znd z, for points to retain...
// as expressed in the robot's torso frame (torso_lift_link)
const float MIN_X = 0.4; //include points starting 0.4m in front of robot
const float MAX_X = 0.9; //include points out to 0.9m in front of robot
const float MIN_Y = -0.5; //include points starting -0.5m to left of robot
const float MAX_Y = 0.5; //include points up to 0.5m to right of robot
const float MIN_Z = -0.05; //2cm above the table top
const float MAX_Z = 0.1; //consider points up to this height w/rt torso frame

const float TABLE_TOP_MIN = -0.1;
const float TABLE_TOP_MAX = 0.05;
const float TABLE_GRASP_CLEARANCE = 0.01; //add this much to table  top height for gripper clearance

//choose, e.g., resolution of 5mm, so 100x200 image for 0.5m x 1.0m pointcloud
// adjustable--> image  size
// try 400 pix/m...works fine, i.e. 2.5mm resolution
const float PIXELS_PER_METER = 400.0; //200.0;

const int Nu = (int) ((MAX_X - MIN_X) * PIXELS_PER_METER);
const int Nv = (int) ((MAX_Y - MIN_Y) * PIXELS_PER_METER);

Mat_<uchar> g_bw_img(Nu, Nv);
Mat_<int> g_labelImage(Nu, Nv);
Mat dst(g_bw_img.size(), CV_8UC3);

vector<float> g_x_centroids,g_y_centroids;
vector<float> g_x_centroids_wrt_robot,g_y_centroids_wrt_robot;
vector<float> g_avg_z_heights;
vector<float> g_npts_blobs;
Eigen::Affine3f affine_cam_wrt_torso_;



class ObjectFinder {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;

    // here are some message types to communicate with our client(s)
    object_finder::objectFinderGoal goal_; // goal message, received from client
    object_finder::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    //PclUtils pclUtils_;
    tf::TransformListener* tfListener_;
    void initializeSubscribers();
    void initializePublishers();


    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pose
    //bool find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose);
    //bool find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose);
    //float find_table_height();

    double surface_height_;
    bool found_surface_height_;
    bool got_headcam_image_;
    

    vector<int> indices_;
        //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing

    Eigen::Affine3f affine_cam_wrt_torso_;

    
public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal);
    void headcamCB(const sensor_msgs::PointCloud2ConstPtr& cloud);

    XformUtils xformUtils_;
    ros::Subscriber pointcloud_subscriber_; //use this to subscribe to a pointcloud topic
    void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f box_pt_min, 
        Eigen::Vector3f box_pt_max, vector<int> &indices);
    void blob_finder(vector<float> &x_centroids_wrt_robot,vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights,vector<float> &npts_blobs);
    Eigen::Affine3f compute_affine_cam_wrt_torso_lift_link(void);
    float find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz);
    void convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr,  vector<int> indices);
    sensor_msgs::PointCloud2 ros_cloud_, downsampled_cloud_, ros_box_filtered_cloud_; //here are ROS-compatible messages
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCam_clr_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::Publisher pubCloud_;// = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubDnSamp_;// = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt_;// = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);  
    
    bool find_gearbox_top(vector<float> x_centroids_wrt_robot,vector<float>  y_centroids_wrt_robot, 
        vector<float> avg_z_heights,vector<float> npts_blobs, float table_height, vector<geometry_msgs::PoseStamped> &object_poses);
};

ObjectFinder::ObjectFinder() :
object_finder_as_(nh_, "object_finder_action_service", boost::bind(&ObjectFinder::executeCB, this, _1), false),pclCam_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
        box_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>),
transformed_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>) {
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation

    
    object_finder_as_.start(); //start the server running
    tfListener_ = new tf::TransformListener; //create a transform listener
    found_surface_height_=false;
    got_headcam_image_=false;
    initializeSubscribers();
    initializePublishers();    
    affine_cam_wrt_torso_ = compute_affine_cam_wrt_torso_lift_link();
    
    ROS_INFO("waiting for image data");
    while (!got_headcam_image_) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        got_headcam_image_=true;
    }
}

void ObjectFinder::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    pointcloud_subscriber_ = nh_.subscribe("/head_camera/depth_registered/points", 1, &ObjectFinder::headcamCB, this);
    // add more subscribers here, as needed
}

void ObjectFinder::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("/headcam_pointcloud", 1, true);
    pubDnSamp_ = nh_.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1,true);
    pubBoxFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1,true);
}

void ObjectFinder::headcamCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_headcam_image_) { // once only, to keep the data stable
        ROS_INFO("got new image");
        pcl::fromROSMsg(*cloud, *pclCam_clr_ptr_);
        ROS_INFO("image has  %d * %d points", pclCam_clr_ptr_->width, pclCam_clr_ptr_->height);
        got_headcam_image_ = true;
    }
}

void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f box_pt_min,
        Eigen::Vector3f box_pt_max, vector<int> &indices) {
    int npts = input_cloud_ptr->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    cout << "box min: " << box_pt_min.transpose() << endl;
    cout << "box max: " << box_pt_max.transpose() << endl;
    for (int i = 0; i < npts; ++i) {
        pt = input_cloud_ptr->points[i].getVector3fMap();

        //check if in the box:
        if ((pt[0] > box_pt_min[0])&&(pt[0] < box_pt_max[0])&&(pt[1] > box_pt_min[1])&&(pt[1] < box_pt_max[1])&&(pt[2] > box_pt_min[2])&&(pt[2] < box_pt_max[2])) {
            //passed box-crop test; include this point
            //ROS_INFO("point passes test");
            //cout<<"pt passed test: "<<pt.transpose()<<endl;
            indices.push_back(i);
        }
    }
    int n_extracted = indices.size();
    cout << " number of points within box = " << n_extracted << endl;
}


//This fnc was used to confirm the table height relative to torso frame;
// It is no longer needed, since the table height, once found, can be hard coded
// i.e., 7cm below the torso-frame origin

float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz) {
    vector<int> indices;
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    pcl::PointXYZRGB point;
    int ans;
    for (float z = z_min; z < z_max; z += dz) {
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();

        if (npts > 0) ROS_INFO("z=%f; npts = %d", z, npts);
        if (npts > npts_max) {
            npts_max = npts;
            z_table = z + 0.5 * dz;
        }
    }
    ROS_INFO("max pts %d at height z= %f", npts_max, z_table);
    return z_table;
}

//hard-coded xform for Fetch w/ assumed torso lift and head angle:
//assuming head is tilted down 1.0rad, this is the head-camera frame
// with respect to the torso_lift_link frame, expressed as an Eigen::Affine

Eigen::Affine3f ObjectFinder::compute_affine_cam_wrt_torso_lift_link(void) {
    //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_torso;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam << 0.244, 0.02, 0.627;
    affine_cam_wrt_torso.translation() = O_cam;
    Eigen::Vector3f nvec, tvec, bvec;
    //magic numbers, as determined by rosrun tf tf_echo torso_lift_link head_camera_rgb_optical_frame
    // when running robot in Gazebo with headcam tilted down 1.0rad
    nvec << 0, -1, 0;
    tvec << -0.8442, 0, -0.5377;
    bvec << 0.5377, 0, -0.8442;
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;
    affine_cam_wrt_torso.linear() = R_cam;
    return affine_cam_wrt_torso;
}

//given a binary image in g_bw_img, find and label connected regions  (blobs)
// g_labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot
//OPERATES ON GLOBAL VARS g_bw_img and g_labelImage
void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot,vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights,
        vector<float> &npts_blobs) {

    //openCV function to do all the  hard work
    int nLabels = connectedComponents(g_bw_img, g_labelImage, 8); //4 vs 8 connected regions
    ROS_INFO("found %d blobs",nLabels);
    g_x_centroids.resize(nLabels);
    g_y_centroids.resize(nLabels);
    x_centroids_wrt_robot.resize(nLabels);
    g_y_centroids_wrt_robot.resize(nLabels);
    avg_z_heights.resize(nLabels);
    g_npts_blobs.resize(nLabels);
    for (int label = 0; label < nLabels; ++label) {
        g_x_centroids[label]=0.0;
        g_y_centroids[label]=0.0;
        x_centroids_wrt_robot[label]=0.0;
        y_centroids_wrt_robot[label]=0.0;
        avg_z_heights[label]=0.0;
        g_npts_blobs[label]=0.0;
    }
    //colorize the regions and display them:
    //also compute centroids;
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0); //make the background black
    //assign random color to each region label
    for (int label = 1; label < nLabels; ++label) {
        colors[label] = Vec3b((rand()&255), (rand()&255), (rand()&255));
    }

    //for human consumption: assign colors to regions and display result
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            int label = g_labelImage.at<int>(r, c);
            Vec3b &pixel = dst.at<Vec3b>(r, c);
            pixel = colors[label];
            //while  we're combing through pixels, compute centroids as well:
            g_y_centroids[label]+=c;
            g_x_centroids[label]+=r;
            g_npts_blobs[label]+=1.0;
        }
    }
    ROS_INFO("image size: %d cols, %d rows",dst.cols,dst.rows);
           
    for (int label = 0; label < nLabels; ++label) {
        g_x_centroids[label]/=g_npts_blobs[label];
        g_y_centroids[label]/=g_npts_blobs[label];

        ROS_INFO("label %d has %f points w/ centroid %f, %f:",label,g_npts_blobs[label],g_x_centroids[label],g_y_centroids[label]);
    }
    //convert to robot coords:
    //convert to dpixel_x, dpixel_y w/rt image centroid, scale by pixels/m, and add offset from PCL cropping, x,y
    for (int label = 1; label < nLabels; ++label) {    
        x_centroids_wrt_robot[label] = ((dst.rows/2) - g_x_centroids[label])/PIXELS_PER_METER + (MIN_X+MAX_X)/2.0;
        y_centroids_wrt_robot[label] = (g_y_centroids[label]- (dst.cols/2))/PIXELS_PER_METER + (MIN_Y+MAX_Y)/2.0;
        ROS_INFO("label %d has centroid w/rt robot: %f, %f:",label,x_centroids_wrt_robot[label],y_centroids_wrt_robot[label]);
                
    }    
    //display the result in an openCV window
    //imshow("Connected Components", dst); //do this  from "main" instead
}


//operates on global OpenCV matrices g_labelImage and g_bw_img
//provide a transformed cloud pointer and indices of interest (filtered points above table)
void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr,  vector<int> indices) {
    //for each pointcloud point, compute which pixel it maps to and find its z value
        //convert point cloud to top-down 2D projection for OpenCV processing
    float x, y, z;
    int index, u, v;
    Eigen::Vector3f cloud_pt;
    g_bw_img = 0; //initialize image to all black
    g_labelImage = 0; // ditto for result that  will get populated  with region labels
    int npts_cloud = indices.size();
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        index = indices[ipt];
        cloud_pt = transformed_cloud_ptr->points[index].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        //careful: some pointclouds include bad data; test for this
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            //compute pixel values from metric values
            v = round((y - MIN_Y) * PIXELS_PER_METER);
            u = round((x - MIN_X) * PIXELS_PER_METER);
            //flip/invert these so image makes sense visually
            u = Nu - u;
            v = Nv - v;
            //make sure the computed indices fit within the matrix size:
            if ((u > 0)&&(u < Nu - 1)&&(v > 0)&&(v < Nv - 1)) {
                g_bw_img(u, v) = 255; //assign this pixel to be white
            }
        }

    }
}


// if asked to find an object:
//  *take a snapshot
//  *find table height
//  *transform points to table frame
//  *box filter these points above table
//  *convert to 2D and find blobs
//  *call corresponding find-object function to recognize specific parts
//  *for blobs of interest, convert coords back to robot torso-lift frame
//  *fill result message with vector of poses of object of interest
void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    vector<geometry_msgs::PoseStamped> object_poses;
    /*
    bool known_surface_ht = goal->known_surface_ht;
    float surface_height;
    if (known_surface_ht) {
        surface_height = goal->surface_ht;
    }
     * */
    bool found_object = false;
    //get a fresh snapshot:
    got_headcam_image_=false;

    while (!got_headcam_image_) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep();
        ROS_INFO("waiting for snapshot...");
    }
    
    //if here, have a new cloud in *pclCam_clr_ptr_; 
    pcl::toROSMsg(*pclCam_clr_ptr_, ros_cloud_); //convert from PCL cloud to ROS message this way

    //transform this cloud to base-frame coords:
    ROS_INFO("transforming point cloud");
    //transform the head-camera data into the torso frame
    // result will be in "transformed_cloud_ptr_"
    pcl::transformPointCloud(*pclCam_clr_ptr_, *transformed_cloud_ptr_, affine_cam_wrt_torso_);

    //find table height from this snapshot:
    double table_height = find_table_height(transformed_cloud_ptr_, TABLE_TOP_MIN, TABLE_TOP_MAX, 0.01);
    
    //box-filter the points:
    //specify opposite corners of a box to box-filter the transformed points
    //magic numbers are at top of this program
    Eigen::Vector3f box_pt_min, box_pt_max;
    box_pt_min << MIN_X, MIN_Y, table_height+0.01; //1cm above table top
    box_pt_max << MAX_X, MAX_Y, MAX_Z;

    //find which points in the transformed cloud are within the specified box
    //result is in "indices"
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    int npts_cloud = indices_.size();

 

    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_,  indices_);
 
        //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]
    //
    blob_finder(g_x_centroids_wrt_robot,g_y_centroids_wrt_robot, g_avg_z_heights,g_npts_blobs);
    
    result_.object_id = goal->object_id; //by default, set the "found" object_id to the "requested" object_id
    //note--finder might change this ID, if warranted

    switch (object_id) {
        //coordinator::ManipTaskResult::FAILED_PERCEPTION:
        case object_finder::objectFinderGoal::GEARBOX_TOP:
            //specialized functions to find objects of interest:
            
            found_object = find_gearbox_top(g_x_centroids_wrt_robot,g_y_centroids_wrt_robot, g_avg_z_heights,g_npts_blobs,table_height,object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found gearbox_top objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose=0;ipose<nposes;ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }
            break;
 
        default:
            ROS_WARN("this object ID is not implemented");
            result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED;
            object_finder_as_.setAborted(result_);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder_node"); // name this node 

    ROS_INFO("instantiating the object finder action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    //tf::TransformListener tfListener;
    //ROS_INFO("listening for kinect-to-base transform:");
    //tf::StampedTransform stf_kinect_wrt_base;
    /*
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and base_link...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("base_link", "kinect_pc_frame", ros::Time(0), stf_kinect_wrt_base);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("kinect to base_link tf is good");
    object_finder_as.xformUtils_.printStampedTf(stf_kinect_wrt_base);
    tf::Transform tf_kinect_wrt_base = object_finder_as.xformUtils_.get_tf_from_stamped_tf(stf_kinect_wrt_base);
    g_affine_kinect_wrt_base = object_finder_as.xformUtils_.transformTFToAffine3f(tf_kinect_wrt_base);
    cout << "affine rotation: " << endl;
    cout << g_affine_kinect_wrt_base.linear() << endl;
    cout << "affine offset: " << g_affine_kinect_wrt_base.translation().transpose() << endl;
    */
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        object_finder_as.pubCloud_.publish(object_finder_as.ros_cloud_); //publish original point-cloud, so it is viewable in rviz        
        object_finder_as.pubBoxFilt_.publish(object_finder_as.ros_box_filtered_cloud_); //ditto for filtered point cloud   
        ros::Duration(0.1).sleep();
    }

    return 0;
}


bool ObjectFinder::find_gearbox_top(vector<float> x_centroids_wrt_robot,vector<float>  y_centroids_wrt_robot, 
        vector<float> avg_z_heights,vector<float> npts_blobs, float table_height,
        vector<geometry_msgs::PoseStamped> &object_poses) {
        geometry_msgs::PoseStamped object_pose;
        object_poses.clear();
        int n_objects = x_centroids_wrt_robot.size();
        if (n_objects<2) return false; //background is object 0
        object_pose.header.frame_id= "torso_lift_link";
        for (int i_object=1;i_object<n_objects;i_object++) {
            object_pose.pose.position.x = x_centroids_wrt_robot[i_object];
            object_pose.pose.position.y = y_centroids_wrt_robot[i_object];
            object_pose.pose.position.z = table_height + TABLE_GRASP_CLEARANCE;
            //FIX ME!  do not yet have value orientation info
            object_pose.pose.orientation.x = 0;
            object_pose.pose.orientation.y = 0;
            object_pose.pose.orientation.z = 0;
            object_pose.pose.orientation.w = 1;
            object_poses.push_back(object_pose);
        }
        return true;
}
//specialized function: DUMMY...JUST RETURN A HARD-CODED POSE; FIX THIS
/*
bool ObjectFinder::find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose) {
    bool found_object = true;
    object_pose.header.frame_id = "world";
    object_pose.pose.position.x = 0.680;
    object_pose.pose.position.y = -0.205;
    object_pose.pose.position.z = surface_height;
    object_pose.pose.orientation.x = 0;
    object_pose.pose.orientation.y = 0;
    object_pose.pose.orientation.z = 0;
    object_pose.pose.orientation.w = 1;
    return found_object;

}

bool ObjectFinder::find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose) {
    Eigen::Vector3f plane_normal;
    double plane_dist;
    //bool valid_plane;
    Eigen::Vector3f major_axis;
    Eigen::Vector3f centroid;
    bool found_object = true; //should verify this
    double block_height = 0.035; //this height is specific to the TOY_BLOCK model
    //if insufficient points in plane, find_plane_fit returns "false"
    //should do more sanity testing on found_object status
    //hard-coded search bounds based on a block of width 0.035
    found_object = pclUtils_.find_plane_fit(0.4, 1, -0.5, 0.5, surface_height + 0.025, surface_height + 0.045, 0.001,
            plane_normal, plane_dist, major_axis, centroid);
    // need more here, if want to distinguish different types of blocks;
    // write a new pclUtils fnc for this. Then set:
    //result_.object_id = CODE_OF_BLOCK_FOUND;
    //
    if (plane_normal(2) < 0) plane_normal(2) *= -1.0; //in world frame, normal must point UP
    Eigen::Matrix3f R;
    Eigen::Vector3f y_vec;
    R.col(0) = major_axis;
    R.col(2) = plane_normal;
    R.col(1) = plane_normal.cross(major_axis);
    Eigen::Quaternionf quat(R);
    object_pose.header.frame_id = "base_link";
    object_pose.pose.position.x = centroid(0);
    object_pose.pose.position.y = centroid(1);
    //the TOY_BLOCK model has its origin in the middle of the block, not the top surface
    //so lower the block model origin by half the block height from upper surface
    object_pose.pose.position.z = centroid(2)-0.5*block_height;
    //create R from normal and major axis, then convert R to quaternion

    object_pose.pose.orientation.x = quat.x();
    object_pose.pose.orientation.y = quat.y();
    object_pose.pose.orientation.z = quat.z();
    object_pose.pose.orientation.w = quat.w();
    return found_object;
}
*/