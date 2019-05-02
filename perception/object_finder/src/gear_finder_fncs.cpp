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


//here is old, non-functional code:
double getOrientation(vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
 
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
 
    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));
 
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
 
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
 
    // Draw the principal components
    circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    line(img, pos, pos + 0.02 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
    line(img, pos, pos + 0.02 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));
 
    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}

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



 	imwrite( "small_gearImage.png", g_dst );
    Mat bw,img = imread( "small_gearImage.png", IMREAD_GRAYSCALE );
    
    // Copy edges to the images that will display the results in BGR
    cvtColor(img, bw, COLOR_GRAY2BGR);
    cvtColor(bw, bw, COLOR_BGR2GRAY);


	// Apply thresholding;
	threshold(bw, bw, 150, 255, CV_THRESH_BINARY);
	 
	// Find all the contours in the thresholded image
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	cout<<"size of countours is: "<<contours.size()<<endl;
	for (size_t i = 0; i < contours.size(); ++i)
	{
	    // Calculate the area of each contour
	    double area = contourArea(contours[i]);
	    //cout<<"area is:"<<area<<endl;
	    // Ignore contours that are too small or too large
	    if ( area <2 ||1e5 < area) continue;
	 
	    // Draw each contour only for visualisation purposes
	    drawContours(img, contours, i, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);
	    //imshow("Image with center",img);
	    imwrite("Image_w_coubntors.png",img);
	    // Find the orientation of each shape
	    countours_area.push_back(area);
	    countours_orientation.push_back(getOrientation(contours[i], img));
	}



	XformUtils transform;
    int nlabels = viable_labels_.size();
    if ( nlabels>0)
    {
    	
    	for (int label = 0; label < nlabels; ++label) {
        ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);
        if (avg_z_heights[label] <= 91) 
        { 
            if (npts_blobs[label] >=  min_small_gear_pts && npts_blobs[label] <=  max_small_gear_pts ) {

            	object_pose.pose.position.x = x_centroids_wrt_robot[label];
			    object_pose.pose.position.y = y_centroids_wrt_robot[label];
			    object_pose.pose.position.z = 0.005;
			    object_pose.pose.orientation = transform.convertPlanarPsi2Quaternion(countours_orientation[max_element(countours_area.begin(),countours_area.end()) - countours_area.begin()]);
			    object_poses.push_back(object_pose);

            	return true;
            }
            else return false;
        }
    	
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
    if (viable_labels_.size() >0){
    	imwrite( "small_gearImage.png", g_dst );
    	return true;
    }
    else return false; 
}


