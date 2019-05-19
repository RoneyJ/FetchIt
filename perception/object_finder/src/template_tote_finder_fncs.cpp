//tote finding function utilizing template matching
//alpha group of EECS 376 Spring 2019
//Josh Roney, Chris Tam, Jason Sun, Tyler Anderson

#include <stdlib.h>
#include <math.h>
#include <stdio.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
//#include<object_finder_as/object_finder.h>
float min_points=40.0, max_points=25.0;

//static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;
using namespace cv;

const float MIN_X_t = 0.35; //include points starting 0.4m in front of robot
const float MAX_X_t = 5.0; //include points out to 0.9m in front of robot
const float MIN_Y_t = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y_t = 0.7; //include points up to 0.5m to right of robot
const float MIN_DZ_t = 0.02; //box filter from this height above the table top
const float MAX_DZ_t = 0.2; //consider points up to this height above table top
const float MIN_H_ABove_Table = 0.07;
const float MAX_H_ABove_Table = 0.15;

//magic numbers for filtering pointcloud points:
//specify the min and max values, x,y, znd z, for points to retain...
// as expressed in the robot's torso frame (torso_lift_link)





XformUtils *g_xform_ptr;


double convert_rad_to_deg(double radian){ 
    //double pi = 3.14159; 
    return(radian * (180.0/M_PI)); 
} 

std::vector<float> count_bigger(const std::vector<float>& elems) {
    float convrt = 1.0;
    std::vector<float> new_e;
    if( std::count_if(elems.begin(), elems.end(), [](float c){return c > 0;}) < elems.size()/2)
    {
        convrt =-1.0;
    }
    for(std::vector<float>::size_type i = 0; i != elems.size(); i++)
        new_e.push_back(convrt*abs(elems[i]));
    return new_e;
    
}

bool ObjectFinder::find_totes(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {

	geometry_msgs::PoseStamped object_pose;
	object_poses.clear();
	vector<Vec4i> line_group1;
	vector<Vec4i> line_group2;
	vector<Vec4i> test_group;
	vector<float> angle_group1;
	vector<float> angle_group2;
	vector<float> length_group1;
	vector<float> length_group2;
	vector<float> test_group1;
	vector<float> test_group2;
	Point p1, p2;
	Mat dst1, cdst, cdstP;
	vector<Vec4i> linesP; // will hold the results of the detection
	float kit_orientation; 
	int num_short_1 = 0;
	int num_short_2 = 0;

    while(linesP.size() <= 0){

	Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X_t, MIN_Y_t, table_height + MIN_H_ABove_Table ,0; //1cm above table top
    box_pt_max << MAX_X_t, MAX_Y_t, table_height+ MAX_H_ABove_Table,0;
    ROS_INFO("Finding blobs");
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    float npts_cloud = indices_.size();



    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs,viable_labels_);

    if(viable_labels_.size() >0){
   
     //this is needed to update openCV display windows;
    
  
    ROS_INFO("Started hough tranformation");

    imwrite( "BucketImage.png", g_dst );
    Mat dst = imread( "BucketImage.png", IMREAD_GRAYSCALE );
    
    // Edge detection
    Canny(dst, dst1, 50, 200, 3);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst1, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();
    // Probabilistic Line Transform
    
    //while(linesP.size() <= 0){
        HoughLinesP(dst, linesP, 1, CV_PI/180, 20, min_points, max_points ); // runs the actual detection
        // Draw the lines
        ROS_INFO("drawing lines..");
        if(linesP.size() <= 0){
            continue;
        }
    //}
    ROS_INFO("lines drawn");
   
    p1=Point(linesP[0][0], linesP[0][1]);
    p2=Point(linesP[0][2], linesP[0][3]);
    //cout<<"start point: "<<p1.x << " "<< p1.y << endl;
    //cout<<"end point: "<<p2.x << " "<< p2.y << endl;
    float comparison = convert_rad_to_deg(atan2(p1.y - p2.y, p1.x - p2.x));
    line_group1.push_back(linesP[0]);
    angle_group1.push_back(comparison);

    //cout<<"comparison: "<<comparison<<endl;
    float sum_group1 = 0.0,sum_group2=0.0;
    for( size_t i = 1; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        p1=Point(l[0], l[1]);
        p2=Point(l[2], l[3]);
        float length = sqrt(pow((l[2] - l[0]),2) + pow((l[3] - l[1]),2));
        float angle = atan2(p1.y - p2.y, p1.x - p2.x);
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        float deg = convert_rad_to_deg(angle);
       
        float diff = fabs(comparison - deg);
        if((diff <= 20.0 && diff >= 0.0) || (diff <= 200.0 && diff >= 160.0) || (diff <= 360 && diff >= 340))

        {
            line_group1.push_back(l);
            angle_group1.push_back(deg);
            length_group1.push_back(length);
          
        }
        else {
         line_group2.push_back(l);
         angle_group2.push_back(deg);
         length_group2.push_back(length);
       }

    }

    bool abs1 = false;
    bool abs2 = false;
    for(int i = 0; i < angle_group1.size()-1 && !abs1; i++){
        if((angle_group1[i] < 0 && angle_group1[i+1] > 0) || (angle_group1[i] > 0 && angle_group1[i+1] < 0))
            abs1 = true;
    }
    for(int i = 0; i < angle_group2.size()-1 && !abs2; i++){
        if((angle_group2[i] < 0 && angle_group2[i+1] > 0) || (angle_group2[i] > 0 && angle_group2[i+1] < 0))
            abs2 = true;
    }

    for(int i = 0; i < angle_group1.size(); i++){
        if(abs1)
            angle_group1[i] = fabs(angle_group1[i]);
        else if(angle_group1[i] < 0)
            angle_group1[i] += 180;
    }
    for(int i = 0; i < angle_group2.size(); i++){
        if(abs2)
            angle_group2[i] = fabs(angle_group2[i]);
        else if(angle_group2[i] < 0)
            angle_group2[i] += 180;
    }

    if(angle_group2[0] < angle_group1[0]){
        for(int i = 0; i < angle_group1.size(); i++){
                angle_group2.push_back(angle_group1[i] - 90);
        }
        test_group1 = angle_group2;
    }
    else{
        for(int i = 0; i < angle_group2.size(); i++){
                angle_group1.push_back(angle_group2[i] - 90);
        }
        test_group1 = angle_group1;
    }

    for(int i = 0; i < test_group1.size(); i++){
        sum_group1 += test_group1[i];
    }

    kit_orientation = 90 - (sum_group1 / float(test_group1.size()));
    cout<<"average orientation of kit = "<<kit_orientation<<endl;
    
    std::string image_path = ros::package::getPath("object_finder")+"/template/new_template.jpg";
    Mat src = cv::imread(image_path, CV_LOAD_IMAGE_UNCHANGED);
    Mat dest,dest1,dest2,dest3;

    Point2f pc(src.cols/2., src.rows/2.);
    //Template 1
    Mat r = cv::getRotationMatrix2D(pc, kit_orientation, 1.0);

    warpAffine(src, dest, r, src.size()); // what size I should use?

    imwrite("rotated_template1.png", dest);

    //Template 2
    r = cv::getRotationMatrix2D(pc, kit_orientation + 90, 1.0);

    warpAffine(src, dest1, r, src.size()); // what size I should use?

    imwrite("rotated_template2.png", dest1);

    //Template 3
    r = cv::getRotationMatrix2D(pc, kit_orientation + 180, 1.0);

    warpAffine(src, dest2, r, src.size()); // what size I should use?

    imwrite("rotated_template3.png", dest2);

    //Template 4
    r = cv::getRotationMatrix2D(pc, kit_orientation + 270, 1.0);

    warpAffine(src, dest3, r, src.size()); // what size I should use?

    imwrite("rotated_template4.png", dest3);

    //using SQDIFF_NORMED method for matching
    int match_method = CV_TM_SQDIFF_NORMED;

    Mat result1, result2, result3, result4;

    imwrite("Hough.png", cdstP);

    //Template match and normalize 1
    matchTemplate(cdstP,dest,result1,match_method);
    //normalize(result1,result1,0,255,NORM_MINMAX,-1,Mat());

    //Template match and normalize 2
    matchTemplate(cdstP,dest1,result2,match_method);
    //normalize(result2,result2,0,255,NORM_MINMAX,-1,Mat());

    //Template match and normalize 3
    matchTemplate(cdstP,dest2,result3,match_method);
    //normalize(result3,result3,0,255,NORM_MINMAX,-1,Mat());

    //Template match and normalize 4
    matchTemplate(cdstP,dest3,result4,match_method);
    //normalize(result4,result4,0,255,NORM_MINMAX,-1,Mat());

    double minVal, maxVal;
    Point minLoc, maxLoc;

    vector<double> minVals;

    //Find minVal of first template match
    minMaxLoc(result1, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    minVals.push_back(minVal);

    //Find minVal of second template match
    minMaxLoc(result2, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    minVals.push_back(minVal);

    //Find minVal of third template match
    minMaxLoc(result3, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    minVals.push_back(minVal);

    //Find minVal of fourth template match
    minMaxLoc(result4, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    minVals.push_back(minVal);

    double minimum = double(INT_MAX);
    int select;

    cout<<"minVal entries"<<endl;
    for(int i = 0; i < minVals.size(); i++){
        if(minVals[i] < minimum){
            minimum = minVals[i];
            select = i;
        }
        cout<<"minVal = "<<minVals[i]<<endl;
    }

    cout<<"selected entry is "<<select<<endl;

    switch(select) {
        case 0: kit_orientation = kit_orientation;
        break;
        case 1: kit_orientation = kit_orientation + 90;
        break;
        case 2: kit_orientation = kit_orientation + 180;
        break;
        case 3: kit_orientation = kit_orientation + 270;
        break;
        default:
            ROS_WARN("no valid minVal, quitting");
            return false;
    }

    cout<<"kit orientation = "<<kit_orientation<<endl;

    pair<int, int> center_arr[line_group1.size()];
    pair<int, int> center_arr2[line_group1.size()];
    for( size_t i = 0; i < line_group1.size(); i++ )
    {
         p1=Point(line_group1[i][0], line_group1[i][1]);
         p2=Point(line_group1[i][2], line_group1[i][3]);
         center_arr[i] = make_pair((float)((p1.x+p2.x)/2),(float)((p1.y+p2.y)/2));
         center_arr2[i] = make_pair((float)((p1.y+p2.y)/2),(float)((p1.x+p2.x)/2));

    }
    sort(center_arr, center_arr + line_group1.size());
    sort(center_arr2, center_arr2 + line_group1.size());

    Mat img = imread("Hough.png");

    float obj_x_centroid = (float)((center_arr[0].first+center_arr[line_group1.size()-1].first)/2);
    float obj_y_centroid = (float)((center_arr2[0].first+center_arr2[line_group1.size()-1].first)/2);
    Point p(obj_x_centroid,obj_y_centroid); 

    circle(img, p, 5, Scalar(127,0,0), -1);
    ////imshow("Image with center",img);
    Mat fixed_dst(img.size(), CV_8UC3);
    float obj_x_centroid_wrt_robot = ((fixed_dst.rows/2) - obj_y_centroid)/PIXELS_PER_METER + (MIN_X+MAX_X)/2.0;
    float obj_y_centroid_wrt_robot = ((fixed_dst.cols/2) - obj_x_centroid)/PIXELS_PER_METER + (MIN_Y+MAX_Y)/2.0;
    cout<<"fixed_dst rows = "<<fixed_dst.rows<<endl;
    cout<<"Centroid of the object in image coordinates: (" <<obj_x_centroid<<","<<obj_y_centroid<<")\n";
    cout<<"Centroid of the object in robot coordinates: (" <<obj_x_centroid_wrt_robot<<","<<obj_y_centroid_wrt_robot<<")\n";

    double double_kit_orientation = (double) ((kit_orientation+90) * (M_PI/180));

    XformUtils transform;

    object_pose.pose.position.x = obj_x_centroid_wrt_robot;
    object_pose.pose.position.y = obj_y_centroid_wrt_robot;
    object_pose.pose.position.z = 0.08;
    object_pose.pose.orientation = transform.convertPlanarPsi2Quaternion(double_kit_orientation);
    object_poses.push_back(object_pose);

    //waitKey();        
    return true;
   }
    else {
        ROS_WARN("No object found...:<");
        return false;
    }
}
} 
