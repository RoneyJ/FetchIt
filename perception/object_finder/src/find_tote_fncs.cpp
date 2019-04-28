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

bool ObjectFinder::find_totes(vector<float> x_centroids_wrt_robot, vector<float> y_centroids_wrt_robot,
            vector<float> avg_z_heights, vector<float> npts_blobs, float table_height, vector<geometry_msgs::PoseStamped> &object_poses) {
    geometry_msgs::PoseStamped object_pose;
    object_poses.clear();
    int n_objects = x_centroids_wrt_robot.size();
    if (n_objects < 2) return false; //background is object 0
    
    geometry_msgs::PoseStamped kit_pose;
    kit_pose.header.frame_id = "torso_lift_link";
    
    //cv::Mat singulated_image;
    Mat_<uchar> singulated_image(Nu, Nv);
    Mat_<uchar> edge_image(Nu, Nv);   
    Mat_<uchar> blurred_image(Nu, Nv);     
    //singulated_image = g_bw_img.clone();
    int n_viable_labels = viable_labels_.size();
    int viable_label;
    ROS_INFO("analyzing %d blobs with hough transform",n_viable_labels);
    for (int i_object = 1; i_object < n_viable_labels; i_object++) {
        viable_label = viable_labels_[i_object];
        ROS_INFO("object %d analysis: ",i_object);
        //make a binary image, g_dst, from pixels of object i_object:
		for (int r = 0; r < singulated_image.rows; ++r) {
			for (int c = 0; c < singulated_image.cols; ++c){
				int label_num = g_labelImage.at<int>(r,c);
				if(label_num == viable_label){
                                    singulated_image.at<int>(r, c) = 255;  //g_bw_img(u, v) = (unsigned char) grayval;  g_labelImage.at<int>(r, c);
				}
                                else {
                                    singulated_image.at<int>(r, c) = 255;
                                }
			}
		}        



   
     //this is needed to update openCV display windows;
    
    ROS_INFO("Started hough transformation");
    //Mat dst = imread( "BucketImage.png", IMREAD_GRAYSCALE );
    //Mat dst, dst1, cdst, cdstP;
    //want to extract pixels from each label
    //Mat detected_edges;
    //detected_edges = g_bw_img.clone();
    //detected_edges = singulated_image.clone();
    /// Reduce noise with a kernel 3x3
    ROS_INFO("blurring image: ");
    //blur( singulated_image, detected_edges, Size(3,3) );  
    blur( singulated_image, blurred_image, Size(3,3) );   
    
    // Edge detection
    //  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    ROS_INFO("performing edge detection: ");
    //Canny(detected_edges, detected_edges, 50, 200, 3);
    Canny(blurred_image, edge_image, 50, 200, 3);
    cout<<"enter 1: ";
    int ans;
    cin>>ans;

    
    // Copy edges to the images that will display the results in BGR
    //cvtColor(dst1, cdst, COLOR_GRAY2BGR);
    //cdstP = cdst.clone();
    
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    
    //void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )
    //HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    //dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
    //lines: A vector that will store the parameters (r,\theta) of the detected lines
    //rho : The resolution of the parameter r in pixels. We use 1 pixel.
    //theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    //threshold: The minimum number of intersections to “detect” a line
    //srn and stn: Default parameters to zero. Check OpenCV reference for more info.
    float min_points=40.0, max_points=25.0;    
    ROS_INFO("computing Hough transform: ");
    //HoughLinesP(detected_edges, linesP, 1, CV_PI/180, 20, min_points, max_points ); // runs the actual detection
    HoughLinesP(edge_image, linesP, 1, CV_PI/180, 20, min_points, max_points ); // runs the actual detection
    
    // Draw the lines
    //cout<<"drawing lines\n";
    
    cout<<"number of lines "<<linesP.size()<<endl;
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
    p1=Point(linesP[0][0], linesP[0][1]);
    p2=Point(linesP[0][2], linesP[0][3]);
    cout<<"start point: "<<p1.x << " "<< p1.y << endl;
    cout<<"end point: "<<p2.x << " "<< p2.y << endl;
    float comparison = convert_rad_to_deg(atan2(p1.y - p2.y, p1.x - p2.x));
    line_group1.push_back(linesP[0]);
    angle_group1.push_back(comparison);
     /*if(fabs(fabs(comparison) - 180.0) <= 4 || fabs(fabs(comparison) - 90.0) <= 4){
        	comparison = fabs(comparison);
        	
        }*/
    cout<<"comparison: "<<comparison<<endl;
    float sum_group1 = 0.0;
    float sum_group2=0.0;
    float test_sum;
    for( size_t i = 1; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        p1=Point(l[0], l[1]);
        p2=Point(l[2], l[3]);
        cout<<"start point: "<<p1.x << " "<< p1.y << endl;
        cout<<"end point: "<<p2.x << " "<< p2.y << endl;        
        float length = sqrt(pow((l[2] - l[0]),2) + pow((l[3] - l[1]),2));
        float angle = atan2(p1.y - p2.y, p1.x - p2.x);
        //line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        float deg = convert_rad_to_deg(angle);
       
        cout<<"length: "<<length<<"; Degree: "<<deg<<endl;
        float diff = fabs(comparison - deg);
        if((diff <= 15.0 && diff >= 0.0) || (diff <= 195.0 && diff >= 165.0) || (diff <= 360 && diff >= 345))

        {
            line_group1.push_back(l);
            angle_group1.push_back(deg);
            length_group1.push_back(length);
            //sum_group1+=deg;

        }
        else {
         line_group2.push_back(l);
         angle_group2.push_back(deg);
         length_group2.push_back(length);
         //sum_group2+=deg;
        }

        



     //   cout<<"The start coordinates are :"<<linesP[i][0] << "," << linesP[i][1] << endl;
      //  cout<<"The end coordinates are :"<< linesP[i][2] << "," << linesP[i][3] << endl;
       // cout<<"The angle in radians is :"<<angle << "\nangle in degrees: "<<deg<<endl;
       // cout<<"The length of the line is: " << length << endl;
        //cout << "=================" << endl;

    }
    test_group1 = count_bigger(angle_group1);
    test_group2 = count_bigger(angle_group2);

    cout<<"Group 1 degrees"<<endl;
    for(size_t i = 0; i < test_group1.size(); i++){
        sum_group1 += test_group1[i];
        cout<<"deg = "<<test_group1[i]<<endl;
    }
    cout<<"group 2 degrees"<<endl;
    for(size_t i = 0; i < test_group2.size(); i++){
        sum_group2 += test_group2[i];
        cout<<"deg = "<<test_group2[i]<<endl;
    }

    cout<<"group 1 lengths"<<endl;
    int num_short_1 = 0;
    int num_short_2 = 0;
    for(int i = 0; i < length_group1.size(); i++){
    	cout<<length_group1[i]<<endl;
    	if(length_group1[i] <= 60 && length_group1[i] >= 40)
    			num_short_1++;
    }
    cout<<"group 2 lengths"<<endl;
    for(int i = 0; i < length_group2.size(); i++){ 
    	cout<<length_group2[i]<<endl;
    	if(length_group2[i] <= 60 && length_group2[i] >= 40)
    			num_short_2++;
    }

    float kit_orientation; 
    cout<<"Group 1 size: "<<angle_group1.size()<<endl;
    cout<<"Group 2 size: "<<angle_group2.size()<<endl;
    cout<<"num_short_1: "<<num_short_1<<endl;
    cout<<"num_short_2: "<<num_short_2<<endl;



    //changed how to choose test group properly**
    //PARSE THROUGH GROUPS, FIND ONE WITH LESS LENGTHS BETWEEN 45-60(? FINE TUNE), CHOOSE AS TEST
    
    if(num_short_1 <= num_short_2){
        kit_orientation = sum_group1 / float(test_group1.size());
        test_group = line_group1;
    }
    else{
        kit_orientation = sum_group2/ float(test_group2.size());
        test_group = line_group2;
    }

    ROS_INFO("Orientation of kit is: %f", kit_orientation);
    double double_kit_orientation = (double) (kit_orientation * (M_PI/180.0));
//x_centroids_wrt_robot
    kit_pose.pose.position.x = x_centroids_wrt_robot[i_object];
    kit_pose.pose.position.y = y_centroids_wrt_robot[i_object];
    kit_pose.pose.orientation = xformUtils_.convertPlanarPsi2Quaternion(double_kit_orientation);
    
    object_poses.push_back(kit_pose);
    
        }
//#################################################################################################
    /*
    cout<<"Completed\n";
    // Show results
    //imshow("Source", dst);
    //imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
    imwrite( "TestImage.png", cdstP );
    pair<int, int> center_arr[test_group.size()];
    pair<int, int> center_arr2[test_group.size()];
    for( size_t i = 0; i < test_group.size(); i++ )
    {
         p1=Point(test_group[i][0], test_group[i][1]);
         p2=Point(test_group[i][2], test_group[i][3]);
         center_arr[i] = make_pair((float)((p1.x+p2.x)/2),(float)((p1.y+p2.y)/2));
         center_arr2[i] = make_pair((float)((p1.y+p2.y)/2),(float)((p1.x+p2.x)/2));

    }
    sort(center_arr, center_arr + test_group.size());
    sort(center_arr2, center_arr2 + test_group.size());
    cout<<"Test group size is: "<<test_group.size()<<endl;
  
    Mat bw,thr, img = imread("TestImage.png");
    float obj_x_centroid = (float)((center_arr[0].first+center_arr[test_group.size()-1].first)/2);
    float obj_y_centroid = (float)((center_arr2[0].first+center_arr2[test_group.size()-1].first)/2);
    Point p(obj_x_centroid,obj_y_centroid); 

    circle(img, p, 5, Scalar(127,0,0), -1);
    imshow("Image with center",img);
    Mat fixed_dst(img.size(), CV_8UC3);
    float obj_x_centroid_wrt_robot = ((fixed_dst.rows/2) - obj_x_centroid)/PIXELS_PER_METER + (MIN_X+MAX_X)/2.0;
    float obj_y_centroid_wrt_robot = (obj_y_centroid- (fixed_dst.cols/2))/PIXELS_PER_METER + (MIN_Y+MAX_Y)/2.0;
    cout<<"Centroid of the object in image coordinates: (" <<obj_x_centroid<<","<<obj_y_centroid<<")\n";
    cout<<"Centroid of the object in robot coordinates: (" <<obj_x_centroid_wrt_robot<<","<<obj_y_centroid_wrt_robot<<")\n";

    XformUtils transform;
    double pi = 3.14159; 
     * */
    
            

return true;
} 
