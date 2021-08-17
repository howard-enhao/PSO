#include <strategy/edge_detection.h>

void Edge_detection::Catch_image(const sensor_msgs::ImageConstPtr& msg)
{
    if(!checkRealImage)
    {
        cv_bridge::CvImagePtr cv_ptr;
        checkImageSource = true;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        orign_img = cv_ptr->image;
    }
    else
    {
        ROS_ERROR("Please Close Real Image and restart the node.");
    }
}

string type2str(int type)
{
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Edge_detection");
    ros::NodeHandle nh;
    Edge_detection Edge_detection(nh);

    ros::Rate loop_rate(30);

    while(nh.ok())
    {
        Edge_detection.strategymain();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Edge_detection::strategymain()
{
    if(!orign_img.empty() && checkImageSource)
    {
        orign_img = imread("/home/ching/git/test.png");
        imshow("view", orign_img);
        waitKey(30);
        Canny(orign_img, edge, 50, 150, 3);
        imshow("edge", edge);
        waitKey(30);
        cvtColor(edge, frame, cv::COLOR_GRAY2BGR);
        edgeimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        edgeimage_Publisher.publish(edgeimage_msg);

        string ty =  type2str( edge.type() );
        printf("Matrix: %s %dx%d \n\n\n\n", ty.c_str(), edge.cols, edge.rows );
  
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        vector<Point3i> edge_point;
        findContours(edge,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
        Mat imageContours=Mat::zeros(edge.size(),CV_8UC1);
        Mat Contours=Mat::zeros(edge.size(),CV_8UC1);  // 繪製
        for(int i=0;i<contours.size();i++)
        {
            // contours[i]代表的是第i個輪廓，contours[i].size()代表的是第i個輪廓上所有的像素點數
            for(int j=0;j<contours[i].size();j++) 
            {
                // 繪製出contours向量內所有的像素點
                Point P=Point(contours[i][j].x,contours[i][j].y);
                Contours.at<uchar>(P)=255;
                edge_point.push_back(Point3i(contours[i][j].x,contours[i][j].y, 0));
                printf("(i,j) = (%d, %d) , (x,y) = (%d, %d)\n", i, j, contours[i][j].x, contours[i][j].y);
            }
        
            // 輸出hierarchy向量内容
            char ch[256];
            sprintf(ch,"%d",i);
            string str=ch;
            cout<<"向量hierarchy的第" <<str<<" 個元素内容為："<<hierarchy[i]<<endl<<endl;
            printf("point  %d\n", Computational_geometry->isPointInPolygon(edge_point, Point3i(179, 61, 0)));
            edge_point.clear();
            cout<<endl<<endl<<endl;
            // 繪製輪廓
            drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);
        }
        imshow("Contours Image",imageContours); //輪廓
        imshow("Point of Contours",Contours);   //向量contours内保存的所有輪廓點集
        waitKey(30);
        for(int i = 0;i<100000000; i++);
    }
    checkImageSource = false;
}