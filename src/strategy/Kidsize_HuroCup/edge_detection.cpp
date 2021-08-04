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

string type2str(int type) {
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
        imshow("view", orign_img);
        waitKey(30);
        Canny(orign_img, edge, 50, 150, 3);
        imshow("edge", edge);
        waitKey(30);
        cvtColor(edge, frame, cv::COLOR_GRAY2BGR);
        edgeimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        edgeimage_Publisher.publish(edgeimage_msg);
        ROS_INFO("width %d , height %d", edge.rows, edge.cols);

        string ty =  type2str( edge.type() );
        printf("Matrix: %s %dx%d \n", ty.c_str(), edge.cols, edge.rows );
        // for(int i = 0; i<edge.cols; i++)
        // {
        //     for(int j = 0; j<edge.rows; j++)
        //     {
        //         printf("%*u ", frame.at<uchar>(i,j));
        //     }
        //     printf("\n");
        // }
        // printf("\n\n\n");
        for(int i = 0;i<100000000; i++);
    }
    checkImageSource = false;
}