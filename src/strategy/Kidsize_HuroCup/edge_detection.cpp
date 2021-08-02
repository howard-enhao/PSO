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
        
    }
    checkImageSource = false;
}