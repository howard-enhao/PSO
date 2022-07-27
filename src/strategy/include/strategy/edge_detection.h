#ifndef IMAGEMAIN_H
#define IMAGEMAIN_H

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <map>
#include <stack>
#include <algorithm>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "tku_libs/WalkContinuouse.h"
#include "strategy/basketballinfo.h"
#include "strategy/loadparameter.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "strategy/pso.h"
#include "strategy/computational_geometry.h"
#include <geometry_msgs/Polygon.h>
#include "strategy/EdgePointArray.h"
#include "strategy/EdgePointList.h"
#include "strategy/ReachableRegion.h"
#include "std_msgs/Bool.h"

using namespace std;
using namespace cv;



class Edge_detection 
{
    public:
        Edge_detection(ros::NodeHandle &nh)
        {
            strategy_info = StrategyInfoInstance::getInstance();
            Imagesource_subscriber = nh.subscribe("/vision/object_image", 10, &Edge_detection::Catch_image, this);
            image_transport::ImageTransport it(nh);
            edgeimage_Publisher = it.advertise("edge_image", 1);
            Computational_geometry = Computational_geometryInstance::getInstance();
            edgepoint_pub = nh.advertise<strategy::EdgePointList>("/edgepoint_Topic", 1);
            Reachable_region_pub = nh.advertise<strategy::ReachableRegion>("/ReachableRegion_Topic", 1);
            stepcheck_subscriber = nh.subscribe("/stepcheck", 1, &Edge_detection::stepcheck_callback, this);
            FPGAack_subscriber = nh.subscribe("/package/footstepack", 1, &Edge_detection::Footstepack_callback, this);
        };
        ~Edge_detection(){};

        ros::Subscriber Imagesource_subscriber;
        ros::Publisher edgepoint_pub;
        ros::Publisher Reachable_region_pub;
        ros::Subscriber stepcheck_subscriber;
        ros::Subscriber FPGAack_subscriber;
        StrategyInfoInstance *strategy_info;
        Computational_geometryInstance *Computational_geometry;
        image_transport::Publisher edgeimage_Publisher;
        sensor_msgs::ImagePtr edgeimage_msg;
        strategy::ReachableRegion reachable_region;
        void strategymain(ros::NodeHandle nh);
        void Catch_image(const sensor_msgs::ImageConstPtr& msg);
        void stepcheck_callback(const std_msgs::Bool& msg);
        void Footstepack_callback(const std_msgs::Bool& msg);
        void initial();
        void save_image(const Mat orignimage);

        char path_1[200] = "/home/iclab/Desktop/PSO/finalimage.png";
        char path_2[200] = "/home/iclab/Desktop/PSO/data/orign";
        Mat edge;
        Mat orign_img;
        Mat frame;
        bool checkRealImage = false;
        bool checkImageSource = false;
        bool odd_step;
        int now_step = 0;
        int pre_now_step = 999;
        bool Footstepack;
        int name_cnt;
        int height_2;
    // private:
};




#endif // IMAGEMAIN_H