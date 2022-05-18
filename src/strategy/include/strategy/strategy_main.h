#ifndef PSO_STRATEGY_H
#define PSO_STRATEGY_H
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include<iostream>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "tku_libs/WalkContinuouse.h"
#include "FeatureDistance/FeatureDistance.h"

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include "strategy/obstacle.h"
#include "strategy/particle.h"
#include "strategy/step_space.h"
#include "strategy/ReachableRegion.h"
#include "strategy/pso.h"

using namespace std;
int freecenter[2] = {0};
// class KidsizeStrategy;
// typedef double (*pso_obj_fun_t)(double *pos, int dim, void *params);
class KidsizeStrategy : public FeatureDistance
{
	public:
		KidsizeStrategy(ros::NodeHandle &nh) 
		{
			image_transport::ImageTransport it(nh);
			strategy_info = StrategyInfoInstance::getInstance();
			tool = ToolInstance::getInstance();
			ros_com = RosCommunicationInstance::getInstance();
			// walk_con = WalkContinuouseInstance::getInstance();
			pso_fun = PSO_geometryInstance::getInstance();
			sub = nh.subscribe("/Obstaclefreearea_Topic", 100, &KidsizeStrategy::Obstaclefreearea, this);
			pub_stepspace = nh.advertise<strategy::step_space>("/stepspace", 1);
			stepcheck_pub = nh.advertise<std_msgs::Bool>("/stepcheck", 1);
			Reachable_region_sub = nh.subscribe("/ReachableRegion_Topic", 1, &KidsizeStrategy::Reachable_Region, this);
			FPGAack_subscriber = nh.subscribe("/package/footstepack", 1, &KidsizeStrategy::Footstepack_callback, this);
			Depthimage_subscriber = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &KidsizeStrategy::DepthCallback,this);
    		GetIMUData_Subscriber = nh.subscribe("/imu/rpy/filtered", 10, &KidsizeStrategy::GetIMUData,this);
			depthimage_Publisher = it.advertise("depth_image", 1, this);
			RealsenseIMUData = {0.0,0.0,0.0};
		};
		~KidsizeStrategy()
		{
			StrategyInfoInstance::deleteInstance();
			ToolInstance::deleteInstance();
			RosCommunicationInstance::deleteInstance();
			WalkContinuouseInstance::deleteInstance();
			// PSO_geometryInstance::deleteInstance();
		};

		StrategyInfoInstance *strategy_info;
		ToolInstance *tool;
		RosCommunicationInstance *ros_com;
		// WalkContinuouseInstance *walk_con;
		
		// typedef double (*pso_obj_fun_t)(double *pos, int dim, void *params);
		PSO_geometryInstance *pso_fun;
		image_transport::Publisher depthimage_Publisher;
		ros::Publisher pub_stepspace;
		ros::Publisher stepcheck_pub;
		ros::Subscriber sub;
		ros::Subscriber Reachable_region_sub;
        ros::Subscriber FPGAack_subscriber;
        ros::Subscriber GetIMUData_Subscriber;
        ros::Subscriber Depthimage_subscriber;
		sensor_msgs::ImagePtr msg_depth;
		void strategymain(ros::NodeHandle nh);
		void Obstaclefreearea(const strategy::obstacle &msg);
		void Reachable_Region(const strategy::ReachableRegion &msg);
		void Footstepack_callback(const std_msgs::Bool& msg);
		void GetIMUData(const geometry_msgs::Vector3Stamped &msg);
        void DepthCallback(const sensor_msgs::ImageConstPtr& depth_img);
		// double pso_sphere(double *pos, int dim, void *params);
		strategy::step_space freecoordinate;
		struct timeval tstart, tend;
		double timeuse;
		int now_step = 999;
		int pre_step = 999;
		bool get_obs = false;
		bool on_floor = false;
		bool init = true;
		bool walk_in = false;
		std_msgs::Bool odd_step;
		float obs_coordinate[2] = {0};
		float free_limit[4] = {0};  /*free_limit = [xmin, xmax, ymin, ymax]*/
		float freelimit[4] = {0};  /*free_limit = [xmin, xmax, ymin, ymax]*/
		float *free_coordinate[2] = {0}; /**free_coordinate = [x, y]*/
		int cnt;
		/*foot width = 60 , foot hight = 80*/
		int foot_width = 60;
		int foot_height = 80;
		int foot_widthmin = 50;
		int foot_heightmin = 70;

		int obs_side = 0;
        bool Footstepack;
		int walk_x, walk_y, walk_z;
		float depth_distance = 0;
		enum
		{/*obs_side*/
			right_side,
			left_side,
			top_side,
			bottom_side,
			centor_side
		};

};
#endif //PSO_STRATEGY
