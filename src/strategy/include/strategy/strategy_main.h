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
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include "strategy/obstacle.h"
#include "strategy/particle.h"
#include "strategy/step_space.h"
#include "strategy/pso.h"

using namespace std;

// class KidsizeStrategy;
// typedef double (*pso_obj_fun_t)(double *pos, int dim, void *params);
class KidsizeStrategy
{

	public:
		KidsizeStrategy(ros::NodeHandle &nh) 
		{
			pso_fun = PSO_geometryInstance::getInstance();
			sub = nh.subscribe("/Obstaclefreearea_Topic", 100, &KidsizeStrategy::Obstaclefreearea, this);
			pub_stepspace = nh.advertise< strategy::step_space >( "/stepspace", 1000 );
		
		};
		~KidsizeStrategy()
		{
			// PSO_geometryInstance::deleteInstance();
		};
		// typedef double (*pso_obj_fun_t)(double *pos, int dim, void *params);
		PSO_geometryInstance *pso_fun;
		ros::Publisher pub_stepspace;
		ros::Subscriber sub;
		void strategymain(ros::NodeHandle nh);
		void Obstaclefreearea(const strategy::obstacle &msg);
		// double pso_sphere(double *pos, int dim, void *params);
		strategy::step_space freecoordinate;
		struct timeval tstart, tend;
		double timeuse;
		bool get_obs = false;
		bool on_floor = false;
		bool init = true;
		float obs_coordinate[2] = {0};
		float free_limit[4] = {0};  /*free_limit = [xmin, xmax, ymin, ymax]*/
		float freelimit[4] = {80, 160, 90, 230};
		float *free_coordinate[2] = {0}; /**free_coordinate = [x, y]*/

			/*foot width = 60 , foot hight = 80*/
			int foot_width = 60;
			int foot_height = 80;
			int foot_widthmin = 50;
			int foot_heightmin = 70;

		int obs_side = 0;
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
