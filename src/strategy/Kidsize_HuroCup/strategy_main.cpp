// #include "strategy/strategy_main.h"

// int main(int argc, char** argv)
// {
// 	ros::init(argc, argv, "BBthrow");
// 	ros::NodeHandle nh;
// 	KidsizeStrategy KidsizeStrategy;
    
// 	ros::Rate loop_rate(30);

//     // Load->initparameterpath();
// 	while (nh.ok()) 
// 	{
// 		ros::spinOnce();
// 		KidsizeStrategy.strategymain();
// 		loop_rate.sleep();
// 	}
// 	return 0;
// }
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h> // for printf
#include "strategy/pso.h"

#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include "strategy/obstacle.h"
#include "strategy/particle.h"
#include "strategy/step_space.h"
#include<iostream>
using namespace std;
bool get_obs = false;
bool on_floor = false;
bool init = true;
float obs_coordinate[2] = {0};
float free_limit[4] = {0};  /*free_limit = [xmin, xmax, ymin, ymax]*/
float *free_coordinate[2] = {0}; /**free_coordinate = [x, y]*/

    /*foot width = 60 , foot hight = 80*/
    int foot_width = 60;
    int foot_height = 80;
    int foot_widthmin = 50;
    int foot_heightmin = 70;

int obs_side = 0;
enum{
		/*obs_side*/
        right_side,
		left_side,
        top_side,
        bottom_side,
        centor_side
		
	};
//==============================================================
//                  BENCHMARK FUNCTIONS
//==============================================================

double pso_sphere(double *pos, int dim, void *params) {

    // double sum = 0;
    // int i;
    // for (i=0; i<dim; i++)
    //     sum += pow(pos[i]-obs_coordinate[i], 2);  // pow = pos[i]^2
    // return sum;
    float w1 = 0.1;
    float w2 = 0.01;
    float w3 = 0.001;
    float w4 = 0.0001;
    int theta = 0;
    float Tdsp = 0, Tssp = 0;
    double sum_objective_function = 0;
    double objective_function[4] = {0};
    objective_function[0] = w1*abs(0-pos[0]);  /*X direction*/
    objective_function[1] = w2*abs(0-pos[1]);  /*Y direction*/
    objective_function[2] = w3*abs(theta-theta);               /*Rotation*/
    objective_function[3] = w4*(Tdsp+Tssp);                    /*step period*/
    ROS_INFO("pos[0] = %f, pos[1] = %f", pos[0], pos[1]);
    // ROS_INFO("size = %d", sizeof(objective_function));
    
    
    for (int i=0; i<4; i++)
    {
        sum_objective_function += objective_function[i];
        // ROS_INFO("sum%d = %f", i,sum_objective_function);
    }

    ROS_INFO("sum = %f", sum_objective_function);
    return sum_objective_function;
}



double pso_rosenbrock(double *vec, int dim, void *params) {

    double sum = 0;
    int i;
    for (i=0; i<dim-1; i++)
        sum += 100 * pow((vec[i+1] - pow(vec[i], 2)), 2) +	\
            pow((1 - vec[i]), 2);

    return sum;

}


double pso_griewank(double *vec, int dim, void *params) {

    double sum = 0.;
    double prod = 1.;
    int i;
    for (i=0; i<dim;i++) {
        sum += pow(vec[i], 2);
        prod *= cos(vec[i] / sqrt(i+1));
    }

    return sum / 4000 - prod + 1;

}

void Obstaclefreearea(const strategy::obstacle &msg)
{
    get_obs = true;
    float *free_coordinate = (float *)calloc(2, sizeof(float));
    free_limit[0] = -100;
    free_limit[1] = 100;
    free_limit[2] = -100;
    free_limit[3] = 100;
    
    for(int i = 0; i<3; i++)
    {
        float obstacle_x = msg.x[i];
        float obstacle_y = msg.y[i];
        float obstacle_width = msg.width[i];
        float obstacle_height = msg.height[i];
        float obstacle_xmin = obstacle_x - obstacle_width/2;
        float obstacle_ymin = obstacle_y - obstacle_height/2;
        float obstacle_xmax = obstacle_x + obstacle_width/2;
        float obstacle_ymax = obstacle_y + obstacle_height/2;
        ROS_INFO("NO.%d", i+1);
        
        if(obstacle_xmin<0 && obstacle_xmax>0 && obstacle_ymin<0 && obstacle_ymax>0)
        {
            obs_side = centor_side;
            ROS_INFO("centor_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) >= abs(obstacle_y-*(free_coordinate+1)) && obstacle_x-*(free_coordinate) > *(free_coordinate))
        {
            obs_side = right_side;
            ROS_INFO("right_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) < abs(obstacle_y-*(free_coordinate+1)) && obstacle_y-*(free_coordinate+1) > *(free_coordinate))
        {
            obs_side = bottom_side;
            ROS_INFO("bottom_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) >= abs(obstacle_y-*(free_coordinate+1)) && obstacle_x-*(free_coordinate) < *(free_coordinate))
        {
            obs_side = left_side;
            ROS_INFO("left_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) < abs(obstacle_y-*(free_coordinate+1)) && obstacle_y-*(free_coordinate+1) < *(free_coordinate))
        {
            obs_side = top_side;
            ROS_INFO("top_side");
        }
        else
        {
            obs_side = centor_side;
            ROS_INFO("centor_side");
        }
        ROS_INFO("obs_x,y,w,h = %f, %f, %f, %f",obstacle_x, obstacle_y, obstacle_width, obstacle_height);
        
        switch (obs_side)
        {
            case centor_side:
                free_limit[0] = obstacle_xmin;
                free_limit[1] = obstacle_xmax;
                free_limit[2] = obstacle_ymin;
                free_limit[3] = obstacle_ymax;
                *(free_coordinate) = obstacle_x;
                *(free_coordinate+1) = obstacle_y;
                break;
            case right_side:
                free_limit[1] = obstacle_xmin;
                *(free_coordinate) = (free_limit[0]+free_limit[1])/2;
                break;
            case left_side:
                free_limit[0] = obstacle_xmax;
                *(free_coordinate) = (free_limit[0]+free_limit[1])/2;
                break;
            case bottom_side:
                free_limit[3] = obstacle_ymin;
                *(free_coordinate+1) = (free_limit[2]+free_limit[3])/2;
                break;
            case top_side:
                free_limit[2] = obstacle_ymax;
                *(free_coordinate+1) = (free_limit[2]+free_limit[3])/2;
                break;
            default:
                break;
        }
        ROS_INFO("free_min,max= %f, %f, %f, %f", free_limit[0], free_limit[1], free_limit[2], free_limit[3]);
        ROS_INFO("free_x,y = %f, %f", *(free_coordinate), *(free_coordinate+1));

        // if(free_limit[0]-obstacle_xmin>foot_width)
        // {
        //     free_limit[0] = abs(free_limit[0]-obstacle_xmin);
        //     on_floor = true;
        // }
        // else if(free_limit[1]-obstacle_xmax>foot_width)
        // {
        //     free_limit[1] = abs(free_limit[1]-obstacle_xmax);
        //     on_floor = true;
        // }
        // else if(free_limit[2]-obstacle_ymin>foot_height)
        // {
        //     free_limit[2] = abs(free_limit[2]-obstacle_ymin);
        //     on_floor = true;
        // }
        // else if(free_limit[3]-obstacle_ymax>foot_height)
        // {
        //     free_limit[3] = abs(free_limit[3]-obstacle_ymax);
        //     on_floor = true;
        // }
        // else if(obstacle_width>foot_widthmin && obstacle_height>foot_heightmin)
        // {
        //     free_limit[0] = obstacle_xmin;
        //     free_limit[1] = obstacle_xmax;
        //     free_limit[2] = obstacle_ymin;
        //     free_limit[3] = obstacle_ymax;
        //     on_floor = false;
        // }
        // else  /*no place can walk*/
        // {
        //     free_limit[0] = 0;
        //     free_limit[1] = 0;
        //     free_limit[2] = 0;
        //     free_limit[3] = 0;
        // }
        
        sleep(1);
    }

    // memmove(*obs_coordinate, *(free_coordinate), sizeof(float) * 2);
    free(free_coordinate);
    
    
    obs_coordinate[0] = *(free_coordinate);
    obs_coordinate[1] = *(free_coordinate+1);

}


//==============================================================

int main(int argc, char **argv) {
    ros::init(argc, argv, "BBthrow");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/Obstaclefreearea_Topic", 100, Obstaclefreearea);
    ros::Publisher pub_stepspace = nh.advertise< strategy::step_space >( "/stepspace", 1000 );
    strategy::step_space freecoordinate;
    ros::spinOnce();
    ros::Rate loop_rate(30);

    while (nh.ok())
    {
        if(get_obs)
        {
            if(init)
            {
                for(int i = 0; i<4; i++)
                    freecoordinate.step_space.push_back(free_limit[i]);
                pub_stepspace.publish( freecoordinate );
                freecoordinate.step_space.clear();
                init = false;
            }


            pso_settings_t *settings = NULL;
            pso_obj_fun_t obj_fun = NULL;

            // handle the default case (no argument given)
            if (obj_fun == NULL || settings == NULL) {
                obj_fun = pso_sphere;
                settings = pso_settings_new(2, &free_limit[0], &obs_coordinate[0]);
                // settings = pso_settings_new(2, -100, 100);
                printf("Optimizing function: sphere (dim=%d, swarm size=%d)\n", settings->dim, settings->size);
            }

            // set some general PSO settings
            settings->goal = 1e-5;
            // settings->size = 30;
            settings->nhood_strategy = PSO_NHOOD_RING;
            settings->nhood_size = 10;
            settings->w_strategy = PSO_W_LIN_DEC;

            // initialize GBEST solution
            pso_result_t solution;
            // allocate memory for the best position buffer
            solution.gbest = (double *)malloc(settings->dim * sizeof(double));

            printf("dim=%d, swarm size=%d)\n", settings->dim, settings->size);
            sleep(2);

            // run optimization algorithm
            pso_solve(obj_fun, NULL, &solution, settings, nh);

            // free the gbest buffer
            free(solution.gbest);

            // free the settings
            pso_settings_free(settings);
            get_obs = false;
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    
    }
    return 0;

}