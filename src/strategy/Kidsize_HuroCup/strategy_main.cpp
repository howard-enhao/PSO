#include "strategy/strategy_main.h"


//==============================================================
//                  BENCHMARK FUNCTIONS
//==============================================================

double pso_sphere(float *pos, int dim, void *params, bool posInObs) {

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
    double objective_function[5] = {0};
    objective_function[0] = w1*abs(freecenter[0]-pos[0]);  /*X direction*/
    objective_function[1] = w2*abs(freecenter[1]-pos[1]);  /*Y direction*/
    objective_function[2] = w3*abs(theta-theta);               /*Rotation*/
    objective_function[3] = w4*(Tdsp+Tssp);                    /*step period*/
    if(!posInObs)
        objective_function[4] = 100;
    // ROS_INFO("size = %d", sizeof(objective_function));
    
    for (int i=0; i<5; i++)
    {
        sum_objective_function += objective_function[i];
        // ROS_INFO("sum%d = %f", i,sum_objective_function);
    }

    // ROS_INFO("sum = %f", sum_objective_function);
    return sum_objective_function;
}

void KidsizeStrategy::Obstaclefreearea(const strategy::obstacle &msg)
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
        // ROS_INFO("NO.%d", i+1);
        
        if(obstacle_xmin<0 && obstacle_xmax>0 && obstacle_ymin<0 && obstacle_ymax>0)
        {
            obs_side = centor_side;
            // ROS_INFO("centor_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) >= abs(obstacle_y-*(free_coordinate+1)) && obstacle_x-*(free_coordinate) > *(free_coordinate))
        {
            obs_side = right_side;
            // ROS_INFO("right_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) < abs(obstacle_y-*(free_coordinate+1)) && obstacle_y-*(free_coordinate+1) > *(free_coordinate))
        {
            obs_side = bottom_side;
            // ROS_INFO("bottom_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) >= abs(obstacle_y-*(free_coordinate+1)) && obstacle_x-*(free_coordinate) < *(free_coordinate))
        {
            obs_side = left_side;
            // ROS_INFO("left_side");
        }
        else if(abs(obstacle_x-*(free_coordinate)) < abs(obstacle_y-*(free_coordinate+1)) && obstacle_y-*(free_coordinate+1) < *(free_coordinate))
        {
            obs_side = top_side;
            // ROS_INFO("top_side");
        }
        else
        {
            obs_side = centor_side;
            // ROS_INFO("centor_side");
        }
        // ROS_INFO("obs_x,y,w,h = %f, %f, %f, %f",obstacle_x, obstacle_y, obstacle_width, obstacle_height);
        
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
        // ROS_INFO("free_min,max= %f, %f, %f, %f", free_limit[0], free_limit[1], free_limit[2], free_limit[3]);
        // ROS_INFO("free_x,y = %f, %f", *(free_coordinate), *(free_coordinate+1));

        // sleep(1);
    }

    // memmove(*obs_coordinate, *(free_coordinate), sizeof(float) * 2);
    free(free_coordinate);
    
    obs_coordinate[0] = *(free_coordinate);
    obs_coordinate[1] = *(free_coordinate+1);
    // sleep(1);
}

void KidsizeStrategy::Reachable_Region(const strategy::ReachableRegion &msg)
{
    now_step = msg.now_step;
    freelimit[0] = msg.x;
    freelimit[1] = msg.Width+msg.x;
    freelimit[2] = msg.y;
    freelimit[3] = msg.Height+msg.y;
    freecenter[0] = msg.c_x;  //(freelimit[0]+freelimit[1])/2;
    freecenter[1] = msg.c_y;  //(freelimit[2]+freelimit[3])/2;
}

void KidsizeStrategy::Footstepack_callback(const std_msgs::Bool& msg)
{
    Footstepack = msg.data;
}

//==============================================================

int main(int argc, char **argv) {
    ros::init(argc, argv, "BBthrow");
	ros::NodeHandle nh;
	KidsizeStrategy KidsizeStrategy(nh);

	
    ros::spinOnce();
    ros::Rate loop_rate(30);
    while (nh.ok())
    {
        KidsizeStrategy.strategymain(nh);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}

void KidsizeStrategy::strategymain(ros::NodeHandle nh)
{
    if(strategy_info->getStrategyStart())
	{
        if(!walk_in)
        {
            ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
            tool->Delay(2);
            walk_in = true;
        }
        
        // if(Footstepack)
        // {
        //     cnt++;
        //     printf("Footstepack = %d\n", Footstepack);
        //     printf("cnt = %d\n", cnt);
        //     if(cnt == 4 || cnt == 9)
        //     {
        //         printf("in\n");
        //         ros_com->sendContinuousValue(4000, 1000, 2, 0,SensorMode::None);
        //     }
        //     else
                ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            sleep(8);
        //     Footstepack = false;
        // }
            // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            // sleep(2);
            // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            // sleep(2);
            // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            // sleep(2);
            // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            // sleep(2);
            // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            // sleep(8);

        
        // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
        // sleep(2);
        // ros_com->sendContinuousValue(0, 0, 2, 0,SensorMode::None);
        // sleep(2);
        // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
        // sleep(2);
            
        // if(now_step != pre_step)
        // {
        //     if(init)
        //     {
        //         for(int i = 0; i<4; i++)
        //             freecoordinate.step_space.push_back(freelimit[i]);
        //         pub_stepspace.publish(freecoordinate);
        //         freecoordinate.step_space.clear();
        //         init = false;
        //         odd_step.data = true;
        //     }
            

        //     pso_settings_t *settings = NULL;
        //     pso_obj_fun_t obj_fun = NULL;
        //     // handle the default case (no argument given)
        //     if (obj_fun == NULL || settings == NULL) {
        //         obj_fun = pso_sphere;
        //         settings = pso_settings_new(2, &freelimit[0], &obs_coordinate[0]);
        //         // settings = pso_settings_new(2, -100, 100);
        //         printf("Optimizing function: sphere (dim=%d, swarm size=%d)\n", settings->dim, settings->size);
        //     }

        //     // set some general PSO settings
        //     settings->goal = 0.03;//1e-2;
        //     // settings->size = 30;
        //     settings->nhood_strategy = PSO_NHOOD_RING;
        //     settings->nhood_size = 10;
        //     settings->w_strategy = PSO_W_LIN_DEC;

        //     // initialize GBEST solution
        //     pso_result_t solution;
        //     // allocate memory for the best position buffer
        //         solution.gbest = (float *)malloc(settings->dim * sizeof(float));

        //     // printf("(dim=%d, swarm size=%d)\n", settings->dim, settings->size);
        //     // sleep(2);
        //     // run optimization algorithm
        //     pso_fun->pso_solve(obj_fun, NULL, &solution, settings, nh);

        //     // free the gbest buffer
        //     free(solution.gbest);

        //     // free the settings
        //     pso_fun->pso_settings_free(settings);
        //     // get_obs = false;
        //     if(odd_step.data)
        //         odd_step.data = false;
        //     else
        //         odd_step.data = true;
        //     stepcheck_pub.publish(odd_step);
        //     // break;
        //     // ros::shutdown();
        //     // get_image = false;
        //     pre_step = now_step;
        // }
    }
    else
    {
        if(walk_in)
        {
            ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
            tool->Delay(10);
            walk_in = false;
        }
        cnt = 0;
    }
}