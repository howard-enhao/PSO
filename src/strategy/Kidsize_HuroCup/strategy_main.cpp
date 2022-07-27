#include "strategy/strategy_main.h"


//==============================================================
//                  BENCHMARK FUNCTIONS
//==============================================================

void KidsizeStrategy::GetIMUData(const geometry_msgs::Vector3Stamped &msg)
{
    try
    {
        RealsenseIMUData[0] = msg.vector.x;
        RealsenseIMUData[1] = msg.vector.y;
        RealsenseIMUData[2] = msg.vector.z;
        // ROS_INFO("r = %f, p = %f, y = %f",RealsenseIMUData[0],RealsenseIMUData[1],RealsenseIMUData[2]);
              
    }catch(...)
    {
      ROS_INFO("No IMU Data");
      return;
    }
}

void KidsizeStrategy::DepthCallback(const sensor_msgs::ImageConstPtr& depth_img) 
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
        depth_buffer = cv_depth_ptr->image;
        cv::Size dst_sz(depth_buffer.cols,depth_buffer.rows);
        cv::Point2f center(dst_sz.height/2,dst_sz.width/2);
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, 1 * (RealsenseIMUData[1]), 1.0);
        cv::warpAffine(depth_buffer, depth_buffer, rot_mat, dst_sz);
        msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", (cv_depth_ptr->image/3)*255).toImageMsg();
        
        resize(depth_buffer, depth_buffer, cv::Size(640, 480));
        //   imshow("depth_buffer",depth_buffer);
        depthimage_Publisher.publish(msg_depth);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("DepthCallback cv_bridge exception: %s", e.what());
      return;
    }
    // cout << "image data(240, 320): " << (depth_buffer.at<uint16_t>(240, 320))*0.1 <<" cm" << endl;//獲取圖像坐標240,320的深度值,單位是公分
}

double pso_sphere(float *pos, int dim, void *params, bool posInObs) {

    // double sum = 0;
    // int i;
    // for (i=0; i<dim; i++)
    //     sum += pow(pos[i]-obs_coordinate[i], 2);  // pow = pos[i]^2
    // return sum;
    float w1 = 0.01;
    float w2 = 0.1;
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
    // get_obs = true;
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
    get_obs = true;
}

void KidsizeStrategy::Footstepack_callback(const std_msgs::Bool& msg)
{
    Footstepack = msg.data;
    // usleep(2500*1000);
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
        // if(!walk_in)
        // {
        //     ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
        //     tool->Delay(2);
        //     walk_in = true;
        //     // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
        //     // printf("0\n");
        //     // sleep(1);
        // }
        // ros_com->sendContinuousValue(8000, 0, 0, 0,SensorMode::None);
        // sleep(4);
        // ros_com->sendContinuousValue(4000, 0, 0, 0,SensorMode::None);
        // sleep(4);
        // ros_com->sendContinuousValue(3000, 0, 0, 0,SensorMode::None);
        // sleep(4);
        // ros_com->sendContinuousValue(8000, 0, 1800, 0,SensorMode::None);
        // sleep(4);
        // ros_com->sendContinuousValue(3000, 2000, 0, 0,SensorMode::None);
        // sleep(4);
        // ros_com->sendContinuousValue(8000, 0, 0, 0,SensorMode::None);
        // sleep(4);
        // ros_com->sendContinuousValue(3000, 0, 0, 0,SensorMode::None);
        // sleep(4);
        
        

        if(init)
        {
            for(int i = 0; i<4; i++)
                freecoordinate.step_space.push_back(freelimit[i]);
            pub_stepspace.publish(freecoordinate);
            freecoordinate.step_space.clear();
            init = false;
            odd_step.data = true;
            stepcheck_pub.publish(odd_step);
            walk_x = 0;
            walk_y = 0;
            walk_z = 0;
            pre_walk_x = 0;
            pre_walk_y = 0;
            pre_walk_z = 0;
            board_height = 2;
            down_flag = false;
            // ros_com->sendSensorSet(-0.5, -0.1, 0, 0x80);
            if(!walk_in)
            {
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
                tool->Delay(2);
                walk_in = true;
                // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
                // printf("0\n");
                // sleep(1);
            }
        }
        else if(now_step != pre_step && Footstepack && get_obs)
        {

            Footstepack = false;
            
            

            pso_settings_t *settings = NULL;
            pso_obj_fun_t obj_fun = NULL;
            // handle the default case (no argument given)
            if (obj_fun == NULL || settings == NULL) {
                obj_fun = pso_sphere;
                settings = pso_settings_new(2, &freelimit[0], &obs_coordinate[0]);
                // settings = pso_settings_new(2, -100, 100);
                printf("Optimizing function: sphere (dim=%d, swarm size=%d)\n", settings->dim, settings->size);
            }

            // set some general PSO settings
            settings->goal = 0.02;//1e-2;
            // settings->size = 30;
            settings->nhood_strategy = PSO_NHOOD_RING;
            settings->nhood_size = 10;
            settings->w_strategy = PSO_W_LIN_DEC;

            // initialize GBEST solution
            pso_result_t solution;
            // allocate memory for the best position buffer
                solution.gbest = (float *)malloc(settings->dim * sizeof(float));

            // printf("(dim=%d, swarm size=%d)\n", settings->dim, settings->size);
            // sleep(2);
            // run optimization algorithm
            pso_fun->pso_solve(obj_fun, NULL, &solution, settings, nh);
            Distance distance_c;
            distance_c = measure((int)freecenter[0]*2, (int)freecenter[1]*2, CameraType::stereo);
            Distance distance;
            distance = measure((int)solution.gbest[0]*2, (int)solution.gbest[1]*2, CameraType::stereo);
            depth_distance = AvgPixelDistance((int)solution.gbest[0]*2, (int)solution.gbest[1]*2);
            printf("===================================================================\n");
            printf("now_step = %d\n", now_step);
            printf("\ncenter = %d, %d\n", freecenter[0], freecenter[1]);
            printf("\n gbest = %d, %d\n", (int)solution.gbest[0], (int)solution.gbest[1]);
            printf("c_x = %d, y = %d, dis = %d\n", distance_c.y_dis, distance_c.x_dis, distance_c.dis);
            printf("  x = %d, y = %d, dis = %d\n", distance.y_dis, distance.x_dis, distance.dis);
            printf("\ndepth_distance = %f\n", depth_distance);
            
            walk_x = (freecenter[0]-(int)solution.gbest[0])/7;
            walk_y = (freecenter[1]-(int)solution.gbest[1])/10+4;

            //xxxxx
            // if(now_step % 2 == 0)
            // {
            //     // walk_x = 1-distance.x_dis;
            //     if(walk_x<distance_c.x_dis-distance.x_dis+3)
            //         walk_x = distance_c.x_dis-distance.x_dis;
            //     else
            //         walk_x = walk_x;
            //     if(walk_z == board_height)
            //         walk_x = walk_x+1;
            // }
            // else
            // {
            //     // walk_x = 10-distance.x_dis;
            //     if(walk_x>distance_c.x_dis-distance.x_dis-3)
            //         walk_x = distance_c.x_dis-distance.x_dis;
            //     else
            //         walk_x = walk_x;
            //     if(walk_z == board_height)
            //         walk_x = walk_x-1;
            // }
            
            //yyyyy
            // if((distance_c.y_dis-distance.y_dis)>2)
            //     walk_y = (float)(distance_c.y_dis-distance.y_dis)*2+4;
            // else
            //     walk_y = (float)(distance_c.y_dis-distance.y_dis)*1.5+4;
            
            if(walk_z == board_height || pre_walk_z == board_height)
            {
                
                // walk_y = walk_y+3;
                
                if(pre_walk_z == board_height)
                {
                    down_flag = true;
                    if(now_step % 2 == 0)
                        walk_x = 1;
                    else
                        walk_x = -1;
                    walk_y = 7;

                    // ros_com->sendSensorSet(0, 0.1, 0, 0x80);
                }
                else
                {
                    if(now_step % 2 == 0)
                        walk_x = 1;
                    else
                        walk_x = -1;
                    walk_y = 6;
                }
                
                pre_walk_z = walk_z;
            }
            
            //zzzzz
            if((depth_distance>37 && depth_distance<43.1 && depth_distance != 0.000000 && now_step>2 && !down_flag)||now_step == 4)
            {
                walk_z = board_height;
                walk_y = walk_y+5;
            }
            else
                walk_z = 0;
            
            
            if(now_step % 2 == 0)
            {
                if(walk_x<0)
                    walk_x = 0;
                else if(walk_x>3)
                    walk_x = 3;
            }
            else
            {
                if(walk_x>0)
                    walk_x = 0;
                else if(walk_x<-3)
                    walk_x = -3;
            }
            // if(walk_x<-3)
            //         else
            //             walk_x = -1;
            //     walk_x = -3;
            // else if(walk_x>3)
            //     walk_x = 3;

            if(walk_y<0)
                walk_y = 0;
            else if(walk_y>7)
                walk_y = 7;


            pre_walk_x = walk_x;
            pre_walk_y = walk_y;
            // if(!walk_in)
            // {
            //     ros_com->sendBodyAuto(0,0,0,0, WalkingMode::ContinuousStep,SensorMode::None);
            //     tool->Delay(2);
            //     walk_in = true;
            //     // ros_com->sendContinuousValue(0, 0, 0, 0,SensorMode::None);
            //     printf("0\n");
            //     // sleep(1);
            // }

            // if(down_flag)
            // {
            //     printf("111111down_flag = true\n",down_flag);
            //     if(now_step % 2 == 0)
            //     {
            //         ros_com->sendBodySector(183);
            //         tool->Delay(500);
            //         // ros_com->sendBodySector(121);
            //         // tool->Delay(1500);
            //         // ros_com->sendBodySector(122);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(122);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(122);
            //         // tool->Delay(500);
            //         ros_com->sendBodySector(171);
            //         tool->Delay(500);
            //         // ros_com->sendBodySector(181);
            //         // tool->Delay(1000);
            //     }
            //     else
            //     {
            //         // ros_com->sendBodySector(181);
            //         // tool->Delay(1000);
            //         // ros_com->sendBodySector(182);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(182);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(182);
            //         // tool->Delay(500);
            //     }
            //     // down_flag = false;
            //     tool->Delay(2000);

            //     // ros_com->sendSensorSet(-0.4, 0.1, 0, 0x80);
            // }
            

            printf("walkx = %.1f, walky = %.1f, walkz = %.1f\n", walk_y, walk_x, walk_z);
            printf("pre_walk_z = %.1f\n",pre_walk_z);
            sleep(2);
            ros_com->sendContinuousValue(walk_y*1000, walk_x*1000, walk_z*1000, 0,SensorMode::None);
            // ros_com->sendContinuousValue(walk_y*1000, walk_x*1000, walk_z, 0,SensorMode::None);
            // ros_com->sendContinuousValue(4000, 0, 0, 0,SensorMode::None);
            // sleep(3);
            tool->Delay(300);

            // if(down_flag)
            // {
            //     printf("down_flag = true\n",down_flag);
            //     if(now_step % 2 == 0)
            //     {
            //         // ros_com->sendBodySector(183);
            //         // tool->Delay(1000);
            //         ros_com->sendBodySector(121);
            //         tool->Delay(1500);
            //         ros_com->sendBodySector(122);
            //         tool->Delay(500);
            //         ros_com->sendBodySector(122);
            //         tool->Delay(500);
            //         // ros_com->sendBodySector(122);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(122);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(122);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(181);
            //         // tool->Delay(1000);
            //         // ros_com->sendBodySector(172);
            //         // tool->Delay(500);
            //     }
            //     else
            //     {
            //         // ros_com->sendBodySector(181);
            //         // tool->Delay(1000);
            //         // ros_com->sendBodySector(182);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(182);
            //         // tool->Delay(500);
            //         // ros_com->sendBodySector(182);
            //         // tool->Delay(500);
            //     }
            //     down_flag = false;
            //     tool->Delay(2000);

            //     // ros_com->sendSensorSet(-0.4, 0.1, 0, 0x80);
            // }
            // else
                tool->Delay(2000);

            // free the gbest buffer
            free(solution.gbest);

            // free the settings
            pso_fun->pso_settings_free(settings);
            // get_obs = false;

            // if(odd_step.data)
            //     odd_step.data = false;
            // else
            //     odd_step.data = true;
            // stepcheck_pub.publish(odd_step);
            // stepcheck_pub.publish(0);
            // break;
            // ros::shutdown();
            // get_image = false;
            pre_step = now_step;
            printf("===================================================================\n");
        }
        get_obs = false;
    }
    else
    {
        if(walk_in)
        {
            printf("end\n");
            ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
            tool->Delay(2000);
            walk_in = false;
            ros_com->sendBodySector(29);
            tool->Delay(1000);
            // ros_com->sendSensorSet(-0.4, 0.1, 0, 0x80);
            odd_step.data = false;
            stepcheck_pub.publish(odd_step);
        }
        init = true;
        cnt = 0;
        now_step = 0;
        Footstepack = false;
    }
}