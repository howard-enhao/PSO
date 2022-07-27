#include "strategy/pso.h"

PSO_geometryInstance* PSO_geometryInstance::m_pInstance;

PSO_geometryInstance* PSO_geometryInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new PSO_geometryInstance();
    return m_pInstance;
}

void PSO_geometryInstance::deleteInstance()
{
    if(m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = NULL;
    }
}

PSO::PSO()
{
    initialize();
}
PSO::~PSO()
{

}

void PSO::initialize()
{
    Reachable_region_sub = nh.subscribe("/ReachableRegion_Topic", 1, &PSO::Reachable_Region, this);
    // Depthimage_subscriber = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &PSO::DepthCallback,this);
    // GetIMUData_Subscriber = nh.subscribe("/imu/rpy/filtered", 10, &PSO::GetIMUData,this);
    edgepoint_subscriber = nh.subscribe("/edgepoint_Topic", 10, &PSO::get_edgepoint, this);
    edgeimg_subscriber = nh.subscribe("edge_image", 10, &PSO::get_edgeimg, this);
        image_transport::ImageTransport it(nh);
        // image_transport::Publisher edgeimage_Publisher;
    edgeimage_Publisher = it.advertise("final_image", 1, this);
    // depthimage_Publisher = it.advertise("depth_image", 1, this);
    Computational_geometry = Computational_geometryInstance::getInstance();
    // FeatureDistance = FeatureDistanceInstance::getInstance();
    // RealsenseIMUData = {0.0,0.0,0.0};
}

void PSO::Reachable_Region(const strategy::ReachableRegion &msg)
{
    freelimit[0] = msg.x;
    freelimit[1] = msg.Width+msg.x;
    freelimit[2] = msg.y;
    freelimit[3] = msg.Height+msg.y;
    freecenter[0] = msg.c_x;  //(freelimit[0]+freelimit[1])/2;
    freecenter[1] = msg.c_y;  //(freelimit[2]+freelimit[3])/2;
}

// void PSO::GetIMUData(const geometry_msgs::Vector3Stamped &msg)
// {
//     try
//     {
//         RealsenseIMUData[0] = msg.vector.x;
//         RealsenseIMUData[1] = msg.vector.y;
//         RealsenseIMUData[2] = msg.vector.z;
//         // ROS_INFO("r = %f, p = %f, y = %f",RealsenseIMUData[0],RealsenseIMUData[1],RealsenseIMUData[2]);
              
//     }catch(...)
//     {
//       ROS_INFO("No IMU Data");
//       return;
//     }
// }

// void PSO::DepthCallback(const sensor_msgs::ImageConstPtr& depth_img) 
// {
//     cv_bridge::CvImagePtr cv_depth_ptr;
//     try
//     {
//         cv_depth_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
//         depth_buffer = cv_depth_ptr->image;
//         cv::Size dst_sz(depth_buffer.cols,depth_buffer.rows);
//         cv::Point2f center(dst_sz.height/2,dst_sz.width/2);
//         cv::Mat rot_mat = cv::getRotationMatrix2D(center, 1 * (RealsenseIMUData[1]), 1.0);
//         cv::warpAffine(depth_buffer, depth_buffer, rot_mat, dst_sz);
        
//         msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", (cv_depth_ptr->image/3)*255).toImageMsg();
        
//         resize(depth_buffer, depth_buffer, cv::Size(640, 480));
//         //   imshow("depth_buffer",depth_buffer);
//         depthimage_Publisher.publish(msg_depth);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("DepthCallback cv_bridge exception: %s", e.what());
//       return;
//     }
//     // cout << "image data(240, 320): " << (depth_buffer.at<uint16_t>(240, 320))*0.1 <<" cm" << endl;//獲取圖像坐標240,320的深度值,單位是公分
// }

// double PSO::pso_sphere(double *pos, int dim, void *params) {

//     // double sum = 0;
//     // int i;
//     // for (i=0; i<dim; i++)
//     //     sum += pow(pos[i]-obs_coordinate[i], 2);  // pow = pos[i]^2
//     // return sum;
//     float w1 = 0.1;
//     float w2 = 0.01;
//     float w3 = 0.001;
//     float w4 = 0.0001;
//     int theta = 0;
//     float Tdsp = 0, Tssp = 0;
//     // double PSO::sum_objective_function;
//     double objective_function[4] = {0};
//     objective_function[0] = w1*abs(160-pos[0]);  /*X direction*/
//     objective_function[1] = w2*abs(120-pos[1]);  /*Y direction*/
//     objective_function[2] = w3*abs(theta-theta);               /*Rotation*/
//     objective_function[3] = w4*(Tdsp+Tssp);                    /*step period*/
//     ROS_INFO("pos[0] = %f, pos[1] = %f", pos[0], pos[1]);
//     // ROS_INFO("size = %d", sizeof(objective_function));
    
    
//     for (int i=0; i<4; i++)
//     {
//         PSO::sum_objective_function += objective_function[i];
//         // ROS_INFO("sum%d = %f", i,sum_objective_function);
//     }

//     ROS_INFO("sum = %f", PSO::sum_objective_function);
//     return PSO::sum_objective_function;
// }

void PSO::get_edgepoint(const strategy::EdgePointList &msg)
{
    // cout<<"size="<<msg.points.size()<<endl;
    // printf("size = %lu\n", msg.points.size());
    // sleep(3);
    // for(int i = 0; i<msg.points.size(); i++)
    // {
    //     // cout<<"i = "<<i<<"\nmsg = \n"<<msg.points[i]<<endl;
    //     edge_point.push_back(Point3i(msg.points[i].x, msg.points[i].y, msg.points[i].z));
    // }
    edgepoint_list.clear();
    for(int i = 0; i<msg.Edgepointlist.size(); i++)
    {
        
        for(int j = 0; j<msg.Edgepointlist[i].points.size(); j++)
        {
            // cout<<"i = "<<i<<" j = "<<j<<"\nmsg = \n"<<msg.Edgepointlist[i].points[j]<<endl;
            edge_point.push_back(Point3i(msg.Edgepointlist[i].points[j].x, msg.Edgepointlist[i].points[j].y, msg.Edgepointlist[i].points[j].z));
            // cout<<"point"<<edge_point<<endl;
        }
        edgepoint_list.push_back(edge_point);
        edge_point.clear();
        // cout<<"edge = "<<edgepoint_list<<endl;
    }
    // cout<<"size = "<<edgepoint_list.size()<<endl;
    
}

void PSO::get_edgeimg(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    edge_img = cv_ptr->image;
}

void PSO::save_img(int gx, int gy, int radius)
{
    Mat final_img = imread(path_1);
    circle(final_img, Point(gx, gy), radius, Scalar(0,255,0));
    circle(final_img, Point(gx, gy), 2, Scalar(0, 255, 255), 2); //yellow
    circle(final_img, Point(freecenter[0], freecenter[1]), 2, Scalar(255, 0, 255), 2);  //pink
    char save_path[200] = "";
    string tmp = to_string(name_cnt);
    tmp = "/final_image"+tmp+".png";
    strcat(save_path, path_2);
    strcat(save_path, tmp.c_str());

    string point_str;
    point_str = "("+to_string(gx)+","+to_string(gy)+")";

    putText(final_img, point_str.c_str(), Point(30,30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0),4,8);
    imwrite(save_path, final_img);
    printf("a\n");
}

void PSO::show_image(const vector<Point3i>& c, int radius, bool* InRegion, int step, int gx, int gy)
{
    // Mat img = edge_img;
    Mat img = imread(path_1);
    // Mat img = imread("/home/ching/git/PSO/finalimage.png");
    Mat Contours=Mat::zeros(img.size(),CV_8UC3);
    Mat final_img;
    for(int i = 0; i < c.size(); ++i)
    {
        Contours.at<Vec3b>(Point(c[i].x, c[i].y))[0] = 255;
        Contours.at<Vec3b>(Point(c[i].x, c[i].y))[1] = 255;
        Contours.at<Vec3b>(Point(c[i].x, c[i].y))[2] = 0;
        if(InRegion[i])
            circle(Contours, Point(c[i].x, c[i].y), radius, Scalar(0,255,0));
    }
    circle(Contours, Point(gx, gy), 2, Scalar(0, 255, 255), 2); //yellow
    circle(Contours, Point(freecenter[0], freecenter[1]), 2, Scalar(255, 0, 255), 2);  //pink
    addWeighted(Contours, 1, img, 1, 0, final_img);
    edgeimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();
    edgeimage_Publisher.publish(edgeimage_msg);
    // save_img(final_img, gx, gy);
    // printf("b\n");
    // name_cnt++;
    // imshow("img", final_img);
    // char path[50] = "/home/ching/git/test_img/final_";
    // string temp_str = to_string(step);
    // char const* step_num= temp_str.c_str();
    // strcat(path, step_num);
    // strcat(path, ".png");
    // imwrite(path, final_img);
    // waitKey(500);
}
//==============================================================
// calulate swarm size based on dimensionality
int PSO::pso_calc_swarm_size(int dim) {
    int size = 10. + 2. * sqrt(dim);
    return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}

//==============================================================
//          INERTIA WEIGHT UPDATE STRATEGIES
//==============================================================
// calculate linearly decreasing inertia weight  //線性遞減慣性權重
double PSO::calc_inertia_lin_dec(int step, pso_settings_t *settings) {

    // int dec_stage = 3 * settings->steps / 4;
    // if (step <= dec_stage)
    //     return settings->w_min + (settings->w_max - settings->w_min) *	\
    //         (dec_stage - step) / dec_stage;
    // else
    //     return settings->w_min;
    // int dec_stage = settings->steps*4/5;
    int dec_stage = settings->steps;
    // if (step <= dec_stage)
        return settings->w_min + (settings->w_max - settings->w_min)*(dec_stage - step) / dec_stage;
    // else
    //     return settings->w_min;
}

//==============================================================
//          NEIGHBORHOOD (COMM) MATRIX STRATEGIES
//==============================================================
// global neighborhood
void PSO::inform_global(int *comm, float **pos_nb,
		   float **pos_b, float *fit_b,
		   float *gbest, int improved,
		   pso_settings_t *settings)
{

    int i;
    // all particles have the same attractor (gbest)
    // copy the contents of gbest to pos_nb
    for (i=0; i<settings->size; i++)
        memmove((void *)pos_nb[i], (void *)gbest,
                sizeof(float) * settings->dim);

}

// ===============================================================
// general inform function :: according to the connectivity
// matrix COMM, it copies the best position (from pos_b) of the
// informers of each particle to the pos_nb matrix
void PSO::inform(int *comm, float **pos_nb, float **pos_b, float *fit_b,
	    int improved, pso_settings_t * settings)
{
    int i, j;
    int b_n; // best neighbor in terms of fitness

    // for each particle
    for (j=0; j<settings->size; j++) {
        b_n = j; // self is best
        // who is the best informer??
        for (i=0; i<settings->size; i++)
            // the i^th particle informs the j^th particle
            if (comm[i*settings->size + j] && fit_b[i] < fit_b[b_n])
                // found a better informer for j^th particle
                b_n = i;
        // copy pos_b of b_n^th particle to pos_nb[j]
        memmove((void *)pos_nb[j],
                (void *)pos_b[b_n],
                sizeof(float) * settings->dim);
    }
}

// =============
// ring topology
// =============

// topology initialization :: this is a static (i.e. fixed) topology
void PSO::init_comm_ring(int *comm, pso_settings_t * settings) 
{
    int i;
    // reset array
    memset((void *)comm, 0, sizeof(int)*settings->size*settings->size);

    // choose informers
    for (i=0; i<settings->size; i++) {
        // set diagonal to 1
        comm[i*settings->size+i] = 1;
        if (i==0) {
            // look right
            comm[i*settings->size+i+1] = 1;
            // look left
            comm[(i+1)*settings->size-1] = 1;
        } else if (i==settings->size-1) {
            // look right
            comm[i*settings->size] = 1;
            // look left
            comm[i*settings->size+i-1] = 1;
        } else {
            // look right
            comm[i*settings->size+i+1] = 1;
            // look left
            comm[i*settings->size+i-1] = 1;
        }

    }

}

void PSO::inform_ring(int *comm, float **pos_nb,
		 float **pos_b, float *fit_b,
		 float *gbest, int improved,
		 pso_settings_t * settings)
{

    // update pos_nb matrix
    inform(comm, pos_nb, pos_b, fit_b, improved, settings);

}

// ============================
// random neighborhood topology
// ============================
void PSO::init_comm_random(int *comm, pso_settings_t * settings) 
{
    int i, j, k;
    // reset array
    memset((void *)comm, 0, sizeof(int)*settings->size*settings->size);

    // choose informers
    for (i=0; i<settings->size; i++) {
        // each particle informs itself
        comm[i*settings->size + i] = 1;
        // choose kappa (on average) informers for each particle
        for (k=0; k<settings->nhood_size; k++) {
            // generate a random index
            j = RNG_UNIFORM_INT(settings->size);
            // particle i informs particle j
            comm[i*settings->size + j] = 1;
        }
    }
}

void PSO::inform_random(int *comm, float **pos_nb,
		   float **pos_b, float *fit_b,
		   float *gbest, int improved,
		   pso_settings_t * settings)
{


    // regenerate connectivity??
    if (!improved)
        init_comm_random(comm, settings);
    inform(comm, pos_nb, pos_b, fit_b, improved, settings);

}

//==============================================================
// create pso settings
// pso_settings_t *pso_settings_new(int dim, double range_lo, double range_hi) {
pso_settings_t *pso_settings_new(int dim, float* range_limit, float* range_coordinate) {
    pso_settings_t *settings = (pso_settings_t *)malloc(sizeof(pso_settings_t));
    if (settings == NULL) { return NULL; }
    int foot_area[4] = {0};//{-30,30,-40,40};
    // set some default values
    settings->dim = dim;  //dimensionality(維度)
    settings->goal = 1e-5;

    // set up the range arrays
    settings->range_lo = (double *)malloc(settings->dim * sizeof(double));
    if (settings->range_lo == NULL) { free(settings); return NULL; }

    settings->range_hi = (double *)malloc(settings->dim * sizeof(double));
    if (settings->range_hi == NULL) { free(settings); free(settings->range_lo); return NULL; }

    for (int i=0; i<settings->dim; i++) {
        settings->range_lo[i] = range_limit[i*2] - foot_area[i*2];
        settings->range_hi[i] = range_limit[i*2+1] - foot_area[i*2+1];
        // ROS_INFO("range_lo = %f , range_hi = %f", settings->range_lo[i], settings->range_hi[i]);
    }
    // sleep(2);
    // settings->size = pso_calc_swarm_size(settings->dim);
    settings->size = 30;
    settings->print_every = 10;
    settings->steps = 50;//70;
    settings->c1 = 1.496;  //2
    settings->c2 = 1.496;  //2
    settings->w_max = 0.9;
    settings->w_min = 0.4;//0.5;
    // settings->w_max = PSO_INERTIA;
    // settings->w_min = 0.3;

    settings->clamp_pos = 1;
    settings->nhood_strategy = PSO_NHOOD_RING;
    settings->nhood_size = 5;
    // settings->w_strategy = PSO_W_LIN_DEC;

    return settings;
}

// destroy PSO settings
void PSO::pso_settings_free(pso_settings_t *settings) {
    free(settings->range_lo);
    free(settings->range_hi);
    free(settings);
}

float **pso_matrix_new(int size, int dim) {
    float **m = (float **)malloc(size * sizeof(float *));
    for (int i=0; i<size; i++) {
        m[i] = (float *)malloc(dim * sizeof(float));
    }
    return m;
}

void PSO::pso_matrix_free(float **m, int size) {
    for (int i=0; i<size; i++) {
        free(m[i]);
    }
    free(m);
}

void PSO::position_limit(float *pos, float *vel, pso_settings_t *settings) {
    // clamp position within bounds?
    // if (settings->clamp_pos) {
    //     if (pos < settings->range_lo-foot_area[d*2]) {
    //         pos[i][d] = settings->range_lo[d]-foot_area[d*2];
    //         vel[i][d] = 0;
    //     } else if (pos[i][d] > settings->range_hi[d]-foot_area[d*2+1]) {
    //         pos[i][d] = settings->range_hi[d]-foot_area[d*2+1];
    //         vel[i][d] = 0;
    //     }
    // } else {
    //     // enforce periodic boundary conditions
    //     if (pos[i][d] < settings->range_lo[d]) {

    //         pos[i][d] = settings->range_hi[d] - fmod(settings->range_lo[d] - pos[i][d],
    //                                                              settings->range_hi[d] - settings->range_lo[d]);
    //         vel[i][d] = 0;

    //     } else if (pos[i][d] > settings->range_hi[d]) {

    //         pos[i][d] = settings->range_lo[d] + fmod(pos[i][d] - settings->range_hi[d],
    //                                                              settings->range_hi[d] - settings->range_lo[d]);
    //         vel[i][d] = 0;
    //     }
    // }
}

//==============================================================
//                     PSO ALGORITHM
//==============================================================
// minimize the provided obj_fun using PSO with the specified settings
// and store the result in *solution
void PSO::pso_solve(pso_obj_fun_t obj_fun, void *obj_fun_params,
	       pso_result_t *solution, pso_settings_t *settings, ros::NodeHandle nh)
{

    // namedWindow("img");
    // waitKey(3000);
    struct timeval tstart, tend;
    gettimeofday(&tstart, NULL);
    ros::Publisher pub_accel = nh.advertise< strategy::particle >( "accel", 1000 );
    ros::Publisher solution_pub = nh.advertise< strategy::solution >( "solution_topic", 1000 );
    strategy::particle msg_accel;
    strategy::solution solution_msg;
    // Particles
    float **pos = pso_matrix_new(settings->size, settings->dim); // position matrix
    float **vel = pso_matrix_new(settings->size, settings->dim); // velocity matrix
    float **pos_b = pso_matrix_new(settings->size, settings->dim); // best position matrix
    double *fit = (double *)malloc(settings->size * sizeof(double));
    float *fit_b = (float *)malloc(settings->size * sizeof(float));
    // Swarm
    float **pos_nb = pso_matrix_new(settings->size, settings->dim); // what is best informed
    // position for each particle
    int *comm = (int *)malloc(settings->size * settings->size * sizeof(int));
    // rows : those who inform
    // cols : those who are informed
    int improved = 0; // whether solution->error was improved during
    // the last iteration

    int i, d, step;
    float a, b; // for matrix initialization
    float rho1, rho2; // random numbers (coefficients)
    // initialize omega using standard value
    double w = 1;//0.9;
    inform_fun_t inform_fun = NULL; // neighborhood update function
    inertia_fun_t calc_inertia_fun = NULL; // inertia weight update function

    
    double Periodtime;
    bool CCRisInObs = false;
    bool *posInObs = (bool *)malloc(settings->size * sizeof(bool));
    vector<Point3i> cpoint;
    int radius = 20;
    // int foot_area[4] = {-30,30,-40,40};
    int foot_area[4] = {0,0,0,0};
    // initialize random seed
    srand(time(NULL));

    // SELECT APPROPRIATE NHOOD UPDATE FUNCTION
    switch (settings->nhood_strategy)
        {
        // case PSO_NHOOD_GLOBAL:
        //     // comm matrix not used
        //     inform_fun = &PSO::inform_global;
        //     break;
        case PSO_NHOOD_RING:
            init_comm_ring(comm, settings);
            inform_fun = &PSO::inform_ring;
            break;
        // case PSO_NHOOD_RANDOM:
        //     init_comm_random(comm, settings);
        //     inform_fun = &PSO::inform_random;
        //     break;
        default:
            // use global as the default
            inform_fun = &PSO::inform_global;
            break;
        }

    // SELECT APPROPRIATE INERTIA WEIGHT UPDATE FUNCTION
    // switch (settings->w_strategy)
    //     {
    //         /* case PSO_W_CONST : */
    //         /*     calc_inertia_fun = calc_inertia_const; */
    //         /*     break; */
    //     case PSO_W_LIN_DEC :
            calc_inertia_fun = &PSO::calc_inertia_lin_dec;
    //         break;
    //     }

    // INITIALIZE SOLUTION
    solution->error = FLT_MAX;
    // SWARM INITIALIZATION
    // for each particle
    for (i=0; i<settings->size; i++) {
        // for each dimension
        for (d=0; d<settings->dim; d++) {
            // generate two numbers within the specified range
            a = settings->range_lo[d]-foot_area[d*2] + (settings->range_hi[d] - settings->range_lo[d]) * RNG_UNIFORM();
            b = settings->range_lo[d]-foot_area[d*2] + (settings->range_hi[d] - settings->range_lo[d]) * RNG_UNIFORM();
            // initialize position
            pos[i][d] = a;
            // best position is the same
            pos_b[i][d] = a;
            // initialize velocity
            vel[i][d] = (a-b) / 2.;
            if (settings->clamp_pos) {
                if (pos[i][d] < settings->range_lo[d]-foot_area[d*2]) {
                    pos[i][d] = settings->range_lo[d]-foot_area[d*2];
                    vel[i][d] = 0;
                } else if (pos[i][d] > settings->range_hi[d]-foot_area[d*2+1]) {
                    pos[i][d] = settings->range_hi[d]-foot_area[d*2+1]-1;
                    vel[i][d] = 0;
                }
            } else {
                // enforce periodic boundary conditions
                if (pos[i][d] < settings->range_lo[d]) {

                    pos[i][d] = settings->range_hi[d] - fmod(settings->range_lo[d] - pos[i][d],
                                                             settings->range_hi[d] - settings->range_lo[d]);
                    vel[i][d] = 0;

                } else if (pos[i][d] > settings->range_hi[d]) {

                    pos[i][d] = settings->range_lo[d] + fmod(pos[i][d] - settings->range_hi[d],
                                                             settings->range_hi[d] - settings->range_lo[d]);
                    vel[i][d] = 0;
                }
            }
            
        }
        // cout<<"size = "<<edgepoint_list.size()<<endl;
        for(int j = 0; j<edgepoint_list.size(); j++)
        {
            if(!posInObs[i])
            {
                posInObs[i] = Computational_geometry->isCircleInPolygon(edgepoint_list[j], Point3i(pos[i][0], pos[i][1], 0), radius);
                if(posInObs[i])
                    break;
            }
        }
        cpoint.push_back(Point3i(pos[i][0], pos[i][1], 0));
            //for(int i = 0;i<10000000; i++);
        msg_accel.x.push_back(pos[i][0]);
        msg_accel.y.push_back(pos[i][1]);
        if(posInObs[i])
        {        
            // update particle fitness
            fit[i] = (obj_fun)(pos[i], settings->dim, obj_fun_params, posInObs[i]);
        }
        else
        {
            fit[i] = 100;
        }
        // ROS_INFO("fit[%d] = %f", i, fit[i]);
        fit_b[i] = fit[i]; // this is also the personal best
        // update gbest??
        if (fit[i] < solution->error) {
            // update best fitness
            solution->error = fit[i];
            // copy particle pos to gbest vector
            memmove((void *)solution->gbest, (void *)pos[i],
                    sizeof(double) * settings->dim);
        }
        
        // ROS_INFO("no.%d error = %f", i, solution->error);
        
        
    }
    gx = solution->gbest[0];
    gy = solution->gbest[1];
    show_image(cpoint, 15, &posInObs[0], 0, gx, gy);
    printf("q\n");
    memset(posInObs, 0, settings->size * sizeof(bool));
    cpoint.clear();
    printf("q\n");
    msg_accel.cnt=settings->size;
    pub_accel.publish( msg_accel );
    msg_accel.x.clear();
    msg_accel.y.clear();
    solution_msg.x = solution->gbest[0];
    solution_msg.y = solution->gbest[1];
    solution_pub.publish(solution_msg);
    // for(int i = 0;i<100000000; i++);
    // sleep(3);
    // RUN ALGORITHM
    for (step=0; step<settings->steps; step++) {
        // update current step
        settings->step = step;
        // update inertia weight
        // do not bother with calling a calc_w_const function
        if (calc_inertia_fun != NULL) {
            w = w*0.85;//(this->*calc_inertia_fun)(step, settings);  //更新慣性權重
        }
        // check optimization goal
        if (solution->error <= settings->goal) {
            // SOLVED!!
            // if (settings->print_every)
                gettimeofday(&tend, NULL);
                Periodtime  = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;//算週期
                cout<<"solution->error <= settings->goal"<<endl;
                printf("Goal achieved @ step %d (error=%.3e) :-)\n", step, solution->error);
                ROS_INFO("gbest = %f, %f", solution->gbest[0], solution->gbest[1]);
                ROS_INFO("timeuse = %f", Periodtime);
                // ROS_INFO("settings->print_every = %d", settings->print_every);
                // sleep(2);
            break;
        }
        // update pos_nb matrix (find best of neighborhood for all particles)
        (this->*inform_fun)(comm, (float **)pos_nb, (float **)pos_b, fit_b, solution->gbest, improved, settings);
        // the value of improved was just used; reset it
        improved = 0;
        
        // update all particles
        for (i=0; i<settings->size; i++) {
            // for each dimension
            for (d=0; d<settings->dim; d++) {
                // calculate stochastic coefficients
                rho1 = settings->c1 * RNG_UNIFORM();
                rho2 = settings->c2 * RNG_UNIFORM();
                // update velocity
                vel[i][d] = w * vel[i][d] +	\
                    rho1 * (pos_b[i][d] - pos[i][d]) +	\
                    rho2 * (pos_nb[i][d] - pos[i][d]);
                // update position
                pos[i][d] += vel[i][d];
                // clamp position within bounds?
                if (settings->clamp_pos) {
                    if (pos[i][d] < settings->range_lo[d]-foot_area[d*2]) {
                        pos[i][d] = settings->range_lo[d]-foot_area[d*2];
                        vel[i][d] = 0;
                    } else if (pos[i][d] > settings->range_hi[d]-foot_area[d*2+1]) {
                        pos[i][d] = settings->range_hi[d]-foot_area[d*2+1]-1;
                        vel[i][d] = 0;
                    }
                } else {
                    // enforce periodic boundary conditions
                    if (pos[i][d] < settings->range_lo[d]) {

                        pos[i][d] = settings->range_hi[d] - fmod(settings->range_lo[d] - pos[i][d],
                                                                 settings->range_hi[d] - settings->range_lo[d]);
                        vel[i][d] = 0;

                    } else if (pos[i][d] > settings->range_hi[d]) {

                        pos[i][d] = settings->range_lo[d] + fmod(pos[i][d] - settings->range_hi[d],
                                                                 settings->range_hi[d] - settings->range_lo[d]);
                        vel[i][d] = 0;
                    }
                }
            }
            // ROS_INFO("i = %d", i);
                
            msg_accel.x.push_back(pos[i][0]);
            msg_accel.y.push_back(pos[i][1]);
            
            for(int j = 0; j<edgepoint_list.size(); j++)
            {
                if(!posInObs[i])
                {
                    posInObs[i] = Computational_geometry->isCircleInPolygon(edgepoint_list[j], Point3i(pos[i][0], pos[i][1], 0), radius);
                    if(posInObs[i])
                        break;
                }
            }
            cpoint.push_back(Point3i(pos[i][0], pos[i][1], 0));
            if(posInObs[i])
            {
                // update particle fitness
                fit[i] = (obj_fun)(pos[i], settings->dim, obj_fun_params, posInObs[i]);
                // ROS_INFO("i = %d, fit = %f, posx = %f, posy = %f", i, fit[i], msg_accel.x.back(), msg_accel.y.back());
                // ROS_INFO("w = %f", w);
                // update personal best position?
                if (fit[i] < fit_b[i]) {
                    fit_b[i] = fit[i];
                    // copy contents of pos[i] to pos_b[i]
                    memmove((void *)pos_b[i], (void *)pos[i],
                            sizeof(double) * settings->dim);
                    // ROS_INFO("best fit = %f, pos = %f , %f", fit_b[i],  pos_b[i][0], pos_b[i][1]);
                    // update gbest
                    if (fit_b[i] < solution->error) {
                        // ROS_INFO("update!!!!!!!!!!!!!!");
                        improved = 1;
                        // update best fitness
                        solution->error = fit_b[i];

                        // for(int i = 0;i<200000000; i++);
                        // copy particle pos to gbest vector
                        memmove((void *)solution->gbest, (void *)pos[i],
                                sizeof(double) * settings->dim);
                        // ROS_INFO("---------------------------------");
                        // ROS_INFO("gbest = %f, %f", solution->gbest[0], solution->gbest[1]);
                        // ROS_INFO("error = %f", solution->error);
                        // ROS_INFO("---------------------------------");
                        // for(int i = 0;i<500000000; i++);
                    }
                }
            }
            
            
            // ROS_INFO("goal = %f", settings->goal);
            // ROS_INFO("gbest = %f, %f", solution->gbest[0], solution->gbest[1]);
            // ROS_INFO("error = %f", solution->error);
            // //for(int i = 0;i<1000000; i++);
            
            
        }
        gx = solution->gbest[0];
        gy = solution->gbest[1];
        show_image(cpoint, radius, &posInObs[0], step+1, gx, gy);
        // printf("r\n");
        memset(posInObs, 0, settings->size * sizeof(bool));
        cpoint.clear();

        // ROS_INFO("step = %d", step);
        msg_accel.cnt=settings->size;
        pub_accel.publish( msg_accel );
        msg_accel.x.clear();
        msg_accel.y.clear();

                    solution_msg.x = solution->gbest[0];
                    solution_msg.y = solution->gbest[1];
                    solution_pub.publish(solution_msg);
        // for(int i = 0;i<50000000; i++);

        // if (settings->print_every && (step % settings->print_every == 0))
        //     printf("Step %d (w=%.2f) :: min err=%.5e\n", step, w, solution->error);
        // cout<<"goal = "<<settings->goal<<" error = "<<solution->error<<endl;
    }
    gettimeofday(&tend, NULL);
    Periodtime  = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;//算週期
    printf("Goal achieved @ step %d (error=%.3e) :-)\n", step, solution->error);
    printf("gbest = %f, %f\n", solution->gbest[0], solution->gbest[1]);
    printf("timeuse = %f\n", Periodtime);
    // Distance distance;
    // distance = measure((int)solution->gbest[0]*2, (int)solution->gbest[1]*2, CameraType::stereo);
    save_img(solution->gbest[0], solution->gbest[1], radius);
    name_cnt++;
    // sleep(2);
                // break;
    // free resources
    pso_matrix_free(pos, settings->size);
    pso_matrix_free(vel, settings->size);
    free(comm);
    free(fit);
    free(fit_b);
    free(posInObs);
}