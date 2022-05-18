#ifndef PSO_H
#define PSO_H


// CONSTANTS
#define PSO_MAX_SIZE 100 // max swarm size
#define PSO_INERTIA 0.7298 // default value of w (see clerc02)


// === NEIGHBORHOOD SCHEMES ===

//global best topology
#define PSO_NHOOD_GLOBAL 0

// ring topology
#define PSO_NHOOD_RING 1

//Random neighborhood topology
//**see http://clerc.maurice.free.fr/pso/random_topology.pdf**
#define PSO_NHOOD_RANDOM 2

// === INERTIA WEIGHT UPDATE FUNCTIONS ===
#define PSO_W_CONST 0
#define PSO_W_LIN_DEC 1

// generates a double between (0, 1)
#define RNG_UNIFORM() (rand()/(double)RAND_MAX)

// generate an int between 0 and s (exclusive)
#define RNG_UNIFORM_INT(s) (rand()%s)

#include <stdlib.h> // for rand() stuff
#include <stdio.h> // for printf
#include <time.h> // for time()
#include <math.h> // for cos(), pow(), sqrt() etc.
#include <float.h> // for DBL_MAX
#include <string.h> // for mem*
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include "strategy/computational_geometry.h"
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include "strategy/particle.h"
#include "strategy/solution.h"
#include <geometry_msgs/Polygon.h>
#include "strategy/EdgePointList.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "strategy/ReachableRegion.h"

// #include "FeatureDistance/FeatureDistance.h"
using namespace cv;

using namespace std;

// PSO SOLUTION -- Initialized by the user
typedef struct {

    float error;
    float *gbest; // should contain DIM elements!!

} pso_result_t;


class PSO;
// OBJECTIVE FUNCTION TYPE
typedef double (*pso_obj_fun_t)(float *, int, void *, bool);



// PSO SETTINGS
typedef struct {

    int dim; // problem dimensionality
    double *range_lo; // lower range limit (array of length DIM)
    double *range_hi; // higher range limit (array of length DIM)
    float goal; // optimization goal (error threshold)

    int size; // swarm size (number of particles)
    int print_every; // ... N steps (set to 0 for no output)
    int steps; // maximum number of iterations
    int step; // current PSO step
    float c1; // cognitive coefficient
    float c2; // social coefficient
    float w_max; // max inertia weight value
    float w_min; // min inertia weight value

    int clamp_pos; // whether to keep particle position within defined bounds (TRUE)
    // or apply periodic boundary conditions (FALSE)
    int nhood_strategy; // neighborhood strategy (see PSO_NHOOD_*)
    int nhood_size; // neighborhood size
    int w_strategy; // inertia weight strategy (see PSO_W_*)

} pso_settings_t;

pso_settings_t *pso_settings_new(int dim, float* range_limit, float* range_coordinate);

// function type for the different inform functions
typedef void (PSO::*inform_fun_t)(int *comm, float **pos_nb,
                             float **pos_b, float *fit_b,
                             float *gbest, int improved,
                             pso_settings_t *settings);

// function type for the different inertia calculation functions
typedef double (PSO::*inertia_fun_t)(int step, pso_settings_t *settings);

class PSO// : public FeatureDistance
{
    public:
        PSO();
        ~PSO();
        void initialize();
        // FeatureDistanceInstance *FeatureDistance;
        // typedef double (*inertia_fun_t)(int step, pso_settings_t *settings);
        int pso_calc_swarm_size(int dim);
        double calc_inertia_lin_dec(int step, pso_settings_t *settings);
        void pso_matrix_free(float **m, int size);
        void pso_solve(pso_obj_fun_t obj_fun, void *obj_fun_params,
	       pso_result_t *solution, pso_settings_t *settings, ros::NodeHandle nh);
        void position_limit(float *pos, float *vel, pso_settings_t *settings);
        void pso_settings_free(pso_settings_t *settings);
        void inform_random(int *comm, float **pos_nb, float **pos_b, float *fit_b, float *gbest, int improved, pso_settings_t * settings);
        void inform_ring(int *comm, float **pos_nb, float **pos_b, float *fit_b, float *gbest, int improved, pso_settings_t * settings);
        void inform_global(int *comm, float **pos_nb, float **pos_b, float *fit_b, float *gbest, int improved, pso_settings_t *settings);
        void inform(int *comm, float **pos_nb, float **pos_b, float *fit_b, int improved, pso_settings_t * settings);
        void init_comm_ring(int *comm, pso_settings_t * settings);
        void init_comm_random(int *comm, pso_settings_t * settings);
        void Reachable_Region(const strategy::ReachableRegion &msg);
        // void GetIMUData(const geometry_msgs::Vector3Stamped &msg);
        // void DepthCallback(const sensor_msgs::ImageConstPtr& depth_img);
        void get_edgepoint(const strategy::EdgePointList &msg);
        void get_edgeimg(const sensor_msgs::ImageConstPtr& msg);
        void show_image(const vector<Point3i>& c,  int radius, bool* InRegion, int step, int gx, int gy);
        void save_img(int gx, int gy, int radius);
        // double pso_sphere(double *pos, int dim, void *params);
        // pso_settings_t *pso_settings_new(int dim, float* range_limit, float* range_coordinate);
        static double sum_objective_function;
        vector<vector<Point3i>> edgepoint_list;
        vector<Point3i> edge_point;
        sensor_msgs::ImagePtr edgeimage_msg;
        // sensor_msgs::ImagePtr msg_depth;
        int gx, gy;
        float freelimit[4] = {0};
        int freecenter[2] = {0};
        Mat edge_img;
        int name_cnt;
        char path_1[200] = "/home/iclab/Desktop/PSO/finalimage.png";
        char path_2[200] = "/home/iclab/Desktop/PSO/data/final";
        
    private:
        ros::NodeHandle nh;
        // ros::Subscriber GetIMUData_Subscriber;
        // ros::Subscriber Depthimage_subscriber;
		ros::Subscriber edgepoint_subscriber;
        ros::Subscriber edgeimg_subscriber;
        ros::Subscriber Reachable_region_sub;
        image_transport::Publisher edgeimage_Publisher;
        // image_transport::Publisher depthimage_Publisher;
        // edgeimage_Publisher = it.advertise("edge_image", 1, this);
    protected:
        Computational_geometryInstance *Computational_geometry;
        // FeatureDistanceInstance *FeatureDistance;
};

class PSO_geometryInstance : public PSO
{
    public:
        PSO_geometryInstance() : PSO(){}
        ~PSO_geometryInstance(){}
        static PSO_geometryInstance* getInstance();
        static void deleteInstance();
    private:
        static PSO_geometryInstance* m_pInstance;

};

#endif // PSO_H