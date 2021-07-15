#include <stdlib.h> // for rand() stuff
#include <stdio.h> // for printf
#include <time.h> // for time()
#include <math.h> // for cos(), pow(), sqrt() etc.
#include <float.h> // for DBL_MAX
#include <string.h> // for mem*

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <vector>
#include "strategy/pso.h"
#include "strategy/particle.h"

using namespace std;
// generates a double between (0, 1)
#define RNG_UNIFORM() (rand()/(double)RAND_MAX)

// generate an int between 0 and s (exclusive)
#define RNG_UNIFORM_INT(s) (rand()%s)

// function type for the different inform functions
typedef void (*inform_fun_t)(int *comm, double **pos_nb,
                             double **pos_b, double *fit_b,
                             double *gbest, int improved,
                             pso_settings_t *settings);

// function type for the different inertia calculation functions
typedef double (*inertia_fun_t)(int step, pso_settings_t *settings);


//==============================================================
// calulate swarm size based on dimensionality
int pso_calc_swarm_size(int dim) {
    int size = 10. + 2. * sqrt(dim);
    return (size > PSO_MAX_SIZE ? PSO_MAX_SIZE : size);
}


//==============================================================
//          INERTIA WEIGHT UPDATE STRATEGIES
//==============================================================
// calculate linearly decreasing inertia weight  //線性遞減慣性權重
double calc_inertia_lin_dec(int step, pso_settings_t *settings) {

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
void inform_global(int *comm, double **pos_nb,
		   double **pos_b, double *fit_b,
		   double *gbest, int improved,
		   pso_settings_t *settings)
{

    int i;
    // all particles have the same attractor (gbest)
    // copy the contents of gbest to pos_nb
    for (i=0; i<settings->size; i++)
        memmove((void *)pos_nb[i], (void *)gbest,
                sizeof(double) * settings->dim);

}


// ===============================================================
// general inform function :: according to the connectivity
// matrix COMM, it copies the best position (from pos_b) of the
// informers of each particle to the pos_nb matrix
void inform(int *comm, double **pos_nb, double **pos_b, double *fit_b,
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
                sizeof(double) * settings->dim);
    }
}




// =============
// ring topology
// =============

// topology initialization :: this is a static (i.e. fixed) topology
void init_comm_ring(int *comm, pso_settings_t * settings) {
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




void inform_ring(int *comm, double **pos_nb,
		 double **pos_b, double *fit_b,
		 double *gbest, int improved,
		 pso_settings_t * settings)
{

    // update pos_nb matrix
    inform(comm, pos_nb, pos_b, fit_b, improved, settings);

}

// ============================
// random neighborhood topology
// ============================
void init_comm_random(int *comm, pso_settings_t * settings) {

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



void inform_random(int *comm, double **pos_nb,
		   double **pos_b, double *fit_b,
		   double *gbest, int improved,
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

    // set some default values
    settings->dim = dim;  //dimensionality(維度)
    settings->goal = 1e-5;

    // set up the range arrays
    settings->range_lo = (double *)malloc(settings->dim * sizeof(double));
    if (settings->range_lo == NULL) { free(settings); return NULL; }

    settings->range_hi = (double *)malloc(settings->dim * sizeof(double));
    if (settings->range_hi == NULL) { free(settings); free(settings->range_lo); return NULL; }

    for (int i=0; i<settings->dim; i++) {
        settings->range_lo[i] = range_limit[i*2];
        settings->range_hi[i] = range_limit[i*2+1];
        ROS_INFO("range_lo = %f , range_hi = %f", settings->range_lo[i], settings->range_hi[i]);
    }
    sleep(2);
    // settings->size = pso_calc_swarm_size(settings->dim);
    settings->size = 50;
    settings->print_every = 10;
    settings->steps = 100;
    settings->c1 = 1.496;  //2
    settings->c2 = 1.496;  //2
    settings->w_max = 0.9;
    settings->w_min = 0.5;
    // settings->w_max = PSO_INERTIA;
    // settings->w_min = 0.3;

    settings->clamp_pos = 1;
    settings->nhood_strategy = PSO_NHOOD_RING;
    settings->nhood_size = 5;
    // settings->w_strategy = PSO_W_LIN_DEC;

    return settings;
}

// destroy PSO settings
void pso_settings_free(pso_settings_t *settings) {
    free(settings->range_lo);
    free(settings->range_hi);
    free(settings);
}


double **pso_matrix_new(int size, int dim) {
    double **m = (double **)malloc(size * sizeof(double *));
    for (int i=0; i<size; i++) {
        m[i] = (double *)malloc(dim * sizeof(double));
    }
    return m;
}

void pso_matrix_free(double **m, int size) {
    for (int i=0; i<size; i++) {
        free(m[i]);
    }
    free(m);
}


//==============================================================
//                     PSO ALGORITHM
//==============================================================
void pso_solve(pso_obj_fun_t obj_fun, void *obj_fun_params,
	       pso_result_t *solution, pso_settings_t *settings, ros::NodeHandle nh)
{
    struct timeval tstart, tend;
    gettimeofday(&tstart, NULL);
    ros::Publisher pub_accel = nh.advertise< strategy::particle >( "accel", 1000 );
    strategy::particle msg_accel;
    
    // Particles
    double **pos = pso_matrix_new(settings->size, settings->dim); // position matrix
    double **vel = pso_matrix_new(settings->size, settings->dim); // velocity matrix
    double **pos_b = pso_matrix_new(settings->size, settings->dim); // best position matrix
    double *fit = (double *)malloc(settings->size * sizeof(double));
    double *fit_b = (double *)malloc(settings->size * sizeof(double));
    // Swarm
    double **pos_nb = pso_matrix_new(settings->size, settings->dim); // what is best informed
    // position for each particle
    int *comm = (int *)malloc(settings->size * settings->size * sizeof(int));
    // rows : those who inform
    // cols : those who are informed
    int improved = 0; // whether solution->error was improved during
    // the last iteration

    int i, d, step;
    double a, b; // for matrix initialization
    double rho1, rho2; // random numbers (coefficients)
    // initialize omega using standard value
    double w = 0.9;
    inform_fun_t inform_fun = NULL; // neighborhood update function
    inertia_fun_t calc_inertia_fun = NULL; // inertia weight update function

    float qq, ww;
    int ee;
    double Periodtime;

    int foot_area[4] = {-30,30,-40,40};
    // initialize random seed
    srand(time(NULL));

    // SELECT APPROPRIATE NHOOD UPDATE FUNCTION
    switch (settings->nhood_strategy)
        {
        case PSO_NHOOD_GLOBAL:
            // comm matrix not used
            inform_fun = inform_global;
            break;
        case PSO_NHOOD_RING:
            init_comm_ring(comm, settings);
            inform_fun = inform_ring;
            break;
        case PSO_NHOOD_RANDOM:
            init_comm_random(comm, settings);
            inform_fun = inform_random;
            break;
        default:
            // use global as the default
            inform_fun = inform_global;
            break;
        }

    // SELECT APPROPRIATE INERTIA WEIGHT UPDATE FUNCTION
    // switch (settings->w_strategy)
    //     {
    //         /* case PSO_W_CONST : */
    //         /*     calc_inertia_fun = calc_inertia_const; */
    //         /*     break; */
    //     case PSO_W_LIN_DEC :
            calc_inertia_fun = calc_inertia_lin_dec;
    //         break;
    //     }

    // INITIALIZE SOLUTION
    solution->error = DBL_MAX;

    // SWARM INITIALIZATION
    // for each particle
    for (i=0; i<settings->size; i++) {
        // for each dimension
        for (d=0; d<settings->dim; d++) {
            // generate two numbers within the specified range
            a = settings->range_lo[d] + (settings->range_hi[d] - settings->range_lo[d]) * RNG_UNIFORM();
            b = settings->range_lo[d] + (settings->range_hi[d] - settings->range_lo[d]) * RNG_UNIFORM();
            // initialize position
            pos[i][d] = a;
            // best position is the same
            pos_b[i][d] = a;
            // initialize velocity
            vel[i][d] = (a-b) / 2.;
        }
        // update particle fitness
        ROS_INFO("pos[%d][0] = %f, pos[%d][1] = %f", i, pos[i][0], i, pos[i][1]);
        fit[i] = obj_fun(pos[i], settings->dim, obj_fun_params);
        ROS_INFO("fit[%d] = %f", i, fit[i]);
        fit_b[i] = fit[i]; // this is also the personal best
        // update gbest??
        if (fit[i] < solution->error) {
            // update best fitness
            solution->error = fit[i];
            // copy particle pos to gbest vector
            memmove((void *)solution->gbest, (void *)pos[i],
                    sizeof(double) * settings->dim);
        }
        ROS_INFO("no.%d error = %f", i, solution->error);
    }
    // RUN ALGORITHM
    for (step=0; step<settings->steps; step++) {
        // update current step
        settings->step = step;
        // update inertia weight
        // do not bother with calling a calc_w_const function
        if (calc_inertia_fun != NULL) {
            w = calc_inertia_fun(step, settings);  //更新慣性權重
        }
        ROS_INFO("bbberror = %f", solution->error);
        ROS_INFO("rrr");
        // check optimization goal
        if (solution->error <= settings->goal) {
            // SOLVED!!
            // if (settings->print_every)
                gettimeofday(&tend, NULL);
                Periodtime  = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;//算週期
                printf("Goal achieved @ step %d (error=%.3e) :-)\n", step, solution->error);
                ROS_INFO("ee = %d, qq = %f, ww = %f", ee, qq, ww);
                ROS_INFO("pos %f , %f", pos[ee][0], pos[ee][1]);
                ROS_INFO("timeuse = %f", Periodtime);
                // ROS_INFO("settings->print_every = %d", settings->print_every);
                // sleep(2);
            break;
        }
        // ROS_INFO("aaaerror = %f", solution->error);
        // update pos_nb matrix (find best of neighborhood for all particles)
        inform_fun(comm, (double **)pos_nb, (double **)pos_b,
                   fit_b, solution->gbest, improved, settings);
        // the value of improved was just used; reset it
        improved = 0;
        // ROS_INFO("rrrerror = %f", solution->error);
        
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
                        pos[i][d] = settings->range_hi[d]-foot_area[d*2+1];
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
            // ROS_INFO("gggerror = %f", solution->error);
            
            // update particle fitness
            fit[i] = obj_fun(pos[i], settings->dim, obj_fun_params);
            // ROS_INFO("xxerror = %f", solution->error);
            ROS_INFO("i = %d, fit = %f, posx = %f, posy = %f", i, fit[i], msg_accel.x.back(), msg_accel.y.back());
            ROS_INFO("w = %f", w);
            // update personal best position?
            if (fit[i] < fit_b[i]) {
                fit_b[i] = fit[i];
                // copy contents of pos[i] to pos_b[i]
                memmove((void *)pos_b[i], (void *)pos[i],
                        sizeof(double) * settings->dim);
                ROS_INFO("best fit = %f, pos = %f , %f", fit_b[i],  pos_b[i][0], pos_b[i][1]);
                // ROS_INFO("wwwerror = %f", solution->error);
                // update gbest
                if (fit_b[i] < solution->error) {
                    ROS_INFO("update!!!!!!!!!!!!!!");
                    improved = 1;
                    // update best fitness
                    solution->error = fit_b[i];
                    qq = pos[i][0];
                    ww = pos[i][1];
                    ee = i;

                    // for(int i = 0;i<200000000; i++);
                    // copy particle pos to gbest vector
                    memmove((void *)solution->gbest, (void *)pos[i],
                            sizeof(double) * settings->dim);
                    ROS_INFO("---------------------------------");
                    ROS_INFO("gbest = %f, %f", solution->gbest);
                    ROS_INFO("error = %f", solution->error);
                    ROS_INFO("---------------------------------");
                    for(int i = 0;i<1000000000; i++);
                }
            }
            ROS_INFO("goal = %f", settings->goal);
            // ROS_INFO("fit = %f", fit[i]);
            // ROS_INFO("aaaerror = %f", solution->error);
            // ROS_INFO("q = %f", solution->gbest);
            // ROS_INFO("error = %f", solution->error);
            // update gbest??
            // if (fit[i] < solution->error) {
            //     ROS_INFO("update!!!!!!!!!!!!!!");
            //     improved = 1;
            //     // update best fitness
            //     solution->error = fit[i];
            //     qq = pos[i][0];
            //     ww = pos[i][1];
            //     ee = i;

            //     // for(int i = 0;i<200000000; i++);
            //     // copy particle pos to gbest vector
            //     memmove((void *)solution->gbest, (void *)pos[i],
            //             sizeof(double) * settings->dim);
            //     ROS_INFO("---------------------------------");
            //     ROS_INFO("gbest = %f, error = %f", solution->gbest, solution->error);
            //     ROS_INFO("---------------------------------");
            // }
            // ROS_INFO("gbest = %f, error = %f", solution->gbest, solution->error);
            ROS_INFO("gbest = %f, %f", solution->gbest);
            ROS_INFO("error = %f", solution->error);
        }
            ROS_INFO("step = %d", step);
            msg_accel.cnt=settings->size;
            pub_accel.publish( msg_accel );
            msg_accel.x.clear();
            msg_accel.y.clear();
            // for(int i=0; i<settings->size; i++)
            // {
            //     msg_accel.y[i] = 0;
            // }
            // ROS_INFO("msg_accel.cnt = %d", msg_accel.cnt);
            // ros::spinOnce();
            for(int i = 0;i<50000000; i++);

        // if (settings->print_every && (step % settings->print_every == 0))
        //     printf("Step %d (w=%.2f) :: min err=%.5e\n", step, w, solution->error);

    }
    // gettimeofday(&tend, NULL);
    //             Periodtime  = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;//算週期
                printf("Goal achieved @ step %d (error=%.3e) :-)\n", step, solution->error);
                ROS_INFO("ee = %d, qq = %f, ww = %f", ee, qq, ww);
                ROS_INFO("pos %f , %f", pos[ee][0], pos[ee][1]);
                ROS_INFO("timeuse = %f", Periodtime);
                // break;
    // free resources
    pso_matrix_free(pos, settings->size);
    pso_matrix_free(vel, settings->size);
    free(comm);
    free(fit);
    free(fit_b);
}