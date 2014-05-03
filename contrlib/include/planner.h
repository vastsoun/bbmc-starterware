/**
 *  \file   planner.h
 *
 *  \brief  Trajectory planner function library
*/

#ifndef PLANNER_H
#define PLANNER_H


#ifdef __cplusplus
extern "C" {
#endif
/****************************************************************************
**                       MACRO DEFINITIONS
****************************************************************************/

#define PLANNER_MAX_STEP 100000


/****************************************************************************
**                       DATA DEFINITIONS
****************************************************************************/

typedef struct mytrapezoid{
	double t_step;
	double t_f;
	double t_d;
	double t_a;
	double t_curr;
	int step_num;
	double pos_s;
	double pos_f;
	double pos_a;
	double pos_d;
	double pos_curr;
	double spd_curr;
	double spd_ss;
	double acc_d_2;
	double constr_max;
	double constr_min;
} trapezoid_t;


/****************************************************************************
**                    FUNCTION PROTOTYPES
****************************************************************************/

extern void trapezoid_setup(trapezoid_t *state_d, int plmode);
extern void trapezoid_offline(trapezoid_t *state_d);
extern void trapezoid_online(trapezoid_t *state_d);


#ifdef __cplusplus
}
#endif
#endif
