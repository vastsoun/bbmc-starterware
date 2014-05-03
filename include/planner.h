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

typedef struct {
	float t_step;
	float t_f;
	float t_d;
	float t_a;
	float t_curr;
	int step_num;
	float pos_s;
	float pos_f;
	float pos_a;
	float pos_d;
	float pos_curr;
	float spd_curr;
	float spd_ss;
	float acc_d_2;
	float constr_max;
	float constr_min;
} trapezoid_t;


/****************************************************************************
**                    FUNCTION PROTOTYPES
****************************************************************************/

void trapezoid_setup(trapezoid_t *state_d, int plmode);
void trapezoid_offline(trapezoid_t *state_d);
void trapezoid_online(trapezoid_t *state_d);


#ifdef __cplusplus
}
#endif
#endif
