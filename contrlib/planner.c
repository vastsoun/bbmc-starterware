/****************************************************************************
**                    INCLUDES
****************************************************************************/

//#include "math.h"
#include "planner.h"


/****************************************************************************
**                    MACRO DEFINITIONS
****************************************************************************/




/****************************************************************************
**                    EXTERNAL FUNCTION DECLARATIONS
****************************************************************************/



/*****************************************************************************
**                    FUNCTION DEFINITIONS
*****************************************************************************/

void trapezoid_setup(trapezoid_t *state_d, int plmode){
	state_d->step_num = (int)((state_d->t_f)/state_d->t_step);
	state_d->acc_d_2 = ((state_d->spd_ss)/(state_d->t_a))/2;
	state_d->pos_a = state_d->acc_d_2*state_d->t_a*state_d->t_a;
	state_d->pos_d = state_d->spd_ss*(state_d->t_f-1.5*state_d->t_a);
}

void trapezoid_online(trapezoid_t *state_d){
	static float temp;
	state_d->t_curr = state_d->t_curr + state_d->t_step;
	if(state_d->t_curr<=state_d->t_a){
		state_d->pos_curr = state_d->acc_d_2*(state_d->t_curr*state_d->t_curr);
	}
	if((state_d->t_curr>state_d->t_a)&&(state_d->t_curr<=state_d->t_d)){
		state_d->pos_curr = state_d->spd_ss*state_d->t_curr - state_d->pos_a;
	}
	if((state_d->t_curr>state_d->t_d)&&(state_d->t_curr<=state_d->t_f)){
		temp=(state_d->t_f - state_d->t_curr);
		state_d->pos_curr = state_d->pos_d - state_d->acc_d_2*temp*temp;
	}
}

//TODO
void trapezoid_offline(trapezoid_t *state_d){
	;
}
