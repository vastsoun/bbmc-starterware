/**
 * \file   systick.c
 *
 * \brief  system timer tick routines. This can be used to call a function
 *		during specific intervels .
 *
*/

#include "systick.h"

/****************************************************************************
**                        FUNCTION DEFINITION                               
****************************************************************************/

/**
 * \brief   This function registers the periodic handler
 *          
 *          
 *          
 * \param  void (*pfnHandler)(void)  This is a pointer to the periodic handler.
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform 
 *            specific files
 *
 *
 */
void SystickConfigure(void (*pfnHandler)(void))
{    
	 TimerTickConfigure(pfnHandler);
}


/**
 * \brief   This function sets the  period to call the Handler
 *          
 *          
 *          
 * \param  milliSec This is the number of milli seconds for the period
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform
 *            specific files
 *
 *
 */
void SystickPeriodSet(unsigned int milliSec)
{
	TimerTickPeriodSet(milliSec);
	
}

/**
 * \brief   This function Enables the Systick. This has to be called 
 *           to start the periodic function which is registered
 *          
 *          
 * \param  None
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform
 *            specific files
 *
 *
 */
void SystickEnable(void)
{	

	/* Start the timer. Characters from cntArr will be sent from the ISR */
	TimerTickEnable();
}

/**
 * \brief   This function Disables the Systick. This has to be called 
 *           to stop the periodic function which is registered
 *          
 *          
 * \param  milliSec This is the number of milli seconds for the period
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform
 *            specific files
 *
 *
 */
void SystickDisable(void)
{
	TimerTickDisable();
}



