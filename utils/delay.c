/**
 *  \file   delay.c
 *
 *  \brief  This file contains APIs to configure a DMTimer instance for
 *          operation and to generate a requested amount of delay.
*/

#include "delay.h"


/****************************************************************************
**                        FUNCTION DEFINITION
****************************************************************************/

/**
 * \brief   This function configures a DMTimer instance for use.
 *
 * \param   None
 *
 * \return  None.
 *
 * \Note   This a wrapper API. The actual implementation can be found in
 *         platform specific files.
 */

void DelayTimerSetup(void)
{
    SysDelayTimerSetup();
}


/**
 * \brief   This function generates a delay of specified milli-seconds.
 *
 * \param   milliSec     This is the number of milli-seconds of delay.
 *
 * \return  None.
 *
 * \Note   This a wrapper API. The actual implementation can be found in
 *         platform specific files.
 *         This function shouldnot be called when StartTimer, StopTimer and
 *         IsTimerElapsed functionality is in use.
 */

void delay(unsigned int milliSec)
{
    Sysdelay(milliSec);
}

/**
 * \brief   This function starts the timer for millisec timeout.
 *
 * \param   milliSec     This is the number of milli-seconds of delay.
 *
 * \return  None.
 */

void StartTimer(unsigned int millisec)
{
    SysStartTimer(millisec);
}

/**
 * \brief   This function starts the timer for millisec timeout.
 *
 * \param   None.
 *
 * \return  None.
 *
 * \NOTE    delay functionality cannot be used till StopTimer is called.
 */
void StopTimer()
{
    SysStopTimer();
}

/**
 * \brief   This function checks whether timer is expired for set milli secs 
 *
 * \param   None. 
 *
 * \return  None.
 *
 * \NOTE   SysStartTimer has to be called prior to checking status.
 *         delay functionality cannot be used till SysStopTimer is called.
 */
unsigned int IsTimerElapsed(void)
{
    return (SysIsTimerElapsed());
}

