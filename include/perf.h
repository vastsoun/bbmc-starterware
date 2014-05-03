/**
 * \file   perf.h
 *
 * \brief  This file contains the prototypes of the functions present in
 *         utils/perf.c
 */


#ifndef _PERF_H_
#define _PERF_H_

#ifdef __cplusplus
extern "C" {
#endif

extern void PerfTimerSetup(void);
extern void PerfTimerStart(void);
extern unsigned int PerfTimerStop(void);

/*
** External function prototypes
*/
extern void SysPerfTimerSetup(void);
extern unsigned int SysPerfTimerConfig(unsigned int flag);

#ifdef __cplusplus
}
#endif
#endif


