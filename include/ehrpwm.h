/** ============================================================================
 *   \file  ehrpwm.h
 *
 *   \brief This file contains the Macros and API prototypes for ehrpwm driver
 *
 *  ============================================================================
 */

#ifndef _EHRPWM_H_
#define _EHRPWM_H_

/* 
 * Requires Headers 
 */
 
#include "hw_ehrpwm.h"


#ifdef __cplusplus
extern "C" {
#endif


/*
 * Device Macros
 */

/* TB Period load */
#define EHRPWM_PRD_LOAD_SHADOW_MASK     EHRPWM_TBCTL_PRDLD

/* Counter mode */
#define EHRPWM_COUNTER_MODE_MASK        EHRPWM_TBCTL_CTRMODE 
#define EHRPWM_COUNT_UP             (EHRPWM_TBCTL_CTRMODE_UP << \
                                EHRPWM_TBCTL_CTRMODE_SHIFT)
#define EHRPWM_COUNT_DOWN           (EHRPWM_TBCTL_CTRMODE_DOWN << \
                            EHRPWM_TBCTL_CTRMODE_SHIFT)
#define EHRPWM_COUNT_UP_DOWN            (EHRPWM_TBCTL_CTRMODE_UPDOWN << \
                            EHRPWM_TBCTL_CTRMODE_SHIFT)
#define EHRPWM_COUNT_STOP           (EHRPWM_TBCTL_CTRMODE_STOPFREEZE << \
                            EHRPWM_TBCTL_CTRMODE_SHIFT)
/* Synchronization */
#define EHRPWM_SYNC_ENABLE          EHRPWM_TBCTL_PHSEN
//#define EHRPWM_SW_FORCED_SYNC         0x1

#define EHRPWM_SYNCOUT_MASK         EHRPWM_TBCTL_SYNCOSEL   
#define EHRPWM_SYNCOUT_SYNCIN           (EHRPWM_TBCTL_SYNCOSEL_EPWMXSYNCI << \
                            EHRPWM_TBCTL_SYNCOSEL_SHIFT)
#define EHRPWM_SYNCOUT_COUNTER_EQUAL_ZERO   (EHRPWM_TBCTL_SYNCOSEL_TBCTRZERO << \
                            EHRPWM_TBCTL_SYNCOSEL_SHIFT)
#define EHRPWM_SYNCOUT_COUNTER_EQUAL_COMPAREB   (EHRPWM_TBCTL_SYNCOSEL_TBCTRCMPB << \
                            EHRPWM_TBCTL_SYNCOSEL_SHIFT)
#define EHRPWM_SYNCOUT_DISABLE          (EHRPWM_TBCTL_SYNCOSEL_DISABLE << \
                            EHRPWM_TBCTL_SYNCOSEL_SHIFT)

/* Shadow */
#define EHRPWM_SHADOW_WRITE_ENABLE      0x0
#define EHRPWM_SHADOW_WRITE_DISABLE     0x1

/* Emulation mode */
#define EHRPWM_STOP_AFTER_NEXT_TB_INCREMENT (0x0 << EHRPWM_TBCTL_FREE_SOFT_SHIFT)
#define EHRPWM_STOP_AFTER_A_COMPLETE_CYCLE  (0x1 << EHRPWM_TBCTL_FREE_SOFT_SHIFT)
#define EHRPWM_FREE_RUN             (0x2 << EHRPWM_TBCTL_FREE_SOFT_SHIFT)

/* Time base clock */
#define EHRPWM_TBCTL_CLKDIV_1           (0x0001u)
#define EHRPWM_TBCTL_CLKDIV_2           (0x0002u)
#define EHRPWM_TBCTL_CLKDIV_4           (0x0004u)
#define EHRPWM_TBCTL_CLKDIV_8           (0x0008u)
#define EHRPWM_TBCTL_CLKDIV_16          (0x0010u)
#define EHRPWM_TBCTL_CLKDIV_32          (0x0020u)
#define EHRPWM_TBCTL_CLKDIV_64          (0x0040u)
#define EHRPWM_TBCTL_CLKDIV_128         (0x0080u)

#define EHRPWM_TBCTL_HSPCLKDIV_1        (0x0001u)
#define EHRPWM_TBCTL_HSPCLKDIV_2        (0x0002u)
#define EHRPWM_TBCTL_HSPCLKDIV_4        (0x0004u)
#define EHRPWM_TBCTL_HSPCLKDIV_6        (0x0006u)
#define EHRPWM_TBCTL_HSPCLKDIV_8        (0x0008u)
#define EHRPWM_TBCTL_HSPCLKDIV_10       (0x000Au)
#define EHRPWM_TBCTL_HSPCLKDIV_12       (0x000Cu)
#define EHRPWM_TBCTL_HSPCLKDIV_14       (0x000Eu)

/* Count direction after sync */
#define EHRPWM_COUNT_DOWN_AFTER_SYNC        0x0
#define EHRPWM_COUNT_UP_AFTER_SYNC      0x1


/* Counter Compare */
#define EHRPWM_SHADOW_A_EMPTY           (0x0 << EHRPWM_CMPCTL_SHDWAFULL_SHIFT)
#define EHRPWM_SHADOW_A_FULL            (EHRPWM_CMPCTL_SHDWAFULL)
#define EHRPWM_SHADOW_B_EMPTY           (0x0 << EHRPWM_CMPCTL_SHDWBFULL_SHIFT)
#define EHRPWM_SHADOW_B_FULL            (EHRPWM_CMPCTL_SHDWBFULL)

#define EHRPWM_CMPCTL_NOT_OVERWR_SH_FL      0x0
#define EHRPWM_CMPCTL_OVERWR_SH_FL      0x1

/* Compare register load */
#define EHRPWM_COMPB_LOAD_MASK          EHRPWM_CMPCTL_LOADBMODE
#define EHRPWM_COMPB_LOAD_COUNT_EQUAL_ZERO  (EHRPWM_CMPCTL_LOADBMODE_TBCTRZERO << \
                            EHRPWM_CMPCTL_LOADBMODE_SHIFT)
#define EHRPWM_COMPB_LOAD_COUNT_EQUAL_PERIOD    (EHRPWM_CMPCTL_LOADBMODE_TBCTRPRD << \
                            EHRPWM_CMPCTL_LOADBMODE_SHIFT)
#define EHRPWM_COMPB_LOAD_COUNT_EQUAL_ZERO_OR_PERIOD \
                        (EHRPWM_CMPCTL_LOADBMODE_ZEROORPRD << \
                            EHRPWM_CMPCTL_LOADBMODE_SHIFT)
#define EHRPWM_COMPB_NO_LOAD            (EHRPWM_CMPCTL_LOADBMODE_FREEZE << \
                            EHRPWM_CMPCTL_LOADBMODE_SHIFT)


#define EHRPWM_COMPA_LOAD_MASK          EHRPWM_CMPCTL_LOADAMODE
#define EHRPWM_COMPA_LOAD_COUNT_EQUAL_ZERO  (EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO << \
                            EHRPWM_CMPCTL_LOADAMODE_SHIFT)
#define EHRPWM_COMPA_LOAD_COUNT_EQUAL_PERIOD    (EHRPWM_CMPCTL_LOADAMODE_TBCTRPRD << \
                            EHRPWM_CMPCTL_LOADAMODE_SHIFT)
#define EHRPWM_COMPA_LOAD_COUNT_EQUAL_ZERO_OR_PERIOD \
                        (EHRPWM_CMPCTL_LOADAMODE_ZEROORPRD << \
                            EHRPWM_CMPCTL_LOADAMODE_SHIFT)                       
#define EHRPWM_COMPA_NO_LOAD            (EHRPWM_CMPCTL_LOADAMODE_FREEZE << \
                            EHRPWM_CMPCTL_LOADAMODE_SHIFT)


/* Chopper */
#define EHRPWM_CHP_DUTY_12_5_PER        EHRPWM_PCCTL_CHPDUTY_1DIV8
#define EHRPWM_CHP_DUTY_25_PER          EHRPWM_PCCTL_CHPDUTY_2DIV8
#define EHRPWM_CHP_DUTY_37_5_PER        EHRPWM_PCCTL_CHPDUTY_3DIV8
#define EHRPWM_CHP_DUTY_50_PER          EHRPWM_PCCTL_CHPDUTY_4DIV8
#define EHRPWM_CHP_DUTY_62_5_PER        EHRPWM_PCCTL_CHPDUTY_5DIV8
#define EHRPWM_CHP_DUTY_75_PER          EHRPWM_PCCTL_CHPDUTY_6DIV8
#define EHRPWM_CHP_DUTY_87_5_PER        EHRPWM_PCCTL_CHPDUTY_7DIV8

/* TZ */
#define EHRPWM_TZ_ONESHOT           0x0
#define EHRPWM_TZ_CYCLEBYCYCLE          0x1
#define EHRPWM_TZ_ONESHOT_CLEAR         (EHRPWM_TZCLR_OST | EHRPWM_TZCLR_INT)
#define EHRPWM_TZ_CYCLEBYCYCLE_CLEAR        (EHRPWM_TZCLR_CBC | EHRPWM_TZCLR_INT)

#define  ECAP   0x01
#define  EPWM   0x02
#define  EQEP   0x03


/* Functionality Definitions */
#define EHRPWM_WRITE_DIFF       (0)
#define EHRPWM_WRITE_A          (1)
#define EHRPWM_WRITE_B          (2)


/** 
 *  Driver Data Structs
*/

typedef char bool;


/**
 * \struct  ehrpwm_device_t
 * \brief   am335x-ehrpwm device registers
 * 
 * This data struct is interface to access the memory mapped registers of the
 * peripheral hardware.
*/
typedef struct 
{
    hw_devreg16_t tbctl;
    hw_devreg16_t tbsts;
    hw_devreg16_t tbphshr;
    hw_devreg16_t tbphs;
    hw_devreg16_t tbcnt;
    hw_devreg16_t tbprd;
    hw_devreg16_t padding0;
    
    hw_devreg16_t cmpctl;
    hw_devreg16_t cmpahr;
    hw_devreg16_t cmpa;
    hw_devreg16_t cmpb;
    
    hw_devreg16_t aqctla;
    hw_devreg16_t aqctlb;
    hw_devreg16_t aqsfrc;
    hw_devreg16_t aqcsfrc;
    
    hw_devreg16_t dbctl;
    hw_devreg16_t dbred;
    hw_devreg16_t dbfed;
    
    hw_devreg16_t tzsel;
    hw_devreg16_t padding1;
    hw_devreg16_t tzctl;
    hw_devreg16_t tzeint;
    hw_devreg16_t tzflg;
    hw_devreg16_t tzclr;
    hw_devreg16_t tzfrc;
    
    hw_devreg16_t etsel;
    hw_devreg16_t etps;
    hw_devreg16_t etflg;
    hw_devreg16_t etclr;
    hw_devreg16_t etfrc;
    
    hw_devreg16_t pcctl;
    hw_devreg16_t padding2;
    hw_devreg16_t hrctl;
} 
ehrpwm_device_t;


/**
 * \struct  ehrpwm_handle_t
 * \brief   am335x-ehrpwm device handle
 * 
 * This data struct is wraps the interface to the device registers of the
 * peripheral hardware.
 * 
 * \param device_id The id-tag that designates the ehrpwm instance to be handled
 * 
 * \param config This struct holds the (local) configurations (i.e. context of the
 *                handled device. This is passed as an argument to the handler API in order
 *                load the desired configuration onto the actual device register file.
*/
typedef struct
{
    unsigned int device_id;
    
    ehrpwm_device_t config;
} 
ehrpwm_handle_t;




/*
 * Register API Function Definitions: Struct-Handler Model 
 */


/* General Functions */

extern int ehrpwm_open (unsigned int device_id);

extern int ehrpwm_close (unsigned int device_id);

extern int ehrpwm_init (unsigned int device_id);

extern int ehrpwm_enable (unsigned int device_id);

extern int ehrpwm_disable (unsigned int device_id);

extern int ehrpwm_config_set (ehrpwm_handle_t *handle);

extern int ehrpwm_config_get (ehrpwm_handle_t *handle);


/* Specialized functions */

extern int ehrpwm_handle_init (unsigned int device_id, ehrpwm_handle_t *handle);

extern int ehrpwm_config_frequency_set (unsigned int device_id, 
                                         double frequency, 
                                         unsigned int resolution);
                                         
extern int ehrpwm_config_frequency_get (unsigned int device_id, 
                                         double *ret_frequency);

extern int ehrpwm_write (unsigned int device_id, 
                          unsigned int output_mode, 
                          double duty_cycle);



/*
 * Register API Function Definitions: Direct-Acces Model
 */
 
/* Timebase sub-module */

void EHRPWMTimebaseClkConfig(unsigned int baseAddr, unsigned int tbClk, 
        unsigned int moduleClk);
        
void EHRPWMPWMOpFreqSet(unsigned int baseAddr, unsigned int tbClk,
        unsigned int pwmFreq,unsigned int counterDir, 
        bool enableShadowWrite);
        
void EHRPWMTBEmulationModeSet(unsigned int baseAddr, unsigned int mode);

void EHRPWMTimebaseSyncEnable(unsigned int baseAddr, unsigned int tbPhsValue,
            unsigned int phsCountDir);
            
void EHRPWMTimebaseSyncDisable(unsigned int baseAddr); 

void EHRPWMTriggerSWSync(unsigned int baseAddr);

void EHRPWMSyncOutModeSet(unsigned int baseAddr, unsigned int syncOutMode);

void EHRPWMWriteTBCount(unsigned int baseAddr, unsigned int tbCount);

unsigned int EHRPWMReadTBCount(unsigned int baseAddr);

unsigned int EHRPWMTBStatusGet(unsigned int baseAddr, unsigned int tbStatusMask);

void EHRPWMTBClearStatus(unsigned int baseAddr, unsigned int tbStatusMask);



/* Counter compare sub-module */
bool EHRPWMLoadCMPA(unsigned int baseAddr, unsigned int CMPAVal,
        bool enableShadowWrite, unsigned int ShadowToActiveLoadTrigger, 
        bool OverwriteShadowFull);

bool EHRPWMLoadCMPB(unsigned int baseAddr, unsigned int CMPBVal,
        bool enableShadowWrite, unsigned int ShadowToActiveLoadTrigger,
        bool OverwriteShadowFull);

/* Action Qualifier sub-module */
void EHRPWMConfigureAQActionOnA(unsigned int baseAddr, unsigned int zero, 
        unsigned int period,
        unsigned int CAUp,  unsigned int CADown,  unsigned int CBUp,
        unsigned int CBDown, unsigned int SWForced);
        
void EHRPWMConfigureAQActionOnB(unsigned int baseAddr, unsigned int zero, 
        unsigned int period,
        unsigned int CAUp,  unsigned int CADown,  unsigned int CBUp,
        unsigned int CBDown, unsigned int SWForced);
        
void EHRPWMSWForceA(unsigned int baseAddr);

void EHRPWMSWForceB(unsigned int baseAddr);

void EHRPWMAQContSWForceOnA(unsigned int baseAddr, unsigned int forceVal, 
        unsigned int activeRegReloadMode);
        
void EHRPWMAQContSWForceOnB(unsigned int baseAddr, unsigned int forceVal, 
        unsigned int activeRegReloadMode);
        

/* Dead Band Generator */
void EHRPWMDBSourceSelect(unsigned int baseAddr, unsigned int DBgenSource);

void EHRPWMDBPolaritySelect(unsigned int baseAddr, unsigned int DBgenPol);

void EHRPWMDBOutput(unsigned int baseAddr, unsigned int DBgenOpMode);

void EHRPWMDBConfigureRED(unsigned int baseAddr, unsigned int raisingEdgeDelay);

void EHRPWMDBConfigureFED(unsigned int baseAddr, unsigned int fallingEdgeDelay);


/* Chopper sub-module */
void EHRPWMConfigureChopperDuty(unsigned int baseAddr, unsigned int dutyCycle);

void EHRPWMConfigureChopperFreq(unsigned int baseAddr, unsigned int freqDiv);

void EHRPWMConfigureChopperOSPW(unsigned int baseAddr, unsigned int OSPWCycles);

void EHRPWMChopperEnable(unsigned int baseAddr);

void EHRPWMChopperDisable(unsigned int baseAddr);


/* Trip-Zone sub-module */
void EHRPWMTZTripEventEnable(unsigned int baseAddr, bool osht_CBC);

void EHRPWMTZTripEventDisable(unsigned int baseAddr, bool osht_CBC);

void EHRPWMTZForceAOnTrip(unsigned int baseAddr, unsigned int opValue);

void EHRPWMTZForceBOnTrip(unsigned int baseAddr, unsigned int opValue);

void EHRPWMTZIntEnable(unsigned int baseAddr, bool osht_CBC);

void EHRPWMTZIntDisable(unsigned int baseAddr, bool osht_CBC);

unsigned int EHRPWMTZFlagGet(unsigned int baseAddr, unsigned int flagToRead);

void EHRPWMTZFlagClear(unsigned int baseAddr, unsigned int flagToClear);

void EHRPWMTZSWFrcEvent(unsigned int baseAddr, bool osht_CBC);


/* Event Trigger sub-module */
void EHRPWMETIntDisable(unsigned int baseAddr);

void EHRPWMETIntEnable(unsigned int baseAddr);

void EHRPWMETIntSourceSelect(unsigned int baseAddr, unsigned int selectInt);

void EHRPWMETIntPrescale(unsigned int baseAddr, unsigned int prescale);

bool EHRPWMETEventCount(unsigned int baseAddr);

bool EHRPWMETIntStatus(unsigned int baseAddr);

void EHRPWMETIntClear(unsigned int baseAddr);

void EHRPWMETIntSWForce(unsigned int baseAddr);


/* HEPWM sub-module */
void EHRPWMLoadTBPHSHR(unsigned int baseAddr, unsigned int TBPHSHRVal);

void EHRPWMLoadCMPAHR(unsigned int baseAddr, unsigned int CMPAHRVal, 
        unsigned int ShadowToActiveLoadTrigger);
        
void EHRPWMConfigHR(unsigned int baseAddr,unsigned int ctrlMode, 
        unsigned int MEPCtrlEdge);
        
void EHRPWMHRDisable(unsigned int baseAddr);

void EHRPWMClockEnable(unsigned int baseAdd);

void EHRPWMClockDisable(unsigned int baseAdd);

unsigned int EHRPWMClockEnableStatusGet(unsigned int baseAdd);

unsigned int EHRPWMClockDisableStatusGet(unsigned int baseAdd);






#ifdef __cplusplus
}
#endif
#endif


