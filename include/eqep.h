/** 
 *   \file  eqep.h
 *
 *   \brief This file contains the Macros and API prototypes for eqep driver
 *
 */


#ifndef _EQEP_H_
#define _EQEP_H_


/* 
 * Requires Headers 
 */
#include "hw_eqep.h"
#include "hw_types.h"


#ifdef __cplusplus
extern "C" {
#endif

/* QDECCTL Decode Control Register */
#define QSP_NO_EFFECT             (0x0000u)
#define QSP_NEGATE                (0x0020u)

#define QIP_NO_EFFECT             (0x0000u)
#define QIP_NEGATE                (0x0040u)

#define QBP_NO_EFFECT             (0x0000u)
#define QBP_NEGATE                (0x0080u)

#define QAP_NO_EFFECT             (0x0000u)
#define QAP_NEGATE                (0x0100u)

#define IGATE_DISABLE             (0x0000u)
#define IGATE_WITH_STROBE         (0x0200u)

#define SWAP_DISABLE              (0x0000u)
#define SWAP_ENABLE               (0x0400u)

#define XCR_X2                    (0x0000u)
#define XCR_X1                    (0x0800u)

#define SPSEL_INDEX               (0x0000u)
#define SPSEL_STROBE              (0x1000u)

#define SOEN_DISABLE              (0x0000u)
#define SOEN_ENABLE               (0x2000u)

#define QSRC_QUAD_MODE            (0x0000u)
#define QSRC_DIR_MODE             (0x4000u)
#define QSRC_UPCNT_MODE           (0x8000u)
#define QSRC_DWNCNT_MODE          (0xC000u)

/* QEPCTL Control Register */
#define WDE_DISABLE               (0x0000u)
#define WDE_ENABLE                (0x0001u)

#define UTE_DISABLE               (0x0000u)
#define UTE_ENABLE                (0x0002u)

#define QCLM_POSCNT               (0x0000u)
#define QCLM_TIME_OUT             (0x0004u)

#define QPHEN_RESET               (0x0000u)
#define QPHEN_ENABLE              (0x0008u)

#define IEL_RISING                (0x0010u)
#define IEL_FALLING               (0x0020u)
#define IEL_SOFTWARE              (0x0030u)

#define SEL_RISING                (0x0000u)
#define SEL_RISING_FALLING        (0x0040u)

#define SWI_DISABLE               (0x0000u)
#define SWI_POSCNT_INIT           (0x0080u)

#define IEI_RISING                (0x0200u)
#define IEI_FALLING               (0x0300u)

#define SEI_RISING                (0x0800u)
#define SEI_RISING_FALLING        (0x0C00u)

#define PCRM_INDEX                (0x0000u)
#define PCRM_POSMAX               (0x1000u)
#define PCRM_FIRST_INDEX          (0x2000u)
#define PCRM_TIME_EVENT           (0x3000u)

#define QEP_EMULATION_SOFT        (0x4000u)
#define QEP_EMULATION_FREE        (0x8000u)

/* QCAPCTL Capture Control Register */
#define UPPS_X1                     (0x0000u)
#define UPPS_X2                     (0x0001u)
#define UPPS_X4                     (0x0002u)
#define UPPS_X8                     (0x0003u)
#define UPPS_X16                    (0x0004u)
#define UPPS_X32                    (0x0005u)
#define UPPS_X64                    (0x0006u)
#define UPPS_X128                   (0x0007u)
#define UPPS_X256                   (0x0008u)
#define UPPS_X512                   (0x0009u)
#define UPPS_X1024                  (0x000Au)
#define UPPS_X2048                  (0x000Bu)

#define UPPS_SHIFT                  (0)        

#define CCPS_X1                     (0x0000u)
#define CCPS_X2                     (0x0010u)
#define CCPS_X4                     (0x0020u)
#define CCPS_X8                     (0x0030u)
#define CCPS_X16                    (0x0040u)
#define CCPS_X32                    (0x0050u)
#define CCPS_X64                    (0x0060u)
#define CCPS_X128                   (0x0070u)

#define CCPS_SHIFT                  (4)      

#define CEN_DISABLE                 (0x007Fu)
#define CEN_ENABLE                  (0x8000u)


/* QPOSCTL Position-Compare Control Register */

#define EQEP_PCE_DISABLE            (0x0000u)
#define EQEP_PCE_ENABLE             (0x1000u)

#define PCPOL_HIGH                  (0x0000u)
#define PCPOL_LOW                   (0x2000u)

#define PCLOAD_ZERO                 (0x0000u)
#define PCLOAD_QPOSCMP              (0x4000u)

#define PCSHDW_DISABLE              (0x0000u)
#define PCSHDW_ENABLE               (0x8000u)

/* QEPSTS Status Register Fields */

#define UPEVNT                      (0x0080u)
#define UPEVNT_SHIFT                (0x0007u)

#define FDF                         (0x0040u)
#define FDF_SHIFT                   (0x0006u)

#define QDF                         (0x0020u)
#define QDF_SHIFT                   (0x0005u)

#define QDLF                        (0x0010u)
#define QDLF_SHIFT                  (0x0004u)

#define COEF                        (0x0008u)
#define COEF_SHIFT                  (0x0003u)

#define CDEF                        (0x0002u)
#define CDEF_SHIFT                  (0x0001u)

#define FIMF                        (0x0001u)
#define FIMF_SHIFT                  (0x0000u)

#define PCEF                        (0x0000u)
#define PCEF_SHIFT                  (0x0000u)

/* EQEP Configuraion and Status Register Fields */

#define QEINT_DISABLED             (0x0000u)    
#define QEINT_CLEAR                (0xFFFFu)

#define QEINT_UTO                  (0x0800u)
#define QEINT_IEL                  (0x0400u)
#define QEINT_SEL                  (0x0200u)
#define QEINT_PCM                  (0x0100u)
#define QEINT_PCR                  (0x0080u)
#define QEINT_PCO                  (0x0040u)
#define QEINT_PCU                  (0x0020u)
#define QEINT_WTO                  (0x0010u)
#define QEINT_QDC                  (0x0008u)
#define QEINT_PHE                  (0x0004u)
#define QEINT_PCE                  (0x0002u)
#define QEINT_INT                  (0x0001u)

/* eqep_config_specific(): config_mode argument */
#define QDECCTL                    0
#define QEPCTL                     1
#define QCAPCTL                    2
#define QPOSCTL                    3

/* eqep_read(): input_mode argument */
#define EQEP_DUAL                  2
#define EQEP_CAP                   1
#define EQEP_STD                   0

#define EQEP_SYSCLK                (100000000)

/* 
 * Driver Datatype Definitions 
 */
 
/**
 * \struct  eqp_device_t
 * \brief   am335x-eqep device registers
 * 
 * This data struct is interface to access the memory mapped registers of the
 * peripheral hardware.
*/
typedef struct
{
    hw_devreg32_t qposcnt;
    hw_devreg32_t qposinit;
    hw_devreg32_t qposmax;
    hw_devreg32_t qposcmp;
    hw_devreg32_t qposilat;
    hw_devreg32_t qposslat;
    hw_devreg32_t qposlat;
    
    hw_devreg32_t qutmr;
    hw_devreg32_t quprd;
    
    hw_devreg16_t qwdtmr;
    hw_devreg16_t qwdprd;
    
    hw_devreg16_t qdecctl;
    hw_devreg16_t qepctl;
    hw_devreg16_t qcapctl;
    hw_devreg16_t qposctl;
    
    hw_devreg16_t qeint;
    hw_devreg16_t qflg;
    hw_devreg16_t qclr;
    hw_devreg16_t qfrc;
    hw_devreg16_t qepsts;
    
    hw_devreg16_t qctmr;
    hw_devreg16_t qcprd;
    hw_devreg16_t qctmrlat;
    hw_devreg16_t qcprdlat;
    
    hw_devreg32_t padding0[6];
    hw_devreg32_t revid;
}
eqep_device_t;


/**
 * \struct  eqep_handle_t
 * \brief   am335x-qep device handle
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
    int device_id;
    
    eqep_device_t  config;
}
eqep_handle_t;


/**
 * \struct  eqep_data_t (v4)
 * \brief   am335x-ehrpwm device handle
 * 
 * This data struct wraps the interface to the device registers of the
 * peripheral hardware.
 * 
*/
typedef struct 
{
    double speed_std;
    double speed_cap;
    
    double speed;
    
    /* configurable */
    double speed_thr;
    unsigned int speed_mode;
    
    unsigned int count[2];
    unsigned int status;
    
    /* configurable */
    unsigned int sampling_freq;
    unsigned int cprd_min;
    unsigned int cap_prescaler; 
    
    /* never change these - only the eqep_read() uses them */
    unsigned int upevnt;
    unsigned int direction;
    unsigned int speed_correction_count;
}
eqep_data_t;



/*
 * Register API Function Definitions: Struct-Handler Model 
 */


/* General Functions */

extern int eqep_open (unsigned int device_id);

extern int eqep_close (unsigned int device_id);

extern int eqep_init (unsigned int device_id);

extern int eqep_config_set (eqep_handle_t *handle);

extern int eqep_config_get (eqep_handle_t *handle);

extern int eqep_read (unsigned int device_id, 
                      unsigned int input_mode, 
                      eqep_data_t volatile *data);
                       
extern int eqep_write (unsigned int device_id, unsigned int value);


/* Specialized functions */

extern int eqep_handle_init (unsigned int device_id, eqep_handle_t *handle);

extern int eqep_data_init (eqep_data_t volatile *data);

extern int eqep_data_copy (eqep_data_t volatile *src, 
                           eqep_data_t volatile *dest);

extern int eqep_status_get (unsigned int device_id, eqep_data_t volatile *data);

extern int eqep_config_specific (eqep_handle_t *handle, unsigned int config_mode);

extern int eqep_capture_enable (unsigned int device_id);

extern int eqep_capture_disable (unsigned int device_id);

extern int eqep_caputure_config (unsigned int device_id, 
                                  unsigned int unit_position,
                                  unsigned int clk_prescaler);

extern int eqep_interrupt_enable (eqep_handle_t *handle, unsigned int int_mode);

extern int eqep_interrupt_disable (eqep_handle_t *handle, unsigned int int_mode);

extern int eqep_interrupt_status_clear (eqep_handle_t *handle, unsigned int int_mode);



#ifdef __cplusplus
}
#endif
#endif

/*
 * EOF
 */
