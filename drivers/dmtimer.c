/**
 *  \file   dmtimer.c
 *
 *  \brief  DMTIMER APIs.
 *
 *   This file contains the device abstraction layer APIs for DMTimer.
 */


/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include "hw_types.h"
#include "dmtimer.h"
#include "beaglebone.h"
#include "soc_AM335x.h"

/*
 * Driver Definitions
 */
 
#define CAPT_REG_1                         (1u)
#define CAPT_REG_2                         (2u)


/*
 * Driver Private Variables
 */

static dmtimer_device_t volatile *const dmtimer[6] = 
                                            
                                        {
                                            (dmtimer_device_t *)SOC_DMTIMER_2_REGS,
                                            (dmtimer_device_t *)SOC_DMTIMER_3_REGS,
                                            (dmtimer_device_t *)SOC_DMTIMER_4_REGS,
                                            (dmtimer_device_t *)SOC_DMTIMER_5_REGS,
                                            (dmtimer_device_t *)SOC_DMTIMER_6_REGS,
                                            (dmtimer_device_t *)SOC_DMTIMER_7_REGS,
                                        };

//! must also add repsective pointers to RTC and TIMER_1MS


/*
 * Register API Function Definitions: Struct-Handler Model 
 */

/** NOTE: the current version utilizes the pre-existing direct-access functions
 *       in place of most of the private function facilities.
 */

/* Private Driver Function Declarations */

static void 
_dmtimer_clk_enable (unsigned int device_id);


/* General Functions */

int 
dmtimer_open (unsigned int device_id)
{
    unsigned int device_index = device_id - 2;
    
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
    
    _dmtimer_clk_enable (device_id);
    
    DMTimerDisable((unsigned int)dmtimer[device_index]);
    
    return 0;
}

int 
dmtimer_close (unsigned int device_id)
{
    unsigned int device_index = device_id - 2;
    
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
 
    device_id = device_id - 2;
    
    //TODO
    
    return 0;
}

int 
dmtimer_init (unsigned int device_id)
{
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
 
    device_id = device_id - 2;
    
    unsigned int reload_ticks = DMTIMER_1MS;
    
    DMTimerModeConfigure((unsigned int)dmtimer[device_id], DMTIMER_AUTORLD_NOCMP_ENABLE);
    
    DMTimerCounterSet((unsigned int)dmtimer[device_id], reload_ticks);
    
    DMTimerReloadSet((unsigned int)dmtimer[device_id], reload_ticks);
    
    return 0;
}

int 
dmtimer_enable (unsigned int device_id)
{
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
 
    device_id = device_id - 2;
    
    DMTimerEnable((unsigned int)dmtimer[device_id]);
    
    return 0;
}

int 
dmtimer_disable (unsigned int device_id)
{
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
 
    device_id = device_id - 2;
    
    DMTimerDisable((unsigned int)dmtimer[device_id]);
    
    return 0;
}

int 
dmtimer_counter_reset (unsigned int device_id)
{
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
 
    device_id = device_id - 2;
    
    unsigned int reset = DMTimerReloadGet((unsigned int)dmtimer[device_id]);
    
    DMTimerCounterSet((unsigned int)dmtimer[device_id], reset);
    
    return 0;
}

int 
dmtimer_config_set (dmtimer_handle_t *handle)
{
    if (handle == NULL)
    {
        return -1;
    }
    
    unsigned int device_id = handle->device_id - 2;
    
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
    
    //!
    
    return 0;
}

int 
dmtimer_config_get (dmtimer_handle_t *handle)
{
    if (handle == NULL)
    {
        return -1;
    }
    
    unsigned int device_id = handle->device_id - 2;
    
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
    
    //!
    
    return 0;
}


/* Specialized functions */

int 
dmtimer_handle_init (unsigned int device_id, dmtimer_handle_t *handle)
{
    if (handle == NULL)
    {
        return -1;
    }
    
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
    
    handle->device_id = device_id;
    
    handle->config.tidr = 0;
    handle->config.tiocp_cfg = 0;
    handle->config.irq_eoi = 0;
    handle->config.irqstatus_raw = 0;
    handle->config.irqstatus = 0;
    handle->config.irqenable_set = 0;
    handle->config.irqenable_clr = 0;
    handle->config.irqwakeen = 0;
    handle->config.tclr = 0;
    handle->config.tcrr = 0;
    handle->config.tldr = 0;
    handle->config.ttgr = 0;
    handle->config.twps = 0;
    handle->config.tmar = 0;
    handle->config.tcar1 = 0;
    handle->config.tsicr = 0;
    handle->config.tcar2 = 0;
    
    return 0;
}

int 
dmtimer_tldr_config_set (unsigned int device_id, unsigned int load_value)
{
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
    
    device_id = device_id - 2;
    
    DMTimerReloadSet((unsigned int)dmtimer[device_id], load_value);
    
    return 0;
}

unsigned int 
dmtimer_tldr_config_get (unsigned int device_id)
{
    if ((device_id > 7) || (device_id < 2))
    {
        return -1;
    }
    
    device_id = device_id - 2;
    
    return DMTimerReloadGet((unsigned int)dmtimer[device_id]);
}


/* Private Driver Function Definitions */

static void 
_dmtimer_clk_enable (unsigned int device_id)
{
    if (device_id == 2)
    {
        DMTimer2ModuleClkConfig();
    }
    
    else if (device_id == 3)
    {
        DMTimer3ModuleClkConfig();
    }
    
    else if (device_id == 4)
    {
        DMTimer4ModuleClkConfig();
    }
    
    else if (device_id == 5)
    {
        DMTimer5ModuleClkConfig();
    }
    
    else
    {
        ;
    }
}


/*
 * Register API Function Definitions: Direct-Acces Model
 */
 
/**
 * \brief   This API will start the timer. 
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  None.
 *
 * \note    The timer must be configured before it is started/enabled.
 *
 **/
void DMTimerEnable(unsigned int baseAdd)
{
    /* Start the timer */
    HWREG(baseAdd + DMTIMER_TCLR) |= DMTIMER_TCLR_ST;
}

/**
 * \brief   This API will stop the timer. 
 *
 * \param   baseAdd      Base Address of the DMTimer Module Register.
 *
 * \return  None.
 *
 **/
void DMTimerDisable(unsigned int baseAdd)
{
    /* Stop the timer */
    HWREG(baseAdd + DMTIMER_TCLR) &= ~DMTIMER_TCLR_ST;
}

/**
 * \brief   This API will configure the timer in combinations of 
 *          'One Shot timer' and 'Compare' Mode or 'Auto-reload timer' 
 *          and 'Compare' Mode.  
 *
 * \param   baseAdd      Base Address of the DMTimer Module Register.
 * \param   timerMode    Mode for enabling the timer.
 *
 * 'timerMode' can take the following values \n
 *    DMTIMER_ONESHOT_CMP_ENABLE - One shot and compare mode enabled \n
 *    DMTIMER_ONESHOT_NOCMP_ENABLE - One shot enabled, compare mode disabled \n
 *    DMTIMER_AUTORLD_CMP_ENABLE - Auto-reload and compare mode enabled \n
 *    DMTIMER_AUTORLD_NOCMP_ENABLE - Auto-reload enabled, compare mode 
 *                                   disabled \n
 *
 * \return  None.
 *
 **/
void DMTimerModeConfigure(unsigned int baseAdd, unsigned int timerMode)
{
    /* Clear the AR and CE field of TCLR */
    HWREG(baseAdd + DMTIMER_TCLR) &= ~(DMTIMER_TCLR_AR | DMTIMER_TCLR_CE);

    /* Set the timer mode in TCLR register */
    HWREG(baseAdd + DMTIMER_TCLR) |= (timerMode & (DMTIMER_TCLR_AR | 
                                                   DMTIMER_TCLR_CE));
}

/**
 * \brief   This API will configure and enable the pre-scaler clock.
 *
 * \param   baseAdd      Base Address of the DMTimer Module Register.
 * \param   ptv          Pre-scale clock Timer value.
 *
 * 'ptv' can take the following values \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_2 - Timer clock divide by 2 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_4 - Timer clock divide by 4 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_8 - Timer clock divide by 8 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_16 - Timer clock divide by 16 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_32 - Timer clock divide by 32 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_64 - Timer clock divide by 64 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_128 - Timer clock divide by 128 \n
 *    DMTIMER_PRESCALER_CLK_DIV_BY_256 - Timer clock divide by 256 \n
 *
 * \return  None.
 *
 **/
void DMTimerPreScalerClkEnable(unsigned int baseAdd, unsigned int ptv)
{
    /* Clear the PTV field of TCLR */
    HWREG(baseAdd + DMTIMER_TCLR) &= ~DMTIMER_TCLR_PTV;

    /* Set the PTV field and enable the pre-scaler clock */
    HWREG(baseAdd + DMTIMER_TCLR) |= (ptv & (DMTIMER_TCLR_PTV | DMTIMER_TCLR_PRE));
}

/**
 * \brief   This API will disable the pre-scaler clock.
 *
 * \param   baseAdd      Base Address of the DMTimer Module Register.
 *
 * \return  None.
 *
 **/
void DMTimerPreScalerClkDisable(unsigned int baseAdd)
{
    /* Disable Pre-scaler clock */
    HWREG(baseAdd + DMTIMER_TCLR) &= ~DMTIMER_TCLR_PRE;
}

/**
 * \brief   Set/Write the Counter register with the counter value.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   counter       Count value for the timer.
 *
 * \return  None.
 *
 * \note    Value can be loaded into the counter register when the counter is 
 *          stopped or when it is running.
 **/
void DMTimerCounterSet(unsigned int baseAdd, unsigned int counter)
{
    /* Set the counter value */
    HWREG(baseAdd + DMTIMER_TCRR) = counter;
}

/**
 * \brief   Get/Read the counter value from the counter register.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  This API returns the count value present in the DMTimer Counter
 *          register.
 *
 * \note:   Value can be read from the counter register when the counter is
 *          stopped or when it is running.
 **/
unsigned int DMTimerCounterGet(unsigned int baseAdd)
{
    /* Read the counter value from TCRR */
    return (HWREG(baseAdd + DMTIMER_TCRR));
}

/**
 * \brief   Set the reload count value in the timer load register.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   reload        The reload count value of the timer.
 *
 * \return  None.
 *
 * \note:   It is recommended to not use reload value as 0xFFFFFFFF as it can 
 *          lead to undesired results.
 **/
void DMTimerReloadSet(unsigned int baseAdd, unsigned int reload)
{
    /* Load the register with the re-load value */
    HWREG(baseAdd + DMTIMER_TLDR) = reload;
}

/**
 * \brief   Get the reload count value from the timer load register.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  This API returns the value present in DMTimer Load Register.
 *
 **/
unsigned int DMTimerReloadGet(unsigned int baseAdd)
{
    /* Return the contents of TLDR */
    return (HWREG(baseAdd + DMTIMER_TLDR));
}

/**
 * \brief   Configure general purpose output pin.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   gpoCfg        General purpose output.
 * 
 * 'gpoCfg' can take the following values \n
 *    DMTIMER_GPO_CFG_0 - PORGPOCFG drives 0 \n
 *    DMTIMER_GPO_CFG_1 - PORGPOCFG drives 1 \n  
 *
 * \return  None.
 *
 **/
void DMTimerGPOConfigure(unsigned int baseAdd, unsigned int gpoCfg)
{
    /* Clear the GPO_CFG field of TCLR */
    HWREG(baseAdd + DMTIMER_TCLR) &= ~DMTIMER_TCLR_GPO_CFG;

    /* Write to the gpoCfg field of TCLR */
    HWREG(baseAdd + DMTIMER_TCLR) |= (gpoCfg & DMTIMER_TCLR_GPO_CFG);
}

/**
 * \brief   Set the match register with the compare value.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   compareVal    Compare value.
 *
 * \return  None.
 *
 **/
void DMTimerCompareSet(unsigned int baseAdd, unsigned int compareVal)
{
    /* Write the compare value to TMAR */
    HWREG(baseAdd + DMTIMER_TMAR) = compareVal;
}

/**
 * \brief   Get the match register contents.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  This API returns the match register contents.
 *
 **/
unsigned int DMTimerCompareGet(unsigned int baseAdd)
{
    /* Return the TMAR value */
    return (HWREG(baseAdd + DMTIMER_TMAR));
}

/**
 * \brief   Trigger IRQ event by software.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   intFlags      Variable used to trigger the events.
 *
 * 'intFlags' can take the following values \n
 *    DMTIMER_INT_TCAR_IT_FLAG - Trigger IRQ status for capture \n
 *    DMTIMER_INT_OVF_IT_FLAG - Trigger IRQ status for overflow \n
 *    DMTIMER_INT_MAT_IT_FLAG - Trigger IRQ status for match \n
 *
 * \return  None.
 *
 **/
void DMTimerIntRawStatusSet(unsigned int baseAdd, unsigned int intFlags)
{
    /* Trigger the events in IRQSTATUS_RAW register */
    HWREG(baseAdd + DMTIMER_IRQSTATUS_RAW) = (intFlags & 
                                           (DMTIMER_IRQSTATUS_RAW_MAT_IT_FLAG |
                                            DMTIMER_IRQSTATUS_RAW_OVF_IT_FLAG |
                                            DMTIMER_IRQSTATUS_RAW_TCAR_IT_FLAG));
}

/**
 * \brief   Read the status of IRQSTATUS_RAW register.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  This API returns the contents of IRQSTATUS_RAW register.
 *
 **/
unsigned int DMTimerIntRawStatusGet(unsigned int baseAdd)
{
    /* Return the status of IRQSTATUS_RAW register */
    return (HWREG(baseAdd + DMTIMER_IRQSTATUS_RAW));
}

/**
 * \brief   Read the status of IRQ_STATUS register.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  This API returns the status of IRQSTATUS register.
 *
 **/
unsigned int DMTimerIntStatusGet(unsigned int baseAdd)
{
    /* Return the status of IRQSTATUS register */
    return (HWREG(baseAdd + DMTIMER_IRQSTATUS));
}

/**
 * \brief   Clear the status of interrupt events.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   intFlags      Variable used to clear the events.
 *
 * 'intFlags' can take the following values \n
 *    DMTIMER_INT_TCAR_IT_FLAG - Clear IRQ status for capture \n
 *    DMTIMER_INT_OVF_IT_FLAG - Clear IRQ status for overflow \n
 *    DMTIMER_INT_MAT_IT_FLAG - Clear IRQ status for match \n
 *
 * \return  None.
 *
 **/
void DMTimerIntStatusClear(unsigned int baseAdd, unsigned int intFlags)
{
    /* Clear the interrupt status from IRQSTATUS register */
    HWREG(baseAdd + DMTIMER_IRQSTATUS) = (intFlags & 
                                         (DMTIMER_IRQSTATUS_TCAR_IT_FLAG | 
                                          DMTIMER_IRQSTATUS_OVF_IT_FLAG | 
                                          DMTIMER_IRQSTATUS_MAT_IT_FLAG));
}

/**
 * \brief   Enable the DMTimer interrupts.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   intFlags      Variable used to enable the interrupts.
 *
 * 'intFlags' can take the following values \n
 *    DMTIMER_INT_TCAR_EN_FLAG - IRQ enable for capture \n
 *    DMTIMER_INT_OVF_EN_FLAG - IRQ enable for overflow \n
 *    DMTIMER_INT_MAT_EN_FLAG - IRQ enable for match \n
 *
 * \return  None.
 *
 **/
void DMTimerIntEnable(unsigned int baseAdd, unsigned int intFlags)
{
    /* Enable the DMTimer interrupts represented by intFlags */
    HWREG(baseAdd + DMTIMER_IRQENABLE_SET) = (intFlags & 
                                           (DMTIMER_IRQENABLE_SET_TCAR_EN_FLAG |
                                            DMTIMER_IRQENABLE_SET_OVF_EN_FLAG | 
                                            DMTIMER_IRQENABLE_SET_MAT_EN_FLAG));
}

/**
 * \brief   Disable the DMTimer interrupts.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   intFlags      Variable used to disable the interrupts.
 *
 * 'intFlags' can take the following values \n
 *    DMTIMER_INT_TCAR_EN_FLAG - IRQ disable for capture \n
 *    DMTIMER_INT_OVF_EN_FLAG - IRQ disable for overflow \n
 *    DMTIMER_INT_MAT_EN_FLAG - IRQ disable for match \n
 *
 * \return  None.
 *
 **/
void DMTimerIntDisable(unsigned int baseAdd, unsigned int intFlags)
{
    /* Disable the DMTimer interrupts represented by intFlags */
    HWREG(baseAdd + DMTIMER_IRQENABLE_CLR) = (intFlags &
                                           (DMTIMER_IRQENABLE_CLR_TCAR_EN_FLAG |
                                            DMTIMER_IRQENABLE_CLR_OVF_EN_FLAG |
                                            DMTIMER_IRQENABLE_CLR_MAT_EN_FLAG));
}

/**
 * \brief   Set/enable the trigger write access.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  None.
 *
 * \note    When we have enabled the timer in Auto-reload mode, the value from 
 *          TLDR is reloaded into TCRR when a overflow condition occurs. But if 
 *           we want to load the contents from TLDR to TCRR before overflow 
 *          occurs then call this API.
 **/
void DMTimerTriggerSet(unsigned int baseAdd)
{
    /* Write a value to the register */
    HWREG(baseAdd + DMTIMER_TTGR) = DMTIMER_TTGR_TTGR_VALUE;
}

/**
 * \brief   Read the status of IRQENABLE_SET register.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  This API returns the status of IRQENABLE_SET register.
 *
 **/
unsigned int DMTimerIntEnableGet(unsigned int baseAdd)
{
    /* Return the status of register IRQENABLE_SET */
    return (HWREG(baseAdd + DMTIMER_IRQENABLE_SET));
}

/**
 * \brief   Enable/Disable software reset for DMTimer module.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   rstOption     Enable/Disable reset option for DMTimer.
 *          
 * 'rstOption' can take the following values \n
 *    DMTIMER_SFT_RESET_ENABLE - Software reset is enabled \n
 *    DMTIMER_SFT_RESET_DISABLE - Software reset is disabled \n   
 *
 * \return  None.
 *
 **/
void DMTimerResetConfigure(unsigned int baseAdd, unsigned int rstOption)
{
    /* Clear the SFT field of TSICR */
    HWREG(baseAdd + DMTIMER_TSICR) &= ~DMTIMER_TSICR_SFT;

    /* Write the option sent by user to SFT field of TSICR */
    HWREG(baseAdd + DMTIMER_TSICR) |= (rstOption & DMTIMER_TSICR_SFT);
}

/**
 * \brief   Reset the DMTimer module.
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 *
 * \return  None.
 *
 **/
void DMTimerReset(unsigned int baseAdd)
{
    /* Reset the DMTimer module */
    HWREG(baseAdd + DMTIMER_TIOCP_CFG) |= DMTIMER_TIOCP_CFG_SOFTRESET;

    while(DMTIMER_TIOCP_CFG_SOFTRESET == (HWREG(baseAdd + DMTIMER_TIOCP_CFG)
                                          & DMTIMER_TIOCP_CFG_SOFTRESET));
}

/**
 * \brief   This API stores the context of the DMTimer
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   contextPtr    Pointer to the structure where the DM timer context
 *                        need to be saved.
 *
 * \return  None.
 *
 **/
void DMTimerContextSave(unsigned int baseAdd, dmtimerContext *contextPtr)
{
    contextPtr->tldr = HWREG(baseAdd + DMTIMER_TLDR);
    contextPtr->tmar = HWREG(baseAdd + DMTIMER_TMAR);
    contextPtr->irqenableset = HWREG(baseAdd + DMTIMER_IRQENABLE_SET);
    contextPtr->tcrr = HWREG(baseAdd + DMTIMER_TCRR);
    contextPtr->tclr = HWREG(baseAdd + DMTIMER_TCLR);
}

/**
 * \brief   This API restores the context of the DMTimer
 *
 * \param   baseAdd       Base Address of the DMTimer Module Register.
 * \param   contextPtr    Pointer to the structure where the DM timer context
 *                        need to be restored from.
 *
 * \return  None.
 *
 **/
void DMTimerContextRestore(unsigned int baseAdd,  dmtimerContext *contextPtr)
{
    HWREG(baseAdd + DMTIMER_TLDR) = contextPtr->tldr;
    HWREG(baseAdd + DMTIMER_TMAR) = contextPtr->tmar;
    HWREG(baseAdd + DMTIMER_IRQENABLE_SET) = contextPtr->irqenableset;
    HWREG(baseAdd + DMTIMER_TCRR) = contextPtr->tcrr;
    HWREG(baseAdd + DMTIMER_TCLR) = contextPtr->tclr;
}

/**************************** END OF DMTIMER DAL ******************************/








