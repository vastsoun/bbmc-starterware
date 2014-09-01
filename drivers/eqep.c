/**
 *  \file   eqep.c
 *
 *  \brief  This file contains the device abstraction layer APIs for EQEP.
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




/*
 * Driver Includes
 */

/* Driver Interface Header */
#include "eqep.h"
#include "pwmss.h"

/* Hardware headers */
#include "soc_AM335x.h"
#include "hw_control_AM335x.h"
#include "hw_pwmss.h"
#include "hw_cm_per.h"


/*
 * Driver Definitions
 */

#define QPOSINIT_DEFAULT                10
#define QPOSMAX_DEFAULT                 2500000
#define QUPRD_DEFAULT                   2000
#define QWDPRD_DEFAULT                  0
#define QDECCTL_DEFAULT                 (XCR_X1 + QSRC_QUAD_MODE)
#define QEPCTL_DEFAULT                  (PCRM_POSMAX + \
                                        SWI_POSCNT_INIT + \
                                        QPHEN_ENABLE + \
                                        UTE_DISABLE + \
                                        QCLM_TIME_OUT)
#define QCAPCTL_DEFAULT                 (UPPS_X1 + CCPS_X64 + CEN_ENABLE)
#define QPOSCTL_DEFAULT                 EQEP_PCE_DISABLE
#define QEINT_DEFAULT                   QEINT_DISABLED

#define EQEP_CAPTMR_MAX                 65535
#define EQEP_CAP_SPEED_COUNT            15


/* 
 * Driver Private Data
 */

static eqep_device_t volatile *const eqep[3] = {
                                                     (eqep_device_t *)SOC_EQEP_0_REGS,
                                                     (eqep_device_t *)SOC_EQEP_1_REGS,
                                                     (eqep_device_t *)SOC_EQEP_2_REGS
                                                    };


/*
 * Driver Private Function Declarations
 */

static unsigned int _eqep_status_get (unsigned int device_id);


static void _eqep_input_counts_get (unsigned int device_id, 
                                     eqep_data_t volatile *data);
                                  
static void _eqep_input_counts_set (unsigned int device_id, 
                                     unsigned int value);                                  

static void _eqep_input_speed_std (unsigned int device_id, 
                                    eqep_data_t volatile *data);

static void _eqep_input_speed_cap (unsigned int device_id, 
                                    eqep_data_t volatile *data);

static unsigned int _dec32_to_log2 (unsigned int value);

/*
 * Driver Public Functions
 */

/* General Functions */

int eqep_open (unsigned int device_id)
{
    int ret = 0;
    int test = 0;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    test = L3L4_clock_domain_open();
    
    if (test != 0)
    {
        ret = 1;
    }
    
    test = pwmss_clk_enable(device_id);
    
    if (test != 0)
    {
        ret = 2;
    }
    
    test = pwmss_eqep_clk_enable(device_id);
    
    if (test != 0)
    {
        ret = 3;
    }
    
    return ret;
}


int eqep_close (unsigned int device_id)
{
    int ret = 0;
    int test = 0;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    test = pwmss_tbclk_disable(device_id);
    
    if (test != 0)
    {
        ret = 1;
    }
    
    test = pwmss_clk_disable(device_id);
    
    if (test != 0)
    {
        ret = 1;
    }
    
    test = pwmss_eqep_clk_disable(device_id);
    
    if (test != 0)
    {
        ret = 1;
    }
    
    return ret;
}


int eqep_init (unsigned int device_id)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    eqep[device_id]->qposcnt  = 0;
    eqep[device_id]->qposinit = QPOSINIT_DEFAULT;
    eqep[device_id]->qposmax  = QPOSMAX_DEFAULT;
    eqep[device_id]->qposcmp  = 0;
    
    eqep[device_id]->qutmr    = 0;
    eqep[device_id]->quprd    = QUPRD_DEFAULT;
    
    eqep[device_id]->qwdtmr   = 0;
    eqep[device_id]->qwdprd   = QWDPRD_DEFAULT;
    
    eqep[device_id]->qdecctl  = QDECCTL_DEFAULT;
    eqep[device_id]->qepctl   = QEPCTL_DEFAULT;
    eqep[device_id]->qcapctl  = QCAPCTL_DEFAULT;
    eqep[device_id]->qposctl  = QPOSCTL_DEFAULT;
    
    eqep[device_id]->qeint    = QEINT_DEFAULT;
    eqep[device_id]->qclr     = QEINT_CLEAR;
    
    eqep[device_id]->qctmr    = 0;
    eqep[device_id]->qcprd    = 0;
    
    return 0;
}


int eqep_config_set (eqep_handle_t *handle)
{
    unsigned int device_id = handle->device_id;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    eqep[device_id]->qposcnt  = handle->config.qposcnt;
    eqep[device_id]->qposinit = handle->config.qposinit;
    eqep[device_id]->qposmax  = handle->config.qposmax;
    eqep[device_id]->qposcmp  = handle->config.qposcmp;
    
    eqep[device_id]->qutmr    = handle->config.qutmr;
    eqep[device_id]->quprd    = handle->config.quprd;
    
    eqep[device_id]->qwdtmr   = handle->config.qwdtmr;
    eqep[device_id]->qwdprd   = handle->config.qwdprd;
    
    eqep[device_id]->qdecctl  = handle->config.qdecctl;
    eqep[device_id]->qepctl   = handle->config.qepctl;
    eqep[device_id]->qcapctl  = handle->config.qcapctl;
    eqep[device_id]->qposctl  = handle->config.qposctl;
    
    eqep[device_id]->qeint    = handle->config.qeint;
    eqep[device_id]->qclr     = handle->config.qclr;
    
    eqep[device_id]->qctmr    = handle->config.qctmr;
    eqep[device_id]->qcprd    = handle->config.qcprd;
    
    return 0;
}


int eqep_config_get (eqep_handle_t *handle)
{
    unsigned int device_id = handle->device_id;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    handle->config.qposcnt  = eqep[device_id]->qposcnt;
    handle->config.qposinit = eqep[device_id]->qposinit;
    handle->config.qposmax  = eqep[device_id]->qposmax;
    handle->config.qposcmp  = eqep[device_id]->qposcmp;
    
    handle->config.qutmr    = eqep[device_id]->qutmr;
    handle->config.quprd    = eqep[device_id]->quprd;
    
    handle->config.qwdtmr   = eqep[device_id]->qwdtmr;
    handle->config.qwdprd   = eqep[device_id]->qwdprd;
    
    handle->config.qdecctl  = eqep[device_id]->qdecctl;
    handle->config.qepctl   = eqep[device_id]->qepctl;
    handle->config.qcapctl  = eqep[device_id]->qcapctl;
    handle->config.qposctl  = eqep[device_id]->qposctl;
    
    handle->config.qeint    = eqep[device_id]->qeint;
    handle->config.qclr     = eqep[device_id]->qclr;
    
    handle->config.qctmr    = eqep[device_id]->qctmr;
    handle->config.qcprd    = eqep[device_id]->qcprd;
    
    return 0;
}


int 
eqep_read (unsigned int device_id, 
           unsigned int input_mode, 
           eqep_data_t volatile *data)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    if (data == NULL)
    {
        return -3;
    }
    
    if (input_mode == EQEP_DUAL)
    {
        data->count[0] = data->count[1];
        _eqep_input_counts_get(device_id, data);
        
        _eqep_input_speed_std(device_id, data);
        
        data->status = _eqep_status_get(device_id);
        _eqep_input_speed_cap(device_id, data);
    }
    
    else if (input_mode == EQEP_CAP)
    {
        data->count[0] = data->count[1];
        _eqep_input_counts_get(device_id, data);
        
        data->status = _eqep_status_get(device_id);
        _eqep_input_speed_cap(device_id, data);
    }
    
    else if (input_mode == EQEP_STD)
    {
        data->count[0] = data->count[1];
        _eqep_input_counts_get(device_id, data);
        
        data->status = _eqep_status_get(device_id);
        _eqep_input_speed_std(device_id, data);
    }
    
    else
    {
        return -2;
    }
    
    return 0;
}


int 
eqep_write (unsigned int device_id, unsigned int value)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    _eqep_input_counts_set(device_id, value);
    
    return 0;
}


/* Specialized functions */

int
eqep_handle_init (unsigned int device_id, eqep_handle_t *handle)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    handle->device_id = device_id;
    
    handle->config.qposcnt   = 0;
    handle->config.qposinit  = 0;
    handle->config.qposmax   = 0;
    handle->config.qposcmp   = 0;
    handle->config.qposilat  = 0;
    handle->config.qposslat  = 0;
    handle->config.qposlat   = 0;
    
    handle->config.qutmr     = 0;
    handle->config.quprd     = 0;
    
    handle->config.qwdtmr    = 0;
    handle->config.qwdprd    = 0;
    
    handle->config.qdecctl   = 0;
    handle->config.qepctl    = 0;
    handle->config.qcapctl   = 0;
    handle->config.qposctl   = 0;
    
    handle->config.qeint     = 0;
    handle->config.qflg      = 0;
    handle->config.qclr      = 0;
    handle->config.qfrc      = 0;
    handle->config.qepsts    = 0;
    
    handle->config.qctmr     = 0;
    handle->config.qcprd     = 0;
    handle->config.qctmrlat  = 0;
    handle->config.qcprdlat  = 0;
    
    return 0;
}


int 
eqep_data_init (eqep_data_t volatile *data)
{
    if (data == NULL)
    {
        return -1;
    }
    
    data->speed_std = 0;
    data->speed_cap = 0;
    
    data->status = 0;
    
    data->upevnt = 0;
    data->direction = 0;
    data->speed_correction_count = 0;
    
    data->speed_mode = 0;
    
    return 0;
}


int 
eqep_data_copy (eqep_data_t volatile *src, eqep_data_t volatile *dest)
{
    if (src == NULL)
    {
        return -1;
    }
    
    if (dest == NULL)
    {
        return -2;
    }
    
    dest->speed_std              = src->speed_std;
    dest->speed_cap              = src->speed_cap;
    
    dest->speed                  = src->speed;
    
    dest->speed_thr              = src->speed_thr;
    dest->speed_mode             = src->speed_mode;
    
    dest->count[0]               = src->count[0];
    dest->count[1]               = src->count[1];
    dest->status                 = src->status;
    
    dest->sampling_freq          = src->sampling_freq;
    dest->cprd_min               = src->cprd_min;
    dest->cap_prescaler          = src->cap_prescaler;
    
    dest->upevnt                 = src->upevnt;
    dest->direction              = src->direction;
    dest->speed_correction_count = src->speed_correction_count;
    
    dest->speed_mode             = src->speed_mode;
    
    return 0;
}


int 
eqep_status_get(unsigned int device_id, eqep_data_t volatile *data)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    if (data == NULL)
    {
        return -2;
    }
    
    data->status = _eqep_status_get(device_id);
    
    return 0;
}

int 
eqep_config_specific(eqep_handle_t *handle, unsigned int config_mode)
{
     unsigned int device_id = handle->device_id;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    if (config_mode == QDECCTL)
    {
        eqep[device_id]->qdecctl = handle->config.qdecctl;
    }
    
    else if (config_mode == QEPCTL)
    {
        eqep[device_id]->qepctl = handle->config.qepctl;
    }
    
    else if (config_mode == QCAPCTL)
    {
        eqep[device_id]->qcapctl = handle->config.qcapctl;
    }
    
    else if (config_mode == QPOSCTL)
    {
        eqep[device_id]->qposctl = handle->config.qposctl;
    }
    
    else
    {
        return -3;
    }
    
    return 0;
}


int eqep_capture_enable (unsigned int device_id)
{
    unsigned int temp;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    temp = eqep[device_id]->qcapctl;
    temp &= CEN_DISABLE; 
    temp |= CEN_ENABLE;
    
    eqep[device_id]->qcapctl = temp;
    
    return 0;
}


int eqep_capture_disable (unsigned int device_id)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    eqep[device_id]->qcapctl &= CEN_DISABLE;
    
    return 0;
}


int
eqep_caputure_config (unsigned int device_id, 
                      unsigned int unit_position,
                      unsigned int clk_prescaler)
{
    if (device_id > 2)
    {
        return -1;
    }
    
    if ((unit_position > 2048) || (unit_position <= 0))
    {
        return -2;
    }
    
    if ((clk_prescaler > 128) || (clk_prescaler <= 0))
    {
        return -3;
    }
    
    eqep_capture_disable(device_id);
    
    unit_position = _dec32_to_log2(unit_position);
    clk_prescaler = _dec32_to_log2(clk_prescaler);
    
    clk_prescaler = clk_prescaler << CCPS_SHIFT;
    
    eqep[device_id]->qcapctl = clk_prescaler | unit_position;
    
    eqep_capture_enable(device_id);
    
    return 0;
}


int 
eqep_interrupt_enable(eqep_handle_t *handle, unsigned int int_mode)
{
    unsigned int device_id = handle->device_id;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    eqep[device_id]->qeint |= int_mode;
    
    return 0;
}


int 
eqep_interrupt_disable(eqep_handle_t *handle, unsigned int int_mode)
{
    unsigned int device_id = handle->device_id;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    eqep[device_id]->qeint &= (~int_mode & 0x0FFEu);
    
    return 0;
}


int 
eqep_interrupt_status_clear(eqep_handle_t *handle, unsigned int int_mode)
{
    unsigned int device_id = handle->device_id;
    
    if (device_id > 2)
    {
        return -1;
    }
    
    if (handle == NULL)
    {
        return -2;
    }
    
    eqep[device_id]->qclr |= int_mode;
    
    return 0;
}


/*
 * Driver Private Function Definitions
 */

static unsigned int 
_eqep_status_get (unsigned int device_id)
{
    return eqep[device_id]->qepsts;
}

static void
_eqep_input_counts_get (unsigned int device_id, eqep_data_t volatile *data)
{
    data->count[1] = eqep[device_id]->qposcnt; 
}

static void
_eqep_input_counts_set (unsigned int device_id, unsigned int value)
{
    eqep[device_id]->qposcnt = value; 
}

static void
_eqep_input_speed_std(unsigned int device_id, eqep_data_t volatile *data)
{
    double count[2];
    
    count[0] = data->count[0];
    count[1] = data->count[1];
    
    data->speed_std = (count[1] - count[0]) * (data->sampling_freq);
}

static void 
_eqep_input_speed_cap (unsigned int device_id, eqep_data_t volatile *data)
{
    unsigned int upevnt;
    unsigned int direction;
    unsigned int speed_correction_count;
    unsigned int status;
    unsigned int cprd;
    unsigned int qdc;
    double speed_previous;
    
    status = eqep[device_id]->qepsts;
    cprd = eqep[device_id]->qcprd;
    
    upevnt = data->upevnt;
    direction = data->direction;
    speed_correction_count = data->speed_correction_count;
    
    speed_previous = data->speed_cap;
    
    direction += ((status & (EQEP_QEPSTS_QDF)) >> EQEP_QEPSTS_QDF_SHIFT);
    qdc = direction % 2;
    direction = ((status & (EQEP_QEPSTS_QDF)) >> EQEP_QEPSTS_QDF_SHIFT);
    
    if(((status & EQEP_QEPSTS_UPEVNT) >> EQEP_QEPSTS_UPEVNT_SHIFT))
    {
       eqep[device_id]->qepsts |= EQEP_QEPSTS_UPEVNT;
        
        if(((status & EQEP_QEPSTS_COEF) >> EQEP_QEPSTS_COEF_SHIFT))
        {
            eqep[device_id]->qepsts |= EQEP_QEPSTS_COEF;
            
            if(!qdc)
            {
                data->speed_cap = data->cap_prescaler / EQEP_CAPTMR_MAX;
                
                if(!direction)
                {
                    data->speed_cap = -1 * data->speed_cap;
                }
            }
            
            eqep[device_id]->qctmr = 0;
        }
        
        else
        {
            if(!qdc)
            {
                if(cprd >= data->cprd_min)
                {
                    if(!upevnt)
                    {
                        cprd = EQEP_CAPTMR_MAX;
                    }
                    
                    upevnt = 1;
                    data->speed_cap = data->cap_prescaler / cprd;
                    
                    if(!direction)
                    {
                        data->speed_cap = -1 * data->speed_cap;
                    }
                }
            }
        }
    }
    
    else
    {
        if(((status & (EQEP_QEPSTS_COEF)) >> EQEP_QEPSTS_COEF_SHIFT))
        {
            eqep[device_id]->qepsts |= (EQEP_QEPSTS_COEF);
            
            if(!qdc)
            {
                if(upevnt)
                {
                    upevnt = 0;
                    data->speed_cap = data->cap_prescaler / EQEP_CAPTMR_MAX;
                    
                    if(!direction)
                    {
                        data->speed_cap = -1 * data->speed_cap;
                    }
                }
                
                else
                {
                    data->speed_cap = 0;
                    eqep[device_id]->qctmr = 0;
                }
            }
            
            else
            {
                if(upevnt)
                {
                    data->speed_cap = 0;
                    eqep[device_id]->qctmr = 0;
                    upevnt = 0;
                }
            }
        }
        
        else
        {
            ;
        }
    }
    
    if(data->speed_cap == speed_previous)
    {
        speed_correction_count++;
        
        if(speed_correction_count >= EQEP_CAP_SPEED_COUNT)
        {
            data->speed_cap = 0;
        }
    }
    
    else
    {
        speed_correction_count = 0;
    }
    
    data->upevnt = upevnt;
    data->direction = direction;
    data->speed_correction_count = speed_correction_count;
}


static unsigned int 
_dec32_to_log2 (unsigned int value)
{
    unsigned int cnt = 0;
    unsigned int tmp = value;
    
    while (tmp != 1)
    {
        tmp = tmp >> 1;
        cnt++;
    }
    
    return cnt;
}


/*
 * EOF
 */
