/**
 * \file   bb_pwmss.c
 *
 * \brief  Platform specific functions and API's for BeagleBone.
 */


#include "beaglebone.h"
#include "pin_mux.h"


/*
 * BBMC pwmss pin-mux configurations
 */

void eqep_1_pinmux_setup(void)
{
    GpioPinMuxSetup(GPIO_0_8, PAD_SL_RXE_NA_PUPDD(2));
    GpioPinMuxSetup(GPIO_0_9, PAD_SL_RXE_NA_PUPDD(2));
    GpioPinMuxSetup(GPIO_0_10, PAD_SL_RXE_NA_PUPDD(2));
    GpioPinMuxSetup(GPIO_0_11, PAD_SL_RXE_NA_PUPDD(2));
}

void eqep_2_pinmux_setup(void)
{
    GpioPinMuxSetup(GPIO_2_10, PAD_SL_RXE_NA_PUPDD(3));
    GpioPinMuxSetup(GPIO_2_11, PAD_SL_RXE_NA_PUPDD(3));
    GpioPinMuxSetup(GPIO_2_12, PAD_SL_RXE_NA_PUPDD(3));
    GpioPinMuxSetup(GPIO_2_13, PAD_SL_RXE_NA_PUPDD(3));
}

void ehrpwm_1_pinmux_setup(void)
{
    //!PAD_FS_RXD_NA_PUPDD(2)
    GpioPinMuxSetup(GPIO_2_16, CONTROL_CONF_MUXMODE(2));
    GpioPinMuxSetup(GPIO_2_17, CONTROL_CONF_MUXMODE(2));
    //GpioPinMuxSetup(GPIO_2_1x, CONTROL_CONF_MUXMODE(2));
}

void ehrpwm_2_pinmux_setup(void)
{
    //!PAD_FS_RXD_NA_PUPDD(4)
    GpioPinMuxSetup(GPIO_0_22, CONTROL_CONF_MUXMODE(4));
    GpioPinMuxSetup(GPIO_0_23, CONTROL_CONF_MUXMODE(4));
    //GpioPinMuxSetup(GPIO_0_26, CONTROL_CONF_MUXMODE(4));
}



/*
 * EOF
 */
