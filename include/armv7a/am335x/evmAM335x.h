/**
 * \file   evmAM335x.h
 *
 * \brief  This file contains prototype declarations of functions which 
 *         performs EVM configurations.
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


#ifndef _EVM_AM335X_H_
#define _EVM_AM335X_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
**                    INTERNAL MACRO DEFINITIONS
*****************************************************************************/
#define CTRL_NUM_IOPAD_REGS                 (211)

#define PRINT_WRG(x)                        UARTPuts ("\r\n|WARNING|", -1);\
                                            UARTPuts (x, -1); \
                                            UARTPuts ("|\r\n", -1);            
                                 
#define PRINT_ERR(x)                        UARTPuts ("\r\n|ERROR|", -1); \
                                            UARTPuts (x, -1); \
                                            UARTPuts ("|\r\n", -1);                                                                   
                                 
#define PRINT_RESULT_PASS()                 UARTPuts ("\r\n|RESULT|PASS|\r\n", -1);

#define PRINT_RESULT_FAIL()                 UARTPuts ("\r\n|RESULT|FAIL|\r\n", -1);


/*
** Structure to store the control register context. More registers
** can be added to this if need to be saved.
*/
typedef struct ctrlRegContext {
    unsigned int pwmssctrl;
    unsigned int gmiisel;
    unsigned int ioPad[CTRL_NUM_IOPAD_REGS];
}CTRLREGCONTEXT;

/*****************************************************************************
**                    FUNCTION PROTOTYPES
*****************************************************************************/

extern void ControlRegContextSave(CTRLREGCONTEXT *contextPtr);
extern void ControlRegContextRestore(CTRLREGCONTEXT *contextPtr);

extern void EVMMACAddrGet(unsigned int addrIdx, unsigned char *macAddr);

extern unsigned int McSPIPinMuxSetup(unsigned int instanceNum);
extern unsigned int McSPI0CSPinMuxSetup(unsigned int csPinNum);
extern void McSPI0ModuleClkConfig(void);
extern unsigned int McSPIPinMuxSetup(unsigned int instanceNum);
extern unsigned int McSPI0CSPinMuxSetup(unsigned int csPinNum);
extern void McSPI0ModuleClkConfig(void);


extern unsigned int UARTPinMuxSetup(unsigned int instanceNum);
extern void UART0ModuleClkConfig(void);

extern unsigned int I2CPinMuxSetup(unsigned int instanceNum);
extern void I2C0ModuleClkConfig(void);
extern void I2C1ModuleClkConfig(void);

extern void EDMAModuleClkConfig(void);

extern void WatchdogTimer1ModuleClkConfig(void);
extern void WatchdogTimer1ModuleClkConfig(void);

extern unsigned int RTCRevisionInfoGet(void);

extern void DMTimer2ModuleClkConfig(void);
extern void DMTimer3ModuleClkConfig(void);
extern void DMTimer4ModuleClkConfig(void);
extern void DMTimer7ModuleClkConfig(void);

extern void EVMPortRGMIIModeSelect(void);
extern unsigned int EVMProfileGet(void);

extern unsigned int McASP1PinMuxSetup(void);
extern void McASP1ModuleClkConfig(void);

extern unsigned int CPSWPinMuxSetup(void);

extern unsigned int LCDPinMuxSetup(void);

extern unsigned int GPIO0PinMuxSetup(unsigned int pinNum);
extern unsigned int GPIO0Pin6PinMuxSetup(void);
extern unsigned int GPIO1Pin2PinMuxSetup(void);
extern unsigned int GPIO0Pin2PinMuxSetup(void);
extern unsigned int GPIO1PinMuxSetup(unsigned int pinNo);
extern unsigned int GPIO1Pin30PinMuxSetup(void);
extern unsigned int GPIO1Pin16PinMuxSetup(void);
extern unsigned int GPIO0Pin7PinMuxSetup(void);
extern unsigned int GPIO1Pin20PinMuxSetup(void);
extern unsigned int GPIO2Pin24PinMuxSetup(void);
extern unsigned int GPIO1Pin23PinMuxSetup(void);
extern unsigned int GPIO0Pin19PinMuxSetup(void);
extern unsigned int GPIO1Pin28PinMuxSetup(void);
extern void GPIO0ModuleClkConfig(void);
extern void GPIO1ModuleClkConfig(void);

extern void RTCModuleClkConfig(void);
extern unsigned int RTCRevisionInfoGet(void);

extern void EEPROMI2CSetUp(unsigned int slaveAddr);
extern void EEPROMI2CRead(unsigned char *data, unsigned int length, unsigned short offset);

extern void GPMCClkConfig(void);

extern unsigned int NANDPinMuxSetup(void);

extern void LCDModuleClkConfig(void);
extern unsigned int LCDPinMuxSetup(void);
extern void LCDBackLightEnable(void);
extern void LCDBackLightDisable(void);

extern void TSCADCModuleClkConfig(void);

extern void CPSWClkEnable(void);

extern void PWMSSModuleClkConfig(unsigned int instanceNum);
extern void PWMSSClockEnable(unsigned int instance, unsigned int module);
extern void PWMSSTBClkEnable(unsigned int);
extern void PWMSSClockStop(unsigned int instance, unsigned int module);
extern unsigned int PWMSSClockStopStatusGet(unsigned int instance, unsigned int module);
unsigned int PWMSSClockEnableStatusGet(unsigned int instance, unsigned int module);

extern void EPWMSSModuleClkConfig(unsigned int instanceNum);

extern unsigned int EPWM2PinMuxSetup(void);

extern unsigned int ECAPPinMuxSetup(unsigned int instanceNum);

extern void HSMMCSDModuleClkConfig(void);
extern unsigned int HSMMCSDPinMuxSetup(void);

extern unsigned int TSCADCPinMuxSetUp(void);

extern void UPDNPinControl(void);

extern unsigned int DCANPinMuxSetUp(unsigned int instanceNum);
extern void DCANMsgRAMInit(unsigned int instanceNum);
extern void DCANModuleClkConfig(void);

extern void USB0ModuleClkConfig(void);
extern void USBModuleClkEnable(unsigned int ulIndex, unsigned int ulBase);
extern void USBModuleClkDisable(unsigned int ulIndex, unsigned int ulBase);


#ifdef __cplusplus
}
#endif

#endif

/******************************** End of file *******************************/
