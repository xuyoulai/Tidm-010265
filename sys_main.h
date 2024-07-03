//#############################################################################
// $Copyright:
// Copyright (C) 2017-2024 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################


//! \file  solutions/universal_motorcontrol_lab/common/include/sys_main.h
//! \brief  header file to be included in all labs
//!

#ifndef SYS_MAIN_H
#define SYS_MAIN_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup SYS MAIN
//! @{
//
//*****************************************************************************

// modules
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#if !defined(__TMS320C28XX_CLA__)
#include "user.h"
#include "hal.h"
#endif

#include "datalogIF.h"
#include "cpu_time.h"

#if defined(DAC128S_ENABLE)
#include "dac128s085.h"
#endif  // DAC128S_ENABLE


#include "motor_common.h"
#include "motor1_drive.h"

#include "guidatalist.h"
#include "guicontrol.h"
#include "controlparameters.h"
#include "systemcontrol.h"

#define LED_BLINK_FREQ_Hz           (0.5f)          // 1Hz

#define POWER_RELAY_WAIT_TIME_ms    (1000)          // 1s
#define OFFSET_CHECK_WAIT_TIME_ms   (1000)          // 1s

#define POWER_RELAY_ON_VOLTAGE_V    (0.10f * USER_M1_ADC_FULL_SCALE_VOLTAGE_V)        // 100V

//
//! \brief typedefs for the fault
//
typedef struct _FAULT_SYS_BITS_
{             // bits  description
    uint16_t hardware:1;            // 0  Hardware failed
    uint16_t communication:1;       // 1  communication
    uint16_t temperature:1;         // 2  temperature
    uint16_t voltage:1;             // 3  voltage

    uint16_t motorStartFailed:1;    // 4  Motor startup failed
    uint16_t motorRunFailed:1;      // 5  Motor run failed
    uint16_t motorOverCurrent:1;    // 6  Motor over current
    uint16_t reserve7:1;            // 7  Reserve

    uint16_t reserve8:1;            // 8  Reserve
    uint16_t reserve9:1;            // 9  Reserve
    uint16_t reserve10:1;           // 10  Reserve
    uint16_t reserve11:1;           // 11 Reserve

    uint16_t reserve12:1;           // 12 Reserve
    uint16_t reserve13:1;           // 13 Reserve
    uint16_t reserve14:1;           // 14 Reserve
    uint16_t reserve15:1;           // 15 Reserve
} FAULT_SYS_BITS;

typedef union _FAULT_SYS_t
{
    uint16_t        all;
    FAULT_SYS_BITS  bit;
}FAULT_SYS_t;


//------------------------------------------------------------------------
typedef struct _SYSTEM_Vars_t_
{
    float32_t speedRef_Hz;

    uint32_t mainLoopCnt;
    uint32_t timerCnt_1min;
    uint16_t timerCnt_1s;
    uint16_t timerCnt_5ms;
    uint16_t timerBase_1ms;

    uint16_t powerRelayWaitTime_ms;

    uint16_t waitTimeCntLEDS;       //!< LEDS blink time counter
    uint16_t waitTimeSetLEDS;       //!< LEDS blink wait time settings
    uint16_t delayTimeSetLEDS;      //!< LEDS blink cycle delay time settings
    uint16_t blinkStatusLEDS;       //!< LEDs blink status
    uint16_t blinkTimesSetLEDS;     //!< LEDS blinking times settings
    uint16_t blinkTimesCntLEDS;     //!< LEDS blinking times counter
    uint16_t faultViewDelayTimeCnt;
    uint16_t faultComDelayTimeCnt;

    uint16_t estLibVersion;
    Board_Kit_e boardKit;

    FAULT_SYS_t faultSysUse;
    FAULT_SYS_t faultSysView;
    FAULT_SYS_t faultSysCom;

    bool flagEnableSystem;
}SYSTEM_Vars_t;

extern volatile SYSTEM_Vars_t systemVars;

#if defined(BUFDAC_MODE)
extern HAL_BuffDACData_t bufDACData;
#endif  // BUFDAC_MODE

#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1) || defined(WMINVBRD_REV1P0) || \
    defined(BSXL8323RH_REVB) || defined(DRV8329AEVM_REVA) || defined(DRV8353RH_EVM)
extern HAL_PWMDACData_t pwmDACData;
// ( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA | DRV8353RH_EVM)
#else   // !( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA | DRV8353RH_EVM)
#error EPWMDAC is not supported on this kit!
#endif  // !( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA | DRV8353RH_EVM)
#endif  // EPWMDAC_MODE

#if defined(DAC128S_ENABLE)
extern DAC128S_Handle   dac128sHandle;
extern DAC128S_Obj      dac128s;
#endif  // DAC128S_ENABLE

#ifdef CPUTIME_ENABLE
extern CPU_TIME_Obj     cpuTime;
extern CPU_TIME_Handle  cpuTimeHandle;
#endif  // CPUTIME_ENABLE

#if defined(GUI_SCI_EN)
extern void GUI_initUartParams(void);
extern void GUI_setupUart(void);

extern void GUI_initCtrlParms(void);
extern void GUI_updateCtrlSate(void);
#endif  // GUI_SCI_EN

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of SYS_MAIN_H definition
