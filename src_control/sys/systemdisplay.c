//##############################################################################
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
//##############################################################################

//------------------------------------------------------------------------------
// \file   /solutions/tida_010265_wminv/common/source/systemdisplay.c
//
// \brief  This project is used to implement motor control with
//         F28002x/F28003x/F280013x
//
//------------------------------------------------------------------------------

//
// Includes
//
#include "sys_main.h"
#include "systemdisplay.h"

//
// Defines
//

//
// Typedefs
//


//
// Globals
//


//
// Functions
//
#pragma SET_CODE_SECTION ("sysfuncs")

//! \brief display the faults with blinking LED, time base = 5ms
#pragma CODE_SECTION(SYS_displayFault, "sysfuncs");
void SYS_displayFault(void)
{
    // DPM - toggle status LED
    if(systemVars.faultSysView.all == 0)
    {
        if((motorVars_M1.controlStatus == MOTOR_CTRL_RUN) ||
                (motorVars_M1.controlStatus == MOTOR_CTRL_RUN))
        {
            // fast blink 0.2s/0.2s
            systemVars.blinkTimesSetLEDS = LED_SYS_NORM_RUN_TIMES;
            systemVars.waitTimeSetLEDS = LED_RUN_WAIT_TIME;             // 0.2s
            systemVars.delayTimeSetLEDS = LED_RUN_DELAY_TIME;           // 0.2s
        }
        else        // Both motors are stopping
        {
            // slow blink 0.5s/0.5s
            systemVars.blinkTimesSetLEDS = LED_SYS_NORM_STOP_TIMES;
            systemVars.waitTimeSetLEDS = LED_STOP_WAIT_TIME;            // 0.5s
            systemVars.delayTimeSetLEDS = LED_STOP_DELAY_TIME;          // 0.5s
        }
    }
    else
    {
        if(systemVars.faultSysView.bit.motorOverCurrent == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_MTR_OC_FAULT_TIMES;
        }
        else if(systemVars.faultSysView.bit.motorRunFailed == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_MTR_RUN_FAULT_TIMES;
        }
        else if(systemVars.faultSysView.bit.motorStartFailed == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_MTR_STARTUP_FAULT_TIMES;
        }
        else if(systemVars.faultSysView.bit.voltage == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_VOLTAGE_FAULT_TIMES;
        }
        else if(systemVars.faultSysView.bit.temperature == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_TEMPERATURE_FAULT_TIMES;
        }
        else if(systemVars.faultSysView.bit.communication == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_COMMUNICATION_LOSS_TIMES;
        }
        else if(systemVars.faultSysView.bit.hardware == 1)
        {
            systemVars.blinkTimesSetLEDS = LED_HARDWARE_FAILED_TIMES;
        }

        systemVars.waitTimeSetLEDS   = LED_FAULT_WAIT_TIME;      // 0.4s
        systemVars.delayTimeSetLEDS  = LED_FAULT_DELAY_TIME;     // 2.0s
    }

    if(systemVars.blinkTimesCntLEDS >= systemVars.blinkTimesSetLEDS)
    {
        if(systemVars.waitTimeCntLEDS >= systemVars.delayTimeSetLEDS)
        {
            systemVars.blinkStatusLEDS = LED_BLINKING_LIGHT;
            systemVars.blinkTimesCntLEDS = 0;
            systemVars.waitTimeCntLEDS = 0;
        }
        else
        {
            systemVars.waitTimeCntLEDS++;
            systemVars.blinkStatusLEDS = LED_BLINKING_DARK;
        }
    }
    else if(systemVars.waitTimeCntLEDS >= systemVars.waitTimeSetLEDS)
    {
        systemVars.waitTimeCntLEDS = 0;

        if(systemVars.blinkStatusLEDS == LED_BLINKING_DARK)
        {
            systemVars.blinkStatusLEDS = LED_BLINKING_LIGHT;
        }
        else
        {
            systemVars.blinkTimesCntLEDS++;
            systemVars.blinkStatusLEDS = LED_BLINKING_DARK;
        }
    }
    else
    {
        systemVars.waitTimeCntLEDS++;
    }

#if defined(WMINVBRD_REV1P0)
    if(systemVars.blinkStatusLEDS == LED_BLINKING_DARK)     // Off
    {
        HAL_turnOffLED(HAL_GPIO_LED1C);
    }
    else                                    // On
    {
        HAL_turnOnLED(HAL_GPIO_LED1C);
    }
    // WMINVBRD_REV1P0
#elif defined(HVMTRPFC_REV1P1)
    if(systemVars.blinkStatusLEDS == LED_BLINKING_DARK)     // Off
    {
        HAL_turnOffLED(HAL_GPIO_LED1C);
    }
    else                                    // On
    {
        HAL_turnOnLED(HAL_GPIO_LED1C);
    }
    // HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
    if(systemVars.blinkStatusLEDS == LED_BLINKING_DARK)     // Off
    {
        HAL_turnOffLED(HAL_GPIO_LED1C);
    }
    else                                    // On
    {
        HAL_turnOnLED(HAL_GPIO_LED1C);
    }
    // BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
    if(systemVars.blinkStatusLEDS == LED_BLINKING_DARK)     // Off
    {
        HAL_turnOffLED(HAL_GPIO_LED1C);
    }
    else                                    // On
    {
        HAL_turnOnLED(HAL_GPIO_LED1C);
    }
    // DRV8329AEVM_REVA
#else   // !HVMTRPFC_REV1P1 & WMINVBRD_REV1P0 & DRV8329AEVM_REVA
#error Not select a right board for this project
#endif  // !HVMTRPFC_REV1P1 & WMINVBRD_REV1P0 & DRV8329AEVM_REVA

    // Clear Fault for display
    if(systemVars.faultViewDelayTimeCnt > 0)
    {
        systemVars.faultViewDelayTimeCnt--;
    }
    else if(systemVars.faultSysUse.all == 0)
    {
        systemVars.faultSysView.all = 0;
    }

    // Clear Fault for communication
    if(systemVars.faultComDelayTimeCnt > 0)
    {
        systemVars.faultComDelayTimeCnt--;
    }
    else if(systemVars.faultSysUse.all == 0)
    {
        systemVars.faultSysCom.all = 0;
    }

    return;
}

//! \brief processing PFC and motor fault, time base = 5ms
#pragma CODE_SECTION(SYS_processFault, "sysfuncs");
void SYS_processFault(void)
{
    if((motorVars_M1.faultMtrUse.all & M_HARDWARE_BITS) != 0)
    {
        systemVars.faultSysUse.bit.hardware = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.hardware = 0;
    }

    if((motorVars_M1.faultMtrUse.all & M_COMMUNICATION_BITS) != 0)
    {
        systemVars.faultSysUse.bit.communication = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.communication = 0;
    }

    if((motorVars_M1.faultMtrUse.all & M_TEMPERATURE_BITS) != 0)
    {
        systemVars.faultSysUse.bit.temperature = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.temperature = 0;
    }

    if((motorVars_M1.faultMtrUse.all & M_OUVOLTAGE_BITS) != 0)
    {
        systemVars.faultSysUse.bit.voltage = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.voltage = 0;
    }

    if((motorVars_M1.faultMtrUse.all & M_STARTUP_FAILED_BITS) != 0)
    {
        systemVars.faultSysUse.bit.motorStartFailed = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.motorStartFailed = 0;
    }

    if((motorVars_M1.faultMtrUse.all & M_RUN_FAILED_BITS) != 0)
    {
        systemVars.faultSysUse.bit.motorRunFailed = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.motorRunFailed = 0;
    }

    if((motorVars_M1.faultMtrUse.all & M_OVERCURRENT_BITS) != 0)
    {
        systemVars.faultSysUse.bit.motorOverCurrent = 1;
    }
    else
    {
        systemVars.faultSysUse.bit.motorOverCurrent = 0;
    }

    systemVars.faultSysView.all |= systemVars.faultSysUse.all;
    systemVars.faultSysCom.all |= systemVars.faultSysUse.all;

    if(systemVars.faultSysUse.all != 0)
    {
        systemVars.faultViewDelayTimeCnt = FAULT_VIEW_WAIT_TIME;
        systemVars.faultComDelayTimeCnt = FAULT_COM_WAIT_TIME;
    }

    return;
}

//
// End of File
//
