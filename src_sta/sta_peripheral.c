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
// C2000 Diagnostic Library v4.01.00
//
// FILE:  sta_peripheral.c
//
// TITLE: Self Test Application Peripherals (ADC, PWM...) source
//
//------------------------------------------------------------------------------

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "user.h"
#include "hal.h"
#include "sta_peripheral.h"

#if defined(SAFETY_ENABLE)
//
// Defines

//
// Globals
//

//*****************************************************************************
//
// STA_Tests_testDevice(STA_TestsTypes testItem)
//
//*****************************************************************************
uint16_t STA_testADC(HAL_ADCData_t *pADCData)
{
    uint16_t offsetFailFlag = 0;

    // ternary operator
#if defined(MOTOR1_DCLINKSS)    // Single shunt
    pADCData->offsetIdcFilter = (pADCData->offsetIdcFilter>>1) +
            ((pADCData->offsetIdcFilter + pADCData->offsetIdcSense) >> 2);

    offsetFailFlag |= (pADCData->offsetIdcFilter > ADC_IDC_OFFSET_MAX) ? 0x0000 : 0x1000;
    offsetFailFlag |= (pADCData->offsetIdcFilter < ADC_IDC_OFFSET_MIN) ? 0x0000 : 0x1000;
#else   // !(MOTOR1_DCLINKSS)       // 2/3 shunt
    pADCData->offsetIsFilter[0] = (pADCData->offsetIsFilter[0]>>1) +
            ((pADCData->offsetIsFilter[0] + pADCData->offsetIsSense[0]) >> 2);
    pADCData->offsetIsFilter[1] = (pADCData->offsetIsFilter[1]>>1) +
            ((pADCData->offsetIsFilter[1] + pADCData->offsetIsSense[0]) >> 2);
    pADCData->offsetIsFilter[2] = (pADCData->offsetIsFilter[2]>>1) +
            ((pADCData->offsetIsFilter[2] + pADCData->offsetIsSense[0]) >> 2);

    offsetFailFlag |= (pADCData->offsetIsFilter[0] > ADC_IU_OFFSET_MAX) ? 0x0000 : 0x1000;
    offsetFailFlag |= (pADCData->offsetIsFilter[0] < ADC_IU_OFFSET_MIN) ? 0x0000 : 0x1000;

    offsetFailFlag |= (pADCData->offsetIsFilter[1] > ADC_IV_OFFSET_MAX) ? 0x0000 : 0x2000;
    offsetFailFlag |= (pADCData->offsetIsFilter[1] < ADC_IV_OFFSET_MIN) ? 0x0000 : 0x2000;

    offsetFailFlag |= (pADCData->offsetIsFilter[2] > ADC_IW_OFFSET_MAX) ? 0x0000 : 0x4000;
    offsetFailFlag |= (pADCData->offsetIsFilter[2] < ADC_IW_OFFSET_MIN) ? 0x0000 : 0x4000;
#endif  // !(MOTOR1_DCLINKSS)       // 2/3 shunt

    return(offsetFailFlag);
}

#endif  // SAFETY_ENABLE

//
// End of File
//
