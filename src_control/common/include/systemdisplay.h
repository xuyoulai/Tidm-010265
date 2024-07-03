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
// \file   /solutions/tida_010265_wminv/common/include/systemdisplay.h
//
// \brief  header file to be included in all labs
//         support for motor control with F28002x/F28003x/F280013x
//
//------------------------------------------------------------------------------


#ifndef SYSTEM_DISPLAY_H
#define SYSTEM_DISPLAY_H


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
//! \defgroup CODEUPDATE CODEUPDATE
//! @{
//
//*****************************************************************************

// the includes
#include "driverlib.h"
#include "device.h"


//
// Defines
//

// Fault View and Communication delay time, 5ms base
#define FAULT_VIEW_WAIT_TIME            4000    // 20s
#define FAULT_COM_WAIT_TIME             2000    // 10s

// LED status
#define LED_BLINKING_DARK               0
#define LED_BLINKING_LIGHT              1

// LED blink time, 5ms base
#define LED_RUN_WAIT_TIME               40      // 0.2s, 40
#define LED_RUN_DELAY_TIME              40      // 0.2s, 40

#define LED_STOP_WAIT_TIME              160     // 0.8s, 160
#define LED_STOP_DELAY_TIME             160     // 0.8s, 160

#define LED_FAULT_WAIT_TIME             80      // 0.4s, 80
#define LED_FAULT_DELAY_TIME            400     // 2s,   400

// LED blinking times
#define LED_SYS_NORM_RUN_TIMES          1
#define LED_SYS_NORM_STOP_TIMES         1

#define LED_MTR_OC_FAULT_TIMES          2
#define LED_MTR_RUN_FAULT_TIMES         3
#define LED_MTR_STARTUP_FAULT_TIMES     4
#define LED_VOLTAGE_FAULT_TIMES         5
#define LED_TEMPERATURE_FAULT_TIMES     6
#define LED_COMMUNICATION_LOSS_TIMES    7
#define LED_HARDWARE_FAILED_TIMES       8

//
// Typedefs
//


// the modules


// the globals


// the functions
//! \brief display the faults with blinking LED
extern void SYS_displayFault(void);


//! \brief process the faults from motor control
extern void SYS_processFault(void);

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

#endif // end of SYSTEM_DISPLAY_H defines

//
// End of File
//
