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
//!
//! MotorControl SDK
//!
//! \file   /solutions/tida_010265_wminv/common/include/eepemulation.h
//!
//! \brief  header file to be included in all labs
//!         support for motor control with F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


#ifndef EEPROM_EMULATION_H_
#define EEPROM_EMULATION_H_


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
//! \defgroup EEPEMULATION_H_ CODEUPDATE
//! @{
//
//*****************************************************************************

// the includes
#include "driverlib.h"
#include "device.h"

// Include Flash API  header file

#include "flash_kernel_programming.h"
#include "flash_kernel_commands.h"

// must replace with the GPIO number of the SCIRX pin chosen in .syscfg
#define GPIO_SCIRX_NUMBER   GUI_SCI_SCIRX_GPIO

//
// choose baud rate being received from target baud rate device
// closest baud rate to 9600 that can be set in register is 9601
#define TARGETBAUD          GUI_BAUD_RATE + 1

// number of samples in the array (higher = better averaging, lower = faster)
#define NUMSAMPLES          32          // 10bit*10byte = 100bit

#define FAILEDSAMPLES       66          // 10bit*10byte/2=50

#define ONE_OVER_SAMPLES    (float32_t)(0.5f / ((float32_t)NUMSAMPLES))

//
// margin for what is considered a "good" pulse width:
// set this higher to allow more samples to be considered "good" data,
// if losing too much data set this lower to prevent "bad" samples to
// be discarded more strictly
#define MARGINPERCENT       0.065f

// minimum baud rate
#define MIN_BAUD_RATE       (float32_t)(((float32_t)TARGETBAUD) * (1.0f - MARGINPERCENT))

// maximum baud rate
#define MAX_BAUD_RATE       (float32_t)(((float32_t)TARGETBAUD) * (1.0f + MARGINPERCENT))

// minimum bits width of this baud rate
#define MIN_BITS_WIDTH      (uint32_t)(2.0f * (((float32_t)DEVICE_SYSCLK_FREQ) / ((float32_t)MAX_BAUD_RATE)))

// maximum bits width of this baud rate
#define MAX_BITS_WIDTH      (uint32_t)(2.0f * (((float32_t)DEVICE_SYSCLK_FREQ) / ((float32_t)MIN_BAUD_RATE)))

#if defined(BUADTUNE_EN)
extern void BRT_initConfiguration(void);
extern void BRT_tuneBaudRate(void);

extern volatile uint16_t flagBaudRateTuned;
extern volatile uint16_t  stopCaptures;
#endif  // BUADTUNE_EN

// the modules


// the globals


// the functions



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

#endif // end of EEPROM_EMULATION_H_ defines

//
// End of File
//
