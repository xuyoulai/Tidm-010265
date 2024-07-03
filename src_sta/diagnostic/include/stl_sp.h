//###########################################################################
//
// FILE:  stl_sp.h
//
// TITLE: Diagnostic Library Stack Pointer software module header
//
//###########################################################################
// $TI Release: C2000 Diagnostic Library v5.00.Beta $
// $Release Date: Sun Apr  7 09:01:21 IST 2024 $
// $Copyright:
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
//
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
//###########################################################################

#ifndef STL_SP_H
#define STL_SP_H

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

//
// Includes
//
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"

//*****************************************************************************
//
//! \addtogroup stl_sp Stack Pointer API Functions
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_SP_PASS               0U
#define STL_SP_FAIL               1U
#define STL_SP_CONFIG_PASS        2U
#define STL_SP_CONFIG_FAIL        3U

#define PIEVECTTABLE_O_RTOS       ((INT_RTOS >> 16) * 2U)

//
// Typedefs
//

//
//! Values corresponding to valid watchpoint registers
//!
typedef enum
{
    STL_SP_WP0 = 0U,                           //!< Watchpoint 0
    STL_SP_WP1 = 1U                            //!< Watchpoint 1
} STL_SP_Watchpoint;

//
//! \brief Defines the Stack Pointer test object
//!
typedef struct
{
    uint32_t          startAddress;         //!< Stack Start Address
    uint32_t          endAddress;           //!< Stack End Address
    uint32_t          stackRefAddress;      //!< Stack Reference Address
    STL_SP_Watchpoint watchpoint;           //!< Watchpoint 0 or 1
    uint32_t          prevIntVector; //!< Previous Watchpoint Interrupt Vector
                                     //!< Used internally to save/restore it.
} STL_SP_Obj;

//
//! \brief Defines the Stack Pointer test handle
//!
typedef STL_SP_Obj * STL_SP_Handle;

//
// Prototypes
//
__interrupt void STL_SP_testISR(void);

//*****************************************************************************
//
//! \brief Configure watchpoint for stack pointer corruption detection.
//!
//! \param spHandle is a pointer to the Stack Pointer object.
//!
//! This function initializes a hardware debug watchpoint to detect stack
//! pointer corruption. It initializes watchpoint 0 or watchpoint 1 based on
//! the value of the watchpoint member of the SP object. It defines a valid
//! stack pointer range based on the stack start and end addresses.
//! More details about stack pointer detection can be found in the
//! Online Stack Overflow Detection on the TMS320C28x DSP Application Report
//! http://www.ti.com/lit/an/spra820/spra820.pdf
//!
//! \note This function stores the previous RTOS interrupt vector in the
//! STL_SP_Obj struct pointed to by spHandle in case the system uses a
//! different interrupt vector. The system interrupt vector can be restored
//! by calling STL_SP_restoreVector().
//!
//! \return If the watchpoint is configured successfully, the function returns
//! \b STL_SP_CONFIG_PASS. Otherwise, it returns \b STL_SP_CONFIG_FAIL.
//
//*****************************************************************************
extern uint16_t STL_SP_configSP(const STL_SP_Handle spHandle);

//*****************************************************************************
//
//! \brief Write to an address within the monitored range.
//!
//! \param spHandle is a pointer to the Stack Pointer object.
//!
//! This function writes data to an address within the range monitored by the
//! debug watchpoint, which is expected to trigger the STL_SP_testISR RTOS ISR.
//!
//! \return If the interrupt is triggered when the stack pointer is
//! within the monitored range, the function returns \b STL_SP_PASS. Otherwise,
//! it returns \b STL_SP_FAIL.
//
//*****************************************************************************
extern uint16_t STL_SP_writeMonitoredRange(const STL_SP_Handle spHandle);

//*****************************************************************************
//
//! \brief Disables the watchpoint to inject error.
//!
//! \param spHandle is a pointer to the Stack Pointer object.
//!
//! This function disables the watchpoint by clearing the event control
//! register. When the watchpoint is disabled, it will not monitor stack
//! pointer corruption.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_SP_disableWatchpoint(const STL_SP_Handle spHandle);

//*****************************************************************************
//
//! \brief Restores the previous watchpoint (RTOS) interrupt vector.
//!
//! \param spHandle is a pointer to the Stack Pointer object.
//!
//! This function restores the previous RTOS interrupt vector which was stored
//! in the STL_SP_Obj struct pointed to by spHandle.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_SP_restoreVector(const STL_SP_Handle spHandle);

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

#endif  //  STL_SP_H

//
// End of File
//
