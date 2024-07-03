//###########################################################################
//
// FILE:  stl_pie_ram.h
//
// TITLE: Diagnostic Library PIE RAM software module header
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

#ifndef STL_PIE_RAM_H
#define STL_PIE_RAM_H

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
#include "cpu.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "debug.h"
#include "sysctl.h"

//*****************************************************************************
//
//! \addtogroup stl_pie_ram PIE RAM API Functions
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_PIE_RAM_PASS                    0U
#define STL_PIE_RAM_FAIL_HANDLER            1U
#define STL_PIE_RAM_MIN_INDEX               0x6U // Skip EMU boot config regs
#define STL_PIE_RAM_MAX_INDEX               0x100U

#define STL_PIE_RAM_TABLE_ROW_M             0x00FFU
#define STL_PIE_RAM_TABLE_COL_M             0xFF00U
#define STL_PIE_RAM_TABLE_COL_S             8U
#define STL_PIE_RAM_VECT_ID_M               0xFFFF0000UL
#define STL_PIE_RAM_VECT_ID_S               16U

//*****************************************************************************
//
//! PIE RAM mismatch error handler used by STL_PIE_RAM_testHandler().
//!
//! This function sets a static global flag for STL_PIE_RAM_testHandler()
//! and also sets the global error flag for the STL library.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_PIE_RAM_handler(void);

//*****************************************************************************
//
//! Configures a vector mismatch handler.
//!
//! \param handlerPtr is a pointer or address of the PIE RAM vector mismatch
//! error exception handler.
//!
//! This function configures a PIE RAM mismatch error handler for the users
//! application. On a vector fetch mismatch, an exception will be taken and
//! the error handler will be executed.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_PIE_RAM_configHandler(const void *handlerPtr);

//*****************************************************************************
//
//! Injects a fault by injecting a parity error in the PIE RAM vector table.
//!
//! \param entry is an index of the PIE RAM and should be a multiple of 2
//! for 32-bit alignment.
//!
//! This function flips a bit in the data of an entry of the redundant PIE RAM
//! vector table without updating the parity using the PIE RAM test mode in
//! order to inject a fault. The \b entry parameter must be within the
//! boundaries \b STL_PIE_RAM_MIN_INDEX and \b STL_PIE_RAM_MAX_INDEX.
//!
//! For more details see the device's Technical Reference Manual chapter
//! "Vector Address Validity Check."
//!
//! \return None.
//
//*****************************************************************************
extern void STL_PIE_RAM_injectFault(const uint16_t entry);

//*****************************************************************************
//
//! Restores a PIE RAM vector entry.
//!
//! \param entry is an index of the PIE RAM and should be a multiple of 2
//! for 32-bit alignment.
//! \param isrPtr is a pointer to the ISR to be restored at entry.
//!
//! This function restores the PIE RAM vector entry after altered by the fault
//! injected by \b STL_PIE_RAM_injectFault().
//!
//! \return None.
//
//*****************************************************************************
extern void STL_PIE_RAM_restoreVector(const uint16_t entry, const void *isrPtr);

//*****************************************************************************
//
//! Restores the PIE RAM from the initialization source.
//!
//! \param pieTableSourcePtr is a pointer to the source of the PIE vector table
//! in the user's application.
//!
//! This function restores the PIE RAM vector table and consequently the
//! redundant PIE RAM vector table.
//!
//! \note This function restores the entire PIE RAM table and not just a single
//! vector entry.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_PIE_RAM_restoreTable(const uint32_t *pieTableSourcePtr);

//*****************************************************************************
//
//! Tests PIE RAM integrity.
//!
//! \param interruptNumber is the 32-bit interrupt value used in the interrupt
//! driver found in hw_ints.h.
//!
//! This function checks the functionality of the PIE RAM parity error
//! handler. It will inject an error for the input interrupt, enable the
//! interrupt, and force the interrupt. It will then check to see that the error
//! handler was serviced properly. Afterward, it will restore the PIE RAM and
//! and restore the original error handler. It will also acknowledge the PIE
//! so that further interrupts from that group can be serviced.
//!
//! For a more thorough test of the SRAM logic, see the
//! \b sdl_ex_ram_ecc_parity_test example.
//!
//! \note This test only covers PIE interrupts. This test does not cover CPU
//! interrupts.
//!
//! \return If the test passes, it returns \b STL_PIE_RAM_PASS. If the test
//! fails, and the error handler was not serviced, then it returns
//! \b STL_PIE_RAM_FAIL_HANDLER.
//
//*****************************************************************************
extern uint16_t STL_PIE_RAM_testHandler(const uint32_t interruptNumber);

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

#endif // STL_PIE_RAM_H
