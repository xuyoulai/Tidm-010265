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
// FILE:  sta_tests.h
//
// TITLE: Self Test Application Tests header
//
//------------------------------------------------------------------------------

#ifndef STA_TESTS_H
#define STA_TESTS_H

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
// Included files for tests.
//
#include "sta_util.h"
#include "stl_cpu_reg.h"
#include "stl_crc.h"
#include "stl_march.h"
#include "stl_osc_ct.h"
#include "stl_osc_hr.h"
#include "stl_pie_ram.h"
#include "stl_sp.h"

//*****************************************************************************
//
//! \addtogroup sta_tests STA_Tests API Functions
//!
//! This module contains the core test execution function for this application.
//! It defines the list of test cases to be executed and their implementations.
//!
//! The code for this module is contained in <tt>sta_tests.c</tt>, with
//! <tt>sta_tests.h</tt> containing the API declarations.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#if defined(SAFETY_ENABLE)
#define STA_INITIAL_TIMEOUT         4000U       // ms
#define STA_MAINLOOP_TIMEOUT        4000U       // ms

#define STA_TESTS_PASS              0U
#define STA_TESTS_FAIL              1U

#define STA_TESTS_REPORT_SIZE       29
#define STA_TESTS_REPORT_TOTAL      15
#define STA_TESTS_REPORT_PASS       5

#define STA_PROFILER_TIMER_BASE     CPUTIMER1_BASE

#define STA_INITIAL_TIMER_SET       STA_INITIAL_TIMEOUT  / 100U
#define STA_MAINLOOP_TIMER_SET      STA_MAINLOOP_TIMEOUT / 1U

#define FLAG_STA_TEST_START         0x0000      // #0
#define FLAG_STA_TEST_PIE_HANDLER   0x0001      // #1
#define FLAG_STA_CPU_REG            0x0002      // #2
#define FLAG_STA_FPU_REG            0x0004      // #3
#define FLAG_STA_SP_TEST            0x0008      // #4
#define FLAG_STA_MARCH              0x0010      // #5
#define FLAG_STA_MARCH_COPY         0x0020      // #6
#define FLAG_STA_FLASH_CRC          0x0040      // #7
#define FLAG_STA_OSC_CT             0x0080      // #8
#define FLAG_STA_TEST_END           0x8000      // #9

//
// Typedefs
//
typedef enum
{
    STA_TEST_START,             // #0
    STA_TEST_PIE_HANDLER,       // #1, 520/557, DINT@INJ
    STA_CPU_REG,                // #2, 652/306, DINT
    STA_FPU_REG,                // #3, 287/673, DINT
    STA_SP_TEST,                // #4, 119/133
    STA_MARCH,                  // #5, 403/530
    STA_MARCH_COPY,             // #6, 501/666
    STA_FLASH_CRC,              // #7,
    STA_OSC_CT,                 // #8, 145/158
    STA_TEST_END,               // #9
    STA_TESTS_NUMBERS           // #10
} STA_TestsTypes;

//
// Globals
//
extern const STA_TestsTypes STA_Tests_testArray[STA_TESTS_NUMBERS];

#if STA_UTIL_PROFILE
extern uint32_t STA_Tests_cycleCounts[STA_TESTS_NUMBERS];
#endif

extern uint16_t STA_Tests_timer;
extern uint16_t STA_Tests_passCount;
extern uint16_t STA_Tests_index;
extern bool     STA_Tests_injectError;
//
// Linker generated symbols
//
extern uint16_t __stack;
extern uint16_t __TI_STACK_END;

//*****************************************************************************
//
//! Executes the safety library tests as an example.
//!
//! \param testItem is the enumerated type STA_test_types test to perform.
//!
//! This function configures the device for each specific test. If injected
//! errors are enabled, then this function will inject an error for the test
//! being performed. It will then execute each STL test and return a message
//! of characters to be printed to the terminal via SCI.
//!
//! \return Returns a pointer to a message string indicating a pass or fail
//! status and the test name. This string will be printed out of SCIA to the
//! user.
//
//*****************************************************************************
extern uint16_t STA_Tests_testDevice(STA_TestsTypes testItem);

//*****************************************************************************
//
//! Enables errors to be injected into the tests.
//!
//! This function sets the global variable \b STA_Tests_injectError to true.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Tests_injectErrorEnable(void);

//*****************************************************************************
//
//! Disables errors to be injected into the tests.
//!
//! This function sets the global variable \b STA_Tests_injectError to false.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Tests_injectErrorDisable(void);
#endif  // SAFETY_ENABLE

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

#endif

#endif // STA_TESTS_H

//
// End of File
//
