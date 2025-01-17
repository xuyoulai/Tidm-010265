//#############################################################################
//
// FILE:  sta_timer.h
//
// TITLE: Self Test Application Timer header
//
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

#ifndef STA_TIMER_H
#define STA_TIMER_H

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

//*****************************************************************************
//
//! \addtogroup sta_timer STA_Timer API Functions
//!
//! This module handles the initialization and handling of the CPU timer
//! system used to time the execution of the tests run by this application.
//!
//! The code for this module is contained in <tt>sta_timer.c</tt>, with
//! <tt>sta_timer.h</tt> containing the API declarations.
//!
//! @{
//
//*****************************************************************************
#if defined(SAFETY_ENABLE)
//*****************************************************************************
//
//! Timer 0 ISR.
//!
//
//*****************************************************************************
extern __interrupt void STA_Timer_timer0Isr(void);

//*****************************************************************************
//
//! \brief Configures Timer 0 to tick so that the STA will continue to make
//! progress.
//!
//! \param msTimeOut is the number of milliseconds between each execution
//! of a test found in sta_tests.c. Another test will execute every time
//! this STA timer runs out.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Timer_config(void);

//*****************************************************************************
//
//! \brief Checks if Timer 0 has timed-out.
//!
//! \return Returns true if the timer has timed-out and false if it has not.
//
//*****************************************************************************
extern bool STA_Timer_isTimedOut(void);

//*****************************************************************************
//
//! \brief Clears Timer 0 time-out flag.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Timer_clearTimeOut(void);

//*****************************************************************************
//
//! \brief Restarts Timer 0.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Timer_restart(void);

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

#endif // STA_TIMER_H

//
// End of File
//
