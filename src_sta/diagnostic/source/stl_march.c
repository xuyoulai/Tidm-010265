//###########################################################################
//
// FILE:  stl_march.c
//
// TITLE: Diagnostic Library March13N software module source
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

//
// Includes
//
#include "stl_march.h"
#include "stl_util.h"
#include "memcfg.h"
#include "inc/hw_types.h"

//*****************************************************************************
//
// STL_March_injectError(const STL_March_InjectErrorHandle errorHandle)
//
//*****************************************************************************
void STL_March_injectError(const STL_March_InjectErrorHandle errorHandle)
{
    //
    // Enter user-specified test mode.
    //
    MemCfg_setTestMode(errorHandle->ramSection, errorHandle->testMode);

    //
    // XOR the data at the specified address with the mask provided.
    //
    HWREG(errorHandle->address) ^= errorHandle->xorMask;

    //
    // Go back to functional mode.
    //
    MemCfg_setTestMode(errorHandle->ramSection, MEMCFG_TEST_FUNCTIONAL);
}

//*****************************************************************************
//
// STL_March_checkErrorStatus(void)
//
//*****************************************************************************
uint16_t STL_March_checkErrorStatus(void)
{
    uint16_t status;

    if((HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRFLG) != 0U) &&
       (HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRFLG) == 0U))
    {
        status = STL_MARCH_UNC_ERROR;

        //
        // Set global error flag.
        //
        STL_Util_setErrorFlag(STL_UTIL_MARCH);
    }
    else if((HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRFLG) != 0U) &&
            (HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRFLG) == 0U))
    {
        status = STL_MARCH_CORR_ERROR;

        //
        // Set global error flag.
        //
        STL_Util_setErrorFlag(STL_UTIL_MARCH);
    }
    else if((HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRFLG) != 0U) &&
            (HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRFLG) != 0U))
    {
        status = STL_MARCH_BOTH_ERROR;

        //
        // Set global error flag.
        //
        STL_Util_setErrorFlag(STL_UTIL_MARCH);
    }
    else
    {
        status = STL_MARCH_PASS;
    }

    return(status);
}

//
// End of File
//
