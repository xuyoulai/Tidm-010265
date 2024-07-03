//###########################################################################
//
// FILE:  stl_pie_ram.c
//
// TITLE: Diagnostic Library PIE RAM software module source
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
#include "stl_pie_ram.h"
#include "stl_util.h"
#include "interrupt.h"
#include "memcfg.h"

//
// Globals.
//
volatile static uint16_t STL_PIE_RAM_handlerFlag;

//*****************************************************************************
//
// void STL_PIE_RAM_handler(void)
//
//*****************************************************************************
void STL_PIE_RAM_handler(void)
{
    //
    // Set the global flag.
    //
    STL_PIE_RAM_handlerFlag = 1;
}

//*****************************************************************************
//
// STL_PIE_RAM_configHandler(void)
//
//*****************************************************************************
void STL_PIE_RAM_configHandler(const void *handlerPtr)
{
    //
    // Configure the function as the PIE RAM error handler.
    //
    EALLOW;
    HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR) = (uint32_t)handlerPtr;
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_injectFault(uint16_t entry)
//
//*****************************************************************************
void STL_PIE_RAM_injectFault(const uint16_t entry)
{
    //
    // Assert the entry is within the limits of the PIE vector table and
    // a multiple of 2 to ensure it is 32-bit aligned.
    //
    ASSERT(entry >= STL_PIE_RAM_MIN_INDEX);
    ASSERT(entry < STL_PIE_RAM_MAX_INDEX);
    ASSERT((entry & 0x0001U) == 0U);

    //
    // Put the PIE RAM in test mode, where a write to the data does not update
    // the parity bit.
    //
    MemCfg_setTestMode(MEMCFG_SECT_PIEVECT, MEMCFG_TEST_WRITE_DATA);

    //
    // Inject an error by flipping a bit in the entry specified
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE + entry) ^= 0x00001000UL;
    EDIS;

    //
    // Go back to functional mode
    //
    MemCfg_setTestMode(MEMCFG_SECT_PIEVECT, MEMCFG_TEST_FUNCTIONAL);
}

//*****************************************************************************
//
// STL_PIE_RAM_restoreVector(const uint16_t entry, const void *isrPtr)
//
//*****************************************************************************
void STL_PIE_RAM_restoreVector(const uint16_t entry, const void *isrPtr)
{
    //
    // Rewrite the vector table entry. This will update the redundant table
    // as well.
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE + entry) = (uint32_t)isrPtr;
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_retoreTable(const uint32_t *pieTableSourcePtr)
//
//*****************************************************************************
void STL_PIE_RAM_restoreTable(const uint32_t *pieTableSourcePtr)
{
    uint16_t index;

    EALLOW;

    //
    // Restore the PIE vector table.
    //
    for(index = STL_PIE_RAM_MIN_INDEX; index < STL_PIE_RAM_MAX_INDEX;
        index = index + 2U)
    {
        HWREG(PIEVECTTABLE_BASE + index) = pieTableSourcePtr[(index >> 1)];
    }
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_testHandler(void)
//
//*****************************************************************************
uint16_t STL_PIE_RAM_testHandler(const uint32_t interruptNumber)
{

    uint16_t intGroup, groupMask, intMask, testStatus;
    uint32_t prevHandler;
    uint32_t prevVector;

    //
    // Assert the interrupt number is a PIE interrupt.
    // This test will only work for PIE interrupts.
    //
    ASSERT(((uint16_t)(interruptNumber >> 16U) >= 0x20U) &&
           ((uint16_t)(interruptNumber >> 16U) <= 0xDFU));

    //
    // Get the PIE RAM vector entry.
    //
    uint16_t pieEntry = (uint16_t)(((interruptNumber & STL_PIE_RAM_VECT_ID_M) >>
                                    STL_PIE_RAM_VECT_ID_S) * 2U);

    //
    // Configure the default handler for the STL.
    //
    prevHandler = HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR);
    STL_PIE_RAM_configHandler(&STL_PIE_RAM_handler);

    //
    // Save the ISR at entry so we can restore it after injecting an error.
    //
    prevVector = HWREG(PIEVECTTABLE_BASE + pieEntry);

    //
    // Inject a fault at the entry.
    //
    STL_PIE_RAM_injectFault(pieEntry);

    //
    // Enable and force the interruptNumber. This should trigger the mismatch
    // handler. If the handler is not serviced, then this test will FAIL.
    //

    //
    // Determine the PIE interrupt group.
    //
    intGroup = ((uint16_t)(interruptNumber & STL_PIE_RAM_TABLE_COL_M) >>
                STL_PIE_RAM_TABLE_COL_S) - 1U;
    intMask = (uint16_t)1U << ((uint16_t)(interruptNumber &
                                          STL_PIE_RAM_TABLE_ROW_M) - 1U);
    groupMask = (uint16_t)1U << intGroup;

    //
    // Enable the PIE interrupt.
    //
    HWREGH(PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U)) |= intMask;

    //
    // Enable PIE Group Interrupt.
    //
    IER |= groupMask;

    //
    // Reset the global handler flag.
    //
    STL_PIE_RAM_handlerFlag = 0U;

    //
    // Force the PIE interrupt.
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR1 + (intGroup * 2U)) |= intMask;

    //
    // Short delay to ensure the exception was handled.
    //
    SysCtl_delay(5);

    //
    // Check that the PIE error interrupt occurred as expected.
    //
    if(STL_PIE_RAM_handlerFlag == 1U)
    {
        testStatus = STL_PIE_RAM_PASS;
    }
    else
    {
        testStatus = STL_PIE_RAM_FAIL_HANDLER;

        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_PIE_RAM_INT);
    }

    //
    // Restore the interruptNumber in PIE RAM.
    //
    STL_PIE_RAM_restoreVector(pieEntry, (void *)prevVector);

    //
    // Restore the PIEVERRADDR.
    //
    STL_PIE_RAM_configHandler((void *)prevHandler);

    //
    // Acknowledge the PIE group. This must occur or else the PIE group
    // will not be able to service further interrupts.
    //

    //
    // This can also be accomplished in the error handler by looking at the
    // PIECTRL register and determining which group the PIE RAM mismatch
    // occurred for during a vector fetch.
    //
    Interrupt_clearACKGroup(groupMask);
    return(testStatus);
}

//
// End of File
//
