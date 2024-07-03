//###########################################################################
//
// FILE:  stl_sp.c
//
// TITLE: Diagnostic Library Stack Pointer software module source
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
#include "stl_sp.h"
#include "stl_util.h"
#include "interrupt.h"

//
// Defines
//
#define STL_SP_WP1_BASE       0x0828U
#define STL_SP_WP0_BASE       0x0848U
#define STL_SP_MASK           0x0U
#define STL_SP_REF            0x2U
#define STL_SP_EVT_CNTL       0x6U
#define STL_SP_EVT_ID         0x7U
#define STL_SP_RANGE_MASK     0x0007U
#define STL_SP_CNTL_DELAY     __asm(" RPT #1 || NOP")

//
// Number of words before stack end address that the watchpoint is to be set at
//
#define STL_SP_MARGIN         50U

//
// Globals
//
volatile static uint16_t STL_SP_interruptGeneratedFlag = 0U;

//*****************************************************************************
//
// STL_SP_testISR - The interrupt service routine called when stack pointer
//                  is outside valid range
//
//*****************************************************************************
__interrupt void STL_SP_testISR(void)
{
    //
    // Set flag to indicate that interrupt was generated
    //
    STL_SP_interruptGeneratedFlag = 1U;

    //
    // Acknowledge this interrupt located in group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//*****************************************************************************
//
// STL_SP_configSP(const STL_SP_Handle spHandle)
//
//*****************************************************************************
uint16_t STL_SP_configSP(const STL_SP_Handle spHandle)
{
    uint32_t wpBase;
    uint16_t testStatus;

    //
    // Save the previous timer interrupt vector in the structure.
    //
    spHandle->prevIntVector = HWREG(PIEVECTTABLE_BASE +  PIEVECTTABLE_O_RTOS);

    //
    // Disable RTOS Interrupt
    //
    Interrupt_disable(INT_RTOS);

    //
    // Assign an ISR to respond to stack pointer corruption
    //
    Interrupt_register(INT_RTOS, STL_SP_testISR);

    //
    // Enable RTOS Interrupt
    //
    Interrupt_enable(INT_RTOS);

    //
    // Determine which watchpoint to set up
    //
    if(spHandle->watchpoint == STL_SP_WP0)
    {
        wpBase = STL_SP_WP0_BASE;
    }
    else
    {
        wpBase = STL_SP_WP1_BASE;
    }

    //
    // Initialize aligned reference address
    //
    spHandle->stackRefAddress = (spHandle->endAddress - STL_SP_MARGIN) &
                                                           ~STL_SP_RANGE_MASK;

    //
    // Claim control of watchpoint by setting bits 1-0 of the event control
    // register to 01b
    //
    EALLOW;
    HWREGH(wpBase + STL_SP_EVT_CNTL) = 0x0001U;
    EDIS;

    //
    // Wait for previous instruction to settle
    //
    STL_SP_CNTL_DELAY;
    STL_SP_CNTL_DELAY;

    //
    // Exit and return fail if specified stack addresses are not valid or if
    // application doesn't own watch point. Application owns the watchpoint if
    // bits 15-14 of the event ID register are 01b.
    //
    if((((HWREGH(wpBase + STL_SP_EVT_ID)) & 0xC000U) != 0x4000U) ||
       (spHandle->stackRefAddress < spHandle->startAddress) ||
       (spHandle->stackRefAddress > spHandle->endAddress))
    {
        STL_Util_setErrorFlag(STL_UTIL_SP_CORRUPT);
        testStatus = STL_SP_CONFIG_FAIL;
    }
    else
    {
        EALLOW;

        //
        // Assign the mask for unsafe stack pointer ranges
        //
        HWREG(wpBase + STL_SP_MASK) = STL_SP_RANGE_MASK;

        //
        // Assign reference address
        //
        spHandle->stackRefAddress = spHandle->stackRefAddress |
                (uint32_t)STL_SP_RANGE_MASK;
        HWREG(wpBase + STL_SP_REF) = spHandle->stackRefAddress;

        //
        // Configure watchpoint event control register to enable watchpoint and
        // monitor data writes
        //
        if(spHandle->watchpoint == STL_SP_WP0)
        {
            //
            // Bits 12-11: 01b - Write watchpoint
            // Bit 9: 0b - No external event qualifier
            // Bit 7: 0b - Single-shot mode
            // Bits 4-2: 010b - Monitor writes on the data write address bus
            // Bits 1-0: 10b - Enable an owned watchpoint
            //
            HWREGH(wpBase + STL_SP_EVT_CNTL) = 0x084AU;
        }
        else
        {
            //
            // Bits 12-11: 01b - Write watchpoint
            // Bit 9: 0b - No external event qualifier
            // Bit 7: 0b - Single-shot mode
            // Bits 4-2: 110b - Monitor writes on the data write address bus
            // Bits 1-0: 10b - Enable an owned watchpoint
            //
            HWREGH(wpBase + STL_SP_EVT_CNTL) = 0x085AU;
        }

        EDIS;
        testStatus = STL_SP_CONFIG_PASS;
    }

    return(testStatus);
}

//*****************************************************************************
//
// STL_SP_writeMonitoredRange(const STL_SP_Handle spHandle)
//
//*****************************************************************************
uint16_t STL_SP_writeMonitoredRange(const STL_SP_Handle spHandle)
{
    uint32_t invalidAddress;
    uint16_t testStatus;
    STL_SP_interruptGeneratedFlag = 0U;

    //
    // Write to an address less than the reference address that is within the
    // monitored range
    //
    EALLOW;
    invalidAddress = spHandle->stackRefAddress - 1U;
    HWREGH(invalidAddress) = 0xFFFFU;
    EDIS;

    SysCtl_delay(5U);

    if(STL_SP_interruptGeneratedFlag == 1U)
    {
        testStatus = STL_SP_PASS;
    }
    else
    {
        STL_Util_setErrorFlag(STL_UTIL_SP);
        testStatus = STL_SP_FAIL;
    }

    return(testStatus);
}

//*****************************************************************************
//
// STL_SP_disableWatchpoint(const STL_SP_Handle spHandle)
//
//*****************************************************************************
void STL_SP_disableWatchpoint(const STL_SP_Handle spHandle)
{
    //
    // Disable watchpoint to inject error
    //
    EALLOW;
    if(spHandle->watchpoint == STL_SP_WP0)
    {
        HWREGH(STL_SP_WP0_BASE + STL_SP_EVT_CNTL) = 0x0000U;
    }
    else
    {
        HWREGH(STL_SP_WP1_BASE + STL_SP_EVT_CNTL) = 0x0000U;
    }
    EDIS;
}

//*****************************************************************************
//
// STL_SP_restoreVector(const STL_SP_Handle spHandle)
//
//*****************************************************************************
void STL_SP_restoreVector(const STL_SP_Handle spHandle)
{
    //
    // Disable the RTOS interrupt in the PIE.
    //
    Interrupt_disable(INT_RTOS);

    //
    // Restore the previous RTOS interrupt vector.
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE +  PIEVECTTABLE_O_RTOS) = spHandle->prevIntVector;
    EDIS;

    //
    // Enable the RTOS interrupt.
    //
    Interrupt_enable(INT_RTOS);
}

//
// End of File
//
