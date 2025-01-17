//###########################################################################
//
// FILE:    cpu1brom_interrupts.c
//
// TITLE:   CPU1 Boot ROM interrupt handlers
//
// Interrupt handlers to handle itrap, NMI, and PIE vector interrupts
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
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
// Included Files
//
#include "cpu1bootrom.h"

//
// Globals
//
extern uint32_t CPU1BROM_bootStatus;
extern volatile uint16_t CPU1BROM_nmiStatus;
uint32_t CPU1BROM_itrapAddress;

//
// Function Prototypes
//
void load_itrap_address (uint32_t *itrapaddress);
__interrupt void CPU1BROM_pieVectorMismatchHandler(void);

//
// CPU1BROM_pieVectorMismatchHandler - Function to handle PIE vector mismatch
//                                     during CPU1 Boot
//
#pragma CODE_SECTION(CPU1BROM_pieVectorMismatchHandler, "CPU1BROM_PIE_MISMATCH_HANDLER")
__interrupt void CPU1BROM_pieVectorMismatchHandler(void)
{
    uint32_t entryAddress = 0xFFFFFFFFU;

    //
    // Update boot status
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_GOT_A_PIE_MISMATCH;

    //
    // Get the pie vector fetch error handler address to determine action
    //
    EALLOW;
    entryAddress = HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR);
    EDIS;

    if(entryAddress != 0x003FFFFFUL)
    {
        //
        // Address isn't the default reset value, call error handler routine
        //
        ((void (*)(void))entryAddress)();
    }
    else
    {
        //
        // The address is the default reset value; let watchdog reset the
        // system if the error happened not because of NMI fetch, else NMIWD
        // will reset the device
        //
        if(HWREGH(NMI_BASE + NMI_O_FLG) == 0x0U)
        {
            //
            // If watchdog is not already enabled then enable it
            //
            if (false == SysCtl_isWatchdogEnabled())
            {
                SysCtl_enableWatchdog();
            }
        }

        //
        // Wait for NMIWD reset
        //
        while (true)
        {
        }
    }
}

//
// CPU1BROM_nmiHandler - Function to handle NMI interrupts during CPU1 boot
//
#pragma CODE_SECTION(CPU1BROM_nmiHandler, "CPU1BROM_NMI_HANDLER")
__interrupt void CPU1BROM_nmiHandler(void)
{
    uint16_t entryAddress;

    //
    // CPU1 Patch/Escape Point 8
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_8;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Check to see if NMI is triggered
    //
    CPU1BROM_nmiStatus = HWREGH(NMI_BASE + NMI_O_FLG);
    do
    {
        //
        // Check for Clock Fail NMI
        //
        if((CPU1BROM_nmiStatus & NMI_FLG_CLOCKFAIL) == NMI_FLG_CLOCKFAIL)
        {
            //
            // Update boot status
            //
            CPU1BROM_bootStatus |= CPU1_BOOTROM_GOT_A_MCLK_NMI;

            //
            // Clear Clock Fail bit (bit 1)
            //
            CPU1BROM_nmiStatus &= ~(NMI_FLG_CLOCKFAIL);

            EALLOW;
            HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_CLOCKFAIL;
            EDIS;

            CPU1BROM_nmiStatus = HWREGH(NMI_BASE + NMI_O_FLG);

            //
            // Clear Global NMI flag (if set)
            //
            if(CPU1BROM_nmiStatus == NMI_FLG_NMIINT)
            {
                //
                // Clear the NMIINT flag and return because there is no
                // other NMI set
                //
                CPU1BROM_nmiStatus &= ~(NMI_FLG_NMIINT);

                EALLOW;
                HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_NMIINT;
                EDIS;

                return;
            }
        }
//        //
//        // Check for Configurable Logic NMI
//        //
//        if((CPU1BROM_nmiStatus & NMI_FLG_RLNMI) == NMI_FLG_RLNMI)
//        {
//            //
//            // Update boot status
//            //
//            CPU1BROM_bootStatus |= CPU1_BOOTROM_GOT_A_RL_NMI;
//
//            //
//            // Clear flag
//            //
//            CPU1BROM_nmiStatus &= ~(NMI_FLG_RLNMI);
//
//            EALLOW;
//            HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_RLNMI;
//            EDIS;
//        }
//
//        //
//        // Check for ERAD NMI
//        //
//        if((CPU1BROM_nmiStatus & NMI_FLG_SYSDBGNMI) == NMI_FLG_SYSDBGNMI)
//        {
//            //
//            // Update boot status
//            //
//            CPU1BROM_bootStatus |= CPU1_BOOTROM_GOT_AN_ERAD_NMI;
//
//            //
//            // Clear flag
//            //
//            CPU1BROM_nmiStatus &= ~(NMI_FLG_SYSDBGNMI);
//
//            EALLOW;
//            HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_SYSDBGNMI;
//            EDIS;
//        }
        //
        // Check for RAM/Flash Uncorrectable Error NMI
        //
        if((CPU1BROM_nmiStatus & NMI_FLG_UNCERR) == NMI_FLG_UNCERR)
        {
            //
            // Update boot status
            //
            CPU1BROM_bootStatus |= CPU1_BOOTROM_GOT_A_MEM_UNCERR_NMI;

            EALLOW;
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT) |= (MEMCFG_DXINIT_INIT_M0 | MEMCFG_DXINIT_INIT_M1 | MEMCFG_DXINIT_INIT_PIEVECT);
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT) |= (MEMCFG_LSXINIT_INIT_LS0 | MEMCFG_LSXINIT_INIT_LS1);
            EDIS;

            //
            // Wait for RAM inits to complete
            //
            asm(" MOV  @T,   #0x1068 ");;
            asm(" RPT  @T || NOP ");;

            //
            // If debugger connected, pause execution for debug
            //
            asm("   ESTOP0");

            //
            // Let NMIWD reset the device
            //
            while(true)
            {
            }
        }

        //
        // Clear Global NMI flag
        //
        CPU1BROM_nmiStatus &= ~(NMI_FLG_NMIINT);

        EALLOW;
        HWREGH(NMI_BASE + NMI_O_FLGCLR) = NMI_FLG_NMIINT;
        EDIS;

        CPU1BROM_nmiStatus = HWREGH(NMI_BASE + NMI_O_FLG);

    }while((CPU1BROM_nmiStatus & NMI_FLG_NMIINT) == NMI_FLG_NMIINT);
}

//
// CPU1BROM_itrapISR - Function called whenever an illegal interrupt occurs
//                     on CPU1
//
#pragma CODE_SECTION(CPU1BROM_itrapISR, "CPU1BROM_ITRAP_HANDLER")
__interrupt void CPU1BROM_itrapISR(void)
{
    //
    // Update boot status
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_GOT_ITRAP;

    //
    // Clear global address variable. Accessing the global variable locally
    // sets the DP pointer for the below inlined assembly function
    // (load_itrap_address)
    //
    CPU1BROM_itrapAddress = 0xFFFFFFFFU;

    //
    // Get the iTrap address (return address from interrupt) stored in stack.
    // This particular return address stored in stack will change according to
    // compiler optimizations.
    //
    load_itrap_address(&CPU1BROM_itrapAddress);

    //
    // Return address from interrupt will be a PC location where an illegal
    // instruction is executed, so adjust address by 1
    //
    CPU1BROM_itrapAddress -= 1U;

    //
    // If watchdog is not already enabled then enable it
    //
    if (false == SysCtl_isWatchdogEnabled())
    {
        SysCtl_enableWatchdog();
    }
    //
    // If debugger connected, pause execution for debug
    //
    asm("   ESTOP0");

    //
    // Wait for reset via watchdog
    //
    while(true)
    {
    }
}

//
// End of File
//
