//###########################################################################
//
// FILE:    cpu1brom_system_boot.c
//
// TITLE:   CPU1 System Boot
//
// CPU1 System Initialization and associated functions
//
//###########################################################################
// $TI Release: $
// $Release Date:  $
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

/**
* CPU1BROM_configureHardwarePatching - Configure hardware patching registers
*
* \brief Hardware Assisted Patching function
* \param startAddress - The starting address in TI OTP (TRIM) where the patch
*                       table including the hardware address and data code starts
*
* Design: \ref did_hw_assisted_patching_algo
* Requirement: REQ_TAG(C2000BROM-365)
*
* Hardware Assisted Patching function
*
*/
void CPU1BROM_configureHardwarePatching(uint32_t startAddress)
{
    uint32_t tableStartAddress;
    uint32_t patchAddressConfig;
    uint32_t patchDataConfig;
    uint32_t sysctlRegPatchAddressOffset = SYSCTL_O_BROMPATCHADDR0;
    uint32_t sysctlRegDataAddressOffset = SYSCTL_O_BROMPATCHDATA0;
    uint16_t i;

    //
    // Clear ECC errors
    //
    EALLOW;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR)  |= MEMCFG_CERRCLR_CPURDERR;
    EDIS;

    if((HW_PATCH_TABLE_KEY == TI_OTP_HW_PATCH_TABLE_KEY) && (NO_ERROR == HWREAD_DED_STATUS))
    {
        tableStartAddress = startAddress;
        //
        // Initialize all 8 hardware patch registers (each have address and data registers)
        // HW patch only gets enabled once both the address and data register are written.
        //
        for(i = 0U; i < CPU1BROM_HW_NUMBER_OF_PATCHES; i++)
        {
            //
            // Clear ECC errors
            //
            EALLOW;
            HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
            HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
            EDIS;

            patchAddressConfig = HWREG(tableStartAddress);
            patchDataConfig = HWREG(tableStartAddress + CPU1BROM_HW_PATCH_TABLE_DATA_OFFSET);

            //
            // If HW patch "address" data isn't set to be populated, skip configuration and leave
            // HW reg as default value of zero to disable use of the HW patch register instance
            //
            if((patchAddressConfig != CPU1BROM_HW_PATCH_SKIP_CONFIG) && (NO_ERROR == HWREAD_DED_STATUS))
            {
                //
                // Note these registers can only be written when running from boot ROM
                //
                HWREG(CPUSYS_BASE + sysctlRegPatchAddressOffset) = patchAddressConfig;
                HWREG(CPUSYS_BASE + sysctlRegDataAddressOffset) = patchDataConfig;
            }

            tableStartAddress += CPU1BROM_HW_PATCH_TABLE_NEXT_ENTRY;
            sysctlRegPatchAddressOffset += CPU1BROM_HW_PATCH_REG_NEXT_ENTRY;
            sysctlRegDataAddressOffset += CPU1BROM_HW_PATCH_REG_NEXT_ENTRY;
        }
    }
}
