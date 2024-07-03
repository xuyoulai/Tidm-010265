//###########################################################################
//
// FILE:    bootloader_parallel.c
//
// TITLE:   Parallel Port I/O bootloader
//
// Functions involved in running Parallel I/O bootloader
//
// -----------------------------------------------------------------------
// |Opt No.|  BOOTDEF      |  Dx GPIO            |  DSP Ctrl | Host Ctrl |
// -----------------------------------------------------------------------
// |  0    |  0x00         | 0,1,3,4,5,7,28,29   |  224      |  242      |
// |  1    |  0x40         | 0-7                 |  12       |  13       |
// |  2    |  0x40         | 0-7                 |  16       |  29       |
// -----------------------------------------------------------------------
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
#include "bootloader_parallel.h"

static uint32_t Parallel_bootMode;

//
// Function Prototypes
//
uint16_t Parallel_GetWordData_8bit(void);
void Parallel_GPIOSelect(uint32_t  bootMode);

/**
* Parallel_Boot - This module is the main Parallel boot routine. It will load
*                 code via GP I/Os. This boot mode accepts 8-bit data. 8-bit
*                 data is expected to be the order LSB followed by MSB.
*
*                 This function returns a entry point address back
*                 to the system initialization routine which in turn calls
*                 the ExitBoot routine.
*
*
* \brief Parallel Boot
*
* Design: did_parallel_boot_algo did_boot_first_instance_algo)
* Requirement: REQ_TAG(C2000BROM-204), REQ_TAG(C2000BROM-210)
*
* Start parallel Boot
*
*/
uint32_t Parallel_Boot(uint32_t  BootMode)
{
    uint32_t appEntryAddress = 0xFFFFFFFFUL;
    uint16_t entryAddress;
    uint16_t wordData;

    Parallel_bootMode = BootMode;

    //
    // Setup for Parallel boot
    //
    Parallel_GPIOSelect(BootMode);

    //
    // CPU1 Patch/Escape Point 12
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_12;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Check for the key value. This version only
    // supports 8-bit data.
    //
    GetWordData = Parallel_GetWordData_8bit;
    wordData = GetWordData();

    if(wordData != BROM_EIGHT_BIT_HEADER)
    {
       return FLASH_ENTRY_POINT;
    }

    //
    // Read and discard the reserved words
    //
    ReadReservedFn();

    //
    // Get the entry point address
    //
    appEntryAddress = GetLongData();

    //
    // Load the data
    //
    CopyData();

    return appEntryAddress;
}

//
// Parallel_GetWordData_8bit - The 8-bit function is used if the input stream is
//                             an 8-bit input stream and the upper 8-bits of the
//                             GPIO port are ignored.  In the 8-bit case the
//                             first fetches the LSB and then the MSB from the
//                             GPIO port. These two bytes are then put together
//                             to form a single 16-bit word that is then passed
//                             back to the host. Note that in this case, the
//                             input stream from the host is in the order LSB
//                             followed by MSB
//
uint16_t Parallel_GetWordData_8bit(void)
{
    uint16_t wordData = 0;

    switch (Parallel_bootMode)
    {
        case PARALLEL_BOOT_ALT2:
            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(16, 0U);
            while(GPIO_readPin(29) != 0U){}

            //
            // Get LSB (pins 0-7)
            //
            wordData = (uint16_t)(GPIO_readPortData(GPIO_PORT_A) & 0xFFU);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(16, 1U);
            while(GPIO_readPin(29) != 1U){}

            //
            // Fetch the MSB.
            //
            wordData = wordData & 0x00FFU;

            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(16, 0U);
            while(GPIO_readPin(29) != 0U){}

            wordData |= (uint16_t)((GPIO_readPortData(GPIO_PORT_A) & 0xFFU) << 8U);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(16, 1U);
            while(GPIO_readPin(29) != 1U){}
            break;

        case PARALLEL_BOOT_ALT1:
            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(12, 0U);
            while(GPIO_readPin(13) != 0U){}

            //
            // Get LSB
            //
            wordData = (uint16_t)(GPIO_readPortData(GPIO_PORT_A) & 0xFFU);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(12, 1U);
            while(GPIO_readPin(13) != 1U){}

            //
            // Fetch the MSB.
            //
            wordData = wordData & 0x00FFU;

            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(12, 0U);
            while(GPIO_readPin(13) != 0U){}

            wordData |= (uint16_t)((GPIO_readPortData(GPIO_PORT_A) & 0xFFU) << 8U);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(12, 1U);
            while(GPIO_readPin(13) != 1U){}
            break;

        case PARALLEL_BOOT:
        default:
        {
            uint32_t tempData = 0UL;
            uint32_t bits01   = 0UL;
            uint32_t bits234  = 0UL;
            uint32_t bit5     = 0UL;
            uint32_t bits67   = 0UL;

            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(224, 0U);
            while(GPIO_readPin(242) != 0U){}

            //
            // Get LSB (pins 0,1,3,4,5,7,28,29)
            //
            tempData = GPIO_readPortData(GPIO_PORT_A) & (~0xCFFFFF44UL);
            bits01   = (tempData  & 0x00000003UL); // data 0,1
            bits234  = ((tempData & 0x00000038UL) >> 1UL); // data 3,4,5
            bit5     = ((tempData & 0x00000080UL) >> 2UL); // data 7
            bits67   = ((tempData & 0x30000000UL) >> 22UL); // data 28, 29

            wordData = (uint16_t)((bits01 | bits234 | bit5 | bits67) & 0xFFU);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(224, 1U);
            while(GPIO_readPin(242) != 1U){}

            //
            // Fetch the MSB.
            //
            wordData = wordData & 0x00FFU;

            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(224, 0U);
            while(GPIO_readPin(242) != 0U){}

            tempData = GPIO_readPortData(GPIO_PORT_A) & (~0xCFFFFF44UL);
            bits01   = (tempData  & 0x00000003UL); // data 0,1
            bits234  = ((tempData & 0x00000038UL) >> 1UL); // data 3,4,5
            bit5     = ((tempData & 0x00000080UL) >> 2UL); // data 7
            bits67   = ((tempData & 0x30000000UL) >> 22UL); // data 28, 29

            wordData |= (uint16_t)(((bits01 | bits234 | bit5 | bits67) & 0xFFU) << 8U);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(224, 1U);
            while(GPIO_readPin(242) != 1U){}
            break;
        }
    }
    return wordData;
}

/**
* Parallel_GPIOSelect - Configure the GPIOs used for Parallel IO bootloader
*
* HOST_CTRL_GPIO is an input control from the Host
* to the DSP Ack/Rdy
* - may require an external pull-up
*
* DSP_CTRL_GPIO is an output from the DSP Ack/Rdy
* - may require an external pull-up for host to correctly
*   read "1" initially.
*
* DSP_CTRL_GPIO set to 1 initially
* 0 = input   1 = output
*
*
* \brief Parallel Boot
*
* Design: \ref did_parallell_boot_options_algo
* Requirement: REQ_TAG(C2000BROM-204)
*
* Parallel Boot
*
*/
void Parallel_GPIOSelect(uint32_t  bootMode)
{
    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_H, 0xFFFFFFFFUL);

    switch (bootMode)
    {
        case PARALLEL_BOOT_ALT2:
            //
            // Enable Pull-ups on GPIO 0-7
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD)   &= (uint32_t) 0xFFFFFF00UL;
            EDIS;

            //
            // Configure 0-7, 16, 29 as GPIO
            //
            GPIO_setPinConfig(GPIO_0_GPIO0);
            GPIO_setPinConfig(GPIO_1_GPIO1);
            GPIO_setPinConfig(GPIO_2_GPIO2);
            GPIO_setPinConfig(GPIO_3_GPIO3);
            GPIO_setPinConfig(GPIO_4_GPIO4);
            GPIO_setPinConfig(GPIO_5_GPIO5);
            GPIO_setPinConfig(GPIO_6_GPIO6);
            GPIO_setPinConfig(GPIO_7_GPIO7);
            GPIO_setPinConfig(GPIO_16_GPIO16);
            GPIO_setPinConfig(GPIO_29_GPIO29);

            //
            // Configure GPIO0-GPIO7, GPIO29 as input
            // Configure GPIO16 as output
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   &= (uint32_t) 0xDFFFFFF00UL;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   |= (uint32_t) 0x00010000UL;
            EDIS;
            break;

        case PARALLEL_BOOT_ALT1:
            //
            // Enable Pull-ups on GPIO 0-7
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD)   &= (uint32_t) 0xFFFFFF00UL;
            EDIS;

            GPIO_setAnalogMode(12,GPIO_ANALOG_DISABLED);
            GPIO_setAnalogMode(13,GPIO_ANALOG_DISABLED);

            //
            // Configure GPIO 0-7, 13, 12 as GPIO
            //
            GPIO_setPinConfig(GPIO_0_GPIO0);
            GPIO_setPinConfig(GPIO_1_GPIO1);
            GPIO_setPinConfig(GPIO_2_GPIO2);
            GPIO_setPinConfig(GPIO_3_GPIO3);
            GPIO_setPinConfig(GPIO_4_GPIO4);
            GPIO_setPinConfig(GPIO_5_GPIO5);
            GPIO_setPinConfig(GPIO_6_GPIO6);
            GPIO_setPinConfig(GPIO_7_GPIO7);
            GPIO_setPinConfig(GPIO_13_GPIO13);
            GPIO_setPinConfig(GPIO_12_GPIO12);

            //
            // Configure GPIO0-GPIO7, GPIO13 as input
            // Configure GPIO12 as output
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   &= (uint32_t) 0xFFFFDF00UL;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   |= (uint32_t) 0x00001000UL;
            EDIS;
            break;

        case PARALLEL_BOOT:
        default:
            //
            // Enable Pull-ups on GPIO 0,1,3,4,5,7,28,29
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD)   &= (uint32_t) 0xCFFFFF44UL;
            EDIS;

            GPIO_setAnalogMode(28,GPIO_ANALOG_DISABLED);
            GPIO_setAnalogMode(224,GPIO_ANALOG_DISABLED);
            GPIO_setAnalogMode(242,GPIO_ANALOG_DISABLED);

            //
            // Configure 0,1,3,4,5,7,28,29,224,242 as GPIO
            //
            GPIO_setPinConfig(GPIO_0_GPIO0);
            GPIO_setPinConfig(GPIO_1_GPIO1);
            GPIO_setPinConfig(GPIO_3_GPIO3);
            GPIO_setPinConfig(GPIO_4_GPIO4);
            GPIO_setPinConfig(GPIO_5_GPIO5);
            GPIO_setPinConfig(GPIO_7_GPIO7);
            GPIO_setPinConfig(GPIO_28_GPIO28);
            GPIO_setPinConfig(GPIO_29_GPIO29);
            GPIO_setPinConfig(GPIO_224_GPIO224);
            GPIO_setPinConfig(GPIO_242_GPIO242);

            //
            // Configure GPIO 0,1,3,4,5,7,28,29,242 as input
            // Configure GPIO224 as output
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   &= (uint32_t) 0xCFFFFF44UL;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPHDIR)   &= (uint32_t) 0xFFFBFFFFUL;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPHDIR)   |= (uint32_t) 0x00000001UL;
            EDIS;
            break;
    }
}
