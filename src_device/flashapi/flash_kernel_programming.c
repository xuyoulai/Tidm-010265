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
//!
//! MotorControl SDK
//!
//
// FILE:   flash_kernel_programming.c for F280013x
//
// TITLE:  Flash programming functions
//
//! \addtogroup driver_example_list
//! <h1> Flash Programming with AutoECC, DataAndECC, DataOnly and EccOnly </h1>
//!
//! This example demonstrates how to program Flash using API's following options
//! 1. AutoEcc generation
//! 2. DataOnly and EccOnly
//! 3. DataAndECC
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - None.
//!
//------------------------------------------------------------------------------

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

#include "flash_kernel_programming.h"
#include "flash_kernel_commands.h"

//
// Globals
//
#if defined(_LFU_ENABLE)

uint16 dataBuffer[FLASH_BUFFER_NUM];
FLASH_DATA_STATUS_t   flashDataStatus;

Fapi_StatusType          statusFapiCheck;
Fapi_StatusType          statusFapiProgram;
Fapi_FlashStatusType     statusFapiFlash;
Fapi_FlashStatusWordType statusFapiFlashWord;

// All Flash API functions need to be ran from internal RAM. Place all functions
// that contain Flash API calls in ramfuncs to allow copy from Flash to RAM.
#pragma CODE_SECTION(FLASH_eraseSector, "lfufuncs");

#pragma CODE_SECTION(FLASH_writeBufferData, "lfufuncs");
#pragma CODE_SECTION(FLASH_writeWords, "lfufuncs");
#pragma CODE_SECTION(FLASH_writeSingleWord, "lfufuncs");
#pragma CODE_SECTION(FLASH_writeSingleByte, "lfufuncs");

#pragma CODE_SECTION(FLASH_programAutoECC, "lfufuncs");
#pragma CODE_SECTION(FLASH_programDataOnly, "lfufuncs");

#pragma CODE_SECTION(FLASH_activeFlashBank, "lfufuncs");
#pragma CODE_SECTION(FLASH_clearFSMStatus, "lfufuncs");

//
// FLASH_initFlashPrms()
//
void FLASH_initFlashPrms(void)
{
    uint16_t i = 0;

    for(i = 0; i < FLASH_BUFFER_NUM; i++)
    {
        dataBuffer[i] = 0xFFFF;
    }

    flashDataStatus.ptrDataBuffer= &dataBuffer[0];
}

//
// FLASH_activeFlashBank()
//
Fapi_StatusType FLASH_activeFlashBank(Fapi_FlashBankType FlashBank)
{
    //
    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency(in MHz).
    // This function is required to initialize the Flash API based on System
    // frequency before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is
    // changed.
    //
    statusFapiCheck = Fapi_initializeAPI(FlashTech_CPU0_BASE_ADDRESS,
                                         DEVICE_SYSCLK_FREQ/1000000U);

    if(statusFapiCheck != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    // Initialize the Flash banks and FMC for erase and program operations.
    // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for
    // further Flash operations to be performed on the banks.
    statusFapiCheck = Fapi_setActiveFlashBank(FlashBank);

    if(statusFapiCheck != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    FLASH_clearFSMStatus();

    // Enable program/erase protection for select sectors where this example is
    // located
    // CMDWEPROTA is applicable for sectors 0-31
    // Bits 0-11 of CMDWEPROTB is applicable for sectors 32-127, each bit represents
    // a group of 8 sectors, e.g bit 0 represents sectors 32-39, bit 1 represents
    // sectors 40-47, etc
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, 0x00000000);
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, 0x00000000);

    return(statusFapiCheck);
}   // End of FLASH_getValidBank()

//
// FLASH_eraseSector()
//
Fapi_StatusType FLASH_eraseSector(void)
{
    // Issue ClearMore command
    FLASH_clearFSMStatus();

    // Enable program/erase protection for select sectors where this example is
    // located
    // CMDWEPROTA is applicable for sectors 0-31
    // Bits 0-11 of CMDWEPROTB is applicable for sectors 32-127, each bit represents
    // a group of 8 sectors, e.g bit 0 represents sectors 32-39, bit 1 represents
    // sectors 40-47, etc
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, 0x00000000);
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, 0x00000000);

    // Erase the sector that is programmed in the above example
    // Erase these Sectors
    statusFapiCheck = Fapi_issueBankEraseCommand((uint32_t *)flashDataStatus.sectorAddress);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    if(statusFapiCheck != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    statusFapiFlash = Fapi_getFsmStatus();

    if(statusFapiFlash != 3)
    {
        statusFapiCheck = Fapi_Error_Fail;
        return(statusFapiCheck);
    }

    // Do blank check
    // Blank check is performed on all the sectors that are not protected
    // during Bank erase
    // Verify that Bank 0 is erased.
    // The Erase command itself does a verify as it goes.
    // Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    statusFapiCheck = Fapi_doBlankCheck((uint32_t *)flashDataStatus.sectorAddress,
                                flashDataStatus.sectorLength, &statusFapiFlashWord);

    if(statusFapiCheck != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    return(statusFapiCheck);
}   // End of FLASH_eraseSector()

//
// FLASH_writeData()
//
Fapi_StatusType FLASH_writeBufferData(void)
{
    uint16 i = 0;
    uint16 index = 0;

    for(index = 0; index < flashDataStatus.dataLength; index += 8)
    {
        for(i = 0; i < 8; i++)
        {
            if((index + i) < flashDataStatus.dataLength)
            {
                dataBuffer[i] = *(flashDataStatus.ptrDataAddress + index + i);
            }
            else
            {
                dataBuffer[i] = 0xFFFF;
            }
        }

        flashDataStatus.startAddress = flashDataStatus.flashAddress + index;

        // Program data located in dataBuffer to current page
        if(FLASH_programAutoECC() != Fapi_Status_Success)
        {
            return(statusFapiCheck);
        }
    }

    return(statusFapiCheck);
}   // End of FLASH_writeBufferData

//
//  FLASH_writeSingleWord
//
Fapi_StatusType FLASH_writeWords(uint16_t data)
{
    dataBuffer[0] = data;
    dataBuffer[1] = 0xFFFF;
    dataBuffer[2] = 0xFFFF;
    dataBuffer[3] = 0xFFFF;
    dataBuffer[4] = 0xFFFF;
    dataBuffer[5] = 0xFFFF;
    dataBuffer[6] = 0xFFFF;
    dataBuffer[7] = 0xFFFF;

    // Variables needed for Flash API Functions
    if(FLASH_programAutoECC() != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    return(statusFapiCheck);
}   // end of FLASH_writeSingleWord

//
//  FLASH_writeSingleWord
//
Fapi_StatusType FLASH_writeSingleWord(uint16_t data)
{
    dataBuffer[0] = data;
    dataBuffer[1] = 0xFFFF;
    dataBuffer[2] = 0xFFFF;
    dataBuffer[3] = 0xFFFF;
    dataBuffer[4] = 0xFFFF;
    dataBuffer[5] = 0xFFFF;
    dataBuffer[6] = 0xFFFF;
    dataBuffer[7] = 0xFFFF;

    // Variables needed for Flash API Functions
//    FLASH_programDataOnly();
    if(FLASH_programAutoECC() != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    return(statusFapiCheck);
}   // end of FLASH_writeSingleWord

//
//  FLASH_writeSingleByte
//
Fapi_StatusType FLASH_writeSingleByte(uint16_t data)
{
    dataBuffer[0] = data | 0xFF00;
    dataBuffer[1] = 0xFFFF;
    dataBuffer[2] = 0xFFFF;
    dataBuffer[3] = 0xFFFF;
    dataBuffer[4] = 0xFFFF;
    dataBuffer[5] = 0xFFFF;
    dataBuffer[6] = 0xFFFF;
    dataBuffer[7] = 0xFFFF;

    // Variables needed for Flash API Functions
    FLASH_programDataOnly();

    return(statusFapiCheck);
}   // end of FLASH_writeSingleByte

//
// FLASH_programAutoECC()
//      Program data in Flash using "AutoEccGeneration" option.
//      Flash API functions used in this function are executed from RAM
//
Fapi_StatusType FLASH_programAutoECC(void)
{
    //
    // A data buffer of max 8 16-bit words can be supplied to the program
    // function.
    // Each word is programmed until the whole buffer is programmed or a
    // problem is found. However to program a buffer that has more than 8
    // words, program function can be called in a loop to program 8 words for
    // each loop iteration until the whole buffer is programmed.
    //
    // Remember that the main array flash programming must be aligned to
    // 64-bit address boundaries and each 64 bit word may only be programmed
    // once per write/erase cycle.  Meaning the length of the data buffer
    // (3rd parameter for Fapi_issueProgrammingCommand() function) passed
    // to the program function can only be either 4 or 8.
    //
    // Program data in Flash using "AutoEccGeneration" option.
    // When AutoEccGeneration option is used, Flash API calculates ECC for the
    // given 64-bit data and programs it along with the 64-bit main array data.
    // Note that any unprovided data with in a 64-bit data slice
    // will be assumed as 1s for calculating ECC and will be programmed.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify
    // reasons.
    //
    // Monitor ECC address for the sector below while programming with
    // AutoEcc mode.
    //
    // In this example, the number of bytes specified in the flash buffer
    // are programmed in the flash sector below along with auto-generated
    // ECC.
    //
    FLASH_clearFSMStatus();

    // Enable program/erase protection for select sectors where this example is
    // located
    // CMDWEPROTA is applicable for sectors 0-31
    // Bits 0-11 of CMDWEPROTB is applicable for sectors 32-127, each bit represents
    // a group of 8 sectors, e.g bit 0 represents sectors 32-39, bit 1 represents
    // sectors 40-47, etc
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, 0x00000000);
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, 0x00000000);

    //program 8 words at once, 128-bits
    statusFapiCheck = Fapi_issueProgrammingCommand((uint32_t *)flashDataStatus.startAddress,
                                                   (uint16_t *)flashDataStatus.ptrDataBuffer,
                                                   8, 0, 0, Fapi_AutoEccGeneration);

    // Wait until the Flash program operation is over
    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
    {
    }

    if(statusFapiCheck != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    // Read FMSTAT register contents to know the status of FSM after
    // program command to see if there are any program operation related
    // errors
    statusFapiFlash = Fapi_getFsmStatus();

    if(statusFapiFlash != 0x03)
    {
        // oFlashStatus will be 17 when there is a program error due to
        // the sector having protection enabled. In this example,
        // protections are enabled for sectors in which the example
        // itself is written. We don't want to write over the program
        // itself so the protection is enabled. Due to this, we will
        // see an oFlashStatus of 17.
        if(statusFapiFlash != 0x11)
        {
            statusFapiCheck = Fapi_Error_Fail;
            return(statusFapiCheck);
        }
    }

    if(statusFapiFlash != 0x11)
    {
        // Verify the programmed values.  Check for any ECC errors.
        statusFapiCheck = Fapi_doVerify((uint32_t *)flashDataStatus.startAddress,
                                        4, (uint32_t *)(uint32_t)(flashDataStatus.ptrDataBuffer),
                                        &statusFapiFlashWord);

        if(statusFapiCheck != Fapi_Status_Success)
        {
            return(statusFapiCheck);
        }
    }

    return(statusFapiCheck);
}   // End of FLASH_programAutoECC

//
//  FLASH_programDataOnly()
//      Program data in Flash using "DataOnly" option.
//      Flash API functions used in this function are executed from RAM
//
Fapi_StatusType FLASH_programDataOnly(void)
{
    //
    // Program data using "DataOnly" option and ECC using "EccOnly" option.
    //
    // When DataOnly option is used, Flash API will program only the data
    // portion in Flash at the address specified.
    //
    // When EccOnly option is used, Flash API will program only the ECC portion
    // in Flash ECC memory space (Flash main array address should be provided
    // for this function and not the corresponding ECC address).
    // Fapi_calculateEcc is used to calculate the corresponding ECC of the data.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify
    // reasons.
    //
    // In this example, 0x100 bytes are programmed in Flash Sector6
    // along with the specified ECC.
    //
    FLASH_clearFSMStatus();

    // Enable program/erase protection for select sectors where this example is
    // located
    // CMDWEPROTA is applicable for sectors 0-31
    // Bits 0-11 of CMDWEPROTB is applicable for sectors 32-127, each bit represents
    // a group of 8 sectors, e.g bit 0 represents sectors 32-39, bit 1 represents
    // sectors 40-47, etc
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, 0x00000000);
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, 0x00000000);

    statusFapiCheck = Fapi_issueProgrammingCommand((uint32_t *)flashDataStatus.startAddress,
                                                   (uint16_t *)flashDataStatus.ptrDataBuffer,
                                                   8, 0, 0, Fapi_DataOnly);

    //
    // Wait until the Flash program operation is over
    //
    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

    if(statusFapiCheck != Fapi_Status_Success)
    {
        return(statusFapiCheck);
    }

    // Read FMSTAT register contents to know the status of FSM after
    // program command to see if there are any program operation related
    // errors
    statusFapiFlash = Fapi_getFsmStatus();

    if(statusFapiFlash != 3)
    {
        statusFapiCheck = Fapi_Error_Fail;
        return(statusFapiCheck);
    }

    return(statusFapiCheck);
}   // End of FLASH_programDataOnly()


Fapi_StatusType FLASH_clearFSMStatus(void)
{
    // Wait until FSM is done with the previous flash operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    statusFapiFlash = Fapi_getFsmStatus();

    if(statusFapiFlash != 0)
    {
        // Clear the Status register
        statusFapiCheck = Fapi_issueAsyncCommand(Fapi_ClearStatus);

        // Wait until status is cleared
        while (Fapi_getFsmStatus() != 0) {}

        // Check Flash API documentation for possible errors
        if(statusFapiCheck != Fapi_Status_Success)
        {
            return(statusFapiCheck);
        }
    }

    return(statusFapiCheck);
}

//
// Deal this error if an API error is found
//
void FLASH_handleError(Fapi_StatusType status)
{
    //
    //  Error code will be in the status parameter
    //
    __asm("    ESTOP0");
}


//******************************************************************************
// Deal this error if FMSTAT fail occurs
//******************************************************************************
void FLASH_failFMSTAT(void)
{
    __asm("    ESTOP0");
}


//******************************************************************************
// Deal this error if ECC fail occurs
//******************************************************************************
void FLASH_failECC(void)
{
    __asm("    ESTOP0");
}

#endif  // _LFU_ENABLE

//
// End of File
//
