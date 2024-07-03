//###########################################################################
//
// FILE:    bootloader_sci.c
//
// TITLE:   SCI Bootloader Routines
//
// Functions involved in running SCI bootloader
//
// ---------------------------------------------------
// |Opt No.|  BOOTDEF      |  SCITXA    |  SCIRXA    |
// ---------------------------------------------------
//    0    |  0x01(default)|  29        |  28        |
//    1    |  0x21         |  16        |  17        |
//    2    |  0x41         |  8         |  9         |
//    3    |  0x61         |  2         |  3         |
//    4    |  0x81         |  16        |  3         |
// ---------------------------------------------------
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
#include "bootloader_sci.h"

//
// Globals
//
uint16_t SCI_gpioTx;
uint32_t SCI_gpioTxPinConfig;

//
// Function Prototypes
//
uint16_t SCIA_GetWordData(void);
static void SCIBOOT_configure_gpio(uint32_t bootMode);

/**
* SCI_Boot - This module is the main SCI boot routine.
*            It will load code via the SCI-A port.
*
*            It will return a entry point address back
*            to the system initialization routine which in turn calls
*            the ExitBoot routine.
*
*
* \brief SCI Boot
*
* Design: did_sci_boot_algo did_boot_first_instance_algo)
* Requirement: REQ_TAG(C2000BROM-205), REQ_TAG(C2000BROM-210)
*
* Start sci Boot
*
*/
uint32_t SCI_Boot(uint32_t bootMode)
{
    uint32_t appEntryAddress = 0xFFFFFFFFUL;
    uint16_t entryAddress;
    uint16_t byteData;

    //
    // CPU1 Patch/Escape Point 13
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_13;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Assign GetWordData to the SCI-A version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = SCIA_GetWordData;

    //
    // Initialize the SCI-A port for communications
    // with the host.
    //

    //
    // Enable the SCI-A clocks
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4);

    EALLOW;
    HWREGH(SCIA_BASE + SCI_O_FFTX) = SCI_FFTX_SCIRST;

    //
    // 1 stop bit, No parity, 8-bit character
    // No loopback
    //
    HWREGH(SCIA_BASE + SCI_O_CCR) = (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE);
    SCI_setParityMode(SCIA_BASE,SCI_CONFIG_PAR_NONE);

    //
    // Enable TX, RX, Use internal SCICLK
    //
    HWREGH(SCIA_BASE + SCI_O_CTL1) = (SCI_CTL1_TXENA | SCI_CTL1_RXENA);

    //
    // Disable RxErr, Sleep, TX Wake,
    // Disable Rx Interrupt, Tx Interrupt
    //
    HWREGB(SCIA_BASE + SCI_O_CTL2) = 0x0U;

    //
    // Relinquish SCI-A from reset and enable TX/RX
    //
    SCI_enableModule(SCIA_BASE);

    EDIS;

    //
    // GPIO INIT
    //
    SCIBOOT_configure_gpio(bootMode);

    //
    // CPU1 Patch/Escape Point 13
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_13;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Perform autobaud lock with the host.
    // Note that if autobaud never occurs
    // the program will hang in this routine as there
    // is no timeout mechanism included.
    //
    SCI_lockAutobaud(SCIA_BASE);

    //
    // Read data
    //
    byteData = SCI_readCharBlockingNonFIFO(SCIA_BASE);

    //
    // Configure TX pin after autobaud lock
    // (Performed here to allow SCI to double as wait boot)
    //
    GPIO_setPadConfig(SCI_gpioTx,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(SCI_gpioTxPinConfig);

    //
    // Write data to request key
    //
    SCI_writeCharNonBlocking(SCIA_BASE,byteData);

    //
    // If the KeyValue was invalid, abort the load
    // and return the flash entry point.
    //
    if(SCIA_GetWordData() != BROM_EIGHT_BIT_HEADER)
    {
        return FLASH_ENTRY_POINT;
    }

    ReadReservedFn();

    appEntryAddress = GetLongData();

    CopyData();

    return appEntryAddress;
}

//
// SCIA_GetWordData - This routine fetches two bytes from the SCI-A
//                    port and puts them together to form a single
//                    16-bit value.  It is assumed that the host is
//                    sending the data in the order LSB followed by MSB.
//
uint16_t SCIA_GetWordData(void)
{
    uint16_t wordData;
    uint16_t byteData;

    //
    // Fetch the LSB and verify back to the host
    //
    wordData = SCI_readCharBlockingNonFIFO(SCIA_BASE);

    SCI_writeCharNonBlocking(SCIA_BASE,wordData);

    byteData = SCI_readCharBlockingNonFIFO(SCIA_BASE);

    SCI_writeCharNonBlocking(SCIA_BASE,byteData);

    //
    // Form the wordData from the MSB:LSB
    //
    wordData |= (byteData << 8U);

    return wordData;
}

//
// SCIBOOT_configure_gpio - This function configures the required GPIOs for the
//                          specified SCI boot mode
//
static void SCIBOOT_configure_gpio(uint32_t bootMode)
{
    uint16_t gpioRx = 0;
    uint32_t gpioRxPinConfig = 0;
    SCI_gpioTx = 0U;
    SCI_gpioTxPinConfig = 0U;

    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A,0xFFFFFFFFU);

    switch (bootMode)
    {
        case SCI_BOOT_ALT1:
            //
            // GPIO1 = SCIATX
            // GPIO0 = SCIARX
            //
            SCI_gpioTx = 1;
            gpioRx = 0;
            SCI_gpioTxPinConfig = GPIO_1_SCIA_TX;
            gpioRxPinConfig = GPIO_0_SCIA_RX;
            break;

        case SCI_BOOT_ALT2:
            //
            // GPIO8 = SCIATX
            // GPIO9 = SCIARX
            //
            SCI_gpioTx = 8;
            gpioRx = 9;
            SCI_gpioTxPinConfig = GPIO_8_SCIA_TX;
            gpioRxPinConfig = GPIO_9_SCIA_RX;
            break;

        case SCI_BOOT_ALT3:
            //
            // GPIO7 = SCIATX
            // GPIO3 = SCIARX
            //
            SCI_gpioTx = 7;
            gpioRx = 3;
            SCI_gpioTxPinConfig = GPIO_7_SCIA_TX;
            gpioRxPinConfig = GPIO_3_SCIA_RX;
            break;

        case SCI_BOOT_ALT4:
            //
            // GPIO16 = SCIATX
            // GPIO3 = SCIARX
            //
            SCI_gpioTx = 16;
            gpioRx = 3;
            SCI_gpioTxPinConfig = GPIO_16_SCIA_TX;
            gpioRxPinConfig = GPIO_3_SCIA_RX;
            break;

        case SCI_BOOT:
        default:
            //
            // GPIO29 = SCIATX
            // GPIO28 = SCIARX
            //
            SCI_gpioTx = 29;
            gpioRx = 28;
            SCI_gpioTxPinConfig = GPIO_29_SCIA_TX;
            gpioRxPinConfig = GPIO_28_SCIA_RX;

            GPIO_setAnalogMode(28,GPIO_ANALOG_DISABLED);

            break;
    }

    //
    // Configure only the Rx pin so that SCI can receive 'A' for autobaud.
    // Do not configure Tx pin so that SCI can be used as WAIT Boot
    // and only one pin will be driven.
    //

    //
    // Enable pull up on GPIOs pins
    //
    GPIO_setPadConfig(gpioRx,GPIO_PIN_TYPE_PULLUP);

    //
    // Set GPIO configuration for SCI
    //
    GPIO_setPinConfig(gpioRxPinConfig);

    //
    // Configure GPIOs as async pins
    //
    GPIO_setQualificationMode(gpioRx,GPIO_QUAL_ASYNC);
}

//
// End of File
//
