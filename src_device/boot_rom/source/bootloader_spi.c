//###########################################################################
//
// FILE:    bootloader_spi.c
//
// TITLE:   SPI Bootloader routines
//
// Functions involved in running SPI bootloader
//
// -----------------------------------------------------------------------------
// |Opt No.|  BOOTDEF      |  SPIA_SIMO |  SPIA_SOMI |  SPIA_CLK  |  SPIA_STE  |
// -----------------------------------------------------------------------------
// |  1    |  0x06(default)|  2         |  1         |  3         |  5         |
// |  2    |  0x26         |  16        |  1         |  3         |  0         |
// |  3    |  0x46         |  8         |  10        |  9         |  11        |
// |  3    |  0x66         |  8         |  17        |  9         |  11        |
// -----------------------------------------------------------------------------
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
#include "bootloader_spi.h"

//
// Function Prototypes
//
static inline uint16_t SPIA_Transmit_Receive(uint16_t cmdData);
static inline void SPIA_ReservedFn(void);
uint16_t SPIA_GetWordData(void);
uint16_t SPIA_SetAddress_KeyChk(void);
uint32_t SPI_start_Boot(uint16_t gpioSTEA);
void SPIA_Init(void);
uint16_t SPIBOOT_configure_gpio(uint32_t bootMode);

/**
* SPI_Boot - This module is the main SPI boot routine.
*            It will load code via the SPI-A port.
*
*            It will return a entry point address back
*            to the system initialization routine which calls ExitBoot routine.
*
*
* \brief SPI Boot
*
* Design: did_spi_boot_algo did_boot_first_instance_algo)
* Requirement: REQ_TAG(C2000BROM-207), REQ_TAG(C2000BROM-210)
*
* Start SPI Boot
*
*/
uint32_t SPI_Boot(uint32_t bootMode)
{
    uint16_t entryAddress;
    uint16_t gpioSTEA;

    //
    // CPU1 Patch/Escape Point 14
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_14;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Assign GetWordData to the SPI-A version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = SPIA_GetWordData;

    //
    // Initialize SPI-A
    //
    SPIA_Init();

    //
    // Setup SPI GPIOs
    //
    gpioSTEA = SPIBOOT_configure_gpio(bootMode);

    //
    // CPU1 Patch/Escape Point 14
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_14;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    return SPI_start_Boot(gpioSTEA);
}

//
// SPI_start_Boot - Function to begin transmission and copying of data into
//                  device memory
//
uint32_t SPI_start_Boot(uint16_t gpioSTEA)
{
    uint32_t appEntryAddress = 0xFFFFFFFFUL;

    //
    // Enable EEPROM and send EEPROM Read Command
    //
    (void)SPIA_Transmit_Receive(0x0300U);

    //
    // Send Starting Address for Serial EEPROM (16-bit - 0x0000,0000)
    // or Serial Flash (24-bit - 0x0000,0000,0000)
    // Then check for 0x08AA data header, else go to flash
    //
    if(SPIA_SetAddress_KeyChk() != BROM_EIGHT_BIT_HEADER)
    {
        return FLASH_ENTRY_POINT;
    }

    //
    // Check for Clock speed change and reserved words
    //
    SPIA_ReservedFn();

    //
    // Get point of entry address after load
    //
    appEntryAddress = GetLongData();

    //
    // Receive and copy one or more code sections to destination addresses
    //
    CopyData();

    //
    // Disable EEPROM chip enable - high
    // Chip enable - high
    //
    GPIO_writePin(gpioSTEA, 1U);

    return(appEntryAddress);
}

//
// SPIA_SetAddress_KeyChk - This routine sends either a 16-bit address to
//                          Serial EEPROM or a 24-bit address to Serial
//                          FLASH. It then fetches the 2 bytes that make
//                          up the key value  from the SPI-A port and
//                          puts them together to form a single
//                          16-bit value.  It is assumed that the host is
//                          sending the data in the form MSB:LSB.
//
uint16_t SPIA_SetAddress_KeyChk(void)
{
    uint16_t keyValue;

    //
    // Transmit first byte of Serial Memory address
    //
    (void)SPIA_Transmit_Receive(0x0000U);

    //
    // Transmit second byte of Serial Memory address
    //
    (void)SPIA_Transmit_Receive(0x0000U);

    //
    // Transmit third byte of  Serial Memory address (0x00) if using Serial
    // Flash or receive first byte of key value if using Serial EEPROM.
    //
    keyValue = SPIA_Transmit_Receive(0x0000U);

    //
    // If previously received LSB of key value (Serial EEPROM), then fetch
    // MSB of key value
    //
    if(keyValue == 0x00AAU)
    {
        keyValue |= (SPIA_Transmit_Receive(0x0000U) << 8U);
    }
    else
    {
        //
        // Serial Flash is being used - keyValue will be received after 24-bit
        // address in the next 2 bytes
        // Fetch Key Value LSB (after 24-bit addressing)
        //
        keyValue = SPIA_Transmit_Receive(0x0000U);

        //
        // Fetch Key Value MSB (after 24-bit addressing)
        //
        keyValue |= (SPIA_Transmit_Receive(0x0000U) << 8U);
    }

    return(keyValue);
}

//
// SPIA_Transmit_Receive - Send a byte/words through SPI transmit channel
//
static inline uint16_t SPIA_Transmit_Receive(uint16_t cmdData)
{
    uint16_t recvData;

    SPI_writeDataNonBlocking(SPIA_BASE,cmdData);

    while(SPI_getInterruptStatus(SPIA_BASE) != SPI_INT_RX_DATA_TX_EMPTY)
    {
    }

    recvData = SPI_readDataNonBlocking(SPIA_BASE);

    return(recvData);
}

//
// SPIA_ReservedFn - This function reads 8 reserved words in the header.
//                   The first word has parameters for LOSPCP
//                   and SPIBRR register 0xMSB:LSB, LSB = is a three
//                   bit field for LOSPCP change MSB = is a 6bit field
//                   for SPIBRR register update
//
//                   If either byte is the default value of the register
//                   then no speed change occurs.  The default values
//                   are LOSPCP = 0x02 and SPIBRR = 0x7F
//                   The remaining reserved words are read and discarded
//                   and then returns to the main routine.
//
static inline void SPIA_ReservedFn(void)
{
    uint16_t speedData;
    uint16_t i;

    //
    // Update LOSPCP register
    //
    speedData = SPIA_Transmit_Receive((uint16_t)0x0000U);

    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) = speedData;
    EDIS;

    NOP_CYCLES(0x0FU);
    //
    // Update SPIBRR register
    //
    speedData = SPIA_Transmit_Receive((uint16_t)0x0000U);

    HWREGH(SPIA_BASE + SPI_O_BRR) = speedData;

    NOP_CYCLES(0x0FU);

    //
    // Read and discard the next 7 reserved words.
    //
    for(i = 1U; i <= 7U; i++)
    {
       (void)SPIA_GetWordData();
    }

    return;
}

//
// SPIA_GetWordData - This routine fetches two bytes from the SPI-A
//                    port and puts them together to form a single
//                    16-bit value.  It is assumed that the host is
//                    sending the data in the form MSB:LSB.
//
uint16_t SPIA_GetWordData(void)
{
    uint16_t wordData;

    //
    // Fetch the LSB
    //
    wordData = SPIA_Transmit_Receive(0x0000U);

    //
    // Fetch the MSB
    //
    wordData |= (SPIA_Transmit_Receive(0x0000U) << 8U);

    return(wordData);
}

/*
* SPIBOOT_configure_gpio - Configure the GPIOs for the specified SPI boot mode
*
*
* \brief SPI Boot
*
* Design: \ref did_spi_boot_options_algo
* Requirement: REQ_TAG(C2000BROM-207)
*
* Start SPI Boot
*
*/
uint16_t SPIBOOT_configure_gpio(uint32_t bootMode)
{
    uint16_t gpioSIMOA;
    uint16_t gpioSOMIA;
    uint16_t gpioCLKA;
    uint16_t gpioSTEA;
    uint32_t gpioSIMOAPinConfig;
    uint32_t gpioSOMIAPinConfig;
    uint32_t gpioCLKAPinConfig;

    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A,0xFFFFFFFFUL);

    switch (bootMode)
    {
        case SPI_MASTER_BOOT_ALT1:
            //
            // GPIO16 = SPIA_SIMO
            // GPIO1  = SPIA_SOMI
            // GPIO3  = SPIA_CLK
            // GPIO0  = SPIA_STE
            //
            gpioSIMOA = 16U;
            gpioSOMIA = 1U;
            gpioCLKA  = 3U;
            gpioSTEA  = 0U;
            gpioSIMOAPinConfig = GPIO_16_SPIA_SIMO;
            gpioSOMIAPinConfig = GPIO_1_SPIA_SOMI;
            gpioCLKAPinConfig  = GPIO_3_SPIA_CLK;
            break;

        case SPI_MASTER_BOOT_ALT2:
            //
            // GPIO8  = SPIA_SIMO
            // GPIO10 = SPIA_SOMI
            // GPIO9  = SPIA_CLK
            // GPIO11 = SPIA_STE
            //
            gpioSIMOA = 8U;
            gpioSOMIA = 10U;
            gpioCLKA  = 9U;
            gpioSTEA  = 11U;
            gpioSIMOAPinConfig = GPIO_8_SPIA_SIMO;
            gpioSOMIAPinConfig = GPIO_10_SPIA_SOMI;
            gpioCLKAPinConfig = GPIO_9_SPIA_CLK;
            break;

        case SPI_MASTER_BOOT_ALT3:
            //
            // GPIO16 = SPIA_SIMO
            // GPIO13 = SPIA_SOMI
            // GPIO12 = SPIA_CLK
            // GPIO29 = SPIA_STE
            //
            gpioSIMOA = 16U;
            gpioSOMIA = 13U;
            gpioCLKA  = 12U;
            gpioSTEA  = 29U;
            gpioSIMOAPinConfig = GPIO_16_SPIA_SIMO;
            gpioSOMIAPinConfig = GPIO_13_SPIA_SOMI;
            gpioCLKAPinConfig = GPIO_12_SPIA_CLK;

            GPIO_setAnalogMode(12,GPIO_ANALOG_DISABLED);
            GPIO_setAnalogMode(13,GPIO_ANALOG_DISABLED);

            break;

        case SPI_MASTER_BOOT:
        default:
            //
            // GPIO7 = SPIA_SIMO
            // GPIO1 = SPIA_SOMI
            // GPIO3 = SPIA_CLK
            // GPIO5 = SPIA_STE
            //
            gpioSIMOA = 7U;
            gpioSOMIA = 1U;
            gpioCLKA  = 3U;
            gpioSTEA  = 5U;
            gpioSIMOAPinConfig = GPIO_7_SPIA_SIMO;
            gpioSOMIAPinConfig = GPIO_1_SPIA_SOMI;
            gpioCLKAPinConfig  = GPIO_3_SPIA_CLK;
            break;
    }

    //
    // Enable pull up on GPIOs pins
    //
    GPIO_setPadConfig(gpioSIMOA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioSOMIA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioCLKA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioSTEA,GPIO_PIN_TYPE_PULLUP);

    //
    // Set PIN config for SPI
    //
    GPIO_setPinConfig(gpioSIMOAPinConfig);
    GPIO_setPinConfig(gpioSOMIAPinConfig);
    GPIO_setPinConfig(gpioCLKAPinConfig);

    //
    // Configure SPIA_STE as GPIO output
    //
    GPIO_setDirectionMode(gpioSTEA,GPIO_DIR_MODE_OUT);

    //
    // Configure GPIOs pins as async pins
    //
    GPIO_setQualificationMode(gpioSIMOA,GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(gpioSOMIA,GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(gpioCLKA,GPIO_QUAL_ASYNC);

    //
    // Pull SPIA_STE low to enable Chip Select
    //
    GPIO_writePin(gpioSTEA,0);

    return(gpioSTEA);
}

//
// SPIA_Init - SPI-A initialization routine for communication with host
//
void SPIA_Init(void)
{
    //
    // Enable SPI-A clocks
    //
    EALLOW;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4);

    //
    // Enable FIFO reset bit only (FIFO can resume transmit/receive operation)
    //
    HWREGH(SPIA_BASE + SPI_O_FFTX) = SPI_FFTX_SPIRST;

    //
    // Setup SPI configurations
    // Note - The sysclk passed in is divided by 4 due to the API requiring the
    //        low speed clock value (by default /4 is the divider for LSPCLK)
    //
    SPI_disableModule(SPIA_BASE);
    SPI_setConfig(SPIA_BASE,(BOOTROM_SYSCLK/4U),SPI_PROT_POL0PHA1,
                  SPI_MODE_MASTER,SPI_BIT_RATE,SPI_CHARACTER_LENGTH);
    SPI_enableModule(SPIA_BASE);

    EDIS;
}

//
// End of File
//
