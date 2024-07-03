//###########################################################################
//
// FILE:    bootloader_i2c.c
//
// TITLE:   I2C Bootloader routines
//
// Functions involved in running I2C bootloader
//
// ---------------------------------------------------
// |Opt No.|  BOOTDEF      |  I2CA_SDA  |  I2CA_SCL  |
// ---------------------------------------------------
// |  1    |  0x07(default)|  00        |  01        |
// |  2    |  0x27         |  32        |  33        |
// |  3    |  0x47         |  05        |  04        |
// ---------------------------------------------------
//
// Notes:
//     The I2C code contained here is specifically streamlined for the
//     bootloader. It can be used to load code via the I2C port into the
//     RAM and jump to an entry point within that code.
//
//     Features/Limitations:
//     - The I2C boot loader code is written to communicate with an EEPROM
//       device at address 0x50. The EEPROM must adhere to conventional I2C
//       EEPROM protocol (see the boot rom documentation) with a 16-bit
//       base address architecture (as opposed to 8-bits). The base address
//       of the code should be contained at address 0x0000 in the EEPROM.
//     - At boot, the internal oscillator will run at 10Mhz.  The boot ROM
//       will configure the DIVSEL to be /1 for a 10Mhz system clock.
//       This is due to a requirement that the I2C clock be between 7Mhz
//       and 12Mhz to meet all of the I2C specification timing requirements.
//       The I2CPSC default value is hardcoded to 0 so that the I2C
//       clock will not be divided down from the system clock.
//       The I2CPSC value can be modified after receiving
//       the first few bytes from the EEPROM (see the boot rom documentation),
//       but it is advisable not to, as this can cause the I2C to operate out
//       of specification with a system clock between 7Mhz and 12Mhz.
//     - The bit period prescalers (I2CCLKH and I2CCLKL) are configured to
//       run the I2C at 50% duty cycle at 100kHz bit rate (standard I2C mode)
//       when the system clock is 12Mhz. These registers can be modified after
//       receiving the first few bytes from the EEPROM (see the boot rom
//       documentation). This allows the communication to be increased up to
//       a 400kHz bit rate (fast I2C mode) during the remaining data reads.
//     - Arbitration, bus busy, and slave signals are not checked. Therefore,
//       no other master is allowed to control the bus during this
//       initialization phase. If the application requires another master
//       during I2C boot mode, that master must be configured to hold off
//       sending any I2C messages until the F280x application software
//       signals that it is past the bootloader portion of initialization.
//     - The non-acknowledgment bit is only checked during the first message
//       sent to initialize the EEPROM base address. This ensures that an
//       EEPROM is present at address 0x50 before continuing on. If an EEPROM
//       is not present, code will jump to the Flash entry point. The
//       non-acknowledgment bit is not checked during the address phase of
//       the data read messages (I2C_GetWord). If a non-acknowledge is
//       received during the data read messages, the I2C bus will hang.
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
#include "bootloader_i2c.h"

//
// Function Prototypes
//
static inline uint16_t I2C_CheckKeyVal(void);
static inline void I2C_ReservedFn(void);
uint16_t I2C_GetWord(void);

void I2CBOOT_configure_gpio(uint32_t bootMode);

/**
* I2C_Boot - This module is the main I2C boot routine.
*           It will load code via the I2C-A port.
*
*            It will return an entry point address back
*            to the system initialization routine which will call ExitBoot.
*
*
* \brief I2C Boot
*
* Design: did_i2c_boot_algo
* Requirement: REQ_TAG(C2000BROM-208)
*
* Start I2C Boot
*
*/
uint32_t I2C_Boot(uint32_t  bootMode)
{
    uint32_t appEntryAddress = 0xFFFFFFFFUL;
    uint16_t entryAddress;

    //
    // CPU1 Patch/Escape Point 11
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_11;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Assign GetWordData to the I2C-A version of the
    // function.  GetWordData is a pointer to a function.
    //
    GetWordData = I2C_GetWord;

    //
    // Turn on I2C module clock
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);

    //
    // Set Slave address - EEPROM control code
    //
    I2C_setSlaveAddress(I2CA_BASE, SLAVE_ADDRESS);

    //
    // Initialize I2C in master transmitter mode
    //
    // I2C input frequency (SYSCLKOUT)       = 10 MHz
    // I2C module frequency(Module clock)    = 10 MHz
    // I2C bit rate (in freq) (Output clock) = 100 KHz
    //
    I2C_initMaster(I2CA_BASE,BOOTROM_SYSCLK,I2CA_SCL_CLOCK,I2C_DUTYCYCLE_50);
    I2C_setConfig(I2CA_BASE,I2C_MASTER_SEND_MODE);
    I2C_setBitCount(I2CA_BASE,I2C_BITCOUNT_8);
    I2C_enableModule(I2CA_BASE);

    //
    // Enable FIFO
    //
    I2C_enableFIFO(I2CA_BASE);

    //
    // GPIO INIT
    //
    I2CBOOT_configure_gpio(bootMode);

    //
    // CPU1 Patch/Escape Point 11
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_11;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Check for 0x08AA data header, else go to flash
    //
    if(I2C_CheckKeyVal() == I2C_ERROR)
    {
        return FLASH_ENTRY_POINT;
    }

    //
    // Check for clock and prescaler speed changes and reserved words
    //
    I2C_ReservedFn();

    //
    // Get point of entry address after load
    //
    appEntryAddress = GetLongData();

    //
    // Receive and copy one or more code sections to  destination addresses
    //
    CopyData();

    return appEntryAddress;
}

//
// I2C_CheckKeyVal - This routine sets up the starting address in the
//                   EEPROM by writing two bytes (0x0000) via the
//                   I2C-A port to slave address 0x50. Without
//                   sending a stop bit, the communication is then
//                   restarted and two bytes are read from the EEPROM.
//                   If these two bytes read do not equal 0x08AA
//                   (little endian), an error is returned.
//
static inline uint16_t I2C_CheckKeyVal(void)
{
    //
    // To read a word from the EEPROM, an address must be given first in
    // master transmitter mode. Then a restart is performed and data can
    // be read back in master receiver mode.
    //
    // Setup how many bytes to send
    //
    I2C_setDataCount(I2CA_BASE, NUM_DATA_BYTES);

    //
    // Configure fifo data for byte address of 0x0000
    //
    I2C_putData(I2CA_BASE, 0x0U);
    I2C_putData(I2CA_BASE, 0x0U);

    //
    // Send data to setup EEPROM address
    //
    I2C_sendStartCondition(I2CA_BASE);

    //
    // Wait until communication complete and registers ready
    //
    while((I2C_getStatus(I2CA_BASE) & I2C_STR_ARDY) == 0U)
    {
    }

    //
    // Set stop bit & return error if NACK received
    //
    if((I2C_getStatus(I2CA_BASE) & I2C_STR_NACK) == I2C_STR_NACK)
    {
       I2C_sendStopCondition(I2CA_BASE);
       return I2C_ERROR;
    }

    //
    // Check to make sure key value received is correct
    //
    if(I2C_GetWord() != BROM_EIGHT_BIT_HEADER)
    {
        return I2C_ERROR;
    }

    return I2C_NO_ERROR;
}


//
// I2C_ReservedFn - This function reads 8 reserved words in the header.
//                    1st word - parameters for I2CPSC register
//                    2nd word - parameters for I2CCLKH register
//                    3rd word - parameters for I2CCLKL register
//
//                  The remaining reserved words are read and discarded
//                  and then program execution returns to the main routine.
//
static inline void I2C_ReservedFn(void)
{
    uint16_t I2CPrescaler;
    uint16_t I2cClkHData;
    uint16_t I2cClkLData;
    uint16_t i;

    //
    // Get I2CPSC, I2CCLKH, and I2CCLKL values
    //
    I2CPrescaler = I2C_GetWord();
    I2cClkHData  = I2C_GetWord();
    I2cClkLData  = I2C_GetWord();

    //
    // Store I2C clock prescalers
    //
    I2C_disableModule(I2CA_BASE);

    HWREGH(I2CA_BASE + I2C_O_CLKL) = I2cClkLData;
    HWREGH(I2CA_BASE + I2C_O_CLKH) = I2cClkHData;
    HWREGH(I2CA_BASE + I2C_O_PSC)  = I2CPrescaler;

    I2C_enableModule(I2CA_BASE);

    //
    // Read and discard the next 5 reserved words
    //
    for (i=1U; i<=5U; i++)
    {
        (void)I2C_GetWord();
    }

    return;
}


//
// I2C_GetWord - This routine fetches two bytes from the I2C-A
//               port and puts them together little endian style
//               to form a single 16-bit value.
//
uint16_t I2C_GetWord(void)
{
    uint16_t lowByte;

    //
    // Setup how many bytes to expect
    //
    I2C_setDataCount(I2CA_BASE, NUM_DATA_BYTES);

    //
    // Send start as master receiver
    //
    I2C_setConfig(I2CA_BASE,I2C_MASTER_RECEIVE_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    //
    // Wait until communication done
    //
    while(I2C_getStopConditionStatus(I2CA_BASE)){}

    //
    // Combine two bytes to one word & return
    //
    lowByte = I2C_getData(I2CA_BASE);

    return(lowByte | (I2C_getData(I2CA_BASE) << 8U));
}


//
// I2CBOOT_configure_gpio - Configure GPIOs for specified I2C boot mode
//
void I2CBOOT_configure_gpio(uint32_t bootMode)
{
    uint16_t gpioSDAA;
    uint16_t gpioSCLA;
    uint32_t gpioSDAAPinConfig;
    uint32_t gpioSCLAPinConfig;    
    
    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A,0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_B,0xFFFFFFFFUL);

    switch (bootMode)
    {
        case I2C_MASTER_BOOT_ALT1:
            //
            // GPIO0 = I2CA_SDA
            // GPIO1 = I2CA_SCL
            //
            gpioSDAA = 32U;
            gpioSCLA = 33U;
            gpioSDAAPinConfig = GPIO_32_I2CA_SDA;
            gpioSCLAPinConfig = GPIO_33_I2CA_SCL;
            break;

        case I2C_MASTER_BOOT_ALT2:
            //
            // GPIO10 = I2CA_SDA
            // GPIO8 = I2CA_SCL
            //
            gpioSDAA = 5U;
            gpioSCLA = 4U;
            gpioSDAAPinConfig = GPIO_5_I2CA_SDA;
            gpioSCLAPinConfig = GPIO_4_I2CA_SCL;
            break;

        case I2C_MASTER_BOOT:
        default:
            //
            // GPIO32 = I2CA_SDA
            // GPIO33 = I2CA_SCL
            //
            gpioSDAA = 0U;
            gpioSCLA = 1U;
            gpioSDAAPinConfig = GPIO_0_I2CA_SDA;
            gpioSCLAPinConfig = GPIO_1_I2CA_SCL;
            break;

    }
    
    //
    // Enable pull up on GPIOs pins
    //
    GPIO_setPadConfig(gpioSDAA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioSCLA,GPIO_PIN_TYPE_PULLUP);

    //
    // Set GPIO configuration for I2C
    //
    GPIO_setPinConfig(gpioSDAAPinConfig);
    GPIO_setPinConfig(gpioSCLAPinConfig);

    //
    // Configure GPIOs as async pins
    //
    GPIO_setQualificationMode(gpioSDAA,GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(gpioSCLA,GPIO_QUAL_ASYNC);    
}

//
// End of File
//
