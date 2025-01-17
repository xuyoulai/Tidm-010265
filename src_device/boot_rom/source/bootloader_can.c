//###########################################################################
//
// FILE:    bootloader_can.c
//
// TITLE:   CAN Bootloader
//
// Functions involved in running CAN bootloader
//
// ---------------------------------------------------
// |Opt No.|  BOOTDEF      |  CANATXA   |  CANARXA   |
// ---------------------------------------------------
// |  1    |  0x02(default)|  04        |  05        |
// |  2    |  0x22         |  32        |  33        |
// |  3    |  0x42         |  02        |  03        |
// |  4    |  0x62         |  13        |  12        |
// ---------------------------------------------------
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
#include "bootloader_can.h"

//
// Defines
//
#define SELECT_EXTERNAL_OSC    0x1UL
#define CLOCK_DIVIDER_1        0x0U
#define CAN_DISABLE_PARITY     0x5UL
#define CAN_ENABLE_PARITY      0xAUL
#define CAN_11_BIT_ID_S        18U
#define CAN_RX_MSG_ID          0x1UL
#define CAN_TX_MSG_ID          0x2UL
#define CAN_MSG_OBJ_1          0x1U
#define CAN_MSG_OBJ_2          0x2U
#define CAN_DLC_SIZE           0x2U

//
// The key value for RAM initialization
//
#define CAN_RAM_INIT_KEY           (0xAU)

//
// CAN bit timing settings for running CAN at 100kbps with a 20MHz crystal
//
// See CAN timing header for these values

//
// Function Prototypes
//
static void DCAN_Boot_GPIO(uint32_t bootMode);
static void DCAN_Boot_Init(uint32_t btrReg, uint16_t switchToXTAL);
static uint16_t DCAN_GetWordData(void);
static void DCAN_SendWordData(uint16_t data);
static void DCAN_ParseReservedWords(void);

/**
* DCAN_Boot - Run the CAN bootloader setup with the specified GPIOs for the
*             requested CAN boot mode
*
*
* \brief CAN Boot
*
* Design: did_can_boot_algo did_boot_first_instance_algo)
* Requirement: REQ_TAG(C2000BROM-208), REQ_TAG(C2000BROM-210)
*
* Start CAN Boot
*
*/
uint32_t DCAN_Boot(uint32_t bootMode, uint32_t bitTimingRegValue,
                   uint16_t switchToXTAL)
{
    uint32_t appEntryAddress = 0xFFFFFFFFUL;
    uint16_t entryAddress;

    //
    //Temporary variable as MISRA does not let arguments be modified.
    //
    uint32_t timingValue = bitTimingRegValue;

    //
    // CPU1 Patch/Escape Point 10
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_10;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // If DCAN-A is not enabled, bypass the boot loader
    //
    if(SysCtl_isPeripheralPresent(SYSCTL_PERIPH_PRESENT_CANA) == false)
    {
       return(FLASH_ENTRY_POINT);
    }

    //
    // Assign the CAN data reader function to the global
    // function pointer for loading data.
    //
    GetWordData = &DCAN_GetWordData;

    //
    // Set up the GPIO mux for the chosen pinout
    //
    DCAN_Boot_GPIO(bootMode);

    //
    // Set up the CAN to receive data. Pass the user-provided bit timing
    // register value if one was provided, otherwise pass the default for
    // 100 kbps with a 20 MHz crystal.
    //
    if(bitTimingRegValue == 0U)
    {
        timingValue = CAN_CALC_BTRREG;
    }

    DCAN_Boot_Init(timingValue, switchToXTAL);

    //
    // CPU1 Patch/Escape Point 10
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_10;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Testing Only: Send two tests frames if the boot mode says so
    //
    if(bootMode >= CAN_BOOT_SENDTEST)
    {
        DCAN_SendWordData(0x0320U);
        DCAN_SendWordData(0xf280U);
    }

    //
    // The first data word should be a valid key. If it's not,
    // bypass the bootloader.
    //
    if(DCAN_GetWordData() != BROM_EIGHT_BIT_HEADER)
    {
        return FLASH_ENTRY_POINT;
    }

    //
    // Use the shared utility functions to load the data.
    //
    DCAN_ParseReservedWords();
    appEntryAddress = GetLongData();
    CopyData();

    return appEntryAddress;
}

/**
* DCAN_Boot_GPIO - Configure the peripheral mux to connect CAN-A to the
*                  chosen GPIOs
*
*
* \brief CAN Boot GPIO select
*
* Design: did_can_boot_algo
* Requirement: REQ_TAG(C2000BROM-206)
*
* Start I2C Boot
*
*/
static void DCAN_Boot_GPIO(uint32_t bootMode)
{
    uint16_t gpioTx;
    uint16_t gpioRx;
    uint32_t gpioTxPinConfig;
    uint32_t gpioRxPinConfig;

    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A,0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_B,0xFFFFFFFFUL);

    //
    // Decode the GPIO selection, then set up the mux and configure the inputs
    // for asynchronous qualification.
    //
    switch (bootMode)
    {

        case CAN_BOOT_ALT1:
        case CAN_BOOT_ALT1_SENDTEST:
            //
            // GPIO32 = CANATX
            // GPIO33 = CANARX
            //
            gpioTx = 32U;
            gpioRx = 33U;
            gpioTxPinConfig = GPIO_32_CANA_TX;
            gpioRxPinConfig = GPIO_33_CANA_RX;

            break;

        case CAN_BOOT_ALT2:
        case CAN_BOOT_ALT2_SENDTEST:
            //
            // GPIO2 = CANATX
            // GPIO3 = CANARX
            //
            gpioTx = 2U;
            gpioRx = 3U;
            gpioTxPinConfig = GPIO_2_CANA_TX;
            gpioRxPinConfig = GPIO_3_CANA_RX;

            break;

        case CAN_BOOT_ALT3:
        case CAN_BOOT_ALT3_SENDTEST:
            //
            // GPIO12 = CANARX
            // GPIO13 = CANATX
            //
            gpioTx = 13U;
            gpioRx = 12U;
            gpioTxPinConfig = GPIO_13_CANA_TX;
            gpioRxPinConfig = GPIO_12_CANA_RX;

            GPIO_setAnalogMode(12,GPIO_ANALOG_DISABLED);
            GPIO_setAnalogMode(13,GPIO_ANALOG_DISABLED);

            break;

        case CAN_BOOT:
        case CAN_BOOT_SENDTEST:
        default:
            //
            // GPIO4 = CANATX
            // GPIO5 = CANARX
            //
            gpioTx = 4U;
            gpioRx = 5U;
            gpioTxPinConfig = GPIO_4_CANA_TX;
            gpioRxPinConfig = GPIO_5_CANA_RX;

            break;

    }

    //
    // Enable pull up on GPIOs pins
    //
    GPIO_setPadConfig(gpioTx,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioRx,GPIO_PIN_TYPE_PULLUP);

    //
    // Set GPIO configuration for CAN
    //
    GPIO_setPinConfig(gpioTxPinConfig);
    GPIO_setPinConfig(gpioRxPinConfig);

    //
    // Configure GPIOs as async pins
    //
    GPIO_setQualificationMode(gpioTx,GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(gpioRx,GPIO_QUAL_ASYNC);
}

//
// DCAN_Boot_Init - Initialize the CAN-A module and configure its bit clock
//                  and message objects
//
static void DCAN_Boot_Init(uint32_t btrReg, uint16_t switchToXTAL)
{
    //
    // Select XTAL for CAN clock
    //
    EALLOW;
    if(switchToXTAL == CAN_BOOT_USE_XTAL)
    {
	    //Turn on XTAL and select crystal mode
	    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= (uint16_t)~SYSCTL_XTALCR_OSCOFF;
	    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= (uint16_t)~SYSCTL_XTALCR_SE;

	    //Wait for the X1 clock to saturate
	    HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) |= SYSCTL_X1CNT_CLR;
	    while(HWREGH(CLKCFG_BASE + SYSCTL_O_X1CNT) != 0x7FFU) {;}

		HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &= (uint32_t)~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;
		HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) |= 0x1UL;
		NOP_CYCLES(16);
		HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &= (uint32_t)~SYSCTL_CLKSRCCTL2_CANABCLKSEL_M;
        HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) |= (uint32_t)(1U << SYSCTL_CLKSRCCTL2_CANABCLKSEL_S);
		HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 0x0;
    }

    //
    // Turn on the clock to the DCAN-A module
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR10) |= SYSCTL_PCLKCR10_CAN_A;
    EDIS;

    //
    // Put the CAN module into initialization mode, then issue a software reset
    // via the self-clearing SWR bit.
    //
    HWREG_BP(CANA_BASE + CAN_O_CTL) = (uint32_t)((CAN_DISABLE_PARITY << CAN_CTL_PMD_S) |
                                      CAN_CTL_INIT);
    NOP_CYCLES(16);
    EALLOW;
    HWREG_BP(CANA_BASE + CAN_O_CTL) |= CAN_CTL_SWR;
    EDIS;
    NOP_CYCLES(16);

    //
    // Initialize the CAN message RAM
    //
    HWREG_BP(CANA_BASE + CAN_O_RAM_INIT) = CAN_RAM_INIT_CAN_RAM_INIT |
                                           CAN_RAM_INIT_KEY;
    while((HWREG_BP(CANA_BASE + CAN_O_RAM_INIT) & CAN_RAM_INIT_RAM_INIT_DONE) !=
          CAN_RAM_INIT_RAM_INIT_DONE)
    {
    }

    //
    // Enable config register access, set up the bit timing, and make sure
    // parity stays enabled.
    //
    HWREG_BP(CANA_BASE + CAN_O_CTL) = (uint32_t)((CAN_ENABLE_PARITY << CAN_CTL_PMD_S) |
                                      CAN_CTL_CCE | CAN_CTL_INIT);
    HWREG_BP(CANA_BASE + CAN_O_BTR) = btrReg;

    //
    // Set up a receive message object via interface 1, then transfer it to the
    // message RAM.
    //
    HWREG_BP(CANA_BASE + CAN_O_IF1ARB) = (uint32_t)(CAN_IF1ARB_MSGVAL | 
                               ((uint32_t)(CAN_RX_MSG_ID << CAN_11_BIT_ID_S)));
    HWREG_BP(CANA_BASE + CAN_O_IF1MCTL) = CAN_IF1MCTL_EOB | CAN_DLC_SIZE;
    HWREG_BP(CANA_BASE + CAN_O_IF1MSK) = (uint32_t)(CAN_IF1MSK_MSK_M |
                                                    CAN_IF1MSK_MDIR |
                                                    CAN_IF1MSK_MXTD);
    HWREG_BP(CANA_BASE + CAN_O_IF1CMD) = (uint32_t)(CAN_IF1CMD_DIR |
                                                 CAN_IF1CMD_MASK |
                                                 CAN_IF1CMD_ARB |
                                                 CAN_IF1CMD_CONTROL |
                                                 CAN_IF1CMD_CLRINTPND |
                                                 CAN_MSG_OBJ_1);

    while((HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    //
    // Set up a transmit object via interface 2 for debug, then transfer it to
    // the message RAM.
    //
    HWREG_BP(CANA_BASE + CAN_O_IF2ARB) = (uint32_t)(CAN_IF2ARB_MSGVAL | CAN_IF2ARB_DIR |
                                  ((uint32_t)(CAN_TX_MSG_ID << CAN_11_BIT_ID_S)));
    HWREG_BP(CANA_BASE + CAN_O_IF2MCTL) = CAN_IF2MCTL_EOB | CAN_DLC_SIZE;
    HWREG_BP(CANA_BASE + CAN_O_IF2MSK) = (uint32_t)(CAN_IF2MSK_MSK_M |
                                                    CAN_IF2MSK_MDIR |
                                                    CAN_IF2MSK_MXTD);
    HWREG_BP(CANA_BASE + CAN_O_IF2CMD) = (uint32_t)(CAN_IF2CMD_DIR |
                                                    CAN_IF2CMD_MASK |
                                                    CAN_IF2CMD_ARB |
                                                    CAN_IF2CMD_CONTROL |
                                                    CAN_IF2CMD_CLRINTPND |
                                                    CAN_MSG_OBJ_2);

    while((HWREGH(CANA_BASE + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) == CAN_IF2CMD_BUSY)
    {
    }

    //
    // Leave initialization mode and disable timing register access and
    // automatic retransmission.
    //
    HWREGH(CANA_BASE + CAN_O_CTL) &= (uint16_t)(~(CAN_CTL_CCE | CAN_CTL_INIT));
}

//
// DCAN_GetWordData - Read 16 bits from an incoming DCAN message sent to ID #1.
//                    If no message has been received, wait for one to arrive.
//
static uint16_t DCAN_GetWordData(void)
{
    //
    // Wait for a new CAN message to be received in mailbox 1
    //
    while((HWREG_BP(CANA_BASE + CAN_O_NDAT_21) & CAN_MSG_OBJ_1) != CAN_MSG_OBJ_1)
    {
    }

    //
    // Read the message object via IF1 and return the data
    //
    HWREG_BP(CANA_BASE + CAN_O_IF1CMD) = (uint32_t)(CAN_IF1CMD_TXRQST |
                                                    CAN_IF1CMD_DATA_A |
                                                    CAN_IF1CMD_DATA_B |
                                                    CAN_MSG_OBJ_1);

    NOP_CYCLES(2);

    while((HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    return(HWREGH(CANA_BASE + CAN_O_IF1DATA));
}

//
// DCAN_SendWordData - Send a CAN message to ID #2 for external testing and
//                     data rate measurement. Wait for the transmission to
//                     complete.
//
static void DCAN_SendWordData(uint16_t data)
{
    HWREG_BP(CANA_BASE + CAN_O_IF2DATA) = data;
    HWREG_BP(CANA_BASE + CAN_O_IF2CMD) = (uint32_t)(CAN_IF2CMD_DIR |
                                                    CAN_IF2CMD_TXRQST |
                                                    CAN_IF2CMD_DATA_A |
                                                    CAN_IF2CMD_DATA_B |
                                                    CAN_MSG_OBJ_2);

    NOP_CYCLES(255);

    while((HWREGH(CANA_BASE + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) == CAN_IF2CMD_BUSY)
    {
    }

    while((HWREGH(CANA_BASE + CAN_O_TXRQ_21) & CAN_MSG_OBJ_2) == CAN_MSG_OBJ_2)
    {
    }
}

//
// DCAN_ParseReservedWords - Parse the eight reserved words and check whether
//                           there's a new bit timing register value in the
//                           first pair.
//
static void DCAN_ParseReservedWords(void)
{
    uint32_t newBtrReg;
    uint16_t word;

    //
    // Read the new bit timing value, LSW first
    //
    newBtrReg = (uint32_t)DCAN_GetWordData();
    newBtrReg |= (uint32_t)DCAN_GetWordData() << 16UL;

    //
    // Skip the rest of the reserved words
    //
    for(word = 3U; word <= 8U; word++)
    {
        (void)DCAN_GetWordData();
    }

    //
    // If a new bit timing value was received, switch to the new settings
    //
    if(newBtrReg != 0x00000000UL)
    {
        DCAN_Boot_Init(newBtrReg, 0U);
    }
}

//
// End of File
//
