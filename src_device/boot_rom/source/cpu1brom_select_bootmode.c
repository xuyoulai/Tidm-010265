//###########################################################################
//
// FILE:    cpu1brom_select_bootmode.c
//
// TITLE:   CPU1 Boot Mode selection routines
//
// This includes functions to select the boot mode (standalone or emulation)
// and decode the boot pins.
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
// Function Prototypes
//
uint32_t CPU1BROM_getStandaloneBootMode(uint32_t z2OTPBootpinCfgKey, uint32_t z1OTPBootpinCfgKey);
uint32_t CPU1BROM_getEmulationBootMode(uint32_t emuBootpinConfigKey);
uint16_t CPU1BROM_decodeBootPins(uint32_t pinConfig);
uint16_t CPU1BROM_isFlashProgrammed(void);
uint32_t cbrom_GPIO_ReadPin(uint32_t pin);

//
// Read the GPyDAT register bit for the specified pin.
//
uint32_t cbrom_GPIO_ReadPin(uint32_t pin)
{
    if(((224UL <= pin) && (pin <= 245UL)) ||
        (12UL == pin) || (13UL == pin) || (20UL == pin) || (21UL == pin) || (28UL == pin))
    {
        GPIO_setAnalogMode(pin,GPIO_ANALOG_DISABLED);
    }

    return GPIO_readPin(pin);
}

/**
* CPU1BROM_decodeBootPins - Decode boot mode table index from boot mode
*                           select pins
*
*
* \brief Decode Pins
*
* Design: \ref did_peripheral_pin_compatible_algo did_custom_boot_mode_config_algo
*              did_custom_boot_pin_config_algo did_custom_boot_modes_algo
*              did_safety_gpio_qualification_algo did_custom_boot_override_algo
*              did_default_boot_modes_algo
* Requirement: REQ_TAG(C2000BROM-184), REQ_TAG(C2000BROM-177), REQ_TAG(C2000BROM-178),
*              REQ_TAG(C2000BROM-175), REQ_TAG(C2000BROM-176), REQ_TAG(C2000BROM-237),
*
*
* Decode Pins
*
*/
uint16_t CPU1BROM_decodeBootPins(uint32_t pinConfig)
{
    uint32_t bootPin0 = (pinConfig & CPU1_PIN_CONFIG_MASK);
    uint32_t bootPin1 = ((pinConfig >> 8U) & CPU1_PIN_CONFIG_MASK);
    uint32_t bootPin2 = ((((uint32_t)pinConfig) >> 16U) & CPU1_PIN_CONFIG_MASK);
    uint16_t bootTableIndex;

    //
    // Check if all boot pins disabled (zero pin config), else decode GPIOs
    //
    if((uint32_t)(pinConfig & CPU1_ALL_BMSP_DISABLED_MASK) == 
       CPU1_ALL_BMSP_DISABLED_MASK)
    {
        //
        // Zero pin config scenario - Table index always zero
        //
        return (0U);
    }

    //
    // Reset to default BMSP if the GPIO selected is not supported in the device
    //
    if((bootPin0 == 14UL) || (bootPin0 == 15UL) || (bootPin0 == 25UL) || (bootPin0 == 26UL) || 
       (bootPin0 == 27UL) || (bootPin0 == 30UL) || (bootPin0 == 31UL) || (bootPin0 == 34UL) ||
       (bootPin0 == 36UL) || (bootPin0 == 38UL) || ((42UL < bootPin0) && (bootPin0 < 224UL)))
    {
        bootPin0 = FACTORY_DEFAULT_BMSP0;
    }

    if((bootPin1 == 14UL) || (bootPin1 == 15UL) || (bootPin1 == 25UL) || (bootPin1 == 26UL) || 
       (bootPin1 == 27UL) || (bootPin1 == 30UL) || (bootPin1 == 31UL) || (bootPin1 == 34UL) ||
       (bootPin1 == 36UL) || (bootPin1 == 38UL) || ((42UL < bootPin1) && (bootPin1 < 224UL)))
    {
        bootPin1 = FACTORY_DEFAULT_BMSP1;
    }

    if((bootPin2 == 14UL) || (bootPin2 == 15UL) || (bootPin2 == 25UL) || (bootPin2 == 26UL) || 
       (bootPin2 == 27UL) || (bootPin2 == 30UL) || (bootPin2 == 31UL) || (bootPin2 == 34UL) ||
       (bootPin2 == 36UL) || (bootPin2 == 38UL) || ((42UL < bootPin2) && (bootPin2 < 224UL)))
    {
        bootPin2 = CPU1_BMSP_DISABLED;
    }

    //
    // Set GPIO qualification of 6 samples for each BMSP
    // and wait 30 clock cycles to get accurate sample window
    //
    if(bootPin0 != CPU1_BMSP_DISABLED)
    {
        GPIO_setQualificationMode(bootPin0, GPIO_QUAL_6SAMPLE);
    }
    if(bootPin1 != CPU1_BMSP_DISABLED)
    {
        GPIO_setQualificationMode(bootPin1, GPIO_QUAL_6SAMPLE);
    }
    if(bootPin2 != CPU1_BMSP_DISABLED)
    {
        GPIO_setQualificationMode(bootPin2, GPIO_QUAL_6SAMPLE);
    }
    asm(" MOV    @T,#30 ");
    asm(" RPT    @T \
            ||NOP ");

    //
    // Check if any of the pins are disabled, else read the GPIO state
    //
    if(bootPin2 == CPU1_BMSP_DISABLED)
    {
        bootTableIndex = 0x0U;
    }
    else
    {
        bootTableIndex = (uint16_t)cbrom_GPIO_ReadPin(bootPin2) << 2U;
    }

    if(bootPin1 == CPU1_BMSP_DISABLED)
    {
        bootTableIndex &= 0xFDU; // zero out bit 1
    }
    else
    {
        bootTableIndex &= 0xFDU;
        bootTableIndex |= (uint16_t)cbrom_GPIO_ReadPin(bootPin1) << 1U;
    }

    if(bootPin0 == CPU1_BMSP_DISABLED)
    {
        bootTableIndex &= 0xFEU; // zero out bit 0
    }
    else
    {
        bootTableIndex &= 0xFEU;
        bootTableIndex |= (uint16_t)cbrom_GPIO_ReadPin(bootPin0);
    }

    //
    // Remove GPIO qualification for each BMSP
    //
    if(bootPin0 != CPU1_BMSP_DISABLED)
    {
        GPIO_setQualificationMode(bootPin0, GPIO_QUAL_SYNC);
    }
    if(bootPin1 != CPU1_BMSP_DISABLED)
    {
        GPIO_setQualificationMode(bootPin1, GPIO_QUAL_SYNC);
    }
    if(bootPin2 != CPU1_BMSP_DISABLED)
    {
        GPIO_setQualificationMode(bootPin2, GPIO_QUAL_SYNC);
    }

    return(bootTableIndex & 0x7U);
}

/**
* Get the Boot Mode Configured for Standalone Boot
*
* Param - None
*
* This function reads the OTP key and bootpin config locations to determine the
* boot mode to return.
*
* Return - Boot Mode Value
*
*
* \brief Standalone boot mode function
*
* Design: \ref did_standalone_in_emulation_boot_algo did_standalone_boot_algo did_custom_boot_pin_emu_invalid_key_algo
*              did_custom_boot_config_z1_z2_algo did_peripheral_pin_compatible_algo did_custom_boot_pin_invalid_index_algo
* Requirement: REQ_TAG(C2000BROM-234), REQ_TAG(C2000BROM-233), REQ_TAG(C2000BROM-179),
*              REQ_TAG(C2000BROM-182), REQ_TAG(C2000BROM-235), REQ_TAG(C2000BROM-180),
*              REQ_TAG(C2000BROM-236)
*
* Standalone bbootmode function
*
*/
uint32_t CPU1BROM_getStandaloneBootMode(uint32_t z2OTPBootpinCfgKey, uint32_t z1OTPBootpinCfgKey)
{
    uint32_t bootMode = 0U;
    uint32_t bootTableIndex;
    uint16_t entryAddress;

    //
    // Check whether to use Z2 OTP for boot configuration settings. 
    // Else use Z1 or un-programmed, default settings
    //
    if(z2OTPBootpinCfgKey != BOOTPIN_CONFIG_KEY)
    {
        //
        // Check if Z1 OTP key is set or not
        //
        if(z1OTPBootpinCfgKey != BOOTPIN_CONFIG_KEY)
        {
            //
            // Bad Z2 and Z1 OTP Key
            //
            
            //
            // Decode boot mode from default boot mode select pins
            //
            bootMode |= cbrom_GPIO_ReadPin(FACTORY_DEFAULT_BMSP1) << 1U;
            bootMode |= cbrom_GPIO_ReadPin(FACTORY_DEFAULT_BMSP0);
            
            bootMode &= 0x3U;

            //
            // CPU1 Patch/Escape Point 4
            //
            if(SW_PATCH_POINT_KEY == swPatchKey)
            {
                entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_4;
                if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
                {
                    //
                    // If OTP is programmed, then call OTP patch function
                    //
                    EXECUTE_ESCAPE_POINT(entryAddress);
                }
            }

            return(bootMode);
        }
        else
        {
            //
            // Use Z1 OTP Boot Configurations
            //
            
            //
            // Decode boot mode from OTP BOOTPIN configuration
            //
            bootTableIndex = (uint32_t)CPU1BROM_decodeBootPins(HWREG(Z1_OTP_BOOTPIN_CONFIG));

            if(bootTableIndex < 4U)
            {
                //
                // Decode boot index (lower BOOTDEF table 0 to 3)
                //
                bootMode = (uint32_t)HWREAD_Z1_OTP_BOOTDEF_L(bootTableIndex);
            }
            else if((3U < bootTableIndex) && (bootTableIndex < 8U))
            {
                //
                // Decode boot index (lower BOOTDEF table 4 to 7)
                //
                bootMode = (uint32_t)HWREAD_Z1_OTP_BOOTDEF_H(bootTableIndex);
            }
            else
            {
                //
                // Incorrect boot table index, pause execution when connected to
                // debugger. Set boot mode to "flash".
                //
#ifndef LDRA_FILEIO
                asm("   ESTOP0");
#endif
                bootMode = FLASH_BOOT;
            }
        }     
    }
    else
    {
        //
        // Use Z2 OTP Boot Configurations
        //
        
        //
        // Decode boot mode from OTP BOOTPIN configuration
        //
        bootTableIndex = (uint32_t)CPU1BROM_decodeBootPins(HWREG(Z2_OTP_BOOTPIN_CONFIG));

        if(bootTableIndex < 4U)
        {
            //
            // Decode boot index (lower BOOTDEF table 0 to 3)
            //
            bootMode = (uint32_t)HWREAD_Z2_OTP_BOOTDEF_L(bootTableIndex);
        }
        else if((3U < bootTableIndex) && (bootTableIndex < 8U))
        {
            //
            // Decode boot index (lower BOOTDEF table 4 to 7)
            //
            bootMode = (uint32_t)HWREAD_Z2_OTP_BOOTDEF_H(bootTableIndex);
        }
        else
        {
            //
            // Incorrect boot table index, pause execution when connected to
            // debugger. Set boot mode to "flash".
            //
#ifndef LDRA_FILEIO
            asm("   ESTOP0");
#endif
            bootMode = FLASH_BOOT;
        }        
    }
    
    return(bootMode);
}

/**
* Get the Boot Mode Configured for Emulation Boot
*
* Param - None
*
* This function reads the emulation key and bootpin config locations to
* determine the boot mode to return.
*
* Return - Boot Mode Value
*
*
* \brief Emulation boot mode function
*
* Design: \ref did_emulation_boot_algo
* Requirement: REQ_TAG(C2000BROM-232)
*
* Standalone bbootmode function
*
*/
uint32_t CPU1BROM_getEmulationBootMode(uint32_t emuBootpinConfigKey)
{
    uint32_t bootMode = 0U;
    uint16_t bootTableIndex;

    if(emuBootpinConfigKey == BOOTPIN_CONFIG_STANDALONE_KEY)
    {
        //
        // Emu BOOTPIN Config Key is 0xA5, emulation standalone/true boot flow
        //
        bootMode = CPU1BROM_getStandaloneBootMode(HWREAD_Z2_OTP_BOOTPIN_CONFIG_KEY, HWREAD_Z1_OTP_BOOTPIN_CONFIG_KEY);
    }
    else if(emuBootpinConfigKey != BOOTPIN_CONFIG_KEY)
    {
        //
        // Invalid Emu BOOTPIN config key (not 0x5A or 0xA5),
        // set boot mode to "wait boot"
        //
        bootMode = WAIT_BOOT;
    }
    else
    {
        //
        // EMU Key is Valid (0x5A)
        // Decode boot mode from EMU BOOTPIN configuration
        //
        bootTableIndex = CPU1BROM_decodeBootPins(HWREG(EMU_BOOTPIN_CONFIG));

        if(bootTableIndex < 4U)
        {
            //
            // Decode boot index (lower BOOTDEF table 0 to 3)
            //
            bootMode = EMU_BOOTDEF_L((uint32_t)bootTableIndex);
        }
        else if((3U < bootTableIndex) && (bootTableIndex < 8U))
        {
            //
            // Decode boot index (lower BOOTDEF table 4 to 7)
            //
            bootMode = EMU_BOOTDEF_H((uint32_t)bootTableIndex);
        }
        else
        {
            //
            // Incorrect boot table index, pause execution when connected to
            // debugger. Set boot mode to "wait boot".
            //
#ifndef LDRA_FILEIO
            asm("   ESTOP0");
#endif
            bootMode = WAIT_BOOT;
        }
    }

    return(bootMode);
}

/**
* Select the Boot Mode based on Emulation or Standalone
*
* Param - None
*
* This function reads the debugger status and determines whether to get boot
* mode through emulation or standalone process
*
* Return - Boot Mode Value
*
*
* \brief Select bootmode function
*
* Design: \ref did_select_bootmode_usecase did_otp_compatibility_algo
* Requirement: REQ_TAG(C2000BROM-175), REQ_TAG(C2000BROM-176), REQ_TAG(C2000BROM-177),
*              REQ_TAG(C2000BROM-180), REQ_TAG(C2000BROM-184), REQ_TAG(C2000BROM-235),
*              REQ_TAG(C2000BROM-236), REQ_TAG(C2000BROM-232), REQ_TAG(C2000BROM-233),
*              REQ_TAG(C2000BROM-234)
*
* Select bootmode function
*
*/
uint32_t CPU1BROM_selectBootMode(void)
{
    uint32_t bootMode;

    EALLOW;

    if(((uint32_t)HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & 
        (uint32_t)SYSCTL_RESC_DCON) == SYSCTL_RESC_DCON)
    {
        //
        // Run emulation boot flow when debugger connected.
        //
        bootMode = CPU1BROM_getEmulationBootMode(EMU_BOOTPIN_CONFIG_KEY);
    }
    else
    {
        //
        // Run standalone/true boot flow when debugger isn't connected.
        //
        bootMode = CPU1BROM_getStandaloneBootMode(HWREAD_Z2_OTP_BOOTPIN_CONFIG_KEY, HWREAD_Z1_OTP_BOOTPIN_CONFIG_KEY);
    }

    EDIS;

    return(bootMode);
}

//
// End of File
//
