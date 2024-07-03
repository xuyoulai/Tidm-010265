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
#include "hw_asysctl.h"
#include "hw_adc.h"
#include "hw_flash.h"
#include "cpu1brom_ubgpio.h"
#include "cpu1brom_utils.h"
#include "cpubrom_dcsm.h"
#include "cpu1brom_cmac.h"
#include "cpu1brom_crc.h"

//
// Globals
//
#pragma DATA_SECTION(CPU1BROM_bootMode, "UserBootModeVariable");
uint32_t CPU1BROM_bootMode;

#pragma DATA_SECTION(CPU1BROM_pbistStatus, "PBISTStatusVariable");
uint32_t CPU1BROM_pbistStatus;
uint32_t CPU1BROM_corrErrorAddr;
uint32_t CPU1BROM_bootConfigureWord;
uint32_t swPatchKey;

#pragma DATA_SECTION(CPU1BROM_bootStatus, "BootStatusVariable");
uint32_t CPU1BROM_bootStatus;

extern uint32_t CPU1BROM_itrapAddress;

volatile uint16_t CPU1BROM_nmiStatus;

uint16_t CPU1BROM_trimKeySEC;

bool readNoError;

void WaitBoot(void);
void CPU1BROM_setupDeviceSystems(void);
void CPU1BROM_performDeviceConfiguration(void);
uint32_t CPU1BROM_startSystemBoot(void);
void CPU1BROM_runDFTBoot(uint32_t jtagMMRValue);
static uint16_t CPU1BROM_WDGSelfTestAndEnable(uint32_t CPU1BROM_bootConfig);

bool CPU1BROM_setPMMTrims(void);
bool CPU1BROM_setAPLLTrims(void);
bool CPU1BROM_setOscTrims(void);
bool CPU1BROM_setFlashTrims(void);

void CPU1BROM_verifySecureFlash(uint32_t entryAddress);
bool CPU1BROM_getValidTestInsertion(const uint32_t * const testKeyAddresses,
                                    uint32_t *validTestKeyAddress);
static inline bool CPU1BROM_checkECCErrors(void);

#ifdef LDRA_FILEIO
extern int main(void);
#endif// LDRA_FILEIO

/**
* CPU1BROM_runDFTBoot - Based on the value of JTAG_MMR, take either the
*                       functional path, the DMLED test path, or the Boot ROM
*                       test path.
*
* \brief Run DFT Boot Flow
*
* Design: \ref did_dft_boot_algo
*
* Requirement: REQ_TAG(C2000BROM-337), REQ_TAG(C2000BROM-338)
*
* Run DFT Boot Flow
*
*/
void CPU1BROM_runDFTBoot(uint32_t jtagMMRValue)
{
    //
    // If DMLED test path requested, execute this function and wait in loop.
    // Otherwise, for a regular boot or a Boot ROM test, return immediately.
    //
    if(DFT_BOOT_DMLED_KEY == jtagMMRValue)
    {
        //
        // Disable missing clock detection logic and watchdog
        //
        SysCtl_disableMCD();
        SysCtl_disableWatchdog();

        //
        // Ensure NMI flags are cleared
        //
        EALLOW;
        HWREGH(NMI_BASE + NMI_O_FLGCLR) = 0xFFFFU;

        //
        // Disable Flash ECC checking
        //
        HWREGH(FLASH0ECC_BASE + FLASH_O_ECC_ENABLE) = 0U;
        HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
        HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
        EDIS;

        //
        // Write device cal trims and copy DIEID. Skip if wait for 2T ready timed out.
        //
        if(CPU1_BOOTROM_FLASH_2T_NOT_READY != (CPU1BROM_bootStatus & CPU1_BOOTROM_FLASH_2T_NOT_READY))
        {
            CPU1BROM_devcalInit();
            copyDIEID();
        }

        //
        // Wait indefinitely
        //
#ifndef LDRA_FILEIO
        while(true)
        {
            ;
        }
#endif // LDRA_FILEIO
    }
}

/**
* CPU1BROM_setPMMTrims - Write PMM trims from OTP to analog registers
*
* \brief PMM Trim function
*
* Design: \ref did_pmm_trims_usecase did_pmm_trims_blanking_window_algo did_safety_critical_trims_pmm_algo
* Requirement: REQ_TAG(C2000BROM-146)
*
* PMM Trim function
*
*/
bool CPU1BROM_setPMMTrims(void)
{
    uint32_t trimKeyAddresses[NUM_OF_TEST_INSERTIONS];
    uint32_t validTrimKeyAddress = 0UL;
    uint32_t pmmTrimData;
    uint32_t pmmTrimStatus = 0UL;

    //
    // Read enMASK from TI OTP and Configure. Blanking window has to be enabled
    // before configuring the PMM Trim.
    //
    EALLOW;
    if((TI_OTP_REG_VREGCTL_ENMASK_KEY) == VREGCTL_ENMASK_KEY)
    {
        // OTP bits programmed zero means POR/BOR generation is masked in HW
        // this means no need to implement Blanking period in SW (but has to be enabled)
        if(TI_OTP_REG_VREGCTL_ENMASK_VAL == 0x0U)
        {
            // Blanking window is implemented in HW if the ENMASK bit (bit 15 of VREGCTL) is set to '1'
            // the default of this bit is '0'.
            HWREGH(ANALOGSUBSYS_BASE + BROM_ANALOG_SYSCTL_O_VREGCTL) =
                    ((HWREGH(ANALOGSUBSYS_BASE + BROM_ANALOG_SYSCTL_O_VREGCTL) & 0x7FFFU)| 0x8000U);

            //make sure there is a delay of at least 100ns after this
            // device is running at 10MHz +/- efuse trimmed (or not) so a count of so 1 cycle is about
            // 1/10MHz = .1uS = 100ns; so three NOPS should be good enough margin here
            asm(" NOP ");
            asm(" NOP ");
            asm(" NOP ");
        }
    }
    EDIS;

    //
    // PMM REF Trim
    //
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_PMM_REF_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_PMM_REF_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_PMM_REF_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            pmmTrimData = regDoubleRead_32(validTrimKeyAddress + TI_OTP_PMM_TRIM_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into analog reg
                //
                EALLOW;
                regReadAfterWrite_32(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMREFTRIM,
                                     PMM_REF_MASK,
                                     pmmTrimData, false);
                EDIS;

                pmmTrimStatus |= PMM_REF_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            pmmTrimStatus |= PMM_REF_TRIM_LOADED;
        }
    }

    //
    // PMM VMON Trim
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_PMM_VMON_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_PMM_VMON_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_PMM_VMON_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            pmmTrimData = regDoubleRead_16(validTrimKeyAddress + TI_OTP_PMM_TRIM_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into analog reg
                //
                EALLOW;
                regReadAfterWrite_16(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMVMONTRIM,
                                     PMM_VMON_MASK,
                                     (uint16_t)pmmTrimData, false);
                EDIS;

                pmmTrimStatus |= PMM_VMON_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            pmmTrimStatus |= PMM_VMON_TRIM_LOADED;
        }
    }

    //
    // PMM VREG Trim
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_PMM_VREG_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_PMM_VREG_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_PMM_VREG_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            pmmTrimData = regDoubleRead_16(validTrimKeyAddress + TI_OTP_PMM_TRIM_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into analog reg
                //
                EALLOW;
                regReadAfterWrite_16(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMVREGTRIM,
                                     PMM_VREG_MASK,
                                     (uint16_t)pmmTrimData, false);
                EDIS;

                pmmTrimStatus |= PMM_VREG_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            pmmTrimStatus |= PMM_VREG_TRIM_LOADED;
        }
    }

    return (pmmTrimStatus == (PMM_REF_TRIM_LOADED | PMM_VMON_TRIM_LOADED | PMM_VREG_TRIM_LOADED));
}

/**
* CPU1BROM_setOscTrims - Write internal oscillator 1/2 trims and Ext-R based oscillator trims
*                        from OTP to analog registers
*
* \brief Oscillator Trim function
*
* Design: \ref did_intosc_trims_usecase did_intosc_trims_blanking_window_algo did_flash_2t_ready_check_algo
*              did_flash_1t_ready_check_algo did_safety_critical_trims_intoscsr_algo did_safety_critical_trims_intosc_algo
*
* Requirement: REQ_TAG(C2000BROM-147), REQ_TAG(C2000BROM-421), REQ_TAG(C2000BROM-460)
*
* Oscillator Trim function
*
*/
bool CPU1BROM_setOscTrims(void)
{
    uint32_t trimKeyAddresses[NUM_OF_TEST_INSERTIONS];
    uint32_t validTrimKeyAddress = 0UL;
    uint32_t oscTrimStatus = 0UL;

    //
    // INTOSC1/2 Trim
    //
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_INTOSC_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_INTOSC_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_INTOSC_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t intosc1TrimData = regDoubleRead_32(validTrimKeyAddress + TI_OTP_INTOSC_TRIM1_OFFSET, false);
            uint32_t intosc2TrimData = regDoubleRead_32(validTrimKeyAddress + TI_OTP_INTOSC_TRIM2_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into analog reg
                //
                EALLOW;
                regReadAfterWrite_32(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC1TRIM,
                                     OSC_TRIM_MASK,
                                     intosc1TrimData, false);

                regReadAfterWrite_32(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC2TRIM,
                                     OSC_TRIM_MASK,
                                     intosc2TrimData, false);
                EDIS;

                oscTrimStatus |= INT_OSC_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            oscTrimStatus |= INT_OSC_TRIM_LOADED;
        }
    }

    //
    // INTOSC SR
    //
    if(TRIM_32BIT_KEY == HWREG(TI_OTP_INTOSC_SR_KEY_ADDRESS))
    {
        uint16_t sr2Config;
        uint16_t sr3Config;

        //
        // Clear SEC/DED ECC error status
        //
        EALLOW;
        HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
        HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
        EDIS;

        sr2Config = HWREGH(TI_OTP_INTOSC_SR2_CONFIG_ADDRESS);
        sr3Config = HWREGH(TI_OTP_INTOSC_SR3_CONFIG_ADDRESS);

        if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
        {
            EALLOW;
            regReadAfterWrite_16(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSCCSR2, 
                                 INTOSC_SR_MASK, sr2Config, false);
            regReadAfterWrite_16(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSCCSR3, 
                                 INTOSC_SR_MASK, sr3Config, false);
            EDIS;
        }
    }

    //
    // EXTROSC Trim
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_EXTROSC_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_EXTROSC_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_EXTROSC_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t extroscTrimData = regDoubleRead_32(validTrimKeyAddress + TI_OTP_EXTROSC_TRIM_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into analog reg
                //
                EALLOW;
                regReadAfterWrite_32(ANALOGSUBSYS_BASE + ASYSCTL_O_EXTROSC2TRIM,
                                     OSC_TRIM_MASK,
                                     extroscTrimData, false);
                EDIS;

                oscTrimStatus |= EXT_R_OSC_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            oscTrimStatus |= EXT_R_OSC_TRIM_LOADED;
        }
    }

    return ((oscTrimStatus == (INT_OSC_TRIM_LOADED | EXT_R_OSC_TRIM_LOADED)) ? true : false);
}

/**
* CPU1BROM_setAPLLTrims() - Write A-PLL trims from OTP to APLL registers
*
*
* \brief Set APLL trims function
*
* Design: \ref did_safety_config_apll_usecase did_apll_trims_config_algo did_apll_trims_config_lock_algo
*              did_config_apll_usecase
* Requirement: REQ_TAG(C2000BROM-212), REQ_TAG(C2000BROM-213)
*
* Set APLL trims function
*
*/
bool CPU1BROM_setAPLLTrims(void)
{
    uint32_t trimKeyAddresses[NUM_OF_TEST_INSERTIONS];
    uint32_t validTrimKeyAddress = 0UL;
    uint32_t apllTrimStatus = 0UL;

    //
    // APLL LDO Trim
    // (FT is skipped)
    //
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_APLLLDO_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_APLLLDO_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_APLLLDO_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint16_t apllldoTrimData = regDoubleRead_16(validTrimKeyAddress + TI_OTP_APLL_LDO_TRIM_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into APLL LDO trim reg
                //
                EALLOW;
                regReadAfterWrite_16(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLLDOTRIM,
                                     APLL_DOT_TRIM_MASK,
                                     apllldoTrimData, false);
                EDIS;

                apllTrimStatus |= APLL_LDO_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            apllTrimStatus |= APLL_LDO_TRIM_LOADED;
        }
    }

    return ((apllTrimStatus == APLL_LDO_TRIM_LOADED) ? true : false);
}

/**
* CPU1BROM_setFlashTrims() - Write refsys trims from OTP to registers
*
* \brief Configure REFSYS (Flash) TRIM
*
* Design: \ref did_config_flash_usecase did_safety_critical_trims_flash_algo
* Requirement: REQ_TAG(C2000BROM-335)
*
*/
bool CPU1BROM_setFlashTrims(void)
{
    uint32_t trimKeyAddresses[NUM_OF_TEST_INSERTIONS];
    uint32_t validTrimKeyAddress = 0UL;
    uint32_t flashTrimStatus = 0UL;

    //
    // Flash Pump Trim
    //
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_PUMP_TRIM_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_PUMP_TRIM_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_PUMP_TRIM_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t flashPumpTrim0Data = regDoubleRead_32(validTrimKeyAddress + TI_OTP_NWFLASH_PUMP_TRIM0_OFFSET, false);
            uint32_t flashPumpTrim1Data = regDoubleRead_32(validTrimKeyAddress + TI_OTP_NWFLASH_PUMP_TRIM1_OFFSET, false);
            uint32_t flashPumpTrim2Data = regDoubleRead_32(validTrimKeyAddress + TI_OTP_NWFLASH_PUMP_TRIM2_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into flashnw reg
                //
                EALLOW;
                regReadAfterWrite_32bp((BROM_FLASHNW_BASE + FLASH_O_PUMPTRIM0),
                                     FLASH_PUMPTRIM0_MASK,
                                     flashPumpTrim0Data, false);

                regReadAfterWrite_32bp((BROM_FLASHNW_BASE + FLASH_O_PUMPTRIM1),
                                     FLASH_PUMPTRIM1_MASK,
                                     flashPumpTrim1Data, false);

                regReadAfterWrite_32bp((BROM_FLASHNW_BASE + FLASH_O_PUMPTRIM2),
                                     FLASH_PUMPTRIM2_MASK,
                                     flashPumpTrim2Data, false);
                EDIS;

                flashTrimStatus |= FLASH_PUMP_TRIM_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            flashTrimStatus |= FLASH_PUMP_TRIM_LOADED;
        }
    }

    //
    // Flash PUMP Trim Read
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_PUMP_READ_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_PUMP_READ_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_PUMP_READ_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t pumpTrimRead = regDoubleRead_32(validTrimKeyAddress + TI_OTP_NWFLASH_PUMP_TRIM_READ_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into flashnw reg
                //
                EALLOW;
                regReadAfterWrite_32bp((BROM_FLASHNW_BASE + FLASH_O_PUMPTRIMREAD),
                                     FLASH_PUMPTRIMREAD_MASK,
                                     pumpTrimRead, false);
                EDIS;

                flashTrimStatus |= FLASH_PUMP_TRIM_READ_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error). 
            // So boot code will proceed with default values.
            //
            flashTrimStatus |= FLASH_PUMP_TRIM_READ_LOADED;
        }
    }

    //
    // Flash B0 Trim 0
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_B0_TRIM0_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_B0_TRIM0_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_B0_TRIM0_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t flashB0Trim1Data = regDoubleRead_32(validTrimKeyAddress + TI_OTP_NWFLASH_B0_TRIM0_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into flashnw reg
                //
                EALLOW;
                regReadAfterWrite_32bp((BROM_FLASHNW_BASE + FLASH_O_BANK0TRIM0),
                                     FLASH_BANK0TRIM0_MASK,
                                     flashB0Trim1Data, false);
                EDIS;

                flashTrimStatus |= FLASH_BANK0_TRIM0_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            flashTrimStatus |= FLASH_BANK0_TRIM0_LOADED;
        }
    }

    //
    // Flash B0 Trim read (Repair)
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_B0_TRIM_READ_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_B0_TRIM_READ_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_NWFLASH_B0_TRIM_READ_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t flashB0Trim2Data = regDoubleRead_32(validTrimKeyAddress + TI_OTP_NWFLASH_B0_TRIM_READ_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into flashnw reg
                //
                EALLOW;
                regReadAfterWrite_32bp((BROM_FLASHNW_BASE + FLASH_O_BANK0TRIMREAD),
                                     FLASH_BANK0TRIMREAD_MASK,
                                     flashB0Trim2Data, false);
                EDIS;

                flashTrimStatus |= FLASH_BANK0_TRIM_READ_LOADED;
            }
        }
        else
        {
            //
            // In this case there is no valid trim, but there is no trim error.
            // (trims written with 0xFFFF and NO ECC error).
            // So boot code will proceed with default values.
            //
            flashTrimStatus |= FLASH_BANK0_TRIM_READ_LOADED;
        }
    }

    return ((flashTrimStatus == (FLASH_PUMP_TRIM_LOADED | FLASH_PUMP_TRIM_READ_LOADED |
             FLASH_BANK0_TRIM0_LOADED |FLASH_BANK0_TRIM_READ_LOADED)) ? true : false);
}


/**
* CPU1BROM_devcalInit() - Write ADC Trims that includes Reftrim, Offset trim and INL trim
*
*
* \brief Set Dev Cal Trim function
*
* Design: \ref did_devcal_non_critical_adc_trims_algo did_devcal_non_critical_dac_trims_algo
*              did_devcal_non_critical_pmm_trims_algo
*
* Requirement: REQ_TAG(C2000BROM-167), REQ_TAG(C2000BROM-168), REQ_TAG(C2000BROM-169),
*              REQ_TAG(C2000BROM-170), REQ_TAG(C2000BROM-171)
*
*
*/
void CPU1BROM_devcalInit(void)
{
    uint32_t trimKeyAddresses[NUM_OF_TEST_INSERTIONS];
    uint32_t validTrimKeyAddress = 0UL;
    uint32_t clockstate;

    //Set PORBORFILTERTRIM ( PMMVMONTRIM[19:16] ) to 1000 per analog design
    HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMVMONTRIM) |= 0x00080000UL;

    //
    // ADC Reference Trim
    // (FT is skipped)
    //
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_ADCREF_FT_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_ADCREF_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_ADCREF_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            //
            // Read trim data from valid test insertion
            //
            uint32_t adcrefTrimData = regDoubleRead_32(validTrimKeyAddress + TI_OTP_ADCREF_TRIM_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into ANAREFTRIMA reg
                //
                EALLOW;
                regReadAfterWrite_32(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMA,
                                     0xFFFFFFFFUL,
                                     adcrefTrimData, false);
                EDIS;
            }
        }
    }
    // save ADC peripheral clock state
    clockstate = HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR13);
    EALLOW;
    // turn on ADC peripheral clocks
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR13) |= (uint32_t)(SYSCTL_PCLKCR13_ADC_A |  SYSCTL_PCLKCR13_ADC_C);
    //
    // Delay 5 cycles
    //
    asm(" MOV    @T,#5 ");
    asm(" RPT    @T \
        || NOP ");
    EDIS;

    //
    // Read trim data from valid test insertion

    //
    // ADC offset Trim
    // (FT is skipped)
    //
    validTrimKeyAddress = 0UL;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_ADCOFF_FT_TRIM_KEY_ADDRESS; // should be 0x0
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_ADCOFF_MP3_TRIM_KEY_ADDRESS;
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_ADCOFF_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            uint16_t adaAOffTrim = regDoubleRead_16(validTrimKeyAddress + TI_OTP_ADCA_OFFTRIM_OFFSET, false) & 
                                                             (ADC_OFFTRIM_OFFTRIMH_M | ADC_OFFTRIM_OFFTRIM_M);
            
            uint16_t adaCOffTrim = regDoubleRead_16(validTrimKeyAddress + TI_OTP_ADCC_OFFTRIM_OFFSET, false) & 
                                                             (ADC_OFFTRIM_OFFTRIMH_M | ADC_OFFTRIM_OFFTRIM_M);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into ADCAOFFTRIM and ADCCOFFTRIM reg
                //
                EALLOW;
                // populate ADC offset trims (8 bit LSB only by default, which is 2.5V as this is bit-packed, final trim to be set drivelib function)
                HWREGH(ADCA_BASE + ADC_O_OFFTRIM) = adaAOffTrim;
                HWREGH(ADCC_BASE + ADC_O_OFFTRIM) = adaCOffTrim;
                EDIS;
            }
        }
    }
    //
    // Read trim data from valid test insertion

    //
    // ADC INL Trim
    // (FT & MP3 are skipped)
    //
    validTrimKeyAddress = 0;
    trimKeyAddresses[FT_TRIM_KEY_INDEX] = TI_OTP_ADCINL_FT_TRIM_KEY_ADDRESS; // should be 0x0
    trimKeyAddresses[MP3_TRIM_KEY_INDEX] = TI_OTP_ADCINL_MP3_TRIM_KEY_ADDRESS; // should be 0x0
    trimKeyAddresses[MP1_TRIM_KEY_INDEX] = TI_OTP_ADCINL_MP1_TRIM_KEY_ADDRESS;

    //
    // Determine which test insertion has valid trim key (Priority FT>MP3>MP1)
    //
    if(NO_TRIM_ERROR == CPU1BROM_getValidTestInsertion(trimKeyAddresses, &validTrimKeyAddress))
    {
        if(validTrimKeyAddress != 0UL)
        {
            uint32_t adaAInlTrim2 = regDoubleRead_32(validTrimKeyAddress + TI_OTP_ADCA_INLTRIM2_OFFSET, false);
            uint32_t adaAInlTrim3 = regDoubleRead_32(validTrimKeyAddress + TI_OTP_ADCA_INLTRIM3_OFFSET, false);
            uint32_t adaCInlTrim2 = regDoubleRead_32(validTrimKeyAddress + TI_OTP_ADCC_INLTRIM2_OFFSET, false);
            uint32_t adaCInlTrim3 = regDoubleRead_32(validTrimKeyAddress + TI_OTP_ADCC_INLTRIM3_OFFSET, false);

            //
            // Check that no ECC errors occurred upon reading the trim
            //
            if(NO_TRIM_ERROR == CPU1BROM_checkECCErrors())
            {
                //
                // Write trim data into INLTRIM2 and INLTRIM3 regs
                //
                EALLOW;
                // populate INL trim regs 2 and 3
                HWREG(ADCA_BASE + ADC_O_INLTRIM2) = adaAInlTrim2;
                HWREG(ADCA_BASE + ADC_O_INLTRIM3) = adaAInlTrim3;
                HWREG(ADCC_BASE + ADC_O_INLTRIM2) = adaCInlTrim2;
                HWREG(ADCC_BASE + ADC_O_INLTRIM3) = adaCInlTrim3;
                EDIS;
            }
        }
    }

    // restore ADC peripheral clock state
    EALLOW;
    HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR13) = clockstate;
    EDIS;
}

//
// CPU1BROM_getValidTestInsertion
//
// testKeyAddresses - Pointer to array with the 3 trim key addresses
// validTestKeyAddress - The valid test key address returned by the function
//                       (only valid if the function returns true)
//
// This functions reads the test insertion trim keys in priority order and
// determines which (if any) are valid to use for trimming.
//
// Note a test key address of zero indicates this test insertion isn't
// used and that test insertion will be skipped
//
// Returns true if a valid test insertion is determined and non ECC error occurred.
// Returns false if there is no valid test insertion with passing ECC.
//
/**
* \brief Get trim key used for trimming
*
* Design: \ref did_safety_critical_trims_support_multiple_test_insertions_algo
*              did_safety_critical_trims_invalid_key_skip_load_algo
* Requirement: REQ_TAG(C2000BROM-330), REQ_TAG(C2000BROM-331)
*
*/
bool CPU1BROM_getValidTestInsertion(const uint32_t * const testKeyAddresses, uint32_t *validTestKeyAddress)
{
    uint32_t trimKey;
    uint16_t entryAddress;
    uint16_t i;
    bool keyNoErr = false;
    bool breakLoop = false;

    //
    // CPU1 Patch/Escape Point 1
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_1;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Determine Trims to use (FT, MP3, MP1)
    //
    for(i = 0U; ((i < NUM_OF_TEST_INSERTIONS) && (!breakLoop)); i++)
    {
        //
        // For trims that don't check all test insertions,
        // a key address of 0x0 indicates to skip that check
        //
        if(SKIP_TEST_INSERTION != testKeyAddresses[i])
        {
            //
            // Clear SEC/DED ECC error status
            //
            EALLOW;
            HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
            HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
            EDIS;

            //
            // Read trim key
            //
            trimKey = HWREG(testKeyAddresses[i]);

            //
            // Check for valid key and SEC/DED errors
            //
            if((TRIM_32BIT_KEY == trimKey) && 
			   (NO_ERROR == HWREAD_DED_STATUS))
            {
                //
                // Store valid trim key address
                //
                *validTestKeyAddress = testKeyAddresses[i];

                keyNoErr = true;
                breakLoop = true;
				
				//
				// to log if there is an SEC error
				//
				(void)CPU1BROM_checkECCErrors();
            }
            // Invalid key, NO ECC error
            else if((TRIM_32BIT_KEY != trimKey) &&
               (NO_ERROR == HWREAD_SEC_STATUS) &&
               (NO_ERROR == HWREAD_DED_STATUS))
            {

                // Don't reset the device
                keyNoErr = true;

                // Loop again to check next insertion
            }
			else if((TRIM_32BIT_KEY == trimKey) && 
			        (ERROR == HWREAD_DED_STATUS))
            {
				keyNoErr = false;
                breakLoop = true;
				
			    //
				// to log if there is an DED error
				//
				(void)CPU1BROM_checkECCErrors();
            }
            else
            {
				//
				// Invalid key with SEC/DED error will reach here.
				// Check next iteration or return trim error
				//
                keyNoErr = false;
            }
        }
    }

    return(keyNoErr);
}

//
// CPU1BROM_checkECCErrors
//
// This function checks for ECC errors and logs any identified errors
//
// Note trim values must be read before calling API
//
// Returns true if no error or only SEC, else returns false if DED
//
/**
* \brief Check ECC Errors
*
* Design: \ref did_safety_critical_trims_ecc_error_algo
*              did_safety_critical_trims_error_log_algo
* Requirement: REQ_TAG(C2000BROM-333), REQ_TAG(C2000BROM-334), REQ_TAG(C2000BROM-335)
*
*/
static inline bool CPU1BROM_checkECCErrors(void)
{
    bool retVal = false;

    //
    // Check for ECC DED
    //
    if(NO_ERROR == HWREAD_DED_STATUS)
    {
        //
        // Log error if SEC occurred
        //
        if(ERROR == HWREAD_SEC_STATUS)
        {
            LOG_TRIMERROR_SEC();
        }

        retVal = true;
    }
    else
    {
        //
        // Log the ECC DED error
        //
        LOG_TRIMERROR_DED();

        retVal = false;
    }

    return(retVal);
}

/*flag position means location or start of flag*/
static inline bool isBootOTPConfigFlagSet(uint32_t bootConfig, uint32_t flagPosition)
{
    bool retVal = false;
    if ((BOOTPIN_CONFIG_KEY == ((bootConfig & BOOTPIN_COFIG_KEY_MASK) >> BOOTPIN_COFIG_KEY_START)) &&
        /*checking that the flag is set*/
        (0U != (bootConfig & ((uint32_t)(1UL << flagPosition)))))
    {
        retVal = true;
    }

    return (retVal);
}

static uint16_t CPU1BROM_WDGSelfTestAndEnable(uint32_t bootConfig)
{
    uint16_t status = ERROR;

    if(true == isBootOTPConfigFlagSet(bootConfig, BOOT_CONFIG_ENABLE_WD_S))
    {
        uint32_t currWatchdogCountValue = 0UL;

        //
        //with the configured watchdog clock 2 instruction cycles are sufficient
        //to increment watchdog count by 1. Configuring the timeout value to 10
        //which is equivalent to 10 * 24 cycles the while loop below takes ~ 24 cycles
        //
        uint32_t WDSelftestTimeoutValue = 10UL;

        //
        //set watchdog prescale and predivide to min values so as to ensure maximum value for WD clock
        //
        SysCtl_setWatchdogPrescaler(WD_PRESCALE_VALUE_MIN);
        SysCtl_setWatchdogPredivider(WD_PREDIV_VALUE_MIN);
        SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);
        SysCtl_enableWatchdog();

        //
        //reset the counter to 0
        //
        SysCtl_serviceWatchdog();

        //
        //wait until watchdog counter increments by 1
        //
        while((currWatchdogCountValue < 1UL)&&(WDSelftestTimeoutValue > 0UL))
        {
            currWatchdogCountValue = SysCtl_getWatchdogCounterValue();
            WDSelftestTimeoutValue--;
        }

        //
        //disable the watchdog before validation of current watchdog count
        //
        SysCtl_disableWatchdog();

        //
        //the watchdog counter value read should be greater than 0
        //
        if(currWatchdogCountValue > 0UL)
        {
            /*watchdog self test successful*/

            //
            //reset the prescale and prediv values for 104ms timeout value
            //
            SysCtl_setWatchdogPrescaler(WD_PRESCALE_VALUE);
            SysCtl_setWatchdogPredivider(WD_PREDIV_VALUE);
            SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);
            SysCtl_enableWatchdog();
            status = NO_ERROR;
        }
    }
    else
    {
        //
        // In case OTP is not configured then return NO_ERROR.
        // But disable the watchdog as key is not valid (will prevent unexpected resets in fresh device)
        // or wdg enable flag is not set.
        //
        status = NO_ERROR;
        SysCtl_disableWatchdog();
    }

    return status;
}

/**
* CPU1BROM_setupDeviceSystems - Adjust dividers, setup flash
*                               configurations, flash power up,
*                               and trim PMM/INTOSC/APLL
*
*
* \brief Set up Device Systems
*
* Design: \ref did_setup_device_usecase did_sysclk_div_config_interface
*              did_flash_pump_default_config_algo did_safety_flash_pump_timeout_algo
*              did_flash_pump_powerup_algo did_flash_bank_powerup_algo did_patch_points_algo
*              did_safety_wdg_enable_algo did_safety_wdg_enable_interface did_trigger_apll_lock_usecase
*              did_patch_points_key_check_algo did_hw_assisted_patching_algo did_clocking_algo did_clocking
*
* Requirement: REQ_TAG(C2000BROM-143), REQ_TAG(C2000BROM-144), REQ_TAG(C2000BROM-148),
*              REQ_TAG(C2000BROM-145), REQ_TAG(C2000BROM-216), REQ_TAG(C2000BROM-148),
*              REQ_TAG(C2000BROM-146), REQ_TAG(C2000BROM-147), REQ_TAG(C2000BROM-212),
*              REQ_TAG(C2000BROM-238), REQ_TAG(C2000BROM-214), REQ_TAG(C2000BROM-245),
*              REQ_TAG(C2000BROM-332), REQ_TAG(C2000BROM-365), REQ_TAG(C2000BROM-329)
*
* Set up Device Systems
*
*/
#ifdef LDRA_FILEIO
extern void __TI_auto_init();
#endif// LDRA_FILEIO
void CPU1BROM_setupDeviceSystems(void)
{
    /*LDRA_NOANALYSIS*/
    uint16_t entryAddress;
    bool PMMTrimNoError = true;
    bool OSCTrimNoError = true;
    bool APLLTrimNoError = true;
    bool FlashTrimNoError = true;
    volatile bool TRIMNoError = true;
    uint32_t jtagMMRValue;

#ifdef LDRA_FILEIO
    __TI_auto_init();
#endif// LDRA_FILEIO

    /*LDRA_ANALYSIS*/
    readNoError = true;
    CPU1BROM_nmiStatus = 0U;
    CPU1BROM_trimKeySEC = 0U;

    // Initialize the variable before usage.
    CPU1BROM_bootStatus = 0;
	
    //
    // Set the divider to /1 before waking up flash
    //
    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 0x0U;
    EDIS;

    //
    // Disable wdg here to simplify the wait for 2T ready status loop.
    // Wdg my get enabled later in this fucntion based on OTP value.
    //
    SysCtl_disableWatchdog();

    //
    // Wait for Flash 2T to be ready, timeout in worst case.
    //
    // Time needed for 2T ready is 5us. As worst case timeout ~100us is chosen.
    // At 15MHz 100us needs 1500 counts. Cycles per below loop is ~20.
    uint32_t timeOut = 1500UL/20UL;
    while(BANK2TRDY_M != (HWREG(BROM_FLASHNW_BASE + FLASH_O_STATMODE) & BANK2TRDY_M))
    {
        if(timeOut == 0U)
        {
            CPU1BROM_bootStatus |= (CPU1_BOOTROM_FLASH_2T_NOT_READY | CPU1_BOOTROM_TRIM_LOAD_ERROR);

            //
            // Check JTAG MMR and if valid, run DFT boot flow
            //
            CPU1BROM_runDFTBoot(regDoubleRead_32((CPUSYS_BASE + SYSCTL_O_JTAG_MMR_REG), false));

            asm("   ESTOP0");
            SysCtl_resetDevice();
        }
        timeOut--;
    }

    //
    // Clear SEC/DED ECC error status
    // Read and store patch point key in a variable
    // If there is ECC error, invalidate the key
    //
    EALLOW;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
    EDIS;

    swPatchKey = TI_OTP_SW_PATCH_POINT_KEY;
    if(false == CPU1BROM_checkECCErrors())
    {
        swPatchKey = 0xFFFFFFFFUL;
    }

    EALLOW;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
    EDIS;

    //
    // CPU1 Patch/Escape Point 1. Check patch point key match.
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_1;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Update flash read wait-state
    //
    EALLOW;
    HWREG(FLASH0CTRL_BASE + FLASH_O_FRDCNTL) = ((HWREG(FLASH0CTRL_BASE + FLASH_O_FRDCNTL) &
             ~(uint32_t)FLASH_FRDCNTL_RWAIT_M  &
             ~(uint32_t)FLASH_FRDCNTL_TRIMENGRRWAIT_M) |
              (((uint32_t)CPU1_FLASH_15MHZ_RWAIT << FLASH_FRDCNTL_RWAIT_S) |
              ((uint32_t)CPU1_FLASH_15MHZ_TRIMENGRRWAIT << FLASH_FRDCNTL_TRIMENGRRWAIT_S)));
    EDIS;

    //
    // Double Read the OTP Configuration Word and store in global variable
    //
    CPU1BROM_bootConfigureWord = regDoubleRead_32(OTP_BOOT_CONFIGURE_WORD_ADDRESS, false);

    //
    // Check to determine if to enable watchdog during boot flow
    // Check if there is NO ECC (SEC/DED) error while reading key value
    //
    if((HWREAD_TRIM_ERROR_STATUS & (TRIM_ERROR_DED | TRIM_ERROR_SEC)) == NO_ERROR)
    {
        if(ERROR == CPU1BROM_WDGSelfTestAndEnable(CPU1BROM_bootConfigureWord))
        {
            CPU1BROM_bootStatus |= CPU1_BOOTROM_WATCHDOG_SELFTEST_FAIL;
        }
        else
        {
            CPU1BROM_bootStatus &= ~CPU1_BOOTROM_WATCHDOG_SELFTEST_FAIL;
        }
    }
    else
    {
        SysCtl_disableWatchdog();
    }

#ifndef LDRA_FILEIO
    DEBUGLABEL(WatchdogEnableTest);
#endif

    //
    // Perform HW assisted ROM patching registers config
    //
    CPU1BROM_configureHardwarePatching(CPU1BROM_HW_PATCH_TABLE_START);

    //
    // CPU1 Patch/Escape Point 6
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_6;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Perform PMM, INTOSC, APLL, and FLASH trimming
    //
    PMMTrimNoError = CPU1BROM_setPMMTrims();
    OSCTrimNoError = CPU1BROM_setOscTrims();
    APLLTrimNoError = CPU1BROM_setAPLLTrims();
    FlashTrimNoError = CPU1BROM_setFlashTrims();
    TRIMNoError = (PMMTrimNoError && OSCTrimNoError && APLLTrimNoError && FlashTrimNoError);

    //
    // CPU1 Patch/Escape Point 6
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_6;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Set power up control indicator that trims are loaded
    // (TRIMOVER status will be checked later)
    //
    if((TRIMNoError == true) && (readNoError == true))
    {
        //
        // Enable TRIM LOADED signal to analog sysctl
        //
        EALLOW;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_PWRUPCTL) |= ASYSCTL_PWRUPCTL_TRIMLOAD;
        EDIS;

        //
        // Wait for "Trim Over" to indicate PMM trims have taken effect
        // (Timeout is ~600us at 15MHz with ~20cycles per loop)
        // 600 us is chosen as double of 300us from spec to account for worst case scenarios.
        //
        timeOut = 9000UL/20UL;
        while((HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_PWRUPSTS) & ASYSCTL_PWRUPSTS_TRIMOVER)!=
               ASYSCTL_PWRUPSTS_TRIMOVER)
        {
            if(timeOut == 0U)
            {
                TRIMNoError = TRIMNoError && false;
                break;
            }
            timeOut--;
        }

        //
        // Flash 1T trim enable (trim data is now valid)
        //
        if(TRIMNoError == true)
        {
            EALLOW;
            HWREGH(BROM_FLASHNW_BASE + FLASH_O_TRIMCTL) |= FLASH_TRIMCTL_ENABLE;
            EDIS;
        }

        //
        // INTOSC settling time
        // ~25uS
        //
        asm(" MOV    @T,#475 ");
        asm(" RPT    @T \
              || NOP ");
    }


    //
    // Check, and if valid, run DFT boot flow. Read as late as possible to
    // enable test bench to get longer window for updating the MMR.
    //
    jtagMMRValue = regDoubleRead_32((CPUSYS_BASE + SYSCTL_O_JTAG_MMR_REG), false);
    CPU1BROM_runDFTBoot(jtagMMRValue);

    //
    // Set trim load status
    //
    if((TRIMNoError == true) && (readNoError == true))
    {
        CPU1BROM_bootStatus &= ~CPU1_BOOTROM_TRIM_LOAD_ERROR;
    }
    else
    {
        CPU1BROM_bootStatus |= CPU1_BOOTROM_TRIM_LOAD_ERROR;
    }

    //
    // If trims are loaded successfully lock them, otherwise reset the device.
    // Skip the reset regardless of errors if boot test mode is requested.
    //
    if(DFT_BOOT_TEST_ALT_KEY == jtagMMRValue)
    {
        //
        // Boot test mode--proceed regardless of errors
        //
    }
    else if((TRIMNoError == true) && (readNoError == true))
    {
        //
        // Flash Trim locking
        //
        HWREG(BROM_FLASHNW_BASE + FLASH_O_TRIMLOCK) = TRIMLOCKOTHER | TRIMLOCKREAD;

        //
        // Flash Trim committing
        //
        HWREG(BROM_FLASHNW_BASE + FLASH_O_TRIMCOMMIT) = TRIMCOMMITOTHER | TRIMCOMMITREAD;
    }
    else
    {
        asm("   ESTOP0");
        SysCtl_resetDevice();
    }


    //
    // Check if DED Trim error occurred during trimming process
    // or SEC error occurred during trim key read, if yes, reset device
    // If watchdog test failed reset the device.
    // Skip the reset regardless of errors if boot test mode is requested.
    //
    if(((NO_ERROR != (HWREAD_TRIM_ERROR_STATUS & TRIM_ERROR_DED)) ||
        (CPU1_BOOTROM_WATCHDOG_SELFTEST_FAIL == (CPU1BROM_bootStatus & CPU1_BOOTROM_WATCHDOG_SELFTEST_FAIL))) &&
       (DFT_BOOT_TEST_ALT_KEY != jtagMMRValue))
    {
        //
        // Trim error occurred
        //
        CPU1BROM_bootStatus |= CPU1_BOOTROM_TRIM_LOAD_ERROR;
        asm("   ESTOP0");
        SysCtl_resetDevice();
    }

	//
	// If enabled via OTP, Validate PLL o/p with DCC and Switch sysclk to PLL
	//
	if((BOOTPIN_CONFIG_KEY == ((CPU1BROM_bootConfigureWord & 0xFF000000UL) >> 24U))
	      && (BOOT_CONFIGURE_ENABLE_PLL == (CPU1BROM_bootConfigureWord & 0x3U)))
	{
        //
        // Set Multiplier and divider for 190MHz and trigger the lock process.
        // This function will not switch the sysclk to PLL output.
        //
        CPU1BROM_triggerSysPLLLock(APLL_MULT_38, APLL_DIV_2);

        //
        // CPU1 Patch/Escape Point 15
        //
        if(SW_PATCH_POINT_KEY == swPatchKey)
        {
            entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_15;
            if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
            {
                //
                // If OTP is programmed, then call OTP patch function
                //
                EXECUTE_ESCAPE_POINT(entryAddress);
            }
        }

        //
        // RAM init on POR
        //
#ifndef LDRA_FILEIO
        // Note: RAM init will clear stack and other RAM memories, any variables
        //       initialized that are using these memories must be re-initialized
        //       after running RAM init. Additionally in this case, the return
        //       address is stored in RPC and doens't require re-initializing.
        //
        if(SYSCTL_RESC_POR == (HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)))
        {
            EALLOW;
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT) |= (MEMCFG_DXINIT_INIT_M0 | MEMCFG_DXINIT_INIT_M1 | MEMCFG_DXINIT_INIT_PIEVECT);
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT) |= (MEMCFG_LSXINIT_INIT_LS0 | MEMCFG_LSXINIT_INIT_LS1);
            EDIS;

            //
            // Wait for 2KB RAM inits to complete
            // (512 + 32 buffer cycles)
            //
            asm(" MOV  @T,   #544 ");
            asm(" RPT  @T || NOP ");

            //
            // Reset CPU1BROM_bootConfigureWord since M0RAM cleared
            //
            CPU1BROM_bootConfigureWord = regDoubleRead_32(OTP_BOOT_CONFIGURE_WORD_ADDRESS, true);
        }
#endif // LDRA_FILEIO

        //
        // This function will try to switch the sysclk to PLL output.
        //
        (void)CPU1BROM_switchToPLL(CPU1BROM_bootConfigureWord);
	}
	else
	{
	    //
	    // RAM init on POR
	    //
#ifndef LDRA_FILEIO
        // Note: RAM init will clear stack and other RAM memories, any variables
        //       initialized that are using these memories must be re-initialized
        //       after running RAM init. Additionally in this case, the return
        //       address is stored in RPC and doens't require re-initializing.
	    //
	    if(SYSCTL_RESC_POR == (HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)))
	    {
            EALLOW;
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT) |= (MEMCFG_DXINIT_INIT_M0 | MEMCFG_DXINIT_INIT_M1 | MEMCFG_DXINIT_INIT_PIEVECT);
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT) |= (MEMCFG_LSXINIT_INIT_LS0 | MEMCFG_LSXINIT_INIT_LS1);
            EDIS;

            //
            // Wait for 2KB RAM inits to complete
            // (512 + 32 buffer cycles)
            //
            asm(" MOV  @T,   #544 ");
            asm(" RPT  @T || NOP ");

            //
            // Reset CPU1BROM_bootConfigureWord since M0RAM cleared
            //
            CPU1BROM_bootConfigureWord = regDoubleRead_32(OTP_BOOT_CONFIGURE_WORD_ADDRESS, true);
        }
#endif // LDRA_FILEIO
    }

    //
    // Wait for Flash 1T to be ready
    // (Watchdog handles timeout)
    //
    while(BANK1TRDY_M != (HWREG(BROM_FLASHNW_BASE + FLASH_O_STATMODE) & BANK1TRDY_M))
    {}
}

/**
* CPU1BROM_performDeviceConfiguration - Set device configuration registers
*                                       from OTP
*
*
* \brief Device configuration function
*
* Design: \ref did_device_config_usecase did_dcc_verify_pll_clock_algo did_device_config_algo did_aes_disable_algo
*              did_dcc_disable_algo did_safety_dcc_disable_algo did_part_id_algo did_external_resistor_config_algo
*              did_die_id_config_algo
* Requirement: REQ_TAG(C2000BROM-150), REQ_TAG(C2000BROM-149), REQ_TAG(C2000BROM-369)
*              REQ_TAG(C2000BROM-151), REQ_TAG(C2000BROM-215), REQ_TAG(C2000BROM-366)
*              REQ_TAG(C2000BROM-247), REQ_TAG(C2000BROM-352)
*
* Device configuration function
*
*/
void CPU1BROM_performDeviceConfiguration(void)
{
    uint16_t entryAddress;
    uint32_t pkgType;

    EALLOW;

    //
    // Set PARTIDL
    //
	HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) = HWREAD_TI_OTP_PARTID_L;
	HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) = HWREAD_TI_OTP_PARTID_H;

	//
	// Device Configuration Registers
	//
	HWREG(DEVCFG_BASE + SYSCTL_O_DC11)   = BROM_DCX_ENABLE_HIGH | regDoubleRead_16(TI_OTP_ADDR_DC11, true);
    HWREG(DEVCFG_BASE + SYSCTL_O_DC31)   = BROM_DCX_ENABLE_HIGH | regDoubleRead_16(TI_OTP_ADDR_DC31, true);

    //
    // Load PKGTYPE - If KEY is programmed in TI_OTP_PKG_TYPE[15:8] == 0x5A
    //
    if(((HWREAD_TI_OTP_PKG_TYPE & 0xFF00U) >> 8U) == PKG_TYPE_KEY)
    {
        pkgType = (((uint32_t)PKG_TYPE_KEY << SYSCTL_PKGTYPE_BROM_KEY_S) |
                   ((uint32_t)HWREAD_TI_OTP_PKG_TYPE & 0x0000000FUL));

        //
        // Write package type with key
        //
        HWREG(DEVCFG_BASE + SYSCTL_O_PKGTYPE) = pkgType;

        //
        // For 32 QFN package configure supply override with configuration from OTP.
        //
        if(PKG_TPYE_32_QFN == (pkgType & (SYSCTL_PKGTYPE_PKGTYPE_M << SYSCTL_PKGTYPE_PKGTYPE_S)))
        {
            HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) =
                   (HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) & ~ASYSCTL_ANAREFCTL_ANAREFSEL_SUP_OVERRIDE) |
                   (regDoubleRead_16(TI_OTP_ANAREFCTL_ADDR, true) & ASYSCTL_ANAREFCTL_ANAREFSEL_SUP_OVERRIDE);
        }
    }

	//
	// DC CPUROM Configuration
	//
    HWREG(DEVCFG_BASE + SYSCTL_O_CPUROM_DC1)   = BROM_DCX_ENABLE_HIGH | regDoubleRead_16(TI_OTP_ADDR_CPUROM_DC1, true);
    HWREG(DEVCFG_BASE + SYSCTL_O_CPUROM_DC2)   = BROM_DCX_ENABLE_HIGH | regDoubleRead_16(TI_OTP_ADDR_CPUROM_DC2, true);

	//
	// External Resistor Configuration and GPIO
	//
	HWREG(DEVCFG_BASE + SYSCTL_O_PERCNF1) = (((uint32_t)regDoubleRead_16(TI_OTP_ADDR_PERCNF1, true) & PERCNF1_MASK)) << 16UL;
	HWREGH(DEVCFG_BASE + SYSCTL_O_PERCNF2) = regDoubleRead_16(TI_OTP_ADDR_PERCNF2, true) & PERCNF2_MASK;
    EDIS;

    copyDIEID();

    //
    // CPU1 Patch/Escape Point 7
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_7;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

}

/**
* \brief Pullup Unbonded GPIOs
*
* Design: \ref did_pullup_unbonded_gpio_usecase did_package_type_enum_algo
*              did_unbonded_gpio_pullup_config_algo did_package_type_enum_algo
*              did_package_type_algo
* Requirement: REQ_TAG(C2000BROM-165), REQ_TAG(C2000BROM-151)
*
*
* Pullup Unbonded GPIOs
*
*/
static void CPU1BROM_enableUnbondedGpioPullups(uint16_t pin_count)
{
    uint32_t gpaValue = 0xFFFFFFFFUL;
    uint32_t gpbValue = 0xFFFFFFFFUL;

    switch(pin_count)
    {
        case 2U:
                gpaValue = TOPOARIA_48_QFP_PT_GPA;
                gpbValue = TOPOARIA_48_QFP_PT_GPB;
                break;
        case 3U:
                gpaValue = TOPOARIA_48_QFN_RGZ_GPA;
                gpbValue = TOPOARIA_48_QFN_RGZ_GPB;
                break;
        case 6U:
                gpaValue = TOPOARIA_64_QFP_PM_VREGENZ_GPA;
                gpbValue = TOPOARIA_64_QFP_PM_VREGENZ_GPB;
                break;
        case 7U:
                gpaValue = TOPOARIA_64_QFP_PM_GPA;
                gpbValue = TOPOARIA_64_QFP_PM_GPB;
                break;
        case 12U:
                gpaValue = TOPOARIA_32_QFN_RTM_GPA;
                gpbValue = TOPOARIA_32_QFN_RTM_GPB;
                break;
        default:
                //
                // Empty default to comply with MISRA
                //
                break;
    }

    EALLOW;
    regReadAfterWrite_32(GPIOCTRL_BASE + GPIO_O_GPAPUD,
                         GPA_WRITE_MASK,
                         regDoubleRead_32(GPIOCTRL_BASE + GPIO_O_GPAPUD, true) & gpaValue, true);

    regReadAfterWrite_32(GPIOCTRL_BASE + GPIO_O_GPBPUD,
                         GPB_WRITE_MASK,
                         regDoubleRead_32(GPIOCTRL_BASE + GPIO_O_GPBPUD, true) & gpbValue, true);
    EDIS;

}

/*inline function to derive flash entry point address*/
static inline uint32_t getFlashEntryPointAddress(uint32_t bootMode)
{
    uint32_t entryPointAddress = FLASH_ENTRY_POINT;

    uint32_t flashEntryPointAddress[]=
    {
        FLASH_ENTRY_POINT,
        FLASH_ENTRY_POINT_ALT1,
        FLASH_ENTRY_POINT_ALT2,
        FLASH_ENTRY_POINT_ALT3,
        FLASH_ENTRY_POINT_ALT4,
        FLASH_ENTRY_POINT_ALT5
    };

    if(TI_OTP_KEY_32B == HWREAD_FLASH_ENTRY_POINT_OVERRIDE_KEY)
    {
        entryPointAddress = HWREAD_FLASH_ENTRY_POINT_OVERRIDE_ADDR;
        /*do a validation on entry address and set it to default FLASH_ENTRY_POINT
         * in case the address does not map to flash region.
         */
        if((entryPointAddress < BROM_FLASH_ALL_START) ||(entryPointAddress >= BROM_FLASH_ALL_END))
        {
            entryPointAddress = FLASH_ENTRY_POINT;
        }
    }
    else
    {
        /*the flash alt values are like 0x0AU, 0x2A, 0x4A, 0x6A, ... 0xEA so
          the 1st nibble varies from 0x0 to 0xE in steps of 2. We use this to derive
          the index in flashEntryPointAddress array*/
        uint16_t flashEntryPointIndex = (uint16_t)((bootMode & 0x000000F0UL)>>4U)/2U;
        entryPointAddress = flashEntryPointAddress[flashEntryPointIndex];

    }

    return entryPointAddress;
}

/**
* CPU1BROM_startSystemBoot - CPU1 System Initialization procedure and boot
*                            selection
*
*
* \brief Start system boot
*
* Design: \ref did_start_system_boot_usecase did_wait_boot_algo did_ram_boot_algo did_wdg_enable_algo
*              did_flash_boot_options_algo did_gpreg2_key_algo did_gpreg2_error_sts_algo
*              did_gpreg2_cjtag_node_id_algo did_nmi_handlers_algo did_itrap_exception_handling_algo
*              did_safety_enable_watchdog_before_application_execution_algo did_disable_pll_algo did_boot_status_resource
*              did_flash_single_bit_error_algo did_safety_devcal_non_critical_trims_algo did_select_bootmode_usecase
*              did_gpreg2_config_algo did_boot_pins_compatibility_algo did_can_fd_boot_algo did_secureflash_boot_algo
*              did_parallel_boot_options_algo did_sci_boot_options_algo did_can_boot_options_algo did_can_fd_boot_options_algo
*              did_spi_boot_options_algo did_i2c_boot_options_algo did_gpreg2_config_algo did_boot_pins_compatibility_algo
*              did_parallel_boot_options_algo did_sci_boot_options_algo did_can_boot_options_algo did_safety_switch_to_pll_clock_usecase
*              did_can_fd_boot_options_algo did_spi_boot_options_algo did_i2c_boot_options_algo did_pll_lock_fail_status_usecase
*              did_enable_nmi_algo did_enable_watchdog_before_application_execution_algo did_boot_status_reset_algo
*              did_safety_mpost_gpreg2_enabled_algo did_safety_mpost_clock_rate_config_algo did_safety_mpost_fail_clock_config_skip_algo
*              did_safety_mpost_restore_cpu_variables_algo did_gpreg2_error_sts_interface did_mpost_otp_interface did_handle_por_xrs_resets_algo
*
*
*
* Requirement: REQ_TAG(C2000BROM-226), REQ_TAG(C2000BROM-172), REQ_TAG(C2000BROM-173),
*              REQ_TAG(C2000BROM-241), REQ_TAG(C2000BROM-160), REQ_TAG(C2000BROM-163),
*              REQ_TAG(C2000BROM-179), REQ_TAG(C2000BROM-174), REQ_TAG(C2000BROM-211),
*              REQ_TAG(C2000BROM-225), REQ_TAG(C2000BROM-227), REQ_TAG(C2000BROM-228),
*              REQ_TAG(C2000BROM-230), REQ_TAG(C2000BROM-161), REQ_TAG(C2000BROM-175),
*              REQ_TAG(C2000BROM-166), REQ_TAG(C2000BROM-240), REQ_TAG(C2000BROM-203),
*              REQ_TAG(C2000BROM-202), REQ_TAG(C2000BROM-181), REQ_TAG(C2000BROM-209),
*              REQ_TAG(C2000BROM-161), REQ_TAG(C2000BROM-162), REQ_TAG(C2000BROM-245),
*              REQ_TAG(C2000BROM-159), REQ_TAG(C2000BROM-167), REQ_TAG(C2000BROM-168),
*              REQ_TAG(C2000BROM-169), REQ_TAG(C2000BROM-170), REQ_TAG(C2000BROM-171),
*              REQ_TAG(C2000BROM-180), REQ_TAG(C2000BROM-183), REQ_TAG(C2000BROM-243),
*              REQ_TAG(C2000BROM-244), REQ_TAG(C2000BROM-160), REQ_TAG(C2000BROM-201),
*              REQ_TAG(C2000BROM-184), REQ_TAG(C2000BROM-209), REQ_TAG(C2000BROM-204),
*              REQ_TAG(C2000BROM-205), REQ_TAG(C2000BROM-206), REQ_TAG(C2000BROM-209),
*              REQ_TAG(C2000BROM-207), REQ_TAG(C2000BROM-208), REQ_TAG(C2000BROM-215),
*              REQ_TAG(C2000BROM-164), REQ_TAG(C2000BROM-176), REQ_TAG(C2000BROM-177),
*              REQ_TAG(C2000BROM-235), REQ_TAG(C2000BROM-236), REQ_TAG(C2000BROM-229),
*              REQ_TAG(C2000BROM-402), REQ_TAG(C2000BROM-403), REQ_TAG(C2000BROM-224)
*
* Start system boot
*
*/
uint32_t CPU1BROM_startSystemBoot(void)
{
	uint32_t ZxOtpGpreg2Errsts = 0UL;
	uint32_t ZxOtpGpreg2CjtagNodeId = 0UL;
	uint32_t ZxOtpGpreg2PbistConfig = 0UL;
    uint32_t appEntryAddress;
    uint16_t entryAddress;
    bool proceedToPBIST = false;

    DCSM_InitStatus dcsmInitStatus = DCSM_ERROR_NONE;
    //
    // On POR or XRS reset, init boot status variable
    // (Status is non-volatile on any other reset and up to user to clear)
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
        (SYSCTL_RESC_POR | SYSCTL_RESC_XRSN)) != 0U)
    {
        CPU1BROM_bootStatus = 0U;
    }

    //
    // On POR - Init PBIST status
    // (Retain status on any other reset)
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
        SYSCTL_RESC_POR) == SYSCTL_RESC_POR)
    {
        CPU1BROM_pbistStatus = 0U;
    }

    //
    // Init boot mode selection
    //
    CPU1BROM_bootMode = 0xFFFFFFFFUL;

    //
    // Update boot status - Boot has started
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_SYSTEM_START_BOOT;

    //
    // Initialize iTrap address
    //
    CPU1BROM_itrapAddress = 0xFFFFFFFFUL;

    //
    // Initialize NMI Status
    //
    CPU1BROM_nmiStatus = 0U;

    //
    // Before reading from OTP check if flash is ready. This is needed sysrsn boot flow as well.
    // Wait for Flash 2T and 1T to be ready
    //
    while((BANK2TRDY_M | BANK1TRDY_M) != (HWREG(BROM_FLASHNW_BASE + FLASH_O_STATMODE) & (BANK2TRDY_M | BANK1TRDY_M)))
    {}

    //
    // Clear SEC/DED ECC error status
    // Read and store patch point key in a variable
    // If there is ECC error, invalidate the key
    //
    EALLOW;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
    EDIS;

    swPatchKey = TI_OTP_SW_PATCH_POINT_KEY;
    if(false == CPU1BROM_checkECCErrors())
    {
        swPatchKey = 0xFFFFFFFFUL;
    }

    EALLOW;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_UCERRCLR) |= MEMCFG_UCERRCLR_CPURDERR;
    HWREG(MEMORYERROR_BASE + MEMCFG_O_CERRCLR) |= MEMCFG_CERRCLR_CPURDERR;
    EDIS;

    //
    // CPU1 Patch/Escape Point 2
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_2;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Initialize DCSM
    //
    dcsmInitStatus = CPU1BROM_initDCSM();

    if(DCSM_ERROR_LINKPOINTERERR_SET == dcsmInitStatus)
    {
        /*record the error and proceed with boot*/
        CPU1BROM_bootStatus |= CPU1_BOOTROM_DCSM_INIT_LP_ERROR_SET;
    }
    else
    {
        //
        // Update boot status - DCSM init is complete
        //
        CPU1BROM_bootStatus |= CPU1_BOOTROM_DCSM_INIT_COMPLETE;
    }

#ifndef LDRA_FILEIO
    DEBUGLABEL(DCSMinitComplete);
#endif

    //
    // Enable writing to the EALLOW protected registers
    //
    EALLOW;

    //
    // Lock DCx registers
    //
    HWREGH(DEVCFG_BASE + SYSCTL_O_DEVCFGLOCK2) |=
                                       SYSCTL_DEVCFGLOCK2_DC_PERCNF_PARTID;

    //
    // Lock HW Patching registers
    //
    HWREG(CPUSYS_BASE + SYSCTL_O_CPUSYSLOCK2) |= SYSCTL_CPUSYSLOCK2_BROMPATCH;


    //
    // Enable NMI
    //
    regReadAfterWrite_16(NMI_BASE + NMI_O_CFG, NMI_ENABLE_MASK, NMI_CFG_NMIE, true);

    //
    // Disable writing to the EALLOW protected registers
    //
    EDIS;

    // Capture any single bit errors after DCSM init
    // If the below address is valid then user has to
    // debug this further
    //
    CPU1BROM_corrErrorAddr  = HWREG(MEMCFG_BASE + MEMCFG_O_CCPUREADDR);

    //
    // CPU1 Patch/Escape Point 2
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_2;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Check Z2 or Z1 GPREG2 key to determine whether to configure
    // Error Status pin and cjtag node it and run PBIST memory test
    // (Z2 takes priority when programmed)
    //
	if(HWREAD_Z2_OTP_BOOT_GPREG2_KEY == GPREG2_KEY)
	{
        uint32_t z2Gpreg2 = regDoubleRead_32(Z2_OTP_BOOT_GPREG2, true);
		ZxOtpGpreg2Errsts      = ((z2Gpreg2 & (uint32_t)0x00000030UL) >> 4U);
		ZxOtpGpreg2CjtagNodeId = ((z2Gpreg2 & (uint32_t)0x0000000FUL));
		ZxOtpGpreg2PbistConfig = ((z2Gpreg2 & (uint32_t)0x000000C0UL) >> 6U);
	}
    else if(HWREAD_Z1_OTP_BOOT_GPREG2_KEY == GPREG2_KEY)
    {
        uint32_t z1Gpreg2 = regDoubleRead_32(Z1_OTP_BOOT_GPREG2, true);
        ZxOtpGpreg2Errsts      = ((z1Gpreg2 & (uint32_t)0x00000030UL) >> 4U);
        ZxOtpGpreg2CjtagNodeId = ((z1Gpreg2 & (uint32_t)0x0000000FUL));
        ZxOtpGpreg2PbistConfig = ((z1Gpreg2 & (uint32_t)0x000000C0UL) >> 6U);
	}
    else
    {
	    //
        // To avoid misra violation
		//
    }


	//
	// Zx OTP - Check for Key
	//
	if( (HWREAD_Z2_OTP_BOOT_GPREG2_KEY == GPREG2_KEY) ||
	                         (HWREAD_Z1_OTP_BOOT_GPREG2_KEY == GPREG2_KEY))
	{
        //
        // CPU1 Patch/Escape Point 3
        //
        if(SW_PATCH_POINT_KEY == swPatchKey)
        {
            entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_3;
            if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
            {
                //
                // If OTP is programmed, then call OTP patch function
                //
                EXECUTE_ESCAPE_POINT(entryAddress);
            }
        }

		//
		// Set ERROR_STS pin if enabled by user
		//
		if(ZxOtpGpreg2Errsts == ERRORSTS_PIN_24)
		{
			GPIO_setPinConfig(GPIO_24_ERRORSTS);
			GPIO_lockPortConfig(GPIO_PORT_A, 0x01000000U); //lock pin 24
		}
		else if(ZxOtpGpreg2Errsts == ERRORSTS_PIN_28)
		{
			GPIO_setPinConfig(GPIO_28_ERRORSTS);
			GPIO_setAnalogMode(28,GPIO_ANALOG_DISABLED);
			GPIO_lockPortConfig(GPIO_PORT_A, 0x10000000U); //lock pin 28
		}
		else if (ZxOtpGpreg2Errsts == ERRORSTS_PIN_29)
		{
			GPIO_setPinConfig(GPIO_29_ERRORSTS);	//
			GPIO_lockPortConfig(GPIO_PORT_A, 0x20000000U); //lock pin 29
		}
        else
        {
		    //
            // To avoid misra violation
			//
        }

		if(0UL != (HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
                  ((uint32_t)SYSCTL_RESC_POR | (uint32_t)SYSCTL_RESC_XRSN)))
		{
			//set CJTAGNODEID[3:0]
            EALLOW;
			HWREG(DEVCFG_BASE + SYSCTL_O_CJTAGNODEID) =
                              ((HWREG(DEVCFG_BASE + SYSCTL_O_CJTAGNODEID) &
                               ((uint32_t)0xF0UL)) | (ZxOtpGpreg2CjtagNodeId));
            EDIS;
		}

		if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)) == SYSCTL_RESC_POR)
		{
            uint16_t pllStatus = BROM_PLL_CONFIG_ERROR;
			uint16_t pbistConfig = ((uint16_t)ZxOtpGpreg2PbistConfig) & 0x3U;

            if(SW_PATCH_POINT_KEY == swPatchKey)
            {
                entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_3;
                if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
                {
                    //
                    // If OTP is programmed, then call OTP patch function
                    //
                    EXECUTE_ESCAPE_POINT(entryAddress);
                }
            }

            //
            // Call PBIST only if user has asked for it to RUN.
            //
			if(pbistConfig != GPREG2_PBIST_DISABLED)
			{
				//
                // Check LS RAMs to verify they have completed initialization, else
                // wait ~4200 cycles to guarantee RAM init is complete
                //
                if(HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE) == RAM_LSX_NOT_DONE)
                {
                    asm(" MOV    @T,#0x1068 ");
                    asm(" RPT    @T \
                          || NOP ");
                }

                if( (pbistConfig == GPREG2_PBIST_RUN_SYSCLK_95MHZ) ||
				    (pbistConfig == GPREG2_PBIST_RUN_SYSCLK_47_5MHZ))
				{
                    //
                    // If OTP is configured as to NOT use PLL in main sequence,
                    // then PLL Lock would not have been attempted before.
                    // In such case try to lock the PLL here.
                    //
                    if( (BOOTPIN_CONFIG_KEY != ((CPU1BROM_bootConfigureWord & 0xFF000000UL) >> 24U))
                          || (BOOT_CONFIGURE_ENABLE_PLL != (CPU1BROM_bootConfigureWord & 0x3U)) )
					{
						// Setting up for 190MHz
						CPU1BROM_triggerSysPLLLock(APLL_MULT_38, APLL_DIV_2);

                        //
                        // CPU1 Patch/Escape Point 15
                        //
                        if(SW_PATCH_POINT_KEY == swPatchKey)
                        {
                            entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_1;
                            if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
                            {
                                //
                                // If OTP is programmed, then call OTP patch function
                                //
                                EXECUTE_ESCAPE_POINT(entryAddress);
                            }
                        }

					    // Switch SYSCLK to PLL (at max of 95Mhz)
					    (void)CPU1BROM_switchToPLL(CPU1BROM_bootConfigureWord);
					}

                    //
                    // If PLL O/P drives SYSCLK, update the divider to get required frequency.
                    //
					if(SYSCTL_SYSPLLCTL1_PLLCLKEN ==
                                   (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &
                                                   SYSCTL_SYSPLLCTL1_PLLCLKEN))
					{
                        // Switch sysclk to bypass clock, before changing the divider
                        // (as there can be cases of switching from lower to higher frequency).
                        EALLOW;
	                    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= (~SYSCTL_SYSPLLCTL1_PLLCLKEN);
                        EDIS;

					    if(pbistConfig == GPREG2_PBIST_RUN_SYSCLK_95MHZ)
                        {
                            //
                            // Set divider for 95MHz (divide by 2)
                            //
                            EALLOW;
                            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                                ((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                                 ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | SYSCLK_DIV_2);
                            EDIS;
                        }
					    else if(pbistConfig == GPREG2_PBIST_RUN_SYSCLK_47_5MHZ)
					    {
                            //
                            // Set divider for 47.5 MHz (divide by 4)
                            //
                            EALLOW;
                            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                                ((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                                 ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | SYSCLK_DIV_4);
                            EDIS;
					    }
                        else
                        {
                            //
                            // To avoid misra violation
			                //
                        }

                        // Switch sysclk to PLL clock, after changing the divider.
                        EALLOW;
	                    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |= SYSCTL_SYSPLLCTL1_PLLCLKEN;
                        EDIS;
					    pllStatus = BROM_PLL_CONFIG_SUCCESS;

                        //
                        // ~200 PLLSYSCLK delay to allow voltage regulator to stabilize
                        //
                        asm(" MOV    @T,#200 ");
                        asm(" RPT    @T \
                              || NOP ");

					}
				}
	            else if (pbistConfig == GPREG2_PBIST_RUN_PLL_BYPASS) // PLL Bypass
				{
				    // Switch sysclk to bypass clock
                    EALLOW;
	                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= (~SYSCTL_SYSPLLCTL1_PLLCLKEN);
                    EDIS;

					pllStatus = BROM_PLL_CONFIG_SUCCESS;
				}
                else
                {
				    //
                    // To avoid misra violation
					//
                }

                //
                // CPU1 Patch/Escape Point 15
                //
                if(SW_PATCH_POINT_KEY == swPatchKey)
                {
                    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_15;
                    if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
                    {
                        //
                        // If OTP is programmed, then call OTP patch function
                        //
                        EXECUTE_ESCAPE_POINT(entryAddress);
                    }
                }
#ifndef LDRA_FILEIO
                //
                // Skip PBIST and CRC test if configured to use PLL and PLL enable fails
                //
                if((BROM_PLL_CONFIG_SUCCESS == pllStatus))
                {
                    uint32_t goldenValue = HWREG(TI_OTP_GOLDEN_CRC_ADDRESS);

                    proceedToPBIST = ((CPU1BROM_crcCheck(CRC_START_ADDRESS, CRC_END_ADDRESS, CRC_SEED)
                                      == goldenValue)
                                      ? true : false);
                }
                if(proceedToPBIST)
                {
                         //
                        // PBIST Memory Test
                        //
                        CPU1BROM_pbistStatus = PBIST_PORMemoryTest();

                        //
                        // PBIST function return here using RPC and reinitialize RPC
                        //
                        asm(" .ref ExitPBISTLoc");
                        asm(" PUSH XAR7");
                        asm(" MOVL XAR7, #ExitPBISTLoc");
                        asm(" PUSH XAR7");
                        asm(" POP RPC");
                        asm(" POP XAR7");

                        //
                        // Reinitialize Variables lost during PBIST RAMINIT
                        //
                        CPU1BROM_bootMode = 0xFFFFFFFFUL;
                        CPU1BROM_itrapAddress = 0xFFFFFFFFUL;

                        //
                        // Set boot status
                        //
                        CPU1BROM_bootStatus = (uint32_t)(CPU1_BOOTROM_BOOTSTS_SYSTEM_START_BOOT |
                                               CPU1_BOOTROM_DCSM_INIT_COMPLETE |
                                               CPU1_BOOTROM_RAM_INIT_COMPLETE |
                                               CPU1_BOOTROM_POR_MEM_TEST_COMPLETE);

                        //
                        // Re-init the single bit error address status.
                        //
                        CPU1BROM_corrErrorAddr  = HWREG(MEMCFG_BASE + MEMCFG_O_CCPUREADDR);

                        //
                        //Capture the BGCRC pass in the variable
                        //
                        CPU1BROM_pbistStatus = CPU1BROM_pbistStatus | CRC_TEST_PASS;

                        //
                        // Restore the boot config word after PBIST execution
                        //
                        CPU1BROM_bootConfigureWord = regDoubleRead_32(OTP_BOOT_CONFIGURE_WORD_ADDRESS, true);
                }
                else
                {
                    CPU1BROM_pbistStatus = CPU1BROM_pbistStatus & ~(CRC_TEST_PASS);
                }
#endif// LDRA_FILEIO
			}
		}
    }

    //
    // If PLL was configured to be enabled, check DCC status to
    // know if PLL was locked successfully.
    //
    if( (BOOTPIN_CONFIG_KEY == ((CPU1BROM_bootConfigureWord & 0xFF000000UL) >> 24U))
          && (BOOT_CONFIGURE_ENABLE_PLL == (CPU1BROM_bootConfigureWord & 0x3U)) )
    {
        if((HWREGH(DCC0_BASE + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == DCC_STATUS_DONE)
        {
            CPU1BROM_bootStatus |= BOOTROM_PLL_ENABLE_SUCCESS;
        }
    }

    //
    // CPU1 Patch/Escape Point 3
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_3;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Override unbonded GPIO configuration with value from OTP.
    //
    if(TI_OTP_KEY == HWREAD_TI_OTP_GPXPUD_KEY)
    {
        EALLOW;
        HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= HWREAD_TI_OTP_GPA_PUD_CONFIG;
        HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= HWREAD_TI_OTP_GPB_PUD_CONFIG;
        EDIS;
    }

    else
    {
        if(((HWREAD_TI_OTP_PKG_TYPE & 0xFF00U) >> 8U) == PKG_TYPE_KEY)
        {
            //
            // Enable unbonded GPIO pullups
            //
            CPU1BROM_enableUnbondedGpioPullups(HWREAD_TI_OTP_PKG_TYPE & 0x0000FU);
        }
    }

    //
    // CPU1 Patch/Escape Point 7
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_7;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Device Calibration (NonCritical Trims). Calibrate ADC(INL, Offset), GPDAC(Offset)
    //
    CPU1BROM_devcalInit();

    //
    // Disable the watchdog in all cases
    //
    SysCtl_disableWatchdog();
    /*reset the dividers to default value*/
    SysCtl_setWatchdogPrescaler(WD_DEF_PRESCALE_VALUE);
    SysCtl_setWatchdogPredivider(WD_DEF_PREDIV_VALUE);

#ifndef LDRA_FILEIO
    DEBUGLABEL(WatchdogDisableTest);
#endif

    //
    // POR Only
    //
    // Check LS RAMs to verify they have completed initialization
    // Wait for RAM init to complete, if not, timeout
    // (4208 cycles for LS to complete with an estimated 16 cycles per loop)
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)) ==
       SYSCTL_RESC_POR)
    {
        uint32_t ramInitTimeout = 4208UL / 16UL;
        while(HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE) == RAM_LSX_NOT_DONE)
        {
            if(ramInitTimeout == 0UL)
            {
                CPU1BROM_bootStatus |= CPU1_BOOTROM_RAM_INIT_ERROR;
                break;
            }
            ramInitTimeout--;
        }
        if(ramInitTimeout != 0UL)
        {
            //
            // Update boot status - RAM Init complete
            //
            CPU1BROM_bootStatus |= CPU1_BOOTROM_RAM_INIT_COMPLETE;
        }
    }

    //
    // Bypass PLL (if enabled)
    //
    if(0UL != (HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
              ((uint32_t)SYSCTL_RESC_POR | (uint32_t)SYSCTL_RESC_XRSN )))
    {
        EALLOW;

        // bypass PLL here
        if((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) & SYSCTL_SYSPLLCTL1_PLLCLKEN) != 0UL)
        {
            HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLCLKEN;
            //
            // Delay 25 cycles
            //
            asm(" MOV    @T,#25 ");
            asm(" RPT    @T \
                  || NOP ");
        }

        //
        // Set PLL multiplier to 0x0
        //
        HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) = 0;

        //
        // Set the divider to /1
        //
        HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 0;

        //
        // Turn off PLL and delay for power down
        //
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLEN;

        //
        // Delay 25 cycles
        //
        asm(" MOV    @T,#25 ");
        asm(" RPT    @T \
              || NOP ");

        EDIS;
    }
	
    //
    // On POR
    //  - Handle POR/XRSn RESC bits so that bootROM would not go into
    //    a unwanted reset handling if it got reset for some reason
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)) ==
       SYSCTL_RESC_POR)
    {
        //
        // Clear POR and XRSn
        //
        SysCtl_clearResetCause(SYSCTL_RESC_POR | SYSCTL_RESC_XRSN);
        CPU1BROM_bootStatus |= (uint32_t)((uint32_t)CPU1_BOOTROM_HANDLED_XRSN |
                                          (uint32_t)CPU1_BOOTROM_HANDLED_POR);

    }

    //
    // On XRS - Clear RESC bits so that bootROM would not go into
    // a unwanted reset handling if it got reset for some reason
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_XRSN)) ==
       SYSCTL_RESC_XRSN)
    {
        SysCtl_clearResetCause(SYSCTL_RESC_XRSN);
        CPU1BROM_bootStatus |= (uint32_t)(CPU1_BOOTROM_HANDLED_XRSN);
    }

    //
    // Update boot status - Reset cause clearing complete
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_RESC_HANDLED;

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

    //
    // Get boot mode selected
    //
    CPU1BROM_bootMode = CPU1BROM_selectBootMode();

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

#ifdef LDRA_FILEIO
    CPU1BROM_bootMode = WAIT_BOOT;
#endif
    //
    // Update boot status - Clear booting status field before setting boot mode status
    //
    CPU1BROM_bootStatus &= ~CPU1_BOOTROM_BOOTSTS_BOOT_MASK;
    //
    // Run selected boot mode
    //
    switch(CPU1BROM_bootMode)
    {
        case FLASH_BOOT:
        case FLASH_BOOT_ALT1:
        case FLASH_BOOT_ALT2:
        case FLASH_BOOT_ALT3:
        case FLASH_BOOT_ALT4:
        case FLASH_BOOT_ALT5:
            appEntryAddress = getFlashEntryPointAddress(CPU1BROM_bootMode);
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            break;

        case SECURE_FLASH_BOOT:
        case SECURE_FLASH_BOOT_ALT1:
        case SECURE_FLASH_BOOT_ALT2:
        case SECURE_FLASH_BOOT_ALT3:
        case SECURE_FLASH_BOOT_ALT4:
            appEntryAddress = getFlashEntryPointAddress(CPU1BROM_bootMode);
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_SECURE_FLASH_BOOT;
            CPU1BROM_verifySecureFlash(appEntryAddress);
            break;

        case RAM_BOOT:
            appEntryAddress = RAM_ENTRY_POINT;
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_RAM_BOOT;
            break;

        case WAIT_BOOT:
        case WAIT_BOOT_ALT1:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_WAIT_BOOT;
            break;

        case PARALLEL_BOOT:
        case PARALLEL_BOOT_ALT1:
        case PARALLEL_BOOT_ALT2:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_PARALLEL_BOOT;
            appEntryAddress = Parallel_Boot(CPU1BROM_bootMode);
            break;

        case SCI_BOOT:
        case SCI_BOOT_ALT1:
        case SCI_BOOT_ALT2:
        case SCI_BOOT_ALT3:
        case SCI_BOOT_ALT4:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_SCI_BOOT;
            appEntryAddress = SCI_Boot(CPU1BROM_bootMode);
            break;

        case CAN_BOOT:
        case CAN_BOOT_ALT1:
        case CAN_BOOT_ALT2:
        case CAN_BOOT_ALT3:
        case CAN_BOOT_SENDTEST:
        case CAN_BOOT_ALT1_SENDTEST:
        case CAN_BOOT_ALT2_SENDTEST:
        case CAN_BOOT_ALT3_SENDTEST:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_CAN_BOOT;
            appEntryAddress = DCAN_Boot(CPU1BROM_bootMode,
                                     CAN_BOOT_DEFAULT_BIT_TIMING,
                                     CAN_BOOT_USE_XTAL);
            break;

        case SPI_MASTER_BOOT:
        case SPI_MASTER_BOOT_ALT1:
        case SPI_MASTER_BOOT_ALT2:
        case SPI_MASTER_BOOT_ALT3:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_SPI_BOOT;
            appEntryAddress = SPI_Boot(CPU1BROM_bootMode);
            break;

        case I2C_MASTER_BOOT:
        case I2C_MASTER_BOOT_ALT1:
        case I2C_MASTER_BOOT_ALT2:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_I2C_BOOT;
            appEntryAddress = I2C_Boot(CPU1BROM_bootMode);
            break;

        default:
            //
            // Check if debugger is connected
            //
            if(((uint32_t)HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
                (uint32_t)SYSCTL_RESC_DCON) == SYSCTL_RESC_DCON)
            {
                //
                // CPU1 Patch/Escape Point 5
                //
                if(SW_PATCH_POINT_KEY == swPatchKey)
                {
                    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_5;
                    if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
                    {
                        //
                        // If OTP is programmed, then call OTP patch function
                        //
                        EXECUTE_ESCAPE_POINT(entryAddress);
                    }
                }
                CPU1BROM_bootMode = WAIT_BOOT;
                CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_WAIT_BOOT;
            }
            else
            {
                //
                // CPU1 Patch/Escape Point 5
                //
                if(SW_PATCH_POINT_KEY == swPatchKey)
                {
                    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_5;
                    if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
                    {
                        //
                        // If OTP is programmed, then call OTP patch function
                        //
                        EXECUTE_ESCAPE_POINT(entryAddress);
                    }
                }

                CPU1BROM_bootMode = FLASH_BOOT;
                appEntryAddress = FLASH_ENTRY_POINT;
                CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            }
            break;
    }
    //
    // Update boot status - Boot Complete
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOT_COMPLETE;

    //
    // Revert the flash read wait-state to default values
    //
    EALLOW;
    HWREG(FLASH0CTRL_BASE + FLASH_O_FRDCNTL) = ((HWREG(FLASH0CTRL_BASE + FLASH_O_FRDCNTL) &
             ~(uint32_t)FLASH_FRDCNTL_RWAIT_M  &
             ~(uint32_t)FLASH_FRDCNTL_TRIMENGRRWAIT_M) |
              (((uint32_t)CPU1_FLASH_DEFAULT_RWAIT << FLASH_FRDCNTL_RWAIT_S) |
              ((uint32_t)CPU1_FLASH_DEFAULT_TRIMENGRRWAIT << FLASH_FRDCNTL_TRIMENGRRWAIT_S)));
    EDIS;

    //
    // Enter specified wait boot or enable watchdog, then branch to address
    //
    if(CPU1BROM_bootMode == WAIT_BOOT)
    {
#ifndef LDRA_FILEIO
        SysCtl_enableWatchdog();
        asm("   ESTOP0");
        for(;;)
        {
        }
#endif// LDRA_FILEIO
#ifdef LDRA_FILEIO
        SysCtl_disableWatchdog();
        appEntryAddress = (uint32_t)main;
        return(appEntryAddress);
#endif// LDRA_FILEIO
    }
    else if(CPU1BROM_bootMode == WAIT_BOOT_ALT1)
    {
        asm("   ESTOP0");
        for(;;)
        {
        }
    }
    else
    {
#ifdef LDRA_FILEIO
        //
        //after upload of the execution history
        //
        upload_execution_history();
#endif

        SysCtl_enableWatchdog();

        return(appEntryAddress);
    }
}

/**
 * @brief API for verifying secure flash against golden signature.
 *
 * This function runs the secure flash verification procedure.
 * A CMAC calculation is run on  the 16KB of  flash, starting at the
 * entry address provided. If the CMAC calculation passes, this function
 * returns and the boot process continues to branch to entry point.
 * If the CMAC calculation fails, this  function either halts debugger
 * or enables watchdog to reset device.
 *
 * @param entryAddress entry address starting from which 16kb of flash has to
 *                     be verified.
 *
 * @return none
 *
 * Design: \ref did_secure_flash_boot_verify_flash did_secure_lfu_boot_algo
 *              did_secureflash_boot_interface did_secureflash_boot_algo
 *
 * Requirement: REQ_TAG(C2000BROM-201) REQ_TAG(C2000BROM-364), REQ_TAG(C2000BROM-201),
 */
void CPU1BROM_verifySecureFlash(uint32_t appEntryAddress)
{
    uint16_t entryAddress = 0xFFFFU;

    //
    // CPU1 Patch/Escape Point 9
    //
    if(SW_PATCH_POINT_KEY == swPatchKey)
    {
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_9;
        if((entryAddress < 0xFFFFU) && (entryAddress > 0x0000U))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            EXECUTE_ESCAPE_POINT(entryAddress);
        }
    }

    //
    // Calculate CMAC
    // - Calculation starts at appEntryAddress
    // - End address of calculation is appEntryAddress + 16KB (8192 words)
    //   (CMAC is calculated from start address to (end address - 1)
    // - Golden CMAC signature is always stored at appEntryAddress + 2 words
    //   (Signature will be considered all 1s in calculation)
    //
    if(CPU1BROM_calculateCMAC(appEntryAddress, (appEntryAddress + 0x2000U),
                              (appEntryAddress + 0x2U)) != CMAC_ERROR_NONE)
    {
        CPU1BROM_bootStatus |= CPU1_BOOTROM_FLASH_VERIFICATION_ERROR;
        //
        // CMAC failed, halt if using debugger, else reset device
        //
        asm("   ESTOP0");

        SysCtl_enableWatchdog();
        for(;;)
        {
        }
    }

    //
    // CMAC passed, return and boot to flash
    //
}

