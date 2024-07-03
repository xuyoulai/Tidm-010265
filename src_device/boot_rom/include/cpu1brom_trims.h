//###########################################################################
//
// FILE:   cpu1brom_trims.h
//
// TITLE:  BootROM Definitions related to trimming
//
//###########################################################################
// $TI Release:  $
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



#ifndef C_BOOTROM_TRIMS_H_
#define C_BOOTROM_TRIMS_H_

#include <stdint.h>

//---------------------------------------------------------------------------
// PMM Trims
//---------------------------------------------------------------------------

//
// PMM Trim - Analog Subsystem Register Offsets
//
// Refer to hw_asysctl.h

//
// Trim offset from key
//
#define TI_OTP_PMM_TRIM_OFFSET 0x2U

//
// PMM FT OTP Key Addresses
//
#define TI_OTP_PMM_REF_FT_TRIM_KEY_ADDRESS   0x7118CUL
#define TI_OTP_PMM_VMON_FT_TRIM_KEY_ADDRESS  0x71190UL
#define TI_OTP_PMM_VREG_FT_TRIM_KEY_ADDRESS  0x71196UL

//
// PMM MP3 OTP Key Addresses
//
#define TI_OTP_PMM_REF_MP3_TRIM_KEY_ADDRESS  0x710F8UL
#define TI_OTP_PMM_VMON_MP3_TRIM_KEY_ADDRESS 0x710FCUL
#define TI_OTP_PMM_VREG_MP3_TRIM_KEY_ADDRESS 0x71102UL

//
// PMM MP1 OTP Key Addresses
//
#define TI_OTP_PMM_REF_MP1_TRIM_KEY_ADDRESS  0x71020UL
#define TI_OTP_PMM_VMON_MP1_TRIM_KEY_ADDRESS 0x71024UL
#define TI_OTP_PMM_VREG_MP1_TRIM_KEY_ADDRESS 0x7102AUL

//
// Masks
//
#define PMM_VREG_MASK                        0x00FFU
#define PMM_REF_MASK                         0x0001FFFFUL
#define PMM_VMON_MASK                        0xFF0FU

////bits15:8 is the KEY ; if Value == 0x5A then the remaining bits are valid
////bits 7:2 => reserved
////bits 0:1 if set to b'00 BROM will program 0x01 in VREGCTL.ENMASK
////          - any other value the VREGCTL.ENMASK will be left at reset state.
//
//#define TI_OTP_REG_VREGCTL_ENMASK_VAL   ((TI_OTP_REG_VREGCTL_ENMASK) & 0x03U)
//
//#define TI_OTP_REG_VREGCTL_ENMASK_KEY   (((TI_OTP_REG_VREGCTL_ENMASK) & 0xFF00U) >> 0x8U)
//
//#define TI_OTP_SECDC                    0x703F0

//---------------------------------------------------------------------------
// INTOSC Trims
//---------------------------------------------------------------------------

//
// Trim offset from key
//
#define TI_OTP_INTOSC_TRIM1_OFFSET 0x2U
#define TI_OTP_INTOSC_TRIM2_OFFSET 0x4U
#define TI_OTP_EXTROSC_TRIM_OFFSET 0x2U

//
// INTOSC1/2 OTP Key Addresses
//
#define TI_OTP_INTOSC_FT_TRIM_KEY_ADDRESS    0x7115EUL
#define TI_OTP_INTOSC_MP3_TRIM_KEY_ADDRESS   0x71106UL
#define TI_OTP_INTOSC_MP1_TRIM_KEY_ADDRESS   0x7102EUL

//
// INTOSC SR
//
#define TI_OTP_INTOSC_SR_KEY_ADDRESS         0x71052UL
#define TI_OTP_INTOSC_SR2_CONFIG_ADDRESS     0x71054UL
#define TI_OTP_INTOSC_SR3_CONFIG_ADDRESS     0x71055UL
#define INTOSC_SR_MASK                       0xFFFFU

//
// EXTROSC OTP Key Addresses
//
#define TI_OTP_EXTROSC_FT_TRIM_KEY_ADDRESS   0x71164UL
#define TI_OTP_EXTROSC_MP3_TRIM_KEY_ADDRESS  0x7110CUL
#define TI_OTP_EXTROSC_MP1_TRIM_KEY_ADDRESS  0x71034UL

//
// Bits 23-16 and 11-0 of INTOSC1TRIM/INTOSC2TRIM/EXTROSC are reserved for trims
//
#define OSC_TRIM_MASK                        0x00FF0FFFUL

//---------------------------------------------------------------------------
// APLL Trims
//---------------------------------------------------------------------------

//
// APLL Trim - Analog Subsystem Register Offsets
//
// Refer to hw_asysctl.h

//
// Trim offset from key
//
#define TI_OTP_APLL_LDO_TRIM_OFFSET 0x2U
#define TI_OTP_ADCREF_TRIM_OFFSET   0x2U

//
// APLL LDO OTP Key Addresses
// (No FT, only MP3 and MP1)
//
#define TI_OTP_APLLLDO_FT_TRIM_KEY_ADDRESS    0x0UL
#define TI_OTP_APLLLDO_MP3_TRIM_KEY_ADDRESS   0x7111CUL
#define TI_OTP_APLLLDO_MP1_TRIM_KEY_ADDRESS   0x7104EUL

//
// ADC REF OTP Key Addresses
// (FT, MP3 and MP1)
//
#define TI_OTP_ADCREF_FT_TRIM_KEY_ADDRESS     0x71188UL
#define TI_OTP_ADCREF_MP3_TRIM_KEY_ADDRESS    0x71110UL
#define TI_OTP_ADCREF_MP1_TRIM_KEY_ADDRESS    0x71038UL

//
// ADC OFF OTP Key Addresses
// (No FT, No MP3, Only MP1)
//
#define TI_OTP_ADCOFF_FT_TRIM_KEY_ADDRESS     0x0UL
#define TI_OTP_ADCOFF_MP3_TRIM_KEY_ADDRESS    0x71114UL
#define TI_OTP_ADCOFF_MP1_TRIM_KEY_ADDRESS    0x7103CUL

#define APLL_DOT_TRIM_MASK                    0x00FFU

#define TI_OTP_ADCA_OFFTRIM_OFFSET            0x2U
#define TI_OTP_ADCC_OFFTRIM_OFFSET            0x3U

//
// ADC INL OTP Key Addresses
// (No FT, No MP3, Only MP1)
//
#define TI_OTP_ADCINL_FT_TRIM_KEY_ADDRESS     0x0UL
#define TI_OTP_ADCINL_MP3_TRIM_KEY_ADDRESS    0x0UL
#define TI_OTP_ADCINL_MP1_TRIM_KEY_ADDRESS    0x71040UL

#define TI_OTP_ADCA_INLTRIM2_OFFSET           0x2U
#define TI_OTP_ADCA_INLTRIM3_OFFSET           0x4U
#define TI_OTP_ADCC_INLTRIM2_OFFSET           0x6U
#define TI_OTP_ADCC_INLTRIM3_OFFSET           0x8U



//
// APLL Lock - Analog Subsystem Register Offset
//
// Refer to hw_asysctl.h
#define APLL_CONFIG_LOCK                0x40009FFFUL

//---------------------------------------------------------------------------
// ADC Trims
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// DAC Trims
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// Flash Trims
//---------------------------------------------------------------------------

#define FLASH_PUMPTRIM0_MASK                           0x03FF03FFUL

#define FLASH_PUMPTRIM1_MASK                           0x07FF7FFFUL

#define FLASH_PUMPTRIM2_MASK                           0x0FFFFFFFUL

#define FLASH_BANK0TRIM0_MASK                          0xFFFFFFFFUL

#define FLASH_BANK0TRIMREAD_MASK                       0x0FFFFFFFUL

#define FLASH_PUMPTRIMREAD_MASK                        0x00003FFFUL
//
// NW Flash Base Address
//
#define BROM_FLASHNW_BASE                              (FLASH0CMD_BASE - 0x1000UL)

//
// Trim offset from key
//
#define TI_OTP_NWFLASH_B0_TRIM0_OFFSET       0x2U
#define TI_OTP_NWFLASH_B0_TRIM_READ_OFFSET   0x2U
#define TI_OTP_NWFLASH_PUMP_TRIM0_OFFSET     0x2U
#define TI_OTP_NWFLASH_PUMP_TRIM1_OFFSET     0x4U
#define TI_OTP_NWFLASH_PUMP_TRIM2_OFFSET     0x6U
#define TI_OTP_NWFLASH_PUMP_TRIM_READ_OFFSET 0x2U

//
// NW Flash Bank0 Trim 0 OTP Key Addresses
//
#define TI_OTP_NWFLASH_B0_TRIM0_FT_TRIM_KEY_ADDRESS    0x71180UL
#define TI_OTP_NWFLASH_B0_TRIM0_MP3_TRIM_KEY_ADDRESS   0x710F0UL
#define TI_OTP_NWFLASH_B0_TRIM0_MP1_TRIM_KEY_ADDRESS   0x71018UL

//
// NW Flash PUMP Trim Read OTP Key Addresses
//
#define TI_OTP_NWFLASH_PUMP_READ_FT_TRIM_KEY_ADDRESS    0x7117CUL
#define TI_OTP_NWFLASH_PUMP_READ_MP3_TRIM_KEY_ADDRESS   0x710ECUL
#define TI_OTP_NWFLASH_PUMP_READ_MP1_TRIM_KEY_ADDRESS   0x71014UL

//
// NW Flash Bank0 Trim READ (Repair) OTP Key Addresses
//
#define TI_OTP_NWFLASH_B0_TRIM_READ_FT_TRIM_KEY_ADDRESS    0x71170UL
#define TI_OTP_NWFLASH_B0_TRIM_READ_MP3_TRIM_KEY_ADDRESS   0x710E0UL
#define TI_OTP_NWFLASH_B0_TRIM_READ_MP1_TRIM_KEY_ADDRESS   0x71008UL

//
// NW Flash Pump Trim OTP Key Addresses
//
#define TI_OTP_NWFLASH_PUMP_TRIM_FT_TRIM_KEY_ADDRESS   0x71174UL
#define TI_OTP_NWFLASH_PUMP_TRIM_MP3_TRIM_KEY_ADDRESS  0x710E4UL
#define TI_OTP_NWFLASH_PUMP_TRIM_MP1_TRIM_KEY_ADDRESS  0x7100CUL

//---------------------------------------------------------------------------
// Trim error checking and logging
//---------------------------------------------------------------------------

#define NUM_OF_TEST_INSERTIONS          3U
#define SKIP_TEST_INSERTION             0UL
#define FT_TRIM_KEY_INDEX               0U
#define MP3_TRIM_KEY_INDEX              1U
#define MP1_TRIM_KEY_INDEX              2U
#define TRIM_32BIT_KEY                  0x5A5A5A5AUL
#define NO_TRIM_ERROR                   (bool)true

#define PMM_REF_TRIM_LOADED             0x1UL
#define PMM_VMON_TRIM_LOADED            0x2UL
#define PMM_VREG_TRIM_LOADED            0x4UL

#define INT_OSC_TRIM_LOADED             0x1UL
#define EXT_R_OSC_TRIM_LOADED           0x2UL

#define APLL_LDO_TRIM_LOADED            0x1UL

#define FLASH_PUMP_TRIM_LOADED          0x1UL
#define FLASH_BANK0_TRIM0_LOADED        0x2UL
#define FLASH_BANK0_TRIM_READ_LOADED    0x4UL
#define FLASH_PUMP_TRIM_READ_LOADED     0x8UL

//
// Check SED,DED ECC Error Status
//
#define HWREAD_SEC_STATUS              (HWREGH(MEMORYERROR_BASE + MEMCFG_O_CERRFLG) & MEMCFG_CERRFLG_CPURDERR)
#define HWREAD_DED_STATUS              (HWREGH(MEMORYERROR_BASE + MEMCFG_O_UCERRFLG) & MEMCFG_UCERRFLG_CPURDERR)

//
// SEC, DED ECC Trim Error Status
//
#define TRIM_ERROR_SEC                  0x1U
#define TRIM_ERROR_DED                  0x2U
#define HWREAD_TRIM_ERROR_STATUS        (HWREGH(DEVCFG_BASE + SYSCTL_O_TRIMERRSTS))
#define LOG_TRIMERROR_SEC()             (HWREGH(DEVCFG_BASE + SYSCTL_O_TRIMERRSTS) |= TRIM_ERROR_SEC)
#define LOG_TRIMERROR_DED()             (HWREGH(DEVCFG_BASE + SYSCTL_O_TRIMERRSTS) |= TRIM_ERROR_DED)

#define TRIMLOCKREAD                    0x1UL
#define TRIMLOCKOTHER                   0x2UL

#define TRIMCOMMITREAD                  0x1UL
#define TRIMCOMMITOTHER                 0x0UL

#define BANK2TRDY_M                     0x00010000UL
#define BANK1TRDY_M                     0x00020000UL

#endif //C_BOOTROM_TRIMS_H_
