//###########################################################################
//
// FILE:    cpu1brom_dcsm.c
//
// TITLE:   DCSM Initialization
//
//###########################################################################
// $TI Release:  $
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
#include "cpubrom_dcsm.h"
#include "dcsm_otp_offsets.h"
#include "cpu1brom_utils.h"

const uint32_t dcsm_default_keys_z1[] = {
        0x4d7fffffUL,         //Z1-ZSB0-CSMPSWD1
        0x5f7fffffUL,         //Z1-ZSB1-CSMPSWD1
        0x1dffffffUL,         //Z1-ZSB2-CSMPSWD1
        0xaf7fffffUL,         //Z1-ZSB3-CSMPSWD1
        0x1bffffffUL,         //Z1-ZSB4-CSMPSWD1
        0x17ffffffUL,         //Z1-ZSB5-CSMPSWD1
        0xbd7fffffUL,         //Z1-ZSB6-CSMPSWD1
        0x9f7fffffUL,         //Z1-ZSB7-CSMPSWD1
        0x2bffffffUL,         //Z1-ZSB8-CSMPSWD1
        0x27ffffffUL,         //Z1-ZSB9-CSMPSWD1
        0x7b7fffffUL,         //Z1-ZSB10-CSMPSWD1
        0xc9ffffffUL,         //Z1-ZSB11-CSMPSWD1
        0x7d7fffffUL,         //Z1-ZSB12-CSMPSWD1
        0x6f7fffffUL,         //Z1-ZSB13-CSMPSWD1
        0x33ffffffUL          //Z1-ZSB14-CSMPSWD1
};

const uint32_t dcsm_default_keys_z2[] = {
        0x1f7fffffUL,         //Z2-ZSB0-CSMPSWD1
        0xe57fffffUL,         //Z2-ZSB1-CSMPSWD1
        0x4fffffffUL,         //Z2-ZSB2-CSMPSWD1
        0xe37fffffUL,         //Z2-ZSB3-CSMPSWD1
        0x57ffffffUL,         //Z2-ZSB4-CSMPSWD1
        0x5bffffffUL,         //Z2-ZSB5-CSMPSWD1
        0xf17fffffUL,         //Z2-ZSB6-CSMPSWD1
        0x3b7fffffUL,         //Z2-ZSB7-CSMPSWD1
        0x8fffffffUL,         //Z2-ZSB8-CSMPSWD1
        0x6bffffffUL,         //Z2-ZSB9-CSMPSWD1
        0x377fffffUL,         //Z2-ZSB10-CSMPSWD1
        0x9bffffffUL,         //Z2-ZSB11-CSMPSWD1
        0x2f7fffffUL,         //Z2-ZSB12-CSMPSWD1
        0xcb7fffffUL,         //Z2-ZSB13-CSMPSWD1
        0x97ffffffUL          //Z2-ZSB14-CSMPSWD1
};


//
// Function Prototypes
//
static uint32_t Gather_Z1_ZSB(uint32_t *z1_key1);
static uint32_t Gather_Z2_ZSB(uint32_t *z2_key1);

/**
#################################################
 void CPU1BROM_initDCSM(void)
--------------------------------------------
 This function initializes code security module, until this function is executed
 all access to RAM and JTAG is blocked.
--------------------------------------------
*
* \brief DCSM initialization function
*
* @return DCSM_InitStatus
*
* Design: \ref did_dcsm_init_usecase did_dcsm_init_interface did_dcsm_init_algo
*              did_dcsm_init_load_dcsm_reg_algo did_dcsm_init_link_pointer_algo
*              did_dcsm_init_dummy_read_csm_passwords_algo did_dcsm_init_load_default_csm_keys_algo
* Requirement: REQ_TAG(C2000BROM-154),  REQ_TAG(C2000BROM-155),  REQ_TAG(C2000BROM-157),
*              REQ_TAG(C2000BROM-158)
*
* DCSM initialization function
*
*/
DCSM_InitStatus CPU1BROM_initDCSM(void)
{
    DCSM_InitStatus initStatus = DCSM_ERROR_NONE;
    uint32_t Z1_ZSB  = 0U;
    uint32_t Z2_ZSB  = 0U;
    uint32_t z1_key1 = 0U;
    uint32_t z2_key1 = 0U;

    EALLOW;

    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_LINKPOINTER1); //Zone 1 Contents
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_LINKPOINTER2);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_LINKPOINTER3);

    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_LINKPOINTER1); //Zone 2 Contents
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_LINKPOINTER2);
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_LINKPOINTER3);

    HWREG(TIOTP1_SECDC); // TI OTP SECDC register read

    //
    // OTPSECLOCK and other boot related register reads from
    // Zone 1 and Zone 2 of USER OTP
    //
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_JLM_ENABLE);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_PSWDLOCK);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_PSWDLOCK + 0x2U);// DCSM_O_Z1OTP_CRCLOCK); //No CRC
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_GPREG1);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_GPREG2);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_GPREG3);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_GPREG4);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_JTAGPSWDH0);
    HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_JTAGPSWDH1);

    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_PSWDLOCK);
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_PSWDLOCK +0x2U); //DCSM_O_Z2OTP_CRCLOCK);
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_GPREG1);
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_GPREG2);
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_GPREG3);
    HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_GPREG4);

    Z1_ZSB = Gather_Z1_ZSB(&z1_key1); //Gather ZSB of ZONE1 from BANK0
    Z2_ZSB = Gather_Z2_ZSB(&z2_key1); //Gather ZSB of ZONE2 from BANK0

    //
    // Zone 1 ZSB
    //
    HWREG(Z1_ZSB + DCSM_O_Zx_GRABSECT1);// Zone Select Block contents
    HWREG(Z1_ZSB + DCSM_O_Zx_GRABSECT2);
    HWREG(Z1_ZSB + DCSM_O_Zx_GRABSECT3);
    HWREG(Z1_ZSB + DCSM_O_Zx_GRABRAM1);
    HWREG(Z1_ZSB + DCSM_O_Zx_GRABRAM2);
    HWREG(Z1_ZSB + DCSM_O_Zx_GRABRAM3);
    HWREG(Z1_ZSB + DCSM_O_Zx_EXEONLYSECT1);
    HWREG(Z1_ZSB + DCSM_O_Zx_EXEONLYSECT2);
    HWREG(Z1_ZSB + DCSM_O_Zx_EXEONLYRAM1);
    HWREG(Z1_ZSB + DCSM_O_Z1_JTAGPSWDL0);
    HWREG(Z1_ZSB + DCSM_O_Z1_JTAGPSWDL1);

    //
    // Zone 2 ZSB
    //
    HWREG(Z2_ZSB + DCSM_O_Zx_GRABSECT1);// Zone Select Block contents
    HWREG(Z2_ZSB + DCSM_O_Zx_GRABSECT2);
    HWREG(Z2_ZSB + DCSM_O_Zx_GRABSECT3);
    HWREG(Z2_ZSB + DCSM_O_Zx_GRABRAM1);
    HWREG(Z2_ZSB + DCSM_O_Zx_GRABRAM2);
    HWREG(Z2_ZSB + DCSM_O_Zx_GRABRAM3);
    HWREG(Z2_ZSB + DCSM_O_Zx_EXEONLYSECT1);
    HWREG(Z2_ZSB + DCSM_O_Zx_EXEONLYSECT2);
    HWREG(Z2_ZSB + DCSM_O_Zx_EXEONLYRAM1);

#ifndef LDRA_FILEIO
    DEBUGLABEL(DCSMRegisterDummyLoadTest);
#endif
    //
    // BLOCKED State --> LOCKED State
    //
    HWREG(Z1_ZSB + DCSM_O_Zx_CSMPSWD0);
    HWREG(Z1_ZSB + DCSM_O_Zx_CSMPSWD1);
    HWREG(Z1_ZSB + DCSM_O_Zx_CSMPSWD2);
    HWREG(Z1_ZSB + DCSM_O_Zx_CSMPSWD3);

    HWREG(Z2_ZSB + DCSM_O_Zx_CSMPSWD0);
    HWREG(Z2_ZSB + DCSM_O_Zx_CSMPSWD1);
    HWREG(Z2_ZSB + DCSM_O_Zx_CSMPSWD2);
    HWREG(Z2_ZSB + DCSM_O_Zx_CSMPSWD3);

#ifndef LDRA_FILEIO
    DEBUGLABEL(DCSMArmedState);
#endif

    if ((HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CR) & DCSM_Z1_CR_UNSECURE) == 0U)
    {
        //LOCKED State --> ARMED State
        HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CSMKEY0) = 0xFFFFFFFFUL; //Zone 1 CSMKEY Loads
        HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CSMKEY1) = (uint32_t) z1_key1;
        HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CSMKEY2) = 0xFFFFFFFFUL;
        HWREG(DCSM_Z1_BASE + DCSM_O_Z1_CSMKEY3) = 0xFFFFFFFFUL;
    }

    if ((HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CR) & DCSM_Z2_CR_UNSECURE) == 0U)
    {
        HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CSMKEY0) = 0xFFFFFFFFUL; //Zone 2 CSMKEY Loads
        HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CSMKEY1) = (uint32_t) z2_key1;
        HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CSMKEY2) = 0xFFFFFFFFUL;
        HWREG(DCSM_Z2_BASE + DCSM_O_Z2_CSMKEY3) = 0xFFFFFFFFUL;
    }

#ifndef LDRA_FILEIO
    DEBUGLABEL(DCSMUnlockedState);
#endif

    /*if any of the linkpointer error is set then update the return status,
     This is not a failure and is just an indication indicating that one of the
     linkpointer in OTP is diferring*/
    if ((HWREG(DCSM_Z1_BASE + DCSM_O_Z1_LINKPOINTERERR) != 0U)
            || (HWREG(DCSM_Z2_BASE + DCSM_O_Z2_LINKPOINTERERR) != 0U))
    {
        initStatus = DCSM_ERROR_LINKPOINTERERR_SET;
    }

    EDIS;

    return initStatus;
}

//
// Gather_Z1_ZSB
//
static uint32_t Gather_Z1_ZSB(uint32_t *z1_key1)
{
    uint32_t linkPointer;
    uint32_t ZSBBase; // base address of the ZSB
    int32_t bitPos = 13;
    int32_t zeroFound = 0;

    linkPointer = HWREG(DCSM_Z1_BASE + DCSM_O_Z1_LINKPOINTER);
    linkPointer = linkPointer << 18; // Bits 31 - 14 as most-significant 0 are
                                     //invalid LinkPointer options

    while((zeroFound == 0) && (bitPos > -1))
    {
        if((linkPointer & 0x80000000U) == 0U)
        {
            zeroFound = 1;
            ZSBBase = (DCSMBANK0_Z1OTP_BASE + (((uint32_t)bitPos + 2U) * 0x20U));
        }
        else
        {
            bitPos--;
            linkPointer = linkPointer << 1;
        }
    }

    if(zeroFound == 0)
    {
        ZSBBase = (DCSMBANK0_Z1OTP_BASE + 0X20U);
    }

    *z1_key1 = dcsm_default_keys_z1[bitPos+1];

    return ZSBBase;
}

//
// Gather_Z2_ZSB
//
static uint32_t Gather_Z2_ZSB(uint32_t *z2_key1)
{
    uint32_t linkPointer;
    uint32_t ZSBBase; // base address of the ZSB
    int32_t bitPos = 13;
    int32_t zeroFound = 0;

    linkPointer = HWREG(DCSM_Z2_BASE + DCSM_O_Z2_LINKPOINTER);
    linkPointer = linkPointer << 18; // Bits 31 - 14 as most-significant 0 are
                                     //invalid LinkPointer options

    while((zeroFound == 0) && (bitPos > -1))
    {
        if((linkPointer & 0x80000000U) == 0U)
        {
            zeroFound = 1;
            ZSBBase = (DCSMBANK0_Z2OTP_BASE + (((uint32_t)bitPos + 2U) * 0x20U));
        }
        else
        {
            bitPos--;
            linkPointer = linkPointer << 1;
        }
    }

    if(zeroFound == 0)
    {
        ZSBBase = (DCSMBANK0_Z2OTP_BASE + 0X20U);
    }

    *z2_key1 = dcsm_default_keys_z2[bitPos+1];

    return ZSBBase;
}

//
// End of File
//
