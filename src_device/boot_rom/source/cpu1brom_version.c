//###########################################################################
//
// FILE:   version.c
//
// TITLE:  API to return the version number of the bootROM in use.
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

#include "cpu1brom_version.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>


#pragma DATA_SECTION(cpu1brom_version, ".CPU1Version")
#pragma RETAIN(cpu1brom_version)
/**
* Version info - Version information of ROM is stored here
*
* \brief version information for ROM
*
* Design: \ref did_init_c28x_mode_algo did_init_core_config_algo did_version_and_date_algo
*              did_build_info_algo
*
*
* Requirement: REQ_TAG(C2000BROM-138), REQ_TAG(C2000BROM-139), REQ_TAG(C2000BROM-192),
*              REQ_TAG(C2000BROM-255)
*/
const cpu1brom_Version cpu1brom_version =
{
    /*store version information in a uint16, MSB is major version and LSB is minor version*/
    (uint16_t)((ROM_MAJOR_VERSION << 8U)|ROM_MINOR_VERSION),
    /*date MSB is month LSB is year (ex: �0x0920� represents �09/20� or �September 2020� */
    (uint16_t)0x0422U,
    /*build version of ROM*/
    (uint16_t)ROM_BUILD_VERSION
};
//*****************************************************************************
//
// Version_getLibVersion
//
//*****************************************************************************
bool getVersionNumber(uint32_t versionBlock[]) {
    bool retVal = false;
    if (versionBlock != NULL) {
        versionBlock[0] = (uint32_t)ROM_MAJOR_VERSION;
        versionBlock[1] = (uint32_t)ROM_MINOR_VERSION;
        versionBlock[2] = (uint32_t)ROM_PATCH_VERSION;
        versionBlock[3] = (uint32_t)ROM_BUILD_VERSION;
        retVal = true;
    }
    else {
        retVal = false;
    }

    return retVal;
}
