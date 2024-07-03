//###########################################################################
//
// FILE:   cpu1brom_version.h
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

#ifndef CPU1BROM_VERSION_H
#define CPU1BROM_VERSION_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/* Version number defines for including build numbers into ROM. These are used
in an array with a directive to place the data directly after the interrupt
table in the appropriate code ROM */

#define ROM_MAJOR_VERSION 0x00000002UL
#define ROM_MINOR_VERSION 0x00000000UL
#define ROM_PATCH_VERSION 0x00000001UL
#define ROM_BUILD_VERSION 0x00000001UL

bool getVersionNumber(uint32_t versionBlock[]);

typedef struct cpu1brom_Version_
{
    /*store version information in a uint16, MSB is major version and LSB is minor version*/
    uint16_t version;
    /*date MSB is month LSB is year (ex: �0x0920� represents �09/20� or �September 2020�)*/
    uint16_t date;
    /*build version of ROM*/
    uint16_t buildVersion;
}cpu1brom_Version;

#ifdef __cplusplus
}
#endif

#endif // CPU1BROM_VERSION_H
