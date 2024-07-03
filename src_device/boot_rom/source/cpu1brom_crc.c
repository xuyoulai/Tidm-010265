//###########################################################################
//
// FILE:    cpu1brom_crc.c
//
// TITLE:   CRC routine for Boot ROM integrity test
//
//
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

#include "cpu1brom_crc.h"
#include <stdint.h>


//*****************************************************************************
//
// This macro executes one iteration of the CRC-32.
//
//*****************************************************************************




/**
* \brief SW CRC
*
* Design: \ref did_software_crc_usecase did_sw_crc_interface
* Requirement: REQ_TAG(C2000BROM-230)
*
*
* SW CRC
*
*/
uint32_t CPU1BROM_crcCheck(uint32_t startAddress,
                           uint32_t endAddress,
                           uint32_t seed)
{
    uint32_t crc = seed;
    uint32_t data = 0UL;
    uint32_t i;

    for(i = startAddress; i < endAddress; i = i + 2UL)
    {
        data = HWREG(i);

        crc = CRC32_ITER(crc, data);
        crc = CRC32_ITER(crc, data >> 8);
        crc = CRC32_ITER(crc, data >> 16);
        crc = CRC32_ITER(crc, data >> 24);

    }

    return crc;
}
