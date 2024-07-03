//###########################################################################
//
// FILE:   cpu1brom_crc.h
//
// TITLE:  Header File for Software CRC functionality for F280013x Boot ROM.
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

#ifndef CPU1BROM_CRC_H
#define CPU1BROM_CRC_H

#include "cpu1bootrom.h"
#include "cpu1brom_crc_common.h"
//
//Golden CRC Address is immediately after test signature address (ROM_START_ADDRESS)
//
#define TI_OTP_GOLDEN_CRC_ADDRESS    0x715E8UL

//
// 8 - for skipping first location and version in CRC calculation.
//
#define CRC_START_ADDRESS       (ROM_START_ADDRESS + 8UL)

//
//CRC calculated till the end of vector table
//
#define CRC_END_ADDRESS         0x003FFFFFUL

//
// Initial value of crc prior to calculation
//
#define CRC_SEED                0x0UL

//
//Final CRC pass flag captured in cpubrom_boot.c
//

#define CRC_TEST_PASS           0x02000000UL

/**
*Function to calculate CRC-32 (polynomial: 0x04C11DB7) over any given block of memory
*and compare it against a golden value.
*
*Arguments:
*@param startAddress        : First address of the block of memory for CRC32 computation.
*@param endAddress          : Last address of the block of memory for CRC32 computation.
*@param seed                : Initial seed value of CRC.
*
*@return                    : Computed CRC value
*
*
* Design: \ref did_safety_crc_algo
* Requirement: REQ_TAG(C2000BROM-230)
*
*/
extern uint32_t CPU1BROM_crcCheck(uint32_t startAddress,
                                  uint32_t endAddress,
                                  uint32_t seed);


#endif
