//###########################################################################
//
// FILE:    bootloader_shared_funcs.c
//
// TITLE:   Bootloader shared functions
//
// Shared Functions:
//
//     void CopyData(void)
//     uint32_t GetLongData(void)
//     void ReadReservedFn(void)
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
// Globals
//
uint16fptr GetWordData; // GetWordData is a pointer to the function that
                        // interfaces to the peripheral. Each loader assigns
                        // this pointer to it's particular GetWordData function.

//
// CopyData - This routine copies multiple blocks of data from the host
//            to the specified RAM locations.  There is no error
//            checking on any of the destination addresses.
//            That is because it is assumed all addresses and block size
//            values are correct.
//
//            Multiple blocks of data are copied until a block
//            size of 00 00 is encountered.
//
void CopyData(void)
{
    struct HEADER {
        uint32_t DestAddr;        
        uint16_t BlockSize;
    } BlockHeader;

    uint16_t wordData;
    uint16_t i;

    //
    // Get the size in words of the first block
    //
    BlockHeader.BlockSize = (*GetWordData)();

    //
    // While the block size is > 0 copy the data
    // to the DestAddr.  There is no error checking
    // as it is assumed the DestAddr is a valid
    // memory location
    //
    while(BlockHeader.BlockSize != (uint16_t)0x0000U)
    {
        BlockHeader.DestAddr = GetLongData();

        for(i = 1; i <= BlockHeader.BlockSize; i++)
        {
            wordData = (*GetWordData)();
            *(uint16_t *)BlockHeader.DestAddr = wordData;
            BlockHeader.DestAddr+=1U;
        }

        //
        // Get the size of the next block
        //
        BlockHeader.BlockSize = (*GetWordData)();
    }

    return;
}

//
// GetLongData - This routine fetches a 32-bit value from the peripheral
//               input stream.
//
uint32_t GetLongData(void)
{
    uint32_t longData;

    //
    // Fetch the upper 1/2 of the 32-bit value
    //
    longData = ((uint32_t)((*GetWordData)()) << 16U);

    //
    // Fetch the lower 1/2 of the 32-bit value
    //
    longData |= (uint32_t)(*GetWordData)();

    return longData;
}

//
// ReadReservedFn - This function reads 8 reserved words in the header.
//                  None of these reserved words are used by the
//                  this boot loader at this time, they may be used in
//                  future devices for enhancements. Loaders that use
//                  these words use their own read function.
//
void ReadReservedFn(void)
{
    uint16_t i;

    //
    // Read and discard the 8 reserved words.
    //
    for(i = 1U; i <= 8U; i++)
    {
       (void)GetWordData();
    }

    return;
}

//
// End of File
//
