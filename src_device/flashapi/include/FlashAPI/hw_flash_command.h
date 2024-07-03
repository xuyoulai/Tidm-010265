//###########################################################################
//
// FILE:    hw_flash_command.h
//
// TITLE:   Definitions for the FLASH registers.
//
//###########################################################################
// $TI Release: F280013x Support Library v5.02.00.00 $
// $Release Date: 04-07-2024 $
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef HW_FLASH_COMMAND_H
#define HW_FLASH_COMMAND_H

#ifdef __cplusplus
extern "C"
{
#endif
//*************************************************************************************************
//
// The following are defines for the FLASH register offsets
//
//*************************************************************************************************

#define FLASH_O_CMDWEPROTA        0x11D0U   // Command Write Erase Protect A Register
#define FLASH_O_CMDWEPROTB        0x11D4U   // Command Write Erase Protect B Register
#define FLASH_O_CMDWEPROT_UO      0x1210U   // Command Write Erase Protect USER DCSM OTP Register

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif