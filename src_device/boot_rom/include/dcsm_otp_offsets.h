//###########################################################################
//
// FILE:    dcsm_otp_offsets.h
//
// TITLE:   DCSM otp offsets and addresses
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

#ifndef DCSM_OTP_OFFSETS_H_
#define DCSM_OTP_OFFSETS_H_


#define TIOTP1_SECDC                    0x000715F0UL     // TI OTP SECDC Location

//DCSM ZSB Content Offsets
#define DCSM_O_Zx_CSMPSWD0              0x0UL
#define DCSM_O_Zx_CSMPSWD1              0x2UL
#define DCSM_O_Zx_CSMPSWD2              0x4UL
#define DCSM_O_Zx_CSMPSWD3              0x6UL
#define DCSM_O_Zx_GRABSECT1             0x8UL
#define DCSM_O_Zx_GRABSECT2             0xAUL
#define DCSM_O_Zx_GRABSECT3             0xCUL
#define DCSM_O_Zx_GRABRAM1              0xEUL
#define DCSM_O_Zx_GRABRAM2              0x10UL
#define DCSM_O_Zx_GRABRAM3              0x12UL
#define DCSM_O_Zx_EXEONLYSECT1          0x14UL
#define DCSM_O_Zx_EXEONLYSECT2          0x16UL
#define DCSM_O_Zx_EXEONLYRAM1           0x18UL
#define DCSM_O_Zx_RESERVED3             0x1AUL
#define DCSM_O_Z1_JTAGPSWDL0            0x1CUL
#define DCSM_O_Z1_JTAGPSWDL1            0x1EUL


#endif /* DCSM_OTP_OFFSETS_H_  */



