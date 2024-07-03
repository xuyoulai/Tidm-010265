//###########################################################################
//
// FILE:   cpu1brom_escape_point_table.h
//
// TITLE:  Contains escape point table
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

#ifndef BROM_ESCAPE_POINT_H
#define BROM_ESCAPE_POINT_H

//
// Hardware Assisted Table
//
#define CPU1BROM_HW_PATCH_TABLE_START           0x7156AUL
#define CPU1BROM_HW_NUMBER_OF_PATCHES           0x8U
#define CPU1BROM_HW_PATCH_TABLE_DATA_OFFSET     0x2UL
#define CPU1BROM_HW_PATCH_TABLE_NEXT_ENTRY      0x4UL
#define CPU1BROM_HW_PATCH_REG_NEXT_ENTRY        0x2U
#define CPU1BROM_HW_PATCH_SKIP_CONFIG           0xFFFFFFFFUL

#define TI_OTP_HW_PATCH_TABLE_KEY               (HWREG(0x71568UL))
#define HW_PATCH_TABLE_KEY                      0x5A5A5A5AUL

//
// Escape Point Table
//

#define TI_OTP_SW_PATCH_POINT_KEY               (HWREG(0x715B0UL))
#define SW_PATCH_POINT_KEY                      0x5A5A5A5AUL

#define OTP_BOOT_ESCAPE_TABLE_START             0x715B2UL
#define CPU1BROM_TI_OTP_ESCAPE_POINT_1          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+0U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_2          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+1U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_3          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+2U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_4          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+3U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_5          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+4U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_6          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+5U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_7          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+6U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_8          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+7U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_9          (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+8U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_10         (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+9U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_11         (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+10U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_12         (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+11U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_13         (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+12U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_14         (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+13U))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_15         (HWREGH((OTP_BOOT_ESCAPE_TABLE_START)+14U))


#define EXECUTE_ESCAPE_POINT(PATCH_OFFSET)      ((void (*)(void))(0x71000UL + (uint32_t)PATCH_OFFSET))()

extern uint32_t swPatchKey;

#endif // BROM_ESCAPE_POINT_H
