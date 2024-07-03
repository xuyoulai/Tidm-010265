//###########################################################################
//
// FILE:   cpu1brom_boot_modes.h
//
// TITLE:  Contains boot modes and entry addresses
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

#ifndef BROM_BOOT_MODES_H
#define BROM_BOOT_MODES_H

//
// BootROM System Clock (10MHz)
//
#define BOOTROM_SYSCLK         10000000UL

//
// Boot Mode Values
//
#define PARALLEL_BOOT           0x00U   //0,1,3,4,5,7,28,29,224,242
#define PARALLEL_BOOT_ALT1      0x20U   //0-7,12,13
#define PARALLEL_BOOT_ALT2      0x40U   //0-7,16,29

#define SCI_BOOT                0x01U   //GPIO29; GPIO28 (CCARD)
#define SCI_BOOT_ALT1           0x21U   //GPIO1;  GPIO0
#define SCI_BOOT_ALT2           0x41U   //GPIO8;  GPIO9
#define SCI_BOOT_ALT3           0x61U   //GPIO7;  GPIO3
#define SCI_BOOT_ALT4           0x81U   //GPIO16; GPIO3

#define CAN_BOOT                0x02U   //GPIO4; GPIO5
#define CAN_BOOT_ALT1           0x22U   //GPIO32; GPIO33
#define CAN_BOOT_ALT2           0x42U   //GPIO2; GPIO3
#define CAN_BOOT_ALT3           0x62U   //GPIO13, GPIO12
#define CAN_BOOT_SENDTEST       0x82U   //GPIO4; GPIO5
#define CAN_BOOT_ALT1_SENDTEST  0xA2U   //GPIO32; GPIO33
#define CAN_BOOT_ALT2_SENDTEST  0xC2U   //GPIO2; GPIO3
#define CAN_BOOT_ALT3_SENDTEST  0xE2U   //GPIO13, GPIO12

#define FLASH_BOOT              0x03U   //BANK0 sector 0
#define FLASH_BOOT_ALT1         0x23U   //BANK0 sector 32
#define FLASH_BOOT_ALT2         0x43U   //BANK0 end of sector 63
#define FLASH_BOOT_ALT3         0x63U   //BANK0 sector 64
#define FLASH_BOOT_ALT4         0x83U   //BANK0 sector 96
#define FLASH_BOOT_ALT5         0xA3U   //BANK0 end of sector 127

#define WAIT_BOOT               0x04U   //with WDOG enabled
#define WAIT_BOOT_ALT1          0x24U   //without WDOG enabled

#define RAM_BOOT                0x05U

#define SPI_MASTER_BOOT         0x06U   //GPIO 7,1,3,5
#define SPI_MASTER_BOOT_ALT1    0x26U   //GPIO 16,1,3,0
#define SPI_MASTER_BOOT_ALT2    0x46U   //GPIO 8,10,9,11
#define SPI_MASTER_BOOT_ALT3    0x66U   //GPIO 16,13,12,29

#define I2C_MASTER_BOOT         0x07U   //GPIO0(SDA), GPIO1(SCL)
#define I2C_MASTER_BOOT_ALT1    0x27U   //GPIO32 (SDA), GPIO33 (SCL)
#define I2C_MASTER_BOOT_ALT2    0x47U   //GPIO5(SDA), GPIO4 (SCL)

#define SECURE_FLASH_BOOT       0x0AU    //BANK0 sector 0
#define SECURE_FLASH_BOOT_ALT1  0x2AU    //BANK0 sector 32
#define SECURE_FLASH_BOOT_ALT2  0x4AU    //BANK0 end of sector 63
#define SECURE_FLASH_BOOT_ALT3  0x6AU    //BANK0 sector 64
#define SECURE_FLASH_BOOT_ALT4  0x8AU    //BANK0 sector 96

//
// Entry Addresses
//
#define FLASH_ENTRY_POINT       0x00080000UL    //BANK0 sector 0
#define FLASH_ENTRY_POINT_ALT1  0x00088000UL    //BANK0 sector 32
#define FLASH_ENTRY_POINT_ALT2  0x0008FFF0UL    //BANK0 end of sector 63
#define FLASH_ENTRY_POINT_ALT3  0x00090000UL    //BANK0 sector 64
#define FLASH_ENTRY_POINT_ALT4  0x00098000UL    //BANK0 sector 96
#define FLASH_ENTRY_POINT_ALT5  0x0009FFF0UL    //BANK0 end of sector 127

#define RAM_ENTRY_POINT         0x000000U       //M0 start address

//
// Misc
//
#define BROM_EIGHT_BIT_HEADER   0x08AAU

//
// Bootloader function pointer
//
typedef uint16_t (*uint16fptr)(void);
extern  uint16fptr GetWordData;

//
// Function Prototypes
//
extern uint32_t GetLongData(void);
extern void CopyData(void);
extern void ReadReservedFn(void);

#endif // BROM_BOOT_MODES_H
