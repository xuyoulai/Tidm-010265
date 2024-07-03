//###########################################################################
//
// FILE:   bootloader_spi.h
//
// TITLE:  SPI Bootloader Header
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

#ifndef BOOTLOADER_SPI_H
#define BOOTLOADER_SPI_H

//
// Includes
//
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "gpio.h"
#include "spi.h"
#include "sysctl.h"
#include "pin_map.h"
#include "cpu1brom_utils.h"
#include "cpu1brom_boot_modes.h"
#include "cpu1brom_escape_point_table.h"

//
// Defines
//
#define SPI_CHARACTER_LENGTH 8

//
// SPI Bitrate
// Based on 2.5MHz LSPCLK and SPIBRR = 127. Baud Rate = LSPCLK/(SPIBRR + 1)
//
#define SPI_BIT_RATE         19531U

//
//Function Prototype
//
extern uint32_t SPI_Boot(uint32_t bootMode);

#endif // BOOTLOADER_SPI_H
