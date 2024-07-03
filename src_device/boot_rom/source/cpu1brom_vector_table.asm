;;###########################################################################
;;
;; FILE:    cpu1brom_vector_table.asm
;;
;; TITLE:   CPU1 Boot Rom vector table.
;;
;; This section of code populates the vector table in the boot ROM.
;; The reset vector at 0x3FFFC0 points to the entry into the boot loader
;; functions (InitBoot()).
;; The rest of the vectors are populated for handling/catching issues during 
;; boot up and test purposes only.
;;
;;###########################################################################
;; $TI Release: $
;; $Release Date: $
;; $Copyright:
;// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
;//
;// Redistribution and use in source and binary forms, with or without 
;// modification, are permitted provided that the following conditions 
;// are met:
;// 
;//   Redistributions of source code must retain the above copyright 
;//   notice, this list of conditions and the following disclaimer.
;// 
;//   Redistributions in binary form must reproduce the above copyright
;//   notice, this list of conditions and the following disclaimer in the 
;//   documentation and/or other materials provided with the   
;//   distribution.
;// 
;//   Neither the name of Texas Instruments Incorporated nor the names of
;//   its contributors may be used to endorse or promote products derived
;//   from this software without specific prior written permission.
;// 
;// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
;// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
;// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
;// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
;// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;// $
;;###########################################################################

;-----------------------------------------------------------
;
; The vector table located in boot ROM at 0x3F FFC0 - 0x3F FFFF
; will be filled with the following data.
;
; Only the reset vector, which points to the InitBoot
; routine will be used during normal operation.  The remaining
; vectors are set for internal testing purposes along with handling issues
; during boot up and will not be fetched from this location during 
; normal operation.
;
; On reset vector is always fetched from this table.
;
;----------------------------------------------------------
     .ref InitBoot
     .ref CPU1BROM_nmiHandler
     .ref CPU1BROM_itrapISR
     .ref CPU1BROM_pieVectorMismatchHandler
     .sect ".BootVecs"
      BF CPU1BROM_pieVectorMismatchHandler, UNC  ;pie vect mismatch handler at address 0x3fffbe
     .long InitBoot               ;Reset
     .long 0x000042
     .long 0x000044
     .long 0x000046
     .long 0x000048
     .long 0x00004a
     .long 0x00004c
     .long 0x00004e
     .long 0x000050
     .long 0x000052
     .long 0x000054
     .long 0x000056
     .long 0x000058
     .long 0x00005a
     .long 0x00005c
     .long 0x00005e
     .long 0x000060
     .long 0x000062
     .long CPU1BROM_nmiHandler    ;NMI
     .long CPU1BROM_itrapISR      ;ITRAP
     .long 0x000068
     .long 0x00006a
     .long 0x00006c
     .long 0x00006e
     .long 0x000070
     .long 0x000072
     .long 0x000074
     .long 0x000076
     .long 0x000078
     .long 0x00007a
     .long 0x00007c
     .long 0x00007e

;//
;// End of file.
;//
