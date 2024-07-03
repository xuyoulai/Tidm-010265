;;###########################################################################
;;
;; FILE:    cpu1brom_utility_funcs.asm
;;
;; TITLE:   Boot Rom C callable assembly utility functions.
;;
;; Utility functions written in assembly used as part of boot.
;;
;;###########################################################################
;; $TI Release: $
;; $Release Date:  $
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
    .global load_itrap_address

;
; Function: _load_itrap_address
; C-calling convention: load_itrap_address (unsigned long *itrapaddress)
; Parameter is stored in XAR4
;
; Function loads the address where the ITRAP occurred stored from the stack and
; into the address pointed to by the itrapaddress pointer.
; The stack location where the ITRAP return address is stored on the stack will
; change according to compiler optimizations, The below value is without any
; optimizations - if the code uses optimizations please re-check the below value
;
load_itrap_address:
    MOVL ACC, *-SP[30];   ; load ITRAP address location on stack into ACC
    MOVL *XAR4, ACC       ; return ITRAP address back to calling function
    LRETR

;//
;// End of file.
;//
