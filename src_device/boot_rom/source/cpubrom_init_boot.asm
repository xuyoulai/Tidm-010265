;;###########################################################################
;;
;; FILE:    cpu1brom_init_boot.asm
;;
;; TITLE:   CPU1 Boot Initialization and Exit routines.
;;
;; Functions:
;;
;;     InitBoot
;;     ExitBoot
;;     ExitPBISTLoc
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

;
; Globals
;
    .global InitBoot
    .def    ExitBoot
    .def    ExitPBISTLoc
    .ref    CPU1BROM_startSystemBoot
    .ref    CPU1BROM_setupDeviceSystems
    .ref    CPU1BROM_performDeviceConfiguration
    .ref    CPU1BROM_runDFTBoot
    .ref    CPU1BROM_bootStatus

     .sect ".test_signature1"
     .long   0xFFFFFF81                          ;~(*(0x3FFFFE)) - used for production testing

;
; Function: InitBoot
;
; This function performs the initial boot routine
; for the CPU1 boot ROM.
;
; This module performs the following actions:
;
;     1) Initializes the stack pointer
;     2) Sets the device for C28x operating mode
;     3) Setup device systems
;     4) Run RAM initialization and call the main boot function
;
; /// Design: \ref did_build_info_algo did_version_and_date_algo did_reset_non_por_algo did_rom_signature_resource
; ///			   did_reset_por_algo did_reset_non_por_algo
; /// Requirement: REQ_TAG(C2000BROM-192), REQ_TAG(C2000BROM-255), REQ_TAG(C2000BROM-153), REQ_TAG(C2000BROM-193),
; ///			   REQ_TAG(C2000BROM-142)
    .sect ".InitBoot"
InitBoot:

;
; Create stack section
;
__stack:    .usect ".stack",0

;
; Initialize the stack pointer
;
    MOV     SP, #__stack

;
; Initialize the device for running in C28x mode.
;
    C28OBJ                                       ;Select C28x object mode
    C28ADDR                                      ;Select C27x/C28x addressing
    C28MAP                                       ;Set blocks M0/M1 for C28x mode
    CLRC    PAGE0                                ;Always use stack addressing mode
    MOVW    DP, #0                               ;Initialize DP to point to the low 64 K
    CLRC    OVM                                  ;Clear overflow mode bit
;
; Set Product Mode shift to 0
;
    SPM     0

;
; Check if reset cause is from POR or XRS
; /// Design: \ref did_clear_stack_por_algo did_handle_por_xrs_resets_algo did_reset_por_algo
; /// Requirement: REQ_TAG(C2000BROM-142),  REQ_TAG(C2000BROM-172), REQ_TAG(C2000BROM-173)
;
    MOVW    DP, #0x174e                          ;Set DP to CpuSysRegs.RESC
    MOVL    ACC, @0x0                            ;Load ACC with CpuSysRegs.RESC
    MOV     AH, #0x0                             ;Clear upper 16-bit of ACC
    ANDB    AL, #0x3                             ;Mask to only include POR and XRS RESC bits
    TEST    ACC
    SBF     dft_check, EQ                       ;Branch if ACC is zero (Not a POR or XRS reset) to DFT flow check,
                                                ;else continue on with device setup/config

;
; For POR/XRS only - initialize 48 locations of M0 RAM can be used for stack
;
    MOV     AL, #0x30                            ;Load value for initializing 48 locations
    MOV     AH, #0
por_xrs_ram_zero_loop:
    MOV     *SP++, #0                            ;Write zeros to all 48 locations
    SUBB    ACC, #1
    BF      por_xrs_ram_zero_loop,GEQ
    MOV     SP, #__stack                         ;Re-Initialize the stack pointer

;
; For POR/XRS only - Setup Device Systems - Configure clocks
;                    and trim PMM/OSC/APLL/Flash
;
    LCR     CPU1BROM_setupDeviceSystems

;
; For POR/XRS only - Perform Device Configuration setup via OTP
;
    LCR     CPU1BROM_performDeviceConfiguration

;
;   Skip stack init and continue execution
;
    SB      cpu1brom_perform_ram_inits_complete, UNC

dft_check:
;
; Run DFT boot flow if enabled (check is performed within API)
;
    MOVW    DP, #0x174E                          ; Read JTAG_MMR at 0x5D3B0
    MOVL    ACC, @0x30                           ; Pass its value to runDFTBoot
    LCR     CPU1BROM_runDFTBoot

;
; Not POR - Initialize the stack used for boot to zero
;
    MOV     AL, #0xE0                           ;Size of stack
    MOV     AH, #0
stack_ram_zero_loop:
    MOV     *SP++, #0                            ;Zero out RAM for stack
    SUBB    ACC, #1
    BF      stack_ram_zero_loop, GEQ
stack_ram_init_done:
    MOV     SP, #__stack                         ;Re-Initialize the stack pointer

;
; RAM Init or Stack Init is complete, prepare to begin system initialization
;
cpu1brom_perform_ram_inits_complete:
    MOVW    DP,#0                                ;Initialize DP to point to the low 64 K
    CLRC    OVM                                  ;Clear overflow mode bit
    SPM     0                                    ;Set Product Mode shift to 0
;
; Branch to system initialization to run final initializations and execute boot mode
;
    LCR    CPU1BROM_startSystemBoot


;
; Function: ExitPBISTLoc
;
; Cleanup and exit after PBIST. At this point the EntryAddr
; is located in the ACC register
;
ExitPBISTLoc:
    BF      ExitBoot,UNC

;
; Function: ExitBoot
;
; This module cleans up after boot
;
; 1) Make sure the stack is re-initialized
; 2) Push 0 onto the stack so RPC will be
;    0 after using LRETR to jump to the
;    entry point
; 2) Load RPC with the entry point
; 3) Clear all XARn registers
; 4) Clear ACC, P and XT registers
; 5) LRETR - this will also clear the RPC
;    register since 0 was on the stack
;
ExitBoot:
;
;   Insure that the stack is re-initialized
;
    MOV     SP,#__stack

;
; Clear the bottom of the stack.  This will endup
; in RPC when we are finished
;
    MOV     *SP++,#0
    MOV     *SP++,#0

;
; Load RPC with the entry point as determined
; by the boot mode.  This address will be returned
; in the ACC register.
;
    PUSH    ACC
    POP     RPC

;
; Put registers back in their reset state.
;
; Clear all the XARn, ACC, XT, and P and DP
; registers
;
; NOTE: Leave the device in C28x operating mode
;       (OBJMODE = 1, AMODE = 0)
;
    ZAPA
    MOVL    XT,ACC
    MOVZ    AR0,AL
    MOVZ    AR1,AL
    MOVZ    AR2,AL
    MOVZ    AR3,AL
    MOVZ    AR4,AL
    MOVZ    AR5,AL
    MOVZ    AR6,AL
    MOVZ    AR7,AL
    MOVW    DP, #0

;
;   Restore ST0 and ST1.  Note OBJMODE is
;   the only bit not restored to its reset state.
;   OBJMODE is left set for C28x object operating
;   mode.
;
;  ST0 = 0x0000     ST1 = 0x0A0B
;  15:10 OVC = 0    15:13      ARP = 0
;   9: 7  PM = 0       12       XF = 0
;      6   V = 0       11  M0M1MAP = 1
;      5   N = 0       10  reserved
;      4   Z = 0        9  OBJMODE = 1
;      3   C = 0        8    AMODE = 0
;      2  TC = 0        7 IDLESTAT = 0
;      1 OVM = 0        6   EALLOW = 0
;      0 SXM = 0        5     LOOP = 0
;                       4      SPA = 0
;                       3     VMAP = 1
;                       2    PAGE0 = 0
;                       1     DBGM = 1
;                       0     INTM = 1
;
    MOV     *SP++,#0
    MOV     *SP++,#0x0A0B
    POP     ST1
    POP     ST0

;
;   Jump to the EntryAddr as defined by the
;   boot mode selected and continue execution
;
    LRETR

;
; End of file.
;
