//###########################################################################
//
// FILE:   pbist.h
//
// TITLE:  PBIST Definitions
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



#ifndef C_BOOTROM_PBIST_H
#define C_BOOTROM_PBIST_H


//---------------------------------------------------------------------------------------------------------
//                                          CONSTANT DEFINITIONS
//---------------------------------------------------------------------------------------------------------

                                                                // PBIST register addresses
#define PBIST_DLRT                          0x5E364             // PBIST Data Logger Register
#define PBIST_STR                           0x5E36C             // Resume testing
#define PBIST_PACT                          0x5E380             // PBIST Activate Register
#define PBIST_OVERRIDE                      0x5E388             // PBIST Override Register
#define PBIST_FAIL_STATUS_0                 0x5E390             // Fail Status - Port 0
#define PBIST_FAIL_STATUS_1                 0x5E394             // Fail Status - Port 1
#define PBIST_ALGO                          0x5E3C4             // PBIST Algorithm Register
#define PBIST_RINFOL                        0x5E3C8             // RAM Info Mask Register Lower
#define PBIST_RINFOU                        0x5E3CC             // RAM Info Mask Register Higher

#define PBIST_PIE12_IER                     (0x0CE0U + 0x02U)   // PBIST Interrupt Enable Register
#define PBIST_PIE12_IFR                     (0x0CE0U + 0x03U)   // PBIST Interrupt Flag Register

                                                                // ERROR CODES - Used as return values
#define PBIST_ALWAYSFAIL_TIMEOUT_ERROR       0xFF00FF00U        // Expected fail test did not complete
#define PBIST_ALWAYSFAIL_INCORRECT_OPERATION 0xFF11FF01U
#define PBIST_DOUBLEINT_TIMEOUT_ERROR        0xFF22FF02U
#define PBIST_MEMORYTEST_TIMEOUT_ERROR       0xFF33FF03U

#define PBIST_MEMORY_TEST_FAIL_ERROR         0xFF44FF04U

#define PBIST_MEMORY_TEST_IN_PROGRESS        0xFAAB1234UL       // Return status while memory testing is in progress
#define PBIST_MEMORY_TEST_PASS               0xFAABDEEDUL       // Marker for successful completion of mem test


                                                                // Timeout values for various test configs
#define TIMEOUT_COUNT_FOR_ALWAYS_FAIL       1000
#define TIMEOUT_COUNT_FOR_FLUSHOUT          25000
#define TIMEOUT_COUNT_FOR_MEMORY_TEST       150000

#define INIT_MEM_LSX                        (MEMCFG_LSXINIT_INIT_LS0 | \
                                             MEMCFG_LSXINIT_INIT_LS1)              // Local Shared LS1-0 supported

#define INIT_MEM_DX                         (MEMCFG_DXINIT_INIT_M0 | \
                                             MEMCFG_DXINIT_INIT_M1 | \
                                             MEMCFG_DXINIT_INIT_PIEVECT )

#define TIMEOUT_OCCURRED                    0L

/**
 * Memory Sections Information:
 *
 * MEMORY GROUP NUMBER  |   MEMORY GROUP
 * ---------------------|-------------------------------------
 * 1                    |  Instruct ROM
 * 2                    |  BOOTR0
 * 3                    |  BOOTR1
 * 4                    |  m0TMURS1i
 * 5                    |  m0TMURS2i
 * 6                    |  m0TMURY0i
 * 7                    |  LSx/Mx DC BRGA Single Port RAM
 * 8                    |  PIE Single Port RAM
 * 9                    |  DCAN BYGA Two Port RAM
 * 10                   |  LS0, LS1, M0, M1 Single Port RAM
 * 11                   |  DCAN ST/FR RAM Two Port RAM
 *-----------------------------------------------------------
*/



//
//RINFOL Register Bits Description
//
#define PBIST_RINFOL_M0INSTR_ROM                ((uint32_t)1UL << 0) // ROM: m0INSTR

#define PBIST_RINFOL_M0BOOTR0                   ((uint32_t)1UL << 1)  // ROM:  m0BOOTR0
#define PBIST_RINFOL_M0BOOTR1                   ((uint32_t)1UL << 2)  // ROM:  m0BOOTR1

#define PBIST_RINFOL_TMURS1                     ((uint32_t)1UL << 3)  // ROM:  m0TMUS1
#define PBIST_RINFOL_TMURS2                     ((uint32_t)1UL << 4)  // ROM:  m0TMUS2
#define PBIST_RINFOL_TMURY0                     ((uint32_t)1UL << 5)  // ROM:  m0TMUY0

#define PBIST_RINFOL_SINGLE_PORT_DC             ((uint32_t)1UL << 6)  //LSx/Mx DC BRGA Single Port RAM
#define PBIST_RINFOL_SINGLE_PORT_PIE            ((uint32_t)1UL << 7)  // PIE Single Port RAM

#define PBIST_RINFOL_TWO_PORT_DCAN_DC_BYGA      ((uint32_t)1UL << 8)  // DCAN BYGA Two Port RAM

#define PBIST_RINFOL_SINGLE_PORT_SRAM           ((uint32_t)1UL << 9)  // LS0, LS1, M0, M1 Single Port RAM
#define PBIST_RINFOL_TWO_PORT_DCAN_ST_FR_RAM    ((uint32_t)1UL << 10) // DCAN ST/FR RAM Two Port RAM


//
//Boot ROM not included
//
#define PBIST_RINFOL_ROM                        (PBIST_RINFOL_M0INSTR_ROM | \
                                                 PBIST_RINFOL_TMURS1      | \
                                                 PBIST_RINFOL_TMURS2      | \
                                                 PBIST_RINFOL_TMURY0)

#define PBIST_RINFOL_RAM                        (PBIST_RINFOL_SINGLE_PORT_DC            | \
                                                 PBIST_RINFOL_SINGLE_PORT_PIE           | \
                                                 PBIST_RINFOL_TWO_PORT_DCAN_DC_BYGA     | \
                                                 PBIST_RINFOL_SINGLE_PORT_SRAM)

#define PBIST_RINFOL_ALL_MEM                    (PBIST_RINFOL_ROM |\
                                                 PBIST_RINFOL_RAM )

//
//ALGO register bits description.
//Note: The ALGO "groups" mentioned are DEFAULT VALUES when RINFOL is overriden.
//      When RINFOL specifies respective memories, these algorithms are run on said memories.
//
#define PBIST_ALGO_TR_M0INSTR_ROM                           ((uint32_t)1UL << 0) //Triple Read Test on group 1

#define PBIST_ALGO_TRXR_M0BOOTR0                            ((uint32_t)1UL << 1) //Triple Read XOR Test on group 2
#define PBIST_ALGO_TRXR_M0BOOTR1                            ((uint32_t)1UL << 2) //Triple Read XOR Test on group 3

#define PBIST_ALGO_TRXR_TMURS1                              ((uint32_t)1UL << 3) //Triple Read XOR Test on group 4
#define PBIST_ALGO_TRXR_TMURS2                              ((uint32_t)1UL << 4) //Triple Read XOR Test on group 5
#define PBIST_ALGO_TRXR_TMURY0                              ((uint32_t)1UL << 5) //Triple Read XOR Test on group 6

#define PBIST_ALGO_TR_M0BOOTR0                              ((uint32_t)1UL << 6) //Triple Read Test on group 2
#define PBIST_ALGO_TR_M0BOOTR1                              ((uint32_t)1UL << 7) //Triple Read Test on group 3

#define PBIST_ALGO_TR_TMURS1                                ((uint32_t)1UL << 8) //Triple Read Test on group 4
#define PBIST_ALGO_TR_TMURS2                                ((uint32_t)1UL << 9) //Triple Read Test on group 5
#define PBIST_ALGO_TR_TMURY0                                ((uint32_t)1UL << 10)//Triple Read Test on group 6

#define PBIST_ALGO_D2_SINGLE_PORT_SRAM                      ((uint32_t)1UL << 11)//Down2 Single Port Test on groups 7, 8
#define PBIST_ALGO_D2_TWO_PORT_SRAM                         ((uint32_t)1UL << 12)//Down2 Two Port Test on group 9

#define PBIST_ALGO_M13N_SINGLE_PORT_SRAM                    ((uint32_t)1UL << 13)//March13 Single Port Test on groups 7, 8
#define PBIST_ALGO_M13N_TWO_PORT_SRAM                       ((uint32_t)1UL << 14)//March13 Two Port Test on group 9

#define PBIST_ALGO_MARCH_DISTRUB_INC_SINGLE_PORT_SRAM       ((uint32_t)1UL << 15)//March Disturb INC Single Port Test on groups 10, 8
#define PBIST_ALGO_MARCH_DISTRUB_INC_TWO_PORT_SRAM          ((uint32_t)1UL << 16)//March Disturb INC Two Port Test on group 11

#define PBIST_ALGO_MARCH_DISTRUB_DEC_SINGLE_PORT_SRAM       ((uint32_t)1UL << 17)//March Disturb DEC Single Port Test on groups 10, 8
#define PBIST_ALGO_MARCH_DISTRUB_DEC_TWO_PORT_SRAM          ((uint32_t)1UL << 18)//March Disturb DEC Two Port Test on groups 11


//
//Boot ROM not included
//
#define PBIST_ALGO_ROM                      (PBIST_ALGO_TR_M0INSTR_ROM    | \
                                             PBIST_ALGO_TRXR_TMURS1       | \
                                             PBIST_ALGO_TRXR_TMURS2       | \
                                             PBIST_ALGO_TRXR_TMURY0)

#define PBIST_ALGO_RAM                      (PBIST_ALGO_M13N_SINGLE_PORT_SRAM | \
                                             PBIST_ALGO_M13N_TWO_PORT_SRAM)

#define PBIST_ALGO_ALL_MEM                  (PBIST_ALGO_ROM |\
                                             PBIST_ALGO_RAM )

                                             
                                                                // For always-fail -> Executing MARCH13n on ROM
#define PBIST_RINFOL_FAIL_ROM_MEM_GROUP     PBIST_RINFOL_M0INSTR_ROM
                                                                // ALGO GROUP --> March13n
#define PBIST_ALGO_ROM_FAIL_ALGO            PBIST_ALGO_M13N_SINGLE_PORT_SRAM    


                                                                // Macros for setting bits within a 32 bit register
#define U32_SET_BIT0                        0x00000001UL
#define U32_SET_BIT1                        0x00000002UL
#define U32_SET_BIT2                        0x00000004UL
#define U32_SET_BIT3                        0x00000008UL
#define U32_SET_BIT4                        0x00000010UL
#define U32_SET_BIT5                        0x00000020UL
#define U32_SET_BIT6                        0x00000040UL
#define U32_SET_BIT7                        0x00000080UL
#define U32_SET_BIT8                        0x00000100UL
#define U32_SET_BIT9                        0x00000200UL
#define U32_SET_BIT10                       0x00000400UL
#define U32_SET_BIT11                       0x00000800UL
#define U32_SET_BIT12                       0x00001000UL
#define U32_SET_BIT13                       0x00002000UL
#define U32_SET_BIT14                       0x00004000UL
#define U32_SET_BIT15                       0x00008000UL
#define U32_SET_BIT16                       0x00010000UL
#define U32_SET_BIT17                       0x00020000UL
#define U32_SET_BIT18                       0x00040000UL
#define U32_SET_BIT19                       0x00080000UL
#define U32_SET_BIT20                       0x00100000UL
#define U32_SET_BIT21                       0x00200000UL
#define U32_SET_BIT22                       0x00400000UL
#define U32_SET_BIT23                       0x00800000UL
#define U32_SET_BIT24                       0x01000000UL
#define U32_SET_BIT25                       0x02000000UL
#define U32_SET_BIT26                       0x04000000UL
#define U32_SET_BIT27                       0x08000000UL
#define U32_SET_BIT28                       0x10000000UL
#define U32_SET_BIT29                       0x20000000UL
#define U32_SET_BIT30                       0x40000000UL
#define U32_SET_BIT31                       0x80000000UL

#define U32_SET_ALL                         0xFFFFFFFFUL
#define U32_CLEAR_ALL                       0x00000000UL

#define PACT_REG_SET_ENABLE                 U32_SET_BIT0        // Set PBIST Activate Register ENABLE bit
                                                                // PBIST Data Logger Register bit definitions
#define DLRT_REG_CLEAR                      U32_CLEAR_ALL
#define DLRT_REG_IDDQ_TEST                  U32_SET_BIT1                                            
#define DLRT_REG_ROM_TEST                   U32_SET_BIT2        // Writing 1 to this reg bit starts ROM-based testing

#define DLRT_REG_TCK_GATED                  U32_SET_BIT3        // PBIST interrupt and fails driven to registers
#define DLRT_REG_CONFIG_ACC_CPU_PBIST       U32_SET_BIT4        // CPU is configured to access PBIST  
#define DLRT_REG_GONOGO_TEST                U32_SET_BIT9        // Required for ROM based testing
                                            
#define OVERRIDE_REG_RINFO_MEM_OVER         U32_SET_BIT0        // Configure to use the Algorithm register
#define OVERRIDE_REG_ALGO_OVER              U32_SET_BIT3        // Algorithm to use selected ROM specified in RINFO
#define OVERRIDE_REG_CLEAR_OVERRIDES        U32_CLEAR_ALL       // Clear PBIST Overrides

#define STR_REG_START                       U32_SET_BIT0        // Start / Time Stamp Mode Restart.

#define FSRF0_REG_PORT0_TEST_FAILED         0x00000001UL        // One or more memory test failed.
#define FSRF0_REG_PORT0_TEST_PASSED         0x00000000UL        // No Memory test failure occurred  

#define FSRF1_REG_PORT1_TEST_FAILED         0x00000001UL        // One or more memory test failed.
#define FSRF1_REG_PORT1_TEST_PASSED         0x00000000UL        // No Memory test failure occurred   

#define PBIST_CLEAR_INTERRUPTS              0U                  // For clearing the interrupt enable register
#define PBIST_CLEAR_INTERRUPT_FLAGS         0U                  // Clear any residual state on BOOTROM startup
#define PBIST_TEST_COMPLETE                 0x20U               // PBIST_PIE12_IFR is set, test ran to completion
#define SYSERR_CLEAR_STATUS                 0x8001UL            //Clear the SYSERR status

#endif                                                          // end of F28002X_TOPOLINO_PBIST_H definition

//---------------------------------------------------------------------------------------------------------
// End of File
//---------------------------------------------------------------------------------------------------------

