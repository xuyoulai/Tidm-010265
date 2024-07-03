//###########################################################################
//
// FILE:    cpu1brom_pbist.c
//
// TITLE:   pbist code for CPU1-Core
//
//
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


// --------------------------------------------------------------------------------------------------------
//                                           INCLUDE FILES
// --------------------------------------------------------------------------------------------------------
#include "hw_pbist.h"
#include "cpu1bootrom.h"
#include "cpu1brom_pbist.h"


uint32_t PBIST_PORMemoryTest(void);

// --------------------------------------------------------------------------------------------------------
//                                             MACROS
// --------------------------------------------------------------------------------------------------------

//
// Enable Peripherals Clocks
// PCLKCR10-DCAN, and
// PCLKCR22-PBIST.
//
#define PBIST_ENABLE_ALL_PERIPHERAL_CLOCKS()                  \
        {                                                     \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR10) = 0x1UL;   \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR22) = 0x1UL;   \
        }

//
// Disable peripherals Clocks
// PCLKCR10-DCAN and
// PCLKCR22-PBIST.
//
#define PBIST_DISABLE_PERIPHERAL_CLOCKS()                     \
        {                                                     \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR10) = 0x0UL;   \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR22) = 0x0UL;   \
        }

//
// Soft Peripherals Reset
// 12-DUMMY to provide enough cycles so that Softpres
// can take effect. In the beginning, PBIST_ACTIVE = 1,
// all peripherals be mapped to CPU1 SYSCTL_O_SOFTPRES10 --> DCAN
//
#define PBIST_SOFT_RESET_PERIPHERALS()                        \
        {                                                     \
            HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES10) = 0x1UL; \
            asm(" RPT #14 || NOP");                           \
            HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES10) = 0x0UL; \
            asm(" RPT #14 || NOP");                           \
            HWREG(MPOST_BASE + PBIST_O_PACT)         = 0x0UL; \
            asm(" RPT #20 || NOP");                           \
            HWREGH(PIECTRL_BASE + PBIST_PIE12_IFR)   = 0x0U;  \
        }

//
// SRAM Init, LS0, LS1, M0, M1, PIEVECTRAM
// Calculation of RAM init wait time
// Largest Memory block = LSx = 16KB per block
// = [256 * (size of largest memory block in KB)/4] + 104 (buffer)
// = 4096 + 104  = 4200
//
#define PBIST_RAM_INIT()                                               \
        {                                                              \
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT) = INIT_MEM_LSX;      \
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT)  = INIT_MEM_DX;       \
            asm(" MOV  @T,   #4200 ");                                 \
            asm(" RPT  @T || NOP ");                                   \
        }

/**
*-------------------------------------------------------------------------------------------------
* uint32_t PBIST_PORMemoryTest(void)
*-------------------------------------------------------------------------------------------------
*
*! \brief      Test all on chip ROMs using Triple Read XOR Read (TRXR) memory test algorithm.
*              and test all on chip RAM using the March13 memory test algorithm.
*
*! \param[in]  None
*
*  \return     uint32_t return type using following macros with an encoded PBIST status
*                       1 PBIST_MEMORY_TEST_PASS               - All tests passed successfully
*                       2 PBIST_ALWAYSFAIL_TIMEOUT_ERROR       - Expected fail test did not complete
*                                                                Timeout occurred waiting for interrupt
*                       3 PBIST_ALWAYSFAIL_INCORRECT_OPERATION - PBIST Fail status set
*
*                       4 PBIST_DOUBLEINT_TIMEOUT_ERROR        - Error waiting for expected
*                                                                second interrupt
*                       5 PBIST_MEMORYTEST_TIMEOUT_ERROR       - Timeout occurred waiting for
*                                                                memory test to complete
*                       6 PBIST_MEMORY_TEST_FAIL_ERROR         - Memory test failure occurred
*                       7 PBIST_MEMORY_LS_INITDONE_ERROR       - LS Memory initialization failed
*                       8 PBIST_MEMORY_GX_INITDONE_ERROR       - GS Memory initialization failed
*                       9 PBIST_MEMORY_M0M1_INITDONE_ERROR     - M0 and/or M1 memory init failed
*
*
*  \note   1. Test 1: Setup negative test with the expectation that the test must fail.  Running
*             march13 on ROM, is destined to fail.
*
*          2. 1500 clock cycles are taken for PBIST to start testing and report the first fail and
*             PBIST to trigger an interrupt. Count gets decremented every 16 cycles according to
*             the shown loop structure. A safe value for count would be decimal of 750.
*
*          3. Test 2:  All available ROMs and RAMs are tested. RINFOL (RAM Info Mask Register Low)
*             is programmed using the PBIST_RINFOL_ALL_MEM macro and the Algorithm register is
*             programmed using the PBIST_ALGO_ALL_MEM macro.
*
*          4. PBIST test is destructive on all RAMs, hence register pbist_status to maintain
*             the status through the tests.
*
*          5. Comprehend the following error conditions:
*             a. Both PBIST and RAM Init complete successfully, return PBIST_MEMORY_TEST_PASS
*                If( (PBIST == PASS) && (RAM Init == PASS) then return PBIST PASS
*             b. PBIST completes successfully but RAM Init fails
*                If( (PBIST == PASS) && (RAM Init == FAIL) then return RAM FAIL
*             c. PBIST fails but RAM Init passes
*                If( (PBIST == FAIL) && (RAM Init == PASS) then return PBIST FAIL
*             d. PBIST fails and RAM Init fails, create a composite error
*                pbist error is in the upper 16 bits, ram init error is in the lower 16 bits
-------------------------------------------------------------------------------------------------

* \brief PBIST
*
* Design: \ref did_pbist_usecase did_clear_pbist_status_algo did_ram_init_por_algo did_ram_init_non_por_algo
*              did_ram_init_fail_algo did_MPOST_usecase did_ram_init_complete_check_algo did_safety_mpost_status_reset_algo
* Requirement: REQ_TAG(C2000BROM-164), REQ_TAG(C2000BROM-227), REQ_TAG(C2000BROM-229).
*              REQ_TAG(C2000BROM-231), REQ_TAG(C2000BROM-226), REQ_TAG(C2000BROM-228),
*              REQ_TAG(C2000BROM-226), REQ_TAG(C2000BROM-152), REQ_TAG(C2000BROM-153),
*              REQ_TAG(C2000BROM-174), REQ_TAG(C2000BROM-336)
*
* PBIST
*
*/
uint32_t PBIST_PORMemoryTest(void)
{

    register uint32_t pbist_status =  PBIST_MEMORY_TEST_IN_PROGRESS;    // register running status, See Note 4, Initialize pbist_status to a known state
    register int32_t  timeout_count;

    DINT;                                                               // Disable interrupts.
    DRTM;                                                               // Disable realtime mode

    EALLOW;
    PBIST_ENABLE_ALL_PERIPHERAL_CLOCKS();                               // Enable necessary peripheral clocks.

                                                                        // Test 1: Configuring PBIST for expected fail test
                                                                        // Performing March 13n test on TMU ROM.  See Note 1.
    HWREGH(PBIST_PIE12_IER) = PBIST_CLEAR_INTERRUPTS;
    HWREGH(PBIST_PIE12_IFR) = PBIST_CLEAR_INTERRUPT_FLAGS;              // Clear any possible left over hardware state
    HWREG(PBIST_PACT)       = PACT_REG_SET_ENABLE;                      // Activate PBIST, begin setup for PBIST usage
                                                                        // Configure to Override Algo and RINFO registers
    HWREG(PBIST_OVERRIDE)   = (OVERRIDE_REG_ALGO_OVER | OVERRIDE_REG_RINFO_MEM_OVER);

    HWREG(PBIST_DLRT)       = DLRT_REG_CONFIG_ACC_CPU_PBIST;    // Configure CPU to use PBIST

    HWREG(PBIST_RINFOL)     = PBIST_RINFOL_FAIL_ROM_MEM_GROUP;  //
    HWREG(PBIST_RINFOU)     = 0;                                // RINFO upper is unused/don't care
    HWREG(PBIST_ALGO)       = PBIST_ALGO_ROM_FAIL_ALGO;         // Effectively selects M13 for ROM. Expect fail.

    HWREG(PBIST_OVERRIDE)   = OVERRIDE_REG_CLEAR_OVERRIDES;     // Use algorithm register and RINFOL
                                                                // Initiate ROM test.  Expect test to complete and fail
    HWREG(PBIST_DLRT)       = (DLRT_REG_CONFIG_ACC_CPU_PBIST | DLRT_REG_ROM_TEST);
    EDIS;

    timeout_count     = TIMEOUT_COUNT_FOR_ALWAYS_FAIL;

    while (PBIST_TEST_COMPLETE != HWREGH(PBIST_PIE12_IFR))      // Expect intrpt flag set on test completion. See Note 2
    {
        timeout_count--;                                        // If test completion flag not set, wait for timeout

        if (TIMEOUT_OCCURRED >= timeout_count)
        {
                                                                // PBIST ctlr has timed-out. Clean up and return an error
            pbist_status = PBIST_ALWAYSFAIL_TIMEOUT_ERROR;
            break;
        }
    }

    if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
                                                                // Ensure expected fault occurred.
        if ((FSRF0_REG_PORT0_TEST_PASSED == (HWREG(PBIST_FAIL_STATUS_0))) &&
            (FSRF1_REG_PORT1_TEST_PASSED == (HWREG(PBIST_FAIL_STATUS_1))))
        {
                                                                // Expected failure did not occur fail status is not set
            pbist_status = PBIST_ALWAYSFAIL_INCORRECT_OPERATION;
        }
    }

    EALLOW;
    HWREG(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_INT_CLR) = SYSERR_CLEAR_STATUS; //Clear the SYSERR status
    EDIS;


    if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
   {
        EALLOW;
        HWREGH(PBIST_PIE12_IFR) = PBIST_CLEAR_INTERRUPTS;

        HWREG(PBIST_DLRT)       = DLRT_REG_CONFIG_ACC_CPU_PBIST | \
                                  DLRT_REG_ROM_TEST             | \
                                  DLRT_REG_IDDQ_TEST;

        HWREG(PBIST_STR)        = STR_REG_START;                // Start / Time Stamp Mode Restart.
        EDIS;

        timeout_count = TIMEOUT_COUNT_FOR_FLUSHOUT;

        while (PBIST_TEST_COMPLETE != HWREGH(PBIST_PIE12_IFR))  // Expect and process second interrupt
        {
            timeout_count--;
            if(TIMEOUT_OCCURRED >= timeout_count)
            {
                pbist_status = PBIST_DOUBLEINT_TIMEOUT_ERROR;   // Timeout occurred waiting for second interrupt
                break;
            }
        }

        EALLOW;
        HWREG(SYSSTAT_BASE + SYSCTL_O_SYS_ERR_INT_CLR) = SYSERR_CLEAR_STATUS; //Clear the SYSERR status
        EDIS;

    }
                                                               // Test 2:  Configure to test all memories. See Note 3
    if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
        EALLOW;
        HWREGH(PBIST_PIE12_IFR) = PBIST_CLEAR_INTERRUPTS;
        HWREG(PBIST_DLRT)       = DLRT_REG_CLEAR;
        HWREG(PBIST_OVERRIDE)   = (OVERRIDE_REG_ALGO_OVER | OVERRIDE_REG_RINFO_MEM_OVER);

        HWREG(PBIST_DLRT)       = DLRT_REG_CONFIG_ACC_CPU_PBIST;// Configure CPU to use PBIST

        HWREG(PBIST_RINFOL)     = PBIST_RINFOL_ALL_MEM;         // Configure RAM Info Reg to test all memories
        HWREG(PBIST_RINFOU)     = 0UL;

        HWREG(PBIST_ALGO)       = PBIST_ALGO_ALL_MEM;           // Configure Algorithm Reg to test all memories
        HWREG(PBIST_OVERRIDE)   = OVERRIDE_REG_RINFO_MEM_OVER;  // Configure to use the Algorithm register

        HWREG(PBIST_DLRT)       = DLRT_REG_GONOGO_TEST          | \
                                  DLRT_REG_CONFIG_ACC_CPU_PBIST | \
                                  DLRT_REG_ROM_TEST;
        EDIS;
        timeout_count = TIMEOUT_COUNT_FOR_MEMORY_TEST;

        while (PBIST_TEST_COMPLETE != HWREGH(PBIST_PIE12_IFR))  // Check interrupt flag to see if test completed
        {
            timeout_count--;
            if (TIMEOUT_OCCURRED >= timeout_count)
            {
                pbist_status = PBIST_MEMORYTEST_TIMEOUT_ERROR;  // Timeout occurred waiting for completion interrupt
                break;
            }
        }
    }

	if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
																// If there is an error, cleanup and report error
		if((FSRF0_REG_PORT0_TEST_FAILED == HWREG(PBIST_FAIL_STATUS_0)) ||
		   (FSRF0_REG_PORT0_TEST_FAILED == HWREG(PBIST_FAIL_STATUS_1)))
		{
			pbist_status = PBIST_MEMORY_TEST_FAIL_ERROR;        // PBIST Fail status indicates memory test failed
		}
		else
		{
            pbist_status = PBIST_MEMORY_TEST_PASS;              // All memories were tested successfully
		}
	}
                                                                // All Done! Cleanup and return status of test
    EALLOW;
    PBIST_SOFT_RESET_PERIPHERALS();
    PBIST_DISABLE_PERIPHERAL_CLOCKS();
    PBIST_RAM_INIT();
    EDIS;

    return (pbist_status);
}

//-------------------------------------------------------------------------------------------------------
//                                               END OF FILE
//-------------------------------------------------------------------------------------------------------

