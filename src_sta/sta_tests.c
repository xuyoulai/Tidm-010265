//##############################################################################
// $Copyright:
// Copyright (C) 2017-2024 Texas Instruments Incorporated - http://www.ti.com/
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
//##############################################################################

//------------------------------------------------------------------------------
// C2000 Diagnostic Library v4.01.00
//
// FILE:  sta_tests.c
//
// TITLE: Self Test Application Tests source
//
//------------------------------------------------------------------------------

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

#include "sta_tests.h"
#include "sta_user.h"
#include "sta_util.h"
#include "sta_comm.h"
#include "sta_timer.h"
#include "stl_util.h"
#include "sta_peripheral.h"

#include <stdio.h>

//
// Defines
//

//
// Globals
//
#if defined(SAFETY_ENABLE)
const STA_TestsTypes STA_Tests_testArray[STA_TESTS_NUMBERS] =
{
    STA_TEST_START,
    STA_TEST_PIE_HANDLER,
    STA_CPU_REG,
    STA_FPU_REG,
    STA_SP_TEST,
    STA_MARCH,
    STA_MARCH_COPY,
    STA_FLASH_CRC,
    STA_OSC_CT,
    STA_TEST_END
};

#if STA_UTIL_PROFILE
uint32_t STA_Tests_cycleCounts[STA_TESTS_NUMBERS] = {0};
#endif

uint16_t STA_Tests_timer = 0U;
uint16_t STA_Tests_passCount = 0U;
uint16_t STA_Tests_index = 0U;
bool     STA_Tests_injectError = false;

//*****************************************************************************
//
// STA_Tests_testDevice(STA_TestsTypes testItem)
//
//*****************************************************************************
uint16_t STA_Tests_testDevice(STA_TestsTypes testItem)
{
#if defined(SATETY_OUTP)
    char *testReport;
#endif  // SATETY_OUTP

    uint16_t STA_Tests_reports = 0U;

    switch(testItem)
    {
        case STA_TEST_START:
        {
#if defined(SATETY_OUTP)
            testReport = "\r\n\n\n Starting Test Loop\0";
#endif  // SATETY_OUTP
            STA_Tests_reports = 0x0000;
            STA_Tests_passCount = 0U;
            break;
        }

        case STA_TEST_PIE_HANDLER:
        {
            uint16_t intGroup =
                ((uint16_t)(STA_USER_PIE_TEST_INT & STL_PIE_RAM_TABLE_COL_M) >>
                 STL_PIE_RAM_TABLE_COL_S) - 1U;
            uint16_t intMask = 1U << ((uint16_t)(STA_USER_PIE_TEST_INT &
                                                 STL_PIE_RAM_TABLE_ROW_M) - 1U);

            //
            // This test itself is injecting a fault and verifying that the
            // safety mechanism is working properly, but it can be forced to
            // fail by disabling interrupts and preventing the vector from being
            // fetched.
            //
            if(STA_Tests_injectError)
            {
                DINT;
            }

            //
            // Inject and error in the PIE RAM redundant table and generate
            // an interrupt for that entry. This will test the PIEVERRADDR
            // and PIE RAM mismatch functionality.
            //
#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            uint16_t returnVal = STL_PIE_RAM_testHandler(STA_USER_PIE_TEST_INT);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_TEST_PIE_HANDLER] = cycleCount;
#endif
            if(STL_PIE_RAM_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_TEST_PIE_HANDLER;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: PIE RAM Handler Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: PIE RAM Handler Test!\0";
#endif  // SATETY_OUTP
            }

            if(STA_Tests_injectError)
            {
                //
                // Unforce the PIE interrupt.
                //
                // Note that altering PIEIFRx in an actual application is not
                // recommended because the read/modify/write may result in
                // another interrupt in the PIE group being unintentionally
                // cleared. If you need to do it for a test like this, do it
                // at start up before your interrupts are enabled or use an
                // interrupt that is the only one enabled in its PIE group. See
                // your device technical reference manual for more details.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IFR1 + (intGroup * 2U)) &= ~intMask;

                //
                // Disable the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U)) &= ~intMask;
                Interrupt_clearACKGroup(STA_USER_PIE_TEST_INT_GROUP_M);

                //
                // Unforce the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IFR1 + (intGroup * 2U)) &= ~intMask;

                //
                // Disable the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U)) &= ~intMask;

                STL_Util_delayUS(5);

                IFR &= ~STA_USER_PIE_TEST_INT_GROUP_M;

                STL_Util_delayUS(5);

                EINT;
            }

            break;
        }
        case STA_CPU_REG:
        {
            //
            // It's recommended to disable interrupts during this test.
            //
            DINT;

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            uint16_t returnVal =
                    STL_CPU_REG_checkCPURegisters(STA_Tests_injectError);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_CPU_REG] = cycleCount;
#endif
            EINT;

            if(STL_CPU_REG_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_CPU_REG;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: CPU Register Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: CPU Register Test!\0";
#endif  // SATETY_OUTP
            }

            break;
        }

        case STA_FPU_REG:
        {
            //
            // It's recommended to disable interrupts during this test.
            //
            DINT;

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            uint16_t returnVal =
                    STL_CPU_REG_checkFPURegisters(STA_Tests_injectError);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_FPU_REG] = cycleCount;
#endif
            EINT;

            if(STL_CPU_REG_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_FPU_REG;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: FPU Register Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: FPU Register Test!\0";
#endif  // SATETY_OUTP
            }

            break;
        }

        case STA_SP_TEST:
        {
            STA_User_spObj.startAddress = (uint32_t)_symval(&__stack);
            STA_User_spObj.endAddress = (uint32_t)_symval(&__TI_STACK_END);
            STA_User_spObj.watchpoint = STL_SP_WP0;
            STA_User_spHandle = &STA_User_spObj;

            //
            // Set up watchpoint register and enable RTOS interrupt
            //
            uint16_t returnVal = STL_SP_configSP(STA_User_spHandle);

            //
            // Disable WP0, which would result in the RTOS interrupt to not be
            // generated on a write to a monitored address
            //
            if(STA_Tests_injectError)
            {
                STL_SP_disableWatchpoint(STA_User_spHandle);
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif
            //
            // Tests by writing to monitored address & ensuring RTOS interrupt
            // is generated
            //
            if(returnVal == STL_SP_CONFIG_PASS)
            {
                returnVal = STL_SP_writeMonitoredRange(STA_User_spHandle);
            }

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_SP_TEST] = cycleCount;
#endif
            if(STL_SP_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_SP_TEST;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: SP Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: SP Test!\0";
#endif  // SATETY_OUTP
            }
            break;
        }

        case STA_MARCH:
        {
            //
            // Clear RAM error status flags.
            //
            MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD);
            MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD);

            //
            // Initialize the inject error handle for stl_march.
            //
            STA_User_initMarch();

            //
            // Inject an error in the memory.
            //
            if(STA_Tests_injectError)
            {
                // This test should always PASS. Cannot inject error in middle
                // of March13N test since the test starts with a write that
                // triggers an update of the ECC.
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            STL_March_testRAM(STL_MARCH_PATTERN_TWO,
                              (uint32_t)STA_User_marchTestData,
                              (STA_USER_MARCH_DATA_SIZE / 2U) - 1U);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_MARCH] = cycleCount;
#endif

            uint16_t returnVal = STL_March_checkErrorStatus();

            // Clear RAM error status flags.
            MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD);
            MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD);

            if(STL_MARCH_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_MARCH;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: March13N No Copy Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
                // Since there is no way to inject an error for this test,
                // under normal circumstances, this code will not be reached.
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: March13N No Copy Test!\0";
#endif  // SATETY_OUTP
            }
            break;
        }

        case STA_MARCH_COPY:
        {
            uint32_t originalValue;
            uint16_t index;

            // Filling STA_User_marchTestData to make the copy easier to
            // observe.
            for(index = 0; index < STA_USER_MARCH_DATA_SIZE; index++)
            {
                STA_User_marchTestData[index] = index;
            }

            // Clear RAM error status flags.
            MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD);
            MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD);

            // Initialize the inject error handle for stl_march.
            STA_User_initMarch();

            // Inject an error in the memory.
            if(STA_Tests_injectError)
            {
                // Cause a single bit error in M0 (ECC, correctable)
                originalValue = HWREG(STA_User_marchErrorObj.address);
                STA_User_marchErrorObj.testMode = MEMCFG_TEST_WRITE_DATA;
                STA_User_marchErrorObj.xorMask = 0x00000001U;
                STL_March_injectError(STA_User_marchErrorHandle);
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            STL_March_testRAMCopy(STL_MARCH_PATTERN_ONE,
                                  (uint32_t)STA_User_marchTestData,
                                  (STA_USER_MARCH_DATA_SIZE / 2U) - 1U,
                                  (uint32_t)STA_User_marchTestDataCopy);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_MARCH_COPY] = cycleCount;
#endif

            uint16_t returnVal = STL_March_checkErrorStatus();

            // Restore value and clear status flags if error was injected.
            if(STA_Tests_injectError)
            {
                HWREG(STA_User_marchErrorObj.address) = originalValue;
                MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD);
                MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD);
            }

            if(STL_MARCH_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_MARCH_COPY;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: March13N Copy Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: March13N Copy Test!\0";
#endif  // SATETY_OUTP
            }
            break;
        }

        case STA_FLASH_CRC:
        {
            // Set range to calculate CRC for and the golden CRC value. For this
            // example, the first several words of the DCSM OTP are used.
            uint32_t startAddress = (uint32_t)STA_User_crcData;
            uint32_t endAddress = (uint32_t)(STA_User_crcData +
                                             STA_USER_CRC_DATA_SIZE) - 1U;
            uint32_t goldenCRC = STA_USER_GOLDEN_CRC;

            if(STA_Tests_injectError)
            {
                goldenCRC = goldenCRC ^ 0x0001U;
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            uint16_t returnVal = STL_CRC_checkCRC(startAddress,
                                                  endAddress,
                                                  goldenCRC);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_FLASH_CRC] = cycleCount;
#endif
            if(STL_CRC_PASS == returnVal)
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_FLASH_CRC;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: Flash CRC Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: Flash CRC Test!\0";
#endif  // SATETY_OUTP
            }
            break;
        }

        case STA_OSC_CT:
        {
            // Initialize the Oscillator Timer2 test object.
            STA_User_initOSCTimer2Test();

            if(STA_Tests_injectError)
            {
                // Allow for zero range.
                STA_User_oscTimer2Obj.minCount = STA_USER_OSC_DELAY_US;
                STA_User_oscTimer2Obj.maxCount = STA_USER_OSC_DELAY_US;
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            STL_OSC_CT_startTest(STA_User_oscTimer2Handle);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_OSC_CT] = cycleCount;
#endif

            STL_Util_delayUS(STA_USER_OSC_DELAY_US);

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(STA_PROFILER_TIMER_BASE);
#endif

            uint16_t returnVal =
                STL_OSC_CT_stopTest(STA_User_oscTimer2Handle);

#if STA_UTIL_PROFILE
            cycleCount = STA_Util_stopProfiler(STA_PROFILER_TIMER_BASE);
            STA_Tests_cycleCounts[STA_OSC_CT] += cycleCount;
#endif
            // Run the oscillator test using Timer2.
            if(STL_OSC_CT_FAIL == returnVal)
            {
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST FAIL: OSC TIMER 2 Test!\0";
#endif  // SATETY_OUTP
            }
            else
            {
                STA_Tests_passCount++;
                STA_Tests_reports |= FLAG_STA_OSC_CT;
#if defined(SATETY_OUTP)
                testReport = "\r\n\n\n TEST PASS: OSC TIMER 2 Test!\0";
#endif  // SATETY_OUTP
            }

            break;
        }

        case STA_TEST_END:
        {
            //
            // Convert test counts to characters and insert into the report.
            //
#if defined(SATETY_OUTP)
            testReport = "\r\n\n\n No Test Specified\0";
#endif  // SATETY_OUTP
            break;
        }

        default:
        {
#if defined(SATETY_OUTP)
            testReport = "\r\n\n\n No Test Specified\0";
#endif  // SATETY_OUTP
            break;
        }
    }

#if defined(SATETY_OUTP)
    printf("SDL Test index %d \n", testItem);
    printf(testReport);
#endif  // SATETY_OUTP

    return(STA_Tests_reports);
}

//*****************************************************************************
//
// STA_Tests_injectErrorEnable(void)
//
//*****************************************************************************
void STA_Tests_injectErrorEnable(void)
{
    STA_Tests_injectError = true;
}

//*****************************************************************************
//
// STA_Tests_injectErrorDisable(void)
//
//*****************************************************************************
void STA_Tests_injectErrorDisable(void)
{
    STA_Tests_injectError = false;
}
#endif  // SAFETY_ENABLE

//
// End of File
//
