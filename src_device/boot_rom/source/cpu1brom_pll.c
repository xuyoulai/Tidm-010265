//###########################################################################
//
// FILE:    cpu1brom_pll.c
//
// TITLE:   PLL Enable and Power up Functions
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

//
// Included Files
//
#include "cpu1bootrom.h"

//
// Function Prototypes
//
uint16_t BROMDCC_verifySingleShotClock(DCC_Count0ClockSource clk0src,
                                       DCC_Count1ClockSource clk1src, uint16_t dccCounterSeed0,
                                       uint16_t dccCounterSeed1, uint16_t dccValidSeed0);

/**
* CPU1BROM_triggerSysPLLLock - Power up and lock the SYS PLL.
* The "divider" configured in this routine is PLL Output Divider (ODIV)
* and not "SYSCLKDIVSEL".
*
*
* \brief PLL Lock function
*
* Design: \ref did_trigger_apll_lock_usecase did_enable_pll_lock_algo
*              did_pll_lock_fail_status_algo
* Requirement: REQ_TAG(C2000BROM-214), REQ_TAG(C2000BROM-164)
*
* PLL Lock function
*
*/
void CPU1BROM_triggerSysPLLLock(uint32_t multiplier, uint32_t divider)
{
	EALLOW;

    //
    // Bypass PLL from SYSCLK
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLCLKEN;

    //
    // Delay of at least 120 OSCCLK cycles required post PLL bypass
    //
    asm(" MOV    @T,#120 ");
    asm(" RPT    @T \
          || NOP ");

    //
    // Use INTOSC2 (~10 MHz) as the main PLL clock source
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &= ~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

    //
    // Delay of at least 60 OSCCLK cycles after clock source change
    //
    asm(" MOV    @T,#60 ");
    asm(" RPT    @T \
          || NOP ");

	//
	// Turn off PLL and delay for power down
	//
	HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLEN;

    //
    // Delay 66 cycles from powerdown to powerup
    //
    asm(" MOV    @T,#66 ");
    asm(" RPT    @T \
          || NOP ");

    //
    // Set PLL Multiplier and Output Clock Divider (ODIV)
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) =
                     ((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                     ~(SYSCTL_SYSPLLMULT_ODIV_M | SYSCTL_SYSPLLMULT_IMULT_M)) |
                              (divider << SYSCTL_SYSPLLMULT_ODIV_S) |
                              (multiplier << SYSCTL_SYSPLLMULT_IMULT_S));

	//
	// Enable the sys PLL
	//
	HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |= SYSCTL_SYSPLLCTL1_PLLEN;

    EDIS;
}

/**
* \brief Switch to PLL output
*
* Design: \ref did_safety_switch_to_pll_clock_usecase
* Requirement: REQ_TAG(C2000BROM-215), REQ_TAG(C2000BROM-164)
*
* PLL Lock function
*
*/
uint16_t CPU1BROM_switchToPLL(uint32_t bootConfig)
{
	uint16_t count = 1024; // timeout
	uint16_t dccCnt0Seed, dccCnt1Seed, dccValid0Seed;
	uint16_t dccStatus;
    uint16_t sysclkdiv;

    //
    // Setup DCC Values
    //
    dccCnt0Seed = 1128U;
    dccValid0Seed = 144U;

    //
    // + below is to convert bit field values to actual divider values
    // ((uint16_t)((120UL * pllMultiplier)/(pllDivider + 1UL)));
    //
	dccCnt1Seed = 22800U; 

	//
	// Wait for the SYSPLL lock counter
	//
	while(((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLSTS) &
			SYSCTL_SYSPLLSTS_LOCKS) == 0U) && (count != 0U))
	{
		count--;
	}

	//
	// Using DCC to verify the PLL clock
	//
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
	dccStatus = BROMDCC_verifySingleShotClock((DCC_Count0ClockSource)DCC_COUNT0SRC_INTOSC2,
											  (DCC_Count1ClockSource)DCC_COUNT1SRC_PLL,
											  dccCnt0Seed, dccCnt1Seed, dccValid0Seed);
	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_DCC0);

	//
    // If DCC failed to verify PLL clock is running correctly, update status
	// and power down PLL
    //
    if(ERROR == dccStatus)
    {
        //
        // Turn off PLL and delay for power down
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLEN;
        EDIS;

        //
        // Delay 25 cycles
        //
        asm(" MOV    @T,#25 ");
        asm(" RPT    @T \
              || NOP ");
    }
    else
    {
        //
        // Read clkdiv from OTP. 
        //
        sysclkdiv = (uint16_t)((bootConfig & 0x00FCUL) >> 2);
        
        //
        // If it is invalid value (< /2) set the divider to /2.
        // sysclkdiv == 0 means div by 1, so set it to div by 2 to ensure the 
        // sysclk doesnt go beyond 95Mhz (based on APLL_MULT_38 and APLL_DIV_2 
        // set above).
        //
        if(0UL == sysclkdiv)
        {
            sysclkdiv = 1;
        }

        EALLOW;        
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = ((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                                                    ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | sysclkdiv);
        
        //
        // Switch sysclk to PLL clock
        //
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |= SYSCTL_SYSPLLCTL1_PLLCLKEN;
        EDIS;
        
        //
        // ~200 PLLSYSCLK delay to allow voltage regulator to stabilize
        //
        asm(" MOV    @T,#200 ");
        asm(" RPT    @T \
              || NOP ");
    }
    return (dccStatus);
}

uint16_t BROMDCC_verifySingleShotClock(DCC_Count0ClockSource clk0src,
                                       DCC_Count1ClockSource clk1src, uint16_t dccCounterSeed0,
                                       uint16_t dccCounterSeed1, uint16_t dccValidSeed0)
{
    uint16_t j = dccCounterSeed1;
    uint16_t status;

    //
    // Clear DONE and ERROR flags
    //
    EALLOW;
    HWREGH(DCC0_BASE + DCC_O_STATUS) = 3U;
    EDIS;

    //
    // Disable DCC
    //
    DCC_disableModule(DCC0_BASE);

    //
    // Disable Error Signal
    //
    DCC_disableErrorSignal(DCC0_BASE);

    //
    // Disable Done Signal
    //
    DCC_disableDoneSignal(DCC0_BASE);

    //
    // Configure Clock Source0 to whatever set as a clock source for PLL
    //
    DCC_setCounter0ClkSource(DCC0_BASE, clk0src);

    //
    // Configure Clock Source1 to PLL
    //
    DCC_setCounter1ClkSource(DCC0_BASE, clk1src);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(DCC0_BASE, dccCounterSeed0, dccValidSeed0,
                        dccCounterSeed1);

    //
    // Enable Single Shot mode
    //
    DCC_enableSingleShotMode(DCC0_BASE, DCC_MODE_COUNTER_ZERO);

    //
    // Enable DCC to start counting
    //
    DCC_enableModule(DCC0_BASE);

    //
    // Wait until Error or Done Flag is generated or timeout
    //
    while(((HWREGH(DCC0_BASE + DCC_O_STATUS) &
           (DCC_STATUS_ERR | DCC_STATUS_DONE)) == 0U) && (j != 0U))
    {
		// j is decremented to give enough timeout for HW to complete 
		// the comparision. The result will be determined from the values 
		// in status register.
       j--;
    }

    //
    // Returns NO_ERROR if DCC completes without error
    //
    if((HWREGH(DCC0_BASE + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == DCC_STATUS_DONE)
	{
		status = NO_ERROR;
	}
	else
	{
		status = ERROR;		
	}

    return status;
}

//
// End of File
//
