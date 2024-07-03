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
//!
//! MotorControl SDK
//!
//! FILE:    baudratetune.c
//!
//! TITLE:   Tune Baud Rate Via UART.
//!
//!
//! This implements the process of tuning the UART/SCI baud rate of
//! a C2000 device based on the UART input from another device. As UART does
//! not have a clock signal, reliable communication requires baud rates to
//! be reasonably matched. This example addresses cases where  a clock
//! mismatch between devices is greater than is acceptable for communications,
//! requiring baud compensation between boards. As reliable communication
//! only requires matching the EFFECTIVE baud rate, it does not matter which
//! of the two boards executes the tuning (the board with the less-accurate
//! clock source does not need to be the one to tune; as long as one of the
//! two devices tunes to the other, then proper communication can
//! be established).
//!
//! To tune the baud rate of this device, SCI data (of the desired baud rate)
//! must be sent to this device. The input SCI baud rate must be within the
//! +/- MARGINPERCENT of the TARGETBAUD chosen below. These two variables are
//! defined below, and should be chosen based on the application requirements.
//! Higher MARGINPERCENT will allow more data to be considered "correct" in
//! noisy conditions, and  may decrease accuracy. The TARGETBAUD is what was
//! expected to be the baud rate, but due to clock differences, needs to be
//! tuned for better communication robustness with the other device.
//!
//! NOTE: Lower baud rates have more granularity in register options,
//! and therefore tuning is more affective at these speeds.
//!
//! \b External \b Connections for Control Card \n
//! - SCIA_RX/eCAP1 is on GPIO29, connect to incoming SCI communications
//! - SCIA_TX is on GPIO28, for observation externally
//!
//! \b Watch \b Variables \n
//! - \b avgBaud - Baud rate that was detected and set after tuning
//------------------------------------------------------------------------------

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

#include "user.h"
#include "hal.h"

#include "motor_common.h"
#include "motor1_drive.h"

#include "baudratetune.h"

//
// Defines
//

//
// Globals
//
#if defined(BUADTUNE_EN)
volatile float32_t sampleCountSum;
volatile uint32_t  capCountArr[4];
volatile uint32_t  capCountMin;
volatile uint32_t  capCountMax;

volatile float32_t averageBitWidth;
volatile uint32_t  avgBaudRate;
volatile uint32_t  capCountSum;

volatile uint16_t  flagBaudRateTuned;
volatile uint16_t  stopCaptures;

volatile uint16_t  capCountIter;
volatile uint16_t  capCountFail;
volatile uint16_t  capCountFailRec;
volatile uint16_t  capBufIter;
volatile uint16_t  capCountWait;

//
// Function Prototypes
//
__interrupt void BRT_ecap1ISR(void);

void BRT_initConfiguration(void);
void BRT_tuneBaudRate(void);
uint32_t  BRT_calcAverageBaud(void);

//
#pragma CODE_SECTION(BRT_tuneBaudRate, "sysfuncs");
void BRT_initConfiguration(void)
{
    flagBaudRateTuned = 1;

    stopCaptures = 0;
    capCountIter = 0;
    capBufIter   = 0;
    capCountWait = 0;

    capCountSum  = 0;
    capCountFail = 0;
    capCountFailRec = 0;

    capCountMin = MIN_BITS_WIDTH;
    capCountMax = MAX_BITS_WIDTH;

    // Configure SCIRX pin's GPIO location as eCAP input
    // Fill this GPIO number in based on SCIRX from .syscfg file
    XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT7, GUI_SCI_SCIRX_GPIO);

    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    Interrupt_register(INT_ECAP1, &BRT_ecap1ISR);

    // Initialize basic settings for eCAP monitoring of SCI RX
    //
    // Disable ,clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(ECAP1_BASE,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));

    ECAP_clearInterrupt(ECAP1_BASE,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    // Disable CAP1-CAP4 register loads
    ECAP_disableTimeStampCapture(ECAP1_BASE);

    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 4.
    //    Set polarity of the events to rising, falling, rising, falling edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR7.
    //    Enable eCAP module.
    //    Enable interrupt.
    ECAP_stopCounter(ECAP1_BASE);
    ECAP_enableCaptureMode(ECAP1_BASE);

    ECAP_setCaptureMode(ECAP1_BASE, ECAP_ONE_SHOT_CAPTURE_MODE, ECAP_EVENT_4);

//    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_3, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_4);

    ECAP_selectECAPInput(ECAP1_BASE, ECAP_INPUT_INPUTXBAR7);

    ECAP_enableLoadCounter(ECAP1_BASE);
    ECAP_setSyncOutMode(ECAP1_BASE, ECAP_SYNC_OUT_DISABLED);
    ECAP_startCounter(ECAP1_BASE);
    ECAP_enableTimeStampCapture(ECAP1_BASE);
    ECAP_reArm(ECAP1_BASE);

    ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);

    // Enable interrupts required for this example
    Interrupt_enable(INT_ECAP1);

    return;
}

#pragma CODE_SECTION(BRT_tuneBaudRate, "sysfuncs");
void BRT_tuneBaudRate(void)
{
    // Loop forever. Suspend or place breakpoints to observe the buffers
    // sample count is filled, begin tuning
    if((stopCaptures == 1) && (flagBaudRateTuned == 1))
    {
        // Get an average baud rate from the array of samples
        avgBaudRate = BRT_calcAverageBaud();

        // if the baud function returns the error code '0', then flag an
        // error
        if(avgBaudRate != 0)
        {
            // Update the device's baud rate to match the measured baud rate
            SCI_setBaud(GUI_SCI_BASE, DEVICE_LSPCLK_FREQ, avgBaudRate);
            SCI_resetRxFIFO(GUI_SCI_BASE);

            stopCaptures = 0;
            capCountSum = 0;
            capBufIter = 0;

            flagBaudRateTuned = 2;
            capCountWait = 0;
        }

        // wait received data from SCI
    }

    // If continuing, reset the array iterator and unlock the ISR for
    // new captures
    if(flagBaudRateTuned == 3)
    {
        capCountWait++;

        if(capCountWait >= 10000)
        {
            flagBaudRateTuned = 4;
        }

        // Disable interrupts required for this example
        Interrupt_disable(INT_ECAP1);
    }
    else if(flagBaudRateTuned == 0)
    {
        BRT_initConfiguration();
    }

    return;
}


#pragma CODE_SECTION(BRT_initECAP, "sysfuncs");
void BRT_initECAP(void)
{

}

#pragma CODE_SECTION(BRT_ecap1ISR, "sysfuncs");
__interrupt void BRT_ecap1ISR(void)
{
    if(stopCaptures == 0)
    {
        // Get the capture counts, interrupt every 4. Can be 1-bit or more
        // wide. Add one to account for partial eCAP counts at higher baud
        // rates (for example: count = 40, but if had higher resolution, this
        // would be 40.5)
        capCountArr[0] = ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_1);
        capCountArr[1] = ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_2);
        capCountArr[2] = ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_3);
        capCountArr[3] = ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_4);

        // Add samples to a buffer. Get average baud and tune if buffer filled.
        capCountIter = 0;

        for(capCountIter = 0; capCountIter < 4; capCountIter++)
        {
            // if we still have samples left to capture, add it to the samples buffer
//            if ((capCountArr[capCountIter] > MIN_BITS_WIDTH)
//                    && (capCountArr[capCountIter] < MIN_BITS_WIDTH))
            if ((capCountArr[capCountIter] > capCountMin)
                    && (capCountArr[capCountIter] < capCountMax))
            {
                capCountSum += capCountArr[capCountIter];
                capBufIter++;
            }
            else
            {
                capCountFail++;

                if(capCountFail > FAILEDSAMPLES)
                {
                    capCountFailRec = capCountFail;
                    capCountFail = 0;
                    capBufIter = 0;
                    capCountSum = 0;
                }
            }

            // all samples were received, break to begin tuning
            if(capBufIter >= NUMSAMPLES)
            {
                stopCaptures = 1;
                capCountFail = 0;
                capCountFailRec = capCountFail;
                break;
            }
        }
    }

    // Clear interrupt flags for more interrupts.
    ECAP_clearInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
    ECAP_clearGlobalInterrupt(ECAP1_BASE);

    // Start eCAP
    ECAP_reArm(ECAP1_BASE);

    // Acknowledge the group interrupt for more interrupts.
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
}

//
// FUNCTION:    getAverageBaud
// PURPOSE:     get the average baud rate of the array
// INPUTS:      array of pulse widths (in number eCAP counts),
//              size of array, and target baud rate
// RETURNS:     average baud rate
#pragma CODE_SECTION(BRT_calcAverageBaud, "sysfuncs");
uint32_t BRT_calcAverageBaud(void)
{
    // convert 2-bit width, 3-bit width, and so on to 1-bit width values by
    // dividing, and average these values. skip unrelated values
    sampleCountSum = (float32_t)capCountSum;
    averageBitWidth = sampleCountSum * ONE_OVER_SAMPLES;

    // get the rounded baud rate from the average number of clocks and the
    // sysclk frequency
    return((uint32_t)(((float32_t)DEVICE_SYSCLK_FREQ/(float32_t)averageBitWidth) + 0.5f));
}
#endif  // BUADTUNE_EN
