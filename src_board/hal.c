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
//! \file   solutions/universal_motorcontrol_lab/f280013x/drivers/hal_all.c
//! \brief  Contains the various functions related to the HAL object
//!
//
//------------------------------------------------------------------------------

// if PFC is enable, one ADC is for PFC, another ADC is for motor drive
// if PFC is disable, all of the two ADCs are for motor drive

//
// the includes
//
#include "user.h"


//
// drivers
//

// modules

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalogIF.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals
HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
HAL_Obj       hal;            //!< the hardware abstraction layer object
#pragma DATA_SECTION(halHandle, "hal_data");
#pragma DATA_SECTION(hal, "hal_data");

// **************************************************************************
// the functions

void HAL_disableGlobalInts(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableMaster();

    return;
} // end of HAL_disableGlobalInts() function

void HAL_disableWdog(HAL_Handle halHandle)
{
    // disable watchdog
    SysCtl_disableWatchdog();

    return;
} // end of HAL_disableWdog() function

void HAL_enableADCInts(HAL_Handle handle)
{
    // enable the ADC interrupts for motor_1
    ADC_enableInterrupt(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);
    return;
} // end of HAL_enableCtrlInts() function


void HAL_enableCtrlInts(HAL_Handle handle)
{
    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(MTR1_PIE_INT_NUM);    // motor_1

    return;
} // end of HAL_enableCtrlInts() function

HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    // Two ADC modules in this device
    // initialize the ADC handles
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCC_BASE;

    // initialize the ADC results
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCCRESULT_BASE;

    // No DAC modules in this device
    // No CLA module in this device

    // initialize SCI handle
    obj->sciHandle[0] = SCIA_BASE;          //!< the SCIA handle for PCGUI or SFRA
    obj->sciHandle[1] = SCIC_BASE;          //!< the SCIC handle for system

    // initialize SPI handle
    obj->spiHandle[0] = SPIA_BASE;          //!< the SPIA handle for DAC128S or AI Sensor

    // initialize timer handles
    obj->timerHandle[TIME_BASE_TIMER] = TIME_BASE_TIMER_BASE;
    obj->timerHandle[CPU_USAGE_TIMER] = CPU_USAGE_TIMER_BASE;
    obj->timerHandle[SYS_RTOS_TIMER]  = SYS_RTOS_TIMER_BASE;

#if defined(EPWMDAC_MODE)
#if defined(WMINVBRD_REV1P0) || defined(HVMTRPFC_REV1P1) || \
    defined(BSXL8323RH_REVB) || defined(DRV8329AEVM_REVA)
    // initialize pwmdac handles
    obj->pwmDACHandle[0] = EPWMDAC1_BASE;
    obj->pwmDACHandle[1] = EPWMDAC2_BASE;
    obj->pwmDACHandle[2] = EPWMDAC3_BASE;
// ( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#error EPWMDAC is not supported on this kit!
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#endif  // EPWMDAC_MODE


    return(handle);
} // end of HAL_init() function

HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_MTR_Handle)pMemory;

    // assign the object
    obj = (HAL_MTR_Obj *)handle;

    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
    obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
    obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    obj->cmpssHandle[0] = MTR1_CMPSS_IDC_BASE;  //!< the CMPSS handle
#else   // !(MOTOR1_DCLINKSS)   // 2/3 shunt
    obj->cmpssHandle[0] = MTR1_CMPSS_U_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[1] = MTR1_CMPSS_V_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[2] = MTR1_CMPSS_W_BASE;    //!< the CMPSS handle
#endif  // !(MOTOR1_DCLINKSS)   // 2/3 shunt

#if defined(CMD_CAP_EN)
    obj->capHandle = MTR1_CAP_FREQ_BASE;        //!< the CAP handle
#endif // CMD_CAP_EN

    // No PGA modules in this device

    // Assign gateEnableGPIO
#if defined(HVMTRPFC_REV1P1)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    // HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateGainGPIO = MTR1_GATE_GAIN_GPIO;
    // BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateSleepGPIO  = MTR1_GATE_nSLEEP_GPIO;
    // DRV8329AEVM_REVA
#endif // HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA

    obj->motorNum = MTR_1;

    return(handle);
} // end of HAL_MTR1_init() function

void HAL_setParams(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableMaster();

    // setup the GPIOs
    HAL_setupGPIOs(handle);

    // setup the ADCs
    HAL_setupADCs(handle);

#if defined(DATALOGF4_EN) || defined(DATALOGF2_EN)
    // setup the DMA
    HAL_setupDMA();
#endif  // DATALOGF4_EN || DATALOGF2_EN

#if !defined(_HSWFREQ_EN)
    // setup the scia for GUI
    HAL_setupSCIA(handle);

    // setup the scic for system
    HAL_setupSCIC(handle);

    // setup the i2c
    HAL_setupI2CA(handle);
#endif // !_HSWFREQ_EN

#if defined(EPWMDAC_MODE)
    // setup the PWM DACs
    HAL_setupPWMDACs(handle, USER_SYSTEM_FREQ_MHz);
#endif  // EPWMDAC_MODE

    return;
} // end of HAL_setParams() function

void HAL_MTR_setParams(HAL_MTR_Handle handle)
{
    // setup the PWMs
    HAL_setupPWMs(handle);

    // setup the CMPSSs
    HAL_setupCMPSSs(handle);

    // setup faults
    HAL_setupMtrFaults(handle);

#if defined(CMD_CAP_EN)
    // setup the CAPs
    HAL_setupCAPs(handle);
#endif  // CMD_CAP_EN


    // disable the PWM
    HAL_disablePWM(handle);

    // Setup Gate Enable
#if defined(HVMTRPFC_REV1P1)
    // turn on the Kit if present
    HAL_enableDRV(handle);
    //HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
    // turn on the DRV8323RH if present
    HAL_enableDRV(handle);
    // BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
    // turn on the DRV8329A if present
    HAL_enableDRV(handle);
    // DRV8329AEVM_REVA
#endif   // HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA

    return;
} // end of HAL_MTR_setParams() function

void HAL_setupADCs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    EALLOW;

    ASysCtl_enableTemperatureSensor();

    SysCtl_delay(500U);

    // Set the analog voltage reference selection to internal.
    ASysCtl_setAnalogReferenceInternal( ASYSCTL_VREFHI );

    // Set the internal analog voltage reference selection to 1.65V.
    ASysCtl_setAnalogReference1P65( ASYSCTL_VREFHI );


    SysCtl_delay(100U);

#if defined(WMINVBRD_REV1P0)
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    // WMINVBRD_REV1P0
#elif defined(HVMTRPFC_REV1P1)
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    // HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB) || defined(DRV8329AEVM_REVA)
    // LAUNCHXL-F2800137 based kits
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    // BSXL8323RH_REVB | DRV8329AEVM_REVA
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#error Select the right clock of ADC reference for the board
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)

    SysCtl_delay(100U);

#if defined(CPU_FRQ_120MHZ)
    // Set main clock scaling factor (50MHz max clock for the ADC module)
    ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);
#elif defined(CPU_FRQ_100MHZ)
    // Set main clock scaling factor (50MHz max clock for the ADC module)
    ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);
#endif  // CPU_FRQ_100MHZ

    // set the ADC interrupt pulse generation to end of conversion
    ADC_setInterruptPulseMode(obj->adcHandle[0], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[1], ADC_PULSE_END_OF_CONV);

    // set priority of SOCs
    ADC_setSOCPriority(obj->adcHandle[0], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[1], ADC_PRI_ALL_HIPRI);

    // enable the ADCs
    ADC_enableConverter(obj->adcHandle[0]);
    ADC_enableConverter(obj->adcHandle[1]);

    // delay to allow ADCs to power up
    SysCtl_delay(1000U);

    //-------------------------------------------------------------------------
    // configure the SOCs for M1
#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // configure the interrupt sources
    // Interrupt for motor 1
    ADC_setInterruptSource(MTR1_ADC_INT_BASE,
                           MTR1_ADC_INT_NUM, MTR1_ADC_INT_SOC);

    // Idc 1st
    ADC_setupSOC(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_SOC_NUM, MTR1_IDC1_TRIGGER_SOC,
                 MTR1_IDC1_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, MTR1_IDC1_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);

    // Idc 2nd
    ADC_setupSOC(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_SOC_NUM, MTR1_IDC2_TRIGGER_SOC,
                 MTR1_IDC2_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, MTR1_IDC2_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);

    // Idc 3rd
    ADC_setupSOC(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_SOC_NUM, MTR1_IDC3_TRIGGER_SOC,
                 MTR1_IDC3_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, MTR1_IDC3_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, 0);

    // Idc 4th
    ADC_setupSOC(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_SOC_NUM, MTR1_IDC4_TRIGGER_SOC,
                 MTR1_IDC4_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, MTR1_IDC4_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, 0);
#else   // !(MOTOR1_DCLINKSS)       // 2/3 shunt
    // configure the interrupt sources
    // Interrupt for motor 1
    ADC_setInterruptSource(MTR1_ADC_INT_BASE,
                           MTR1_ADC_INT_NUM, MTR1_ADC_INT_SOC);
    // ISEN_A_M1
    ADC_setupSOC(MTR1_IU_ADC_BASE, MTR1_IU_ADC_SOC_NUM, MTR1_ADC_TRIGGER_CURR,
                 MTR1_IU_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, MTR1_IU_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, 0);

    // ISEN_B_M1
    ADC_setupSOC(MTR1_IV_ADC_BASE, MTR1_IV_ADC_SOC_NUM, MTR1_ADC_TRIGGER_CURR,
                 MTR1_IV_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, MTR1_IV_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, 0);

    // ISEN_C_M1
    ADC_setupSOC(MTR1_IW_ADC_BASE, MTR1_IW_ADC_SOC_NUM, MTR1_ADC_TRIGGER_CURR,
                 MTR1_IW_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, MTR1_IW_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, 0);
#endif   // !(MOTOR1_DCLINKSS)      // 2/3 shunt

#if defined(MOTOR1_FAST)
    // VSEN_A_M1
    ADC_setupSOC(MTR1_VU_ADC_BASE, MTR1_VU_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_VU_ADC_CH_NUM, MTR1_ADC_V_SAMPLEWINDOW);

    // VSEN_B_M1
    ADC_setupSOC(MTR1_VV_ADC_BASE, MTR1_VV_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_VV_ADC_CH_NUM, MTR1_ADC_V_SAMPLEWINDOW);

    // VSEN_C_M1
    ADC_setupSOC(MTR1_VW_ADC_BASE, MTR1_VW_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_VW_ADC_CH_NUM, MTR1_ADC_V_SAMPLEWINDOW);
#endif  // MOTOR1_FAST

    // VSEN_DCBUS_M1-->Trig Interrupt
    ADC_setupSOC(MTR1_VDC_ADC_BASE, MTR1_VDC_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_VDC_ADC_CH_NUM, MTR1_ADC_V_SAMPLEWINDOW);

#if defined(TEMP_SENSE)
    // Temp_Motor
    ADC_setupSOC(MTR1_TMR_ADC_BASE, MTR1_TMR_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_TMR_ADC_CH_NUM, MTR1_ADC_T_SAMPLEWINDOW);

    // Temp_Module
    ADC_setupSOC(MTR1_TPM_ADC_BASE, MTR1_TPM_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_TPM_ADC_CH_NUM, MTR1_ADC_T_SAMPLEWINDOW);
#endif  // TEMP_SENSE

#if defined(CMD_POT_EN)
    // POT_M1
    ADC_setupSOC(MTR1_POT_ADC_BASE, MTR1_POT_ADC_SOC_NUM, MTR1_ADC_TRIGGER_VOLT,
                 MTR1_POT_ADC_CH_NUM, MTR1_ADC_T_SAMPLEWINDOW);
#endif  // CMD_POT_EN

#if defined(SAFETY_ENABLE)
    // Device Temperature sensor
    ADC_setupSOC(MTR1_TDS_ADC_BASE, MTR1_TDS_ADC_SOC_NUM, MTR1_ADC_TRIGGER_SAFE,
                 MTR1_TDS_ADC_CH_NUM, MTR1_ADC_T_SAMPLEWINDOW);

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // Idc offset
    ADC_setupSOC(MTR1_ODC_ADC_BASE, MTR1_ODC_ADC_SOC_NUM, MTR1_ADC_TRIGGER_SAFE,
                 MTR1_ODC_ADC_CH_NUM, MTR1_ADC_S_SAMPLEWINDOW);
#else   // !(MOTOR1_DCLINKSS)       // 2/3 shunt
    // 1.65V offset U phase
    ADC_setupSOC(MTR1_OSU_ADC_BASE, MTR1_OSU_ADC_SOC_NUM, MTR1_ADC_TRIGGER_SAFE,
                 MTR1_OSU_ADC_CH_NUM, MTR1_ADC_S_SAMPLEWINDOW);

    // 1.65V offset V phase
    ADC_setupSOC(MTR1_OSV_ADC_BASE, MTR1_OSV_ADC_SOC_NUM, MTR1_ADC_TRIGGER_SAFE,
                 MTR1_OSV_ADC_CH_NUM, MTR1_ADC_S_SAMPLEWINDOW);

    // 1.65V offset W phase
    ADC_setupSOC(MTR1_OSW_ADC_BASE, MTR1_OSW_ADC_SOC_NUM, MTR1_ADC_TRIGGER_SAFE,
                 MTR1_OSW_ADC_CH_NUM, MTR1_ADC_S_SAMPLEWINDOW);
#endif  // !(MOTOR1_DCLINKSS)       // 2/3 shunt
#endif  // SAFETY_ENABLE

    EDIS;

    return;
} // end of HAL_setupADCs() function

#if defined(CMD_CAP_EN)
void HAL_setupCAPs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    // Disable ,clear all capture flags and interrupts
    ECAP_disableInterrupt(obj->capHandle,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));

    ECAP_clearInterrupt(obj->capHandle,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    // Disable CAP1-CAP4 register loads
    ECAP_disableTimeStampCapture(obj->capHandle);

    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 3.
    //    Set polarity of the events to rising, falling, rising edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR4/5/6.
    //    Enable eCAP module.
    //    Enable interrupt.
    ECAP_stopCounter(obj->capHandle);
    ECAP_enableCaptureMode(obj->capHandle);

    ECAP_setCaptureMode(obj->capHandle, ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_4);

    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_3, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_4);

    ECAP_enableLoadCounter(obj->capHandle);
    ECAP_setSyncOutMode(obj->capHandle, ECAP_SYNC_OUT_SYNCI);
    ECAP_startCounter(obj->capHandle);
    ECAP_enableTimeStampCapture(obj->capHandle);
    ECAP_reArm(obj->capHandle);

    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_CAP_FREQ_XBAR, MTR1_CAP_FREQ_GPIO);

    ECAP_selectECAPInput(obj->capHandle, MTR1_CAP_FREQ_INSEL);

    return;
}

void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    ECAP_setAPWMPeriod(obj->capHandle, 0);
    ECAP_setAPWMCompare(obj->capHandle, 0x01FFFFFF);
    ECAP_setAPWMShadowPeriod(obj->capHandle, 0x01FFFFFF);
    ECAP_setAPWMShadowCompare(obj->capHandle, 0x01FFFFFF);

    return;
}
#endif  // CMD_CAP_EN

// HAL_setupCMPSSs
void HAL_setupCMPSSs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_DCLINKSS)    // Single shunt
#if defined(WMINVBRD_REV1P0)
    uint16_t cmpsaDACH = MTR1_CMPSS_DACH_VALUE;

    ASysCtl_selectCMPHPMux(MTR1_IDC_CMPHP_SEL, MTR1_IDC_CMPHP_MUX);

    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[0]);

    // NEG signal from DAC for COMP-H
    CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC | CMPSS_OR_ASYNC_OUT_W_FILT);
//    CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);

    // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
    CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                            CMPSS_TRIP_FILTER |
                            CMPSS_TRIPOUT_FILTER);



    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    CMPSS_configFilterHigh(obj->cmpssHandle[0], 31, 15, 14);
    CMPSS_initFilterHigh(obj->cmpssHandle[0]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[0],
               CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-H to allowed MAX +ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], cmpsaDACH);

    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    // WMINVBRD_REV1P0
#elif defined(DRV8329AEVM_REVA)
    uint16_t cmpsaDACH = MTR1_CMPSS_DACH_VALUE;

    ASysCtl_selectCMPHPMux(MTR1_IDC_CMPHP_SEL, MTR1_IDC_CMPHP_MUX);

    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[0]);

    // NEG signal from DAC for COMP-H
    CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

    // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
    CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                            CMPSS_TRIP_FILTER |
                            CMPSS_TRIPOUT_FILTER);

    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    CMPSS_configFilterHigh(obj->cmpssHandle[0], 8, 16, 8);
    CMPSS_initFilterHigh(obj->cmpssHandle[0]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[0],
               CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-H to allowed MAX +ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], cmpsaDACH);

    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    // DRV8329AEVM_REVA
#elif defined(HVMTRPFC_REV1P1)
    uint16_t cmpsaDACL = MTR1_CMPSS_DACL_VALUE;

    ASysCtl_selectCMPLPMux(MTR1_IDC_CMPLP_SEL, MTR1_IDC_CMPLP_MUX);

    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[0]);

    // NEG signal from DAC for COMP-L
    CMPSS_configLowComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

    // Configure the output signals. Both CTRIPH/L and CTRIPOUTH/L will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
    CMPSS_configOutputsLow(obj->cmpssHandle[0],
                           CMPSS_TRIP_FILTER |
                           CMPSS_TRIPOUT_FILTER |
                           CMPSS_INV_INVERTED);

    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    // Initialize the filter logic and start filtering
    CMPSS_configFilterLow(obj->cmpssHandle[0], 32, 32, 30);
    CMPSS_initFilterLow(obj->cmpssHandle[0]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[0],
               CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueLow(obj->cmpssHandle[0], cmpsaDACL);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
    // HVMTRPFC_REV1P1
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | DRV8329AEVM_REVA)
#error Not select a right board for this project
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | DRV8329AEVM_REVA)
#else   // !MOTOR1_DCLINKSS     // 2/3 shunt
    // Refer to the Table 9-2 in Chapter 9 of TMS320F28004x
    // Technical Reference Manual (SPRUI33B), to configure the ePWM X-Bar
    uint16_t cmpsaDACH = MTR1_CMPSS_DACH_VALUE;
    uint16_t cmpsaDACL = MTR1_CMPSS_DACL_VALUE;
    uint16_t i;

#if defined(WMINVBRD_REV1P0) || defined(HVMTRPFC_REV1P1)
    ASysCtl_selectCMPHPMux(MTR1_IU_CMPHP_SEL, MTR1_IU_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IU_CMPLP_SEL, MTR1_IU_CMPLP_MUX);

    ASysCtl_selectCMPHPMux(MTR1_IV_CMPHP_SEL, MTR1_IV_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IV_CMPLP_SEL, MTR1_IV_CMPLP_MUX);

    ASysCtl_selectCMPHPMux(MTR1_IW_CMPHP_SEL, MTR1_IW_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IW_CMPLP_SEL, MTR1_IW_CMPLP_MUX);

    for(i = 0; i < 3; i++)
    {
        //----- U/V/W Phase ------------------------------------------------------------
        // Enable CMPSS and configure the negative input signal to come from the DAC
        CMPSS_enableModule(obj->cmpssHandle[0]);

        // NEG signal from DAC for COMP-H
        CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

        // NEG signal from DAC for COMP-L
        CMPSS_configLowComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

        // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
        // the asynchronous comparator output.
        // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
        CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                                CMPSS_TRIP_FILTER |
                                CMPSS_TRIPOUT_FILTER);

        // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
        CMPSS_configOutputsLow(obj->cmpssHandle[0],
                               CMPSS_TRIP_FILTER |
                               CMPSS_TRIPOUT_FILTER |
                               CMPSS_INV_INVERTED);

        // Configure digital filter. For this example, the maxiumum values will be
        // used for the clock prescale, sample window size, and threshold.
        CMPSS_configFilterHigh(obj->cmpssHandle[0], 32, 32, 30);
        CMPSS_initFilterHigh(obj->cmpssHandle[0]);

        // Initialize the filter logic and start filtering
        CMPSS_configFilterLow(obj->cmpssHandle[0], 32, 32, 30);
        CMPSS_initFilterLow(obj->cmpssHandle[0]);

        // Set up COMPHYSCTL register
        // COMP hysteresis set to 2x typical value
        CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

        // Use VDDA as the reference for the DAC and set DAC value to midpoint for
        // arbitrary reference
        CMPSS_configDAC(obj->cmpssHandle[0],
                        CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

        // Set DAC-H to allowed MAX +ve current
        CMPSS_setDACValueHigh(obj->cmpssHandle[0], cmpsaDACH);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[0], cmpsaDACL);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
    }
    // WMINVBRD_REV1P0 | HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
    ASysCtl_selectCMPHPMux(MTR1_IU_CMPHP_SEL, MTR1_IU_CMPHP_MUX);

    ASysCtl_selectCMPHPMux(MTR1_IV_CMPHP_SEL, MTR1_IV_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IV_CMPLP_SEL, MTR1_IV_CMPLP_MUX);

    ASysCtl_selectCMPLPMux(MTR1_IW_CMPLP_SEL, MTR1_IW_CMPLP_MUX);

    for(i = 0; i < 3; i++)
    {
        //----- U/V/W Phase ------------------------------------------------------------
        // Enable CMPSS and configure the negative input signal to come from the DAC
        CMPSS_enableModule(obj->cmpssHandle[0]);

        // NEG signal from DAC for COMP-H
        CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

        // NEG signal from DAC for COMP-L
        CMPSS_configLowComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

        // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
        // the asynchronous comparator output.
        // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
        CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                                CMPSS_TRIP_FILTER |
                                CMPSS_TRIPOUT_FILTER);

        // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
        CMPSS_configOutputsLow(obj->cmpssHandle[0],
                               CMPSS_TRIP_FILTER |
                               CMPSS_TRIPOUT_FILTER |
                               CMPSS_INV_INVERTED);

        // Configure digital filter. For this example, the maxiumum values will be
        // used for the clock prescale, sample window size, and threshold.
        CMPSS_configFilterHigh(obj->cmpssHandle[0], 32, 32, 30);
        CMPSS_initFilterHigh(obj->cmpssHandle[0]);

        // Initialize the filter logic and start filtering
        CMPSS_configFilterLow(obj->cmpssHandle[0], 32, 32, 30);
        CMPSS_initFilterLow(obj->cmpssHandle[0]);

        // Set up COMPHYSCTL register
        // COMP hysteresis set to 2x typical value
        CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

        // Use VDDA as the reference for the DAC and set DAC value to midpoint for
        // arbitrary reference
        CMPSS_configDAC(obj->cmpssHandle[0],
                        CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

        // Set DAC-H to allowed MAX +ve current
        CMPSS_setDACValueHigh(obj->cmpssHandle[0], cmpsaDACH);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[0], cmpsaDACL);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
    }
    // BSXL8323RH_REVB
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB)
#error Not select a right board for this project
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB)
#endif  // !MOTOR1_DCLINKSS

    return;
} // end of HAL_setupCMPSSs() function

// HAL_setupGate & HAL_enableDRV
#if defined(HVMTRPFC_REV1P1)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to low for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 0);

    return;
} // HAL_setupGate() function
// HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to high for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 1);

    return;
} // HAL_setupGate() function
// BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set nSLEEP to low for clear the fault
    GPIO_writePin(obj->gateSleepGPIO, 0);

    DEVICE_DELAY_US(2.0f);      // delay 2us

    // Set EN_GATE to low for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 0);

    DEVICE_DELAY_US(2.0f);      // 2.0us

    // Set nSLEEP to high for wake the device
    GPIO_writePin(obj->gateSleepGPIO, 1);

    DEVICE_DELAY_US(2.0f);      // delay 2us

    // Set EN_GATE to low for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 0);

    return;
} // HAL_setupGate() function
// DRV8329AEVM_REVA
#endif   // HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA


void HAL_setupMtrFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t cnt;

    // Configure TRIP 7 to OR the High and Low trips from both
    // comparator 5, 3 & 1, clear everything first
    EALLOW;
    HWREG(XBAR_EPWM_CFG_REG_BASE + MTR1_XBAR_TRIP_ADDRL) = 0;
    HWREG(XBAR_EPWM_CFG_REG_BASE + MTR1_XBAR_TRIP_ADDRH) = 0;
    EDIS;

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // Configure TRIP7 to be CTRIP5H and CTRIP5L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IDC_XBAR_EPWM_MUX);

    // Disable all the mux first
    XBAR_disableEPWMMux(MTR1_XBAR_TRIP, 0xFFFF);

    // Enable Mux 0  OR Mux 4 to generate TRIP
    XBAR_enableEPWMMux(MTR1_XBAR_TRIP, MTR1_IDC_XBAR_MUX);
#else   // !MOTOR1_DCLINKSS     // 2/3 shunt
    // Configure TRIP7 to be CTRIP5H and CTRIP5L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IU_XBAR_EPWM_MUX);

    // Configure TRIP7 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IV_XBAR_EPWM_MUX);

    // Configure TRIP7 to be CTRIP3H and CTRIP3L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IW_XBAR_EPWM_MUX);

    // Disable all the mux first
    XBAR_disableEPWMMux(MTR1_XBAR_TRIP, 0xFFFF);

    // Enable Mux 0  OR Mux 4 to generate TRIP
    XBAR_enableEPWMMux(MTR1_XBAR_TRIP, MTR1_IU_XBAR_MUX |
                                       MTR1_IV_XBAR_MUX | MTR1_IW_XBAR_MUX);
#endif  // !MOTOR1_DCLINKSS     // 2/3 shunt

    // Configure Trip Mechanism for the Motor control software
    // -Cycle by cycle trip on CPU halt
    // -One shot fault trip zone
    // These trips need to be repeated for EPWM1 ,2 & 3

    // configure the input x bar for TZ2 to GPIO, where Over Current is connected
    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_XBAR_INPUT1, MTR1_PM_nFAULT_GPIO);
    XBAR_lockInput(INPUTXBAR_BASE, MTR1_XBAR_INPUT1);

    for(cnt=0; cnt<3; cnt++)
    {
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], MTR1_TZ_OSHT1);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_CBC6);

        //enable DC TRIP combinational input
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                              MTR1_DCTRIPIN, EPWM_DC_TYPE_DCAH);

        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                              MTR1_DCTRIPIN, EPWM_DC_TYPE_DCBH);

        // Trigger event when DCAH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_A1,
                                                     EPWM_TZ_EVENT_DCXH_HIGH);

        // Trigger event when DCBH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_B1,
                                                     EPWM_TZ_EVENT_DCXL_HIGH);

        // Configure the DCA path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_A,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        // Configure the DCB path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_B,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_A,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_B,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        // Enable DCA as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCAEVT1);

        // Enable DCB as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCBEVT1);

        // What do we want the OST/CBC events to do?
        // TZA events can force EPWMxA
        // TZB events can force EPWMxB
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZA,
                               EPWM_TZ_ACTION_LOW);

        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZB,
                               EPWM_TZ_ACTION_LOW);
    }

#if defined(MOTOR1_DCLINKSS)    // Single shunt
#if defined(WMINVBRD_REV1P0)
    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);
    // WMINVBRD_REV1P0
#elif defined(DRV8329AEVM_REVA)
    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    // DRV8329AEVM_REVA
#elif defined(HVMTRPFC_REV1P1)
    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
    // HVMTRPFC_REV1P1
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | TIDSMPFC_REV3P2 | DRV8329AEVM_REVA)
#error Not select a right board for this project
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | TIDSMPFC_REV3P2 | DRV8329AEVM_REVA)
#else   // ! MOTOR1_DCLINKSS    // 2/3 shunt
#if defined(HVMTRPFC_REV1P1) || defined(WMINVBRD_REV1P0)
    for(cnt=0; cnt<3; cnt++)
    {
        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], HAL_TZFLAG_INTERRUPT_ALL);
    }
    // (HVMTRPFC_REV1P1 | WMINVBRD_REV1P0)
#elif defined(BSXL8323RH_REVB)
    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
    // BSXL8323RH_REVB
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB)
#error Not Select a Right Board
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB)
#endif  // !MOTOR1_DCLINKSS     // 2/3 shunt

    // Clear any spurious fault
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_setupMtrFaults() function

void HAL_setupGPIOs(HAL_Handle handle)
{
//------------------------------------------------------------------------------
#if defined(WMINVBRD_REV1P0)
#if defined(EPWMDAC_MODE)
    // GPIO0->EPWM1A->PWMDAC1
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_writePin(0, 0);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->PWMDAC2
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_writePin(1, 0);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#elif !defined(DAC128S_ENABLE) && defined(AIC_ENABLE)
    // GPIO0->SPIA_STE->AI_SENSOR_STE
    GPIO_setPinConfig(GPIO_0_SPIA_STE);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->SPIA_SOMI->AI_SENSOR_SDO
    GPIO_setPinConfig(GPIO_1_SPIA_SOMI);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#else   // !AIS_ENABLE
    // GPIO0->Reserve
    GPIO_setPinConfig(GPIO_0_GPIO0);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->Reserve
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#endif  // !AIS_ENABLE

    // GPIO2->EPWM2A->M1_UH
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_writePin(2, 0);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_UL
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_writePin(3, 0);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M1_VH
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_writePin(4, 0);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->EPWM3B->M1_VL
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_writePin(5, 0);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->EPWM4A->M1_WH
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_writePin(6, 0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M1_WL
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_writePin(7, 0);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO11->Reserve, NA on this device
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

#if !defined(VERSION_1P0)     // VERSION_1P1
#if !defined(DAC128S_ENABLE) && defined(AIC_ENABLE)
    // GPIO162>SPIA_CLK->AI_SENSOR_SCLK
    GPIO_setPinConfig(GPIO_12_SPIA_CLK);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);
#else   // DAC128S_ENABLE | !AIC_ENABLE
    // GPIO12->GPIO12->EPWMDAC
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_writePin(12, 0);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);
#endif  // !AIS_ENABLE

    // GPIO13->GPIO13->Power Relay
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_writePin(13, 0);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#else   // VERSION_1P0
    // GPIO12->GPIO12->Power Relay
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_writePin(12, 0);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->Reserve
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_writePin(13, 0);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#endif   // VERSION_1P0

#if !defined(VERSION_1P0)  // VERSION_1P1
#if !defined(DAC128S_ENABLE) && defined(AIC_ENABLE)
    // GPIO16->SPIA_SIMO->AI_SENSOR_SDI
    GPIO_setPinConfig(GPIO_16_SPIA_SIMO);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);
#else   // DAC128S_ENABLE | !AIC_ENABLE
    // GPIO16->Reserve
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);
#endif  // !AIS_ENABLE
#else     // VERSION_1P0
#if defined(DAC128S_ENABLE) && !defined(AIC_ENABLE)
    // GPIO16->SPIA_SIMO->AI_SENSOR_SDI
    GPIO_setPinConfig(GPIO_16_SPIA_SIMO);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);
#else   // !DAC128S_ENABLE | AIS_ENABLE
    // GPIO16->Reserve
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);
#endif  // !AIS_ENABLE
#endif    // VERSION_1P0

#if !defined(VERSION_1P0)  // VERSION_1P1
    // GPIO18->Reserve for external crystal
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
#else     // VERSION_1P0
#if defined(EPWMDAC_MODE)
    // GPIO18->EPWM6_A->DAC3
    GPIO_setPinConfig(GPIO_18_EPWM6_A);
    GPIO_writePin(18, 0);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
#elif defined(AIC_ENABLE)
    // GPIO18->SPIA_CLK->AI_SENSOR_SCLK
    GPIO_setPinConfig(GPIO_18_SPIA_CLK);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
#else   // !AIC_ENABLE
    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
#endif  // !AIC_ENABLE
#endif  // VERSION_1P0

    // GPIO19->Reserve for Oscillator external resistor
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

#if !defined(VERSION_1P0)  // VERSION_1P1
#if defined(DAC128S_ENABLE) && !defined(AIC_ENABLE)
    // GPIO24->SPIA_SIMO for AI Sensor (SCS)
    GPIO_setPinConfig(GPIO_24_SPIA_SIMO);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);
#else   // !DAC128S_ENABLE | AIC_ENABLE
    // GPIO24->Reserve
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);
#endif  // !DAC128S_ENABLE | AIC_ENABLE
#else   // VERSION_1P0
#if !defined(DAC128S_ENABLE) && defined(AIC_ENABLE)
    // GPIO24->SPIA_STE for AI Sensor (SCS)
    GPIO_setPinConfig(GPIO_24_SPIA_STE);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);
#else   // DAC128S_ENABLE | !AIC_ENABLE
    // GPIO24->Reserve
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);
#endif  // DAC128S_ENABLE | !AIC_ENABLE
#endif  // VERSION_1P0

#if defined(DAC128S_SPIA) && defined(DAC128S_ENABLE)
    // GPIO28->SPIA_CLK->DAC_SCLK
    GPIO_setPinConfig(GPIO_28_SPIA_CLK);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->SPIA_STE->DAC_SCS
    GPIO_setPinConfig(GPIO_29_SPIA_STE);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);
#elif defined(GUI_SCI_EN) || defined(SFRA_SCI_EN)
    // GPIO28->SCIA_RX->GUI/SFRA_RX
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->SCIA_TX->GUI/SFRA_TX
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);
#else   //!GUI_SCI_EN & SFRA_SCI_EN
    // GPIO28->Reserve*
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->Reserve*
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);
#endif   //!GUI_SCI_EN & SFRA_SCI_EN

    // GPIO32->LED1C
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_writePin(32, 0);       // 0-light, 1-dark
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);

    // GPIO33->XBAR->PM_nFault
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);

    // GPIO35->SCIC_RX for system
    GPIO_setPinConfig(GPIO_35_SCIC_RX);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

    // GPIO37->SCIC_TX for system
    GPIO_setPinConfig(GPIO_37_SCIC_TX);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);

    // end of WMINVBRD_REV1P0
#elif defined(HVMTRPFC_REV1P1)
    // GPIO0->EPWM1A->M1_UH
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_writePin(0, 0);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_writePin(1, 0);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_writePin(2, 0);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_writePin(3, 0);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M1_WH
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_writePin(4, 0);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->EPWM3B->M1_WL
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_writePin(5, 0);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->EPWM4A->PFC_1H (Force Low)
    GPIO_setPinConfig(GPIO_6_GPIO6);
    GPIO_writePin(6, 0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->PFC_1L (Force Low)
    GPIO_setPinConfig(GPIO_7_GPIO7);
    GPIO_writePin(7, 0);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->Reserve
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // GPIO09->CLR-Fault
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_writePin(9, 1);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);

    // GPIO10->Reserve
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) || defined(AIS_SPI_EN)
    // GPIO11->SPIA_STE->DAC/AIS_SCS
    GPIO_setPinConfig(GPIO_11_SPIA_STE);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
#else   // !DAC128S_ENABLE & AIS_SPI_EN
    // GPIO11->Reserve
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
#endif   // !DAC128S_ENABLE & AIS_SPI_EN

#if defined(EPWMDAC_MODE)
    // GPIO12->PWM7A->DAC1
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->PWM7B->DAC2
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#else   // !EPWMDAC_MODE
    // GPIO12->Reserve
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->Reserve
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#endif  // !EPWMDAC_MODE

#if defined(DAC128S_ENABLE) || defined(AIS_SPI_EN)
    // GPIO16->SPIA_SIMO->DAC/AIS_SDI
    GPIO_setPinConfig(GPIO_16_SPIA_SIMO);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SPIA_SOMI->DAC/AIS_SDO
    GPIO_setPinConfig(GPIO_17_SPIA_SOMI);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#else   // !DAC128S_ENABLE & AIS_SPI_EN
    // GPIO16->Reserve
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->Reserve
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif   // !DAC128S_ENABLE & AIS_SPI_EN

#if defined(EPWMDAC_MODE)
    // GPIO18->EPWM6_A->DAC3
    GPIO_setPinConfig(GPIO_18_EPWM6_A);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->EPWM6_B->DAC4
    GPIO_setPinConfig(GPIO_19_EPWM6_B);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);
#else   // !EPWMDAC_MODE
    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->Reserve
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);
#endif  // !EPWMDAC_MODE

    // GPIO20->EQEP1_A
    GPIO_setPinConfig(GPIO_20_EQEP1_A);
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(20, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(20, 2);

    // GPIO21->EQEP1_B
    GPIO_setPinConfig(GPIO_21_EQEP1_B);
    GPIO_setDirectionMode(21, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(21, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(21, 2);

#if defined(COM_SCI_EN)
    // GPIO22->SCIC_TX->SYS_COM_TX
    GPIO_setPinConfig(GPIO_22_SCIC_TX);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);
#else   // !COM_SCI_EN
    // GPIO22->Reserve
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);
#endif  // !COM_SCI_EN

    // GPIO23->EQEP1_I
    GPIO_setPinConfig(GPIO_23_EQEP1_INDEX);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(23, 2);

    // GPIO24->CCARD_LED1
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);

#if defined(GUI_SCI_EN) || defined(SFRA_SCI_EN)
    // GPIO28->SCIA_RX
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO28->SCIA_TX
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);
#else   // !GUI_SCI_EN & SFRA_SCI_EN
    // GPIO28->Reserve
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO28->Reserve
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);
#endif   // !GUI_SCI_EN & SFRA_SCI_EN

#if defined(DAC128S_ENABLE) || defined(AIS_SPI_EN)
    // GPIO32->SPIA_CLK)->DAC/AIS_SCLK
    GPIO_setPinConfig(GPIO_32_SPIA_CLK);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
#else   // !DAC128S_ENABLE & AIS_SPI_EN
    // GPIO32->Reserve
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
#endif   // !DAC128S_ENABLE & AIS_SPI_EN

#if defined(COM_SCI_EN)
    // GPIO33->SCIC_RX->SYS_COM_RX
    GPIO_setPinConfig(GPIO_33_SCIC_RX);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#else   // !COM_SCI_EN
    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // !COM_SCI_EN

    // GPIO35->CANA_RX
    GPIO_setPinConfig(GPIO_35_CANA_RX);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

    // GPIO37->CANA_TX
    GPIO_setPinConfig(GPIO_37_CANA_TX);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);

    // GPIO39->CCARD_LED2
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);

    // GPIO40->TZ-1(nFAULT)
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(40, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(40, 2);

    // GPIO41->Reserve
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // end of HVMTRPFC_REV1P1
//------------------------------------------------------------------------------
#elif defined(BSXL8323RH_REVB)
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL*
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->Reserve
    GPIO_setPinConfig(GPIO_4_GPIO4);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->Reserve
    GPIO_setPinConfig(GPIO_5_GPIO5);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

#if defined(EPWMDAC_MODE)
    // GPIO6->PWM4A->DAC3
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_writePin(6, 0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
#else   // !EPWMDAC_MODE
    // GPIO6->Reserve
    GPIO_setPinConfig(GPIO_6_GPIO6);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
#endif  // !EPWMDAC_MODE

    // GPIO7->Reserve
    GPIO_setPinConfig(GPIO_7_GPIO7);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE)
    // GPIO08->SPIA_SIMO->DAC128S_SDI
    GPIO_setPinConfig(GPIO_8_SPIA_SIMO);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO8->Reserve GPIO
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE

#if defined(DAC128S_ENABLE)
    // GPIO09->SPIA_CLK->DAC128S_SCLK
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO9->Reserve GPIO
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE

    // GPIO10->EPWM6_A
    GPIO_setPinConfig(GPIO_10_EPWM6_A);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // GPIO11->EPWM6_B
    GPIO_setPinConfig(GPIO_11_EPWM6_B);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

#if defined(EPWMDAC_MODE)
    // GPIO12->PWM7A->DAC1
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_writePin(12, 0);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->PWM7B->DAC2
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_writePin(13, 0);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#else   // !EPWMDAC_MODE
    // GPIO12->Reserve
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->Reserve
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#endif  // !EPWMDAC_MODE

    // GPIO16->Reserve
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE)
    // GPIO17->SPIA_SOMI->DAC_SDO
    GPIO_setPinConfig(GPIO_17_SPIA_SOMI);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#else   // DAC128S_ENABLE
    // GPIO17->Reserve
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->Reserve
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO20->M1_DRV_LED
    GPIO_setPinConfig(GPIO_20_GPIO20);
    GPIO_writePin(20, 1);
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_21_GPIO21);
    GPIO_writePin(21, 1);
    GPIO_setDirectionMode(21, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_STD);

    // GPIO22->M1_DRV_nSCS/GAIN (has a 47k pull down)
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // GPIO23->M1_DRV_CAL, Low->Disable
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // GPIO24->M1_DRV_nFAULT
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(24, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(24, 2);

    // GPIO28->SCIA_RX->SFRA/GUI
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->SCIA_TX->SFRA/GUI
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO32->DRV_EN (jumper to GPIO29 position)
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_writePin(32, 1);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_writePin(33, 0);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO35->Reserve
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE)
    // GPIO37->SPIA_STE->DAC_SCS
    GPIO_setPinConfig(GPIO_37_SPIA_STE);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
#else
    // GPIO37->Reserve
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO39->>Debug_Test
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 1);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->Debug_Test1
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->Debug_Test2
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // end of BSXL8323RH_REVB
//------------------------------------------------------------------------------
#elif defined(DRV8329AEVM_REVA)
    // GPIO0->EPWM1A->M1_UH
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->Reserve
    GPIO_setPinConfig(GPIO_4_GPIO4);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->Reserve
    GPIO_setPinConfig(GPIO_5_GPIO5);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

#if defined(EPWMDAC_MODE)
    // GPIO6->PWM4A->DAC3
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_writePin(6, 0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
#else   // !EPWMDAC_MODE
    // GPIO6->Reserve
    GPIO_setPinConfig(GPIO_6_GPIO6);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
#endif  // !EPWMDAC_MODE

    // GPIO7->Reserve
    GPIO_setPinConfig(GPIO_7_GPIO7);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->SPIA_SIMO->M1_DRV_SDI/DAC128S_SDI
    GPIO_setPinConfig(GPIO_8_SPIA_SIMO);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // GPIO09->SPIA_CLK->M1_DRV_SCLK/DAC128S_SCLK
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);

    // GPIO10->EPWM6_A
    GPIO_setPinConfig(GPIO_10_EPWM6_A);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // GPIO11->EPWM6_B
    GPIO_setPinConfig(GPIO_11_EPWM6_B);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

#if defined(EPWMDAC_MODE)
    // GPIO12->PWM7A->DAC1
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_writePin(12, 0);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->PWM7B->DAC2
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_writePin(13, 0);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#else   // !EPWMDAC_MODE
    // GPIO12->Reserve
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->Reserve
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#endif  // !EPWMDAC_MODE

    // GPIO16->Reserve
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SPIA_SOMI->DAC_SDO
    GPIO_setPinConfig(GPIO_17_SPIA_SOMI);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->DAC128S_SYNC
    GPIO_setPinConfig(GPIO_19_SPIA_STE);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO20->M1_DRV_LED
    GPIO_setPinConfig(GPIO_20_GPIO20);
    GPIO_writePin(20, 1);
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);

    // GPIO22->Reserve
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // GPIO23->Reserve
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // GPIO24->SLEEP_DFLT
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_writePin(24, 0);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);

    // GPIO28->SCIA_RX
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->SCIA_TX
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

    // GPIO32->M1_DRV_nFAULT
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(32, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(32, 2);

    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_writePin(33, 0);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO35->Reserve
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

    // GPIO37->M1_DRV_EN(DRV_OFF), 1-disable, 0-enable
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_writePin(37, 0);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);

    // GPIO39->EQEP1_INDEX
    GPIO_setPinConfig(GPIO_39_EQEP1_INDEX);   // F2800137-LPD
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(39, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(39, 2);

    // GPIO40->EQEP1_A
    GPIO_setPinConfig(GPIO_40_EQEP1_A);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(40, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(40, 2);

    // GPIO41->EQEP1_B
    GPIO_setPinConfig(GPIO_41_EQEP1_B);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(41, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(41, 2);

    // end of DRV8329AEVM_REVA
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#else   // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#error The GPIOs is not configured for motor control board
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)

    return;
}  // end of HAL_setupGPIOs() function


void HAL_updatePWMsPeriod(HAL_MTR_Handle handle,
                          float32_t pwmControl_kHz, uint16_t controlTicksPWM)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    obj->pwmPeriodCycles = (uint16_t)((USER_SYSTEM_FREQ_MHz * 1000.0f / 2.0f)/ pwmControl_kHz);
    obj->numPWMTicksPerISRTick = controlTicksPWM;

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], obj->pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], obj->pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], obj->pwmPeriodCycles);

    // setup the Event Trigger Prescale Register (ETPS)
    if(obj->numPWMTicksPerISRTick > 15)
    {
        obj->numPWMTicksPerISRTick = 15;
    }
    else if(obj->numPWMTicksPerISRTick < 1)
    {
        obj->numPWMTicksPerISRTick = 1;
    }

    EPWM_setInterruptEventCount(obj->pwmHandle[0], obj->numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);
#endif  //MOTOR1_DCLINKSS       // single shunt

    return;
}


void HAL_setupPWMs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    obj->pwmPeriodCycles = (uint16_t)(USER_M1_PWM_TBPRD_NUM);
    obj->numPWMTicksPerISRTick = USER_M1_NUM_PWM_TICKS_PER_ISR_TICK;

    // disable the ePWM module time base clock sync signal
    // to synchronize all of the PWMs
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

#if defined(WMINVBRD_REV1P0) || defined(HVMTRPFC_REV1P1) || \
    defined(BSXL8323RH_REVB) || defined(DRV8329AEVM_REVA)

    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_MODE_UP_DOWN);

        EPWM_disablePhaseShiftLoad(obj->pwmHandle[cnt]);

        EPWM_setPeriodLoadMode(obj->pwmHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);

        EPWM_enableSyncOutPulseSource(obj->pwmHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

        EPWM_setClockPrescaler(obj->pwmHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                                 EPWM_HSCLOCK_DIVIDER_1);

        EPWM_setCountModeAfterSync(obj->pwmHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);

        EPWM_setEmulationMode(obj->pwmHandle[cnt], EPWM_EMULATION_FREE_RUN);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        EPWM_setTimeBaseCounter(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        EPWM_setTimeBasePeriod(obj->pwmHandle[cnt], 0);

        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_C,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_D,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

#if defined(MOTOR1_DCLINKSS)    // Single shunt
        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
#else  // !(MOTOR1_DCLINKSS)        // 2/3 shunt
        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

#endif  // !(MOTOR1_DCLINKSS)       // 2/3 shunt

        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);

        // select EPWMA as the input to the dead band generator
        EPWM_setRisingEdgeDeadBandDelayInput(obj->pwmHandle[cnt],
                                             EPWM_DB_INPUT_EPWMA);

        // configure the right polarity for active high complementary config.
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_RED,
                                      EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_FED,
                                      EPWM_DB_POLARITY_ACTIVE_LOW);

        // setup the Dead-Band Rising Edge Delay Register (DBRED)
        EPWM_setRisingEdgeDelayCount(obj->pwmHandle[cnt], MTR1_PWM_DBRED_CNT);

        // setup the Dead-Band Falling Edge Delay Register (DBFED)
        EPWM_setFallingEdgeDelayCount(obj->pwmHandle[cnt], MTR1_PWM_DBFED_CNT);

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmHandle[cnt], HAL_TZSEL_SIGNALS_ALL);
    }
    // WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA
#else   // !( WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#error The PWM is not configured for motor_1 control
#endif  // !( WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)

    // setup the Event Trigger Prescale Register (ETPS)
    if(obj->numPWMTicksPerISRTick > 15)
    {
        obj->numPWMTicksPerISRTick = 15;
    }
    else if(obj->numPWMTicksPerISRTick < 1)
    {
        obj->numPWMTicksPerISRTick = 1;
    }

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(obj->pwmHandle[0]);

    EPWM_setInterruptEventCount(obj->pwmHandle[0], obj->numPWMTicksPerISRTick);
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);

    // ADC trigger
    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);

#if defined(SAFETY_ENABLE)
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);
    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_B);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_B);
#endif  // SAFETY_ENABLE

    // ADC SOC trigger for the 1st dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_U_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_A);

    // ADC SOC trigger for the 2nd dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                             EPWM_SOC_B,
                             EPWM_SOC_TBCTR_U_CMPD);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_B);

    // ADC SOC trigger for the 3rd dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[2],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_A);

    // ADC SOC trigger for the 4th dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[2],
                             EPWM_SOC_B,
                             EPWM_SOC_TBCTR_D_CMPD);

    EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_B);

    // Single shunt current trigger
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);

    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_B);

    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_B);
#else   //!(MOTOR1_DCLINKSS)        // 2/3 shunt
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(obj->pwmHandle[0]);

    EPWM_setInterruptEventCount(obj->pwmHandle[0], obj->numPWMTicksPerISRTick);
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);


    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);
    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);
    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_B);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                    obj->numPWMTicksPerISRTick);

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_B);

#if defined(SAFETY_ENABLE)
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_A);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A,
                                    obj->numPWMTicksPerISRTick);

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_A);
#endif  // SAFETY_ENABLE
#endif  // !(MOTOR1_DCLINKSS)       // 2/3 shunt

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], obj->pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], obj->pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], obj->pwmPeriodCycles);

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_C, 10);

    // write the PWM data value  for ADC trigger
#if defined(MOTOR1_DCLINKSS)    // Single shunt
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, obj->pwmPeriodCycles>>1);
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, obj->pwmPeriodCycles>>1);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, obj->pwmPeriodCycles>>1);
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, obj->pwmPeriodCycles>>1);

#endif  //MOTOR1_DCLINKSS       // single shunt

    return;
}  // end of HAL_setupPWMs() function


#if defined(EPWMDAC_MODE)
void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;

    // PWMDAC frequency = 100kHz, calculate the period for pwm
    uint16_t  period_cycles = (uint16_t)(systemFreq_MHz *
                                  (float32_t)(1000.0f / HA_PWMDAC_FREQ_KHZ));
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmDACHandle[cnt],
                                    EPWM_COUNTER_MODE_UP);

        EPWM_disablePhaseShiftLoad(obj->pwmDACHandle[cnt]);

        EPWM_setPeriodLoadMode(obj->pwmDACHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);

        EPWM_enableSyncOutPulseSource(obj->pwmDACHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

        EPWM_setClockPrescaler(obj->pwmDACHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                                 EPWM_HSCLOCK_DIVIDER_1);

        EPWM_setCountModeAfterSync(obj->pwmDACHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);

        EPWM_setEmulationMode(obj->pwmDACHandle[cnt], EPWM_EMULATION_FREE_RUN);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmDACHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        EPWM_setTimeBaseCounter(obj->pwmDACHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], 0);

        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(obj->pwmDACHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        // setup the Action-Qualifier Output B Register (AQCTLB)
        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_RED, false);

        EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_FED, false);

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmDACHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmDACHandle[cnt], HAL_TZSEL_SIGNALS_ALL);

        // since the PWM is configured as an up/down counter, the period
        // register is set to one-half of the desired PWM period
        EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], period_cycles);
    }

    return;
}  // end of HAL_setupPWMDACs() function
#endif  // EPWMDAC_MODE


#pragma CODE_SECTION(HAL_setupSCIA, "sysfuncs");
void HAL_setupSCIA(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Initialize SCIA and its FIFO.
    SCI_performSoftwareReset(obj->sciHandle[0]);

    // Configure SCIA for echoback.
    SCI_setConfig(obj->sciHandle[0], DEVICE_LSPCLK_FREQ, GUI_BAUD_RATE,
                        ( SCI_CONFIG_WLEN_8 |
                          SCI_CONFIG_STOP_ONE |
                          SCI_CONFIG_PAR_NONE ) );

    SCI_resetChannels(obj->sciHandle[0]);

    SCI_resetRxFIFO(obj->sciHandle[0]);

    SCI_resetTxFIFO(obj->sciHandle[0]);

    SCI_clearInterruptStatus(obj->sciHandle[0], SCI_INT_TXFF | SCI_INT_RXFF);

    SCI_enableFIFO(obj->sciHandle[0]);

    SCI_enableModule(obj->sciHandle[0]);

    SCI_performSoftwareReset(obj->sciHandle[0]);

    return;
}  // end of HAL_setupSCIA() function


#pragma CODE_SECTION(HAL_setupSCIC, "sysfuncs");
void HAL_setupSCIC(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Initialize SCIC and its FIFO.
    SCI_performSoftwareReset(obj->sciHandle[1]);

    // Configure SCIC for echoback.
    SCI_setConfig(obj->sciHandle[1], DEVICE_LSPCLK_FREQ, COM_BAUD_RATE,
                        ( SCI_CONFIG_WLEN_8 |
                          SCI_CONFIG_STOP_ONE |
                          SCI_CONFIG_PAR_NONE ) );

    SCI_resetChannels(obj->sciHandle[1]);

    SCI_resetRxFIFO(obj->sciHandle[1]);

    SCI_resetTxFIFO(obj->sciHandle[1]);

    SCI_clearInterruptStatus(obj->sciHandle[0], SCI_INT_TXFF | SCI_INT_RXFF);

    SCI_enableFIFO(obj->sciHandle[1]);

    SCI_enableModule(obj->sciHandle[1]);

    SCI_performSoftwareReset(obj->sciHandle[1]);

    return;
}  // end of HAL_setupSCIC() function


#pragma CODE_SECTION(HAL_setupSPIA, "sysfuncs");
void HAL_setupSPIA(HAL_Handle handle)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;

    // Must put SPI into reset before configuring it
    SPI_disableModule(obj->spiHandle[0]);

    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size, 30MHz LSPCLK
    SPI_setConfig(obj->spiHandle[0], DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 400000, 16);

    SPI_disableLoopback(obj->spiHandle[0]);

    SPI_setEmulationMode(obj->spiHandle[0], SPI_EMULATION_FREE_RUN);

    SPI_enableFIFO(obj->spiHandle[0]);
    SPI_setFIFOInterruptLevel(obj->spiHandle[0], SPI_FIFO_TX0, SPI_FIFO_RX0);
    SPI_setTxFifoTransmitDelay(obj->spiHandle[0], 0x10);

    SPI_clearInterruptStatus(obj->spiHandle[0], SPI_INT_TXFF);

    // Configuration complete. Enable the module.
    SPI_enableModule(obj->spiHandle[0]);

    return;
}  // end of HAL_setupSPIA() function

#pragma CODE_SECTION(HAL_setupI2CA, "sysfuncs");
void HAL_setupI2CA(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Must put I2C into reset before configuring it
    I2C_disableModule(obj->i2cHandle);

    // I2C configuration. Use a 400kHz I2CCLK with a 50% duty cycle.
    I2C_initMaster(obj->i2cHandle, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_50);
    I2C_setConfig(obj->i2cHandle, I2C_MASTER_SEND_MODE);
    I2C_setSlaveAddress(obj->i2cHandle, I2C_SLAVE_ADDRESS);
    I2C_disableLoopback(obj->i2cHandle);
    I2C_setBitCount(obj->i2cHandle, I2C_BITCOUNT_8);
    I2C_setDataCount(obj->i2cHandle, 2);
    I2C_setAddressMode(obj->i2cHandle, I2C_ADDR_MODE_7BITS);

    // Enable stop condition and register-access-ready interrupts
    I2C_enableInterrupt(obj->i2cHandle, I2C_INT_ADDR_SLAVE |
                                        I2C_INT_ARB_LOST |
                                        I2C_INT_NO_ACK |
                                        I2C_INT_STOP_CONDITION);

    // FIFO configuration
    I2C_enableFIFO(obj->i2cHandle);
    I2C_setFIFOInterruptLevel(obj->i2cHandle, I2C_FIFO_TXEMPTY, I2C_FIFO_RX2);

//    I2C_clearInterruptStatus(obj->i2cHandle, I2C_INT_RXFF | I2C_INT_TXFF);
    I2C_clearInterruptStatus(obj->i2cHandle, I2C_INT_ARB_LOST | I2C_INT_NO_ACK);

    // Configuration complete. Enable the module.
    I2C_setEmulationMode(obj->i2cHandle, I2C_EMULATION_FREE_RUN);
    I2C_enableModule(obj->i2cHandle);

    return;
}  // end of HAL_setupI2CA() function

void HAL_setupTimeBaseTimer(HAL_Handle handle, const float32_t timeBaseFreq_Hz)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;

    uint32_t timerPeriod = (uint32_t)((USER_SYSTEM_FREQ_MHz * 1000.0f *1000.0f) /
                                      timeBaseFreq_Hz);

    // use timer 0 for CPU usage diagnostics
    CPUTimer_setPreScaler(obj->timerHandle[TIME_BASE_TIMER], 0);

    CPUTimer_setEmulationMode(obj->timerHandle[TIME_BASE_TIMER],
                              CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_setPeriod(obj->timerHandle[TIME_BASE_TIMER], timerPeriod);

    CPUTimer_startTimer(obj->timerHandle[TIME_BASE_TIMER]);

    return;
}  // end of HAL_setupTimeBaseTimer() function

#pragma CODE_SECTION(HAL_setupCPUUsageTimer, "sysfuncs");
void HAL_setupCPUUsageTimer(HAL_Handle handle)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;

    // use timer 2 for CPU usage diagnostics
    CPUTimer_setPreScaler(obj->timerHandle[CPU_USAGE_TIMER], 0);

    CPUTimer_setEmulationMode(obj->timerHandle[CPU_USAGE_TIMER],
                              CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_setPeriod(obj->timerHandle[CPU_USAGE_TIMER], 0xFFFFFFFF);

    CPUTimer_startTimer(obj->timerHandle[CPU_USAGE_TIMER]);

    return;
}  // end of HAL_setupCPUUsageTimer() function

#if defined(DATALOGF4_EN) || defined(DATALOGF2_EN)
void HAL_setupDMA(void)
{
    // Initializes the DMA controller to a known state
    DMA_initController();

    // Sets DMA emulation mode, Continue DMA operation
    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

    return;
}    //end of HAL_setupDMA() function

void HAL_setupDMAforDLOG(HAL_Handle handle, const uint16_t dmaChNum,
                          const void *destAddr, const void *srcAddr)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    DMA_configAddresses(obj->dmaChHandle[dmaChNum], destAddr, srcAddr);

    // configure DMA Channel
    DMA_configBurst(obj->dmaChHandle[dmaChNum], DLOG_BURST_SIZE, 2, 2);
    DMA_configTransfer(obj->dmaChHandle[dmaChNum], DLOG_TRANSFER_SIZE, 1, 1);
    DMA_configWrap(obj->dmaChHandle[dmaChNum], 0xFFFF, 0, 0xFFFF, 0);

    DMA_configMode(obj->dmaChHandle[dmaChNum], DMA_TRIGGER_SOFTWARE,
                   DMA_CFG_ONESHOT_ENABLE | DMA_CFG_CONTINUOUS_ENABLE |
                   DMA_CFG_SIZE_32BIT);

    DMA_setInterruptMode(obj->dmaChHandle[dmaChNum], DMA_INT_AT_END);
    DMA_enableTrigger(obj->dmaChHandle[dmaChNum]);
    DMA_disableInterrupt(obj->dmaChHandle[dmaChNum]);

    return;
}    //end of HAL_setupDMAforDLOG() function
#endif  // DATALOGF4_EN || DATALOGF2_EN

void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount = 0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }
}   //end of HAL_clearDataRAM() function
void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_DCLINKSS)    // Single shunt
#if defined(WMINVBRD_REV1P0)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    // WMINVBRD_REV1P0
#elif defined(DRV8329AEVM_REVA)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    // DRV8329AEVM_REVA
#elif defined(HVMTRPFC_REV1P1)
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);
    // HVMTRPFC_REV1P1
#else   // !(WMINVBRD_REV1P0 | DRV8329AEVM_REVA | HVMTRPFC_REV1P1)
#error Not select a right board for this project
#endif  // !(WMINVBRD_REV1P0 | DRV8329AEVM_REVA | HVMTRPFC_REV1P1)
    // 1-Shunt Hardware Board
#else   // !MOTOR1_DCLINKSS     // 2/3 shunt
#if defined(WMINVBRD_REV1P0) || defined(HVMTRPFC_REV1P1)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[2], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
    // (WMINVBRD_REV1P0 | HVMTRPFC_REV1P1)
#elif defined(BSXL8323RH_REVB)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
    // BSXL8323RH_REVB
#else  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB)
#error Not Select a Right Board   // 3-Shunt Hardware Board
#endif  // !(WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB)
    // 3-Shunt Hardware Board
#endif  // !MOTOR1_DCLINKSS     // 2/3 shunt

    return;
}   // end of HAL_setMtrCMPSSDACValue() function


void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us,
                   const float32_t adcSample_us)
{
    uint16_t deadband =  (uint16_t)(deadband_us * systemFreq_MHz);
    uint16_t noiseWindow =  (uint16_t)(noiseWindow_us * systemFreq_MHz);
    uint16_t adcSample =  (uint16_t)(adcSample_us * systemFreq_MHz);

    pPWMData->deadband = deadband;
    pPWMData->noiseWindow = noiseWindow;
    pPWMData->adcSample = adcSample;

    pPWMData->minCMPValue = deadband + noiseWindow + adcSample;

    return;
}   // end of HAL_setTriggerPrams() function

void HAL_getPWMPeriod(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData)
{

    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    pPWMData->periods = (float32_t)(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));

    return;
}   // end of HAL_getPWMPeriod() function

bool HAL_MTR_setGateDriver(HAL_MTR_Handle handle)
{
    bool driverStatus = false;

    SysCtl_delay(1000U);

#if defined(HVMTRPFC_REV1P1)
    // turn on the HvKit if present
    HAL_enableDRV(handle);
    SysCtl_delay(1000U);
    //HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
    // turn on the DRV8323RH if present
    HAL_enableDRV(handle);
    SysCtl_delay(1000U);
    // BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
    // turn on the DRV8329A if present
    HAL_enableDRV(handle);
    SysCtl_delay(1000U);
    // DRV8329AEVM_REVA
#endif  // HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA

    return(driverStatus);
}


// end of file
