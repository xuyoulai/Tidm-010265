//#############################################################################
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
//#############################################################################

//------------------------------------------------------------------------------
// \file   /solutions/universal_motorcontrol_lab/common/source/sys_main.c
//
// \brief  This project is used to implement motor control with FAST, eSMO
//         Encoder, and Hall sensors based sensored/sensorless-FOC.
//         Supports multiple TI EVM boards
//
//  Target: F280013x/F28003x/F28P55x
//
//------------------------------------------------------------------------------


// include the related header files
//
#include "user.h"
#include "sys_settings.h"
#include "sys_main.h"
#include "controlparameters.h"
#include "guidatalist.h"
#include "guicontrol.h"
#include "systemcontrol.h"
#include "systemdisplay.h"
#if defined(BUADTUNE_EN)
#include "baudratetune.h"
#endif  // BUADTUNE_EN

volatile SYSTEM_Vars_t systemVars;
#pragma DATA_SECTION(systemVars,"sys_data");

#ifdef CPUTIME_ENABLE
// define CPU time for performance test
CPU_TIME_Obj     cpuTime;
CPU_TIME_Handle  cpuTimeHandle;
#pragma DATA_SECTION(cpuTime,"sys_data");
#pragma DATA_SECTION(cpuTimeHandle,"sys_data");
#endif  // CPUTIME_ENABLE

#if defined(BUFDAC_MODE)
HAL_BuffDACData_t bufDACData;
#endif  // BUFDAC_MODE

#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1) || defined(WMINVBRD_REV1P0) || \
    defined(BSXL8323RH_REVB) || defined(DRV8329AEVM_REVA) || defined(DRV8353RH_EVM)
HAL_PWMDACData_t pwmDACData;
#pragma DATA_SECTION(pwmDACData,"sys_data");
// ( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA | DRV8353RH_EVM)
#else   // !( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA | DRV8353RH_EVM)
#error EPWMDAC is not supported on this kit!
#endif  // !( HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA | DRV8353RH_EVM)
#endif  // EPWMDAC_MODE

#if defined(DAC128S_ENABLE)
DAC128S_Handle   dac128sHandle;        //!< the DAC128S interface handle
DAC128S_Obj      dac128s;              //!< the DAC128S interface object
#pragma DATA_SECTION(dac128sHandle,"sys_data");
#pragma DATA_SECTION(dac128s,"sys_data");

#define DAC_SCALE_SET       (4096.0f)     // 12bit
#endif  // DAC128S_ENABLE
#if defined(SFRA_ENABLE)
float32_t   sfraNoiseId;
float32_t   sfraNoiseIq;
float32_t   sfraNoiseSpd;
float32_t   sfraNoiseOut;
float32_t   sfraNoiseFdb;
SFRA_TEST_e sfraTestLoop;        //speedLoop;
bool        sfraCollectStart;

#pragma DATA_SECTION(sfraNoiseId, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseIq, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseSpd, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseOut, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseFdb, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraTestLoop, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraCollectStart, "SFRA_F32_Data");
#endif  // SFRA_ENABLE

// **************************************************************************
// the functions
// !!! Please make sure that you had gone through the user guide, and follow the
// !!! guide to set up the kit and load the right code
void main(void)
{
    // (System Control)         -- SYSCTRL
    // (Parameters Update)      -- PRMUPDATE
    // (GUI_ENABLE, GUI Demo)   -- GUIDEMO
    // (SAFETY_ENABLE)          -- SAFETY

    // Clear memory for system and controller
    // The variables must be assigned to these sector if need to be cleared to zero
    HAL_clearDataRAM((void *)loadStart_est_data, (uint16_t)loadSize_est_data);
    HAL_clearDataRAM((void *)loadStart_user_data, (uint16_t)loadSize_user_data);
    HAL_clearDataRAM((void *)loadStart_hal_data, (uint16_t)loadSize_hal_data);
    HAL_clearDataRAM((void *)loadStart_foc_data, (uint16_t)loadSize_foc_data);
    HAL_clearDataRAM((void *)loadStart_sys_data, (uint16_t)loadSize_sys_data);
    HAL_clearDataRAM((void *)loadStart_ctrl_data, (uint16_t)loadSize_ctrl_data);
    HAL_clearDataRAM((void *)loadStart_datalog_data, (uint16_t)loadSize_datalog_data);
    HAL_clearDataRAM((void *)loadStart_SFRA_F32_Data, (uint16_t)loadSize_SFRA_F32_Data);
#if defined(GUI_SCI_EN)
    HAL_clearDataRAM((void *)loadStart_gui_data, (uint16_t)loadSize_gui_data);
#endif  // GUI_SCI_EN

#if defined(WMINVBRD_REV1P0)
    systemVars.boardKit = BOARD_WMINVBRD_REV1P0;    // WMINVBRD_REV1P0

#if defined(EPWMDAC_MODE) && defined(DAC128S_ENABLE)
#error EPWMDAC and DAC128S can't be supported together.
#endif  // EPWMDAC_MODE & DAC128S_ENABLE

#if defined(GUI_SCI_EN) && defined(DAC128S_ENABLE)
#error GUI based SCI and DAC128S can't be supported together.
#endif  // GUI_SCI_EN & DAC128S_ENABLE
    // WMINVBRD_REV1P0
#elif defined(HVMTRPFC_REV1P1)
    systemVars.boardKit = BOARD_HVMTRPFC_REV1P1;    // HVMTRPFC_REV1P1
    // HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
    systemVars.boardKit = BOARD_BSXL8323RH_REVB;    // BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
    systemVars.boardKit = BOARD_DRV8329AEVM_REVA;    // DRV8329AEVM_REVA
#else   // !HVMTRPFC_REV1P1 & WMINVBRD_REV1P0 & DRV8329AEVM_REVA
#error Not select a right board for this project
#endif  // !HVMTRPFC_REV1P1 & WMINVBRD_REV1P0 & DRV8329AEVM_REVA

#if defined(GUI_ENABLE) && defined(DAC128S_ENABLE)
#error Can't support PC GUI with SCI and DAC128S with SPI at the same time
#elif defined(GUI_ENABLE) && defined(SFRA_ENABLE)
#error Can't support PC GUI with SCI and SFRA GUI with SCI at the same time
#endif  // GUI_ENABLE | DAC128S_ENABLE | SFRA_ENABLE

#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    motorVars_M1.estimatorType = SLEST_TYPE_FAST_ESMO;
    motorVars_M1.estimatorMode = ESTIMATOR_MODE_FAST;
#elif defined(MOTOR1_FAST) && !defined(MOTOR1_ESMO)
    motorVars_M1.estimatorType = SLEST_TYPE_FAST_ONLY;
    motorVars_M1.estimatorMode = ESTIMATOR_MODE_FAST;
#elif !defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    motorVars_M1.estimatorType = SLEST_TYPE_ESMO_ONLY;
    motorVars_M1.estimatorMode = ESTIMATOR_MODE_ESMO;
#else
#error Not select a right estimator
#endif

#if defined(MOTOR1_FAST)
    systemVars.estLibVersion = EST_getFASTVersion();   // gets FAST version
#endif  // MOTOR1_FAST

#if defined(DATALOGF2_EN) && defined(STEP_RP_EN)
#error DATALOG and GRAPH_STEP_RESPONSE can't be used simultaneously on this device
#endif  // DATALOGF2_EN && STEP_RP_EN

#if (defined(MOTOR1_SSIPD) || defined(MOTOR1_OVM)) && defined(MOTOR1_DCLINKSS)
#error Don't enable SSIPD and OVM if enable single shunt
#elif defined(MOTOR1_DCLINKSS)
    motorVars_M1.currentSenType = LSC_TYPE_SINGLE_SHUNT;
#else
    motorVars_M1.currentSenType = LSC_TYPE_THREE_SHUNT;
#endif  // (MOTOR1_SSIPD | MOTOR1_OVM) & (MOTOR1_DCLINKSS

// ** above codes are only for checking the settings, not occupy the memory

    // Initialize device clock and peripherals
    Device_init();                  // call the function in device.c

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();              // call the function in device.c

#if defined(SAFETY_ENABLE)
    // Put watchdog in interrupt mode.
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_INTERRUPT);

    // Disable global interrupts.
    DINT;
#endif  //

    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();         // call the function in driverlib.lib

    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();    // call the function in driverlib.lib

#if defined(SAFETY_ENABLE)
    // Enable NMI. Typically this is already enabled by either the GEL file when
    // debugging or by the boot ROM when running standalone.
    EALLOW;
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = 0xFFFF;
    HWREGH(NMI_BASE + NMI_O_CFG) |= NMI_CFG_NMIE;
    EDIS;

    // Plug a default PIE vector table error handler.
    STL_PIE_RAM_configHandler(&STA_User_pieVectError);

    // Configure Timer 0 as the main time-out timer
    STA_Timer_config();

#if STA_UTIL_PROFILE
    STA_Util_configProfiler(CPUTIMER1_BASE);
#endif  // STA_UTIL_PROFILE
#endif  //

#if defined(SAFETY_ENABLE)
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    EINT;
    ERTM;

    STA_Tests_injectError = false;

    while(STA_Tests_index < STA_TESTS_NUMBERS)
    {
        if(CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE) == true)
        {
            CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
            STA_Tests_timer++;

            if(STA_Tests_timer >= STA_INITIAL_TIMER_SET)
            {
                STA_Tests_timer = 0;

                motorVars_M1.safetyFaultFlag |=
                        STA_Tests_testDevice(STA_Tests_testArray[STA_Tests_index++]);
            }
        }
    }

    STA_Tests_injectError = false;

    // Disable Global Interrupt (INTM) and real time interrupt (DBGM)
    DINT;
    DRTM;
#endif  //

#if defined(_FLASH)
    // Copy time critical code and flash setup code to RAM.
    // The LoadStart, LoadSize, and RunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    memcpy(&runStart_lfufuncs,  &loadStart_lfufuncs,  (size_t)&loadSize_lfufuncs);
    memcpy(&runStart_ctrlfuncs, &loadStart_ctrlfuncs, (size_t)&loadSize_ctrlfuncs);
#if defined(_F28003x)
    memcpy(&HwbistRunStart, &HwbistLoadStart, (size_t)&HwbistLoadSize);
#endif  // _F28003x
#endif  // _FLASH

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    // Sets up the CPU timer for time base
    HAL_setupTimeBaseTimer(halHandle, USER_TIME_BASE_FREQ_Hz);

    // Sets up the timers for CPU usage diagnostics
    HAL_setupCPUUsageTimer(halHandle);

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC/PWM interrupts for control
    // enable interrupts to trig DMA
    HAL_enableADCInts(halHandle);
    HAL_enableCtrlInts(halHandle);

    // set the control parameters for motor 1
    motorHandle_M1    = (MOTOR_Handle)(&motorVars_M1);
    motorSetHandle_M1 = (MOTORSETS_Handle)(&motorSetVars_M1);

    // set the reference speed, this can be replaced or removed
    motorVars_M1.flagEnableRunAndIdentify = false;

#if defined(MOTOR1_POWCTRL)
    motorVars_M1.powerRef_W   = 10.0f;       // W
    motorVars_M1.speedRef_Hz  = 100.0f;      // Hz
    motorVars_M1.speedSet_Hz  = 100.0f;      // Hz
#else
    motorVars_M1.speedRef_Hz  = 50.0f;       // Hz
    motorVars_M1.speedSet_Hz  = 50.0f;       // Hz
#endif  // MOTOR1_POWCTRL

    // false - enables identification, true - disables identification
    userParams_M1.flag_bypassMotorId = true;  //    false;   //

    initMotor1Handles(motorHandle_M1);
    initMotor1CtrlParameters(motorHandle_M1);

#if defined(_PRMS_UPDATE)
    // update control parameters from the list saved in a Flash section
    motorCtrlVars_M1.flagEnableLoadPrms = false;        // false  //
    motorCtrlVars_M1.flagEnableUpdatePrms = false;      // false  //

#if (USER_MOTOR1 == Teknic_M2310PLN04K)
    motorSetVars_M1.motorModel = M1_Teknic_M2310PL;
    motorSetVars_M1.dataPrmsIndexUse = M1_Teknic_M2310PL;
#elif (USER_MOTOR1 == Estun_EMJ_04APB22)
    motorSetVars_M1.motorModel = M2_Estun_04APB22;
    motorSetVars_M1.dataPrmsIndexUse = M2_Estun_04APB22;
#elif (USER_MOTOR1 == LACFAN_ZKSN_750)
    motorSetVars_M1.motorModel = M3_LACFAN_ZKSN_750;
    motorSetVars_M1.dataPrmsIndexUse = M3_LACFAN_ZKSN_750;
#elif (USER_MOTOR1 == Marathon_N56PNRA10102)
    motorSetVars_M1.motorModel = M4_Marathon_N56PNRA;
    motorSetVars_M1.dataPrmsIndexUse = M4_Marathon_N56PNRA;
#else  // The other motors, need to set the motors
    motorSetVars_M1.motorModel = M2_Estun_04APB22;
    motorSetVars_M1.dataPrmsIndexUse = M2_Estun_04APB22;
#endif  // The other motors, need to set the motors

    motorSetVars_M1.dataPrmsIndexPrev = motorSetVars_M1.dataPrmsIndexUse;

    if(motorCtrlVars_M1.flagEnableLoadPrms == true)
    {
        readDataPrmsIndex(motorSetHandle_M1);

        motorCtrlVars_M1.flagStatusLoadPrms = updateControlPrms(motorSetHandle_M1);

        motorCtrlVars_M1.flagEnableLoadPrms = false;
    }
#endif  // _PRMS_UPDATE

    resetMotor1CtrlParameters(motorHandle_M1);

    // setup the GPIOs
    HAL_setupGPIOs(halHandle);

    // set up gate driver after completed GPIO configuration
    motorVars_M1.faultMtrNow.bit.gateDriver =
            HAL_MTR_setGateDriver(motorHandle_M1->halMtrHandle);

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

#if defined(CMD_CAP_EN)
    setExtCmdCapParams(motorHandle_M1);
#endif  // CMD_CAP_EN

#ifdef CPUTIME_ENABLE
    // initialize the CPU usage module
    cpuTimeHandle = CPU_TIME_init(&cpuTime, sizeof(cpuTime));
    CPU_TIME_reset(cpuTimeHandle);
    CPU_TIME_setCtrlPeriod(cpuTimeHandle,
                           HAL_getTimeBasePeriod(motorHandle_M1->halMtrHandle) *
                           motorSetVars_M1.controlTicksPWM);
#endif  // CPUTIME_ENABLE

#if defined(BUFDAC_MODE)        // only F28003x Supports Buffer DAC
    bufDACData.ptrData[0] = &motorVars_M1.angleGen_rad;         // BUFDAC1
    bufDACData.ptrData[1] = &motorVars_M1.angleGen_rad;         // BUFDAC2, N/A

    bufDACData.offset[0] = 4096.0f * 0.5f;                      // BUFDAC1
    bufDACData.offset[1] = 4096.0f * 0.5f;                      // BUFDAC2, N/A

    bufDACData.gain[0] = 4096.0f / MATH_TWO_PI;                 // BUFDAC1
    bufDACData.gain[1] = 4096.0f / MATH_TWO_PI;                 // BUFDAC2, N/A
#endif  // BUFDAC_MODE

#if defined(EPWMDAC_MODE)
    // set DAC parameters
    pwmDACData.periodMax =
            PWMDAC_getPeriod(halHandle->pwmDACHandle[PWMDAC_NUMBER_1]);

    pwmDACData.ptrData[0] = &motorVars_M1.angleFOC_rad;                 // PWMDAC1
//    pwmDACData.ptrData[0] = &motorVars_M1.angleEST_rad;               // PWMDAC1
//    pwmDACData.ptrData[0] = &motorVars_M1.anglePLL_rad;               // PWMDAC1
    pwmDACData.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];         // PWMDAC2
    pwmDACData.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];         // PWMDAC3


    pwmDACData.offset[0] = 0.5f;    // PWMDAC1
//    pwmDACData.offset[1] = 0.5f;    // PWMDAC2
    pwmDACData.offset[1] = 0.5f;    // PWMDAC3
    pwmDACData.offset[2] = 0.5f;    // PWMDAC3

    pwmDACData.gain[0] = 1.0f / MATH_TWO_PI;                            // PWMDAC1
//    pwmDACData.gain[1] = 1.0f / MATH_TWO_PI;                          // PWMDAC2
    pwmDACData.gain[1] = 1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;       // PWMDAC2
    pwmDACData.gain[2] = 1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;       // PWMDAC3
//    pwmDACData.gain[2] = 4.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;     // PWMDAC3
#endif  // EPWMDAC_MODE

#if defined(DATALOGF2_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[0];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.V_V.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.speed_Hz;
//    datalogObj->iptr[0] = &resl_M1.sin_os;
//    datalogObj->iptr[1] = &resl_M1.cos_os;
//    datalogObj->iptr[0] = &isbldc_M1.bemfInt;
//    datalogObj->iptr[1] = &isbldc_M1.VintPhase;
#endif  // DATALOGF2_EN DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);
    HAL_setupDMAforDLOG(halHandle, 2, &datalogBuff3[0], &datalogBuff3[1]);
    HAL_setupDMAforDLOG(halHandle, 3, &datalogBuff4[0], &datalogBuff4[1]);

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[1];
    datalogObj->iptr[2] = &motorVars_M1.adcData.I_A.value[2];
    datalogObj->iptr[3] = &motorVars_M1.angleFOC_rad;
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.V_V.value[1];
    datalogObj->iptr[2] = &motorVars_M1.adcData.V_V.value[2];
    datalogObj->iptr[3] = &motorVars_M1.angleFOC_rad;
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.angleEST_rad;
    datalogObj->iptr[2] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[3] = &motorVars_M1.adcData.V_V.value[0];
#endif  // DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#endif  // DATALOGI4_EN


#if defined(DAC128S_ENABLE)
    // initialize the DAC128S
    dac128sHandle = DAC128S_init(&dac128s);

    DAC128S_setupSPI(dac128sHandle);


// The following settings are for output the values of different variables
// in each build level for debug. The User can select one of these groups in
// different build level as commented note

// DAC_LEVEL4_DCLINK, DAC_LEVEL4_VIBCOMP,
// DAC_LEVEL2_MOTOR1_VS, DAC_LEVEL2_MOTOR1_IS, DAC_LEVEL_MOTOR1_FAST,
// DAC_LEVEL4_FAST_ESMO, DAC_LEVEL4_FAST,  DAC_LEVEL4_ESMO, DAC_LEVEL4_PHADJ,

#define DAC_LEVEL_MOTOR1_FAST            // define the DAC level

#if defined(DAC_LEVEL4_DCLINK)
    // Build_Level_2, verify the current sampling value
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcIs_A.value[0];            // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL4_FAST_ESMO)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.anglePLL_rad;                // CH_B
#if defined(ESMO_DEBUG)
    dac128s.ptrData[2] = &esmo_M1.thetaElec_rad;                    // CH_C
#else   //!ESMO_DEBUG
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
#endif  //!ESMO_DEBUG
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
#if defined(ESMO_DEBUG)
    dac128s.gain[2] = DAC_SCALE_SET / MATH_TWO_PI;
#else   //!ESMO_DEBUG
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
#endif  //!ESMO_DEBUG
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL_MOTOR1_FAST)
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_FAST_ESMO)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_B
    dac128s.ptrData[1] = &motorVars_M1.anglePLL_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_D
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_E

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL4_FAST)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.speed_Hz;                   // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = DAC_SCALE_SET / USER_MOTOR1_FREQ_MAX_Hz;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.0f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL4_ESMO)
    dac128s.ptrData[0] = &motorVars_M1.anglePLL_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.speed_Hz;                   // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = DAC_SCALE_SET / USER_MOTOR1_FREQ_MAX_Hz;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.0f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL4_PHADJ)
    dac128s.ptrData[0] = &motorVars_M1.Vab_out_V.value[0];          // CH_A
    dac128s.ptrData[1] = &motorVars_M1.estInputData.Iab_A.value[0]; // CH_B
    dac128s.ptrData[2] = &motorVars_M1.Eab_V.value[0];              // CH_C
    dac128s.ptrData[3] = &motorVars_M1.Eab_V.value[1];             // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[1] = DAC_SCALE_SET * 4.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL3_MOTOR1_FAST)
    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleGen_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.angleEST_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL2_MOTOR1_IS)
    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 1.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 1.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 1.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);

#elif defined(DAC_LEVEL2_MOTOR1_VS)
    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.V_V.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.V_V.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    DAC128S_writeCommand(dac128sHandle);
#endif  // DAC128S_ENABLE

#if defined(SFRA_ENABLE)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, USER_M1_ISR_FREQ_Hz);

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;
    sfraNoiseOut = 0.0f;
    sfraNoiseFdb = 0.0f;
    sfraTestLoop = SFRA_TEST_D_AXIS;
    sfraCollectStart = false;
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
    GRAPH_init(&stepRPVars,
               &motorVars_M1.speedRef_Hz, &motorVars_M1.speed_Hz,
               &motorVars_M1.IdqRef_A.value[0], &motorVars_M1.Idq_in_A.value[0],
               &motorVars_M1.IdqRef_A.value[1], &motorVars_M1.Idq_in_A.value[1]);
#endif  // STEP_RP_EN

    motorVars_M1.flagEnableOffsetCalc = true;

    // run offset calibration for motor 1
    runMotor1OffsetsCalculation(motorHandle_M1);

#if defined(GUI_SCI_EN)
    GUI_initCtrlParms();

    // Initialize GUI SCI
    GUI_initUartParams();
    GUI_setupUart();

    GUI_initCtrlSettings();

    motorCtrlVars_M1.flagEnableGuiControl = true;
#endif  // GUI_SCI_EN



#if defined(BUADTUNE_EN)
    BRT_initConfiguration();
#endif  // BUADTUNE_EN

    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    EINT;
    ERTM;

    // initialize variables
    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;
    systemVars.timerBase_1ms = 0;

    systemVars.flagEnableSystem = true;                 // Remove this line for AC power input


    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0) == true)
        {
            // read the ADC data with offsets
            HAL_readMtr1ADCData(&motorHandle_M1->adcData);

        #if(DMC_BUILDLEVEL >= DMC_LEVEL_4)
            if(motorHandle_M1->adcData.VdcBus_V >= POWER_RELAY_ON_VOLTAGE_V)    // 100V
            {
                systemVars.timerBase_1ms++;
            }
        #else
            systemVars.timerBase_1ms++;
        #endif

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = true;
                systemVars.timerBase_1ms = 0;

                // Turn on the power relay
                HAL_turnOnPowerRelay(ACPOWER_RELAY_GPIO);
                HAL_turnOnPowerRelay(ACPOWER_RELAY_GPIO);
            }

            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);
        }
    }

    // Turn on the power relay
    HAL_turnOnPowerRelay(ACPOWER_RELAY_GPIO);
    HAL_turnOnPowerRelay(ACPOWER_RELAY_GPIO);

    systemVars.flagEnableSystem = false;
    systemVars.timerBase_1ms = 0;
    systemVars.powerRelayWaitTime_ms = OFFSET_CHECK_WAIT_TIME_ms;

    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            systemVars.timerBase_1ms++;

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = true;
                systemVars.timerBase_1ms = 0;
            }
        }
    }

    motorVars_M1.flagInitializeDone = true;

    while(systemVars.flagEnableSystem == true)
    {
#if defined(BUADTUNE_EN)
        BRT_tuneBaudRate();
#endif  // BUADTUNE_EN

        // loop while the enable system flag is true
        systemVars.mainLoopCnt++;

        // 1ms time base
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            systemVars.timerBase_1ms++;


            switch(systemVars.timerBase_1ms)
            {
                case 1:     // motor 1 protection check
                    runMotorMonitor(motorHandle_M1);
#if defined(SAFETY_ENABLE)
                    STA_Tests_timer++;

                    if(STA_Tests_timer >= STA_MAINLOOP_TIMER_SET)
                    {
                        STA_Tests_timer = 0;
                        STA_Tests_index = STA_CPU_REG;

                        motorVars_M1.safetyFaultFlag |=
                                STA_Tests_testDevice(STA_Tests_testArray[STA_Tests_index]);
                    }
#endif  //
                    break;
                case 2:
#if !defined(_SIMPLE_FAST_LIB) && !defined(_HSWFREQ_EN)
                    // calculate the current/voltage RMS value
                    calculateRMSData(motorHandle_M1);
#endif  // !(_HSWFREQ_EN & _SIMPLE_FAST_LIB)

                    // calculate motor protection value
                    calcMotorOverCurrentThreshold(motorHandle_M1);

#if defined(SAFETY_ENABLE)
                    STA_Tests_timer++;

                    if(STA_Tests_timer >= STA_MAINLOOP_TIMER_SET)
                    {
                        STA_Tests_timer = 0;
                        STA_Tests_index = STA_FPU_REG;

                        motorVars_M1.safetyFaultFlag |=
                                STA_Tests_testDevice(STA_Tests_testArray[STA_Tests_index]);
                    }
#endif  //
                    break;
                case 3:
                    // Tune the gains of the controllers
                    tuneControllerGains(motorHandle_M1);

#if defined(MOTOR1_ESMO)
//                    updateESMOParameters(motorHandle_M1);
#endif  // MOTOR2_ESMO

#if defined(MOTOR1_FAST)
                    updateFASTParameters(motorHandle_M1);
#endif  // MOTOR1_FAST

#if defined(SAFETY_ENABLE)
                    STA_Tests_timer++;

                    if(STA_Tests_timer >= STA_MAINLOOP_TIMER_SET)
                    {
                        STA_Tests_timer = 0;
                        STA_Tests_index = STA_SP_TEST;

                        motorVars_M1.safetyFaultFlag |=
                                STA_Tests_testDevice(STA_Tests_testArray[STA_Tests_index]);
                    }
#endif  //
                    break;
                case 4:
#if defined(SAFETY_ENABLE)
                    STA_Tests_timer++;

                    if(STA_Tests_timer >= STA_MAINLOOP_TIMER_SET)
                    {
                        STA_Tests_timer = 0;
                        STA_Tests_index = STA_FLASH_CRC;

                        motorVars_M1.safetyFaultFlag |=
                                STA_Tests_testDevice(STA_Tests_testArray[STA_Tests_index]);
                    }
#endif  //
                    break;
                case 5:     // system control
#if defined(SAFETY_ENABLE)
                    STA_Tests_timer++;

                    if(STA_Tests_timer >= STA_MAINLOOP_TIMER_SET)
                    {
                        STA_Tests_timer = 0;
                        STA_Tests_index = STA_MARCH;

                        motorVars_M1.safetyFaultFlag |=
                                STA_Tests_testDevice(STA_Tests_testArray[STA_Tests_index]);
                    }
#endif  //
                    systemVars.timerBase_1ms = 0;
                    systemVars.timerCnt_5ms++;

                    SYS_processFault();
                    SYS_displayFault();
#if defined(_PRMS_UPDATE)
                    if((motorVars_M1.controlStatus == MOTOR_STOP_IDLE) &&
                            (motorCtrlVars_M1.updatePrmsDelayTime == 0))
                    {
                        if(motorCtrlVars_M1.flagEnableLoadPrms == true)
                        {
                            motorCtrlVars_M1.flagStatusLoadPrms = updateControlPrms(motorSetHandle_M1);
                            motorCtrlVars_M1.flagEnableUpdatePrms = true;
                        }

                        if(motorCtrlVars_M1.flagEnableUpdatePrms == true)
                        {
                            motorCtrlVars_M1.flagEnableUpdatePrms = false;
                            resetMotor1CtrlParameters(motorHandle_M1);
                            motorCtrlVars_M1.flagStatusUpdatePrms = true;
                        }

#if defined(_LFU_ENABLE)
                        if(motorSetVars_M1.dataPrmsFlashFlag == 1)
                        {
                            motorVars_M1.controlStatus = MOTOR_PRMS_STORE;          // Save parameters start
                            motorCtrlVars_M1.controlStatus = MOTOR_PRMS_STORE;      // Save parameters start
                            Interrupt_disableMaster();

                            storeControlPrms(motorSetHandle_M1);
                            Interrupt_enableMaster();

                            if(motorSetVars_M1.dataPrmsFlashStatus == 0x0000)
                            {
                                motorSetVars_M1.dataPrmsFlashFlag = 0;
                                motorCtrlVars_M1.flagStatusUpdatePrms = true;
                            }
                            else
                            {
                                motorCtrlVars_M1.flagStatusUpdatePrms = false;
                            }

                            motorVars_M1.controlStatus = MOTOR_STOP_IDLE;           // Save parameters end
                            motorCtrlVars_M1.updatePrmsDelayTime = UPDATE_PRMS_DELAY_TIME_SET;
                        }
                        else
                        {
                            motorCtrlVars_M1.flagStatusUpdatePrms = false;
                        }

#endif      // _LFU_ENABLE
                    }   // (motorVars_M1.controlStatus == MOTOR_STOP_IDLE)
                    else if(motorCtrlVars_M1.updatePrmsDelayTime > 0)
                    {
                        motorCtrlVars_M1.updatePrmsDelayTime--;
                    }
#endif  // _PRMS_UPDATE
                    break;
            }

#if defined(GUI_SCI_EN)
            GUI_updateCtrlSate();
#endif  // GUI_SCI_EN

#if defined(CMD_POT_EN)
            updateExtCmdPotFreq(motorHandle_M1);
#endif  // CMD_POT_EN

#if defined(CMD_CAP_EN)
            updateExtCmdCapFreq(motorHandle_M1,
                                HAL_calcCAPCount(motorHandle_M1->halMtrHandle));
#endif  // CMD_CAP_EN

#if defined(SFRA_ENABLE)
            // SFRA test
            SFRA_F32_runBackgroundTask(&sfra1);
            SFRA_GUI_runSerialHostComms(&sfra1);
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
            // Generate Step response
            GRAPH_generateStepResponse(&stepRPVars);
#endif  // STEP_RP_EN

#ifdef CPUTIME_ENABLE
            CPU_TIME_calcCPUWidthRatio(cpuTimeHandle);
#endif  // CPUTIME_ENABLE
        }       // 1ms Timer

#if defined(CMD_SWITCH_EN)
        outputCmdState(motorHandle_M1);
#endif  //CMD_SWITCH_EN

        // runs control for motor 1
        runMotor1Control(motorHandle_M1);       // No time base

#if defined(GUI_SCI_EN)
        GUI_updateReceiveData();
#endif  // GUI_SCI_EN
    } // end of while() loop

    // disable the PWM
    HAL_disablePWM(motorHandle_M1->halMtrHandle);

} // end of main() function

//
//-- end of this file ----------------------------------------------------------
//
