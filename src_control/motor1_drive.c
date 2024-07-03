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
//! \file   /solutions/universal_motorcontrol_lab/common/source/motor1_drive.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         based sensorless-FOC.
//!         Supports multiple TI EVM boards
//!
//------------------------------------------------------------------------------

//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"
#include "motor1_drive.h"


//#pragma CODE_SECTION(motor1CtrlISR, ".TI.ramfunc");
#pragma CODE_SECTION(motor1CtrlISR, "ctrlfuncs");
#pragma INTERRUPT(motor1CtrlISR, {HPI});


// the globals
//!< the hardware abstraction layer object to motor control
volatile MOTOR_Handle motorHandle_M1;
#pragma DATA_SECTION(motorHandle_M1, "ptr_data");

volatile MOTORSETS_Handle motorSetHandle_M1;
#pragma DATA_SECTION(motorSetHandle_M1, "ptr_data");


volatile MOTORCTRL_Handle motorCtrlHandle_M1;
#pragma DATA_SECTION(motorCtrlHandle_M1, "ptr_data");

MOTOR_CtrlVars_t motorCtrlVars_M1;
#pragma DATA_SECTION(motorCtrlVars_M1, "foc_data");

MOTOR_SetVars_t motorSetVars_M1;
#pragma DATA_SECTION(motorSetVars_M1, "foc_data");

MOTOR_Vars_t motorVars_M1;
#pragma DATA_SECTION(motorVars_M1, "foc_data");

HAL_MTR_Obj    halMtr_M1;
#pragma DATA_SECTION(halMtr_M1, "foc_data");

#if defined(MOTOR1_FAST)
//!< the voltage Clarke transform object
CLARKE_Obj    clarke_V_M1;
#pragma DATA_SECTION(clarke_V_M1, "foc_data");
#endif  // MOTOR1_FAST

//!< the current Clarke transform object
CLARKE_Obj    clarke_I_M1;
#pragma DATA_SECTION(clarke_I_M1, "foc_data");

//!< the inverse Park transform object
IPARK_Obj     ipark_V_M1;
#pragma DATA_SECTION(ipark_V_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_I_M1;
#pragma DATA_SECTION(park_I_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_V_M1;
#pragma DATA_SECTION(park_V_M1, "foc_data");

#if defined(MOTOR1_POWCTRL)
//!< the speed PI controller object
PI_Obj        pi_pow_M1;
#pragma DATA_SECTION(pi_pow_M1, "foc_data");
#endif  // MOTOR1_POWCTRL

//!< the speed PI controller object
PI_Obj        pi_spd_M1;
#pragma DATA_SECTION(pi_spd_M1, "foc_data");

//!< the Id PI controller object
PI_Obj        pi_Id_M1;
#pragma DATA_SECTION(pi_Id_M1, "foc_data");

//!< the Iq PI controller object
PI_Obj        pi_Iq_M1;
#pragma DATA_SECTION(pi_Iq_M1, "foc_data");

//!< the space vector generator object
SVGEN_Obj     svgen_M1;
#pragma DATA_SECTION(svgen_M1, "foc_data");

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
SVGENCURRENT_Obj svgencurrent_M1;
#pragma DATA_SECTION(svgencurrent_M1, "foc_data");
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
TRAJ_Obj     traj_spd_M1;
#pragma DATA_SECTION(traj_spd_M1, "foc_data");

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
PI_Obj       pi_fwc_M1;
#pragma DATA_SECTION(pi_fwc_M1, "foc_data");
#endif  // MOTOR1_FWC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
    defined(MOTOR1_ESMO) || defined(MOTOR1_DCLINKSS)
//!< the Angle Generate onject for open loop control
ANGLE_GEN_Obj    angleGen_M1;
#pragma DATA_SECTION(angleGen_M1, "foc_data");
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
VS_FREQ_Obj    VsFreq_M1;
#pragma DATA_SECTION(VsFreq_M1, "foc_data");
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_ESMO)
//!< the speedfr object
SPDFR_Obj spdfr_M1;
#pragma DATA_SECTION(spdfr_M1, "foc_data");

//!< the esmo object
ESMO_Obj   esmo_M1;
#pragma DATA_SECTION(esmo_M1, "foc_data");
#endif  // MOTOR1_ESMO

#if defined(MOTOR1_MTPA)
//!< the Maximum torque per ampere (MTPA) object
MTPA_Obj     mtpa_M1;
#pragma DATA_SECTION(mtpa_M1, "foc_data");
#endif  // MOTOR1_MTPA


#if defined(MOTOR1_DCLINKSS)    // Single shunt
//!< the single-shunt current reconstruction object
DCLINK_SS_Obj    dclink_M1;

#pragma DATA_SECTION(dclink_M1, "foc_data");
#endif // MOTOR1_DCLINKSS       // single shunt

#if defined(MOTOR1_VOLRECT)
//!< the voltage reconstruct object
VOLREC_Obj volrec_M1;
#pragma DATA_SECTION(volrec_M1, "foc_data");
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_FILTERIS)
//!< first order current filter object
FILTER_FO_Obj    filterIs_M1[3];

#pragma DATA_SECTION(filterIs_M1, "foc_data");
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
//!< first order voltage filter object
FILTER_FO_Obj    filterVs_M1[3];

#pragma DATA_SECTION(filterVs_M1, "foc_data");
#endif  // MOTOR1_FILTERVS

#if defined(BSXL8316RT_REVA) && defined(OFFSET_CORRECTION)
MATH_Vec3 I_correct_A;

#pragma DATA_SECTION(I_correct_A, "foc_data");
#endif  // BSXL8316RT_REVA & OFFSET_CORRECTION

#if defined(BENCHMARK_TEST)
BMTEST_Vars_t bmarkTestVars;

#pragma DATA_SECTION(bmarkTestVars, "foc_data");
#endif  // BENCHMARK_TEST


// the control handles for motor 1
void initMotor1Handles(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    // initialize the driver
    obj->halMtrHandle = HAL_MTR1_init(&halMtr_M1, sizeof(halMtr_M1));

    obj->motorCtrlHandle = &motorCtrlVars_M1;
    obj->motorSetsHandle = &motorSetVars_M1;
    obj->userParamsHandle = &userParams_M1;

#if defined(MOTOR1_FWC)
    obj->piHandle_fwc = PI_init(&pi_fwc_M1, sizeof(pi_fwc_M1));
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // initialize the Maximum torque per ampere (MTPA)
    obj->mtpaHandle = MTPA_init(&mtpa_M1, sizeof(mtpa_M1));
#endif  // MOTOR1_MTPA


#if defined(MOTOR1_DCLINKSS)    // Single shunt
    obj->dclinkHandle = DCLINK_SS_init(&dclink_M1, sizeof(dclink_M1));
#endif   // MOTOR1_DCLINKSS     // single shunt

#ifdef MOTOR1_VOLRECT
    // initialize the Voltage reconstruction
    obj->volrecHandle = VOLREC_init(&volrec_M1, sizeof(volrec_M1));
#endif  // MOTOR1_VOLRECT

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC) || defined(MOTOR1_DCLINKSS)
    // initialize the angle generate module
    obj->angleGenHandle = ANGLE_GEN_init(&angleGen_M1, sizeof(angleGen_M1));
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT || MOTOR1_ENC

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    obj->VsFreqHandle = VS_FREQ_init(&VsFreq_M1, sizeof(VsFreq_M1));
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_ESMO)
    // initialize the esmo
    obj->esmoHandle = ESMO_init(&esmo_M1, sizeof(esmo_M1));

    // initialize the spdfr
    obj->spdfrHandle = SPDFR_init(&spdfr_M1, sizeof(spdfr_M1));
#endif  //MOTOR1_ESMO

#if defined(MOTOR1_FAST)
    // initialize the Clarke modules
    obj->clarkeHandle_V = CLARKE_init(&clarke_V_M1, sizeof(clarke_V_M1));
#endif // MOTOR1_FAST

    // initialize the Clarke modules
    obj->clarkeHandle_I = CLARKE_init(&clarke_I_M1, sizeof(clarke_I_M1));

    // initialize the inverse Park module
    obj->iparkHandle_V = IPARK_init(&ipark_V_M1, sizeof(ipark_V_M1));

    // initialize the Park module
    obj->parkHandle_I = PARK_init(&park_I_M1, sizeof(park_I_M1));

    // initialize the Park module
    obj->parkHandle_V = PARK_init(&park_V_M1, sizeof(park_V_M1));

    // initialize the PI controllers
#if defined(MOTOR1_POWCTRL)
    obj->piHandle_pow = PI_init(&pi_pow_M1, sizeof(pi_pow_M1));
#endif  // MOTOR1_POWCTRL
    obj->piHandle_spd = PI_init(&pi_spd_M1, sizeof(pi_spd_M1));
    obj->piHandle_Id  = PI_init(&pi_Id_M1, sizeof(pi_Id_M1));
    obj->piHandle_Iq  = PI_init(&pi_Iq_M1, sizeof(pi_Iq_M1));

    // initialize the speed reference trajectory
    obj->trajHandle_spd = TRAJ_init(&traj_spd_M1, sizeof(traj_spd_M1));

    // initialize the space vector generator module
    obj->svgenHandle = SVGEN_init(&svgen_M1, sizeof(svgen_M1));

#if defined(MOTOR1_FILTERIS)
    // assign the current filter handle (low pass filter)
    obj->filterHandle_Is[0] = FILTER_FO_init((void *)(&filterIs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[1] = FILTER_FO_init((void *)(&filterIs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[2] = FILTER_FO_init((void *)(&filterIs_M1[2]), sizeof(FILTER_FO_Obj));
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    // assign the voltage filter handle (low pass filter)
    obj->filterHandle_Vs[0] = FILTER_FO_init((void *)(&filterVs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Vs[1] = FILTER_FO_init((void *)(&filterVs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Vs[2] = FILTER_FO_init((void *)(&filterVs_M1[2]), sizeof(FILTER_FO_Obj));
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    obj->svgencurrentHandle =
            SVGENCURRENT_init(&svgencurrent_M1, sizeof(svgencurrent_M1));
#endif  // MOTOR1_OVM

#if defined(MOTOR1_FAST)
    // initialize the estimator
    obj->estHandle = EST_initEst(MTR_1);
#endif // MOTOR1_FAST

    return;
}

// initialize control parameters for motor 1
void initMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // initialize the user parameters
    USER_setParams_priv(obj->userParamsHandle);

    objSets->ctrlPeriod_sec = USER_M1_CTRL_PERIOD_sec;
    objSets->ctrlFreq_Hz = USER_M1_ISR_FREQ_Hz;
    objSets->estFreq_Hz = USER_M1_ISR_FREQ_Hz;

    objSets->motorModel = M1_Teknic_M2310PL;

    objSets->overCurrentTimesSet = USER_M1_OVER_CURRENT_TIMES_SET;
    objSets->overLoadTimeSet = USER_M1_OVER_LOAD_TIME_SET;
    objSets->motorStallTimeSet = USER_M1_STALL_TIME_SET;
    objSets->voltageFaultTimeSet = USER_M1_VOLTAGE_FAULT_TIME_SET;
    objSets->startupFailTimeSet = USER_M1_STARTUP_FAIL_TIME_SET;

    objSets->flyingStartTimeDelay = (uint16_t)(objSets->ctrlFreq_Hz * 0.001f);  // 1ms
    objSets->alignTimeDelay = (uint16_t)(objSets->ctrlFreq_Hz * 2.0f);          // 2.0s
    objSets->forceRunTimeDelay = (uint16_t)(objSets->ctrlFreq_Hz * 1.0f);       // 1.0s
    objSets->startupTimeDelay = (uint16_t)(objSets->ctrlFreq_Hz * 2.0f);        // 2.0s

    objSets->motor_type = USER_MOTOR1_TYPE;
    objSets->numPolePairs = USER_MOTOR1_NUM_POLE_PAIRS;
    objSets->Rs_Ohm = USER_MOTOR1_Rs_Ohm;
    objSets->Ls_d_H = USER_MOTOR1_Ls_d_H;
    objSets->Ls_q_H = USER_MOTOR1_Ls_q_H;
    objSets->flux_VpHz = USER_MOTOR1_RATED_FLUX_VpHz;
    objSets->Rr_Ohm = USER_MOTOR1_Rr_Ohm;
    objSets->magneticCurrent_A = USER_MOTOR1_MAGNETIZING_CURRENT_A;

    objSets->voltageFilter_Hz = USER_M1_VOLTAGE_FILTER_POLE_Hz;
    objSets->voltageScale_V = USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    objSets->currentScale_A = USER_M1_ADC_FULL_SCALE_CURRENT_A;
    objSets->pwmControl_kHz = USER_M1_PWM_FREQ_kHz;
    objSets->controlTicksPWM = USER_M1_NUM_PWM_TICKS_PER_ISR_TICK;
    objSets->speedTicksControl = USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK;

    objSets->maxFrequency_Hz = USER_MOTOR1_FREQ_MAX_Hz;
    objSets->maxCurrentSet_A = USER_MOTOR1_MAX_CURRENT_A;
    objSets->maxVoltage_V = USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;
    objSets->maxPeakCurrent_A = USER_M1_ADC_FULL_SCALE_CURRENT_A * 0.4975f;
    objSets->maxVsMag_pu = USER_M1_MAX_VS_MAG_PU;

    objSets->maxCurrentResEst_A = USER_MOTOR1_RES_EST_CURRENT_A;
    objSets->maxCurrentIndEst_A = USER_MOTOR1_IND_EST_CURRENT_A;
    objSets->fluxExcFreq_Hz = USER_MOTOR1_FLUX_EXC_FREQ_Hz;

    objSets->Ls_d_Icomp_coef = USER_MOTOR1_Ls_d_COMP_COEF / USER_MOTOR1_MAX_CURRENT_A;
    objSets->Ls_q_Icomp_coef = USER_MOTOR1_Ls_q_COMP_COEF / USER_MOTOR1_MAX_CURRENT_A;
    objSets->Ls_min_H = USER_MOTOR1_Ls_d_H * USER_MOTOR1_Ls_MIN_NUM_COEF;

    objSets->overCurrent_A = USER_MOTOR1_OVER_CURRENT_A;
    objSets->overLoadSet_W = USER_M1_OVER_LOAD_POWER_W;

    objSets->lostPhaseSet_A = USER_M1_LOST_PHASE_CURRENT_A;
    objSets->unbalanceRatioSet = USER_M1_UNBALANCE_RATIO;
    objSets->stallCurrentSet_A = USER_M1_STALL_CURRENT_A;
    objSets->speedFailMaxSet_Hz = USER_M1_FAIL_SPEED_MAX_HZ;
    objSets->speedFailMinSet_Hz = USER_M1_FAIL_SPEED_MIN_HZ;
    objSets->IsFailedChekSet_A = USER_M1_FAULT_CHECK_CURRENT_A;
    objSets->toqueFailMinSet_Nm = USER_M1_TORQUE_FAILED_SET;

    objSets->overVoltageFault_V = USER_M1_OVER_VOLTAGE_FAULT_V;
    objSets->overVoltageNorm_V = USER_M1_OVER_VOLTAGE_NORM_V;
    objSets->underVoltageFault_V = USER_M1_UNDER_VOLTAGE_FAULT_V;
    objSets->underVoltageNorm_V = USER_M1_UNDER_VOLTAGE_NORM_V;

    objSets->fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
    objSets->alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
    objSets->startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
    objSets->brakingCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    objSets->accelStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;
    objSets->accelStop_Hzps = USER_MOTOR1_ACCEL_STOP_Hzps;
    objSets->accelRun_Hzps = USER_MOTOR1_ACCEL_RUN_Hzps;
    objSets->speedFlyingStart_Hz = USER_MOTOR1_SPEED_FS_Hz;
    objSets->speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;
    objSets->speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;

    objSets->VsRef_pu = USER_M1_VS_REF_MAG_PU;
    objSets->Kp_fwc = USER_M1_FWC_KP;
    objSets->Ki_fwc = USER_M1_FWC_KI;
    objSets->angleFWCMax_rad = USER_M1_FWC_MAX_ANGLE_RAD;

    objSets->RsOnLineCurrent_A = 0.1f * USER_MOTOR1_MAX_CURRENT_A;
    objSets->RsOnLine_Rdelta_Ohm = 0.00002f;
    objSets->RsOnLine_Adelta_rad = 0.0005f;

    objSets->fluxFilterCoef = USER_M1_EST_FLUX_HF_SF;
    objSets->speedFilterCoef = USER_M1_EST_FREQ_HF_SF;
    objSets->bemfFilterCoef = USER_M1_EST_BEMF_HF_SF;
    objSets->speedPole_rps = USER_M1_SPEED_POLE_rps;
    objSets->directionPole_rps = USER_M1_DIRECTION_POLE_rps;
    objSets->fluxPole_rps = USER_M1_FLUX_POLE_rps;

    objSets->esmo_FAST_fsw_Hz = USER_MOTOR1_FREQ_HIGH_Hz;
    objSets->esmo_KslideMax = USER_MOTOR1_KSLIDE_MAX;
    objSets->esmo_KslideMin = USER_MOTOR1_KSLIDE_MIN;
    objSets->esmo_LpfFc_Hz = USER_MOTOR1_SPEED_LPF_FC_Hz;
    objSets->esmo_filterFc_sf = USER_MOTOR1_BEMF_KSLF_FC_SF;
    objSets->esmo_E0 = USER_MOTOR1_BEMF_THRESHOLD;
    objSets->esmo_PLL_KpMax = USER_MOTOR1_PLL_KP_MAX;
    objSets->esmo_PLL_KpMin = USER_MOTOR1_PLL_KP_MIN;
    objSets->esmo_PLL_KpSF = USER_MOTOR1_PLL_KP_SF;
    objSets->esmo_PLL_Ki = USER_MOTOR1_PLL_KI;

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    objSets->hfi_Kspd = IPD_HFI_KSPD;
    objSets->hfi_excMag_coarse_V = IPD_HFI_EXC_MAG_COARSE_V;
    objSets->hfi_excMag_fine_V = IPD_HFI_EXC_MAG_FINE_V;
    objSets->hfi_waitTime_coarse_sec = IPD_HFI_EXC_TIME_COARSE_S;
    objSets->hfi_waitTime_fine_sec = IPD_HFI_EXC_TIME_FINE_S;
    objSets->hfi_excFreq_Hz = IPD_HFI_EXC_FREQ_HZ;
    objSets->hfi_LpfFcSpd_Hz = IPD_HFI_LP_SPD_FILT_HZ;
    objSets->hfi_HpfFcIq_Hz = IPD_HFI_HP_IQ_FILT_HZ;
    objSets->hfi_IqMaxHfi_A = AFSEL_MAX_IQ_REF_HFI_A;
    objSets->hfi_IqMaxEst_A = AFSEL_MAX_IQ_REF_EST_A;
    objSets->hfi_IqSlope_A = AFSEL_IQ_SLOPE_HFI_A;
    objSets->hfi_freqLow_Hz = AFSEL_FREQ_LOW_HZ;
    objSets->hfi_freqHigh_Hz = AFSEL_FREQ_HIGH_HZ;

    objSets->RsOnlineWaitTimeSet = USER_MOTOR1_RSONLINE_WAIT_TIME;
    objSets->RsOnlineWorkTimeSet = USER_MOTOR1_RSONLINE_WORK_TIME;
#endif  // _HSWFREQ_EN

    objSets->anglePLLDelayed_sf = 0.2f;     // 0.2f
    objSets->angleESTDelayed_sf = 0.5f;     // 0.5f

    objSets->restartWaitTimeSet = USER_M1_RESTART_WAIT_TIME_SET;
    objSets->stopWaitTimeSet = USER_M1_STOP_WAIT_TIME_SET;

    objSets->overSpeedTimeSet = USER_M1_OVER_SPEED_TIME_SET;
    objSets->unbalanceTimeSet = USER_M1_UNBALANCE_TIME_SET;
    objSets->lostPhaseTimeSet = USER_M1_LOST_PHASE_TIME_SET;
    objSets->restartTimesSet = USER_M1_START_TIMES_SET;

    objSets->faultMtrMask.all = MTR1_FAULT_MASK_SET;

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    objSets->bootChargeTimeSet = 0;
#endif  // _HSWFREQ_EN

    objSets->dacCMPValH = 2048U + 1024U;    // set default positive peak value
    objSets->dacCMPValL = 2048U - 1024U;    // set default negative peak value

    // set the driver parameters
    HAL_MTR_setParams(obj->halMtrHandle);

    objSets->Kp_spd = 0.05f;
    objSets->Ki_spd = 0.005f;


#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    objSets->windingTemp_sfInv = INV_WINDING_TEMP_COEF_C;
    objSets->windingOverTempMax = WINDING_OVER_TEMP_MAX;
    objSets->windingNormTempMin = WINDING_NORM_TEMP_MIN;
    objSets->motorOverTempTimeSet = WINDING_OVER_TEMP_TIME_SET;

    objSets->tempModuleCoef = USER_M1_MODULE_TEMP_COEF;
    objSets->tempModuleOffset = USER_M1_MODULE_TEMP_OFFSET;

    objSets->tempMotorCoef = USER_M1_MOTOR_TEMP_COEF;
    objSets->tempMotorOffset = USER_M1_MOTOR_TEMP_OFFSET;
#endif  // _HSWFREQ_EN & GUI_SCI_EN

    obj->accelerationSet_Hzps = USER_MOTOR1_ACCEL_RUN_Hzps;
    obj->accelerationMax_Hzps = USER_MOTOR1_ACCEL_RUN_Hzps;

    obj->operationMode = OPERATE_MODE_SPEED;
    obj->flyingStartMode = FLYINGSTART_MODE_HALT;
    obj->RsOnlineMode = RSONLINE_CONTINUE;


    obj->IsSet_A = 0.0f;
    obj->anglePhaseAdj_rad = MATH_PI * 0.001f;

    obj->stopWaitTimeCnt = 0;
    obj->flagEnableRestart = false;

    obj->faultMtrMask.all = MTR1_FAULT_MASK_SET;

    if(objUser->flag_bypassMotorId == true)
    {
#if defined(MOTOR1_DCLINKSS)    // Single shunt
        obj->svmMode = SVM_COM_C;
#else  // !(MOTOR1_DCLINKSS)    // 2/3 shunt
//        obj->svmMode = SVM_MIN_C;
        obj->svmMode = SVM_COM_C;
#endif  // !(MOTOR1_DCLINKSS)   // 2/3 shunt
        obj->flagEnableFWC = true;
    }
    else
    {
        obj->svmMode = SVM_COM_C;
        obj->flagEnableFWC = false;
    }

    obj->flagEnableForceAngle = true;

    // true - enables flying start, false - disables flying start
    obj->flagEnableFlyingStart = true;

    // true - enables SSIPD start, false - disables SSIPD
    obj->flagEnableSSIPD = false;

    obj->flagEnableSpeedCtrl = true;
    obj->flagEnableCurrentCtrl = true;

#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
#elif defined(MOTOR1_FAST)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = false;
#elif defined(MOTOR1_ESMO)
    obj->estimatorMode = ESTIMATOR_MODE_ESMO;

    obj->flagEnableAlignment = true;
#else   // Not select algorithm
#error Not select a right estimator for this project
#endif  // MOTOR1_FAST | MOTOR1_ESMO

    obj->speedAbs_Hz = 0.0f;
    obj->speedFilter_Hz = 0.0f;
    obj->speed_int_Hz = 0.0f;
    obj->speedPLL_Hz = 0.0f;
    obj->speedEST_Hz = 0.0f;
    obj->speed_Hz = 0.0f;

    obj->angleFOC_rad = 0.0f;
    obj->angleEST_rad = 0.0f;
    obj->anglePLL_rad = 0.0f;
    obj->angleGen_rad = 0.0f;

    // configure the speed reference trajectory (Hz)
    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);

    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);

#if defined(MOTOR1_FAST)
    // for Rs re-calculation
    obj->flagEnableRsRecalc = false;
    obj->flagEnableLsUpdate = false;

    // for Rs online calibration
    obj->flagStartRsOnLine = false;
#endif // MOTOR1_FAST


#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC) || defined(MOTOR1_DCLINKSS)
    // initialize the angle generate module
    ANGLE_GEN_setParams(obj->angleGenHandle, objSets->ctrlPeriod_sec);
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT || MOTOR1_ENC

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->Idq_set_A.value[0] = 0.0f;
    obj->Idq_set_A.value[1] = objSets->startCurrent_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    VS_FREQ_setVsMagPu(obj->VsFreqHandle, objSets->maxVsMag_pu);

    VS_FREQ_setMaxFreq(obj->VsFreqHandle, USER_MOTOR1_FREQ_MAX_Hz);

    VS_FREQ_setProfile(obj->VsFreqHandle,
                       USER_MOTOR1_FREQ_LOW_Hz, USER_MOTOR1_FREQ_HIGH_Hz,
                       USER_MOTOR1_VOLT_MIN_V, USER_MOTOR1_VOLT_MAX_V);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_FILTERIS)
    obj->flagEnableFilterIs = true;

    obj->filterIsPole_rps = USER_M1_IS_FILTER_POLE_rps;
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    obj->flagEnableFilterVs = true;

    obj->filterVsPole_rps = USER_M1_VS_FILTER_POLE_rps;
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    SVGENCURRENT_setup(obj->svgencurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM


#ifdef BRAKE_ENABLE

    obj->brakingCurrent_A = USER_MOTOR1_BRAKE_CURRENT_A;

    obj->brakingTimeDelay = USER_MOTOR1_BRAKE_TIME_DELAY;

    obj->flagEnableBraking = false;
    obj->brakingMode = HARDSWITCH_BRAKE_MODE;
#endif  // BRAKE_ENABLE


    // setup the coefficient of the controllers gains
    setupControllerSF(handle);

#if defined(BENCHMARK_TEST)
    bmarkTestVars.recordDataCount = 0;
    bmarkTestVars.recordTicksSet = 15;
#endif  // BENCHMARK_TEST

    return;
}   // end of initMotor1CtrlParameters() function

// update control parameters for motor 1
void resetMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    objSets->reserve_Prms1 = 0xFFFFU;
    objSets->reserve_Prms2 = 0xFFFFU;
#endif  // _HSWFREQ_EN & GUI_SCI_EN

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    objSets->hz2Rpm_sf = 60.0f / objSets->numPolePairs;
#endif  // _HSWFREQ_EN & GUI_SCI_EN

    objSets->pwmPeriod_usec = 1000.0f / objSets->pwmControl_kHz;
    objSets->ctrlPeriod_sec = (objSets->pwmPeriod_usec / objSets->controlTicksPWM) *
                              ( 1.0f / 1000.0f/ 1000.0f);

    objSets->maxAccel_Hzps = objSets->accelRun_Hzps;
    objSets->alignTimeDelay = (uint16_t)(objSets->ctrlFreq_Hz * 0.1f);      // 0.1s

    objSets->currentInv_sf = (4096.0f / objSets->currentScale_A);

    objSets->voltage_sf = objSets->voltageScale_V * ADC_ONE_OVER_FULL_RANGE;
    objSets->current_sf = objSets->currentScale_A * ADC_ONE_OVER_FULL_RANGE;
    obj->adcData.current_sf = objSets->current_sf * USER_M1_SIGN_CURRENT_SF;

    if(obj->adcData.current_sf > 0.0f)
    {
        motorVars_M1.CurrentSenDir = CS_DIR_POSTIVE;
    }
    else
    {
        motorVars_M1.CurrentSenDir = CS_DIR_NEGATIVE;
    }

    obj->adcData.voltage_sf = objSets->voltage_sf;
    obj->adcData.dcBusvoltage_sf = objSets->voltage_sf;

    obj->VIrmsIsrScale = objSets->ctrlFreq_Hz;
    obj->power_sf = MATH_TWO_PI / objSets->numPolePairs;

    objSets->maxCurrent_A = objSets->maxCurrentSet_A;

    // initialize the user parameters
    USER_setMotor1Params(obj->userParamsHandle, obj->motorSetsHandle);

    obj->maxVsMag_pu = objUser->maxVsMag_pu;

    obj->numCtrlTicksPerSpeedTick = objUser->numCtrlTicksPerSpeedTick;

#if defined(MOTOR1_POWCTRL)
    obj->numCtrlTicksPerPowerTick = objUser->numCtrlTicksPerSpeedTick;
#endif  // MOTOR1_POWCTRL

    obj->VsRef_pu = objSets->VsRef_pu;
    obj->VsRef_V = obj->VsRef_pu * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;
    obj->IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;

    HAL_setNumCurrentSensors(obj->halMtrHandle, objUser->numCurrentSensors);
    HAL_setNumVoltageSensors(obj->halMtrHandle, objUser->numVoltageSensors);

    // set the Clarke parameters
    setupClarke_I(obj->clarkeHandle_I, objUser->numCurrentSensors);

    TRAJ_setMinValue(obj->trajHandle_spd, -objSets->maxFrequency_Hz);
    TRAJ_setMaxValue(obj->trajHandle_spd, objSets->maxFrequency_Hz);
    TRAJ_setMaxDelta(obj->trajHandle_spd, (objSets->maxAccel_Hzps * objUser->ctrlPeriod_sec));

#if defined(MOTOR1_ESMO)
    // set parameters for ESMO controller
    ESMO_setKslideParams(obj->esmoHandle,
                         USER_MOTOR1_KSLIDE_MAX, USER_MOTOR1_KSLIDE_MIN);

    ESMO_setPLLParams(obj->esmoHandle, USER_MOTOR1_PLL_KP_MAX,
                      USER_MOTOR1_PLL_KP_MIN, USER_MOTOR1_PLL_KP_SF);

    ESMO_setPLLKi(obj->esmoHandle, USER_MOTOR1_PLL_KI);   // Optional

    ESMO_setBEMFThreshold(obj->esmoHandle, USER_MOTOR1_BEMF_THRESHOLD);
    ESMO_setOffsetCoef(obj->esmoHandle, USER_MOTOR1_THETA_OFFSET_SF);
    ESMO_setBEMFKslfFreq(obj->esmoHandle, USER_MOTOR1_BEMF_KSLF_FC_SF);
    ESMO_setSpeedFilterFreq(obj->esmoHandle, USER_MOTOR1_SPEED_LPF_FC_Hz);

    // set the ESMO controller parameters
    ESMO_setParams(obj->esmoHandle, obj->userParamsHandle);

    // set the spdfr parameters
    SPDFR_setParams(obj->spdfrHandle, obj->userParamsHandle);

    obj->anglePLLDelayed_sf = objSets->anglePLLDelayed_sf * MATH_TWO_PI * objSets->ctrlPeriod_sec;
    obj->frswPos_sf = 0.6f;
#endif  //MOTOR1_ESMO

#if defined(MOTOR1_FAST)
    // initialize the estimator
    // set the default estimator parameters
    EST_setParams(obj->estHandle, obj->userParamsHandle);
    EST_setFlag_enableForceAngle(obj->estHandle, obj->flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(obj->estHandle, obj->flagEnableRsRecalc);

    // set the scale factor for high frequency low inductance motor
    EST_setOneOverFluxGain_sf(obj->estHandle,
                              obj->userParamsHandle, USER_M1_EST_FLUX_HF_SF);
    EST_setFreqLFP_sf(obj->estHandle,
                      obj->userParamsHandle, USER_M1_EST_FREQ_HF_SF);
    EST_setBemf_sf(obj->estHandle,
                   obj->userParamsHandle, USER_M1_EST_BEMF_HF_SF);

    objSets->Ls_d_comp_H = EST_getLs_d_H(obj->estHandle);
    objSets->Ls_q_comp_H = EST_getLs_q_H(obj->estHandle);

    // if motor is an induction motor, configure default state of PowerWarp
    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
    {
        EST_setFlag_enablePowerWarp(obj->estHandle, obj->flagEnablePowerWarp);
        EST_setFlag_bypassLockRotor(obj->estHandle, obj->flagBypassLockRotor);
    }

    // set the Clarke parameters
    setupClarke_V(obj->clarkeHandle_V, objUser->numVoltageSensors);
#endif // MOTOR1_FAST

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    DCLINK_SS_setInitialConditions(obj->dclinkHandle,
                                   HAL_getTimeBasePeriod(obj->halMtrHandle), 0.5f);

    //disable full sampling
//    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, false);     // default
    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, true);    // test, not recommend in most cases

    //enable sequence control
//    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, false);  // default
    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, true); // test, not recommend in most cases

    // Tdt  =  55 ns (Dead-time between top and bottom switch)
    // Tpd  = 140 ns (Gate driver propagation delay)
    // Tr   = 136 ns (Rise time of amplifier including power switches turn on time)
    // Ts   = 800 ns (Settling time of amplifier)
    // Ts&h = 100 ns (ADC sample&holder = 1+(9)+2 = 12 SYSCLK)
    // T_MinAVDuration = Tdt+Tr+Tpd+Ts+Ts&h
    //                 = 55+140+136+800+100 = 1231(ns) => 148 SYSCLK cycles
    // T_SampleDelay   = Tdt+Tpd+Tr+Ts
    //                 = 55+140+136+800     = 1131(ns) => 136 SYSCLK cycles
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, USER_M1_DCLINKSS_MIN_DURATION);
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, USER_M1_DCLINKSS_SAMPLE_DELAY);

    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.0f, 0.0f, 0.0f);

    HAL_getPWMPeriod(obj->halMtrHandle, &obj->pwmData);
#else   // !MOTOR1_DCLINKSS     // three shunt
//    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
//                        0.01f, 0.01f, 0.14f);
    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.005f, 0.005f, 0.075f);

    HAL_getPWMPeriod(obj->halMtrHandle, &obj->pwmData);
#endif   // !MOTOR1_DCLINKSS    // single shunt

#ifdef MOTOR1_VOLRECT
    // configure the Voltage reconstruction
    VOLREC_setParams(obj->volrecHandle,
                     objUser->voltageFilterPole_rps,
                     objUser->ctrlFreq_Hz);

    VOLREC_disableFlagEnableSf(obj->volrecHandle);
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_FWC)
    // set the FWC controller
    PI_setGains(obj->piHandle_fwc, USER_M1_FWC_KP, USER_M1_FWC_KI);
    PI_setUi(obj->piHandle_fwc, 0.0);
    PI_setMinMax(obj->piHandle_fwc, USER_M1_FWC_MAX_ANGLE_RAD,
                 USER_M1_FWC_MIN_ANGLE_RAD);
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // compute the motor constant for MTPA
    MTPA_computeParameters(obj->mtpaHandle,
                           objUser->motor_Ls_d_H,
                           objUser->motor_Ls_q_H,
                           objUser->motor_ratedFlux_Wb);
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_FILTERIS)
    float32_t beta_lp_Is = obj->filterIsPole_rps * objSets->ctrlPeriod_sec;

    float32_t a1_Is = (beta_lp_Is - (float32_t)2.0f) / (beta_lp_Is + (float32_t)2.0f);
    float32_t b0_Is = beta_lp_Is / (beta_lp_Is + (float32_t)2.0f);
    float32_t b1_Is = b0_Is;

    // set filter coefficients for current filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[0], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[0], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[1], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[1], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[2], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[2], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    float32_t beta_lp_Vs = obj->filterVsPole_rps * objSets->ctrlPeriod_sec;

    float32_t a1_Vs = (beta_lp_Vs - (float32_t)2.0f) / (beta_lp_Vs + (float32_t)2.0f);
    float32_t b0_Vs = beta_lp_Vs / (beta_lp_Vs + (float32_t)2.0f);
    float32_t b1_Vs = b0_Vs;

    // set filter coefficients for voltage filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[0], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[0], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[1], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[1], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[2], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[2], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    SVGENCURRENT_setup(obj->svgencurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM


#ifdef BRAKE_ENABLE

    obj->brakingCurrent_A = USER_MOTOR1_BRAKE_CURRENT_A;

    obj->brakingTimeDelay = USER_MOTOR1_BRAKE_TIME_DELAY;

    obj->flagEnableBraking = false;
    obj->brakingMode = HARDSWITCH_BRAKE_MODE;
#endif  // BRAKE_ENABLE


    // setup the controllers, speed, d/q-axis current pid regulator
    setupControllers(handle);

    // update the coefficient of the controllers gains
    updateControllerSF(handle);

#if defined(MOTOR1_FAST)
    obj->flagMotorIdentified = EST_isMotorIdentified(obj->estHandle);
#endif  // MOTOR1_FAST

    obj->flagEnableTuneController = true;

    return;
}   // end of initMotor1CtrlParameters() function

void runMotor1OffsetsCalculation(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    HAL_MTR_Obj *objHal = (HAL_MTR_Obj *)(obj->halMtrHandle);

    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, 5);

    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, 5);
#endif  // MOTOR1_DCLINKSS  // single shunt

        // Offsets in phase current sensing
#if defined(MOTOR1_DCLINKSS)    // Single shunt
    ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                              MTR1_IDC1_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                              MTR1_IDC2_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                              MTR1_IDC3_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                              MTR1_IDC4_ADC_PPB_NUM);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#else // !(MOTOR1_DCLINKSS)     // 2/3 shunt
    ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                              USER_M1_IA_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                              USER_M1_IB_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                              USER_M1_IC_OFFSET_AD);

    obj->adcData.offset_I_ad.value[0]  = USER_M1_IA_OFFSET_AD;
    obj->adcData.offset_I_ad.value[1]  = USER_M1_IB_OFFSET_AD;
    obj->adcData.offset_I_ad.value[2]  = USER_M1_IC_OFFSET_AD;
#endif // !( MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST)
    // Offsets in phase voltage sensing
    obj->adcData.offset_V_sf.value[0]  = USER_M1_VA_OFFSET_SF;
    obj->adcData.offset_V_sf.value[1]  = USER_M1_VB_OFFSET_SF;
    obj->adcData.offset_V_sf.value[2]  = USER_M1_VC_OFFSET_SF;
#endif  // MOTOR1_FAST

    // calculate motor protection value
    calcMotorOverCurrentThreshold(handle);

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagEnableOffsetCalc == true)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
        float32_t invCurrentSf = 1.0f / obj->adcData.current_sf;

#if defined(MOTOR1_FAST)
        float32_t invVdcbus;
#endif  // MOTOR1_FAST

        uint16_t offsetCnt;

        DEVICE_DELAY_US(2.0f);      // delay 2us

#if defined(MOTOR1_DCLINKSS)    // Single shunt
        HAL_setOffsetTrigger(obj->halMtrHandle);

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, 0);

        obj->adcData.offset_Idc_ad  = USER_M1_IDC_OFFSET_AD * USER_M1_CURRENT_SF;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

#else  // !(MOTOR1_DCLINKSS)        // 2/3 shunt
        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, 0);

        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * obj->adcData.current_sf;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif // !(MOTOR1_DCLINKSS)        // 2/3 shunt
        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);

        for(offsetCnt = 0; offsetCnt < 21000; offsetCnt++)
        {
//            offsetCnt = 20000;

            // clear the ADC interrupt flag
            ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

            while(ADC_getInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM) == false);

            HAL_readMtr1ADCData(&obj->adcData);

            if(offsetCnt >= 1000)       // Ignore the first 1000 times
            {
                // Offsets in phase current sensing
#if defined(MOTOR1_DCLINKSS)    // Single shunt
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                               0.25f * offsetK2 *(obj->adcData.Idc1_A.value[0] +
                                                  obj->adcData.Idc1_A.value[1] +
                                                  obj->adcData.Idc2_A.value[0] +
                                                  obj->adcData.Idc2_A.value[1]);
#else // !(MOTOR1_DCLINKSS)         // 2/3 shunt
                obj->adcData.offset_I_ad.value[0] =
                        offsetK1 * obj->adcData.offset_I_ad.value[0] +
                        obj->adcData.I_A.value[0] * offsetK2;

                obj->adcData.offset_I_ad.value[1] =
                        offsetK1 * obj->adcData.offset_I_ad.value[1] +
                        obj->adcData.I_A.value[1] * offsetK2;

                obj->adcData.offset_I_ad.value[2] =
                        offsetK1 * obj->adcData.offset_I_ad.value[2] +
                        obj->adcData.I_A.value[2] * offsetK2;
#endif // !(MOTOR1_DCLINKSS)        // 2/3 shunt

#if defined(MOTOR1_FAST)
                invVdcbus = 1.0f / obj->adcData.VdcBus_V;

                // Offsets in phase voltage sensing
                obj->adcData.offset_V_sf.value[0] =
                         offsetK1 * obj->adcData.offset_V_sf.value[0] +
                         (invVdcbus * obj->adcData.V_V.value[0]) * offsetK2;

                obj->adcData.offset_V_sf.value[1] =
                         offsetK1 * obj->adcData.offset_V_sf.value[1] +
                         (invVdcbus * obj->adcData.V_V.value[1]) * offsetK2;

                obj->adcData.offset_V_sf.value[2] =
                         offsetK1 * obj->adcData.offset_V_sf.value[2] +
                         (invVdcbus * obj->adcData.V_V.value[2]) * offsetK2;
#endif  // MOTOR1_FAST
            }
            else if(offsetCnt <= 150)
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);

#if defined(MOTOR1_DCLINKSS)    // Single shunt
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);
#else // !(MOTOR1_DCLINKSS)     // 2/3 shunt
        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * invCurrentSf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * invCurrentSf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[0]);

        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[1]);

        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[2]);
#endif // !(MOTOR1_DCLINKSS)    // 2/3 shunt
    }   // flagEnableOffsetCalc = true

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

#if defined(SAFETY_ENABLE)
    obj->adcData.offsetIdcFilter = (uint16_t)obj->adcData.offset_Idc_ad;
#endif  // SAFETY_ENABLE
#else // !(MOTOR1_DCLINKSS)     // 2/3 shunt
    // Check current and voltage offset
    if( (obj->adcData.offset_I_ad.value[0] > USER_M1_IA_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[0] < USER_M1_IA_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[1] > USER_M1_IB_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[1] < USER_M1_IB_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[2] > USER_M1_IC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[2] < USER_M1_IC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

#if defined(SAFETY_ENABLE)
    obj->adcData.offsetIsFilter[0] = (uint16_t)obj->adcData.offset_I_ad.value[0];
    obj->adcData.offsetIsFilter[1] = (uint16_t)obj->adcData.offset_I_ad.value[1];
    obj->adcData.offsetIsFilter[2] = (uint16_t)obj->adcData.offset_I_ad.value[2];
#endif  // SAFETY_ENABLE
#endif // !(MOTOR1_DCLINKSS)    // 2/3 shunt

#if defined(MOTOR1_FAST)
    if( (obj->adcData.offset_V_sf.value[0] > USER_M1_VA_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[0] < USER_M1_VA_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[1] > USER_M1_VB_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[1] < USER_M1_VB_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[2] > USER_M1_VC_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[2] < USER_M1_VC_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }
#endif  // MOTOR1_FAST

    if((obj->faultMtrNow.bit.voltageOffset == 0) &&
            (obj->faultMtrNow.bit.currentOffset == 0))
    {
        obj->flagEnableOffsetCalc = false;
    }

    return;
} // end of runMotor1OffsetsCalculation() function


// always call  this function in the background loop when the background is free
void runMotor1Control(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        if(HAL_getMtrTripFaults(obj->halMtrHandle) != 0)
        {
            obj->faultMtrNow.bit.moduleOverCurrent = 1;
        }
    }

    obj->faultMtrPrev.all |= obj->faultMtrNow.all;
    obj->faultMtrUse.all = obj->faultMtrNow.all & obj->faultMtrMask.all;

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagClearFaults == true)
    {
        HAL_clearMtrFaultStatus(obj->halMtrHandle);

        obj->faultMtrNow.all &= MTR_FAULT_CLEAR;
        obj->flagClearFaults = false;
    }

    // convert the feedback speed to rpm
#if defined(_HSWFREQ_EN)
    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);
#endif  // _HSWFREQ_EN

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    obj->speed_rpm = obj->speed_Hz * objSets->hz2Rpm_sf;
#endif  // _HSWFREQ_EN & GUI_SCI_EN

    if(obj->flagEnableRunAndIdentify == true)
    {
        obj->speedRef_Hz = obj->speedSet_Hz;
        obj->accelerationMax_Hzps = obj->accelerationSet_Hzps;

        // Had some faults to stop the motor
        if(obj->faultMtrUse.all != 0)
        {
            if(obj->flagRunIdentAndOnLine == true)
            {
                obj->flagRunIdentAndOnLine = false;
                obj->controlStatus = MOTOR_FAULT_STOP;

                obj->stopWaitTimeCnt = objSets->restartWaitTimeSet;
                obj->restartTimesCnt++;

                if(obj->flagEnableRestart == false)
                {
                    obj->flagEnableRunAndIdentify = false;
                    obj->stopWaitTimeCnt = 0;
                }
            }
            else if(obj->stopWaitTimeCnt == 0)
            {
                if(obj->restartTimesCnt < objSets->restartTimesSet)
                {
                    obj->flagClearFaults = 1;
                }
                else
                {
                    obj->flagEnableRunAndIdentify = false;
                }
            }
        }
        // Restart
        else if((obj->flagRunIdentAndOnLine == false) &&
                (obj->stopWaitTimeCnt == 0))
        {
            restartMotorControl(handle);
        }
    }
    // if(obj->flagEnableRunAndIdentify == false)
    else if(obj->flagRunIdentAndOnLine == true)
    {
        obj->speedRef_Hz = 0.0f;

        stopMotorControl(handle);

        if(obj->flagEnableFlyingStart == false)
        {
            obj->stopWaitTimeCnt = objSets->stopWaitTimeSet;
        }
        else
        {
            obj->stopWaitTimeCnt = 0;
        }
    }

#if defined(MOTOR1_FAST)
    // enable or disable bypassLockRotor flag
    if((objUser->motor_type == MOTOR_TYPE_INDUCTION)
        && (obj->flagMotorIdentified == true))
    {
        EST_setFlag_bypassLockRotor(obj->estHandle,
                                    obj->flagBypassLockRotor);
    }
#endif // MOTOR1_FAST

    if(obj->flagRunIdentAndOnLine == true)      // Start the motor
    {
        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
        {
#if defined(MOTOR1_FAST)
            // enable the estimator
            EST_enable(obj->estHandle);

            // enable the trajectory generator
            EST_enableTraj(obj->estHandle);
#endif // MOTOR1_FAST

            // enable the PWM
            HAL_enablePWM(obj->halMtrHandle);
        }   // (HAL_getPwmEnableStatus(obj->halMtrHandle) == false)

#if defined(MOTOR1_FAST)
        if(obj->flagMotorIdentified == true)
#endif  // MOTOR1_FAST
        {

            if(obj->speedRef_Hz > 0.0f)
            {
                obj->direction = 1.0f;
            }
            else
            {
                obj->direction = -1.0f;
            }

        #if defined(MOTOR1_FAST)
            // enable or disable force angle
            EST_setFlag_enableForceAngle(obj->estHandle,
                                         obj->flagEnableForceAngle);

            // enable or disable stator resistance (Rs) re-calculation
            EST_setFlag_enableRsRecalc(obj->estHandle,
                                       obj->flagEnableRsRecalc);
        #endif  // MOTOR1_FAST

            // Sets the target speed for the speed trajectory
        #if defined(MOTOR1_ESMO)
            if(obj->controlStatus >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (objSets->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_FAST)
            TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
        #else   // !MOTOR1_ESMO && !MOTOR1_FAST
        #error No select a right estimator for motor_1 control
        #endif  // MOTOR1_ESMO || MOTOR1_FAST

            if((fabsf(obj->speed_Hz) > objSets->speedStart_Hz) ||
                    (obj->controlStatus == MOTOR_CTRL_RUN))
            {
                // Inject the current for debugging
                obj->IdInj_A = objSets->IdInj_A;
                obj->IqInj_A = objSets->IqInj_A;

                obj->accelerationMax_Hzps = obj->accelerationSet_Hzps;

                if(obj->controlStatus == MOTOR_CL_RUNNING)
                {
                    if(obj->stateRunTimeCnt == objSets->startupTimeDelay)
                    {
                        obj->Idq_out_A.value[0] = 0.0f;
                        obj->controlStatus = MOTOR_CTRL_RUN;
                    }
                }

#if defined(MOTOR1_FAST) && defined(MOTOR1_LS_CAL)
                if(obj->flagEnableLsUpdate ==  true)
                {
                    // Calculate the Ld and Lq which reduce with current
                    objSets->Ls_d_comp_H = objUser->motor_Ls_d_H * (1.0f - obj->Is_A * objSets->Ls_d_Icomp_coef);
                    objSets->Ls_q_comp_H = objUser->motor_Ls_q_H * (1.0f - obj->Is_A * objSets->Ls_q_Icomp_coef);

                    if(objSets->Ls_d_comp_H < objSets->Ls_min_H)
                    {
                        objSets->Ls_d_comp_H = objSets->Ls_min_H;
                    }

                    if(objSets->Ls_q_comp_H < objSets->Ls_min_H)
                    {
                        objSets->Ls_q_comp_H = objSets->Ls_min_H;
                    }

                    // Update the Ld and Lq for motor control
                    EST_setLs_d_H(obj->estHandle, objSets->Ls_d_comp_H);
                    EST_setLs_q_H(obj->estHandle, objSets->Ls_q_comp_H);
                }
#endif //MOTOR1_FAST & MOTOR1_LS_CAL

#if defined(MOTOR1_POWCTRL)
                if(obj->flagEnablePowerCtrl == true)
                {
                    if(obj->controlStatus == MOTOR_CTRL_RUN)
                    {
                        PI_setMinMax(obj->piHandle_pow, -objSets->maxCurrent_A, objSets->maxCurrent_A);
                    }
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -objSets->maxCurrent_A, objSets->maxCurrent_A);
                }
#else   // !MOTOR1_POWCTRL
                PI_setMinMax(obj->piHandle_spd, -objSets->maxCurrent_A, objSets->maxCurrent_A);
#endif  // MOTOR1_POWCTRL

                SVGEN_setMode(obj->svgenHandle, obj->svmMode);
            }
            else    // Start to run the motor
            {
                obj->accelerationMax_Hzps = objSets->accelStart_Hzps;

#if defined(MOTOR1_POWCTRL)
                if(obj->flagEnablePowerCtrl == true)
                {
                    PI_setMinMax(obj->piHandle_pow, -obj->IsSet_A, obj->IsSet_A);
                    PI_setMinMax(obj->piHandle_spd, -obj->IsSet_A, obj->IsSet_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->IsSet_A, obj->IsSet_A);
                }
#else   // !MOTOR1_POWCTRL
                if(obj->speed_int_Hz >= 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->IsSet_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->IsSet_A, 0.0f);
                }
#endif  // MOTOR1_POWCTRL
            }

            //  Sets the acceleration / deceleration for the speed trajectory
            TRAJ_setMaxDelta(obj->trajHandle_spd,
              (obj->accelerationMax_Hzps * objUser->ctrlPeriod_sec));
        }   // obj->flagMotorIdentified == true
        // Identification
#if(DMC_BUILDLEVEL == DMC_LEVEL_3)
        obj->Idq_out_A.value[0] = obj->Idq_set_A.value[0];
        obj->Idq_out_A.value[1] = obj->Idq_set_A.value[1] * obj->direction;

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)
    }
    else    // (obj->flagRunIdentAndOnLine == false) 
    {
        // reset motor control parameters
        resetMotorControl(handle);
    }       // (obj->flagRunIdentAndOnLine == false) 

#if defined(MOTOR1_FAST)
    // check the trajectory generator
    if(EST_isTrajError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else    // (EST_isTrajError(obj->estHandle) == false)
    {
        // update the trajectory generator state
        EST_updateTrajState(obj->estHandle);
    }       // (EST_isTrajError(obj->estHandle) == false)

    // check the estimator
    if(EST_isError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else    // (EST_isError(obj->estHandle) == false)
    {
        bool flagEstStateChanged = false;

        float32_t Id_target_A = EST_getIntValue_Id_A(obj->estHandle);

        if(obj->flagMotorIdentified == true)
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, 0.0f);
        }
        else    // obj->flagMotorIdentified = false
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, Id_target_A);
        }       // obj->flagMotorIdentified = false

        if(flagEstStateChanged == true)
        {
            // configure the trajectory generator, enter once every state
            EST_configureTraj(obj->estHandle);

            if(obj->flagMotorIdentified == false)
            {
                // configure the controllers, enter once every state
                EST_configureTrajState(obj->estHandle, obj->userParamsHandle,
                                       obj->piHandle_spd,
                                       obj->piHandle_Id, obj->piHandle_Iq);
                if((EST_isLockRotor(obj->estHandle) == true) ||
                        ( (EST_isMotorIdentified(obj->estHandle) == true)
                                  && (EST_isIdle(obj->estHandle) == true) ) )
                {
                    if(EST_isMotorIdentified(obj->estHandle) == true)
                    {
                        obj->flagMotorIdentified = true;

                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;

                        obj->controlStatus = MOTOR_STOP_IDLE;

                        // disable the estimator
                        EST_disable(obj->estHandle);

                        // enable the trajectory generator
                        EST_disableTraj(obj->estHandle);

                        obj->faultMtrUse.all = 0x0000;
                        obj->faultMtrNow.all = 0x0000;
                    }

                    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
                    {
                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;
                    }
                }   // obj->flagMotorIdentified
            }   // objUser->flag_bypassMotorId = false
        }
    }   // (EST_isError(obj->estHandle) == false)

#else   // !MOTOR1_FAST
    obj->flagMotorIdentified = true;
#endif //  !MOTOR1_FAST

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagSetupController == true)
        {
            // update the controller
            updateControllers(handle);
        }
        else
        {
            obj->flagSetupController = true;

            setupControllers(handle);
        }
    }


#if defined(MOTOR1_FAST)
    // run Rs online
    runRsOnLine(handle);
#endif // MOTOR1_FAST

    // update the global variables
    updateGlobalVariables(handle);

#if defined(MOTOR1_ESMO)
    if(obj->controlStatus >= MOTOR_CTRL_RUN)
    {
        ESMO_updateFilterParams(obj->esmoHandle);
        ESMO_updatePLLParams(obj->esmoHandle);
    }
#endif  // MOTOR2_ESMO

    return;
}   // end of the runMotor1Control() function

__interrupt void motor1CtrlISR(void)
{
#if defined(TEST_ENABLE)
    HAL_setGPIOHigh(ISR_CHECK_GPIO);
#endif  // TEST_ENABLE


    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)motorHandle_M1;

#if !defined(_HSWFREQ_EN) || defined(MOTOR1_ESMO)
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
#endif  // _HSWFREQ_EN

#if !defined(_HSWFREQ_EN)
    motorVars_M1.ISRCount++;
#endif  // !_HSWFREQ_EN

    // acknowledge the ADC interrupt
    HAL_ackMtr1ADCInt();

    // read the ADC data with offsets
    HAL_readMtr1ADCData(&obj->adcData);
//------------------------------------------------------------------------------
// 180-degree Sinusoidal Sensorless-FOC
//******************************************************************************
#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // run single-shunt current reconstruction
    DCLINK_SS_runCurrentReconstruction(obj->dclinkHandle,
                                     &obj->adcData.Idc1_A, &obj->adcData.Idc2_A);

    obj->adcData.I_A.value[0] = DCLINK_SS_getIa(obj->dclinkHandle);
    obj->adcData.I_A.value[1] = DCLINK_SS_getIb(obj->dclinkHandle);
    obj->adcData.I_A.value[2] = DCLINK_SS_getIc(obj->dclinkHandle);

#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->adcData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->adcData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->adcData.I_A.value[2]);

    if(obj->flagEnableFilterIs == true)
    {
        obj->adcData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->adcData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->adcData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS
#else // !(MOTOR1_DCLINKSS) // 2/3 shunt
#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->adcData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->adcData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->adcData.I_A.value[2]);

    if(obj->flagEnableFilterIs == true)
    {
        obj->adcData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->adcData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->adcData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_OVM)
    if(obj->flagEnableOVM == true)
    {
        // Over Modulation Supporting, run the current reconstruction algorithm
        SVGENCURRENT_RunRegenCurrent(obj->svgencurrentHandle,
                                     &obj->adcData.I_A, &obj->adcDataPrev);
    }
#endif  // MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)    // 2/3 shunt

#if defined(GUI_SCI_EN)
    // Transmit and Receive SCI Data
    guiTRxWaitTimeCnt++;

    if(guiEnableTRx == true)
    {
        if(guiTRxWaitTimeCnt >= guiTRxWaitTimeSet)
        {
            guiTRxWaitTimeCnt = 0;

            // Update transmit data
            GUI_updateTransmitData();
            // Transmit SCI Data
            GUI_writeTxData();
        }
    }

    // Receive SCI Data
    GUI_readRxData();

#endif  // GUI_SCI_EN
#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)    // (OK<->OK)
    // sensorless-FOC
    MATH_Vec2 phasor;
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#if defined(MOTOR1_VOLRECT)
    VOLREC_run(obj->volrecHandle, obj->adcData.VdcBus_V,
               &(obj->pwmData.Vabc_pu), &(obj->estInputData.Vab_V));
#else  // !MOTOR1_VOLRECT

    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

#if defined(MOTOR1_FILTERVS)
    // run first order filters for voltage sensing
    obj->adcVs_V.value[0] = FILTER_FO_run(obj->filterHandle_Vs[0], obj->adcData.V_V.value[0]);
    obj->adcVs_V.value[1] = FILTER_FO_run(obj->filterHandle_Vs[1], obj->adcData.V_V.value[1]);
    obj->adcVs_V.value[2] = FILTER_FO_run(obj->filterHandle_Vs[2], obj->adcData.V_V.value[2]);

    if(obj->flagEnableFilterVs == true)
    {
        obj->adcData.V_V.value[0] = obj->adcVs_V.value[0];
        obj->adcData.V_V.value[1] = obj->adcVs_V.value[1];
        obj->adcData.V_V.value[2] = obj->adcVs_V.value[2];
    }
#endif  // MOTOR1_FILTERVS

    // run Clarke transform on voltage
    CLARKE_run_threeInput(obj->clarkeHandle_V,
                          &obj->adcData.V_V, &obj->estInputData.Vab_V);
#endif  // !MOTOR1_VOLRECT
    // run Clarke transform on current
    CLARKE_run_threeInput(obj->clarkeHandle_I,
                          &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {
        obj->Idq_out_A.value[0] = 0.0f;
        obj->controlStatus = MOTOR_CTRL_RUN;

        // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

#if defined(MOTOR1_POWCTRL)
        obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL
    }
    else if(obj->flagMotorIdentified == true)   // Normal Running
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

#if defined(MOTOR1_POWCTRL)
            if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
            {
                obj->enablePowerCtrl = obj->flagEnablePowerCtrl;
            }
#endif  // MOTOR1_POWCTRL

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
#if defined(MOTOR1_POWCTRL)
            obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;

    // run the FAST estimator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleESTCOMP_rad =
            obj->angleESTDelayed_sf * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    // For eSMO
    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;

    // run the eSMO
//  ESMO_setSpeedRef(obj->esmoHandle, obj->speed_Hz);
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->estInputData.Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->anglePLLDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->controlStatus >= MOTOR_CTRL_RUN)
        {
            obj->angleFOC_rad = obj->angleEST_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->controlStatus == MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->controlStatus == MOTOR_OL_START)
        {
#if defined(MOTOR1_DCLINKSS)    // Single shunt
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = objSets->startCurrent_A;
                obj->Idq_out_A.value[1] = objSets->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -objSets->startCurrent_A;
                obj->Idq_out_A.value[1] = -objSets->startCurrent_A;
            }

            if(fabsf(obj->estInputData.speed_ref_Hz) >= objSets->speedForce_Hz)
            {
                TRAJ_setIntValue(obj->trajHandle_spd, obj->estInputData.speed_ref_Hz);

                if(obj->stateRunTimeCnt > objSets->forceRunTimeDelay)
                {
//                    EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    obj->controlStatus = MOTOR_CL_RUNNING;
                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }

#else   // !(MOTOR1_DCLINKSS)   // 2/3 shunt
            obj->angleFOC_rad = obj->angleEST_rad;
            obj->controlStatus = MOTOR_CL_RUNNING;

            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
#endif  // !(MOTOR1_DCLINKSS)   // 2/3 shunt
        }
        else if(obj->controlStatus == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = objSets->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > objSets->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->controlStatus = MOTOR_OL_START;
                obj->Idq_out_A.value[0] = objSets->fluxCurrent_A;


                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->controlStatus == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > objSets->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;
                obj->counterSpeed = 0;

                if(obj->speedAbs_Hz > objSets->speedFlyingStart_Hz)
                {
                    if(obj->speedRef_Hz > 0.0f)
                    {
                        obj->speed_int_Hz = obj->speed_Hz + 10.0f;
                    }
                    else
                    {
                        obj->speed_int_Hz = obj->speed_Hz - 10.0f;
                    }

                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->controlStatus = MOTOR_CL_RUNNING;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
                }
                else
                {
                    obj->controlStatus = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ESMO)
    {
        obj->speed_Hz = obj->speedPLL_Hz;

        if(obj->controlStatus >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->anglePLL_rad;

            ESMO_updateKslide(obj->esmoHandle);

#if defined(MOTOR1_POWCTRL)
            if(obj->controlStatus == MOTOR_CTRL_RUN)
            {
                obj->enablePowerCtrl = obj->flagEnablePowerCtrl;
            }
            else
            {
                obj->enablePowerCtrl = false;

                obj->IsPower_A = obj->IsRef_A;
                PI_setUi(obj->piHandle_pow, obj->IsPower_A);
            }
#endif  // MOTOR1_POWCTRL
        }
        else if(obj->controlStatus == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = objSets->startCurrent_A;
                obj->Idq_out_A.value[1] = objSets->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -objSets->startCurrent_A;
                obj->Idq_out_A.value[1] = -objSets->startCurrent_A;
            }

            if(fabsf(obj->estInputData.speed_ref_Hz) >= objSets->speedForce_Hz)
            {
                TRAJ_setIntValue(obj->trajHandle_spd, obj->estInputData.speed_ref_Hz);

                if(obj->stateRunTimeCnt > objSets->forceRunTimeDelay)
                {
                    obj->controlStatus = MOTOR_CL_RUNNING;

                    EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }
        }
        else if(obj->controlStatus == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = objSets->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
            ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > objSets->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->controlStatus = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = objSets->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->controlStatus == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > objSets->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > objSets->speedFlyingStart_Hz)
                {
                    if(obj->speedRef_Hz > 0.0f)
                    {
                        obj->speed_int_Hz = obj->speed_Hz + 10.0f;
                    }
                    else
                    {
                        obj->speed_int_Hz = obj->speed_Hz - 10.0f;
                    }

                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->controlStatus = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->controlStatus = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#ifdef MOTOR1_VOLRECT
    if(obj->controlStatus == MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->controlStatus == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(obj->speedAbs_Hz >= obj->speedForce_Hz)
        {
            obj->controlStatus = MOTOR_CL_RUNNING;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            PI_setUi(obj->piHandle_spd, obj->startCurrent_A * 0.5f);
        }
    }
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif
    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));
    // End of MOTOR1_FAST && MOTOR1_ESMO
//------------------------------------------------------------------------------
#elif defined(MOTOR1_ESMO)
    // sensorless-FOC
    MATH_Vec2 phasor;
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
    // run Clarke transform on current
    CLARKE_run_threeInput(obj->clarkeHandle_I,
                          &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

#if defined(MOTOR1_POWCTRL)
        if(obj->controlStatus == MOTOR_CTRL_RUN)
        {
            obj->enablePowerCtrl = obj->flagEnablePowerCtrl;
        }
        else
        {
            obj->enablePowerCtrl = false;

            obj->IsPower_A = obj->IsRef_A;
            PI_setUi(obj->piHandle_pow, obj->IsPower_A);
        }
#endif  // MOTOR1_POWCTRL
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
#if defined(MOTOR1_POWCTRL)
        obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;

    // run the eSMO
//  ESMO_setSpeedRef(obj->esmoHandle, obj->speed_Hz);
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->anglePLLDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);
    obj->speed_Hz = obj->speedPLL_Hz;

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->stateRunTimeCnt++;

    if(obj->controlStatus >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->anglePLL_rad;

        ESMO_updateKslide(obj->esmoHandle);
    }
    else if(obj->controlStatus == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = objSets->startCurrent_A;
            obj->Idq_out_A.value[1] = objSets->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -objSets->startCurrent_A;
            obj->Idq_out_A.value[1] = -objSets->startCurrent_A;
        }

        if(fabsf(obj->speed_int_Hz) >= objSets->speedForce_Hz)
        {
            TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

            if(obj->stateRunTimeCnt > objSets->forceRunTimeDelay)
            {
                obj->controlStatus = MOTOR_CL_RUNNING;
                obj->stateRunTimeCnt = 0;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));

#if defined(MOTOR1_POWCTRL)
                obj->IsPower_A = obj->IsRef_A;
                PI_setUi(obj->piHandle_pow, obj->IsPower_A);
#endif  // MOTOR1_POWCTRL
            }
        }
    }
    else if(obj->controlStatus == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = objSets->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

        if((obj->stateRunTimeCnt > objSets->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->controlStatus = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;

            obj->Idq_out_A.value[0] = objSets->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->controlStatus == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > objSets->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > objSets->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->controlStatus = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->controlStatus = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif
    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));
// End of MOTOR1_ESMO

//------------------------------------------------------------------------------
#elif defined(MOTOR1_FAST)
    // sensorless-FOC
    MATH_Vec2 phasor;

#if ((DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT)) || defined(MOTOR1_DCLINKSS)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // ((DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT))

#if defined(MOTOR1_VOLRECT)
    VOLREC_run(obj->volrecHandle, obj->adcData.VdcBus_V,
               &(obj->pwmData.Vabc_pu), &(obj->estInputData.Vab_V));
#else  // !MOTOR1_VOLRECT
    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

#if defined(MOTOR1_FILTERVS)
    // run first order filters for voltage sensing
    obj->adcVs_V.value[0] = FILTER_FO_run(obj->filterHandle_Vs[0], obj->adcData.V_V.value[0]);
    obj->adcVs_V.value[1] = FILTER_FO_run(obj->filterHandle_Vs[1], obj->adcData.V_V.value[1]);
    obj->adcVs_V.value[2] = FILTER_FO_run(obj->filterHandle_Vs[2], obj->adcData.V_V.value[2]);

    if(obj->flagEnableFilterVs == true)
    {
        obj->adcData.V_V.value[0] = obj->adcVs_V.value[0];
        obj->adcData.V_V.value[1] = obj->adcVs_V.value[1];
        obj->adcData.V_V.value[2] = obj->adcVs_V.value[2];
    }
#endif  // MOTOR1_FILTERVS

#if !defined(_HSWFREQ_EN)
    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);
#else   // _HSWFREQ_EN
    // run Clarke transform on voltage
    CLARKE_run_threeInput(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);
#endif  // _HSWFREQ_EN
#endif  // !MOTOR1_VOLRECT

#if !defined(_HSWFREQ_EN)
    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I,
               &obj->adcData.I_A, &obj->estInputData.Iab_A);
#else   // _HSWFREQ_EN
    // run Clarke transform on current
    CLARKE_run_threeInput(obj->clarkeHandle_I,
                          &obj->adcData.I_A, &obj->estInputData.Iab_A);
#endif  // _HSWFREQ_EN

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;

    // configure the trajectory generator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleESTCOMP_rad =
            obj->angleESTDelayed_sf * obj->estOutputData.fm_lp_rps;

#if !defined(_HSWFREQ_EN)
    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);
    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    obj->speed_Hz = obj->speedEST_Hz;
    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;
#else   // _HSWFREQ_EN
    obj->angleFOC_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);

    obj->speed_Hz = EST_getFm_lp_Hz(obj->estHandle);
#endif  // _HSWFREQ_EN

#if !defined(_HSWFREQ_EN)
    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {
        obj->Idq_out_A.value[0] = 0.0f;
        obj->controlStatus = MOTOR_CTRL_RUN;

        // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

#if defined(MOTOR1_POWCTRL)
        obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL
    }
    else if(obj->flagMotorIdentified == true)   // Normal Running
#endif  // _HSWFREQ_EN
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

#if defined(MOTOR1_POWCTRL)
            obj->enablePowerCtrl = obj->flagEnablePowerCtrl;
#endif  // MOTOR1_POWCTRL

#if !defined(_HSWFREQ_EN) && !defined(_SIMPLE_FAST_LIB)
            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
#endif  // !(_HSWFREQ_EN & _SIMPLE_FAST_LIB)
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
#if defined(MOTOR1_POWCTRL)
            obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL
        }

        obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    }

    obj->estInputData.speed_ref_Hz = obj->speed_int_Hz;

#if !defined(_HSWFREQ_EN)
    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);
#endif  // _HSWFREQ_EN

    // Running state
    obj->stateRunTimeCnt++;

#if !defined(_HSWFREQ_EN)
    if(obj->controlStatus >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->controlStatus == MOTOR_OL_START)
    {
#if defined(MOTOR1_DCLINKSS)    // Single shunt
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = objSets->startCurrent_A;
            obj->Idq_out_A.value[1] = objSets->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -objSets->startCurrent_A;
            obj->Idq_out_A.value[1] = -objSets->startCurrent_A;
        }

        if(fabsf(obj->estInputData.speed_ref_Hz) >= objSets->speedForce_Hz)
        {
            TRAJ_setIntValue(obj->trajHandle_spd, obj->estInputData.speed_ref_Hz);

            if(obj->stateRunTimeCnt > objSets->forceRunTimeDelay)
            {
                obj->controlStatus = MOTOR_CL_RUNNING;
                PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
            }
        }

#else   // !(MOTOR1_DCLINKSS)   // 2/3 shunt
        obj->angleFOC_rad = obj->angleEST_rad;
        obj->controlStatus = MOTOR_CL_RUNNING;
#endif  // !(MOTOR1_DCLINKSS)   // 2/3 shunt

    }
    else if(obj->controlStatus == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

#if defined(MOTOR1_POWCTRL)
        obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = objSets->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);

        if((obj->stateRunTimeCnt > objSets->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->controlStatus = MOTOR_OL_START;
            obj->Idq_out_A.value[0] = objSets->fluxCurrent_A;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            PI_setUi(obj->piHandle_spd, objSets->alignCurrent_A);

#if defined(MOTOR1_POWCTRL)
            PI_setUi(obj->piHandle_pow, 0.0f);
#endif  // MOTOR1_POWCTRL
        }
    }
    else if(obj->controlStatus == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

#if defined(MOTOR1_POWCTRL)
        obj->enablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        obj->angleFOC_rad = obj->angleEST_rad;

        if(obj->stateRunTimeCnt > objSets->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;
            obj->counterSpeed = 0;

            if(obj->speedAbs_Hz > objSets->speedFlyingStart_Hz)
            {
                if(obj->speedRef_Hz > 0.0f)
                {
                    obj->speed_int_Hz = obj->speed_Hz + 10.0f;
                }
                else
                {
                    obj->speed_int_Hz = obj->speed_Hz - 10.0f;
                }

                TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

#if defined(MOTOR1_POWCTRL)
                PI_setUi(obj->piHandle_pow, 0.0f);
#endif  // MOTOR1_POWCTRL

                obj->controlStatus = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->controlStatus = MOTOR_ALIGNMENT;
            }
        }
    }
#endif  // !_HSWFREQ_EN

#ifdef MOTOR1_VOLRECT
    if(obj->controlStatus == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(obj->speedAbs_Hz >= obj->speedForce_Hz)
        {
            obj->controlStatus = MOTOR_CL_RUNNING;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            PI_setUi(obj->piHandle_spd, obj->startCurrent_A * 0.5f);
        }
    }
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif



    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST
//------------------------------------------------------------------------------
#else   // No Any Estimator
#error Not select a right estimator for this project
#endif  // (ESTIMATOR)


//---------- Common Speed and Current Loop for all observers -------------------
#if(DMC_BUILDLEVEL >= DMC_LEVEL_4)

#if defined(SFRA_ENABLE)

    if(sfraCollectStart == true)
    {
        collectSFRA(motorHandle_M1);    // Collect noise feedback from loop
    }

    //  SFRA injection
    injectSFRA();                   // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = true;       // enable SFRA data collection
#endif  // SFRA_ENABLE

#if defined(MOTOR1_POWCTRL)
    // run the Power controller
    obj->counterPower++;

    if(obj->counterPower >= obj->numCtrlTicksPerPowerTick)
    {
        obj->counterPower = 0;

        if(obj->enablePowerCtrl == true)
        {
            PI_run(obj->piHandle_pow,
                   obj->powerRef_W, obj->powerActive_W,
                   (float32_t *)&obj->IsPower_A);

            PI_setMinMax(obj->piHandle_spd, -obj->IsPower_A, obj->IsPower_A);
        }
    }
#endif  // MOTOR1_POWCTRL

    // run the speed controller
    obj->counterSpeed++;

    if(obj->counterSpeed >= obj->numCtrlTicksPerSpeedTick)
    {
        obj->counterSpeed = 0;

        if(obj->enableSpeedCtrl == true)
        {
#if !defined(_HSWFREQ_EN)
            obj->Is_ffwd_A = 0.0f;
#endif  // _HSWFREQ_EN

#if defined(SFRA_ENABLE)
            PI_run_series(obj->piHandle_spd,
                   (obj->speed_int_Hz + sfraNoiseSpd), obj->speed_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#else     // !SFRA_ENABLE
#if !defined(_HSWFREQ_EN)
            PI_run_series(obj->piHandle_spd,
                   obj->speed_int_Hz, obj->speed_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#else  // _HSWFREQ_EN
            PI_run(obj->piHandle_spd,
                   obj->speed_int_Hz, obj->speed_Hz,
                   (float32_t *)&obj->IdqRef_A.value[1]);
#endif  // _HSWFREQ_EN
#endif  // !SFRA_ENABLE
        }    // (obj->enableSpeedCtrl == true)
#if !defined(_SIMPLE_FAST_LIB)
        else if((obj->controlStatus >= MOTOR_CL_RUNNING) &&
                (obj->flagMotorIdentified == true))
#else   // !_SIMPLE_FAST_LIB
        else if(obj->controlStatus >= MOTOR_CL_RUNNING)
#endif  // !_SIMPLE_FAST_LIB
        {
            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->IsSet_A;
            }
            else
            {
                obj->IsRef_A = -obj->IsSet_A;
            }

            // for switching back speed closed-loop control
            PI_setUi(obj->piHandle_spd, obj->IsRef_A);
        }   // (obj->enableSpeedCtrl == false)
    }   // (obj->counterSpeed >= obj->numCtrlTicksPerSpeedTick)

#if !defined(_HSWFREQ_EN)
#if defined(MOTOR1_FWC) && defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad =
                (obj->angleFWC_rad > obj->angleMTPA_rad) ?
                        obj->angleFWC_rad : obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if((obj->flagEnableFWC == true) || (obj->flagEnableMTPA == true))
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = objSets->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad =
                    MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_FWC)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleFWC_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableFWC == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableMTPA == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad = MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#else   // !MOTOR1_MTPA && !MOTOR1_FWC
    obj->Idq_out_A.value[1] = obj->IsRef_A;
#endif  // !MOTOR1_MTPA && !MOTOR1_FWC/

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[0] = obj->Idq_out_A.value[0] + obj->IdRated_A + obj->IdInj_A;
#endif  // STEP_RP_EN

#if defined(MOTOR1_FAST)
    // update Id reference for Rs OnLine
    EST_updateId_ref_A(obj->estHandle, &obj->IdqRef_A.value[0]);
#endif  // MOTOR1_FAST

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1] + obj->IqInj_A;
#else   // STEP_RP_EN
    if(GRAPH_getBufferMode(&stepRPVars) != GRAPH_STEP_RP_TORQUE)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
    }
    else
    {
        PI_setUi(obj->piHandle_spd, obj->IdqRef_A.value[1]);
    }
#endif  // STEP_RP_EN
#endif  // !_HSWFREQ_EN


#elif(DMC_BUILDLEVEL == DMC_LEVEL_3)
    obj->IdqRef_A.value[0] = obj->Idq_set_A.value[0];
    obj->IdqRef_A.value[1] = obj->Idq_set_A.value[1];
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    if(obj->enableCurrentCtrl == true)
    {
#if !defined(_HSWFREQ_EN)
        obj->Vdq_ffwd_V.value[0] = 0.0f;
        obj->Vdq_ffwd_V.value[1] = 0.0f;
#endif  // _HSWFREQ_EN



        // Maximum voltage output
        obj->VsMax_V = obj->maxVsMag_pu * obj->adcData.VdcBus_V;
        PI_setMinMax(obj->piHandle_Id, -obj->VsMax_V, obj->VsMax_V);

#if defined(SFRA_ENABLE)
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      (obj->IdqRef_A.value[0] + sfraNoiseId), obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run_series(obj->piHandle_Iq,
                      (obj->IdqRef_A.value[1] + sfraNoiseIq), obj->Idq_in_A.value[1],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[1]);

#else     // !SFRA_ENABLE
#if !defined(_HSWFREQ_EN)
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      obj->IdqRef_A.value[0], obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run_series(obj->piHandle_Iq,
                      obj->IdqRef_A.value[1], obj->Idq_in_A.value[1],
                      obj->Vdq_ffwd_V.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);
#else   // _HSWFREQ_EN
        PI_run(obj->piHandle_Id,
               obj->IdqRef_A.value[0], obj->Idq_in_A.value[0],
               (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq,
               obj->IdqRef_A.value[1], obj->Idq_in_A.value[1],
               (float32_t*)&obj->Vdq_out_V.value[1]);
#endif  // _HSWFREQ_EN
#endif  // !SFRA_ENABLE

#if defined(MOTOR1_FAST)
#if !defined(_HSWFREQ_EN) && !defined(_SIMPLE_FAST_LIB)
        // set the Id reference value in the estimator
        EST_setId_ref_A(obj->estHandle, obj->IdqRef_A.value[0]);
        EST_setIq_ref_A(obj->estHandle, obj->IdqRef_A.value[1]);
#endif  // (!_HSWFREQ_EN & _SIMPLE_FAST_LIB)
#endif  // MOTOR1_FAST
    }

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    VS_FREQ_run(obj->VsFreqHandle, obj->speed_int_Hz);
    obj->Vdq_out_V.value[0] = VS_FREQ_getVd_out(obj->VsFreqHandle);
    obj->Vdq_out_V.value[1] = VS_FREQ_getVq_out(obj->VsFreqHandle);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(PHASE_ADJ_EN)
    if(obj->flagPhaseAdjustEnable == true)
    {
        obj->angleFOCAdj_rad =
                MATH_incrAngle(obj->angleFOC_rad, obj->anglePhaseAdj_rad);

        // compute the sin/cos phasor
        phasor.value[0] = __cos(obj->angleFOCAdj_rad);
        phasor.value[1] = __sin(obj->angleFOCAdj_rad);
    }
    else
    {
        obj->angleFOCAdj_rad = obj->angleFOC_rad;
    }
#endif  // PHASE_ADJ_EN
    // set the phasor in the inverse Park transform
    IPARK_setPhasor(obj->iparkHandle_V, &phasor);

    // run the inverse Park module
    IPARK_run(obj->iparkHandle_V,
              &obj->Vdq_out_V, &obj->Vab_out_V);

#if defined(MOTOR1_FAST)
    // setup the space vector generator (SVGEN) module
    SVGEN_setup(obj->svgenHandle, obj->estOutputData.oneOverDcBus_invV);
#else  // MOTOR1_FAST
    // setup the space vector generator (SVGEN) module
    SVGEN_setup(obj->svgenHandle, obj->oneOverDcBus_invV);
#endif  // MOTOR1_FAST

    // run the space vector generator (SVGEN) module
    SVGEN_run(obj->svgenHandle,
              &obj->Vab_out_V, &(obj->pwmData.Vabc_pu));
#if(DMC_BUILDLEVEL == DMC_LEVEL_1)
    // output 50%
    obj->pwmData.Vabc_pu.value[0] = 0.0f;
    obj->pwmData.Vabc_pu.value[1] = 0.0f;
    obj->pwmData.Vabc_pu.value[2] = 0.0f;
#endif

#if !defined(_HSWFREQ_EN)
    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
    {
        // clear PWM data
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;
    }
#endif  // !_HSWFREQ_EN

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    // revise PWM compare(CMPA/B) values for shifting switching pattern
    // and, update SOC trigger point
    HAL_runSingleShuntCompensation(obj->halMtrHandle, obj->dclinkHandle,
                         &obj->Vab_out_V, &obj->pwmData, obj->adcData.VdcBus_V);
#else   // !(MOTOR1_DCLINKSS)   // 2/3 shunt
#if defined(MOTOR1_OVM)
    else if(obj->flagEnableOVM == true)
    {
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPWMData(obj->svgencurrentHandle,
                                 &obj->pwmData.Vabc_pu, &obj->pwmDataPrev);
    }

    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    if(obj->flagEnableOVM == true)
    {
        obj->ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(obj->svgencurrentHandle);
        obj->midVolShunt = SVGENCURRENT_getVmid(obj->svgencurrentHandle);

        // Set trigger point in the middle of the low side pulse
        HAL_setTrigger(obj->halMtrHandle,
                       &obj->pwmData, obj->ignoreShuntNextCycle, obj->midVolShunt);
    }
#else   // !MOTOR1_OVM
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif  // !MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)    // 2/3 shunt

#if !defined(_SIMPLE_FAST_LIB) && !defined(_HSWFREQ_EN)
    // Collect current and voltage data to calculate the RMS value
    collectRMSData(motorHandle_M1);
#endif  // !(_SIMPLE_FAST_LIB & _HSWFREQ_EN)

#if defined(BENCHMARK_TEST)
    recordSpeedData(motorHandle_M1);
#endif  // BENCHMARK_TEST

#if defined(STEP_RP_EN)
    // Collect predefined data into arrays
    GRAPH_updateBuffer(&stepRPVars);
#endif  // STEP_RP_EN


#if defined(EPWMDAC_MODE)
    // connect inputs of the PWMDAC module.
    HAL_writePWMDACData(halHandle, &pwmDACData);
#endif  // EPWMDAC_MODE

#if defined(DATALOGF2_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
    }
#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
        HAL_trigDMAforDLOG(halHandle, 2);
        HAL_trigDMAforDLOG(halHandle, 3);
    }
#endif  // DATALOGF4_EN || DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    // Write the variables data value to DAC128S085
    DAC128S_writeData(dac128sHandle);
#endif  // DAC128S_ENABLE



#if defined(SAFETY_ENABLE)
    obj->safetyFaultFlag |= STA_testADC(&obj->adcData);
#endif  // SAFETY_ENABLE



#if defined(TEST_ENABLE)
    HAL_setGPIOLow(ISR_CHECK_GPIO);
#endif  // TEST_ENABLE

    return;
} // end of motor1CtrlISR() function

//
//-- end of this file ----------------------------------------------------------
//
