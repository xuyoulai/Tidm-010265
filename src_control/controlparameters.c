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
//! \file   /solutions/tida_010265_wminv/common/source/controlparameters.c
//!
//! \brief  This project is used to implement motor control with
//!         F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


//
// Includes
//
#include "sys_main.h"

#include "controlparameters.h"

//
// Defines
//
#define x1aF  0xFFFF
#define x2aF  x1aF,  x1aF
#define x4aF  x2aF,  x2aF
#define x8aF  x4aF,  x4aF
#define x16aF x8aF,  x8aF

#define x1fF  0xFFFFFFFF
#define x2fF  x1fF,  x1fF
#define x4fF  x2fF,  x2fF
#define x8fF  x4fF,  x4fF
#define x16fF x8fF,  x8fF
#define x32fF x16fF, x16fF

//
// Typedefs
//


//
// Globals
//
#if defined(_PRMS_UPDATE)
#pragma DATA_SECTION(prmsIndexList, "prms_index1");

const uint16_t prmsIndexList[32] = {x16aF, x16aF};

// define the parameters list (Copy from LCINV*.xlsx)
#pragma DATA_SECTION(ctrlParamsList, "prms_datas1");

// Define motor M1 control parameters
const CTRL_Params_t ctrlParamsList[MOTOR_MAX_NUM] = { \
    // Define motor M1 control parameters
    {  \
        M1_MOTORMODEL,   /* 0,  motorModel,  */  \
        M1_OVERCURRENTTIMESSET,   /* 1,  overCurrentTimesSet,  */  \
        M1_OVERLOADTIMESET,   /* 2,  overLoadTimeSet,  */  \
        M1_MOTORSTALLTIMESET,   /* 3,  motorStallTimeSet,  */  \
        M1_VOLTAGEFAULTTIMESET,   /* 4,  voltageFaultTimeSet,  */  \
        M1_STARTUPFAILTIMESET,   /* 5,  startupFailTimeSet,  */  \
        M1_OVERSPEEDTIMESET,   /* 6,  overSpeedTimeSet,  */  \
        M1_UNBALANCETIMESET,   /* 7,  unbalanceTimeSet,  */  \
        M1_LOSTPHASETIMESET,   /* 8,  lostPhaseTimeSet,  */  \
        M1_FLYINGSTARTTIMEDELAY,   /* 9,  flyingStartTimeDelay,  */  \
        M1_ALIGNTIMEDELAY,   /* 10,  alignTimeDelay,  */  \
        M1_FORCERUNTIMEDELAY,   /* 11,  forceRunTimeDelay,  */  \
        M1_CONTROLTICKSPWM,   /* 12,  controlTicksPWM,  */  \
        M1_SPEEDTICKSCONTROL,   /* 13,  speedTicksControl,  */  \
        M1_MOTOR_TYPE  ,   /* 14,  motor_type  ,  */  \
        M1_NUMPOLEPAIRS,   /* 15,  numPolePairs,  */  \
        M1_RS_OHM,   /* 16,  Rs_Ohm,  */  \
        M1_LS_D_H,   /* 17,  Ls_d_H,  */  \
        M1_LS_Q_H,   /* 18,  Ls_q_H,  */  \
        M1_FLUX_VPHZ,   /* 19,  flux_VpHz,  */  \
        M1_RR_OHM,   /* 20,  Rr_Ohm,  */  \
        M1_MAGNETICCURRENT_A,   /* 21,  magneticCurrent_A,  */  \
        M1_MAXCURRENTRESEST_A,   /* 22,  maxCurrentResEst_A,  */  \
        M1_MAXCURRENTINDEST_A,   /* 23,  maxCurrentIndEst_A,  */  \
        M1_FLUXEXCFREQ_HZ,   /* 24,  fluxExcFreq_Hz,  */  \
        M1_FLUXFILTERCOEF,   /* 25,  fluxFilterCoef,  */  \
        M1_SPEEDFILTERCOEF,   /* 26,  speedFilterCoef,  */  \
        M1_BEMFFILTERCOEF,   /* 27,  bemfFilterCoef,  */  \
        M1_SPEEDPOLE_RPS,   /* 28,  speedPole_rps,  */  \
        M1_DIRECTIONPOLE_RPS,   /* 29,  directionPole_rps,  */  \
        M1_FLUXPOLE_RPS,   /* 30,  fluxPole_rps,  */  \
        M1_RSONLINE_RDELTA_OHM,   /* 31,  RsOnLine_Rdelta_Ohm,  */  \
        M1_RSONLINE_ADELTA_RAD,   /* 32,  RsOnLine_Adelta_rad,  */  \
        M1_VOLTAGEFILTER_HZ,   /* 33,  voltageFilter_Hz,  */  \
        M1_VOLTAGESCALE_V,   /* 34,  voltageScale_V,  */  \
        M1_CURRENTSCALE_A,   /* 35,  currentScale_A,  */  \
        M1_PWMCONTROL_KHZ,   /* 36,  pwmControl_kHz,  */  \
        M1_RSONLINECURRENT_A,   /* 37,  RsOnLineCurrent_A,  */  \
        M1_ANGLEESTDELAYED_SF,   /* 38,  angleESTDelayed_sf,  */  \
        M1_MAXFREQUENCY_HZ,   /* 39,  maxFrequency_Hz,  */  \
        M1_MAXCURRENT_A,   /* 40,  maxCurrent_A,  */  \
        M1_MAXVOLTAGE_V,   /* 41,  maxVoltage_V,  */  \
        M1_MAXPEAKCURRENT_A,   /* 42,  maxPeakCurrent_A,  */  \
        M1_MAXVSMAG_PU,   /* 43,  maxVsMag_pu,  */  \
        M1_LS_D_ICOMP_COEF,   /* 44,  Ls_d_Icomp_coef,  */  \
        M1_LS_Q_ICOMP_COEF,   /* 45,  Ls_q_Icomp_coef,  */  \
        M1_LS_MIN_H,   /* 46,  Ls_min_H,  */  \
        M1_POWERCTRLSET_W,   /* 47,  powerCtrlSet_W,  */  \
        M1_OVERCURRENT_A,   /* 48,  overCurrent_A,  */  \
        M1_OVERLOADSET_W,   /* 49,  overLoadSet_W,  */  \
        M1_LOSTPHASESET_A,   /* 50,  lostPhaseSet_A,  */  \
        M1_UNBALANCERATIOSET,   /* 51,  unbalanceRatioSet,  */  \
        M1_STALLCURRENTSET_A,   /* 52,  stallCurrentSet_A,  */  \
        M1_SPEEDFAILMAXSET_HZ,   /* 53,  speedFailMaxSet_Hz,  */  \
        M1_SPEEDFAILMINSET_HZ,   /* 54,  speedFailMinSet_Hz,  */  \
        M1_ISFAILEDCHEKSET_A,   /* 55,  IsFailedChekSet_A,  */  \
        M1_TOQUEFAILMINSET_NM,   /* 56,  toqueFailMinSet_Nm,  */  \
        M1_OVERVOLTAGEFAULT_V,   /* 57,  overVoltageFault_V,  */  \
        M1_OVERVOLTAGENORM_V,   /* 58,  overVoltageNorm_V,  */  \
        M1_UNDERVOLTAGEFAULT_V,   /* 59,  underVoltageFault_V,  */  \
        M1_UNDERVOLTAGENORM_V,   /* 60,  underVoltageNorm_V,  */  \
        M1_FLUXCURRENT_A,   /* 61,  fluxCurrent_A,  */  \
        M1_ALIGNCURRENT_A,   /* 62,  alignCurrent_A,  */  \
        M1_STARTCURRENT_A,   /* 63,  startCurrent_A,  */  \
        M1_BRAKINGCURRENT_A,   /* 64,  brakingCurrent_A,  */  \
        M1_ACCELSTART_HZPS,   /* 65,  accelStart_Hzps,  */  \
        M1_ACCELSTOP_HZPS,   /* 66,  accelStop_Hzps,  */  \
        M1_ACCELRUN_HZPS,   /* 67,  accelRun_Hzps,  */  \
        M1_SPEEDFLYINGSTART_HZ,   /* 68,  speedFlyingStart_Hz,  */  \
        M1_SPEEDFORCE_HZ,   /* 69,  speedForce_Hz,  */  \
        M1_SPEEDSTART_HZ ,   /* 70,  speedStart_Hz ,  */  \
        M1_VSREF_PU,   /* 71,  VsRef_pu,  */  \
        M1_KP_FWC,   /* 72,  Kp_fwc,  */  \
        M1_KI_FWC,   /* 73,  Ki_fwc,  */  \
        M1_ANGLEFWCMAX_RAD,   /* 74,  angleFWCMax_rad,  */  \
        M1_GAIN_SPEED_HIGH_HZ,   /* 75,  Gain_speed_high_Hz,  */  \
        M1_GAIN_SPEED_LOW_HZ,   /* 76,  Gain_speed_low_Hz,  */  \
        M1_KP_SPD_HIGH_SF,   /* 77,  Kp_spd_high_sf,  */  \
        M1_KI_SPD_HIGH_SF,   /* 78,  Ki_spd_high_sf,  */  \
        M1_KP_SPD_LOW_SF,   /* 79,  Kp_spd_low_sf,  */  \
        M1_KI_SPD_LOW_SF,   /* 80,  Ki_spd_low_sf,  */  \
        M1_KP_IQ_SF,   /* 81,  Kp_Iq_sf,  */  \
        M1_KI_IQ_SF,   /* 82,  Ki_Iq_sf,  */  \
        M1_KP_ID_SF,   /* 83,  Kp_Id_sf,  */  \
        M1_KI_ID_SF,   /* 84,  Ki_Id_sf,  */  \
        M1_KP_POW_SF,   /* 85,  Kp_pow_sf,  */  \
        M1_KI_POW_SF,   /* 86,  Ki_pow_sf,  */  \
        M1_KP_SPD_START_SF,   /* 87,  Kp_spd_start_sf,  */  \
        M1_KI_SPD_START_SF,   /* 88,  Ki_spd_start_sf,  */  \
        M1_ESMO_FAST_FSW_HZ,   /* 89,  esmo_FAST_fsw_Hz,  */  \
        M1_ESMO_KSLIDEMAX,   /* 90,  esmo_KslideMax,  */  \
        M1_ESMO_KSLIDEMIN,   /* 91,  esmo_KslideMin,  */  \
        M1_ESMO_LPFFC_HZ,   /* 92,  esmo_LpfFc_Hz,  */  \
        M1_ESMO_FILTERFC_SF,   /* 93,  esmo_filterFc_sf,  */  \
        M1_ESMO_E0,   /* 94,  esmo_E0,  */  \
        M1_ESMO_PLL_KPMAX,   /* 95,  esmo_PLL_KpMax,  */  \
        M1_ESMO_PLL_KPMIN,   /* 96,  esmo_PLL_KpMin,  */  \
        M1_ESMO_PLL_KPSF,   /* 97,  esmo_PLL_KpSF,  */  \
        M1_ESMO_PLL_KI,   /* 98,  esmo_PLL_Ki,  */  \
        M1_ANGLEPLLDELAYED_SF,   /* 99,  anglePLLDelayed_sf,  */  \
        M1_HFI_KSPD,   /* 100,  hfi_Kspd,  */  \
        M1_HFI_EXCMAG_COARSE_V,   /* 101,  hfi_excMag_coarse_V,  */  \
        M1_HFI_EXCMAG_FINE_V,   /* 102,  hfi_excMag_fine_V,  */  \
        M1_HFI_WAITTIME_COARSE_SEC,   /* 103,  hfi_waitTime_coarse_sec,  */  \
        M1_HFI_WAITTIME_FINE_SEC,   /* 104,  hfi_waitTime_fine_sec,  */  \
        M1_HFI_EXCFREQ_HZ,   /* 105,  hfi_excFreq_Hz,  */  \
        M1_HFI_LPFFCSPD_HZ,   /* 106,  hfi_LpfFcSpd_Hz,  */  \
        M1_HFI_HPFFCIQ_HZ,   /* 107,  hfi_HpfFcIq_Hz,  */  \
        M1_HFI_IQMAXHFI_A,   /* 108,  hfi_IqMaxHfi_A,  */  \
        M1_HFI_IQMAXEST_A,   /* 109,  hfi_IqMaxEst_A,  */  \
        M1_HFI_IQSLOPE_A,   /* 110,  hfi_IqSlope_A,  */  \
        M1_HFI_FREQLOW_HZ,   /* 111,  hfi_freqLow_Hz,  */  \
        M1_HFI_FREQHIGH_HZ,   /* 112,  hfi_freqHigh_Hz,  */  \
        M1_PIR_SPD_F0,   /* 113,  pir_spd_f0,  */  \
        M1_PIR_SPD_FC,   /* 114,  pir_spd_fc,  */  \
        M1_PIR_SPD_K,   /* 115,  pir_spd_k,  */  \
        M1_PIR_IDQ_FC,   /* 116,  pir_Idq_fc,  */  \
        M1_PIR_ID_K,   /* 117,  pir_Id_k,  */  \
        M1_PIR_IQ_K,   /* 118,  pir_Iq_k,  */  \
        M1_VBC_ANGLE_DELTA,   /* 119,  vbc_angle_delta,  */  \
        M1_VBC_IQ_AMP,   /* 120,  vbc_Iq_amp,  */  \
        M1_VBC_IQ_SF,   /* 121,  vbc_Iq_sf,  */  \
        M1_VBC_FREQ_SF,   /* 122,  vbc_freq_sf,  */  \
        M1_VBC_ANGLE_SF,   /* 123,  vbc_angle_sf,  */  \
        M1_VBC_KI_SF,   /* 124,  vbc_Ki_sf,  */  \
        M1_FILTERISFC,   /* 125,  filterIsFc,  */  \
        M1_FILTERVSFC,   /* 126,  filterVsFc,  */  \
        M1_V_DECOUP_SF,   /* 127,  V_decoup_sf,  */  \
        M1_CONTROLTYPES,   /* 128,  controlTypes,  */  \
        M1_CONTROLFUNCS,   /* 129,  controlFuncs,  */  \
        M1_FAULTMTRMASK,   /* 130,  faultMtrMask,  */  \
        M1_OVERTEMPERATUREMOTOR,   /* 131,  overTemperatureMotor,  */  \
        M1_OVERTEMPERATUREMODULE,   /* 132,  overTemperatureModule,  */  \
        M1_RSONLINEWAITTIMESET,   /* 133,  RsOnlineWaitTimeSet,  */  \
        M1_RSONLINEWORKTIMESET,   /* 134,  RsOnlineWorkTimeSet,  */  \
        M1_SAMPLETRIGDELAY,   /* 135,  sampleTrigDelay,  */  \
        M1_DSSCMINDURATION,   /* 136,  dsscMinDuration,  */  \
        M1_DSSCSAMPLEDELAY,   /* 137,  dsscSampleDelay,  */  \
        M1_STARTUPTIMEDELAY,   /* 138,  startupTimeDelay,  */  \
        M1_STOPWAITTIMESET,   /* 139,  stopWaitTimeSet,  */  \
        M1_RESTARTWAITTIMESET,   /* 140,  restartWaitTimeSet,  */  \
        M1_RESTARTTIMESSET,   /* 141,  restartTimesSet,  */  \
        M1_OOBCHECKTIMESET,   /* 142,  oobCheckTimeSet,  */  \
        M1_OOBSPEEDSET_RPM,   /* 143,  oobSpeedSet_rpm,  */  \
        M1_OOBCALCCOEFSET,   /* 144,  oobCalcCoefSet,  */  \
        M1_OOBACCELSET_RPMPS,   /* 145,  oobAccelSet_rpmps,  */  \
        M1_WEIGHTCHECKTIMESET,   /* 146,  weightCheckTimeSet,  */  \
        M1_WEIGHTSPEEDSET_RPM,   /* 147,  weightSpeedSet_rpm,  */  \
        M1_WEIGHTCALCCOEFSET,   /* 148,  weightCalcCoefSet,  */  \
        M1_WEIGHTACCELSET_RPMPS,   /* 149,  weightAccelSet_rpmps,  */  \
        M1_RESERVE_PRMS1,   /* 150,  reserve_Prms1,  */  \
        M1_RESERVE_PRMS2,   /* 151,  reserve_Prms2,  */  \
    },   /* // End of motor M1 control parameters  */   \
    // Define motor M2 control parameters
    {  \
        M2_MOTORMODEL,   /* 0,  motorModel,  */  \
        M2_OVERCURRENTTIMESSET,   /* 1,  overCurrentTimesSet,  */  \
        M2_OVERLOADTIMESET,   /* 2,  overLoadTimeSet,  */  \
        M2_MOTORSTALLTIMESET,   /* 3,  motorStallTimeSet,  */  \
        M2_VOLTAGEFAULTTIMESET,   /* 4,  voltageFaultTimeSet,  */  \
        M2_STARTUPFAILTIMESET,   /* 5,  startupFailTimeSet,  */  \
        M2_OVERSPEEDTIMESET,   /* 6,  overSpeedTimeSet,  */  \
        M2_UNBALANCETIMESET,   /* 7,  unbalanceTimeSet,  */  \
        M2_LOSTPHASETIMESET,   /* 8,  lostPhaseTimeSet,  */  \
        M2_FLYINGSTARTTIMEDELAY,   /* 9,  flyingStartTimeDelay,  */  \
        M2_ALIGNTIMEDELAY,   /* 10,  alignTimeDelay,  */  \
        M2_FORCERUNTIMEDELAY,   /* 11,  forceRunTimeDelay,  */  \
        M2_CONTROLTICKSPWM,   /* 12,  controlTicksPWM,  */  \
        M2_SPEEDTICKSCONTROL,   /* 13,  speedTicksControl,  */  \
        M2_MOTOR_TYPE  ,   /* 14,  motor_type  ,  */  \
        M2_NUMPOLEPAIRS,   /* 15,  numPolePairs,  */  \
        M2_RS_OHM,   /* 16,  Rs_Ohm,  */  \
        M2_LS_D_H,   /* 17,  Ls_d_H,  */  \
        M2_LS_Q_H,   /* 18,  Ls_q_H,  */  \
        M2_FLUX_VPHZ,   /* 19,  flux_VpHz,  */  \
        M2_RR_OHM,   /* 20,  Rr_Ohm,  */  \
        M2_MAGNETICCURRENT_A,   /* 21,  magneticCurrent_A,  */  \
        M2_MAXCURRENTRESEST_A,   /* 22,  maxCurrentResEst_A,  */  \
        M2_MAXCURRENTINDEST_A,   /* 23,  maxCurrentIndEst_A,  */  \
        M2_FLUXEXCFREQ_HZ,   /* 24,  fluxExcFreq_Hz,  */  \
        M2_FLUXFILTERCOEF,   /* 25,  fluxFilterCoef,  */  \
        M2_SPEEDFILTERCOEF,   /* 26,  speedFilterCoef,  */  \
        M2_BEMFFILTERCOEF,   /* 27,  bemfFilterCoef,  */  \
        M2_SPEEDPOLE_RPS,   /* 28,  speedPole_rps,  */  \
        M2_DIRECTIONPOLE_RPS,   /* 29,  directionPole_rps,  */  \
        M2_FLUXPOLE_RPS,   /* 30,  fluxPole_rps,  */  \
        M2_RSONLINE_RDELTA_OHM,   /* 31,  RsOnLine_Rdelta_Ohm,  */  \
        M2_RSONLINE_ADELTA_RAD,   /* 32,  RsOnLine_Adelta_rad,  */  \
        M2_VOLTAGEFILTER_HZ,   /* 33,  voltageFilter_Hz,  */  \
        M2_VOLTAGESCALE_V,   /* 34,  voltageScale_V,  */  \
        M2_CURRENTSCALE_A,   /* 35,  currentScale_A,  */  \
        M2_PWMCONTROL_KHZ,   /* 36,  pwmControl_kHz,  */  \
        M2_RSONLINECURRENT_A,   /* 37,  RsOnLineCurrent_A,  */  \
        M2_ANGLEESTDELAYED_SF,   /* 38,  angleESTDelayed_sf,  */  \
        M2_MAXFREQUENCY_HZ,   /* 39,  maxFrequency_Hz,  */  \
        M2_MAXCURRENT_A,   /* 40,  maxCurrent_A,  */  \
        M2_MAXVOLTAGE_V,   /* 41,  maxVoltage_V,  */  \
        M2_MAXPEAKCURRENT_A,   /* 42,  maxPeakCurrent_A,  */  \
        M2_MAXVSMAG_PU,   /* 43,  maxVsMag_pu,  */  \
        M2_LS_D_ICOMP_COEF,   /* 44,  Ls_d_Icomp_coef,  */  \
        M2_LS_Q_ICOMP_COEF,   /* 45,  Ls_q_Icomp_coef,  */  \
        M2_LS_MIN_H,   /* 46,  Ls_min_H,  */  \
        M2_POWERCTRLSET_W,   /* 47,  powerCtrlSet_W,  */  \
        M2_OVERCURRENT_A,   /* 48,  overCurrent_A,  */  \
        M2_OVERLOADSET_W,   /* 49,  overLoadSet_W,  */  \
        M2_LOSTPHASESET_A,   /* 50,  lostPhaseSet_A,  */  \
        M2_UNBALANCERATIOSET,   /* 51,  unbalanceRatioSet,  */  \
        M2_STALLCURRENTSET_A,   /* 52,  stallCurrentSet_A,  */  \
        M2_SPEEDFAILMAXSET_HZ,   /* 53,  speedFailMaxSet_Hz,  */  \
        M2_SPEEDFAILMINSET_HZ,   /* 54,  speedFailMinSet_Hz,  */  \
        M2_ISFAILEDCHEKSET_A,   /* 55,  IsFailedChekSet_A,  */  \
        M2_TOQUEFAILMINSET_NM,   /* 56,  toqueFailMinSet_Nm,  */  \
        M2_OVERVOLTAGEFAULT_V,   /* 57,  overVoltageFault_V,  */  \
        M2_OVERVOLTAGENORM_V,   /* 58,  overVoltageNorm_V,  */  \
        M2_UNDERVOLTAGEFAULT_V,   /* 59,  underVoltageFault_V,  */  \
        M2_UNDERVOLTAGENORM_V,   /* 60,  underVoltageNorm_V,  */  \
        M2_FLUXCURRENT_A,   /* 61,  fluxCurrent_A,  */  \
        M2_ALIGNCURRENT_A,   /* 62,  alignCurrent_A,  */  \
        M2_STARTCURRENT_A,   /* 63,  startCurrent_A,  */  \
        M2_BRAKINGCURRENT_A,   /* 64,  brakingCurrent_A,  */  \
        M2_ACCELSTART_HZPS,   /* 65,  accelStart_Hzps,  */  \
        M2_ACCELSTOP_HZPS,   /* 66,  accelStop_Hzps,  */  \
        M2_ACCELRUN_HZPS,   /* 67,  accelRun_Hzps,  */  \
        M2_SPEEDFLYINGSTART_HZ,   /* 68,  speedFlyingStart_Hz,  */  \
        M2_SPEEDFORCE_HZ,   /* 69,  speedForce_Hz,  */  \
        M2_SPEEDSTART_HZ ,   /* 70,  speedStart_Hz ,  */  \
        M2_VSREF_PU,   /* 71,  VsRef_pu,  */  \
        M2_KP_FWC,   /* 72,  Kp_fwc,  */  \
        M2_KI_FWC,   /* 73,  Ki_fwc,  */  \
        M2_ANGLEFWCMAX_RAD,   /* 74,  angleFWCMax_rad,  */  \
        M2_GAIN_SPEED_HIGH_HZ,   /* 75,  Gain_speed_high_Hz,  */  \
        M2_GAIN_SPEED_LOW_HZ,   /* 76,  Gain_speed_low_Hz,  */  \
        M2_KP_SPD_HIGH_SF,   /* 77,  Kp_spd_high_sf,  */  \
        M2_KI_SPD_HIGH_SF,   /* 78,  Ki_spd_high_sf,  */  \
        M2_KP_SPD_LOW_SF,   /* 79,  Kp_spd_low_sf,  */  \
        M2_KI_SPD_LOW_SF,   /* 80,  Ki_spd_low_sf,  */  \
        M2_KP_IQ_SF,   /* 81,  Kp_Iq_sf,  */  \
        M2_KI_IQ_SF,   /* 82,  Ki_Iq_sf,  */  \
        M2_KP_ID_SF,   /* 83,  Kp_Id_sf,  */  \
        M2_KI_ID_SF,   /* 84,  Ki_Id_sf,  */  \
        M2_KP_POW_SF,   /* 85,  Kp_pow_sf,  */  \
        M2_KI_POW_SF,   /* 86,  Ki_pow_sf,  */  \
        M2_KP_SPD_START_SF,   /* 87,  Kp_spd_start_sf,  */  \
        M2_KI_SPD_START_SF,   /* 88,  Ki_spd_start_sf,  */  \
        M2_ESMO_FAST_FSW_HZ,   /* 89,  esmo_FAST_fsw_Hz,  */  \
        M2_ESMO_KSLIDEMAX,   /* 90,  esmo_KslideMax,  */  \
        M2_ESMO_KSLIDEMIN,   /* 91,  esmo_KslideMin,  */  \
        M2_ESMO_LPFFC_HZ,   /* 92,  esmo_LpfFc_Hz,  */  \
        M2_ESMO_FILTERFC_SF,   /* 93,  esmo_filterFc_sf,  */  \
        M2_ESMO_E0,   /* 94,  esmo_E0,  */  \
        M2_ESMO_PLL_KPMAX,   /* 95,  esmo_PLL_KpMax,  */  \
        M2_ESMO_PLL_KPMIN,   /* 96,  esmo_PLL_KpMin,  */  \
        M2_ESMO_PLL_KPSF,   /* 97,  esmo_PLL_KpSF,  */  \
        M2_ESMO_PLL_KI,   /* 98,  esmo_PLL_Ki,  */  \
        M2_ANGLEPLLDELAYED_SF,   /* 99,  anglePLLDelayed_sf,  */  \
        M2_HFI_KSPD,   /* 100,  hfi_Kspd,  */  \
        M2_HFI_EXCMAG_COARSE_V,   /* 101,  hfi_excMag_coarse_V,  */  \
        M2_HFI_EXCMAG_FINE_V,   /* 102,  hfi_excMag_fine_V,  */  \
        M2_HFI_WAITTIME_COARSE_SEC,   /* 103,  hfi_waitTime_coarse_sec,  */  \
        M2_HFI_WAITTIME_FINE_SEC,   /* 104,  hfi_waitTime_fine_sec,  */  \
        M2_HFI_EXCFREQ_HZ,   /* 105,  hfi_excFreq_Hz,  */  \
        M2_HFI_LPFFCSPD_HZ,   /* 106,  hfi_LpfFcSpd_Hz,  */  \
        M2_HFI_HPFFCIQ_HZ,   /* 107,  hfi_HpfFcIq_Hz,  */  \
        M2_HFI_IQMAXHFI_A,   /* 108,  hfi_IqMaxHfi_A,  */  \
        M2_HFI_IQMAXEST_A,   /* 109,  hfi_IqMaxEst_A,  */  \
        M2_HFI_IQSLOPE_A,   /* 110,  hfi_IqSlope_A,  */  \
        M2_HFI_FREQLOW_HZ,   /* 111,  hfi_freqLow_Hz,  */  \
        M2_HFI_FREQHIGH_HZ,   /* 112,  hfi_freqHigh_Hz,  */  \
        M2_PIR_SPD_F0,   /* 113,  pir_spd_f0,  */  \
        M2_PIR_SPD_FC,   /* 114,  pir_spd_fc,  */  \
        M2_PIR_SPD_K,   /* 115,  pir_spd_k,  */  \
        M2_PIR_IDQ_FC,   /* 116,  pir_Idq_fc,  */  \
        M2_PIR_ID_K,   /* 117,  pir_Id_k,  */  \
        M2_PIR_IQ_K,   /* 118,  pir_Iq_k,  */  \
        M2_VBC_ANGLE_DELTA,   /* 119,  vbc_angle_delta,  */  \
        M2_VBC_IQ_AMP,   /* 120,  vbc_Iq_amp,  */  \
        M2_VBC_IQ_SF,   /* 121,  vbc_Iq_sf,  */  \
        M2_VBC_FREQ_SF,   /* 122,  vbc_freq_sf,  */  \
        M2_VBC_ANGLE_SF,   /* 123,  vbc_angle_sf,  */  \
        M2_VBC_KI_SF,   /* 124,  vbc_Ki_sf,  */  \
        M2_FILTERISFC,   /* 125,  filterIsFc,  */  \
        M2_FILTERVSFC,   /* 126,  filterVsFc,  */  \
        M2_V_DECOUP_SF,   /* 127,  V_decoup_sf,  */  \
        M2_CONTROLTYPES,   /* 128,  controlTypes,  */  \
        M2_CONTROLFUNCS,   /* 129,  controlFuncs,  */  \
        M2_FAULTMTRMASK,   /* 130,  faultMtrMask,  */  \
        M2_OVERTEMPERATUREMOTOR,   /* 131,  overTemperatureMotor,  */  \
        M2_OVERTEMPERATUREMODULE,   /* 132,  overTemperatureModule,  */  \
        M2_RSONLINEWAITTIMESET,   /* 133,  RsOnlineWaitTimeSet,  */  \
        M2_RSONLINEWORKTIMESET,   /* 134,  RsOnlineWorkTimeSet,  */  \
        M2_SAMPLETRIGDELAY,   /* 135,  sampleTrigDelay,  */  \
        M2_DSSCMINDURATION,   /* 136,  dsscMinDuration,  */  \
        M2_DSSCSAMPLEDELAY,   /* 137,  dsscSampleDelay,  */  \
        M2_STARTUPTIMEDELAY,   /* 138,  startupTimeDelay,  */  \
        M2_STOPWAITTIMESET,   /* 139,  stopWaitTimeSet,  */  \
        M2_RESTARTWAITTIMESET,   /* 140,  restartWaitTimeSet,  */  \
        M2_RESTARTTIMESSET,   /* 141,  restartTimesSet,  */  \
        M2_OOBCHECKTIMESET,   /* 142,  oobCheckTimeSet,  */  \
        M2_OOBSPEEDSET_RPM,   /* 143,  oobSpeedSet_rpm,  */  \
        M2_OOBCALCCOEFSET,   /* 144,  oobCalcCoefSet,  */  \
        M2_OOBACCELSET_RPMPS,   /* 145,  oobAccelSet_rpmps,  */  \
        M2_WEIGHTCHECKTIMESET,   /* 146,  weightCheckTimeSet,  */  \
        M2_WEIGHTSPEEDSET_RPM,   /* 147,  weightSpeedSet_rpm,  */  \
        M2_WEIGHTCALCCOEFSET,   /* 148,  weightCalcCoefSet,  */  \
        M2_WEIGHTACCELSET_RPMPS,   /* 149,  weightAccelSet_rpmps,  */  \
        M2_RESERVE_PRMS1,   /* 150,  reserve_Prms1,  */  \
        M2_RESERVE_PRMS2,   /* 151,  reserve_Prms2,  */  \
    },   /* // End of motor M2 control parameters  */   \
    // Define motor M3 control parameters
    {  \
        M3_MOTORMODEL,   /* 0,  motorModel,  */  \
        M3_OVERCURRENTTIMESSET,   /* 1,  overCurrentTimesSet,  */  \
        M3_OVERLOADTIMESET,   /* 2,  overLoadTimeSet,  */  \
        M3_MOTORSTALLTIMESET,   /* 3,  motorStallTimeSet,  */  \
        M3_VOLTAGEFAULTTIMESET,   /* 4,  voltageFaultTimeSet,  */  \
        M3_STARTUPFAILTIMESET,   /* 5,  startupFailTimeSet,  */  \
        M3_OVERSPEEDTIMESET,   /* 6,  overSpeedTimeSet,  */  \
        M3_UNBALANCETIMESET,   /* 7,  unbalanceTimeSet,  */  \
        M3_LOSTPHASETIMESET,   /* 8,  lostPhaseTimeSet,  */  \
        M3_FLYINGSTARTTIMEDELAY,   /* 9,  flyingStartTimeDelay,  */  \
        M3_ALIGNTIMEDELAY,   /* 10,  alignTimeDelay,  */  \
        M3_FORCERUNTIMEDELAY,   /* 11,  forceRunTimeDelay,  */  \
        M3_CONTROLTICKSPWM,   /* 12,  controlTicksPWM,  */  \
        M3_SPEEDTICKSCONTROL,   /* 13,  speedTicksControl,  */  \
        M3_MOTOR_TYPE  ,   /* 14,  motor_type  ,  */  \
        M3_NUMPOLEPAIRS,   /* 15,  numPolePairs,  */  \
        M3_RS_OHM,   /* 16,  Rs_Ohm,  */  \
        M3_LS_D_H,   /* 17,  Ls_d_H,  */  \
        M3_LS_Q_H,   /* 18,  Ls_q_H,  */  \
        M3_FLUX_VPHZ,   /* 19,  flux_VpHz,  */  \
        M3_RR_OHM,   /* 20,  Rr_Ohm,  */  \
        M3_MAGNETICCURRENT_A,   /* 21,  magneticCurrent_A,  */  \
        M3_MAXCURRENTRESEST_A,   /* 22,  maxCurrentResEst_A,  */  \
        M3_MAXCURRENTINDEST_A,   /* 23,  maxCurrentIndEst_A,  */  \
        M3_FLUXEXCFREQ_HZ,   /* 24,  fluxExcFreq_Hz,  */  \
        M3_FLUXFILTERCOEF,   /* 25,  fluxFilterCoef,  */  \
        M3_SPEEDFILTERCOEF,   /* 26,  speedFilterCoef,  */  \
        M3_BEMFFILTERCOEF,   /* 27,  bemfFilterCoef,  */  \
        M3_SPEEDPOLE_RPS,   /* 28,  speedPole_rps,  */  \
        M3_DIRECTIONPOLE_RPS,   /* 29,  directionPole_rps,  */  \
        M3_FLUXPOLE_RPS,   /* 30,  fluxPole_rps,  */  \
        M3_RSONLINE_RDELTA_OHM,   /* 31,  RsOnLine_Rdelta_Ohm,  */  \
        M3_RSONLINE_ADELTA_RAD,   /* 32,  RsOnLine_Adelta_rad,  */  \
        M3_VOLTAGEFILTER_HZ,   /* 33,  voltageFilter_Hz,  */  \
        M3_VOLTAGESCALE_V,   /* 34,  voltageScale_V,  */  \
        M3_CURRENTSCALE_A,   /* 35,  currentScale_A,  */  \
        M3_PWMCONTROL_KHZ,   /* 36,  pwmControl_kHz,  */  \
        M3_RSONLINECURRENT_A,   /* 37,  RsOnLineCurrent_A,  */  \
        M3_ANGLEESTDELAYED_SF,   /* 38,  angleESTDelayed_sf,  */  \
        M3_MAXFREQUENCY_HZ,   /* 39,  maxFrequency_Hz,  */  \
        M3_MAXCURRENT_A,   /* 40,  maxCurrent_A,  */  \
        M3_MAXVOLTAGE_V,   /* 41,  maxVoltage_V,  */  \
        M3_MAXPEAKCURRENT_A,   /* 42,  maxPeakCurrent_A,  */  \
        M3_MAXVSMAG_PU,   /* 43,  maxVsMag_pu,  */  \
        M3_LS_D_ICOMP_COEF,   /* 44,  Ls_d_Icomp_coef,  */  \
        M3_LS_Q_ICOMP_COEF,   /* 45,  Ls_q_Icomp_coef,  */  \
        M3_LS_MIN_H,   /* 46,  Ls_min_H,  */  \
        M3_POWERCTRLSET_W,   /* 47,  powerCtrlSet_W,  */  \
        M3_OVERCURRENT_A,   /* 48,  overCurrent_A,  */  \
        M3_OVERLOADSET_W,   /* 49,  overLoadSet_W,  */  \
        M3_LOSTPHASESET_A,   /* 50,  lostPhaseSet_A,  */  \
        M3_UNBALANCERATIOSET,   /* 51,  unbalanceRatioSet,  */  \
        M3_STALLCURRENTSET_A,   /* 52,  stallCurrentSet_A,  */  \
        M3_SPEEDFAILMAXSET_HZ,   /* 53,  speedFailMaxSet_Hz,  */  \
        M3_SPEEDFAILMINSET_HZ,   /* 54,  speedFailMinSet_Hz,  */  \
        M3_ISFAILEDCHEKSET_A,   /* 55,  IsFailedChekSet_A,  */  \
        M3_TOQUEFAILMINSET_NM,   /* 56,  toqueFailMinSet_Nm,  */  \
        M3_OVERVOLTAGEFAULT_V,   /* 57,  overVoltageFault_V,  */  \
        M3_OVERVOLTAGENORM_V,   /* 58,  overVoltageNorm_V,  */  \
        M3_UNDERVOLTAGEFAULT_V,   /* 59,  underVoltageFault_V,  */  \
        M3_UNDERVOLTAGENORM_V,   /* 60,  underVoltageNorm_V,  */  \
        M3_FLUXCURRENT_A,   /* 61,  fluxCurrent_A,  */  \
        M3_ALIGNCURRENT_A,   /* 62,  alignCurrent_A,  */  \
        M3_STARTCURRENT_A,   /* 63,  startCurrent_A,  */  \
        M3_BRAKINGCURRENT_A,   /* 64,  brakingCurrent_A,  */  \
        M3_ACCELSTART_HZPS,   /* 65,  accelStart_Hzps,  */  \
        M3_ACCELSTOP_HZPS,   /* 66,  accelStop_Hzps,  */  \
        M3_ACCELRUN_HZPS,   /* 67,  accelRun_Hzps,  */  \
        M3_SPEEDFLYINGSTART_HZ,   /* 68,  speedFlyingStart_Hz,  */  \
        M3_SPEEDFORCE_HZ,   /* 69,  speedForce_Hz,  */  \
        M3_SPEEDSTART_HZ ,   /* 70,  speedStart_Hz ,  */  \
        M3_VSREF_PU,   /* 71,  VsRef_pu,  */  \
        M3_KP_FWC,   /* 72,  Kp_fwc,  */  \
        M3_KI_FWC,   /* 73,  Ki_fwc,  */  \
        M3_ANGLEFWCMAX_RAD,   /* 74,  angleFWCMax_rad,  */  \
        M3_GAIN_SPEED_HIGH_HZ,   /* 75,  Gain_speed_high_Hz,  */  \
        M3_GAIN_SPEED_LOW_HZ,   /* 76,  Gain_speed_low_Hz,  */  \
        M3_KP_SPD_HIGH_SF,   /* 77,  Kp_spd_high_sf,  */  \
        M3_KI_SPD_HIGH_SF,   /* 78,  Ki_spd_high_sf,  */  \
        M3_KP_SPD_LOW_SF,   /* 79,  Kp_spd_low_sf,  */  \
        M3_KI_SPD_LOW_SF,   /* 80,  Ki_spd_low_sf,  */  \
        M3_KP_IQ_SF,   /* 81,  Kp_Iq_sf,  */  \
        M3_KI_IQ_SF,   /* 82,  Ki_Iq_sf,  */  \
        M3_KP_ID_SF,   /* 83,  Kp_Id_sf,  */  \
        M3_KI_ID_SF,   /* 84,  Ki_Id_sf,  */  \
        M3_KP_POW_SF,   /* 85,  Kp_pow_sf,  */  \
        M3_KI_POW_SF,   /* 86,  Ki_pow_sf,  */  \
        M3_KP_SPD_START_SF,   /* 87,  Kp_spd_start_sf,  */  \
        M3_KI_SPD_START_SF,   /* 88,  Ki_spd_start_sf,  */  \
        M3_ESMO_FAST_FSW_HZ,   /* 89,  esmo_FAST_fsw_Hz,  */  \
        M3_ESMO_KSLIDEMAX,   /* 90,  esmo_KslideMax,  */  \
        M3_ESMO_KSLIDEMIN,   /* 91,  esmo_KslideMin,  */  \
        M3_ESMO_LPFFC_HZ,   /* 92,  esmo_LpfFc_Hz,  */  \
        M3_ESMO_FILTERFC_SF,   /* 93,  esmo_filterFc_sf,  */  \
        M3_ESMO_E0,   /* 94,  esmo_E0,  */  \
        M3_ESMO_PLL_KPMAX,   /* 95,  esmo_PLL_KpMax,  */  \
        M3_ESMO_PLL_KPMIN,   /* 96,  esmo_PLL_KpMin,  */  \
        M3_ESMO_PLL_KPSF,   /* 97,  esmo_PLL_KpSF,  */  \
        M3_ESMO_PLL_KI,   /* 98,  esmo_PLL_Ki,  */  \
        M3_ANGLEPLLDELAYED_SF,   /* 99,  anglePLLDelayed_sf,  */  \
        M3_HFI_KSPD,   /* 100,  hfi_Kspd,  */  \
        M3_HFI_EXCMAG_COARSE_V,   /* 101,  hfi_excMag_coarse_V,  */  \
        M3_HFI_EXCMAG_FINE_V,   /* 102,  hfi_excMag_fine_V,  */  \
        M3_HFI_WAITTIME_COARSE_SEC,   /* 103,  hfi_waitTime_coarse_sec,  */  \
        M3_HFI_WAITTIME_FINE_SEC,   /* 104,  hfi_waitTime_fine_sec,  */  \
        M3_HFI_EXCFREQ_HZ,   /* 105,  hfi_excFreq_Hz,  */  \
        M3_HFI_LPFFCSPD_HZ,   /* 106,  hfi_LpfFcSpd_Hz,  */  \
        M3_HFI_HPFFCIQ_HZ,   /* 107,  hfi_HpfFcIq_Hz,  */  \
        M3_HFI_IQMAXHFI_A,   /* 108,  hfi_IqMaxHfi_A,  */  \
        M3_HFI_IQMAXEST_A,   /* 109,  hfi_IqMaxEst_A,  */  \
        M3_HFI_IQSLOPE_A,   /* 110,  hfi_IqSlope_A,  */  \
        M3_HFI_FREQLOW_HZ,   /* 111,  hfi_freqLow_Hz,  */  \
        M3_HFI_FREQHIGH_HZ,   /* 112,  hfi_freqHigh_Hz,  */  \
        M3_PIR_SPD_F0,   /* 113,  pir_spd_f0,  */  \
        M3_PIR_SPD_FC,   /* 114,  pir_spd_fc,  */  \
        M3_PIR_SPD_K,   /* 115,  pir_spd_k,  */  \
        M3_PIR_IDQ_FC,   /* 116,  pir_Idq_fc,  */  \
        M3_PIR_ID_K,   /* 117,  pir_Id_k,  */  \
        M3_PIR_IQ_K,   /* 118,  pir_Iq_k,  */  \
        M3_VBC_ANGLE_DELTA,   /* 119,  vbc_angle_delta,  */  \
        M3_VBC_IQ_AMP,   /* 120,  vbc_Iq_amp,  */  \
        M3_VBC_IQ_SF,   /* 121,  vbc_Iq_sf,  */  \
        M3_VBC_FREQ_SF,   /* 122,  vbc_freq_sf,  */  \
        M3_VBC_ANGLE_SF,   /* 123,  vbc_angle_sf,  */  \
        M3_VBC_KI_SF,   /* 124,  vbc_Ki_sf,  */  \
        M3_FILTERISFC,   /* 125,  filterIsFc,  */  \
        M3_FILTERVSFC,   /* 126,  filterVsFc,  */  \
        M3_V_DECOUP_SF,   /* 127,  V_decoup_sf,  */  \
        M3_CONTROLTYPES,   /* 128,  controlTypes,  */  \
        M3_CONTROLFUNCS,   /* 129,  controlFuncs,  */  \
        M3_FAULTMTRMASK,   /* 130,  faultMtrMask,  */  \
        M3_OVERTEMPERATUREMOTOR,   /* 131,  overTemperatureMotor,  */  \
        M3_OVERTEMPERATUREMODULE,   /* 132,  overTemperatureModule,  */  \
        M3_RSONLINEWAITTIMESET,   /* 133,  RsOnlineWaitTimeSet,  */  \
        M3_RSONLINEWORKTIMESET,   /* 134,  RsOnlineWorkTimeSet,  */  \
        M3_SAMPLETRIGDELAY,   /* 135,  sampleTrigDelay,  */  \
        M3_DSSCMINDURATION,   /* 136,  dsscMinDuration,  */  \
        M3_DSSCSAMPLEDELAY,   /* 137,  dsscSampleDelay,  */  \
        M3_STARTUPTIMEDELAY,   /* 138,  startupTimeDelay,  */  \
        M3_STOPWAITTIMESET,   /* 139,  stopWaitTimeSet,  */  \
        M3_RESTARTWAITTIMESET,   /* 140,  restartWaitTimeSet,  */  \
        M3_RESTARTTIMESSET,   /* 141,  restartTimesSet,  */  \
        M3_OOBCHECKTIMESET,   /* 142,  oobCheckTimeSet,  */  \
        M3_OOBSPEEDSET_RPM,   /* 143,  oobSpeedSet_rpm,  */  \
        M3_OOBCALCCOEFSET,   /* 144,  oobCalcCoefSet,  */  \
        M3_OOBACCELSET_RPMPS,   /* 145,  oobAccelSet_rpmps,  */  \
        M3_WEIGHTCHECKTIMESET,   /* 146,  weightCheckTimeSet,  */  \
        M3_WEIGHTSPEEDSET_RPM,   /* 147,  weightSpeedSet_rpm,  */  \
        M3_WEIGHTCALCCOEFSET,   /* 148,  weightCalcCoefSet,  */  \
        M3_WEIGHTACCELSET_RPMPS,   /* 149,  weightAccelSet_rpmps,  */  \
        M3_RESERVE_PRMS1,   /* 150,  reserve_Prms1,  */  \
        M3_RESERVE_PRMS2,   /* 151,  reserve_Prms2,  */  \
    },   /* // End of motor M3 control parameters  */   \
    // Define motor M4 control parameters
    {  \
        M4_MOTORMODEL,   /* 0,  motorModel,  */  \
        M4_OVERCURRENTTIMESSET,   /* 1,  overCurrentTimesSet,  */  \
        M4_OVERLOADTIMESET,   /* 2,  overLoadTimeSet,  */  \
        M4_MOTORSTALLTIMESET,   /* 3,  motorStallTimeSet,  */  \
        M4_VOLTAGEFAULTTIMESET,   /* 4,  voltageFaultTimeSet,  */  \
        M4_STARTUPFAILTIMESET,   /* 5,  startupFailTimeSet,  */  \
        M4_OVERSPEEDTIMESET,   /* 6,  overSpeedTimeSet,  */  \
        M4_UNBALANCETIMESET,   /* 7,  unbalanceTimeSet,  */  \
        M4_LOSTPHASETIMESET,   /* 8,  lostPhaseTimeSet,  */  \
        M4_FLYINGSTARTTIMEDELAY,   /* 9,  flyingStartTimeDelay,  */  \
        M4_ALIGNTIMEDELAY,   /* 10,  alignTimeDelay,  */  \
        M4_FORCERUNTIMEDELAY,   /* 11,  forceRunTimeDelay,  */  \
        M4_CONTROLTICKSPWM,   /* 12,  controlTicksPWM,  */  \
        M4_SPEEDTICKSCONTROL,   /* 13,  speedTicksControl,  */  \
        M4_MOTOR_TYPE  ,   /* 14,  motor_type  ,  */  \
        M4_NUMPOLEPAIRS,   /* 15,  numPolePairs,  */  \
        M4_RS_OHM,   /* 16,  Rs_Ohm,  */  \
        M4_LS_D_H,   /* 17,  Ls_d_H,  */  \
        M4_LS_Q_H,   /* 18,  Ls_q_H,  */  \
        M4_FLUX_VPHZ,   /* 19,  flux_VpHz,  */  \
        M4_RR_OHM,   /* 20,  Rr_Ohm,  */  \
        M4_MAGNETICCURRENT_A,   /* 21,  magneticCurrent_A,  */  \
        M4_MAXCURRENTRESEST_A,   /* 22,  maxCurrentResEst_A,  */  \
        M4_MAXCURRENTINDEST_A,   /* 23,  maxCurrentIndEst_A,  */  \
        M4_FLUXEXCFREQ_HZ,   /* 24,  fluxExcFreq_Hz,  */  \
        M4_FLUXFILTERCOEF,   /* 25,  fluxFilterCoef,  */  \
        M4_SPEEDFILTERCOEF,   /* 26,  speedFilterCoef,  */  \
        M4_BEMFFILTERCOEF,   /* 27,  bemfFilterCoef,  */  \
        M4_SPEEDPOLE_RPS,   /* 28,  speedPole_rps,  */  \
        M4_DIRECTIONPOLE_RPS,   /* 29,  directionPole_rps,  */  \
        M4_FLUXPOLE_RPS,   /* 30,  fluxPole_rps,  */  \
        M4_RSONLINE_RDELTA_OHM,   /* 31,  RsOnLine_Rdelta_Ohm,  */  \
        M4_RSONLINE_ADELTA_RAD,   /* 32,  RsOnLine_Adelta_rad,  */  \
        M4_VOLTAGEFILTER_HZ,   /* 33,  voltageFilter_Hz,  */  \
        M4_VOLTAGESCALE_V,   /* 34,  voltageScale_V,  */  \
        M4_CURRENTSCALE_A,   /* 35,  currentScale_A,  */  \
        M4_PWMCONTROL_KHZ,   /* 36,  pwmControl_kHz,  */  \
        M4_RSONLINECURRENT_A,   /* 37,  RsOnLineCurrent_A,  */  \
        M4_ANGLEESTDELAYED_SF,   /* 38,  angleESTDelayed_sf,  */  \
        M4_MAXFREQUENCY_HZ,   /* 39,  maxFrequency_Hz,  */  \
        M4_MAXCURRENT_A,   /* 40,  maxCurrent_A,  */  \
        M4_MAXVOLTAGE_V,   /* 41,  maxVoltage_V,  */  \
        M4_MAXPEAKCURRENT_A,   /* 42,  maxPeakCurrent_A,  */  \
        M4_MAXVSMAG_PU,   /* 43,  maxVsMag_pu,  */  \
        M4_LS_D_ICOMP_COEF,   /* 44,  Ls_d_Icomp_coef,  */  \
        M4_LS_Q_ICOMP_COEF,   /* 45,  Ls_q_Icomp_coef,  */  \
        M4_LS_MIN_H,   /* 46,  Ls_min_H,  */  \
        M4_POWERCTRLSET_W,   /* 47,  powerCtrlSet_W,  */  \
        M4_OVERCURRENT_A,   /* 48,  overCurrent_A,  */  \
        M4_OVERLOADSET_W,   /* 49,  overLoadSet_W,  */  \
        M4_LOSTPHASESET_A,   /* 50,  lostPhaseSet_A,  */  \
        M4_UNBALANCERATIOSET,   /* 51,  unbalanceRatioSet,  */  \
        M4_STALLCURRENTSET_A,   /* 52,  stallCurrentSet_A,  */  \
        M4_SPEEDFAILMAXSET_HZ,   /* 53,  speedFailMaxSet_Hz,  */  \
        M4_SPEEDFAILMINSET_HZ,   /* 54,  speedFailMinSet_Hz,  */  \
        M4_ISFAILEDCHEKSET_A,   /* 55,  IsFailedChekSet_A,  */  \
        M4_TOQUEFAILMINSET_NM,   /* 56,  toqueFailMinSet_Nm,  */  \
        M4_OVERVOLTAGEFAULT_V,   /* 57,  overVoltageFault_V,  */  \
        M4_OVERVOLTAGENORM_V,   /* 58,  overVoltageNorm_V,  */  \
        M4_UNDERVOLTAGEFAULT_V,   /* 59,  underVoltageFault_V,  */  \
        M4_UNDERVOLTAGENORM_V,   /* 60,  underVoltageNorm_V,  */  \
        M4_FLUXCURRENT_A,   /* 61,  fluxCurrent_A,  */  \
        M4_ALIGNCURRENT_A,   /* 62,  alignCurrent_A,  */  \
        M4_STARTCURRENT_A,   /* 63,  startCurrent_A,  */  \
        M4_BRAKINGCURRENT_A,   /* 64,  brakingCurrent_A,  */  \
        M4_ACCELSTART_HZPS,   /* 65,  accelStart_Hzps,  */  \
        M4_ACCELSTOP_HZPS,   /* 66,  accelStop_Hzps,  */  \
        M4_ACCELRUN_HZPS,   /* 67,  accelRun_Hzps,  */  \
        M4_SPEEDFLYINGSTART_HZ,   /* 68,  speedFlyingStart_Hz,  */  \
        M4_SPEEDFORCE_HZ,   /* 69,  speedForce_Hz,  */  \
        M4_SPEEDSTART_HZ ,   /* 70,  speedStart_Hz ,  */  \
        M4_VSREF_PU,   /* 71,  VsRef_pu,  */  \
        M4_KP_FWC,   /* 72,  Kp_fwc,  */  \
        M4_KI_FWC,   /* 73,  Ki_fwc,  */  \
        M4_ANGLEFWCMAX_RAD,   /* 74,  angleFWCMax_rad,  */  \
        M4_GAIN_SPEED_HIGH_HZ,   /* 75,  Gain_speed_high_Hz,  */  \
        M4_GAIN_SPEED_LOW_HZ,   /* 76,  Gain_speed_low_Hz,  */  \
        M4_KP_SPD_HIGH_SF,   /* 77,  Kp_spd_high_sf,  */  \
        M4_KI_SPD_HIGH_SF,   /* 78,  Ki_spd_high_sf,  */  \
        M4_KP_SPD_LOW_SF,   /* 79,  Kp_spd_low_sf,  */  \
        M4_KI_SPD_LOW_SF,   /* 80,  Ki_spd_low_sf,  */  \
        M4_KP_IQ_SF,   /* 81,  Kp_Iq_sf,  */  \
        M4_KI_IQ_SF,   /* 82,  Ki_Iq_sf,  */  \
        M4_KP_ID_SF,   /* 83,  Kp_Id_sf,  */  \
        M4_KI_ID_SF,   /* 84,  Ki_Id_sf,  */  \
        M4_KP_POW_SF,   /* 85,  Kp_pow_sf,  */  \
        M4_KI_POW_SF,   /* 86,  Ki_pow_sf,  */  \
        M4_KP_SPD_START_SF,   /* 87,  Kp_spd_start_sf,  */  \
        M4_KI_SPD_START_SF,   /* 88,  Ki_spd_start_sf,  */  \
        M4_ESMO_FAST_FSW_HZ,   /* 89,  esmo_FAST_fsw_Hz,  */  \
        M4_ESMO_KSLIDEMAX,   /* 90,  esmo_KslideMax,  */  \
        M4_ESMO_KSLIDEMIN,   /* 91,  esmo_KslideMin,  */  \
        M4_ESMO_LPFFC_HZ,   /* 92,  esmo_LpfFc_Hz,  */  \
        M4_ESMO_FILTERFC_SF,   /* 93,  esmo_filterFc_sf,  */  \
        M4_ESMO_E0,   /* 94,  esmo_E0,  */  \
        M4_ESMO_PLL_KPMAX,   /* 95,  esmo_PLL_KpMax,  */  \
        M4_ESMO_PLL_KPMIN,   /* 96,  esmo_PLL_KpMin,  */  \
        M4_ESMO_PLL_KPSF,   /* 97,  esmo_PLL_KpSF,  */  \
        M4_ESMO_PLL_KI,   /* 98,  esmo_PLL_Ki,  */  \
        M4_ANGLEPLLDELAYED_SF,   /* 99,  anglePLLDelayed_sf,  */  \
        M4_HFI_KSPD,   /* 100,  hfi_Kspd,  */  \
        M4_HFI_EXCMAG_COARSE_V,   /* 101,  hfi_excMag_coarse_V,  */  \
        M4_HFI_EXCMAG_FINE_V,   /* 102,  hfi_excMag_fine_V,  */  \
        M4_HFI_WAITTIME_COARSE_SEC,   /* 103,  hfi_waitTime_coarse_sec,  */  \
        M4_HFI_WAITTIME_FINE_SEC,   /* 104,  hfi_waitTime_fine_sec,  */  \
        M4_HFI_EXCFREQ_HZ,   /* 105,  hfi_excFreq_Hz,  */  \
        M4_HFI_LPFFCSPD_HZ,   /* 106,  hfi_LpfFcSpd_Hz,  */  \
        M4_HFI_HPFFCIQ_HZ,   /* 107,  hfi_HpfFcIq_Hz,  */  \
        M4_HFI_IQMAXHFI_A,   /* 108,  hfi_IqMaxHfi_A,  */  \
        M4_HFI_IQMAXEST_A,   /* 109,  hfi_IqMaxEst_A,  */  \
        M4_HFI_IQSLOPE_A,   /* 110,  hfi_IqSlope_A,  */  \
        M4_HFI_FREQLOW_HZ,   /* 111,  hfi_freqLow_Hz,  */  \
        M4_HFI_FREQHIGH_HZ,   /* 112,  hfi_freqHigh_Hz,  */  \
        M4_PIR_SPD_F0,   /* 113,  pir_spd_f0,  */  \
        M4_PIR_SPD_FC,   /* 114,  pir_spd_fc,  */  \
        M4_PIR_SPD_K,   /* 115,  pir_spd_k,  */  \
        M4_PIR_IDQ_FC,   /* 116,  pir_Idq_fc,  */  \
        M4_PIR_ID_K,   /* 117,  pir_Id_k,  */  \
        M4_PIR_IQ_K,   /* 118,  pir_Iq_k,  */  \
        M4_VBC_ANGLE_DELTA,   /* 119,  vbc_angle_delta,  */  \
        M4_VBC_IQ_AMP,   /* 120,  vbc_Iq_amp,  */  \
        M4_VBC_IQ_SF,   /* 121,  vbc_Iq_sf,  */  \
        M4_VBC_FREQ_SF,   /* 122,  vbc_freq_sf,  */  \
        M4_VBC_ANGLE_SF,   /* 123,  vbc_angle_sf,  */  \
        M4_VBC_KI_SF,   /* 124,  vbc_Ki_sf,  */  \
        M4_FILTERISFC,   /* 125,  filterIsFc,  */  \
        M4_FILTERVSFC,   /* 126,  filterVsFc,  */  \
        M4_V_DECOUP_SF,   /* 127,  V_decoup_sf,  */  \
        M4_CONTROLTYPES,   /* 128,  controlTypes,  */  \
        M4_CONTROLFUNCS,   /* 129,  controlFuncs,  */  \
        M4_FAULTMTRMASK,   /* 130,  faultMtrMask,  */  \
        M4_OVERTEMPERATUREMOTOR,   /* 131,  overTemperatureMotor,  */  \
        M4_OVERTEMPERATUREMODULE,   /* 132,  overTemperatureModule,  */  \
        M4_RSONLINEWAITTIMESET,   /* 133,  RsOnlineWaitTimeSet,  */  \
        M4_RSONLINEWORKTIMESET,   /* 134,  RsOnlineWorkTimeSet,  */  \
        M4_SAMPLETRIGDELAY,   /* 135,  sampleTrigDelay,  */  \
        M4_DSSCMINDURATION,   /* 136,  dsscMinDuration,  */  \
        M4_DSSCSAMPLEDELAY,   /* 137,  dsscSampleDelay,  */  \
        M4_STARTUPTIMEDELAY,   /* 138,  startupTimeDelay,  */  \
        M4_STOPWAITTIMESET,   /* 139,  stopWaitTimeSet,  */  \
        M4_RESTARTWAITTIMESET,   /* 140,  restartWaitTimeSet,  */  \
        M4_RESTARTTIMESSET,   /* 141,  restartTimesSet,  */  \
        M4_OOBCHECKTIMESET,   /* 142,  oobCheckTimeSet,  */  \
        M4_OOBSPEEDSET_RPM,   /* 143,  oobSpeedSet_rpm,  */  \
        M4_OOBCALCCOEFSET,   /* 144,  oobCalcCoefSet,  */  \
        M4_OOBACCELSET_RPMPS,   /* 145,  oobAccelSet_rpmps,  */  \
        M4_WEIGHTCHECKTIMESET,   /* 146,  weightCheckTimeSet,  */  \
        M4_WEIGHTSPEEDSET_RPM,   /* 147,  weightSpeedSet_rpm,  */  \
        M4_WEIGHTCALCCOEFSET,   /* 148,  weightCalcCoefSet,  */  \
        M4_WEIGHTACCELSET_RPMPS,   /* 149,  weightAccelSet_rpmps,  */  \
        M4_RESERVE_PRMS1,   /* 150,  reserve_Prms1,  */  \
        M4_RESERVE_PRMS2,   /* 151,  reserve_Prms2,  */  \
    },   /* // End of motor M4 control parameters  */   \
};
#endif      // PRMS_UPDATE_EN

//
// Functions
//
#if defined(_PRMS_UPDATE)
bool updateControlPrms(MOTORSETS_Handle motorSetHandle)
{
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(motorSetHandle);

    uint16_t *ptrMotorSets;
    const uint16_t *ptrParmsList;
    uint16_t index;

    objSets->dataPrmsLength = &motorSetVars_M1.restartTimesSet - &motorSetVars_M1.motorModel;
    ptrMotorSets = &motorSetVars_M1.motorModel;
    ptrParmsList = &ctrlParamsList[objSets->dataPrmsIndexUse].motorModel;

    for(index = 0; index <= objSets->dataPrmsLength; index++)
    {
        *(ptrMotorSets + index) = *(ptrParmsList + index);
    }

    return(true);
}
#endif  // _PRMS_UPDATE

#if defined(_LFU_ENABLE) && defined(_PRMS_UPDATE)
void readDataPrmsIndex(MOTORSETS_Handle motorSetHandle)
{
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(motorSetHandle);
    uint16_t i = 0;

    FLASH_initFlashPrms();

    flashDataStatus.flashBank = Fapi_FlashBank0;

    flashDataStatus.flashAddress = DATAS1_START_ADDR;
    flashDataStatus.sectorAddress = DATAS1_SECTOR_START;
    flashDataStatus.sectorLength = DATAS1_SECTOR_LENGTH;
    flashDataStatus.dataLength = MOTOR_PRMS_LIST_WORDS;
    flashDataStatus.ptrDataAddress = &motorSetVars_M1.motorModel;

    objSets->dataPrmsNumValue = prmsIndexList[INDEX1_START_OFFSET];
    objSets->dataPrmsIndexRead = 0;

    for(i = 0; i < 16; i++)
    {
        if((objSets->dataPrmsNumValue & (0x0001<<i)) == 0)
        {
            objSets->dataPrmsIndexRead = i + 1;
        }
    }

    if(objSets->dataPrmsIndexRead != 0)
    {
        objSets->dataPrmsIndexSave = objSets->dataPrmsIndexRead;
        objSets->dataPrmsIndexUse  = objSets->dataPrmsIndexRead;
    }
    else
    {
//        objSets->dataPrmsIndexSave = objSets->dataPrmsIndexUse;
        objSets->dataPrmsIndexSave = MOTOR_PRMS_LIST_INIT_SET;
    }

    return;
}

#pragma CODE_SECTION(storeControlPrms, "lfufuncs")
void storeControlPrms(MOTORSETS_Handle motorSetHandle)
{
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(motorSetHandle);
    uint16_t i = 0;

    objSets->dataPrmsNumValue = 0xFFFF;
    objSets->dataPrmsFlashStatus = 0x8000;

    // Variables needed for Flash API Functions
    if(FLASH_activeFlashBank(flashDataStatus.flashBank) != Fapi_Status_Success)
    {
        objSets->dataPrmsFlashStatus |= 0x0100;
        return;
    }

    objSets->dataPrmsIndexSave = objSets->dataPrmsIndexSave + 1;

    if(objSets->dataPrmsIndexSave < MOTOR_PRMS_LIST_MAX)
    {
        for(i = 0; i < objSets->dataPrmsIndexSave; i++)
        {
            objSets->dataPrmsNumValue = objSets->dataPrmsNumValue - (0x0001<<i);
        }
    }
    else
    {
        objSets->dataPrmsIndexSave = 0;
        objSets->dataPrmsIndexUse = 0;

        objSets->dataPrmsNumValue = 0xFFFF;

        if(FLASH_eraseSector() != Fapi_Status_Success)
        {
            objSets->dataPrmsFlashStatus |= 0x0200;
            return;
        }
    }

    for(i = 0; i < FLASH_BUFFER_NUM; i++)
    {
        dataBuffer[i] = 0xFFFF;
    }
    dataBuffer[0] = objSets->dataPrmsNumValue;
    dataBuffer[1] = objSets->dataPrmsNumValue;
    dataBuffer[2] = objSets->dataPrmsNumValue;
    dataBuffer[3] = objSets->dataPrmsNumValue;

    flashDataStatus.dataLength = MOTOR_INDEX_LIST_LENGTH;
    flashDataStatus.flashAddress = INDEX1_START_ADDR;
    flashDataStatus.ptrDataAddress = &dataBuffer[0];
    if(FLASH_writeBufferData() != Fapi_Status_Success)
    {
        objSets->dataPrmsFlashStatus |= 0x0400;
        return;
    }

    flashDataStatus.dataLength = MOTOR_PRMS_LIST_WORDS;
    flashDataStatus.flashAddress = DATAS1_START_ADDR + MOTOR_PRMS_LIST_WORDS * objSets->dataPrmsIndexSave;
    flashDataStatus.ptrDataAddress = &motorSetVars_M1.motorModel;

    if(FLASH_writeBufferData() != Fapi_Status_Success)
    {
        objSets->dataPrmsFlashStatus |= 0x0800;
        return;
    }

    objSets->dataPrmsFlashStatus = 0x0000;

    return;
}
#endif      // _LFU_ENABLE & _PRMS_UPDATE

//
// End of File
//
