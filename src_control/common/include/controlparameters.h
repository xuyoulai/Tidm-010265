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
//! \file   /solutions/tida_010265_wminv/common/include/controlparameters.h
//!
//! \brief  header file to be included in all labs
//!         support for motor control with F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


#ifndef CONTROL_PARAMETERS_H_
#define CONTROL_PARAMETERS_H_


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup CODEUPDATE CODEUPDATE
//! @{
//
//*****************************************************************************

// the includes
#include "driverlib.h"
#include "device.h"

#if defined(_LFU_ENABLE)
#include "flash_kernel_programming.h"
#include "flash_kernel_commands.h"
#endif  //_LFU_ENABLE

//
// Defines
//

//
// Typedefs
//
typedef struct _CTRL_Params_t_
{
    MOTOR_Model_e motorModel;           // P-0 the motor model

    uint16_t overCurrentTimesSet;       // P-1
    uint16_t overLoadTimeSet;           // P-2
    uint16_t motorStallTimeSet;         // P-3
    uint16_t voltageFaultTimeSet;       // P-4
    uint16_t startupFailTimeSet;        // P-5
    uint16_t overSpeedTimeSet;          // P-6
    uint16_t unbalanceTimeSet;          // P-7
    uint16_t lostPhaseTimeSet;          // P-8

    uint16_t flyingStartTimeDelay;      // P-9
    uint16_t alignTimeDelay;            // P-10
    uint16_t forceRunTimeDelay;         // P-11

    uint16_t  controlTicksPWM;          // P-12
    uint16_t  speedTicksControl;        // P-13

    MOTOR_Type_e  motor_type;           // P-14 the motor type
    uint16_t  numPolePairs;             // P-15 the number of pole pairs for the motor
    float32_t Rs_Ohm;                   // P-16 the stator stator resistance of the motor, Ohm
    float32_t Ls_d_H;                   // P-17 the direct stator inductance of the motor, H
    float32_t Ls_q_H;                   // P-18 the quadrature stator inductance of the motor, H
    float32_t flux_VpHz;                // P-19 the rated flux of the motor, Hz
    float32_t Rr_Ohm;                   // P-20 the rotor resistance of the motor, Ohm
    float32_t magneticCurrent_A;        // P-21

    float32_t maxCurrentResEst_A;       // P-22 the maximum current value for resistance estimation, A
    float32_t maxCurrentIndEst_A;       // P-23 the maximum current value for inductance estimation, A
    float32_t fluxExcFreq_Hz;           // P-24 the flux excitation frequency, Hz

    float32_t fluxFilterCoef;           // P-25
    float32_t speedFilterCoef;          // P-26
    float32_t bemfFilterCoef;           // P-27
    float32_t speedPole_rps;            // P-28
    float32_t directionPole_rps;        // P-29
    float32_t fluxPole_rps;             // P-30

    float32_t RsOnLine_Rdelta_Ohm;      // P-31
    float32_t RsOnLine_Adelta_rad;      // P-32

    float32_t voltageFilter_Hz;         // P-33 the analog voltage filter pole location, Hz
    float32_t voltageScale_V;           // P-34 the maximum voltage at the AD converter
    float32_t currentScale_A;           // P-35 the maximum current at the AD converter
    float32_t pwmControl_kHz;           // P-36

    float32_t RsOnLineCurrent_A;        // P-37
    float32_t angleESTDelayed_sf;       // P-38

    float32_t maxFrequency_Hz;          // P-39 the maximum/rated frequency value of the motor, Hz
    float32_t maxCurrentSet_A;          // P-40 the maximum/rated current value of the motor, A
    float32_t maxVoltage_V;             // P-41 the maximum/rated voltage value of the motor, V
    float32_t maxPeakCurrent_A;         // P-42 the maximum current value of the motor, A
    float32_t maxVsMag_pu;              // P-43

    float32_t Ls_d_Icomp_coef;          // P-44
    float32_t Ls_q_Icomp_coef;          // P-45
    float32_t Ls_min_H;                 // P-46

    float32_t powerCtrlSet_W;           // P-47

    float32_t overCurrent_A;            // P-48
    float32_t overLoadSet_W;            // P-49

    float32_t lostPhaseSet_A;           // P-50
    float32_t unbalanceRatioSet;        // P-51
    float32_t stallCurrentSet_A;        // P-52
    float32_t speedFailMaxSet_Hz;       // P-53
    float32_t speedFailMinSet_Hz;       // P-54
    float32_t IsFailedChekSet_A;        // P-55
    float32_t toqueFailMinSet_Nm;       // P-56

    float32_t overVoltageFault_V;       // P-57
    float32_t overVoltageNorm_V;        // P-58
    float32_t underVoltageFault_V;      // P-59
    float32_t underVoltageNorm_V;       // P-60

    float32_t fluxCurrent_A;            // P-61
    float32_t alignCurrent_A;           // P-62
    float32_t startCurrent_A;           // P-63
    float32_t brakingCurrent_A;         // P-64

    float32_t accelStart_Hzps;          // P-65
    float32_t accelStop_Hzps;           // P-66
    float32_t accelRun_Hzps;            // P-67
    float32_t speedFlyingStart_Hz;      // P-68
    float32_t speedForce_Hz;            // P-69
    float32_t speedStart_Hz;            // P-70

    float32_t VsRef_pu;                 // P-71 the maximum Vsref for field weakening control
    float32_t Kp_fwc;                   // P-72
    float32_t Ki_fwc;                   // P-73
    float32_t angleFWCMax_rad;          // P-74

    float32_t Gain_speed_high_Hz;       // P-75
    float32_t Gain_speed_low_Hz;        // P-76
    float32_t Kp_spd_high_sf;           // P-77
    float32_t Ki_spd_high_sf;           // P-78
    float32_t Kp_spd_low_sf;            // P-79
    float32_t Ki_spd_low_sf;            // P-80

    float32_t Kp_Iq_sf;                 // P-81
    float32_t Ki_Iq_sf;                 // P-82
    float32_t Kp_Id_sf;                 // P-83
    float32_t Ki_Id_sf;                 // P-84

    float32_t Kp_pow_sf;                // P-85
    float32_t Ki_pow_sf;                // P-86

    float32_t Kp_spd_start_sf;          // P-87
    float32_t Ki_spd_start_sf;          // P-88

    float32_t esmo_FAST_fsw_Hz;         // P-89 esmp<->fast switching frequency
    float32_t esmo_KslideMax;           // P-90 sliding control gain, maximum value
    float32_t esmo_KslideMin;           // P-91 sliding control gain, minimum value
    float32_t esmo_LpfFc_Hz;            // P-92 Low Pass Filter desired cut off frequency (Hz)
    float32_t esmo_filterFc_sf;         // P-93 smo filter frequency coefficient
    float32_t esmo_E0;                  // P-94 estimated bemf threshold
    float32_t esmo_PLL_KpMax;           // P-95 maximum proportional gain
    float32_t esmo_PLL_KpMin;           // P-96 minimum proportional loop gain
    float32_t esmo_PLL_KpSF;            // P-97 proportional gain coefficient
    float32_t esmo_PLL_Ki;              // P-98 integral gain
    float32_t anglePLLDelayed_sf;       // P-99

    float32_t hfi_Kspd;                 // P-100 the speed estimation gain value
    float32_t hfi_excMag_coarse_V;      // P-101 the coarse IPD excitation voltage magnitude, V
    float32_t hfi_excMag_fine_V;        // P-102 the fine IPD excitation voltage magnitude, V
    float32_t hfi_waitTime_coarse_sec;  // P-103 the coarse wait time, sec
    float32_t hfi_waitTime_fine_sec;    // P-104 the fine wait time, sec
    float32_t hfi_excFreq_Hz;           // P-105 the excitation frequency, Hz
    float32_t hfi_LpfFcSpd_Hz;          // P-106 the low pass filter cutoff frequency, Hz
    float32_t hfi_HpfFcIq_Hz;           // P-107 the high pass filter cutoff frequency, Hz
    float32_t hfi_IqMaxHfi_A;           // P-108 the maximum Iq current when the LF/HFI estimator is active
    float32_t hfi_IqMaxEst_A;           // P-109 the maximum Iq current when the HF/EST estimator is active
    float32_t hfi_IqSlope_A;            // P-110 the trajectory slope of the Iq current
    float32_t hfi_freqLow_Hz;           // P-111 the frequency below which the LF estimator is active
    float32_t hfi_freqHigh_Hz;          // P-112 the frequency above which the HF estimator is active

    float32_t pir_spd_f0;               // P-113
    float32_t pir_spd_fc;               // P-114
    float32_t pir_spd_k;                // P-115
    float32_t pir_Idq_fc;               // P-116
    float32_t pir_Id_k;                 // P-117
    float32_t pir_Iq_k;                 // P-118

    float32_t vbc_angle_delta;          // P-119
    float32_t vbc_Iq_amp;               // P-120
    float32_t vbc_Iq_sf;                // P-121
    float32_t vbc_freq_sf;              // P-122
    float32_t vbc_angle_sf;             // P-123
    float32_t vbc_Ki_sf;                // P-124

    float32_t filterIsFc;               // P-125
    float32_t filterVsFc;               // P-126

    float32_t V_decoup_sf;              // P-127

    CONTROL_TYPES_t controlTypes;       // P-128
    CONTROL_FUNCS_t controlFuncs;       // P-129

    FAULT_MTR_REG_t faultMtrMask;       // P-130

    int16_t  overTemperatureMotor;      // P-131
    int16_t  overTemperatureModule;     // P-132

    uint16_t RsOnlineWaitTimeSet;       // P-133
    uint16_t RsOnlineWorkTimeSet;       // P-134

    uint16_t sampleTrigDelay;           // P-135
    uint16_t dsscMinDuration;           // P-136
    uint16_t dsscSampleDelay;           // P-137

    uint16_t startupTimeDelay;          // P-138
    uint16_t stopWaitTimeSet;           // P-139

    uint16_t restartWaitTimeSet;        // P-140
    uint16_t restartTimesSet;           // P-141

    uint16_t oobCheckTimeSet;           // P-142
    uint16_t oobSpeedSet_rpm;           // P-143
    uint16_t oobCalcCoefSet;            // P-144
    uint16_t oobAccelSet_rpmps;         // P-145

    uint16_t weightCheckTimeSet;        // P-146
    uint16_t weightSpeedSet_rpm;        // P-147
    uint16_t weightCalcCoefSet;         // P-148
    uint16_t weightAccelSet_rpmps;      // P-149

    uint16_t reserve_Prms1;             // P-150
    uint16_t reserve_Prms2;             // P-151
}CTRL_Params_t;

#define UPDATE_PRMS_DELAY_TIME_SET      2000         // 2000*5ms=10s
#define UPDATE_PRMS_WAIT_TIME_SET       1000         // 1000*5ms=5s

//
// define the control parameters (Copy from LCINV*.xlsx)

//-----------------------------------------------------------------------------------
// Start of the motor electrical and control parameters definitions
//-----------------------------------------------------------------------------------

#define MOTOR_PRMS_LIST_LENGTH      (150U)  //  the length of parameters list
#define MOTOR_PRMS_LIST_WORDS       (264U)
#define MOTOR_PRMS_LIST_MAX_03X     (15U)
#define MOTOR_PRMS_LIST_MAX_013X    (7U)
#define MOTOR_PRMS_LIST_INIT_SET    (3U)
#define MOTOR_PRMS_LIST_OFFSET      (4U)
#define MOTOR_INDEX_LIST_LENGTH     (4U)

// Define motor M1 control parameters
// M1 Definitions
#define M1_MOTORMODEL   M1_Teknic_M2310PL   // Motor_Model_e    motorModel,     Unit: NA,
#define M1_OVERCURRENTTIMESSET  (6U)    // uint16_t overCurrentTimesSet,    Unit: 1ms,
#define M1_OVERLOADTIMESET  (201U)  // uint16_t overLoadTimeSet,    Unit: 1ms,
#define M1_MOTORSTALLTIMESET    (201U)  // uint16_t motorStallTimeSet,  Unit: 1ms,
#define M1_VOLTAGEFAULTTIMESET  (500U)  // uint16_t voltageFaultTimeSet,    Unit: 1ms,
#define M1_STARTUPFAILTIMESET   (2000U) // uint16_t startupFailTimeSet,     Unit: 1ms,
#define M1_OVERSPEEDTIMESET (0U)    // uint16_t overSpeedTimeSet,   Unit: 1ms,
#define M1_UNBALANCETIMESET (0U)    // uint16_t unbalanceTimeSet,   Unit: 1ms,
#define M1_LOSTPHASETIMESET (0U)    // uint16_t lostPhaseTimeSet,   Unit: 1ms,
#define M1_FLYINGSTARTTIMEDELAY (15U)   // uint16_t flyingStartTimeDelay,   Unit: 1ms,
#define M1_ALIGNTIMEDELAY   (1500U) // uint16_t alignTimeDelay,     Unit: 1ms,
#define M1_FORCERUNTIMEDELAY    (15000U)    // uint16_t forceRunTimeDelay,  Unit: 1ms,
#define M1_CONTROLTICKSPWM  (1U)    // uint16_t controlTicksPWM,    Unit: 1,
#define M1_SPEEDTICKSCONTROL    (10U)   // uint16_t speedTicksControl,  Unit: 1,
#define M1_MOTOR_TYPE   MOTOR_TYPE_PM   // MOTOR_Type_e     motor_type  ,   Unit: NUM,
#define M1_NUMPOLEPAIRS (4U)    // uint16_t numPolePairs,   Unit: 1,
#define M1_RS_OHM   (0.393955588f)  // float32_t    Rs_Ohm,     Unit: ohm,
#define M1_LS_D_H   (0.000190442806f)   // float32_t    Ls_d_H,     Unit: H,
#define M1_LS_Q_H   (0.000190442806f)   // float32_t    Ls_q_H,     Unit: H,
#define M1_FLUX_VPHZ    (0.0399353318f) // float32_t    flux_VpHz,  Unit: V/Hz,
#define M1_RR_OHM   (0.0f)  // float32_t    Rr_Ohm,     Unit: ohm,
#define M1_MAGNETICCURRENT_A    (0.0f)  // float32_t    magneticCurrent_A,  Unit: A,
#define M1_MAXCURRENTRESEST_A   (1.5f)  // float32_t    maxCurrentResEst_A,     Unit: A,
#define M1_MAXCURRENTINDEST_A   (-1.0f) // float32_t    maxCurrentIndEst_A,     Unit: A,
#define M1_FLUXEXCFREQ_HZ   (60.0f) // float32_t    fluxExcFreq_Hz,     Unit: Hz,
#define M1_FLUXFILTERCOEF   (0.25f) // float32_t    fluxFilterCoef,     Unit: NUM,
#define M1_SPEEDFILTERCOEF  (1.0f)  // float32_t    speedFilterCoef,    Unit: NUM,
#define M1_BEMFFILTERCOEF   (1.0f)  // float32_t    bemfFilterCoef,     Unit: NUM,
#define M1_SPEEDPOLE_RPS    (100.0f)    // float32_t    speedPole_rps,  Unit: rps,
#define M1_DIRECTIONPOLE_RPS    (62.8318558f)   // float32_t    directionPole_rps,  Unit: rps,
#define M1_FLUXPOLE_RPS (10.0f) // float32_t    fluxPole_rps,   Unit: rps,
#define M1_RSONLINE_RDELTA_OHM  (0.0000199999995f)  // float32_t    RsOnLine_Rdelta_Ohm,    Unit: ohm,
#define M1_RSONLINE_ADELTA_RAD  (0.000500000024f)   // float32_t    RsOnLine_Adelta_rad,    Unit: rad,
#define M1_VOLTAGEFILTER_HZ (680.483887f)   // float32_t    voltageFilter_Hz,   Unit: Hz,
#define M1_VOLTAGESCALE_V   (57.5284576f)   // float32_t    voltageScale_V,     Unit: V,
#define M1_CURRENTSCALE_A   (47.1428566f)   // float32_t    currentScale_A,     Unit: A,
#define M1_PWMCONTROL_KHZ   (15.0f) // float32_t    pwmControl_kHz,     Unit: kHz,
#define M1_RSONLINECURRENT_A    (0.66f) // float32_t    RsOnLineCurrent_A,  Unit: A,
#define M1_ANGLEESTDELAYED_SF   (0.5f)  // float32_t    angleESTDelayed_sf,     Unit: 1,
#define M1_MAXFREQUENCY_HZ  (600.0f)    // float32_t    maxFrequency_Hz,    Unit: Hz,
#define M1_MAXCURRENT_A (6.5999999f)    // float32_t    maxCurrent_A,   Unit: A,
#define M1_MAXVOLTAGE_V (48.0f) // float32_t    maxVoltage_V,   Unit: V,
#define M1_MAXPEAKCURRENT_A (22.3928566f)   // float32_t    maxPeakCurrent_A,   Unit: A,
#define M1_MAXVSMAG_PU  (0.660000026f)  // float32_t    maxVsMag_pu,    Unit: 1,
#define M1_LS_D_ICOMP_COEF  (0.0227272734f) // float32_t    Ls_d_Icomp_coef,    Unit: 1,
#define M1_LS_Q_ICOMP_COEF  (0.0530303046f) // float32_t    Ls_q_Icomp_coef,    Unit: 1,
#define M1_LS_MIN_H (0.000104743543f)   // float32_t    Ls_min_H,   Unit: H,
#define M1_POWERCTRLSET_W   (90.0f) // float32_t    powerCtrlSet_W,     Unit: W,
#define M1_OVERCURRENT_A    (7.5f)  // float32_t    overCurrent_A,  Unit: A,
#define M1_OVERLOADSET_W    (90.0f) // float32_t    overLoadSet_W,  Unit: W,
#define M1_LOSTPHASESET_A   (0.00999999978f)    // float32_t    lostPhaseSet_A,     Unit: A,
#define M1_UNBALANCERATIOSET    (0.200000003f)  // float32_t    unbalanceRatioSet,  Unit: %,
#define M1_STALLCURRENTSET_A    (10.0f) // float32_t    stallCurrentSet_A,  Unit: A,
#define M1_SPEEDFAILMAXSET_HZ   (1800.0f)   // float32_t    speedFailMaxSet_Hz,     Unit: Hz,
#define M1_SPEEDFAILMINSET_HZ   (5.0f)  // float32_t    speedFailMinSet_Hz,     Unit: Hz,
#define M1_ISFAILEDCHEKSET_A    (0.200000003f)  // float32_t    IsFailedChekSet_A,  Unit: A,
#define M1_TOQUEFAILMINSET_NM   (0.000000999999997f)    // float32_t    toqueFailMinSet_Nm,     Unit: N.m,
#define M1_OVERVOLTAGEFAULT_V   (54.5f) // float32_t    overVoltageFault_V,     Unit: V,
#define M1_OVERVOLTAGENORM_V    (52.5f) // float32_t    overVoltageNorm_V,  Unit: V,
#define M1_UNDERVOLTAGEFAULT_V  (8.0f)  // float32_t    underVoltageFault_V,    Unit: V,
#define M1_UNDERVOLTAGENORM_V   (10.0f) // float32_t    underVoltageNorm_V,     Unit: V,
#define M1_FLUXCURRENT_A    (0.5f)  // float32_t    fluxCurrent_A,  Unit: A,
#define M1_ALIGNCURRENT_A   (1.5f)  // float32_t    alignCurrent_A,     Unit: A,
#define M1_STARTCURRENT_A   (3.5f)  // float32_t    startCurrent_A,     Unit: A,
#define M1_BRAKINGCURRENT_A (6.5999999f)    // float32_t    brakingCurrent_A,   Unit: A,
#define M1_ACCELSTART_HZPS  (10.0f) // float32_t    accelStart_Hzps,    Unit: Hz/s,
#define M1_ACCELSTOP_HZPS   (10.0f) // float32_t    accelStop_Hzps,     Unit: Hz/s,
#define M1_ACCELRUN_HZPS    (20.0f) // float32_t    accelRun_Hzps,  Unit: Hz/s,
#define M1_SPEEDFLYINGSTART_HZ  (3.0f)  // float32_t    speedFlyingStart_Hz,    Unit: Hz,
#define M1_SPEEDFORCE_HZ    (30.0f) // float32_t    speedForce_Hz,  Unit: Hz,
#define M1_SPEEDSTART_HZ    (35.0f) // float32_t    speedStart_Hz ,     Unit: Hz,
#define M1_VSREF_PU (0.646800041f)  // float32_t    VsRef_pu,   Unit: 1,
#define M1_KP_FWC   (0.0524999984f) // float32_t    Kp_fwc,     Unit: 1,
#define M1_KI_FWC   (0.00325000007f)    // float32_t    Ki_fwc,     Unit: 1,
#define M1_ANGLEFWCMAX_RAD  (-0.261799395f) // float32_t    angleFWCMax_rad,    Unit: rad,
#define M1_GAIN_SPEED_HIGH_HZ   (150.0f)    // float32_t    Gain_speed_high_Hz,     Unit: Hz,
#define M1_GAIN_SPEED_LOW_HZ    (60.0f) // float32_t    Gain_speed_low_Hz,  Unit: Hz,
#define M1_KP_SPD_HIGH_SF   (1.0f)  // float32_t    Kp_spd_high_sf,     Unit: 1,
#define M1_KI_SPD_HIGH_SF   (1.0f)  // float32_t    Ki_spd_high_sf,     Unit: 1,
#define M1_KP_SPD_LOW_SF    (2.0f)  // float32_t    Kp_spd_low_sf,  Unit: 1,
#define M1_KI_SPD_LOW_SF    (2.0f)  // float32_t    Ki_spd_low_sf,  Unit: 1,
#define M1_KP_IQ_SF (1.0f)  // float32_t    Kp_Iq_sf,   Unit: 1,
#define M1_KI_IQ_SF (1.0f)  // float32_t    Ki_Iq_sf,   Unit: 1,
#define M1_KP_ID_SF (1.0f)  // float32_t    Kp_Id_sf,   Unit: 1,
#define M1_KI_ID_SF (1.0f)  // float32_t    Ki_Id_sf,   Unit: 1,
#define M1_KP_POW_SF    (1.5f)  // float32_t    Kp_pow_sf,  Unit: 1,
#define M1_KI_POW_SF    (1.5f)  // float32_t    Ki_pow_sf,  Unit: 1,
#define M1_KP_SPD_START_SF  (1.5f)  // float32_t    Kp_spd_start_sf,    Unit: 1,
#define M1_KI_SPD_START_SF  (1.5f)  // float32_t    Ki_spd_start_sf,    Unit: 1,
#define M1_ESMO_FAST_FSW_HZ (400.0f)    // float32_t    esmo_FAST_fsw_Hz,   Unit: Hz,
#define M1_ESMO_KSLIDEMAX   (0.5f)  // float32_t    esmo_KslideMax,     Unit: 1,
#define M1_ESMO_KSLIDEMIN   (10.0f) // float32_t    esmo_KslideMin,     Unit: 1,
#define M1_ESMO_LPFFC_HZ    (1.5f)  // float32_t    esmo_LpfFc_Hz,  Unit: Hz,
#define M1_ESMO_FILTERFC_SF (5.0f)  // float32_t    esmo_filterFc_sf,   Unit: 1,
#define M1_ESMO_E0  (0.00000281250004f) // float32_t    esmo_E0,    Unit: 1,
#define M1_ESMO_PLL_KPMAX   (60.0f) // float32_t    esmo_PLL_KpMax,     Unit: 1,
#define M1_ESMO_PLL_KPMIN   (2.0f)  // float32_t    esmo_PLL_KpMin,     Unit: 1,
#define M1_ESMO_PLL_KPSF    (1.0f)  // float32_t    esmo_PLL_KpSF,  Unit: 1,
#define M1_ESMO_PLL_KI  (0.5f)  // float32_t    esmo_PLL_Ki,    Unit: 1,
#define M1_ANGLEPLLDELAYED_SF   (0.5f)  // float32_t    anglePLLDelayed_sf,     Unit: 1,
#define M1_HFI_KSPD (60.0f) // float32_t    hfi_Kspd,   Unit: 1,
#define M1_HFI_EXCMAG_COARSE_V  (2.0f)  // float32_t    hfi_excMag_coarse_V,    Unit: V,
#define M1_HFI_EXCMAG_FINE_V    (1.0f)  // float32_t    hfi_excMag_fine_V,  Unit: V,
#define M1_HFI_WAITTIME_COARSE_SEC  (0.5f)  // float32_t    hfi_waitTime_coarse_sec,    Unit: sec,
#define M1_HFI_WAITTIME_FINE_SEC    (0.5f)  // float32_t    hfi_waitTime_fine_sec,  Unit: sec,
#define M1_HFI_EXCFREQ_HZ   (750.0f)    // float32_t    hfi_excFreq_Hz,     Unit: Hz,
#define M1_HFI_LPFFCSPD_HZ  (35.0f) // float32_t    hfi_LpfFcSpd_Hz,    Unit: Hz,
#define M1_HFI_HPFFCIQ_HZ   (100.0f)    // float32_t    hfi_HpfFcIq_Hz,     Unit: Hz,
#define M1_HFI_IQMAXHFI_A   (0.400000006f)  // float32_t    hfi_IqMaxHfi_A,     Unit: A,
#define M1_HFI_IQMAXEST_A   (0.400000006f)  // float32_t    hfi_IqMaxEst_A,     Unit: A,
#define M1_HFI_IQSLOPE_A    (0.00000666666665f) // float32_t    hfi_IqSlope_A,  Unit: A,
#define M1_HFI_FREQLOW_HZ   (15.0f) // float32_t    hfi_freqLow_Hz,     Unit: Hz,
#define M1_HFI_FREQHIGH_HZ  (25.0f) // float32_t    hfi_freqHigh_Hz,    Unit: Hz,
#define M1_PIR_SPD_F0   (2.0f)  // float32_t    pir_spd_f0,     Unit: Hz,
#define M1_PIR_SPD_FC   (1.0f)  // float32_t    pir_spd_fc,     Unit: Hz,
#define M1_PIR_SPD_K    (0.5f)  // float32_t    pir_spd_k,  Unit: NUM,
#define M1_PIR_IDQ_FC   (0.5f)  // float32_t    pir_Idq_fc,     Unit: Hz,
#define M1_PIR_ID_K (750.0f)    // float32_t    pir_Id_k,   Unit: NUM,
#define M1_PIR_IQ_K (35.0f) // float32_t    pir_Iq_k,   Unit: NUM,
#define M1_VBC_ANGLE_DELTA  (0.2f)  // float32_t    vbc_angle_delta,    Unit: rad,
#define M1_VBC_IQ_AMP   (1.0f)  // float32_t    vbc_Iq_amp,     Unit: A,
#define M1_VBC_IQ_SF    (0.5f)  // float32_t    vbc_Iq_sf,  Unit: NUM,
#define M1_VBC_FREQ_SF  (0.5f)  // float32_t    vbc_freq_sf,    Unit: NUM,
#define M1_VBC_ANGLE_SF (0.5f)  // float32_t    vbc_angle_sf,   Unit: NUM,
#define M1_VBC_KI_SF    (0.5f)  // float32_t    vbc_Ki_sf,  Unit: NUM,
#define M1_FILTERISFC   (100.0f)    // float32_t    filterIsFc,     Unit: Hz,
#define M1_FILTERVSFC   (100.0f)    // float32_t    filterVsFc,     Unit: Hz,
#define M1_V_DECOUP_SF  (0.075f)    // float32_t    V_decoup_sf,    Unit: NUM,
#define M1_CONTROLTYPES 0   // CONTROL_TYPES_t  controlTypes,   Unit: NUM,
#define M1_CONTROLFUNCS 0   // CONTROL_FUNCS_t  controlFuncs,   Unit: NUM,
#define M1_FAULTMTRMASK 0   // FAULT_MTR_REG_t  faultMtrMask,   Unit: NUM,
#define M1_OVERTEMPERATUREMOTOR 101 // int16_t  overTemperatureMotor,   Unit: 1ms,
#define M1_OVERTEMPERATUREMODULE    91  // int16_t  overTemperatureModule,  Unit: 1ms,
#define M1_RSONLINEWAITTIMESET  (30000U)    // uint16_t RsOnlineWaitTimeSet,    Unit: 1ms,
#define M1_RSONLINEWORKTIMESET  (24000U)    // uint16_t RsOnlineWorkTimeSet,    Unit: 1ms,
#define M1_SAMPLETRIGDELAY  (30U)   // uint16_t sampleTrigDelay,    Unit: us,
#define M1_DSSCMINDURATION  (30U)   // uint16_t dsscMinDuration,    Unit: us,
#define M1_DSSCSAMPLEDELAY  (30U)   // uint16_t dsscSampleDelay,    Unit: us,
#define M1_STARTUPTIMEDELAY (30000U)    // uint16_t startupTimeDelay,   Unit: 1ms,
#define M1_STOPWAITTIMESET  (0U)    // uint16_t stopWaitTimeSet,    Unit: 1ms,
#define M1_RESTARTWAITTIMESET   (0U)    // uint16_t restartWaitTimeSet,     Unit: 1ms,
#define M1_RESTARTTIMESSET  (0U)    // uint16_t restartTimesSet,    Unit: 1ms,
#define M1_OOBCHECKTIMESET  (0U)    // uint16_t oobCheckTimeSet,    Unit: 1ms,
#define M1_OOBSPEEDSET_RPM  (0U)    // uint16_t oobSpeedSet_rpm,    Unit: 1rpm,
#define M1_OOBCALCCOEFSET   (0U)    // uint16_t oobCalcCoefSet,     Unit: 1,
#define M1_OOBACCELSET_RPMPS    (0U)    // uint16_t oobAccelSet_rpmps,  Unit: 1rpm/s,
#define M1_WEIGHTCHECKTIMESET   (0U)    // uint16_t weightCheckTimeSet,     Unit: 1ms,
#define M1_WEIGHTSPEEDSET_RPM   (0U)    // uint16_t weightSpeedSet_rpm,     Unit: 1rpm,
#define M1_WEIGHTCALCCOEFSET    (110U)  // uint16_t weightCalcCoefSet,  Unit: 1,
#define M1_WEIGHTACCELSET_RPMPS (10U)   // uint16_t weightAccelSet_rpmps,   Unit: 1rpm/s,
#define M1_RESERVE_PRMS1    (0xFFFFU)   // uint16_t reserve_Prms1,  Unit: ,
#define M1_RESERVE_PRMS2    (0xFFFFU)   // uint16_t reserve_Prms2,  Unit: ,
// End of motor M1 control parameters
// Define motor M2 control parameters
// M2 Definitions
#define M2_MOTORMODEL   M2_Estun_04APB22    // Motor_Model_e    motorModel,     Unit: NA,
#define M2_OVERCURRENTTIMESSET  (7U)    // uint16_t overCurrentTimesSet,    Unit: 1ms,
#define M2_OVERLOADTIMESET  (202U)  // uint16_t overLoadTimeSet,    Unit: 1ms,
#define M2_MOTORSTALLTIMESET    (202U)  // uint16_t motorStallTimeSet,  Unit: 1ms,
#define M2_VOLTAGEFAULTTIMESET  (500U)  // uint16_t voltageFaultTimeSet,    Unit: 1ms,
#define M2_STARTUPFAILTIMESET   (2000U) // uint16_t startupFailTimeSet,     Unit: 1ms,
#define M2_OVERSPEEDTIMESET (0U)    // uint16_t overSpeedTimeSet,   Unit: 1ms,
#define M2_UNBALANCETIMESET (0U)    // uint16_t unbalanceTimeSet,   Unit: 1ms,
#define M2_LOSTPHASETIMESET (0U)    // uint16_t lostPhaseTimeSet,   Unit: 1ms,
#define M2_FLYINGSTARTTIMEDELAY (15U)   // uint16_t flyingStartTimeDelay,   Unit: 1ms,
#define M2_ALIGNTIMEDELAY   (1500U) // uint16_t alignTimeDelay,     Unit: 1ms,
#define M2_FORCERUNTIMEDELAY    (15000U)    // uint16_t forceRunTimeDelay,  Unit: 1ms,
#define M2_CONTROLTICKSPWM  (1U)    // uint16_t controlTicksPWM,    Unit: 1,
#define M2_SPEEDTICKSCONTROL    (10U)   // uint16_t speedTicksControl,  Unit: 1,
#define M2_MOTOR_TYPE   MOTOR_TYPE_PM   // MOTOR_Type_e     motor_type  ,   Unit: NUM,
#define M2_NUMPOLEPAIRS (4U)    // uint16_t numPolePairs,   Unit: 1,
#define M2_RS_OHM   (2.3679111f)    // float32_t    Rs_Ohm,     Unit: ohm,
#define M2_LS_D_H   (0.00836551283f)    // float32_t    Ls_d_H,     Unit: H,
#define M2_LS_Q_H   (0.00836551283f)    // float32_t    Ls_q_H,     Unit: H,
#define M2_FLUX_VPHZ    (0.390533477f)  // float32_t    flux_VpHz,  Unit: V/Hz,
#define M2_RR_OHM   (0.0f)  // float32_t    Rr_Ohm,     Unit: ohm,
#define M2_MAGNETICCURRENT_A    (0.0f)  // float32_t    magneticCurrent_A,  Unit: A,
#define M2_MAXCURRENTRESEST_A   (1.0f)  // float32_t    maxCurrentResEst_A,     Unit: A,
#define M2_MAXCURRENTINDEST_A   (-1.0f) // float32_t    maxCurrentIndEst_A,     Unit: A,
#define M2_FLUXEXCFREQ_HZ   (40.0f) // float32_t    fluxExcFreq_Hz,     Unit: Hz,
#define M2_FLUXFILTERCOEF   (0.25f) // float32_t    fluxFilterCoef,     Unit: NUM,
#define M2_SPEEDFILTERCOEF  (1.0f)  // float32_t    speedFilterCoef,    Unit: NUM,
#define M2_BEMFFILTERCOEF   (1.0f)  // float32_t    bemfFilterCoef,     Unit: NUM,
#define M2_SPEEDPOLE_RPS    (100.0f)    // float32_t    speedPole_rps,  Unit: rps,
#define M2_DIRECTIONPOLE_RPS    (62.8318558f)   // float32_t    directionPole_rps,  Unit: rps,
#define M2_FLUXPOLE_RPS (10.0f) // float32_t    fluxPole_rps,   Unit: rps,
#define M2_RSONLINE_RDELTA_OHM  (0.0000199999995f)  // float32_t    RsOnLine_Rdelta_Ohm,    Unit: ohm,
#define M2_RSONLINE_ADELTA_RAD  (0.000500000024f)   // float32_t    RsOnLine_Adelta_rad,    Unit: rad,
#define M2_VOLTAGEFILTER_HZ (375.549988f)   // float32_t    voltageFilter_Hz,   Unit: Hz,
#define M2_VOLTAGESCALE_V   (409.899994f)   // float32_t    voltageScale_V,     Unit: V,
#define M2_CURRENTSCALE_A   (19.9950008f)   // float32_t    currentScale_A,     Unit: A,
#define M2_PWMCONTROL_KHZ   (15.0f) // float32_t    pwmControl_kHz,     Unit: kHz,
#define M2_RSONLINECURRENT_A    (0.66f) // float32_t    RsOnLineCurrent_A,  Unit: A,
#define M2_ANGLEESTDELAYED_SF   (0.5f)  // float32_t    angleESTDelayed_sf,     Unit: 1,
#define M2_MAXFREQUENCY_HZ  (400.0f)    // float32_t    maxFrequency_Hz,    Unit: Hz,
#define M2_MAXCURRENT_A (6.5f)  // float32_t    maxCurrent_A,   Unit: A,
#define M2_MAXVOLTAGE_V (220.0f)    // float32_t    maxVoltage_V,   Unit: V,
#define M2_MAXPEAKCURRENT_A (9.49762535f)   // float32_t    maxPeakCurrent_A,   Unit: A,
#define M2_MAXVSMAG_PU  (0.660000026f)  // float32_t    maxVsMag_pu,    Unit: 1,
#define M2_LS_D_ICOMP_COEF  (0.0230769236f) // float32_t    Ls_d_Icomp_coef,    Unit: 1,
#define M2_LS_Q_ICOMP_COEF  (0.0538461544f) // float32_t    Ls_q_Icomp_coef,    Unit: 1,
#define M2_LS_MIN_H (0.00460103201f)    // float32_t    Ls_min_H,   Unit: H,
#define M2_POWERCTRLSET_W   (250.0f)    // float32_t    powerCtrlSet_W,     Unit: W,
#define M2_OVERCURRENT_A    (6.5f)  // float32_t    overCurrent_A,  Unit: A,
#define M2_OVERLOADSET_W    (250.0f)    // float32_t    overLoadSet_W,  Unit: W,
#define M2_LOSTPHASESET_A   (0.00999999978f)    // float32_t    lostPhaseSet_A,     Unit: A,
#define M2_UNBALANCERATIOSET    (0.200000003f)  // float32_t    unbalanceRatioSet,  Unit: %,
#define M2_STALLCURRENTSET_A    (7.5f)  // float32_t    stallCurrentSet_A,  Unit: A,
#define M2_SPEEDFAILMAXSET_HZ   (500.0f)    // float32_t    speedFailMaxSet_Hz,     Unit: Hz,
#define M2_SPEEDFAILMINSET_HZ   (5.0f)  // float32_t    speedFailMinSet_Hz,     Unit: Hz,
#define M2_ISFAILEDCHEKSET_A    (0.200000003f)  // float32_t    IsFailedChekSet_A,  Unit: A,
#define M2_TOQUEFAILMINSET_NM   (0.000000999999997f)    // float32_t    toqueFailMinSet_Nm,     Unit: N.m,
#define M2_OVERVOLTAGEFAULT_V   (380.0f)    // float32_t    overVoltageFault_V,     Unit: V,
#define M2_OVERVOLTAGENORM_V    (350.0f)    // float32_t    overVoltageNorm_V,  Unit: V,
#define M2_UNDERVOLTAGEFAULT_V  (12.0f) // float32_t    underVoltageFault_V,    Unit: V,
#define M2_UNDERVOLTAGENORM_V   (15.0f) // float32_t    underVoltageNorm_V,     Unit: V,
#define M2_FLUXCURRENT_A    (0.5f)  // float32_t    fluxCurrent_A,  Unit: A,
#define M2_ALIGNCURRENT_A   (1.0f)  // float32_t    alignCurrent_A,     Unit: A,
#define M2_STARTCURRENT_A   (2.0f)  // float32_t    startCurrent_A,     Unit: A,
#define M2_BRAKINGCURRENT_A (6.5f)  // float32_t    brakingCurrent_A,   Unit: A,
#define M2_ACCELSTART_HZPS  (10.0f) // float32_t    accelStart_Hzps,    Unit: Hz/s,
#define M2_ACCELSTOP_HZPS   (10.0f) // float32_t    accelStop_Hzps,     Unit: Hz/s,
#define M2_ACCELRUN_HZPS    (20.0f) // float32_t    accelRun_Hzps,  Unit: Hz/s,
#define M2_SPEEDFLYINGSTART_HZ  (3.0f)  // float32_t    speedFlyingStart_Hz,    Unit: Hz,
#define M2_SPEEDFORCE_HZ    (30.0f) // float32_t    speedForce_Hz,  Unit: Hz,
#define M2_SPEEDSTART_HZ    (35.0f) // float32_t    speedStart_Hz ,     Unit: Hz,
#define M2_VSREF_PU (0.646800041f)  // float32_t    VsRef_pu,   Unit: 1,
#define M2_KP_FWC   (0.0524999984f) // float32_t    Kp_fwc,     Unit: 1,
#define M2_KI_FWC   (0.00325000007f)    // float32_t    Ki_fwc,     Unit: 1,
#define M2_ANGLEFWCMAX_RAD  (-0.261799395f) // float32_t    angleFWCMax_rad,    Unit: rad,
#define M2_GAIN_SPEED_HIGH_HZ   (150.0f)    // float32_t    Gain_speed_high_Hz,     Unit: Hz,
#define M2_GAIN_SPEED_LOW_HZ    (60.0f) // float32_t    Gain_speed_low_Hz,  Unit: Hz,
#define M2_KP_SPD_HIGH_SF   (1.0f)  // float32_t    Kp_spd_high_sf,     Unit: 1,
#define M2_KI_SPD_HIGH_SF   (1.0f)  // float32_t    Ki_spd_high_sf,     Unit: 1,
#define M2_KP_SPD_LOW_SF    (2.0f)  // float32_t    Kp_spd_low_sf,  Unit: 1,
#define M2_KI_SPD_LOW_SF    (2.0f)  // float32_t    Ki_spd_low_sf,  Unit: 1,
#define M2_KP_IQ_SF (1.0f)  // float32_t    Kp_Iq_sf,   Unit: 1,
#define M2_KI_IQ_SF (1.0f)  // float32_t    Ki_Iq_sf,   Unit: 1,
#define M2_KP_ID_SF (1.0f)  // float32_t    Kp_Id_sf,   Unit: 1,
#define M2_KI_ID_SF (1.0f)  // float32_t    Ki_Id_sf,   Unit: 1,
#define M2_KP_POW_SF    (1.5f)  // float32_t    Kp_pow_sf,  Unit: 1,
#define M2_KI_POW_SF    (1.5f)  // float32_t    Ki_pow_sf,  Unit: 1,
#define M2_KP_SPD_START_SF  (1.5f)  // float32_t    Kp_spd_start_sf,    Unit: 1,
#define M2_KI_SPD_START_SF  (1.5f)  // float32_t    Ki_spd_start_sf,    Unit: 1,
#define M2_ESMO_FAST_FSW_HZ (400.0f)    // float32_t    esmo_FAST_fsw_Hz,   Unit: Hz,
#define M2_ESMO_KSLIDEMAX   (0.5f)  // float32_t    esmo_KslideMax,     Unit: 1,
#define M2_ESMO_KSLIDEMIN   (10.0f) // float32_t    esmo_KslideMin,     Unit: 1,
#define M2_ESMO_LPFFC_HZ    (1.5f)  // float32_t    esmo_LpfFc_Hz,  Unit: Hz,
#define M2_ESMO_FILTERFC_SF (5.0f)  // float32_t    esmo_filterFc_sf,   Unit: 1,
#define M2_ESMO_E0  (0.00000281250004f) // float32_t    esmo_E0,    Unit: 1,
#define M2_ESMO_PLL_KPMAX   (60.0f) // float32_t    esmo_PLL_KpMax,     Unit: 1,
#define M2_ESMO_PLL_KPMIN   (2.0f)  // float32_t    esmo_PLL_KpMin,     Unit: 1,
#define M2_ESMO_PLL_KPSF    (1.0f)  // float32_t    esmo_PLL_KpSF,  Unit: 1,
#define M2_ESMO_PLL_KI  (0.5f)  // float32_t    esmo_PLL_Ki,    Unit: 1,
#define M2_ANGLEPLLDELAYED_SF   (0.5f)  // float32_t    anglePLLDelayed_sf,     Unit: 1,
#define M2_HFI_KSPD (60.0f) // float32_t    hfi_Kspd,   Unit: 1,
#define M2_HFI_EXCMAG_COARSE_V  (2.0f)  // float32_t    hfi_excMag_coarse_V,    Unit: V,
#define M2_HFI_EXCMAG_FINE_V    (1.0f)  // float32_t    hfi_excMag_fine_V,  Unit: V,
#define M2_HFI_WAITTIME_COARSE_SEC  (0.5f)  // float32_t    hfi_waitTime_coarse_sec,    Unit: sec,
#define M2_HFI_WAITTIME_FINE_SEC    (0.5f)  // float32_t    hfi_waitTime_fine_sec,  Unit: sec,
#define M2_HFI_EXCFREQ_HZ   (750.0f)    // float32_t    hfi_excFreq_Hz,     Unit: Hz,
#define M2_HFI_LPFFCSPD_HZ  (35.0f) // float32_t    hfi_LpfFcSpd_Hz,    Unit: Hz,
#define M2_HFI_HPFFCIQ_HZ   (100.0f)    // float32_t    hfi_HpfFcIq_Hz,     Unit: Hz,
#define M2_HFI_IQMAXHFI_A   (0.400000006f)  // float32_t    hfi_IqMaxHfi_A,     Unit: A,
#define M2_HFI_IQMAXEST_A   (0.400000006f)  // float32_t    hfi_IqMaxEst_A,     Unit: A,
#define M2_HFI_IQSLOPE_A    (0.00000666666665f) // float32_t    hfi_IqSlope_A,  Unit: A,
#define M2_HFI_FREQLOW_HZ   (15.0f) // float32_t    hfi_freqLow_Hz,     Unit: Hz,
#define M2_HFI_FREQHIGH_HZ  (25.0f) // float32_t    hfi_freqHigh_Hz,    Unit: Hz,
#define M2_PIR_SPD_F0   (2.0f)  // float32_t    pir_spd_f0,     Unit: Hz,
#define M2_PIR_SPD_FC   (1.0f)  // float32_t    pir_spd_fc,     Unit: Hz,
#define M2_PIR_SPD_K    (0.5f)  // float32_t    pir_spd_k,  Unit: NUM,
#define M2_PIR_IDQ_FC   (0.5f)  // float32_t    pir_Idq_fc,     Unit: Hz,
#define M2_PIR_ID_K (750.0f)    // float32_t    pir_Id_k,   Unit: NUM,
#define M2_PIR_IQ_K (35.0f) // float32_t    pir_Iq_k,   Unit: NUM,
#define M2_VBC_ANGLE_DELTA  (0.2f)  // float32_t    vbc_angle_delta,    Unit: rad,
#define M2_VBC_IQ_AMP   (1.0f)  // float32_t    vbc_Iq_amp,     Unit: A,
#define M2_VBC_IQ_SF    (0.5f)  // float32_t    vbc_Iq_sf,  Unit: NUM,
#define M2_VBC_FREQ_SF  (0.5f)  // float32_t    vbc_freq_sf,    Unit: NUM,
#define M2_VBC_ANGLE_SF (0.5f)  // float32_t    vbc_angle_sf,   Unit: NUM,
#define M2_VBC_KI_SF    (0.5f)  // float32_t    vbc_Ki_sf,  Unit: NUM,
#define M2_FILTERISFC   (100.0f)    // float32_t    filterIsFc,     Unit: Hz,
#define M2_FILTERVSFC   (100.0f)    // float32_t    filterVsFc,     Unit: Hz,
#define M2_V_DECOUP_SF  (0.075f)    // float32_t    V_decoup_sf,    Unit: NUM,
#define M2_CONTROLTYPES 0   // CONTROL_TYPES_t  controlTypes,   Unit: NUM,
#define M2_CONTROLFUNCS 0   // CONTROL_FUNCS_t  controlFuncs,   Unit: NUM,
#define M2_FAULTMTRMASK 0   // FAULT_MTR_REG_t  faultMtrMask,   Unit: NUM,
#define M2_OVERTEMPERATUREMOTOR 102 // int16_t  overTemperatureMotor,   Unit: 1ms,
#define M2_OVERTEMPERATUREMODULE    92  // int16_t  overTemperatureModule,  Unit: 1ms,
#define M2_RSONLINEWAITTIMESET  (30000U)    // uint16_t RsOnlineWaitTimeSet,    Unit: 1ms,
#define M2_RSONLINEWORKTIMESET  (24000U)    // uint16_t RsOnlineWorkTimeSet,    Unit: 1ms,
#define M2_SAMPLETRIGDELAY  (30U)   // uint16_t sampleTrigDelay,    Unit: us,
#define M2_DSSCMINDURATION  (30U)   // uint16_t dsscMinDuration,    Unit: us,
#define M2_DSSCSAMPLEDELAY  (30U)   // uint16_t dsscSampleDelay,    Unit: us,
#define M2_STARTUPTIMEDELAY (30000U)    // uint16_t startupTimeDelay,   Unit: 1ms,
#define M2_STOPWAITTIMESET  (0U)    // uint16_t stopWaitTimeSet,    Unit: 1ms,
#define M2_RESTARTWAITTIMESET   (0U)    // uint16_t restartWaitTimeSet,     Unit: 1ms,
#define M2_RESTARTTIMESSET  (0U)    // uint16_t restartTimesSet,    Unit: 1ms,
#define M2_OOBCHECKTIMESET  (0U)    // uint16_t oobCheckTimeSet,    Unit: 1ms,
#define M2_OOBSPEEDSET_RPM  (0U)    // uint16_t oobSpeedSet_rpm,    Unit: 1rpm,
#define M2_OOBCALCCOEFSET   (0U)    // uint16_t oobCalcCoefSet,     Unit: 1,
#define M2_OOBACCELSET_RPMPS    (0U)    // uint16_t oobAccelSet_rpmps,  Unit: 1rpm/s,
#define M2_WEIGHTCHECKTIMESET   (0U)    // uint16_t weightCheckTimeSet,     Unit: 1ms,
#define M2_WEIGHTSPEEDSET_RPM   (0U)    // uint16_t weightSpeedSet_rpm,     Unit: 1rpm,
#define M2_WEIGHTCALCCOEFSET    (120U)  // uint16_t weightCalcCoefSet,  Unit: 1,
#define M2_WEIGHTACCELSET_RPMPS (11U)   // uint16_t weightAccelSet_rpmps,   Unit: 1rpm/s,
#define M2_RESERVE_PRMS1    (0xFFFFU)   // uint16_t reserve_Prms1,  Unit: ,
#define M2_RESERVE_PRMS2    (0xFFFFU)   // uint16_t reserve_Prms2,  Unit: ,
// End of motor M2 control parameters
// Define motor M3 control parameters
// M3 Definitions
#define M3_MOTORMODEL   M3_LACFAN_ZKSN_750  // Motor_Model_e    motorModel,     Unit: NA,
#define M3_OVERCURRENTTIMESSET  (8U)    // uint16_t overCurrentTimesSet,    Unit: 1ms,
#define M3_OVERLOADTIMESET  (203U)  // uint16_t overLoadTimeSet,    Unit: 1ms,
#define M3_MOTORSTALLTIMESET    (203U)  // uint16_t motorStallTimeSet,  Unit: 1ms,
#define M3_VOLTAGEFAULTTIMESET  (500U)  // uint16_t voltageFaultTimeSet,    Unit: 1ms,
#define M3_STARTUPFAILTIMESET   (2000U) // uint16_t startupFailTimeSet,     Unit: 1ms,
#define M3_OVERSPEEDTIMESET (0U)    // uint16_t overSpeedTimeSet,   Unit: 1ms,
#define M3_UNBALANCETIMESET (0U)    // uint16_t unbalanceTimeSet,   Unit: 1ms,
#define M3_LOSTPHASETIMESET (0U)    // uint16_t lostPhaseTimeSet,   Unit: 1ms,
#define M3_FLYINGSTARTTIMEDELAY (15U)   // uint16_t flyingStartTimeDelay,   Unit: 1ms,
#define M3_ALIGNTIMEDELAY   (1500U) // uint16_t alignTimeDelay,     Unit: 1ms,
#define M3_FORCERUNTIMEDELAY    (15000U)    // uint16_t forceRunTimeDelay,  Unit: 1ms,
#define M3_CONTROLTICKSPWM  (1U)    // uint16_t controlTicksPWM,    Unit: 1,
#define M3_SPEEDTICKSCONTROL    (10U)   // uint16_t speedTicksControl,  Unit: 1,
#define M3_MOTOR_TYPE   MOTOR_TYPE_PM   // MOTOR_Type_e     motor_type  ,   Unit: NUM,
#define M3_NUMPOLEPAIRS (4U)    // uint16_t numPolePairs,   Unit: 1,
#define M3_RS_OHM   (2.71590972f)   // float32_t    Rs_Ohm,     Unit: ohm,
#define M3_LS_D_H   (0.0518590212f) // float32_t    Ls_d_H,     Unit: H,
#define M3_LS_Q_H   (0.0518590212f) // float32_t    Ls_q_H,     Unit: H,
#define M3_FLUX_VPHZ    (1.44512928f)   // float32_t    flux_VpHz,  Unit: V/Hz,
#define M3_RR_OHM   (0.0f)  // float32_t    Rr_Ohm,     Unit: ohm,
#define M3_MAGNETICCURRENT_A    (0.0f)  // float32_t    magneticCurrent_A,  Unit: A,
#define M3_MAXCURRENTRESEST_A   (1.5f)  // float32_t    maxCurrentResEst_A,     Unit: A,
#define M3_MAXCURRENTINDEST_A   (-1.0f) // float32_t    maxCurrentIndEst_A,     Unit: A,
#define M3_FLUXEXCFREQ_HZ   (60.0f) // float32_t    fluxExcFreq_Hz,     Unit: Hz,
#define M3_FLUXFILTERCOEF   (0.25f) // float32_t    fluxFilterCoef,     Unit: NUM,
#define M3_SPEEDFILTERCOEF  (1.0f)  // float32_t    speedFilterCoef,    Unit: NUM,
#define M3_BEMFFILTERCOEF   (1.0f)  // float32_t    bemfFilterCoef,     Unit: NUM,
#define M3_SPEEDPOLE_RPS    (100.0f)    // float32_t    speedPole_rps,  Unit: rps,
#define M3_DIRECTIONPOLE_RPS    (62.8318558f)   // float32_t    directionPole_rps,  Unit: rps,
#define M3_FLUXPOLE_RPS (10.0f) // float32_t    fluxPole_rps,   Unit: rps,
#define M3_RSONLINE_RDELTA_OHM  (0.0000199999995f)  // float32_t    RsOnLine_Rdelta_Ohm,    Unit: ohm,
#define M3_RSONLINE_ADELTA_RAD  (0.000500000024f)   // float32_t    RsOnLine_Adelta_rad,    Unit: rad,
#define M3_VOLTAGEFILTER_HZ (375.549988f)   // float32_t    voltageFilter_Hz,   Unit: Hz,
#define M3_VOLTAGESCALE_V   (409.899994f)   // float32_t    voltageScale_V,     Unit: V,
#define M3_CURRENTSCALE_A   (19.9950008f)   // float32_t    currentScale_A,     Unit: A,
#define M3_PWMCONTROL_KHZ   (15.0f) // float32_t    pwmControl_kHz,     Unit: kHz,
#define M3_RSONLINECURRENT_A    (0.66f) // float32_t    RsOnLineCurrent_A,  Unit: A,
#define M3_ANGLEESTDELAYED_SF   (0.5f)  // float32_t    angleESTDelayed_sf,     Unit: 1,
#define M3_MAXFREQUENCY_HZ  (100.0f)    // float32_t    maxFrequency_Hz,    Unit: Hz,
#define M3_MAXCURRENT_A (6.5999999f)    // float32_t    maxCurrent_A,   Unit: A,
#define M3_MAXVOLTAGE_V (220.0f)    // float32_t    maxVoltage_V,   Unit: V,
#define M3_MAXPEAKCURRENT_A (22.3928566f)   // float32_t    maxPeakCurrent_A,   Unit: A,
#define M3_MAXVSMAG_PU  (0.660000026f)  // float32_t    maxVsMag_pu,    Unit: 1,
#define M3_LS_D_ICOMP_COEF  (0.0227272734f) // float32_t    Ls_d_Icomp_coef,    Unit: 1,
#define M3_LS_Q_ICOMP_COEF  (0.0530303046f) // float32_t    Ls_q_Icomp_coef,    Unit: 1,
#define M3_LS_MIN_H (0.000104743543f)   // float32_t    Ls_min_H,   Unit: H,
#define M3_POWERCTRLSET_W   (90.0f) // float32_t    powerCtrlSet_W,     Unit: W,
#define M3_OVERCURRENT_A    (7.5f)  // float32_t    overCurrent_A,  Unit: A,
#define M3_OVERLOADSET_W    (90.0f) // float32_t    overLoadSet_W,  Unit: W,
#define M3_LOSTPHASESET_A   (0.00999999978f)    // float32_t    lostPhaseSet_A,     Unit: A,
#define M3_UNBALANCERATIOSET    (0.200000003f)  // float32_t    unbalanceRatioSet,  Unit: %,
#define M3_STALLCURRENTSET_A    (10.0f) // float32_t    stallCurrentSet_A,  Unit: A,
#define M3_SPEEDFAILMAXSET_HZ   (1800.0f)   // float32_t    speedFailMaxSet_Hz,     Unit: Hz,
#define M3_SPEEDFAILMINSET_HZ   (5.0f)  // float32_t    speedFailMinSet_Hz,     Unit: Hz,
#define M3_ISFAILEDCHEKSET_A    (0.200000003f)  // float32_t    IsFailedChekSet_A,  Unit: A,
#define M3_TOQUEFAILMINSET_NM   (0.000000999999997f)    // float32_t    toqueFailMinSet_Nm,     Unit: N.m,
#define M3_OVERVOLTAGEFAULT_V   (54.5f) // float32_t    overVoltageFault_V,     Unit: V,
#define M3_OVERVOLTAGENORM_V    (52.5f) // float32_t    overVoltageNorm_V,  Unit: V,
#define M3_UNDERVOLTAGEFAULT_V  (8.0f)  // float32_t    underVoltageFault_V,    Unit: V,
#define M3_UNDERVOLTAGENORM_V   (10.0f) // float32_t    underVoltageNorm_V,     Unit: V,
#define M3_FLUXCURRENT_A    (0.5f)  // float32_t    fluxCurrent_A,  Unit: A,
#define M3_ALIGNCURRENT_A   (1.5f)  // float32_t    alignCurrent_A,     Unit: A,
#define M3_STARTCURRENT_A   (3.5f)  // float32_t    startCurrent_A,     Unit: A,
#define M3_BRAKINGCURRENT_A (6.5999999f)    // float32_t    brakingCurrent_A,   Unit: A,
#define M3_ACCELSTART_HZPS  (10.0f) // float32_t    accelStart_Hzps,    Unit: Hz/s,
#define M3_ACCELSTOP_HZPS   (10.0f) // float32_t    accelStop_Hzps,     Unit: Hz/s,
#define M3_ACCELRUN_HZPS    (20.0f) // float32_t    accelRun_Hzps,  Unit: Hz/s,
#define M3_SPEEDFLYINGSTART_HZ  (3.0f)  // float32_t    speedFlyingStart_Hz,    Unit: Hz,
#define M3_SPEEDFORCE_HZ    (30.0f) // float32_t    speedForce_Hz,  Unit: Hz,
#define M3_SPEEDSTART_HZ    (35.0f) // float32_t    speedStart_Hz ,     Unit: Hz,
#define M3_VSREF_PU (0.646800041f)  // float32_t    VsRef_pu,   Unit: 1,
#define M3_KP_FWC   (0.0524999984f) // float32_t    Kp_fwc,     Unit: 1,
#define M3_KI_FWC   (0.00325000007f)    // float32_t    Ki_fwc,     Unit: 1,
#define M3_ANGLEFWCMAX_RAD  (-0.261799395f) // float32_t    angleFWCMax_rad,    Unit: rad,
#define M3_GAIN_SPEED_HIGH_HZ   (150.0f)    // float32_t    Gain_speed_high_Hz,     Unit: Hz,
#define M3_GAIN_SPEED_LOW_HZ    (60.0f) // float32_t    Gain_speed_low_Hz,  Unit: Hz,
#define M3_KP_SPD_HIGH_SF   (1.0f)  // float32_t    Kp_spd_high_sf,     Unit: 1,
#define M3_KI_SPD_HIGH_SF   (1.0f)  // float32_t    Ki_spd_high_sf,     Unit: 1,
#define M3_KP_SPD_LOW_SF    (2.0f)  // float32_t    Kp_spd_low_sf,  Unit: 1,
#define M3_KI_SPD_LOW_SF    (2.0f)  // float32_t    Ki_spd_low_sf,  Unit: 1,
#define M3_KP_IQ_SF (1.0f)  // float32_t    Kp_Iq_sf,   Unit: 1,
#define M3_KI_IQ_SF (1.0f)  // float32_t    Ki_Iq_sf,   Unit: 1,
#define M3_KP_ID_SF (1.0f)  // float32_t    Kp_Id_sf,   Unit: 1,
#define M3_KI_ID_SF (1.0f)  // float32_t    Ki_Id_sf,   Unit: 1,
#define M3_KP_POW_SF    (1.5f)  // float32_t    Kp_pow_sf,  Unit: 1,
#define M3_KI_POW_SF    (1.5f)  // float32_t    Ki_pow_sf,  Unit: 1,
#define M3_KP_SPD_START_SF  (1.5f)  // float32_t    Kp_spd_start_sf,    Unit: 1,
#define M3_KI_SPD_START_SF  (1.5f)  // float32_t    Ki_spd_start_sf,    Unit: 1,
#define M3_ESMO_FAST_FSW_HZ (400.0f)    // float32_t    esmo_FAST_fsw_Hz,   Unit: Hz,
#define M3_ESMO_KSLIDEMAX   (0.5f)  // float32_t    esmo_KslideMax,     Unit: 1,
#define M3_ESMO_KSLIDEMIN   (10.0f) // float32_t    esmo_KslideMin,     Unit: 1,
#define M3_ESMO_LPFFC_HZ    (1.5f)  // float32_t    esmo_LpfFc_Hz,  Unit: Hz,
#define M3_ESMO_FILTERFC_SF (5.0f)  // float32_t    esmo_filterFc_sf,   Unit: 1,
#define M3_ESMO_E0  (0.00000281250004f) // float32_t    esmo_E0,    Unit: 1,
#define M3_ESMO_PLL_KPMAX   (60.0f) // float32_t    esmo_PLL_KpMax,     Unit: 1,
#define M3_ESMO_PLL_KPMIN   (2.0f)  // float32_t    esmo_PLL_KpMin,     Unit: 1,
#define M3_ESMO_PLL_KPSF    (1.0f)  // float32_t    esmo_PLL_KpSF,  Unit: 1,
#define M3_ESMO_PLL_KI  (0.5f)  // float32_t    esmo_PLL_Ki,    Unit: 1,
#define M3_ANGLEPLLDELAYED_SF   (0.5f)  // float32_t    anglePLLDelayed_sf,     Unit: 1,
#define M3_HFI_KSPD (60.0f) // float32_t    hfi_Kspd,   Unit: 1,
#define M3_HFI_EXCMAG_COARSE_V  (2.0f)  // float32_t    hfi_excMag_coarse_V,    Unit: V,
#define M3_HFI_EXCMAG_FINE_V    (1.0f)  // float32_t    hfi_excMag_fine_V,  Unit: V,
#define M3_HFI_WAITTIME_COARSE_SEC  (0.5f)  // float32_t    hfi_waitTime_coarse_sec,    Unit: sec,
#define M3_HFI_WAITTIME_FINE_SEC    (0.5f)  // float32_t    hfi_waitTime_fine_sec,  Unit: sec,
#define M3_HFI_EXCFREQ_HZ   (750.0f)    // float32_t    hfi_excFreq_Hz,     Unit: Hz,
#define M3_HFI_LPFFCSPD_HZ  (35.0f) // float32_t    hfi_LpfFcSpd_Hz,    Unit: Hz,
#define M3_HFI_HPFFCIQ_HZ   (100.0f)    // float32_t    hfi_HpfFcIq_Hz,     Unit: Hz,
#define M3_HFI_IQMAXHFI_A   (0.400000006f)  // float32_t    hfi_IqMaxHfi_A,     Unit: A,
#define M3_HFI_IQMAXEST_A   (0.400000006f)  // float32_t    hfi_IqMaxEst_A,     Unit: A,
#define M3_HFI_IQSLOPE_A    (0.00000666666665f) // float32_t    hfi_IqSlope_A,  Unit: A,
#define M3_HFI_FREQLOW_HZ   (15.0f) // float32_t    hfi_freqLow_Hz,     Unit: Hz,
#define M3_HFI_FREQHIGH_HZ  (25.0f) // float32_t    hfi_freqHigh_Hz,    Unit: Hz,
#define M3_PIR_SPD_F0   (120.0f)    // float32_t    pir_spd_f0,     Unit: Hz,
#define M3_PIR_SPD_FC   (60.0f) // float32_t    pir_spd_fc,     Unit: Hz,
#define M3_PIR_SPD_K    (0.01f) // float32_t    pir_spd_k,  Unit: NUM,
#define M3_PIR_IDQ_FC   (120.0f)    // float32_t    pir_Idq_fc,     Unit: Hz,
#define M3_PIR_ID_K (0.001f)    // float32_t    pir_Id_k,   Unit: NUM,
#define M3_PIR_IQ_K (0.01f) // float32_t    pir_Iq_k,   Unit: NUM,
#define M3_VBC_ANGLE_DELTA  (0.2f)  // float32_t    vbc_angle_delta,    Unit: rad,
#define M3_VBC_IQ_AMP   (1.0f)  // float32_t    vbc_Iq_amp,     Unit: A,
#define M3_VBC_IQ_SF    (0.5f)  // float32_t    vbc_Iq_sf,  Unit: NUM,
#define M3_VBC_FREQ_SF  (0.5f)  // float32_t    vbc_freq_sf,    Unit: NUM,
#define M3_VBC_ANGLE_SF (0.5f)  // float32_t    vbc_angle_sf,   Unit: NUM,
#define M3_VBC_KI_SF    (0.5f)  // float32_t    vbc_Ki_sf,  Unit: NUM,
#define M3_FILTERISFC   (100.0f)    // float32_t    filterIsFc,     Unit: Hz,
#define M3_FILTERVSFC   (100.0f)    // float32_t    filterVsFc,     Unit: Hz,
#define M3_V_DECOUP_SF  (0.075f)    // float32_t    V_decoup_sf,    Unit: NUM,
#define M3_CONTROLTYPES 0   // CONTROL_TYPES_t  controlTypes,   Unit: NUM,
#define M3_CONTROLFUNCS 0   // CONTROL_FUNCS_t  controlFuncs,   Unit: NUM,
#define M3_FAULTMTRMASK 0   // FAULT_MTR_REG_t  faultMtrMask,   Unit: NUM,
#define M3_OVERTEMPERATUREMOTOR 103 // int16_t  overTemperatureMotor,   Unit: 1ms,
#define M3_OVERTEMPERATUREMODULE    93  // int16_t  overTemperatureModule,  Unit: 1ms,
#define M3_RSONLINEWAITTIMESET  (30000U)    // uint16_t RsOnlineWaitTimeSet,    Unit: 1ms,
#define M3_RSONLINEWORKTIMESET  (24000U)    // uint16_t RsOnlineWorkTimeSet,    Unit: 1ms,
#define M3_SAMPLETRIGDELAY  (30U)   // uint16_t sampleTrigDelay,    Unit: us,
#define M3_DSSCMINDURATION  (30U)   // uint16_t dsscMinDuration,    Unit: us,
#define M3_DSSCSAMPLEDELAY  (30U)   // uint16_t dsscSampleDelay,    Unit: us,
#define M3_STARTUPTIMEDELAY (30000U)    // uint16_t startupTimeDelay,   Unit: 1ms,
#define M3_STOPWAITTIMESET  (0U)    // uint16_t stopWaitTimeSet,    Unit: 1ms,
#define M3_RESTARTWAITTIMESET   (0U)    // uint16_t restartWaitTimeSet,     Unit: 1ms,
#define M3_RESTARTTIMESSET  (0U)    // uint16_t restartTimesSet,    Unit: 1ms,
#define M3_OOBCHECKTIMESET  (0U)    // uint16_t oobCheckTimeSet,    Unit: 1ms,
#define M3_OOBSPEEDSET_RPM  (0U)    // uint16_t oobSpeedSet_rpm,    Unit: 1rpm,
#define M3_OOBCALCCOEFSET   (0U)    // uint16_t oobCalcCoefSet,     Unit: 1,
#define M3_OOBACCELSET_RPMPS    (0U)    // uint16_t oobAccelSet_rpmps,  Unit: 1rpm/s,
#define M3_WEIGHTCHECKTIMESET   (0U)    // uint16_t weightCheckTimeSet,     Unit: 1ms,
#define M3_WEIGHTSPEEDSET_RPM   (0U)    // uint16_t weightSpeedSet_rpm,     Unit: 1rpm,
#define M3_WEIGHTCALCCOEFSET    (130U)  // uint16_t weightCalcCoefSet,  Unit: 1,
#define M3_WEIGHTACCELSET_RPMPS (12U)   // uint16_t weightAccelSet_rpmps,   Unit: 1rpm/s,
#define M3_RESERVE_PRMS1    (0xFFFFU)   // uint16_t reserve_Prms1,  Unit: ,
#define M3_RESERVE_PRMS2    (0xFFFFU)   // uint16_t reserve_Prms2,  Unit: ,
// End of motor M3 control parameters
// Define motor M4 control parameters
// M4 Definitions
#define M4_MOTORMODEL   M4_Marathon_N56PNRA // Motor_Model_e    motorModel,     Unit: NA,
#define M4_OVERCURRENTTIMESSET  (9U)    // uint16_t overCurrentTimesSet,    Unit: 1ms,
#define M4_OVERLOADTIMESET  (204U)  // uint16_t overLoadTimeSet,    Unit: 1ms,
#define M4_MOTORSTALLTIMESET    (204U)  // uint16_t motorStallTimeSet,  Unit: 1ms,
#define M4_VOLTAGEFAULTTIMESET  (500U)  // uint16_t voltageFaultTimeSet,    Unit: 1ms,
#define M4_STARTUPFAILTIMESET   (2000U) // uint16_t startupFailTimeSet,     Unit: 1ms,
#define M4_OVERSPEEDTIMESET (0U)    // uint16_t overSpeedTimeSet,   Unit: 1ms,
#define M4_UNBALANCETIMESET (0U)    // uint16_t unbalanceTimeSet,   Unit: 1ms,
#define M4_LOSTPHASETIMESET (0U)    // uint16_t lostPhaseTimeSet,   Unit: 1ms,
#define M4_FLYINGSTARTTIMEDELAY (15U)   // uint16_t flyingStartTimeDelay,   Unit: 1ms,
#define M4_ALIGNTIMEDELAY   (1500U) // uint16_t alignTimeDelay,     Unit: 1ms,
#define M4_FORCERUNTIMEDELAY    (15000U)    // uint16_t forceRunTimeDelay,  Unit: 1ms,
#define M4_CONTROLTICKSPWM  (1U)    // uint16_t controlTicksPWM,    Unit: 1,
#define M4_SPEEDTICKSCONTROL    (10U)   // uint16_t speedTicksControl,  Unit: 1,
#define M4_MOTOR_TYPE   MOTOR_TYPE_PM   // MOTOR_Type_e     motor_type  ,   Unit: NUM,
#define M4_NUMPOLEPAIRS (4U)    // uint16_t numPolePairs,   Unit: 1,
#define M4_RS_OHM   (2.20022106f)   // float32_t    Rs_Ohm,     Unit: ohm,
#define M4_LS_D_H   (0.00872102287f)    // float32_t    Ls_d_H,     Unit: H,
#define M4_LS_Q_H   (0.00872102287f)    // float32_t    Ls_q_H,     Unit: H,
#define M4_FLUX_VPHZ    (0.38469851f)   // float32_t    flux_VpHz,  Unit: V/Hz,
#define M4_RR_OHM   (0.0f)  // float32_t    Rr_Ohm,     Unit: ohm,
#define M4_MAGNETICCURRENT_A    (0.0f)  // float32_t    magneticCurrent_A,  Unit: A,
#define M4_MAXCURRENTRESEST_A   (1.5f)  // float32_t    maxCurrentResEst_A,     Unit: A,
#define M4_MAXCURRENTINDEST_A   (-1.0f) // float32_t    maxCurrentIndEst_A,     Unit: A,
#define M4_FLUXEXCFREQ_HZ   (60.0f) // float32_t    fluxExcFreq_Hz,     Unit: Hz,
#define M4_FLUXFILTERCOEF   (0.25f) // float32_t    fluxFilterCoef,     Unit: NUM,
#define M4_SPEEDFILTERCOEF  (1.0f)  // float32_t    speedFilterCoef,    Unit: NUM,
#define M4_BEMFFILTERCOEF   (1.0f)  // float32_t    bemfFilterCoef,     Unit: NUM,
#define M4_SPEEDPOLE_RPS    (100.0f)    // float32_t    speedPole_rps,  Unit: rps,
#define M4_DIRECTIONPOLE_RPS    (62.8318558f)   // float32_t    directionPole_rps,  Unit: rps,
#define M4_FLUXPOLE_RPS (10.0f) // float32_t    fluxPole_rps,   Unit: rps,
#define M4_RSONLINE_RDELTA_OHM  (0.0000199999995f)  // float32_t    RsOnLine_Rdelta_Ohm,    Unit: ohm,
#define M4_RSONLINE_ADELTA_RAD  (0.000500000024f)   // float32_t    RsOnLine_Adelta_rad,    Unit: rad,
#define M4_VOLTAGEFILTER_HZ (375.549988f)   // float32_t    voltageFilter_Hz,   Unit: Hz,
#define M4_VOLTAGESCALE_V   (409.899994f)   // float32_t    voltageScale_V,     Unit: V,
#define M4_CURRENTSCALE_A   (19.9950008f)   // float32_t    currentScale_A,     Unit: A,
#define M4_PWMCONTROL_KHZ   (15.0f) // float32_t    pwmControl_kHz,     Unit: kHz,
#define M4_RSONLINECURRENT_A    (0.66f) // float32_t    RsOnLineCurrent_A,  Unit: A,
#define M4_ANGLEESTDELAYED_SF   (0.5f)  // float32_t    angleESTDelayed_sf,     Unit: 1,
#define M4_MAXFREQUENCY_HZ  (600.0f)    // float32_t    maxFrequency_Hz,    Unit: Hz,
#define M4_MAXCURRENT_A (6.5999999f)    // float32_t    maxCurrent_A,   Unit: A,
#define M4_MAXVOLTAGE_V (220.0f)    // float32_t    maxVoltage_V,   Unit: V,
#define M4_MAXPEAKCURRENT_A (22.3928566f)   // float32_t    maxPeakCurrent_A,   Unit: A,
#define M4_MAXVSMAG_PU  (0.660000026f)  // float32_t    maxVsMag_pu,    Unit: 1,
#define M4_LS_D_ICOMP_COEF  (0.0227272734f) // float32_t    Ls_d_Icomp_coef,    Unit: 1,
#define M4_LS_Q_ICOMP_COEF  (0.0530303046f) // float32_t    Ls_q_Icomp_coef,    Unit: 1,
#define M4_LS_MIN_H (0.000104743543f)   // float32_t    Ls_min_H,   Unit: H,
#define M4_POWERCTRLSET_W   (90.0f) // float32_t    powerCtrlSet_W,     Unit: W,
#define M4_OVERCURRENT_A    (7.5f)  // float32_t    overCurrent_A,  Unit: A,
#define M4_OVERLOADSET_W    (90.0f) // float32_t    overLoadSet_W,  Unit: W,
#define M4_LOSTPHASESET_A   (0.00999999978f)    // float32_t    lostPhaseSet_A,     Unit: A,
#define M4_UNBALANCERATIOSET    (0.200000003f)  // float32_t    unbalanceRatioSet,  Unit: %,
#define M4_STALLCURRENTSET_A    (10.0f) // float32_t    stallCurrentSet_A,  Unit: A,
#define M4_SPEEDFAILMAXSET_HZ   (1800.0f)   // float32_t    speedFailMaxSet_Hz,     Unit: Hz,
#define M4_SPEEDFAILMINSET_HZ   (5.0f)  // float32_t    speedFailMinSet_Hz,     Unit: Hz,
#define M4_ISFAILEDCHEKSET_A    (0.200000003f)  // float32_t    IsFailedChekSet_A,  Unit: A,
#define M4_TOQUEFAILMINSET_NM   (0.000000999999997f)    // float32_t    toqueFailMinSet_Nm,     Unit: N.m,
#define M4_OVERVOLTAGEFAULT_V   (54.5f) // float32_t    overVoltageFault_V,     Unit: V,
#define M4_OVERVOLTAGENORM_V    (52.5f) // float32_t    overVoltageNorm_V,  Unit: V,
#define M4_UNDERVOLTAGEFAULT_V  (8.0f)  // float32_t    underVoltageFault_V,    Unit: V,
#define M4_UNDERVOLTAGENORM_V   (10.0f) // float32_t    underVoltageNorm_V,     Unit: V,
#define M4_FLUXCURRENT_A    (0.5f)  // float32_t    fluxCurrent_A,  Unit: A,
#define M4_ALIGNCURRENT_A   (1.5f)  // float32_t    alignCurrent_A,     Unit: A,
#define M4_STARTCURRENT_A   (3.5f)  // float32_t    startCurrent_A,     Unit: A,
#define M4_BRAKINGCURRENT_A (6.5999999f)    // float32_t    brakingCurrent_A,   Unit: A,
#define M4_ACCELSTART_HZPS  (10.0f) // float32_t    accelStart_Hzps,    Unit: Hz/s,
#define M4_ACCELSTOP_HZPS   (10.0f) // float32_t    accelStop_Hzps,     Unit: Hz/s,
#define M4_ACCELRUN_HZPS    (20.0f) // float32_t    accelRun_Hzps,  Unit: Hz/s,
#define M4_SPEEDFLYINGSTART_HZ  (3.0f)  // float32_t    speedFlyingStart_Hz,    Unit: Hz,
#define M4_SPEEDFORCE_HZ    (30.0f) // float32_t    speedForce_Hz,  Unit: Hz,
#define M4_SPEEDSTART_HZ    (35.0f) // float32_t    speedStart_Hz ,     Unit: Hz,
#define M4_VSREF_PU (0.646800041f)  // float32_t    VsRef_pu,   Unit: 1,
#define M4_KP_FWC   (0.0524999984f) // float32_t    Kp_fwc,     Unit: 1,
#define M4_KI_FWC   (0.00325000007f)    // float32_t    Ki_fwc,     Unit: 1,
#define M4_ANGLEFWCMAX_RAD  (-0.261799395f) // float32_t    angleFWCMax_rad,    Unit: rad,
#define M4_GAIN_SPEED_HIGH_HZ   (150.0f)    // float32_t    Gain_speed_high_Hz,     Unit: Hz,
#define M4_GAIN_SPEED_LOW_HZ    (60.0f) // float32_t    Gain_speed_low_Hz,  Unit: Hz,
#define M4_KP_SPD_HIGH_SF   (1.0f)  // float32_t    Kp_spd_high_sf,     Unit: 1,
#define M4_KI_SPD_HIGH_SF   (1.0f)  // float32_t    Ki_spd_high_sf,     Unit: 1,
#define M4_KP_SPD_LOW_SF    (2.0f)  // float32_t    Kp_spd_low_sf,  Unit: 1,
#define M4_KI_SPD_LOW_SF    (2.0f)  // float32_t    Ki_spd_low_sf,  Unit: 1,
#define M4_KP_IQ_SF (1.0f)  // float32_t    Kp_Iq_sf,   Unit: 1,
#define M4_KI_IQ_SF (1.0f)  // float32_t    Ki_Iq_sf,   Unit: 1,
#define M4_KP_ID_SF (1.0f)  // float32_t    Kp_Id_sf,   Unit: 1,
#define M4_KI_ID_SF (1.0f)  // float32_t    Ki_Id_sf,   Unit: 1,
#define M4_KP_POW_SF    (1.5f)  // float32_t    Kp_pow_sf,  Unit: 1,
#define M4_KI_POW_SF    (1.5f)  // float32_t    Ki_pow_sf,  Unit: 1,
#define M4_KP_SPD_START_SF  (1.5f)  // float32_t    Kp_spd_start_sf,    Unit: 1,
#define M4_KI_SPD_START_SF  (1.5f)  // float32_t    Ki_spd_start_sf,    Unit: 1,
#define M4_ESMO_FAST_FSW_HZ (400.0f)    // float32_t    esmo_FAST_fsw_Hz,   Unit: Hz,
#define M4_ESMO_KSLIDEMAX   (0.5f)  // float32_t    esmo_KslideMax,     Unit: 1,
#define M4_ESMO_KSLIDEMIN   (10.0f) // float32_t    esmo_KslideMin,     Unit: 1,
#define M4_ESMO_LPFFC_HZ    (1.5f)  // float32_t    esmo_LpfFc_Hz,  Unit: Hz,
#define M4_ESMO_FILTERFC_SF (5.0f)  // float32_t    esmo_filterFc_sf,   Unit: 1,
#define M4_ESMO_E0  (0.00000281250004f) // float32_t    esmo_E0,    Unit: 1,
#define M4_ESMO_PLL_KPMAX   (60.0f) // float32_t    esmo_PLL_KpMax,     Unit: 1,
#define M4_ESMO_PLL_KPMIN   (2.0f)  // float32_t    esmo_PLL_KpMin,     Unit: 1,
#define M4_ESMO_PLL_KPSF    (1.0f)  // float32_t    esmo_PLL_KpSF,  Unit: 1,
#define M4_ESMO_PLL_KI  (0.5f)  // float32_t    esmo_PLL_Ki,    Unit: 1,
#define M4_ANGLEPLLDELAYED_SF   (0.5f)  // float32_t    anglePLLDelayed_sf,     Unit: 1,
#define M4_HFI_KSPD (60.0f) // float32_t    hfi_Kspd,   Unit: 1,
#define M4_HFI_EXCMAG_COARSE_V  (2.0f)  // float32_t    hfi_excMag_coarse_V,    Unit: V,
#define M4_HFI_EXCMAG_FINE_V    (1.0f)  // float32_t    hfi_excMag_fine_V,  Unit: V,
#define M4_HFI_WAITTIME_COARSE_SEC  (0.5f)  // float32_t    hfi_waitTime_coarse_sec,    Unit: sec,
#define M4_HFI_WAITTIME_FINE_SEC    (0.5f)  // float32_t    hfi_waitTime_fine_sec,  Unit: sec,
#define M4_HFI_EXCFREQ_HZ   (750.0f)    // float32_t    hfi_excFreq_Hz,     Unit: Hz,
#define M4_HFI_LPFFCSPD_HZ  (35.0f) // float32_t    hfi_LpfFcSpd_Hz,    Unit: Hz,
#define M4_HFI_HPFFCIQ_HZ   (100.0f)    // float32_t    hfi_HpfFcIq_Hz,     Unit: Hz,
#define M4_HFI_IQMAXHFI_A   (0.400000006f)  // float32_t    hfi_IqMaxHfi_A,     Unit: A,
#define M4_HFI_IQMAXEST_A   (0.400000006f)  // float32_t    hfi_IqMaxEst_A,     Unit: A,
#define M4_HFI_IQSLOPE_A    (0.00000666666665f) // float32_t    hfi_IqSlope_A,  Unit: A,
#define M4_HFI_FREQLOW_HZ   (15.0f) // float32_t    hfi_freqLow_Hz,     Unit: Hz,
#define M4_HFI_FREQHIGH_HZ  (25.0f) // float32_t    hfi_freqHigh_Hz,    Unit: Hz,
#define M4_PIR_SPD_F0   (120.0f)    // float32_t    pir_spd_f0,     Unit: Hz,
#define M4_PIR_SPD_FC   (60.0f) // float32_t    pir_spd_fc,     Unit: Hz,
#define M4_PIR_SPD_K    (0.01f) // float32_t    pir_spd_k,  Unit: NUM,
#define M4_PIR_IDQ_FC   (120.0f)    // float32_t    pir_Idq_fc,     Unit: Hz,
#define M4_PIR_ID_K (0.001f)    // float32_t    pir_Id_k,   Unit: NUM,
#define M4_PIR_IQ_K (0.01f) // float32_t    pir_Iq_k,   Unit: NUM,
#define M4_VBC_ANGLE_DELTA  (0.2f)  // float32_t    vbc_angle_delta,    Unit: rad,
#define M4_VBC_IQ_AMP   (1.0f)  // float32_t    vbc_Iq_amp,     Unit: A,
#define M4_VBC_IQ_SF    (0.5f)  // float32_t    vbc_Iq_sf,  Unit: NUM,
#define M4_VBC_FREQ_SF  (0.5f)  // float32_t    vbc_freq_sf,    Unit: NUM,
#define M4_VBC_ANGLE_SF (0.5f)  // float32_t    vbc_angle_sf,   Unit: NUM,
#define M4_VBC_KI_SF    (0.5f)  // float32_t    vbc_Ki_sf,  Unit: NUM,
#define M4_FILTERISFC   (100.0f)    // float32_t    filterIsFc,     Unit: Hz,
#define M4_FILTERVSFC   (100.0f)    // float32_t    filterVsFc,     Unit: Hz,
#define M4_V_DECOUP_SF  (0.075f)    // float32_t    V_decoup_sf,    Unit: NUM,
#define M4_CONTROLTYPES 0   // CONTROL_TYPES_t  controlTypes,   Unit: NUM,
#define M4_CONTROLFUNCS 0   // CONTROL_FUNCS_t  controlFuncs,   Unit: NUM,
#define M4_FAULTMTRMASK 0   // FAULT_MTR_REG_t  faultMtrMask,   Unit: NUM,
#define M4_OVERTEMPERATUREMOTOR 104 // int16_t  overTemperatureMotor,   Unit: 1ms,
#define M4_OVERTEMPERATUREMODULE    94  // int16_t  overTemperatureModule,  Unit: 1ms,
#define M4_RSONLINEWAITTIMESET  (30000U)    // uint16_t RsOnlineWaitTimeSet,    Unit: 1ms,
#define M4_RSONLINEWORKTIMESET  (24000U)    // uint16_t RsOnlineWorkTimeSet,    Unit: 1ms,
#define M4_SAMPLETRIGDELAY  (30U)   // uint16_t sampleTrigDelay,    Unit: us,
#define M4_DSSCMINDURATION  (30U)   // uint16_t dsscMinDuration,    Unit: us,
#define M4_DSSCSAMPLEDELAY  (30U)   // uint16_t dsscSampleDelay,    Unit: us,
#define M4_STARTUPTIMEDELAY (30000U)    // uint16_t startupTimeDelay,   Unit: 1ms,
#define M4_STOPWAITTIMESET  (0U)    // uint16_t stopWaitTimeSet,    Unit: 1ms,
#define M4_RESTARTWAITTIMESET   (0U)    // uint16_t restartWaitTimeSet,     Unit: 1ms,
#define M4_RESTARTTIMESSET  (0U)    // uint16_t restartTimesSet,    Unit: 1ms,
#define M4_OOBCHECKTIMESET  (0U)    // uint16_t oobCheckTimeSet,    Unit: 1ms,
#define M4_OOBSPEEDSET_RPM  (0U)    // uint16_t oobSpeedSet_rpm,    Unit: 1rpm,
#define M4_OOBCALCCOEFSET   (0U)    // uint16_t oobCalcCoefSet,     Unit: 1,
#define M4_OOBACCELSET_RPMPS    (0U)    // uint16_t oobAccelSet_rpmps,  Unit: 1rpm/s,
#define M4_WEIGHTCHECKTIMESET   (0U)    // uint16_t weightCheckTimeSet,     Unit: 1ms,
#define M4_WEIGHTSPEEDSET_RPM   (0U)    // uint16_t weightSpeedSet_rpm,     Unit: 1rpm,
#define M4_WEIGHTCALCCOEFSET    (140U)  // uint16_t weightCalcCoefSet,  Unit: 1,
#define M4_WEIGHTACCELSET_RPMPS (13U)   // uint16_t weightAccelSet_rpmps,   Unit: 1rpm/s,
#define M4_RESERVE_PRMS1    (0xFFFFU)   // uint16_t reserve_Prms1,  Unit: ,
#define M4_RESERVE_PRMS2    (0xFFFFU)   // uint16_t reserve_Prms2,  Unit: ,
// End of motor M4 control parameters

//-----------------------------------------------------------------------------------
// End of the motor electrical and control parameters definitions of the different motor models
//----------------------------------------------------------------------------------

// the modules
#if defined(_F280013x)
#define MOTOR_PRMS_LIST_MAX     MOTOR_PRMS_LIST_MAX_013X

#define BOOTAPI_SECTOR_START    Bzero_Sector0_start       // Sector0~3
#define BOOTAPI_SECTOR_NUM      4

#define DATAS1_SECTOR_START     Bzero_Sector4_start
#define DATAS1_SECTOR_NUM       2                         // Sector4~5
#define DATAS1_SECTOR_LENGTH    0x400U                    // Sector4~5, 32bit

#define INDEX1_START_OFFSET     0x0010                    //
#define INDEX1_START_ADDR       0x081010                  // Sector4,
#define DATAS1_START_ADDR       0x081040                  // Sector4~5

#define DATAS2_SECTOR_START     Bzero_Sector62_start
#define DATAS2_SECTOR_NUM       2                         // Sector62~63
#define DATAS2_SECTOR_LENGTH    0x400U                    // Sector62~63, 32bit

#define INDEX2_START_ADDR       0x08F800                  // Sector62
#define DATAS2_START_ADDR       0x08F820                  // Sector62~63

#define CODES1A_SECTOR_START    Bzero_Sector6_start
#define CODES1A_SECTOR_NUM      34                        // Sector6~39

#define CODES2A_SECTOR_START    Bzero_Sector40_start
#define CODES2A_SECTOR_NUM      12                        // Sector40~51

#define CODES2B_SECTOR_START    Bzero_Sector52_start
#define CODES2B_SECTOR_NUM      10                        // Sector52~61

#define CODES3A_SECTOR_START    Bzero_Sector64_start
#define CODES3A_SECTOR_NUM      16                        // Sector64~79

#define CODES3B_SECTOR_START    Bzero_Sector80_start
#define CODES3B_SECTOR_NUM      16                        // Sector80~95

#define CODES4A_SECTOR_START    Bzero_Sector96_start
#define CODES4A_SECTOR_NUM      16                        // Sector96~111

#define CODES4B_SECTOR_START    Bzero_Sector112_start
#define CODES4B_SECTOR_NUM      16                        // Sector112~127

#elif defined(_F28003x)
#define MOTOR_PRMS_LIST_MAX    MOTOR_PRMS_LIST_MAX_03X

#define DATAS1_SECTOR_START     Bzero_Sector15_start
#define DATAS1_SECTOR_NUM       1                         // Sector15
#define DATAS1_SECTOR_LENGTH    0x0800                    // Sector15, 32bit

#define INDEX1_START_OFFSET     0x0010                    //
#define INDEX1_START_ADDR       0x08F010                  // Sector15
#define DATAS1_START_ADDR       0x08F040                  // Sector15

#define DATAS2_SECTOR_START     Bone_Sector7_start
#define DATAS2_SECTOR_NUM       1                         // Sector7

#define INDEX2_START_ADDR       0x097000
#define DATAS2_START_ADDR       0x097020                  // Sector7

#define CODESFM_SECTOR_START    Bzero_Sector13_start
#define CODESFM_SECTOR_NUM      2                         // Sector13~14

#define CODES1A_SECTOR_START    Bzero_Sector9_start
#define CODES1A_SECTOR_NUM      4                         // Sector9~12

#define CODES2A_SECTOR_START    Bzero_Sector32_start
#define CODES2A_SECTOR_NUM      16                        // Sector32~47

#define CODES2B_SECTOR_START    Bone_Sector5_start
#define CODES2B_SECTOR_NUM      5                         // Sector5~6

#define CODES3A_SECTOR_START    Bzero_Sector0_start
#define CODES3A_SECTOR_NUM      4                        // Sector0~3

#define CODES3B_SECTOR_START    Bzero_Sector4_start
#define CODES3B_SECTOR_NUM      4                        // Sector4~7

#define CODES4A_SECTOR_START    Bone_Sector8_start
#define CODES4A_SECTOR_NUM      4                         // Sector8~11

#define CODES4B_SECTOR_START    Bone_Sector12_start
#define CODES4B_SECTOR_NUM      4                        // Sector12~15

#define BOOTAPI_SECTOR_START    Bzero_Sector8_start       // Sector8
#define BOOTAPI_SECTOR_NUM      1
#else   // !_F280013x
#error The device is not supported in this project
#endif  //
// the globals

#if defined(_PRMS_UPDATE)
extern const uint16_t prmsIndexList[32];
extern const CTRL_Params_t ctrlParamsList[MOTOR_MAX_NUM];
#endif  // _PRMS_UPDATE

// the functions
#if defined(_PRMS_UPDATE)
extern bool updateControlPrms(MOTORSETS_Handle motorSetHandle);
#endif  // _PRMS_UPDATE

#if defined(_LFU_ENABLE)
extern void readDataPrmsIndex(MOTORSETS_Handle motorSetHandle);
extern void storeControlPrms(MOTORSETS_Handle motorSetHandle);
#endif  //_LFU_ENABLE

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of CONTROL_PARAMETERS_H_ defines

//
// End of File
//
