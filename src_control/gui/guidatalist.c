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
//! \file   /solutions/tida_010265_wminv/common/source/guiprotocol.c
//!
//! \brief  This project is used to implement GUI for motor control
//!         F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


//
// Includes
//
#include<stdlib.h>

#include "user.h"
#include "hal.h"

#include "motor_common.h"
#include "motor1_drive.h"

#include "guidatalist.h"
#include "guicontrol.h"

#include "controlparameters.h"
#include "systemcontrol.h"

//
// Defines
//

//
// Globals
//


//
// GUI_Data_Type + Index (Copy from LCINV*.xlsx)
//
#if defined(GUI_SCI_EN)
#pragma DATA_ALIGN(guiComVarsList, 4)

const GUI_ComVarsList_t guiComVarsList[GUI_DATA_LIST_LENGTH] = { \
   // The parameters list must be the same as the list on the GUI side.
   \
// Write/Read these variables without RX&TX Index
// RX&TX Index,  Data type&spec,   Data Scale,     Data address,
   {0U,    (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorCtrlVars_M1.torqueSet_Nm},
   {1U,    (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorCtrlVars_M1.powerSet_W},
   {2U,    (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorCtrlVars_M1.IqSet_A},
   {3U,    (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorCtrlVars_M1.speedSet_Hz},
   {4U,    (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorCtrlVars_M1.accelerationSet_Hzps},
   {5U,    (GUI_FLOAT + GUI_WR),   1.0f,   1.0f,   &motorCtrlVars_M1.speedSet_rpm},
   {6U,    (GUI_FLOAT + GUI_WR),   1.0f,   1.0f,   &motorCtrlVars_M1.accelerationSet_rpmps},
   {7U,    (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorCtrlVars_M1.accelerationSetTime},
   {8U,    (GUI_UINT + GUI_WO),    1.0f,   1.0f,   &motorCtrlVars_M1.controlCmdRecv},
   {9U,    (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.motorModel},
   {10U,   (GUI_UINT + GUI_WR),    0.1f,   10.0f,  &motorSetVars_M1.overCurrentTimesSet},
   {11U,   (GUI_UINT + GUI_WR),    0.1f,   10.0f,  &motorSetVars_M1.overLoadTimeSet},
   {12U,   (GUI_UINT + GUI_WR),    0.1f,   10.0f,  &motorSetVars_M1.motorStallTimeSet},
   {13U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.voltageFaultTimeSet},
   {14U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.startupFailTimeSet},
   {15U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.overSpeedTimeSet},
   {16U,   (GUI_UINT + GUI_WR),    0.1f,   10.0f,  &motorSetVars_M1.unbalanceTimeSet},
   {17U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.lostPhaseTimeSet},
   {18U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.flyingStartTimeDelay},
   {19U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.alignTimeDelay},
   {20U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.forceRunTimeDelay},
   {21U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.controlTicksPWM},
   {22U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.speedTicksControl},
   {23U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.motor_type},
   {24U,   (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.numPolePairs},
   {25U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Rs_Ohm},
   {26U,   (GUI_FLOAT + GUI_WR),   500000.0f,  0.000002f,  &motorSetVars_M1.Ls_d_H},
   {27U,   (GUI_FLOAT + GUI_WR),   500000.0f,  0.000002f,  &motorSetVars_M1.Ls_q_H},
   {28U,   (GUI_FLOAT + GUI_WR),   2000.0f,    0.0005f,    &motorSetVars_M1.flux_VpHz},
   {29U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Rr_Ohm},
   {30U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.magneticCurrent_A},
   {31U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.maxCurrentResEst_A},
   {32U,   (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.maxCurrentIndEst_A},
   {33U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.fluxExcFreq_Hz},
   {34U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.fluxFilterCoef},
   {35U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.speedFilterCoef},
   {36U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.bemfFilterCoef},
   {37U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.speedPole_rps},
   {38U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.directionPole_rps},
   {39U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.fluxPole_rps},
   {40U,   (GUI_FLOAT + GUI_WR),   100000.0f,  0.00001f,   &motorSetVars_M1.RsOnLine_Rdelta_Ohm},
   {41U,   (GUI_FLOAT + GUI_WR),   100000.0f,  0.00001f,   &motorSetVars_M1.RsOnLine_Adelta_rad},
   {42U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.voltageFilter_Hz},
   {43U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.voltageScale_V},
   {44U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.currentScale_A},
   {45U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.pwmControl_kHz},
   {46U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.RsOnLineCurrent_A},
   {47U,   (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.angleESTDelayed_sf},
   {48U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.maxFrequency_Hz},
   {49U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.maxCurrentSet_A},
   {50U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.maxVoltage_V},
   {51U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.maxPeakCurrent_A},
   {52U,   (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.maxVsMag_pu},
   {53U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.Ls_d_Icomp_coef},
   {54U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.Ls_q_Icomp_coef},
   {55U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.Ls_min_H},
   {56U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.powerCtrlSet_W},
   {57U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.overCurrent_A},
   {58U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.overLoadSet_W},
   {59U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.lostPhaseSet_A},
   {60U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.unbalanceRatioSet},
   {61U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.stallCurrentSet_A},
   {62U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.speedFailMaxSet_Hz},
   {63U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.speedFailMinSet_Hz},
   {64U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.IsFailedChekSet_A},
   {65U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.toqueFailMinSet_Nm},
   {66U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.overVoltageFault_V},
   {67U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.overVoltageNorm_V},
   {68U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.underVoltageFault_V},
   {69U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.underVoltageNorm_V},
   {70U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.fluxCurrent_A},
   {71U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.alignCurrent_A},
   {72U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.startCurrent_A},
   {73U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.brakingCurrent_A},
   {74U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.accelStart_Hzps},
   {75U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.accelStop_Hzps},
   {76U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.accelRun_Hzps},
   {77U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.speedFlyingStart_Hz},
   {78U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.speedForce_Hz},
   {79U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.speedStart_Hz},
   {80U,   (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.VsRef_pu},
   {81U,   (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.Kp_fwc},
   {82U,   (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.Ki_fwc},
   {83U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.angleFWCMax_rad},
   {84U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.Gain_speed_high_Hz},
   {85U,   (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.Gain_speed_low_Hz},
   {86U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Kp_spd_high_sf},
   {87U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Ki_spd_high_sf},
   {88U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Kp_spd_low_sf},
   {89U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Ki_spd_low_sf},
   {90U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Kp_Iq_sf},
   {91U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Ki_Iq_sf},
   {92U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Kp_Id_sf},
   {93U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Ki_Id_sf},
   {94U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Kp_pow_sf},
   {95U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.Ki_pow_sf},
   {96U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Kp_spd_start_sf},
   {97U,   (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.Ki_spd_start_sf},
   {98U,   (GUI_FLOAT + GUI_WR),   1.0f,   1.0f,   &motorSetVars_M1.esmo_FAST_fsw_Hz},
   {99U,   (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.esmo_KslideMax},
   {100U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.esmo_KslideMin},
   {101U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.esmo_LpfFc_Hz},
   {102U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.esmo_filterFc_sf},
   {103U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.esmo_E0},
   {104U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.esmo_PLL_KpMax},
   {105U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.esmo_PLL_KpMin},
   {106U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.esmo_PLL_KpSF},
   {107U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.esmo_PLL_Ki},
   {108U,  (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.anglePLLDelayed_sf},
   {109U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.hfi_Kspd},
   {110U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_excMag_coarse_V},
   {111U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_excMag_fine_V},
   {112U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.hfi_waitTime_coarse_sec},
   {113U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.hfi_waitTime_fine_sec},
   {114U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_excFreq_Hz},
   {115U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_LpfFcSpd_Hz},
   {116U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_HpfFcIq_Hz},
   {117U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.hfi_IqMaxHfi_A},
   {118U,  (GUI_FLOAT + GUI_WR),   100.0f, 0.01f,  &motorSetVars_M1.hfi_IqMaxEst_A},
   {119U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.hfi_IqSlope_A},
   {120U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_freqLow_Hz},
   {121U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.hfi_freqHigh_Hz},
   {122U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.pir_spd_f0},
   {123U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.pir_spd_fc},
   {124U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.pir_spd_k},
   {125U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.pir_Idq_fc},
   {126U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.pir_Id_k},
   {127U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.pir_Iq_k},
   {128U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.vbc_angle_delta},
   {129U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.vbc_Iq_amp},
   {130U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.vbc_Iq_sf},
   {131U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.vbc_freq_sf},
   {132U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.vbc_angle_sf},
   {133U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.vbc_Ki_sf},
   {134U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.filterIsFc},
   {135U,  (GUI_FLOAT + GUI_WR),   10.0f,  0.1f,   &motorSetVars_M1.filterVsFc},
   {136U,  (GUI_FLOAT + GUI_WR),   10000.0f,   0.0001f,    &motorSetVars_M1.V_decoup_sf},
   {137U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.IdInj_A},
   {138U,  (GUI_FLOAT + GUI_WR),   1000.0f,    0.001f, &motorSetVars_M1.IqInj_A},
   {139U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.controlTypes},
   {140U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.controlFuncs},
   {141U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.faultMtrMask.all},
   {142U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.sampleTrigDelay},
   {143U,  (GUI_INT + GUI_WR), 1.0f,   1.0f,   &motorSetVars_M1.overTemperatureMotor},
   {144U,  (GUI_INT + GUI_WR), 1.0f,   1.0f,   &motorSetVars_M1.overTemperatureModule},
   {145U,  (GUI_UINT + GUI_WR),    200.0f, 0.005f, &motorSetVars_M1.RsOnlineWaitTimeSet},
   {146U,  (GUI_UINT + GUI_WR),    200.0f, 0.005f, &motorSetVars_M1.RsOnlineWorkTimeSet},
   {147U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.startupTimeDelay},
   {148U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.stopWaitTimeSet},
   {149U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.restartWaitTimeSet},
   {150U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &motorSetVars_M1.restartTimesSet},
   {151U,  (GUI_UINT + GUI_WR),    0.1f,   10.0f,  &sysWashVars.oobCheckTimeSet},
   {152U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &sysWashVars.oobSpeedSet_rpm},
   {153U,  (GUI_UINT + GUI_WR),    10.0f,  0.1f,   &sysWashVars.oobCalcCoefSet},
   {154U,  (GUI_UINT + GUI_WR),    10.0f,  0.1f,   &sysWashVars.oobAccelSet_rpmps},
   {155U,  (GUI_UINT + GUI_WR),    10.0f,  0.1f,   &sysWashVars.oobCalcBase},
   {156U,  (GUI_UINT + GUI_WR),    0.1f,   10.0f,  &sysWashVars.weightCheckTimeSet},
   {157U,  (GUI_UINT + GUI_WR),    1.0f,   1.0f,   &sysWashVars.weightSpeedSet_rpm},
   {158U,  (GUI_UINT + GUI_WR),    10.0f,  0.1f,   &sysWashVars.weightCalcCoefSet},
   {159U,  (GUI_UINT + GUI_WR),    10.0f,  0.1f,   &sysWashVars.weightAccelSet_rpmps},
   {160U,  (GUI_UINT + GUI_WR),    10.0f,  0.1f,   &sysWashVars.weightCalcBase},
   {161U,  (GUI_UINT + GUI_RO),    1.0f,   1.0f,   &motorCtrlVars_M1.motorCtrlStates},
   {162U,  (GUI_UINT + GUI_RO),    1.0f,   1.0f,   &motorCtrlVars_M1.faultMotor.all},
   {163U,  (GUI_FLOAT + GUI_RO),   10.0f,  0.1f,   &motorVars_M1.speedPLL_Hz},
   {164U,  (GUI_FLOAT + GUI_RO),   10.0f,  0.1f,   &motorVars_M1.speedEST_Hz},
   {165U,  (GUI_FLOAT + GUI_RO),   10.0f,  0.1f,   &motorVars_M1.speed_Hz},
   {166U,  (GUI_FLOAT + GUI_RO),   5000.0f,    0.0002f,    &motorVars_M1.angleFOC_rad},
   {167U,  (GUI_FLOAT + GUI_RO),   5000.0f,    0.0002f,    &motorVars_M1.angleEST_rad},
   {168U,  (GUI_FLOAT + GUI_RO),   5000.0f,    0.0002f,    &motorVars_M1.anglePLL_rad},
   {169U,  (GUI_FLOAT + GUI_RO),   1.0f,   1.0f,   &motorVars_M1.speed_rpm},
   {170U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.adcData.I_A.value[0]},
   {171U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.adcData.I_A.value[1]},
   {172U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.adcData.I_A.value[2]},
   {173U,  (GUI_FLOAT + GUI_RO),   50.0f,  0.02f,  &motorVars_M1.adcData.V_V.value[0]},
   {174U,  (GUI_FLOAT + GUI_RO),   50.0f,  0.02f,  &motorVars_M1.adcData.V_V.value[1]},
   {175U,  (GUI_FLOAT + GUI_RO),   50.0f,  0.02f,  &motorVars_M1.adcData.V_V.value[2]},
   {176U,  (GUI_FLOAT + GUI_RO),   50.0f,  0.02f,  &motorVars_M1.adcData.VdcBus_V},
   {177U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.Idq_in_A.value[0]},
   {178U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.Idq_in_A.value[1]},
   {179U,  (GUI_FLOAT + GUI_RO),   50.0f,  0.02f,  &motorVars_M1.Vdq_out_V.value[0]},
   {180U,  (GUI_FLOAT + GUI_RO),   50.0f,  0.02f,  &motorVars_M1.Vdq_out_V.value[1]},
   {181U,  (GUI_FLOAT + GUI_RO),   1000.0f,    0.001f, &motorVars_M1.Irms_A[0]},
   {182U,  (GUI_FLOAT + GUI_RO),   1000.0f,    0.001f, &motorVars_M1.Irms_A[1]},
   {183U,  (GUI_FLOAT + GUI_RO),   1000.0f,    0.001f, &motorVars_M1.Irms_A[2]},
   {184U,  (GUI_FLOAT + GUI_RO),   1000.0f,    0.001f, &motorVars_M1.torque_Nm},
   {185U,  (GUI_FLOAT + GUI_RO),   4.0f,   0.25f,  &motorVars_M1.powerActive_W},
   {186U,  (GUI_FLOAT + GUI_RO),   2000.0f,    0.0005f,    &motorVars_M1.RsOnLine_Ohm},
   {187U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.IdqRef_A.value[0]},
   {188U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &motorVars_M1.IdqRef_A.value[1]},
   {189U,  (GUI_UINT + GUI_RO),    1.0f,   1.0f,   &motorCtrlVars_M1.temperatureModule},
   {190U,  (GUI_UINT + GUI_RO),    1.0f,   1.0f,   &motorCtrlVars_M1.temperatureMotor},
   {191U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &sysWashVars.oobCalcValue},
   {192U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &sysWashVars.weightCalcValue},
   {193U,  (GUI_FLOAT + GUI_RO),   100.0f, 0.01f,  &sysWashVars.torqueCurrentSum_A},
   {194U,  (GUI_FLOAT + GUI_RO),   10.0f,  0.1f,   &sysWashVars.activePowerSum_W},
   {195U,  (GUI_FLOAT + GUI_RO),   10.0f,  0.1f,   &sysWashVars.speedRipple_rpm}
   // Write/Read these variables without RX&TX Index, copy from xslx file
   \
};
#endif  // GUI_SCI_EN
//
// End of File
//
