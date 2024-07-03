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
//! \file   /solutions/universal_motorcontrol_lab/common/include/user_mtr1.h
//! \brief  Contains the user related definitions
//!         This file is used for each device includes F28002x, F28003x, F280013x,
//!         F280015x, and the other newer C2000 MCUs.
//!
//------------------------------------------------------------------------------


#ifndef USER_MTR1_H
#define USER_MTR1_H


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
//! \defgroup USER USER_MTR1
//! @{
//
//*****************************************************************************

//
// the includes
// platforms
#include "hal.h"

// modules
#include "userParams.h"

#include "user_common.h"

// *****************************************************************************
// the defines

#if defined(WMINVBRD_REV1P0)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (220.0f)

//! \brief Defines the maximum voltage at the AD converter
//  Full scale voltage of AD converter, not the current voltage
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (404.1292683f)

//! \brief Defines the analog voltage filter pole location, Hz
//!
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (416.3602877f)

// High Voltage motor control kit
#if defined(MOTOR1_DCLINKSS)    // Single shunt
#if !defined(SHUNT_R100)
//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A    (15.84f)

// Single shunt
//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF             (1.0f)

//! \brief ADC current offsets for dc-link
#define USER_M1_IDC_OFFSET_A                (7.986f)
#elif defined(SHUNT_R100)
//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A    (7.986f)

// Single shunt
//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF          (1.0f)

//! \brief ADC current offsets for dc-link
#define USER_M1_IDC_OFFSET_A             (3.993f)
#endif  // SHUNT_R100

//! \brief ADC current offsets for dc-link
#define USER_M1_IDC_OFFSET_AD           (2048.0f)

#define USER_M1_IDC_OFFSET_AD_MAX       (USER_M1_IDC_OFFSET_AD + 100.0f)
#define USER_M1_IDC_OFFSET_AD_MIN       (USER_M1_IDC_OFFSET_AD - 100.0f)

#if defined(SAFETY_ENABLE)
#define ADC_IDC_OFFSET_SENSE    ((uint16_t)USER_M1_IDC_OFFSET_AD)

#define ADC_IDC_OFFSET_DELTA    (uint16_t)(280U)

#define ADC_IDC_OFFSET_MAX      (uint16_t)(ADC_IDC_OFFSET_SENSE + ADC_IDC_OFFSET_DELTA)
#define ADC_IDC_OFFSET_MIN      (uint16_t)(ADC_IDC_OFFSET_SENSE - ADC_IDC_OFFSET_DELTA)
#endif  // SAFETY_ENABLE

// Tdt  =  2.20us (Dead-time between top and bottom switch)
// Tpd  =  0.75us (Gate driver propagation delay)
// Tr   =  0.35us (Rise time of amplifier including power switches turn on time)
// Ts   =  1.0us  (Settling time of amplifier)
// Ts&h =  180ns  (ADC sample&holder = (16 + 2) SYSCLK)
// T_MinAVDuration = Tdt+Tpd+Tr+Ts+Ts&h
//                 = 2200+750+350+1000+180 = 4480ns => 450 SYSCLK cycles
// T_SampleDelay   = Tdt+Tpd+Tr+Ts
//                 = 2200+750+350+1000 = 4300ns => 430 SYSCLK cycles
//! \brief Defines the minimum duration, Clock Cycle
//!
#define USER_M1_DCLINKSS_MIN_DURATION       (450U)

//! \brief Defines the sample delay, Clock Cycle
//!
#define USER_M1_DCLINKSS_SAMPLE_DELAY       (430U)

#else  // !(MOTOR1_DCLINKSS)        // 2/3 shunt
#if !defined(SHUNT_R100)
//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A    (15.972f)

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF             (-1.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_A                 (7.9193938f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#define USER_M1_IB_OFFSET_A                 (7.9112699f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#define USER_M1_IC_OFFSET_A                 (7.9212855f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A

#elif defined(SHUNT_R100)
//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A    (7.986f)

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF             (-1.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_A                 (7.9193938f * 0.5f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#define USER_M1_IB_OFFSET_A                 (7.9112699f * 0.5f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#define USER_M1_IC_OFFSET_A                 (7.9212855f * 0.5f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#endif  // SHUNT_R100

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD        (2049.367f)
#define USER_M1_IB_OFFSET_AD        (2042.771f)
#define USER_M1_IC_OFFSET_AD        (2054.451f)

//! \brief ADC current offset for CMPSS
#define USER_M1_IS_OFFSET_CMPSS     (uint16_t)((USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD) / 3.0f)

#if defined(SAFETY_ENABLE)
#define ADC_IU_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)
#define ADC_IV_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)
#define ADC_IW_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)

#define ADC_IS_OFFSET_DELTA     (uint16_t)(480U)

#define ADC_IU_OFFSET_MAX       (uint16_t)(ADC_IU_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)
#define ADC_IV_OFFSET_MAX       (uint16_t)(ADC_IV_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)
#define ADC_IW_OFFSET_MAX       (uint16_t)(ADC_IW_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)

#define ADC_IU_OFFSET_MIN       (uint16_t)(ADC_IU_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#define ADC_IV_OFFSET_MIN       (uint16_t)(ADC_IV_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#define ADC_IW_OFFSET_MIN       (uint16_t)(ADC_IW_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#endif  // SAFETY_ENABLE

#endif   // !(MOTOR1_DCLINKSS)      // 2/3 shunt

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF        (0.503290117f)
#define USER_M1_VB_OFFSET_SF        (0.500881076f)
#define USER_M1_VC_OFFSET_SF        (0.497107089f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (380.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (350.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (100.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (120.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (800.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_M1_NUM_PWM_TICKS_PER_ISR_TICK          (1)

//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
#define USER_M1_PWM_FREQ_kHz        (15.0f)
#define USER_M1_PWM_TBPRD_NUM       (uint16_t)(USER_SYSTEM_FREQ_MHz * 1000.0f / USER_M1_PWM_FREQ_kHz / 2.0f)

// end of WMINVBRD_REV1P0
//------------------------------------------------------------------------------
#elif defined(HVMTRPFC_REV1P1)

// Bypass the 820k resistor for low voltage motor on this kit
//#define LV_JUMPER_EN            // Bypass the 820k resistor


#ifndef LV_JUMPER_EN
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (220.0f)

//! \brief Defines the maximum voltage at the AD converter
//  Full scale voltage of AD converter, not the current voltage
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (409.90f)

//! \brief Defines the analog voltage filter pole location, Hz
//!
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (375.55f)

#else   // Populate jumpers on J1/J2/J3/J4 for short R20/R23/R26/R37 for low voltage motor
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (24.0f)

//! \brief Defines the maximum voltage at the AD converter
//  Full scale voltage of AD converter, not the current voltage
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (112.21f)

//! \brief Defines the analog voltage filter pole location, Hz
//!
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (381.15f)
#endif

// High Voltage motor control kit
#if defined(MOTOR1_DCLINKSS)    // Single shunt
//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (19.995f)

// Single shunt
//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (-1.0f)

//! \brief ADC current offsets for dc-link
#define USER_M1_IDC_OFFSET_A            (-9.997f)

//! \brief ADC current offsets for dc-link
#define USER_M1_IDC_OFFSET_AD           (2048.0f)

#define USER_M1_IDC_OFFSET_AD_MAX       (USER_M1_IDC_OFFSET_AD + 100.0f)
#define USER_M1_IDC_OFFSET_AD_MIN       (USER_M1_IDC_OFFSET_AD - 100.0f)


#if defined(SAFETY_ENABLE)
#define ADC_IDC_OFFSET_SENSE    ((uint16_t)USER_M1_IDC_OFFSET_AD)

#define ADC_IDC_OFFSET_DELTA    (uint16_t)(280U)

#define ADC_IDC_OFFSET_MAX      (uint16_t)(ADC_IDC_OFFSET_SENSE + ADC_IDC_OFFSET_DELTA)
#define ADC_IDC_OFFSET_MIN      (uint16_t)(ADC_IDC_OFFSET_SENSE - ADC_IDC_OFFSET_DELTA)
#endif  // SAFETY_ENABLE

// Tdt  =  2.20us (Dead-time between top and bottom switch)
// Tpd  =  0.75us (Gate driver propagation delay)
// Tr   =  0.35us (Rise time of amplifier including power switches turn on time)
// Ts   =  1.0us  (Settling time of amplifier)
// Ts&h =  180ns  (ADC sample&holder = (16 + 2) SYSCLK)
// T_MinAVDuration = Tdt+Tpd+Tr+Ts+Ts&h
//                 = 2200+750+350+1000+180 = 4480ns => 450 SYSCLK cycles
// T_SampleDelay   = Tdt+Tpd+Tr+Ts
//                 = 2200+750+350+1000 = 4300ns => 430 SYSCLK cycles
//! \brief Defines the minimum duration, Clock Cycle
//!
#define USER_M1_DCLINKSS_MIN_DURATION   (450U)

//! \brief Defines the sample delay, Clock Cycle
//!
#define USER_M1_DCLINKSS_SAMPLE_DELAY   (430U)

#else  // !(MOTOR1_DCLINKSS)        // 2/3 shunt

//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (19.995f)

// Three shunt
//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (1.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_A         (9.9883938f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#define USER_M1_IB_OFFSET_A         (9.982699f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A
#define USER_M1_IC_OFFSET_A         (9.9862855f)   // ~0.5 * USER_M1_ADC_FULL_SCALE_CURRENT_A

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD        (2049.367f)
#define USER_M1_IB_OFFSET_AD        (2042.771f)
#define USER_M1_IC_OFFSET_AD        (2054.451f)

//! \brief ADC current offset for CMPSS
#define USER_M1_IS_OFFSET_CMPSS     (uint16_t)((USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD) / 3.0f)

#if defined(SAFETY_ENABLE)
#define ADC_IU_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)
#define ADC_IV_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)
#define ADC_IW_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)

#define ADC_IS_OFFSET_DELTA     (uint16_t)(480U)

#define ADC_IU_OFFSET_MAX       (uint16_t)(ADC_IU_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)
#define ADC_IV_OFFSET_MAX       (uint16_t)(ADC_IV_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)
#define ADC_IW_OFFSET_MAX       (uint16_t)(ADC_IW_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)

#define ADC_IU_OFFSET_MIN       (uint16_t)(ADC_IU_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#define ADC_IV_OFFSET_MIN       (uint16_t)(ADC_IV_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#define ADC_IW_OFFSET_MIN       (uint16_t)(ADC_IW_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#endif  // SAFETY_ENABLE

#endif   // !(MOTOR1_DCLINKSS)      // 2/3 shunt

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF        (0.503290117f)
#define USER_M1_VB_OFFSET_SF        (0.500881076f)
#define USER_M1_VC_OFFSET_SF        (0.497107089f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (380.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (350.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (12.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (15.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (250.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_M1_NUM_PWM_TICKS_PER_ISR_TICK          (1)

//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
#define USER_M1_PWM_FREQ_kHz        (15.0f)
#define USER_M1_PWM_TBPRD_NUM       (uint16_t)(USER_SYSTEM_FREQ_MHz * 1000.0f / USER_M1_PWM_FREQ_kHz / 2.0f)

// end of HVMTRPFC_REV1P1

#elif defined(BSXL8323RH_REVB)       // LaunchPad-F280025
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (24.0f)

#if !defined(HC_RC5G10_EN)
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (57.52845691f)     //82k/4.99k/47nF

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (680.4839141f)     // 47nF

//! \brief Defines the maximum current at the AD converter
//! Place a 47k(ohm) resistor (R22) pull-down on MODE pin
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (47.14285714f)     // R007/gain=10
#else   // HC_RC5G10_EN
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (40.33406814f)     // 56k/4.99k/33nF

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (1052.631234f)     // 33nF

//! \brief Defines the maximum current at the AD converter
//! Place a 47k(ohm) resistor (R22) pull-down on MODE pin
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (66.0f)            // R005/gain=10
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (132.0f)           // R005/gain=5
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (99.0f)            // R(0.01/3)/gain=10
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (198.0f)           // R(0.01/3)/gain=5
#define USER_M1_ADC_FULL_SCALE_CURRENT_A           (264.0f)           // R(0.01/4)/gain=5
#endif  // HC_RC5G10_EN
//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (1.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2045.40015f)
#define USER_M1_IB_OFFSET_AD    (2012.86694f)
#define USER_M1_IC_OFFSET_AD    (2031.59741f)

//! \brief ADC current offset for CMPSS
#define USER_M1_IS_OFFSET_CMPSS     (uint16_t)((USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD) / 3.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.507042527f)
#define USER_M1_VB_OFFSET_SF    (0.505379438f)
#define USER_M1_VC_OFFSET_SF    (0.50771445f)

#if defined(SAFETY_ENABLE)
#define ADC_IU_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)
#define ADC_IV_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)
#define ADC_IW_OFFSET_SENSE     ((uint16_t)USER_M1_IA_OFFSET_AD)

#define ADC_IS_OFFSET_DELTA     (uint16_t)(480U)

#define ADC_IU_OFFSET_MAX       (uint16_t)(ADC_IU_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)
#define ADC_IV_OFFSET_MAX       (uint16_t)(ADC_IV_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)
#define ADC_IW_OFFSET_MAX       (uint16_t)(ADC_IW_OFFSET_SENSE + ADC_IS_OFFSET_DELTA)

#define ADC_IU_OFFSET_MIN       (uint16_t)(ADC_IU_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#define ADC_IV_OFFSET_MIN       (uint16_t)(ADC_IV_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#define ADC_IW_OFFSET_MIN       (uint16_t)(ADC_IW_OFFSET_SENSE - ADC_IS_OFFSET_DELTA)
#endif  // SAFETY_ENABLE

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (54.5f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (52.5f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (8.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (10.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.01f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (90.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (1800.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_M1_NUM_PWM_TICKS_PER_ISR_TICK      (1)

//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
#if !defined(_HSWFREQ_EN)
#define USER_M1_PWM_FREQ_kHz        (15.0f)
#else   // _HSWFREQ_EN
#define USER_M1_PWM_FREQ_kHz        (20.0f)     // for high speed motor (2500kHz)
#endif  // _HSWFREQ_EN

#define USER_M1_PWM_TBPRD_NUM       (uint16_t)(USER_SYSTEM_FREQ_MHz * 1000.0f / USER_M1_PWM_FREQ_kHz / 2.0f)

// end of BSXL8323RH_REVB

//------------------------------------------------------------------------------
#elif defined(DRV8329AEVM_REVA)       // LaunchPad-F280025
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (76.04549098f)     // 110k/4.99k

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (333.4164129f)     // 110k/4.99k/100nF

//! \brief Defines the maximum current at the AD converter
//! Set the gain of the CSA, the value should be set accordingly.
//! CSAGAIN=5V/V(J2=GND), CSAGAIN=10V/V(J2=50K), CSAGAIN=20V/V(J2=200K), CSAGAIN=40V/V(J2=DNP)
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A       (82.5f)            // Rs=0.001/gain=40/J2=DNP
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A       (66.0f)            // Rs=0.0025/gain=20/J2=200K
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (33.0f)            // Rs=0.005/gain=40/J2=200K
#define USER_M1_ADC_FULL_SCALE_CURRENT_A       (16.5f)            // Rs=0.005/gain=40/J2=DNP

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the noninverting pin (+) of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is negative that
//!        means the same pin of the shunt resistor is connected to ground and
//!        is also connected to the inverting pin (-) of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (1.0f)

//! \brief ADC current offsets for dc-link
// the dc-link offset current for DRV8329AEVM_REVA
#define USER_M1_IDC_OFFSET_A            (USER_M1_ADC_FULL_SCALE_CURRENT_A / 2.0f)

//! \brief ADC current offsets for dc-link
#define USER_M1_IDC_OFFSET_AD           (516.717f)       //~=4096.0/8.0

//! \brief ADC current offset for CMPSS
#define USER_M1_IDC_OFFSET_CMPSS        (uint16_t)(USER_M1_IDC_OFFSET_AD)

#define USER_M1_IDC_OFFSET_AD_MAX       (USER_M1_IDC_OFFSET_AD + 100.0f)
#define USER_M1_IDC_OFFSET_AD_MIN       (USER_M1_IDC_OFFSET_AD - 100.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.507042527f)
#define USER_M1_VB_OFFSET_SF    (0.505379438f)
#define USER_M1_VC_OFFSET_SF    (0.50771445f)

#if defined(SAFETY_ENABLE)
#define ADC_IDC_OFFSET_SENSE    ((uint16_t)USER_M1_IDC_OFFSET_AD)

#define ADC_IDC_OFFSET_DELTA    (uint16_t)(280U)

#define ADC_IDC_OFFSET_MAX      (uint16_t)(ADC_IDC_OFFSET_SENSE + ADC_IDC_OFFSET_DELTA)
#define ADC_IDC_OFFSET_MIN      (uint16_t)(ADC_IDC_OFFSET_SENSE - ADC_IDC_OFFSET_DELTA)
#endif  // SAFETY_ENABLE

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (54.5f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (52.5f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (8.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (10.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.01f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (90.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (1800.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_M1_NUM_PWM_TICKS_PER_ISR_TICK          (1)

//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
#define USER_M1_PWM_FREQ_kHz        (15.0f)
#define USER_M1_PWM_TBPRD_NUM       (uint16_t)(USER_SYSTEM_FREQ_MHz * 1000.0f / USER_M1_PWM_FREQ_kHz / 2.0f)

// end of DRV8329AEVM_REVA
//-----------------------------------------------------------------------------
#else   // No Board Selection
#error The board parameters are not defined in user_mtr1.h
#endif  // No Board Selection

//------------------------------------------------------------------------------
//! \brief ADC current offsets checking value for A, B, and C phases
// the error threshold to check if the ADC offset of the phase current sensing circuit is correct.
#define USER_M1_IS_OFFSET_AD_DELTA      (150.0f)    // The value is 0.0f~1024.0f

// the high threshold of the ADC offsets checking value for A/B/C phase current
#define USER_M1_IA_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MAX        (USER_M1_IB_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MAX        (USER_M1_IC_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)

// the low threshold of the ADC offsets checking value for A phase current
#define USER_M1_IA_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MIN        (USER_M1_IB_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MIN        (USER_M1_IC_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)

#define USER_M1_IS_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD + (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))
#define USER_M1_IS_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD - (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))

//! \brief ADC voltage offsets for A, B, and C phases
// the error threshold to check if the ADC offset of the phase voltage sensing circuit is correct
#define USER_M1_VA_OFFSET_SF_DELTA      (0.05f)     // The value is 0.0f ~ 0.5f

// the high threshold of the ADC offsets checking value for A/B/C phase voltage
#define USER_M1_VA_OFFSET_SF_MAX        (USER_M1_VA_OFFSET_SF + USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VB_OFFSET_SF_MAX        (USER_M1_VB_OFFSET_SF + USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VC_OFFSET_SF_MAX        (USER_M1_VC_OFFSET_SF + USER_M1_VA_OFFSET_SF_DELTA)

// the low threshold of the ADC offsets checking value for A/B/C phase voltage
#define USER_M1_VA_OFFSET_SF_MIN        (USER_M1_VA_OFFSET_SF - USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VB_OFFSET_SF_MIN        (USER_M1_VB_OFFSET_SF - USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VC_OFFSET_SF_MIN        (USER_M1_VC_OFFSET_SF - USER_M1_VA_OFFSET_SF_DELTA)

#define USER_M1_VS_OFFSET_SF_MAX        (USER_M1_VA_OFFSET_SF + USER_M1_VB_OFFSET_SF + USER_M1_VC_OFFSET_SF + (USER_M1_VA_OFFSET_SF_DELTA * 3.0f))
#define USER_M1_VS_OFFSET_SF_MIN        (USER_M1_VA_OFFSET_SF + USER_M1_VB_OFFSET_SF + USER_M1_VC_OFFSET_SF - (USER_M1_VA_OFFSET_SF_DELTA * 3.0f))

//******************************************************************************
//! \brief Defines the number of ISR clock ticks per current controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_CURRENT_TICK      (1)

//! \brief Defines the number of ISR clock ticks per speed controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK        (10)


//! \brief Defines the number of current sensors
//!
#define USER_M1_NUM_CURRENT_SENSORS                 (3)

//! \brief Defines the number of voltage sensors
//!
#define USER_M1_NUM_VOLTAGE_SENSORS                 (3)

//! \brief Defines the Pulse Width Modulation (PWM) period, usec
//!
#define USER_M1_PWM_PERIOD_usec     (1000.0f / USER_M1_PWM_FREQ_kHz)


//! \brief Defines the Pulse Width Modulation (PWM) period, usec
//!
#define USER_M1_PWM_PERIOD_sec      (1.0f / USER_M1_PWM_FREQ_kHz / 1000.0f)

//! \brief Defines the Interrupt Service Routine (ISR) frequency, Hz
//!
#define USER_M1_ISR_FREQ_Hz         (USER_M1_PWM_FREQ_kHz * 1000.0f / (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief Defines the Interrupt Service Routine (ISR) period, sec
//!
#define USER_M1_ISR_PERIOD_sec      (USER_M1_PWM_PERIOD_sec * (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_M1_ISR_PERIOD_usec     (USER_M1_PWM_PERIOD_usec * (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_M1_VD_SF               (0.95f)


//! \brief Defines the voltage scale factor for the system
//!
#define USER_M1_VOLTAGE_SF          (USER_M1_ADC_FULL_SCALE_VOLTAGE_V / 4096.0f)

//! \brief Defines the current scale factor for the system
//!
#define USER_M1_CURRENT_SF          (USER_M1_ADC_FULL_SCALE_CURRENT_A / 4096.0f)


//! \brief Defines the current scale invert factor for the system
//!
#define USER_M1_CURRENT_INV_SF      (4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A)


//! \brief Defines the analog voltage filter pole location, rad/s
//!
#define USER_M1_VOLTAGE_FILTER_POLE_rps  (MATH_TWO_PI * USER_M1_VOLTAGE_FILTER_POLE_Hz)

//! \brief Defines the maximum Vs magnitude in per units allowed
//! \brief This value sets the maximum magnitude for the output of the Id and
//! \brief Iq PI current controllers. The Id and Iq current controller outputs
//! \brief are Vd and Vq. The relationship between Vs, Vd, and Vq is:
//! \brief Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the Vd value is set
//! \brief equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.
//! \brief so the Vq value is set equal to sqrt(USER_MAX_VS_MAG^2 - Vd^2).
//!
//! \brief Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at
//! \brief SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction
//! \brief is needed for this scenario.
//!
//! \brief Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave
//! \brief with a peak at 100% duty cycle.  Current reconstruction
//! \brief will be needed for this scenario (Lab08).
//!
//! \brief Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal
//! \brief voltage waveform.  Current reconstruction will be needed
//! \brief for this scenario (Lab08).
//!
//! \brief For space vector over-modulation, see lab08 for details on
//! \brief system requirements that will allow the SVM generator to
//! \brief go all the way to trapezoidal.
//!
//#define USER_M1_MAX_VS_MAG_PU             (0.66f)         // 1/2/3-phase (>0.57)
//#define USER_M1_MAX_VS_MAG_PU             (0.65f)
//#define USER_M1_MAX_VS_MAG_PU             (0.625f)
#define USER_M1_MAX_VS_MAG_PU               (0.570f)        // 2/3-phase (<0.57)
//#define USER_M1_MAX_VS_MAG_PU             (0.565f)
//#define USER_M1_MAX_VS_MAG_PU             (0.5f)          // Over modulation (>0.5)


//! \brief Defines the reference Vs magnitude in per units allowed
//! \      Set the value equal from 0.5 to 0.95 of the maximum Vs magnitude
#define USER_M1_VS_REF_MAG_PU               (0.9f * USER_M1_MAX_VS_MAG_PU)

//! \brief Defines the R/L excitation frequency, Hz
//!
#define USER_M1_R_OVER_L_EXC_FREQ_Hz        (300.0f)


//! \brief Defines the R/L Kp scale factor, pu
//! \brief Kp used during R/L is USER_M1_R_OVER_L_KP_SF * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V / USER_MOTOR1_MAX_CURRENT_A;
//!
#define USER_M1_R_OVER_L_KP_SF              (0.02f)


//! \brief Defines maximum acceleration for the estimation speed profiles, Hz/sec
//!
#define USER_M1_MAX_ACCEL_Hzps              (2.0f)


//! \brief Defines the controller execution period, usec
//!
#define USER_M1_CTRL_PERIOD_usec            ((float32_t)USER_M1_ISR_PERIOD_usec)


//! \brief Defines the controller execution period, sec
//!
#define USER_M1_CTRL_PERIOD_sec             ((float32_t)USER_M1_CTRL_PERIOD_usec / 1000000.0f)


//! \brief Defines the IdRated delta to use during estimation
//!
#define USER_M1_IDRATED_DELTA_A                 (0.0001f)

#if defined(_FULL_FAST_LIB)
//! \brief Defines the forced angle frequency, Hz
#define USER_M1_FORCE_ANGLE_FREQ_Hz             (1.0f)

//! \brief Defines the forced angle acceleration, Hz
#define USER_M1_FORCE_ANGLE_ACCEL_Hzps          (1.0f)
#else  // !_FULL_FAST_LIB
//! \brief Defines the forced angle frequency, Hz
#define USER_M1_FORCE_ANGLE_FREQ_Hz             (1.0f)
#endif  // !_FULL_FAST_LIB

//! \brief Defines the near zero speed limit for electrical frequency estimation, Hz
//!        The flux integrator uses this limit to regulate flux integration
#define USER_M1_FREQ_NEARZEROSPEEDLIMIT_Hz      (0.5f)

//! \brief Defines the fraction of IdRated to use during inductance estimation
//!
#define USER_M1_IDRATED_FRACTION_FOR_L_IDENT    (0.5f)


//! \brief Defines the fraction of SpeedMax to use during inductance estimation
//!
#define USER_M1_SPEEDMAX_FRACTION_FOR_L_IDENT   (1.0f)


//! \brief Defines the Power Warp gain for computing Id reference
//!
#define USER_M1_PW_GAIN                         (1.0f)


//! \brief Defines the pole location for the DC bus filter, rad/sec
//!
#define USER_M1_DCBUS_POLE_rps                  (100.0f)


//! \brief Defines the pole location for the voltage and current offset estimation, rad/s
//!
#define USER_M1_OFFSET_POLE_rps                 (20.0f)


//! \brief Defines the pole location for the speed control filter, rad/sec
//!
#define USER_M1_SPEED_POLE_rps                  (100.0f)


//! \brief Defines the pole location for the direction filter, rad/sec
//!
#define USER_M1_DIRECTION_POLE_rps              (MATH_TWO_PI * 10.0f)

//! \brief Defines the pole location for the flux estimation, rad/sec
//!
#define USER_M1_FLUX_POLE_rps                   (10.0f)


//! \brief Defines the pole location for the R/L estimation, rad/sec
//!
#define USER_M1_R_OVER_L_POLE_rps               (MATH_TWO_PI * 3.2f)


//! \brief Defines the convergence factor for the estimator
//!
#define USER_M1_EST_KAPPAQ                      (1.5f)

//! \brief Defines the scale factor for the flux estimation
//! the default value is 1.0f, change the value between 0.1f and 1.25f
//!
//#define USER_M1_EST_FLUX_HF_SF                (0.120f)
//#define USER_M1_EST_FLUX_HF_SF                  (0.1750f)
//#define USER_M1_EST_FLUX_HF_SF                  (0.200f)
#define USER_M1_EST_FLUX_HF_SF                  (0.250f)
//#define USER_M1_EST_FLUX_HF_SF                (1.00f)

//! \brief Defines the scale factor for the frequency estimation
//! the default value is 1.0f, change the value between 0.5f and 1.5f
//!
#define USER_M1_EST_FREQ_HF_SF                  (1.00f)
//#define USER_M1_EST_FREQ_HF_SF                  (0.50f)

//! \brief Defines the scale factor for the bemf estimation
//! the default value is 1.0f, change the value between 0.50f and 1.25f
//!
#define USER_M1_EST_BEMF_HF_SF                  (1.00f)
//#define USER_M1_EST_BEMF_HF_SF                  (0.50f)

//------------------------------------------------------------------------------
//! brief Define the Kp gain for Field Weakening Control
#define USER_M1_FWC_KP                          (0.0525f)

//! brief Define the Ki gain for Field Weakening Control
#define USER_M1_FWC_KI                          (0.00325f)

//! brief Define the maximum current vector angle for Field Weakening Control
#define USER_M1_FWC_MAX_ANGLE          -15.0f                        // degree
#define USER_M1_FWC_MAX_ANGLE_RAD      USER_M1_FWC_MAX_ANGLE /180.0f * MATH_PI  // rad

//! brief Define the minimum current vector angle for Field Weakening Control
#define USER_M1_FWC_MIN_ANGLE          0.0f                          // degree
#define USER_M1_FWC_MIN_ANGLE_RAD      USER_M1_FWC_MIN_ANGLE /180.0f * MATH_PI  // rad

//! \brief Defines the number of DC bus over/under voltage setting time
//!  timer base = 5ms
#define USER_M1_VOLTAGE_FAULT_TIME_SET          (500U)      // in 5ms

//! \brief Defines the number of motor over load setting time
//!  timer base = 5ms, 1s
#define USER_M1_OVER_LOAD_TIME_SET              (200U)

//! \brief Defines the number of motor stall setting time
//!  timer base = 5ms, 1s
#define USER_M1_STALL_TIME_SET                  (200U)

//! \brief Defines the number of phase unbalanced setting time
//!  timer base = 5ms, 5s
#define USER_M1_UNBALANCE_TIME_SET              (1000U)

//! \brief Defines the number of lost phase setting time
//!  timer base = 5ms, 10s
#define USER_M1_LOST_PHASE_TIME_SET             (2000U)

//! \brief Defines the number of over speed setting time
//!  timer base = 5ms
#define USER_M1_OVER_SPEED_TIME_SET             (600U)

//! \brief Defines the number of startup failed setting time
//!  timer base = 5ms, 10s
#define USER_M1_STARTUP_FAIL_TIME_SET           (2000U)

//! \brief Defines the number of over load setting times
//!
#define USER_M1_OVER_CURRENT_TIMES_SET          (5U)

//! \brief Defines the number of stop wait time
//!  timer base = 5ms, 10s
#define USER_M1_STOP_WAIT_TIME_SET              (2000U)

//! \brief Defines the number of restart wait time
//!  timer base = 5ms, 10s
#define USER_M1_RESTART_WAIT_TIME_SET           (2000U)

//! \brief Defines the number of restart times when has a fault
//!
#define USER_M1_START_TIMES_SET                 (3U)

//! \brief Defines the number of boot charge time
//!
#define USER_M1_BOOT_CHARGE_TIME_SET            (2000U)

//! \brief Defines the alignment time
//!  timer base = 5ms, 10s
#define USER_M1_ALIGN_TIME_SET              (2000U)

//! \brief Defines the QEP unit ticks
#define USER_M1_QEP_UNIT_TIMER_TICKS        (uint32_t)(USER_SYSTEM_FREQ_MHz/(2.0f * USER_M1_ISR_FREQ_Hz) * 1000000.0f)

//! \brief Defines the current filter pole location, Hz
#define USER_M1_IS_FILTER_POLE_Hz           (7500.0f)      // 7.5kHz

//! \brief Defines the current filter pole location, rad/s
//!
#define USER_M1_IS_FILTER_POLE_rps          (MATH_TWO_PI * USER_M1_IS_FILTER_POLE_Hz)


//! \brief Defines the voltage filter pole location, Hz
//!
#define USER_M1_VS_FILTER_POLE_Hz           (30000.0f)     // 30.0kHz

//! \brief Defines the voltage filter pole location, rad/s
//!
#define USER_M1_VS_FILTER_POLE_rps          (MATH_TWO_PI * USER_M1_VS_FILTER_POLE_Hz)


//! \brief Defines the module temperature coefficient
//!
#define USER_M1_MODULE_TEMP_COEF            (25U)

//! \brief Defines the module temperature offset
//!
#define USER_M1_MODULE_TEMP_OFFSET          (10U)

//! \brief Defines the motor temperature coefficient
//!
#define USER_M1_MOTOR_TEMP_COEF             (25U)

//! \brief Defines the motor temperature offset
//!
#define USER_M1_MOTOR_TEMP_OFFSET           (10U)

// PWM Frequency
#define USER_M1_PWM_FREQ_HZ                 (USER_M1_PWM_FREQ_kHz * 1000.0f)

//==============================================================================
// Only a few listed motor below are tested with the related algorithm as the comments
// Motor defines
// High voltage PMSM Motors
//#define USER_MOTOR1 Estun_EMJ_04APB22            //* Tested, FAST/eSMO
//#define USER_MOTOR1 Anaheim_BLWS235D             //* Tested, FAST
//#define USER_MOTOR1 VFWMPM_DDLG8p4kg
//#define USER_MOTOR1 CHMotor_WM_Test
//#define USER_MOTOR1 LACFAN_ZKSN_750               //* RsOline Test
//#define USER_MOTOR1 WelingFan_1HP_IPM              //* RsOline Test

//#define USER_MOTOR1 Anaheim_BLZ362S
//#define USER_MOTOR1 ziehlab_Fan160hv              //* Tested, FAST/eSMO
//#define USER_MOTOR1 embpast_Fan160hv              //* Tested, FAST/eSMO
//#define USER_MOTOR1 GMCC_KSK89D53U                //* Tested, FAST/eSMO
//#define USER_MOTOR1 QXA_A091ZE190A                //* Tested, FAST/eSMO
//#define USER_MOTOR1 Baldor_BSM90N175              //* Tested, FAST/eSMO
//#define USER_MOTOR1 Marathon_N56PNRA10102         //* Tested, FAST

// Low Voltage PMSM Motors
#define USER_MOTOR1 Teknic_M2310PLN04K              //* Tested, FAST/eSMO
//#define USER_MOTOR1 Anaheim_BLY172S_24V           //* Tested, FAST/eSMO(MD)
//#define USER_MOTOR1 Nedic_EPSMS037_D12V
//#define USER_MOTOR1 Anaheim_BLY341S_Y48V
//#define USER_MOTOR1 Anaheim_BLY341S_D48V
//#define USER_MOTOR1 Anaheim_BLY341S_Y24V
//#define USER_MOTOR1 Anaheim_BLY341S_D24V
//#define USER_MOTOR1 Drone_DJI920KV                //* Tested, FAST
//#define USER_MOTOR1 Drone_BLK2BLADE               //* Tested, FAST
//#define USER_MOTOR1 Drone_SENSEFLY                //* Tested, FAST
//#define USER_MOTOR1 Drone_SF_Black                //* Tested, FAST
//#define USER_MOTOR1 Drone9616_110KV_48V           //* Tested, FAST
//#define USER_MOTOR1 M2221_M16_J3_25V              //* 100krpm/1670Hz motor

//#define USER_MOTOR1 Tamagawa_TS4606N8302          //* Tested, FAST
//#define USER_MOTOR1 AKM21G_CK9NGE00               //* Tested, FAST

//#define USER_MOTOR1 AirFan_MFA0500_24V            //* Tested, FAST
//#define USER_MOTOR1 Tool_Makita_GFD01             //* Tested, FAST

// ACI Motor
//#define USER_MOTOR1 Marathon_5K33GN2A
//#define USER_MOTOR1 Marathon_56H17T2011A
//#define USER_MOTOR1 Dayton_3N352C
//#define USER_MOTOR1 EMSYNERGY_LVACI

//#define USER_MOTOR1 my_pm_motor_1
//#define USER_MOTOR1 my_aci_motor_2
//#define USER_MOTOR1 RL_simulation

//------------------------------------------------------------------------------
#if (USER_MOTOR1 == Teknic_M2310PLN04K)
// the motor type
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM

// the number of pole pairs of the motor
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)

// the rotor resistance value of the motor, in Ohm
#define USER_MOTOR1_Rr_Ohm                 (NULL)

// the stator resistance value of the motor, in Ohm
#define USER_MOTOR1_Rs_Ohm                 (0.407732785f)

// the stator inductance value of the motor in the direct direction, in H
#define USER_MOTOR1_Ls_d_H                 (0.000179637806f)

// the stator inductance value of the motor in the quadrature direction, in H
#define USER_MOTOR1_Ls_q_H                 (0.000179637806f)

// the rated flux value of the motor, in V/Hz
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0412942767f)

// the Id rated current value of the motor, in A. Induction motors only
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)

// the maximum current value for stator resistance (R_s) identification, in A
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)

// the maximum current value to use for stator inductance identification, in A
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0f)

// the maximum current value of the motor, in A
#define USER_MOTOR1_MAX_CURRENT_A          (9.5f)

// the R/L excitation frequency for motor parameters identification, in Hz
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (60.0f)

// the inertia that describes the amount of mass, in Kg.m2
#define USER_MOTOR1_INERTIA_Kgm2           (1.56154e-05)    // with dyno

// the rated voltage of the motor, V
#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)          // V

// the minimum rotation frequency if the motor (Hz)
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)           // Hz

// the maximum/base rotation frequency of the motor (Hz)
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)         // Hz

// V/f Profile Parameters for open-loop in build level 2
// the low frequency f_low  of V/f profile, in Hz,
// set to 10% of rated motor frequency
#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)           // Hz

// the high frequency f_high of V/f profile, in Hz,
// set to 100% of rated motor frequency
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz

// the minimum voltage V_min  of V/f profile,
// the value is suggested to set to 15% of rated motor voltage, in Volt.
#define USER_MOTOR1_VOLT_MIN_V             (1.0f)           // Volt

// the maximum voltage,  V_max of V/f profile,
// the value is suggested to set to 100% of rated motor voltage, in Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

// the current increasing delta value for running the motor with force open-loop , in A
#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A

// the current increasing delta value for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A

// the current for running the motor with force open-loop or startup, in A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A

// the current for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A

// the current for start to run motor with closed-loop when the speed is
//  lower than the startup setting speed, in A
#define USER_MOTOR1_STARTUP_CURRENT_A      (2.0f)           // A

// the current for running the motor with torque control mode when start the motor, in A.
#define USER_MOTOR1_TORQUE_CURRENT_A       (3.5f)           // A

// the over-current threshold for the motor, in A.
// The value can be set to 50%~300% of the rated current of the motor
#define USER_MOTOR1_OVER_CURRENT_A         (12.5f)           // A

// the speed threshold for start the motor, in Hz
#define USER_MOTOR1_SPEED_START_Hz         (35.0f)          // Hz

// the speed threshold for running the motor with force open-loop, in Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (30.0f)          // Hz

// the acceleration for start the motor, in Hz/s.
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)           // Hz/s

// the acceleration for stop the motor, in Hz/s.
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)          // Hz/s

// the maximum acceleration for running the motor, in Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)          // Hz/s

// the speed threshold for running the motor with flying start mode, in Hz
#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)           // Hz

// the current for motor brake, in A.
#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A

// the duration time for motor brake, in 5ms time base
#define USER_MOTOR1_BRAKE_TIME_DELAY        (12000U)         // 60s/5ms

// the slope coefficient for Ld compensation, (0.0f~0.5f)
#define USER_MOTOR1_Ls_d_COMP_COEF          (0.15f)          // 0.0f~0.5f

// the slope coefficient for Lq compensation, (0.0f~0.5f)
#define USER_MOTOR1_Ls_q_COMP_COEF          (0.35f)          // 0.0f~0.5f

// the minimum inductance coefficient for inductance compensation, (0.0f~0.5f)
#define USER_MOTOR1_Ls_MIN_NUM_COEF         (0.55f)          // 0.5f~1.0f

// the near zero speed limit for electrical frequency estimation, Hz.
// The flux integrator uses this limit to regulate flux integration
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)           // Hz

// the waiting time without enabling the Rs online clibrartion,  in 5ms
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (12000U)        // 1min(60s) at 5ms base

// the the duration time with enabling the Rs online clibrartion,  in 5ms
#define USER_MOTOR1_RSONLINE_WORK_TIME      (6000U)         // 0.5min(30s) at 5ms base

// Only for eSMO
// PLL (phase-locked loop)
// PID proportional, integral, derivative
// the sliding mode control maximum gain that equals to Ke*fmax/vscale/sqrt(2)*factor(max),
//  tune the factor(0.1~10) based the test status
#define USER_MOTOR1_KSLIDE_MAX             (0.750f)     // 0.5

// the sliding mode control minimum gain that equals to Ke*fmin/vscale/sqrt(2)*factor(min),
//  tune the factor(0.1~10) based the test status
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)      // 0.05

// the PLL control maximum gain that equals to 2*(Damping factor)*(Natural frequency)*factor(max),
// tune the factor(0.1~10) based the test status
#define USER_MOTOR1_PLL_KP_MAX             (5.0f)      // 5.0

// the PLL control minimum gain that equals to 2*(Damping factor)*(Natural frequency)*factor(min),
// tune the factor(0.1~5) based the test status
#define USER_MOTOR1_PLL_KP_MIN             (0.50f)      // 0.5

// the PLL control gain adjusting coefficient that
// equals to (Kpll_max-Kpll_min)/fscale/fmax
#define USER_MOTOR1_PLL_KP_SF               (2.5f)      // 5.0

// the phase-locked loop control integration gain that
// equals to (Natural frequency)*(Natural frequency)*Ts
#define USER_MOTOR1_PLL_KI                  (2.8125E-06f)    // Not used, reserve

// the threshold of the estimated current error for sliding mode control that
// equals to (motor maximum BEMF voltage / rated voltage), (0.3~0.5 )
#define USER_MOTOR1_BEMF_THRESHOLD          (0.30f)      // 0.25

// the parameters of the low-pass filter for the estimated back EMF,
// Kslf equal to (fc*2*PI()*Ts), (0.5~2.5)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (1.0f)      // 1.0

// the offset coefficient to compensate the error by using
// the low-pass filter that equals to 1.0, or [0.5~1.5]
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)      // 1.0

// the cut-off frequency of the low-pass filter to calculate the estimated speed, (100~400)
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (100.0f)    // 100.0

// Current and Speed PI Regulators Tuning Coefficient
// the low speed threshold for adjusting the Kp and Ki of the speed PI regulator
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)      // 10%~50% of the rated speed

// the high speed threshold for adjusting the Kp and Ki of the speed PI regulator
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)     // 50%~100% of the rated speed

// the gain coefficient to adjust the Kp of the speed PI regulator for startup
#define USER_MOTOR1_KP_SPD_START_SF         (1.5f)       // 0.1~100.0

// the gain coefficient to adjust the Ki of the speed PI regulator for startup
#define USER_MOTOR1_KI_SPD_START_SF         (1.5f)       // 0.1~10.0

// the low gain coefficient  to adjust the Kp of the speed PI regulator
#define USER_MOTOR1_KP_SPD_LOW_SF           (2.0f)       // 0.1~100.0

// the low gain coefficient  to adjust the Ki of the speed PI regulator
#define USER_MOTOR1_KI_SPD_LOW_SF           (2.0f)       // 0.1~10.0

// the high gain coefficient  to adjust the Kp of the speed PI regulator
#define USER_MOTOR1_KP_SPD_HIGH_SF          (1.0f)       // 0.1~100.0

// the high gain coefficient  to adjust the Ki of the speed PI regulator
#define USER_MOTOR1_KI_SPD_HIGH_SF          (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Kp of the q-axis current PI regulator
#define USER_MOTOR1_KP_IQ_SF                (2.0f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the q-axis current PI regulator
#define USER_MOTOR1_KI_IQ_SF                (2.0f)       // 0.1~10.0

// the gain coefficient to adjust the Kp of the q-axis current PI regulator
#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the q-axis current PI regulator
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Kp of the power PI regulator
#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the power PI regulator
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (2.0f)         // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (1.0f)         // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)      // N.m
#define USER_MOTOR1_POWWR_SET_W         (10.0f)     // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)      // A
#define USER_MOTOR1_SPEED_SET_Hz        (50.0f)     // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)     // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//-----------------------------------------------------------------------------
#elif (USER_MOTOR1 == M2221_M16_J3_25V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (1)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (0.0187746398f*1.25f)
#define USER_MOTOR1_Ls_d_H                  (1.97155306e-05)        // 2.06557415e-05
#define USER_MOTOR1_Ls_q_H                  (1.96155306e-05)        // 2.06557415e-05
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.0056031486f)         // 0.0056031486
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (7.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-4.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (200.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (2.8125E-06f)

#if !defined(HC_RC5G10_EN)
    // GAIN = 10
#define USER_MOTOR1_MAX_CURRENT_A           (23.5f)
#define USER_MOTOR1_OVER_CURRENT_A          (31.5f)          // A
#else   // HC_RC5G10_EN
    // GAIN = 5/132
#define USER_MOTOR1_MAX_CURRENT_A           (87.5f)         // Max: 132A
#define USER_MOTOR1_OVER_CURRENT_A          (125.5f)        // Max: 132A
#endif  // HC_RC5G10_EN

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF          (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF          (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF         (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V         (30.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (2000.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (600.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (20.0f)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (2.0f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.5f)          // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)

#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (20.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (50.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (100.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (800.0f)

#define USER_MOTOR1_KP_SPD_START_SF         (0.75f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (0.75f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF           (1.00f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (1.00f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF          (0.75f)      // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (0.75f)      // 0.1~10.0

//#define USER_MOTOR1_KP_SPD_LOW_SF           (0.7500f)    // 0.1~100.0
//#define USER_MOTOR1_KI_SPD_LOW_SF           (0.7500f)    // 0.1~10.0

//#define USER_MOTOR1_KP_SPD_HIGH_SF          (0.00250f)   // 0.1~100.0
//#define USER_MOTOR1_KI_SPD_HIGH_SF          (0.00125f)   // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF          (1.50f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF          (1.50f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.00f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.00f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.00f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.00f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (15.0f)        // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (10.0f)        // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (20.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)          // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (2.3679111f)        // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (70.0f)
#define WINDING_NORM_TEMP_MIN           (60.0f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//--------------------------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Estun_EMJ_04APB22)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (2.3679111f)
#define USER_MOTOR1_Ls_d_H                  (0.00836551283f)
#define USER_MOTOR1_Ls_q_H                  (0.00836551283f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.390533477f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0f)
#define USER_MOTOR1_MAX_CURRENT_A           (6.5f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF          (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF          (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF         (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (15.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)        // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.10f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.10f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (2.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (3.0f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (2.0f)          // A
#define USER_MOTOR1_OVER_CURRENT_A          (6.5f)          // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)

#define USER_MOTOR1_ACCEL_START_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps         (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps          (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX              (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN              (0.75f)

#define USER_MOTOR1_PLL_KP_MAX              (10.0f)
#define USER_MOTOR1_PLL_KP_MIN              (2.0f)
#define USER_MOTOR1_PLL_KP_SF               (5.0f)
#define USER_MOTOR1_PLL_KI                  (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD          (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF         (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF           (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF          (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (15.0f)        // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (10.0f)        // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)          // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (2.68207002f)       // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (65.0f)
#define WINDING_NORM_TEMP_MIN           (55.0f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s


#elif (USER_MOTOR1 == WelingFan_1HP_IPM)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (5)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (2.15664363f)       // 2.15664363
#define USER_MOTOR1_Ls_d_H                  (0.029f)            // 0.0346654505
#define USER_MOTOR1_Ls_q_H                  (0.037f)            // 0.0346654505
#define USER_MOTOR1_RATED_FLUX_VpHz         (1.05193818f)       // 1.05193818
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (2.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.5f)
#define USER_MOTOR1_MAX_CURRENT_A           (6.62f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (2.06154e-5)    // (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF          (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF          (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF         (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (120.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (40.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (220.0f)        // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (0.5f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (4.0f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (3.0f)          // A
#define USER_MOTOR1_OVER_CURRENT_A          (7.85f)         // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)

#define USER_MOTOR1_ACCEL_START_Hzps       (5.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (10.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF         (0.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (0.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF           (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF          (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF          (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (15.0f)        // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (10.0f)        // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (30.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)          // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (2.675715f)         // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (70.0f)
#define WINDING_NORM_TEMP_MIN           (60.0f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s
//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == LACFAN_ZKSN_750)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (2.71590972f)
#define USER_MOTOR1_Ls_d_H                  (0.0518590212f)
#define USER_MOTOR1_Ls_q_H                  (0.0518590212f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (1.44512928f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (2.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.5f)
#define USER_MOTOR1_MAX_CURRENT_A           (6.5f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (7.6500017e-4)   // (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF          (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF          (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF         (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (120.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (40.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (220.0f)        // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (2.0f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.5f)          // A
#define USER_MOTOR1_OVER_CURRENT_A          (6.5f)          // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)

#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF         (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF           (3.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (3.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF          (2.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (2.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF          (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (15.0f)        // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (10.0f)        // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (30.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)          // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (2.675715f)         // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (70.0f)
#define WINDING_NORM_TEMP_MIN           (60.0f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Tool_Makita_GFD01)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (NULL)

// 15kHz
//#define USER_MOTOR1_Rs_Ohm                 (0.0253030714f)
//#define USER_MOTOR1_Ls_d_H                 (2.89102645e-05f)
//#define USER_MOTOR1_Ls_q_H                 (2.89102645e-05f)
//#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0242515542f)

//// 25kHz
//#define USER_MOTOR1_Rs_Ohm                 (0.0250713862f)
//#define USER_MOTOR1_Ls_d_H                 (2.84397684e-05f)
//#define USER_MOTOR1_Ls_q_H                 (2.84397684e-05f)
//#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0242347941f)

// 20kHz
#define USER_MOTOR1_Rs_Ohm                 (0.0379858911f)
#define USER_MOTOR1_Ls_d_H                 (0.000106012929f)
#define USER_MOTOR1_Ls_q_H                 (0.000106012929f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0138726495f)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (5.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.5f)
#define USER_MOTOR1_MAX_CURRENT_A          (25.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (100.0f)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (36.0f)          // V
#define USER_MOTOR1_FREQ_MIN_Hz            (10.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (1500.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0f)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (1500.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V             (3.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (36.0f)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (8.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (5.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (26.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (35.0f)          // Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (30.0f)          // Hz
#define USER_MOTOR1_ACCEL_START_Hzps       (50.0f)          // Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps         (200.0f)         // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)           // Hz

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.50f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (1.50f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration with FAST
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Controller Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s
//-----------------------------------------------------------------------------------------
#elif (USER_MOTOR1 == CHMotor_WM_Test)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (8)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (3.56442809f)
#define USER_MOTOR1_Ls_d_H                  (0.0478876233f)
#define USER_MOTOR1_Ls_q_H                  (0.0478876233f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.377903223f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A           (5.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)        // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (2.5f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)          // A
#define USER_MOTOR1_OVER_CURRENT_A          (4.5f)          // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0f)         // Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps          (20.0f)        // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Marathon_N56PNRA10102)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (3)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (2.25905824f)
#define USER_MOTOR1_Ls_d_H                 (0.0270862058f)
#define USER_MOTOR1_Ls_q_H                 (0.0270862058f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.575999975f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-0.5f)
#define USER_MOTOR1_MAX_CURRENT_A          (5.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (2500)
#define USER_MOTOR1_INERTIA_Kgm2           (6.300017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V        (200.0f)         // V
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (8.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)          // Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0f)          // Hz

#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 1.5f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 2.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 1.0f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 200.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (12000U)    // 1min/60s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (12000U)     //1min/60s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF          (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (750.0f)       // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (12.50f)       // coarse IPD excitation magnitude, pu
#define IPD_HFI_EXC_MAG_FINE_V          (10.25f)       // fine IPD excitation magnitude, pu
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(2.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(2.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(2.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (2.0f)
#define AFSEL_MAX_IQ_REF_HFI_A          (2.0f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == VFWMPM_DDLG8p4kg)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (2.200221f)
#define USER_MOTOR1_Ls_d_H                 (0.008721023f)
#define USER_MOTOR1_Ls_q_H                 (0.008721023f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.3846985f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A          (5.5f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (2500)
#define USER_MOTOR1_INERTIA_Kgm2           (6.300017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V        (200.0f)         // V
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (1000.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (8.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)          // Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0f)          // Hz

#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 1.5f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 2.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 1.0f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 200.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (12000U)    // 1min/60s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (12000U)     //1min/60s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF          (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (750.0f)       // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (12.50f)       // coarse IPD excitation magnitude, pu
#define IPD_HFI_EXC_MAG_FINE_V          (10.25f)       // fine IPD excitation magnitude, pu
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(2.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(2.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(2.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (2.0f)
#define AFSEL_MAX_IQ_REF_HFI_A          (2.0f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Tamagawa_TS4606N8302)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)

#define USER_MOTOR1_Rs_Ohm                 (0.214345068f)       // ohm
#define USER_MOTOR1_Ls_d_H                 (0.000486983976f)    // H
#define USER_MOTOR1_Ls_q_H                 (0.000486983976f)    // H
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0441591553f)      // V/Hz

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.0f)               // A
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0f)              // A
#define USER_MOTOR1_MAX_CURRENT_A          (8.5f)               // A
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (60.0f)              // Hz
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)        // Kgm2

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0f)              // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)             // Hz
#define USER_MOTOR1_VOLT_MIN_V             (3.0f)               // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)              // Volt

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)              // V
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)               // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)             // Hz

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)              // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)              // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)               // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)               // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)               // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (3.0f)               // A
#define USER_MOTOR1_OVER_CURRENT_A         (7.5f)               // A

#define USER_MOTOR1_SPEED_START_Hz         (5.0f)               // Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (5.0f)               // Hz

#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)               // Hz

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)               // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)             // 60s/5ms

#if defined(MOTOR1_FAST)
// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

// for Rs online calibration with FAST
#define USER_MOTOR1_RSONLINE_WAIT_TIME     (60000U)         // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME     (24000U)         //2min/120s at 5ms base
#endif  // MOTOR1_FAST

#if defined(MOTOR1_ESMO)
// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.50f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (1.50f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)
#endif  // MOTOR1_ESMO

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.25f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.05f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.25f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (1.75f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_START_SF          (1.65f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.55f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF           (1.35f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.15f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.15f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.05f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                (1.15f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.05f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == AKM21G_CK9NGE00)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (3)
#define USER_MOTOR1_Rr_Ohm                 (NULL)

#define USER_MOTOR1_Rs_Ohm                 (0.724585772f)
#define USER_MOTOR1_Ls_d_H                 (0.00115217688f)
#define USER_MOTOR1_Ls_q_H                 (0.00115217688f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.105363153f)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A          (8.5f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (60.0f)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0f)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (3.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)          // V
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)         // Hz

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (3.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (7.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (5.0f)           // Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (5.0f)           // Hz
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)          // Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)          // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)           // Hz

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

// for Rs online calibration with FAST
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.50f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (1.50f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Anaheim_BLY172S_24V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.399648607f)
#define USER_MOTOR1_Ls_d_H                 (0.000585399743f)
#define USER_MOTOR1_Ls_q_H                 (0.000585399743f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0343291275f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.5f)
#define USER_MOTOR1_MAX_CURRENT_A          (7.5f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (2000)
#define USER_MOTOR1_INERTIA_Kgm2           (4.80185e-06)

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)        // Hz
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.05f           // A
#define USER_MOTOR1_FLUX_CURRENT_A         (1.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (15.0f)

#define USER_MOTOR1_ACCEL_START_Hzps       (5.0f)
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

// for Rs online calibration with FAST
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)      // 10%~50% of the rated speed
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)     // 50%~100% of the rated speed
#define USER_MOTOR1_KP_SPD_START_SF         (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KP_SPD_LOW_SF           (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_SPD_HIGH_SF          (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_IQ_SF                (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (2.0f)         // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (1.0f)         // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Baldor_BSM90N175)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0)
#define USER_MOTOR1_Rs_Ohm                  (0.636681914f)
#define USER_MOTOR1_Ls_d_H                  (0.00240929401f)
#define USER_MOTOR1_Ls_q_H                  (0.00240929401f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.635046303f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.5f)      // A - 10~30% of rated current of the motor
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.5f)     // A - 10~30% of rated current of the motor, just enough to enable rotation
#define USER_MOTOR1_MAX_CURRENT_A           (9.5f)     // A - 30~150% of rated current of the motor
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0f)     // Hz - 10~30% of rated frequency of the motor
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500.0)
#define USER_MOTOR1_INERTIA_Kgm2            (6.327e-4)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V              (10.0f)          // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (3.0f)           //
#define USER_MOTOR1_STARTUP_CURRENT_A       (5.0f)           //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (9.5f)           //

#define USER_MOTOR1_SPEED_START_Hz          (10.0f)          //
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)          //
#define USER_MOTOR1_ACCEL_START_Hzps        (5.0f)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (10.0f)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX              (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN              (0.75f)

#define USER_MOTOR1_PLL_KP_MAX              (10.0f)
#define USER_MOTOR1_PLL_KP_MIN              (2.0f)
#define USER_MOTOR1_PLL_KP_SF               (5.0f)
#define USER_MOTOR1_PLL_KI                  (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD          (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == QXA_A091ZE190A)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (3)
#define USER_MOTOR1_Rr_Ohm                  (0.0)
#define USER_MOTOR1_Rs_Ohm                  (0.771440625f)
#define USER_MOTOR1_Ls_d_H                  (0.0132690407f)
#define USER_MOTOR1_Ls_q_H                  (0.0132690407f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.50693661f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (2.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-2.0f)
#define USER_MOTOR1_MAX_CURRENT_A           (8.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500.0f)
#define USER_MOTOR1_INERTIA_Kgm2            (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V              (10.0f)          // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (3.0f)           //
#define USER_MOTOR1_STARTUP_CURRENT_A       (5.0f)           //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (9.5f)           //

#define USER_MOTOR1_SPEED_START_Hz          (10.0f)          //
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)          //
#define USER_MOTOR1_ACCEL_START_Hzps        (5.0f)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (10.0f)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX              (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN              (0.75f)

#define USER_MOTOR1_PLL_KP_MAX              (10.0f)
#define USER_MOTOR1_PLL_KP_MIN              (2.0f)
#define USER_MOTOR1_PLL_KP_SF               (5.0f)
#define USER_MOTOR1_PLL_KI                  (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD          (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == GMCC_KSK89D53U)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (3)
#define USER_MOTOR1_Rr_Ohm                  (0.0)
#define USER_MOTOR1_Rs_Ohm                  (1.175)
#define USER_MOTOR1_Ls_d_H                  (0.0076)
#define USER_MOTOR1_Ls_q_H                  (0.0109)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.5415546)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (3.5)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-3.0)
#define USER_MOTOR1_MAX_CURRENT_A           (8.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (20.0)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500.0)
#define USER_MOTOR1_INERTIA_Kgm2            (0.00018)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V              (10.0f)          // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (3.0f)           //
#define USER_MOTOR1_STARTUP_CURRENT_A       (5.0f)           //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (9.5f)           //

#define USER_MOTOR1_SPEED_START_Hz          (10.0f)          //
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)          //
#define USER_MOTOR1_ACCEL_START_Hzps        (5.0f)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (10.0f)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX              (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN              (0.75f)

#define USER_MOTOR1_PLL_KP_MAX              (10.0f)
#define USER_MOTOR1_PLL_KP_MIN              (2.0f)
#define USER_MOTOR1_PLL_KP_SF               (5.0f)
#define USER_MOTOR1_PLL_KI                  (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD          (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Drone_BLK2BLADE)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (6)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0614009798f)
#define USER_MOTOR1_Ls_d_H                 (1.29998243e-05f)
#define USER_MOTOR1_Ls_q_H                 (1.29998243e-05f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.00359785813f)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (4.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
//! \brief Defines the maximum current at the AD converter
// Gain = 12, Rin=2.49k, Rdac=27.4k
#define USER_MOTOR1_MAX_CURRENT_A          (20.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (120.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR1_INERTIA_Kgm2           (3.06154e-04)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (2000.0)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (2.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
//! \brief Defines the maximum current at the AD converter
// Gain = 12, Rin=2.49k, Rdac=27.4k
#define USER_MOTOR1_OVER_CURRENT_A         (22.25)          //

#define USER_MOTOR1_SPEED_START_Hz         (20.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Drone_DJI920KV)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (7)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.115332372f)
#define USER_MOTOR1_Ls_d_H                 (1.76480826e-05f)
#define USER_MOTOR1_Ls_q_H                 (1.76480826e-05f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.00605002558f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (3.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-2.5)
#define USER_MOTOR1_MAX_CURRENT_A          (20.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (120.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)       // N/A
#define USER_MOTOR1_INERTIA_Kgm2           (5.06154e-06)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (12.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (2000.0)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (1200.0)        // Hz
#define USER_MOTOR1_VOLT_MIN_V             (3.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (12.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (16.5)          //

#define USER_MOTOR1_SPEED_START_Hz         (20.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (100.0f)

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Drone_SF_Black)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (7)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0276225284)
#define USER_MOTOR1_Ls_d_H                 (1.25800107e-05)
#define USER_MOTOR1_Ls_q_H                 (1.25800107e-05)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.02469966)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (6.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
#define USER_MOTOR1_MAX_CURRENT_A          (25.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (80.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-05)

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (2000.0)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)          // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (25.0)          //

#define USER_MOTOR1_SPEED_START_Hz         (20.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps         (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (100.0f)

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == AirFan_MFA0500_24V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.150690928)
#define USER_MOTOR1_Ls_d_H                 (2.34591134e-05)
#define USER_MOTOR1_Ls_q_H                 (2.34591134e-05)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.00798716862f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (4.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
#define USER_MOTOR1_MAX_CURRENT_A          (16.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (200.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-05)

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (1500.0)       // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (1000.0)        // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (14.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (2.5)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (4.5)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (16.5)          // A

#define USER_MOTOR1_SPEED_START_Hz         (120.0)
#define USER_MOTOR1_SPEED_FORCE_Hz         (100.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.25f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (5.00f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.25f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME     (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME     (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Nedic_EPSMS037_D12V)
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0)
#define USER_MOTOR1_Rs_Ohm                  (0.0412785485)
#define USER_MOTOR1_Ls_d_H                  (0.000121561985)
#define USER_MOTOR1_Ls_q_H                  (0.000121561985)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.0572981499)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (5.0)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-4.0)
#define USER_MOTOR1_MAX_CURRENT_A           (12.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0)
#define USER_MOTOR1_NUM_ENC_SLOTS           (1000.0)
#define USER_MOTOR1_INERTIA_Kgm2            (0.015)

#define USER_MOTOR1_RATED_VOLTAGE_V         (12.0)
#define USER_MOTOR1_FREQ_MIN_Hz              (5.0)            // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (1500.0)          // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (1000.0)          // Hz
#define USER_MOTOR1_VOLT_MIN_V              (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V              (12.0)          // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A       (2.0)            //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (12.5)            //

#define USER_MOTOR1_SPEED_START_Hz          (10.0)           //
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps          (10.0)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (100.0)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == ziehlab_Fan160hv)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (0.0)
#define USER_MOTOR1_Rs_Ohm                 (24.0002041)
#define USER_MOTOR1_Ls_d_H                 (0.125637174)
#define USER_MOTOR1_Ls_q_H                 (0.125637174)
#define USER_MOTOR1_RATED_FLUX_VpHz        (1.42636561)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.25)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-0.5)
#define USER_MOTOR1_MAX_CURRENT_A          (4.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (30.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (2500.0)         // No Used
#define USER_MOTOR1_INERTIA_Kgm2           (27.5e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (200.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)            // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0)          // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.0)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (0.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.0)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (4.5)           // A

#define USER_MOTOR1_SPEED_START_Hz         (5.0)          //
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0)          //
#define USER_MOTOR1_ACCEL_START_Hzps       (5.0)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps         (10.0)          //

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == embpast_Fan160hv)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (0.0)

#define USER_MOTOR1_Rs_Ohm                 (8.76899433)
#define USER_MOTOR1_Ls_d_H                 (0.0284830406)
#define USER_MOTOR1_Ls_q_H                 (0.0284830406)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.280854076)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.25)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-0.30)
#define USER_MOTOR1_MAX_CURRENT_A          (3.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (2500.0)
#define USER_MOTOR1_INERTIA_Kgm2           (27.5e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V        (200.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)            // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0)          // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.0)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (0.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (2.0)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.25)          // A

#define USER_MOTOR1_SPEED_START_Hz         (20.0)          //
#define USER_MOTOR1_SPEED_FORCE_Hz         (30.0)          //

#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                 (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)       // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (0.25f)         // coarse IPD excitation magnitude, pu
#define IPD_HFI_EXC_MAG_FINE_V          (0.2f)         // fine IPD excitation magnitude, pu
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (20.0f)
#define AFSEL_FREQ_LOW_HZ               (10.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Anaheim_BLWS235D)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (2)
#define USER_MOTOR1_Rr_Ohm                  (NULL)
#define USER_MOTOR1_Rs_Ohm                  (5.44400978)
#define USER_MOTOR1_Ls_d_H                  (0.0224932302)
#define USER_MOTOR1_Ls_q_H                  (0.0224932302)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.560590863)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0)
#define USER_MOTOR1_MAX_CURRENT_A           (2.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0)
#define USER_MOTOR1_INERTIA_Kgm2            (0.0000230206204) // 0.003261 oz-in-sec2

#define USER_MOTOR1_RATED_VOLTAGE_V         (160.0)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0)             // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (200.0)           // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0)            // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0)           // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0)            // Volt
#define USER_MOTOR1_VOLT_MAX_V              (160.0)           // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (1.5)            //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (2.0)            //

#define USER_MOTOR1_SPEED_START_Hz          (30.0)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (100.0)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

#if defined(MOTOR1_FAST)
// Only for FAST
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base
#endif  // MOTOR1_FAST

#if defined(MOTOR1_ESMO)
// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f
#endif  // MOTOR1_ESMO

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Anaheim_BLZ362S)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (NULL)
#define USER_MOTOR1_Rs_Ohm                  (0.562865257f)
#define USER_MOTOR1_Ls_d_H                  (0.00117779721f)
#define USER_MOTOR1_Ls_q_H                  (0.00117779721f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.276417136f)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (2.5)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.5)
#define USER_MOTOR1_MAX_CURRENT_A           (7.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0)
#define USER_MOTOR1_NUM_ENC_SLOTS           (NULL)
#define USER_MOTOR1_INERTIA_Kgm2            (0.0000230206204) // 0.003261 oz-in-sec2

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (160.0)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0)             // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (200.0)           // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0)            // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0)           // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0)            // Volt
#define USER_MOTOR1_VOLT_MAX_V              (160.0)           // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (2.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (4.5)            //
#define USER_MOTOR1_TORQUE_CURRENT_A        (0.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (6.5)            //

#define USER_MOTOR1_SPEED_START_Hz          (30.0)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (100.0)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Regal_Beloit_5SME39DL0756)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (3)
#define USER_MOTOR1_Rr_Ohm                 (0.0)
#define USER_MOTOR1_Rs_Ohm                 (4.581007)
#define USER_MOTOR1_Ls_d_H                 (0.03727356)
#define USER_MOTOR1_Ls_q_H                 (0.03727356)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.6589699)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR1_MAX_CURRENT_A          (2.6)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)      // N.m
#define USER_MOTOR1_POWWR_SET_W         (0.4f)      // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)      // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)     // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)   // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Drone9616_110KV_48V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (7)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0282184016)
#define USER_MOTOR1_Ls_d_H                 (1.39398744e-05)
#define USER_MOTOR1_Ls_q_H                 (1.39398744e-05)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0159354247)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (3.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
#define USER_MOTOR1_MAX_CURRENT_A          (50.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (60.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (4.80185e-06)

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (48.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (10.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (1200.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (48.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5)           //
#define USER_MOTOR1_STARTUP_CURRENT_A      (5.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (75.0)           //

#define USER_MOTOR1_SPEED_START_Hz         (75.0)
#define USER_MOTOR1_SPEED_FORCE_Hz         (60.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (50.0)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX              (1.00f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN              (0.05f)

#define USER_MOTOR1_PLL_KP_MAX              (15.0f)
#define USER_MOTOR1_PLL_KP_MIN              (2.0f)
#define USER_MOTOR1_PLL_KP_SF               (20.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD          (0.05f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Anaheim_BLY341S_Y48V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.423402995f)       // D:
#define USER_MOTOR1_Ls_d_H                 (0.00112280098f)     // D:
#define USER_MOTOR1_Ls_q_H                 (0.00112280098f)     // D:
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0970712677f)      // D:
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-2.0f)
#define USER_MOTOR1_MAX_CURRENT_A          (20.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (1.20185e-05)

#define USER_MOTOR1_RATED_VOLTAGE_V        (48.0f)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)         // Hz
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (48.0f)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.05f           // A
#define USER_MOTOR1_FLUX_CURRENT_A         (1.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (15.0f)

#define USER_MOTOR1_ACCEL_START_Hzps       (5.0f)
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)          // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)          // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

// for Rs online calibration with FAST
#define USER_MOTOR1_RSONLINE_WAIT_TIME     (60000U)        // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME     (24000U)        //2min/120s at 5ms base

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.75f)
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)

#define USER_MOTOR1_PLL_KP_MAX             (7.5f)
#define USER_MOTOR1_PLL_KP_MIN             (1.5f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)      // 10%~50% of the rated speed
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)     // 50%~100% of the rated speed

#define USER_MOTOR1_KP_SPD_START_SF         (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF           (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_SPD_HIGH_SF          (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (2.0f)         // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (1.0f)         // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Anaheim_BLY341S_D48V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.143626466f)       // Y:
#define USER_MOTOR1_Ls_d_H                 (0.000371986447f)    // Y:
#define USER_MOTOR1_Ls_q_H                 (0.000371986447f)    // Y:
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0559178814f)      // Y:
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-2.0f)
#define USER_MOTOR1_MAX_CURRENT_A          (20.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (1.20185e-05)

#define USER_MOTOR1_RATED_VOLTAGE_V        (48.0f)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0)         // Hz
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (5.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (48.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.05f           // A
#define USER_MOTOR1_FLUX_CURRENT_A         (1.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.5f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (15.0f)

#define USER_MOTOR1_ACCEL_START_Hzps       (5.0f)
#define USER_MOTOR1_ACCEL_STOP_Hzps        (10.0f)              // Hz/s
#define USER_MOTOR1_ACCEL_RUN_Hzps         (20.0f)              // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

// for Rs online calibration with FAST
#define USER_MOTOR1_RSONLINE_WAIT_TIME     (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME     (24000U)     //2min/120s at 5ms base

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX              (0.75f)
#define USER_MOTOR1_KSLIDE_MIN              (0.15f)

#define USER_MOTOR1_PLL_KP_MAX              (7.5f)
#define USER_MOTOR1_PLL_KP_MIN              (1.5f)
#define USER_MOTOR1_PLL_KP_SF               (5.0f)
#define USER_MOTOR1_PLL_KI                  (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD          (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF         (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF         (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz         (200.0f)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz       (60.0f)      // 10%~50% of the rated speed
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz      (150.0f)     // 50%~100% of the rated speed
#define USER_MOTOR1_KP_SPD_START_SF         (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF         (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KP_SPD_LOW_SF           (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF           (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_SPD_HIGH_SF          (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF          (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_IQ_SF                (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_SF                (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KP_ID_SF                (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_POW_SF               (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_POW_SF               (1.0f)       // 0.1~10.0

// IPD and AFSEL settings below are not necessarily valid for this motor
#define IPD_HFI_EXC_FREQ_HZ             (750.0f)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ          (35.0f)        // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ           (100.0f)       // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD                    (60.0f)        // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_V        (2.0f)         // coarse IPD excitation magnitude, V
#define IPD_HFI_EXC_MAG_FINE_V          (1.0f)         // fine IPD excitation magnitude, V
#define IPD_HFI_EXC_TIME_COARSE_S       (0.5f)         // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S         (0.5f)         // fine wait time, sec max 0.4

#define AFSEL_FREQ_HIGH_HZ              (25.0f)
#define AFSEL_FREQ_LOW_HZ               (15.0f)
#define AFSEL_IQ_SLOPE_EST_A            ((float32_t)(1.0f/0.1f/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_HFI_A            ((float32_t)(1.0/10.0/USER_M1_ISR_FREQ_Hz))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN     ((float32_t)(1.0/0.05/USER_M1_ISR_FREQ_Hz))
#define AFSEL_MAX_IQ_REF_EST_A          (0.4f)
#define AFSEL_MAX_IQ_REF_HFI_A          (0.4f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Anaheim_BLY341S_Y24V)
// the motor type
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM

// the number of pole pairs of the motor
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)

// the rotor resistance value of the motor, in Ohm
#define USER_MOTOR1_Rr_Ohm                 (NULL)

// the stator resistance value of the motor, in Ohm
#define USER_MOTOR1_Rs_Ohm                 (0.133211181f)

// the stator inductance value of the motor in the direct direction, in H
#define USER_MOTOR1_Ls_d_H                 (0.000275031634f)

// the stator inductance value of the motor in the quadrature direction, in H
#define USER_MOTOR1_Ls_q_H                 (0.000275031634f)

// the rated flux value of the motor, in V/Hz
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0514687076f)

// the Id rated current value of the motor, in A. Induction motors only
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)

// the maximum current value for stator resistance (R_s) identification, in A
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.5f)

// the maximum current value to use for stator inductance identification, in A
#define USER_MOTOR1_IND_EST_CURRENT_A      (-2.0f)

// the maximum current value of the motor, in A
#define USER_MOTOR1_MAX_CURRENT_A          (8.6f)

// the R/L excitation frequency for motor parameters identification, in Hz
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)

// the inertia that describes the amount of mass, in Kg.m2
#define USER_MOTOR1_INERTIA_Kgm2           (3.99683e-05)

// the rated voltage of the motor, V
#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)          // V

// the minimum rotation frequency if the motor (Hz)
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)           // Hz

// the maximum/base rotation frequency of the motor (Hz)
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)         // Hz

// V/f Profile Parameters for open-loop in build level 2
// the low frequency f_low  of V/f profile, in Hz,
// set to 10% of rated motor frequency
#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)           // Hz

// the high frequency f_high of V/f profile, in Hz,
// set to 100% of rated motor frequency
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz

// the minimum voltage V_min  of V/f profile,
// the value is suggested to set to 15% of rated motor voltage, in Volt.
#define USER_MOTOR1_VOLT_MIN_V             (1.0f)           // Volt

// the maximum voltage,  V_max of V/f profile,
// the value is suggested to set to 100% of rated motor voltage, in Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

// the current increasing delta value for running the motor with force open-loop , in A
#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A

// the current increasing delta value for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A

// the current for running the motor with force open-loop, in A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A

// the current for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A

// the current for start to run motor with closed-loop when the speed is
//  lower than the startup setting speed, in A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A

// the current for running the motor with torque control mode when start the motor, in A.
#define USER_MOTOR1_TORQUE_CURRENT_A       (3.0f)           // A

// the over-current threshold for the motor, in A.
// The value can be set to 50%~300% of the rated current of the motor
#define USER_MOTOR1_OVER_CURRENT_A         (7.5f)           // A

// the speed threshold for start the motor, in Hz
#define USER_MOTOR1_SPEED_START_Hz         (35.0f)          // Hz

// the speed threshold for running the motor with force open-loop, in Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (30.0f)          // Hz

// the acceleration for start the motor, in Hz/s.
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)          // Hz/s

// the maximum acceleration for running the motor, in Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)          // Hz/s

// the speed threshold for running the motor with flying start mode, in Hz
#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)           // Hz

// the current for motor brake, in A.
#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A

// the duration time for motor brake, in 5ms time base
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

#if defined(MOTOR1_FAST)
// the slope coefficient for Ld compensation, (0.0f~0.5f)
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f

// the slope coefficient for Lq compensation, (0.0f~0.5f)
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f

// the minimum inductance coefficient for inductance compensation, (0.0f~0.5f)
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

// the near zero speed limit for electrical frequency estimation, Hz.
// The flux integrator uses this limit to regulate flux integration
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)           // Hz

// the waiting time without enabling the Rs online clibrartion,  in 5ms
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base

// the the duration time with enabling the Rs online clibrartion,  in 5ms
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base
#endif  // MOTOR1_FAST

#if defined(MOTOR1_ESMO)
// Only for eSMO
// PLL (phase-locked loop)
// PID proportional, integral, derivative
// the sliding mode control maximum gain that equals to Ke*fmax/vscale/sqrt(2)*factor(max),
//  tune the factor(0.1~10) based the test status
#define USER_MOTOR1_KSLIDE_MAX             (0.50f)

// the sliding mode control minimum gain that equals to Ke*fmin/vscale/sqrt(2)*factor(min),
//  tune the factor(0.1~10) based the test status
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

// the PLL control maximum gain that equals to 2*(Damping factor)*(Natural frequency)*factor(max),
// tune the factor(0.1~10) based the test status
#define USER_MOTOR1_PLL_KP_MAX             (10.0f)

// the PLL control minimum gain that equals to 2*(Damping factor)*(Natural frequency)*factor(min),
// tune the factor(0.1~5) based the test status
#define USER_MOTOR1_PLL_KP_MIN             (1.50f)

// the PLL control gain adjusting coefficient that
// equals to (Kpll_max-Kpll_min)/fscale/fmax
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

// the phase-locked loop control integration gain that
// equals to (Natural frequency)*(Natural frequency)*Ts
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

// the threshold of the estimated current error for sliding mode control that
// equals to (motor maximum BEMF voltage / rated voltage), (0.3~0.5 )
#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)

// the parameters of the low-pass filter for the estimated back EMF,
// Kslf equal to (fc*2*PI()*Ts), (0.5~2.5)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)

// the offset coefficient to compensate the error by using
// the low-pass filter that equals to 1.0, or [0.5~1.5]
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)

// the cut-off frequency of the low-pass filter to calculate the estimated speed, (100~400)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)
#endif  // MOTOR1_ESMO

// Current and Speed PI Regulators Tuning Coefficient
// the low speed threshold for adjusting the Kp and Ki of the speed PI regulator
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)      // 10%~50% of the rated speed

// the high speed threshold for adjusting the Kp and Ki of the speed PI regulator
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)     // 50%~100% of the rated speed

// the gain coefficient to adjust the Kp of the speed PI regulator for startup
#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0

// the gain coefficient to adjust the Ki of the speed PI regulator for startup
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

// the low gain coefficient  to adjust the Kp of the speed PI regulator
#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0

// the low gain coefficient  to adjust the Ki of the speed PI regulator
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

// the high gain coefficient  to adjust the Kp of the speed PI regulator
#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0

// the high gain coefficient  to adjust the Ki of the speed PI regulator
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

// the low current threshold to adjust the Kp and Ki of the q-axis current PI regulator
#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)       // 10%~50% of the rated current

// the high current threshold to adjust the Kp and Ki of the q-axis current PI regulator
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)       // 50%~100% of the rated current

// the gain coefficient to adjust the Kp of the q-axis current PI regulator for startup
#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the q-axis current PI regulator for startup
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

// the low gain coefficient to adjust the Kp of the q-axis current PI regulator
#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0

// the low gain coefficient to adjust the Ki of the q-axis current PI regulator
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

// the high gain coefficient to adjust the Kp of the d-axis current PI regulator
#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

// the high gain coefficient to adjust the Ki of the d-axis current PI regulator
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Kp of the q-axis current PI regulator
#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the q-axis current PI regulator
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0


// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

// Winding temperature calculation parameters
#define WINDING_TEMP_COEF_INV_C         (0.00393f)         // Get the coefficient from motor manufacturer
#define RS_AT_ROOM_TEMP_OHMS            (0.393955578f)      // Get the coefficient from motor manufacturer
#define RS_ROOM_TEMP_C                  (22.0f)             // Get the coefficient from motor manufacturer
#define INV_WINDING_TEMP_COEF_C         (1.0f / WINDING_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_OHMS        (1.0f / RS_AT_ROOM_TEMP_OHMS)

#define WINDING_OVER_TEMP_MAX           (87.5f)
#define WINDING_NORM_TEMP_MIN           (78.5f)
#define WINDING_OVER_TEMP_TIME_SET      (2000)      // 10s

#elif (USER_MOTOR1 == Anaheim_BLY341S_D24V)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0544644073)
#define USER_MOTOR1_Ls_d_H                 (9.58044038e-05)
#define USER_MOTOR1_Ls_q_H                 (9.58044038e-05)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0544644073)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (4.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.5)
#define USER_MOTOR1_MAX_CURRENT_A          (8.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (3.99683e-05)     // 0.00566 oz-in-sec2

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)             // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (300.0)           // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)            // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)           // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0)             // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)            // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)            //
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (8.5)           //

#define USER_MOTOR1_SPEED_START_Hz         (10.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0)

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

#elif (USER_MOTOR1 == EMSYNERGY_LVACI)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (1.05)
#define USER_MOTOR1_Rs_Ohm                 (1.79)
#define USER_MOTOR1_Ls_d_H                 (0.00681)
#define USER_MOTOR1_Ls_q_H                 (0.00681)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165f*14.7f*1.4142f/50.0f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (0.91f*1.4142f)
#define USER_MOTOR1_RES_EST_CURRENT_A      (0.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (2.50)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (10.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (5.0)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0)
#define USER_MOTOR1_SPEED_FORCE_Hz         (25.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0)


// Only for eSMO, N/A
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.25f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

#elif (USER_MOTOR1 == Marathon_5K33GN2A)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (5.508003)
#define USER_MOTOR1_Rs_Ohm                 (10.71121)
#define USER_MOTOR1_Ls_d_H                 (0.05296588)
#define USER_MOTOR1_Ls_q_H                 (0.05296588)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165*220.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (1.378)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (3.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR1_FREQ_MIN_Hz             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)         // Volt

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)      // N.m
#define USER_MOTOR1_POWWR_SET_W         (0.4f)      // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)      // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)     // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)   // Hz/s

#elif (USER_MOTOR1 == Marathon_56H17T2011A)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (5.159403)
#define USER_MOTOR1_Rs_Ohm                 (7.924815)
#define USER_MOTOR1_Ls_d_H                 (0.03904648)
#define USER_MOTOR1_Ls_q_H                 (0.03904648)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8*0.8165*230.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (0.9941965)
#define USER_MOTOR1_RES_EST_CURRENT_A      (0.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (2.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (6.300017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz            (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0f)        // Hz

#define USER_MOTOR1_FREQ_MIN_Hz            (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (400.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (2.5f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.0f)          // A
#define USER_MOTOR1_OVER_CURRENT_A         (4.5f)          // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)         // Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)        // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Control initialized default value
#define USER_MOTOR1_TORQUE_SET_NM       (0.2f)          // N.m
#define USER_MOTOR1_POWWR_SET_W         (40.0f)         // W
#define USER_MOTOR1_IS_Q_SET_A          (1.5f)          // A
#define USER_MOTOR1_SPEED_SET_Hz        (40.0f)         // Hz
#define USER_MOTOR1_ACCEL_SET_Hzps      (20.0f)         // Hz/s

#elif (USER_MOTOR1 == Dayton_3N352C)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (2.428799)
#define USER_MOTOR1_Rs_Ohm                 (2.863202)
#define USER_MOTOR1_Ls_d_H                 (0.02391323)
#define USER_MOTOR1_Ls_q_H                 (0.02391323)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (3.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

//------------------------------------------------------------------
#elif (USER_MOTOR1 == my_pm_motor_1)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (NULL)
#define USER_MOTOR1_Ls_d_H                 (NULL)
#define USER_MOTOR1_Ls_q_H                 (NULL)
#define USER_MOTOR1_RATED_FLUX_VpHz        (NULL)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.5f)
#define USER_MOTOR1_MAX_CURRENT_A          (6.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)

// Number of lines on the motor's quadrature encoder
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)

#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)            // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (1.0f)            // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)           // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.5f)           // A

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)        // 60s/5ms

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (25.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.75f)      //
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)

#define USER_MOTOR1_PLL_KP_MAX             (6.75f)      //
#define USER_MOTOR1_PLL_KP_MIN             (0.75f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

#elif (USER_MOTOR1 == my_aci_motor_2)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (NULL)
#define USER_MOTOR1_Ls_d_H                 (NULL)
#define USER_MOTOR1_Ls_q_H                 (NULL)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165f * 230.0f / 60.0f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (0.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (5.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0f)

// Number of lines on the motor's quadrature encoder
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)

#define USER_MOTOR1_FREQ_MIN_Hz            (5.0f)           // Hz
#define USER_MOTOR1_FREQ_MAX_Hz            (300.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_Hz            (10.0f)          // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

//------------------------------------------------------------------------------
#else
#error No motor type specified
#endif


//! \brief Defines the maximum current slope for Id trajectory
//!
#define USER_M1_MAX_CURRENT_DELTA_A        (USER_MOTOR1_RES_EST_CURRENT_A / USER_M1_ISR_FREQ_Hz)


//! \brief Defines the maximum current slope for Id trajectory during power warp mode
//!
#define USER_M1_MAX_CURRENT_DELTA_PW_A    (0.3 * USER_MOTOR1_RES_EST_CURRENT_A / USER_M1_ISR_FREQ_Hz)


#ifndef USER_MOTOR1
#error Motor type is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_TYPE
#error The motor type is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Rr_Ohm
#error The rotor resistance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Rs_Ohm
#error The stator resistance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Ls_d_H
#error The direct stator inductance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Ls_q_H
#error The quadrature stator inductance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_RATED_FLUX_VpHz
#error The rated flux of motor is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_MAGNETIZING_CURRENT_A
#error The magnetizing current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_RES_EST_CURRENT_A
#error The resistance estimation current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_IND_EST_CURRENT_A
#error The inductance estimation current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_MAX_CURRENT_A
#error The maximum current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_FLUX_EXC_FREQ_Hz
#error The flux excitation frequency is not defined in user_mtr1.h
#endif

#if ((USER_M1_NUM_CURRENT_SENSORS < 2) || (USER_M1_NUM_CURRENT_SENSORS > 3))
#error The number of current sensors must be 2 or 3
#endif

#if (USER_M1_NUM_VOLTAGE_SENSORS != 3)
#error The number of voltage sensors must be 3
#endif

// **************************************************************************
// the Defines

//! \brief Defines the minimum ADC value for pot
#define USER_M1_POT_ADC_MIN         (200U)          // 0 < the value < 4096

//! \brief Defines the maximum ADC value for pot
#define USER_M1_POT_ADC_MAX         (4096U - 200U)  // 0 < the value < 4096

//! \brief Defines the speed conversion coefficient for pot
#define USER_M1_POT_SPEED_SF        USER_MOTOR1_FREQ_MAX_Hz / ((float32_t)(USER_M1_POT_ADC_MAX - USER_M1_POT_ADC_MIN))

//! \brief Defines the minimum frequency for pot
#define USER_M1_POT_SPEED_MIN_Hz    (USER_MOTOR1_FREQ_MAX_Hz * 0.1f)

//! \brief Defines the maximum frequency for pot
#define USER_M1_POT_SPEED_MAX_Hz    (USER_MOTOR1_FREQ_MAX_Hz * 0.5f)

//! \brief Defines the pot signal wait delay time
#define USER_M1_WAIT_TIME_SET       (500U)         // 0.5s/1ms

//! \brief Defines the minimum frequency for pulse input
#define USER_M1_SPEED_CAP_MIN_Hz        (15.0f)

//! \brief Defines the maximum frequency for pulse input
#define USER_M1_SPEED_CAP_MAX_Hz        (600.0f)

//! \brief Defines the pulse capture wait delay time
#define USER_M1_CAP_WAIT_TIME_SET       (125U)     // 0.125s/1ms (8Hz)

//! \brief Defines the switch signal wait delay time
#define USER_M1_SWITCH_WAIT_TIME_SET    (50U)      // 0.05s/1ms

// **************************************************************************
// the typedefs


// **************************************************************************
// the globals
extern USER_Params userParams_M1;


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

#endif // end of USER_MTR1_H definition

