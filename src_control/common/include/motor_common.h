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
//! \file  solutions/universal_motorcontrol_lab/common/include/motor_common.h
//! \brief  header file to be included in all labs
//!
//------------------------------------------------------------------------------

#ifndef _MOTOR_COMMON_H_
#define _MOTOR_COMMON_H_


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
//! \defgroup MOTOR COMMON
//! @{
//
//*****************************************************************************

// includes
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#include "userParams.h"
#include "est.h"

#include "clarke.h"
#include "filter_fo.h"
#include "ipark.h"
#include "park.h"
#include "pi.h"
#include "svgen.h"
#include "svgen_current.h"
#include "traj.h"
#include "mtpa.h"

#include "vs_freq.h"
#include "angle_gen.h"
#include "volt_recons.h"


#if defined(MOTOR1_ESMO)
#include "esmo.h"
#include "speedfr.h"
#endif  // MOTOR1_ESMO



#if defined(MOTOR1_DCLINKSS)    // Single shunt
#include "dclink_ss.h"
#endif // MOTOR1_DCLINKSS       // single shunt


// solutions
#if !defined(__TMS320C28XX_CLA__)
#include "user.h"
#include "hal.h"
#endif

#if defined(SFRA_ENABLE)
#include "sfra_settings.h"
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
#include "step_response.h"
#endif  // STEP_RP_EN

// Included files for test-infrastructure.
//
#if defined(SAFETY_ENABLE)
#include "sta_comm.h"
#include "sta_tests.h"
#include "sta_timer.h"
#include "sta_user.h"
#include "sta_peripheral.h"
#endif  //  SAFETY_ENABLE

//*****************************************************************************
#define M_OVER_VOLTAGE_BIT          0x0001    // DC Bus Over Voltage Fault
#define M_UNDER_VOLTAGE_BIT         0x0002    // DC Bus Under Voltage Fault
#define M_MOTOR_OVER_TEMPER_BIT     0x0004    // Motor over temperature Fault
#define M_MODULE_OVER_TEMPER_BIT    0x0008    // Module over temperature Fault

#define M_MODULE_OVER_CURRENT_BIT   0x0010    // Hardware Over Current Fault
#define M_OVER_PEAK_CURRENT_BIT     0x0020    // internal CMPSS Over Current Fault
#define M_MOTOR_OVER_LOAD_BIT       0x0040    // Over Load Error
#define M_MOTOR_LOST_PHASE_BIT      0x0080    // Motor Lost Phase

#define M_CURRENT_UNBALANCE_BIT     0x0100    // Motor Phase Current Unbalance
#define M_MOTOR_STALL_BIT           0x0200    // Motor Stall
#define M_STARTUP_FAILE_BIT         0x0400    // Startup failed
#define M_MOTOR_OVER_SPEED_BIT      0x0800    // Motor Over Speed

#define M_COM_LOST_BIT              0x1000    // Communication lost
#define M_DRIVER_FAILED_BIT         0x2000    // Gate driver
#define M_CURRENT_OFFSET_BIT        0x4000    // Current offsets
#define M_VOLTAGE_OFFSET_BIT        0x8000    // voltage offsets

#define M_MASK_ALL_FAULT_BITS       0x0000
#define M_ENABLE_ALL_FAULT_BITS     0xFFFF


// Convert the motor fault definitions to system fault definitions
#define M_HARDWARE_BITS             M_CURRENT_OFFSET_BIT                \
                                  + M_VOLTAGE_OFFSET_BIT                \
                                  + M_DRIVER_FAILED_BIT

#define M_COMMUNICATION_BITS        M_COM_LOST_BIT

#define M_TEMPERATURE_BITS          M_MOTOR_OVER_TEMPER_BIT            \
                                  + M_MODULE_OVER_TEMPER_BIT

#define M_OUVOLTAGE_BITS            M_OVER_VOLTAGE_BIT                  \
                                  + M_UNDER_VOLTAGE_BIT

#define M_STARTUP_FAILED_BITS       M_MOTOR_LOST_PHASE_BIT              \
                                  + M_STARTUP_FAILE_BIT                 \
                                  + M_MOTOR_STALL_BIT

#define M_RUN_FAILED_BITS           M_MOTOR_OVER_LOAD_BIT               \
                                  + M_CURRENT_UNBALANCE_BIT             \
                                  + M_MOTOR_OVER_SPEED_BIT

#define M_OVERCURRENT_BITS          M_MODULE_OVER_CURRENT_BIT           \
                                  + M_OVER_PEAK_CURRENT_BIT


// Block all fault protection except current, voltage and temperature faults
#define MTR_FAULT_OV_BRAKE             M_OVER_VOLTAGE_BIT

#define MTR_FAULT_ENABLE_OC             M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT

#define MTR_FAULT_ENABLE_OC_OUV         M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT

// Enable the related faults protection
#define MTR_FAULT_ENABLE_OC_HW          M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_MOTOR_STALL_BIT                      \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT

// Enable the related faults protection
#define MTR_FAULT_ENABLE_OC_HW_RUN      M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_MOTOR_OVER_LOAD_BIT                  \
                                      + M_MOTOR_OVER_SPEED_BIT                 \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT


// Enable all fault protection
#define MTR_FAULT_ENABLE_ALL            M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MOTOR_OVER_TEMPER_BIT                \
                                      + M_MODULE_OVER_TEMPER_BIT               \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_MOTOR_OVER_LOAD_BIT                  \
                                      + M_MOTOR_LOST_PHASE_BIT                 \
                                      + M_CURRENT_UNBALANCE_BIT                \
                                      + M_MOTOR_STALL_BIT                      \
                                      + M_STARTUP_FAILE_BIT                    \
                                      + M_MOTOR_OVER_SPEED_BIT                 \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT


// Clear all fault protection except over/under voltage and offset error
#define MTR_FAULT_CLEAR                 M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MOTOR_OVER_TEMPER_BIT                \
                                      + M_MODULE_OVER_TEMPER_BIT

#define MTR_FAULT_DISABLE_ALL           0x0000

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
#define MTR1_FAULT_MASK_SET             MTR_FAULT_ENABLE_OC
#else
//#define MTR1_FAULT_MASK_SET           MTR_FAULT_ENABLE_OC_OUV
//#define MTR1_FAULT_MASK_SET           MTR_FAULT_ENABLE_OC_HW
#define MTR1_FAULT_MASK_SET             MTR_FAULT_ENABLE_OC_HW_RUN
//#define MTR1_FAULT_MASK_SET           MTR_FAULT_ENABLE_ALL
#endif

#define MOTOR_CTRL_STATUS_CLR       0x00E0
#define MOTOR_CTRL_STATUS_MASK      0x001F

#define MOTOR_PRMS_UPDATE_MASK      0x0080
#define MOTOR_PRMS_UPDATE_DONE      0x0080
#define MOTOR_PRMS_UPDATE_IDLE      0x0000

#define MOTOR_RSONLINE_RUN_MASK     0x0040
#define MOTOR_RSONLINE_RUN_ON       0x0040
#define MOTOR_RSONLINE_RUN_OFF      0x0000

#define MOTOR_FWC_RUN_MASK          0x0020
#define MOTOR_FWC_RUN_ON            0x0020
#define MOTOR_FWC_RUN_OFF           0x0020

//------------------------------------------------------------------------------
// 6 motor models. If add more motor models, need to add the parameters in
//                    aMotorPrms[] in blower_system.c file
typedef enum
{
    M1_Teknic_M2310PL    = 0,       // Teknic_M2310PLN04K
    M2_Estun_04APB22     = 1,       // Estun_EMJ_04APB22
    M3_LACFAN_ZKSN_750   = 2,       // LACFAN_ZKSN_750
    M4_Marathon_N56PNRA  = 3,       // Marathon_N56PNRA10102
    M5_VFWM_DDLG8p4kg    = 4,       // VFWMPM_DDLG8p4kg
    M6_VFWM_WLBW550Q20   = 5,       //
    M7_GMCC_KSK89D53U    = 6,       //
    M8_IDTEST_MOTOR01    = 7,       //
    M9_IDTEST_MOTOR02    = 8,       //
    M10_IDTEST_MOTOR03   = 9,       //
    M11_IDTEST_MOTOR04   = 10,      //
    M12_IDTEST_MOTOR05   = 11,      //
    MOTOR_MAX_NUM        = 4
}MOTOR_Model_e;


//! \brief Enumeration for the device configuration
typedef enum
{
    PRJ_NON_SYSCONFIG  = 0,         //!< Without using Sysconfig
    PRJ_DEV_SYSCONFIG  = 1,         //!< Multiple Sysconfig files only for device configuration
    PRJ_ALL_SYSCONFIG  = 2          //!< A single Sysconfig file for device and project configuration
} Project_Config_e;


//! \brief Enumeration for the kit boards
typedef enum
{
    BOARD_WMINVBRD_REV1P0  = 1,     //!< the board is WMINVBRD_REV1P0, OK,
    BOARD_HVMTRPFC_REV1P1  = 2,     //!< the board is HVMTRPFC_REV1P1, OK, in MCSDK
    BOARD_BSXL8323RH_REVB  = 3,     //!< the board is BOOSTXL_8323RH,  OK, in MCSDK
    BOARD_DRV8329AEVM_REVA = 4,     //!< the board is DRV8312KIT_REVD, OK
    BOARD_TIDSMPFC_REV3P2  = 5,     //!< the board is TIDSMPFC_REV3P2, OK,
    BOARD_DRV8353RH_EVM    = 6      //!< the board is DRV8353RH_EVM, OK
} Board_Kit_e;


//! \brief Enumeration for the applications
typedef enum
{
    APP_SMDC_WASHER    = 0,         //!<
    APP_SMDC_DRYER     = 1,         //!<
    APP_SMPFC_VFAC     = 2,         //!<
    APP_SMDC_HEATPUMP  = 3,         //!<
    APP_SMDC_INDFAN    = 4,         //!<
    APP_SMDC_ACFAN     = 5          //!<
} Application_Type_e;

//! \brief Enumeration for the estimator mode
//  0 -ESTIMATOR_MODE_FAST
//  1 -ESTIMATOR_MODE_ESMO
#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0,             //!< FAST estimator
    ESTIMATOR_MODE_ESMO  = 1              //!< ESMO estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_FAST)
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0              //!< FAST estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ESMO)
typedef enum
{
    ESTIMATOR_MODE_ESMO  = 1              //!< ESMO estimator
} ESTIMATOR_Mode_e;
#else
#error Not select a right estimator for this project
#endif  // MOTOR1_FAST | MOTOR1_ESMO

typedef enum
{
    FAST_TYPE_SOFTLIB_FLASH  = 0,   //!< the FAST software library and run in Flash
    FAST_TYPE_ROMLIB_FLASH   = 1,   //!< the FAST ROM library and run in Flash
    FAST_TYPE_ROMLIB_RAM     = 2,   //!< the FAST ROM library and run in RAM
    FAST_TYPE_NONFAST_FLASH  = 3,   //!< the non FAST and run in Flash
    FAST_TYPE_NONFAST_RAM    = 4    //!< the non FAST and run in RAM
} FASTLIB_Type_e;

#if defined(CMD_POT_EN)
typedef struct _CMDPOT_Vars_t_
{
    float32_t speedSet_Hz;
    float32_t speedMin_Hz;
    float32_t speedMax_Hz;
    float32_t speedConv_sf;
    uint16_t adcMin;
    uint16_t waitTimeCnt;
    uint16_t waitTimeSet;
    bool    flagCmdRun;
    bool    flagEnableCmd;
} CMDPOT_Vars_t;
#endif  // CMD_POT_EN

#if defined(CMD_CAP_EN)
typedef struct _CMDCAP_Vars_t_
{
    float32_t speedRef_Hz;
    float32_t freqScaler;       // Scaler converting 1/N CPU cycles
    float32_t speedMeas_Hz;
    float32_t speedSet_Hz;
    float32_t speedMin_Hz;
    float32_t speedMax_Hz;
    uint32_t  timeStamp;
    uint16_t  waitTimeCnt;
    uint16_t  waitTimeSet;
    bool      flagCmdRun;
    bool      flagEnableCmd;
} CMDCAP_Vars_t;
#endif // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
typedef struct _CMDSWITCH_Vars_t_
{
    uint16_t delayTimeSet;
    uint16_t highTimeCnt;
    uint16_t lowTimeCnt;
    bool     flagCmdRun;
    bool     flagEnablCmd;
} CMDSWITCH_Vars_t;
#endif  // CMD_SWITCH_EN


//! \brief typedefs for the control types
typedef struct _CONTROL_TYPES_BITS_
{             // bits  description
    Estimator_Type_e        estimatorType:2;    // 0~1
    CurrentShunt_Type_e     currentSenType:2;   // 2~3
    SVM_Mode_e              svmMode:2;          // 4~5
    Braking_Mode_e          brakingMode:2;      // 6~7
    Operation_Mode_e        operationMode:3;    // 8~10
    ESTIMATOR_Mode_e        estimatorMode:1;    // 11
    FlyingStart_Mode_e      flyingStartMode:1;  // 12
    RsOnline_Mode_e         RsOnlineMode:1;     // 13
    CurrentSense_Dir_e      currentSenDir:1;    // 14
    Startup_Mode_e          startupMode:1;      // 15
} CONTROL_TYPES_BITS;

typedef union _CONTROL_TYPES_t_
{
    uint16_t            all;
    CONTROL_TYPES_BITS  bit;
}CONTROL_TYPES_t;


//! \brief typedefs for the control functions
typedef struct _CONTROL_FUNCS_BITS_
{             // bits  description
    uint16_t enableRsRecalculate:1;         // 0
    uint16_t enableRsOnline:1;              // 1
    uint16_t enableMTPA:1;                  // 2
    uint16_t enableFieldWeakening:1;        // 3
    uint16_t enableDecoupleCtrl:1;          // 4
    uint16_t enablePirsControl:1;           // 5
    uint16_t enableVibComp:1;               // 6
    uint16_t enableForceAngle:1;            // 7
    uint16_t enableBrakingStop:1;           // 8
    uint16_t enableFlyingStart:1;           // 9
    uint16_t enableSSIPD:1;                 // 10
    uint16_t enableIPDHFI:1;                // 11
    uint16_t enableLsUpdate:1;              // 12
    uint16_t enablePrmsUpdateSave:1;        // 13
    uint16_t enableOverModulation:1;        // 14
    uint16_t enableReserved:1;              // 15
} CONTROL_FUNCS_BITS;


typedef union _CONTROL_FUNCS_t_
{
    uint16_t            all;
    CONTROL_FUNCS_BITS  bit;
}CONTROL_FUNCS_t;


//! \brief typedefs for the fault
typedef struct _FAULT_MTR_BITS_
{             // bits  description
    uint16_t overVoltage:1;         // 0  DC Bus Over Voltage Fault
    uint16_t underVoltage:1;        // 1  DC Bus Under Voltage Fault
    uint16_t motorOverTemp:1;       // 2  Motor over temperature Fault
    uint16_t moduleOverTemp:1;      // 3  Power module over temperature Fault

    uint16_t moduleOverCurrent:1;   // 4  Hardware Over Current Fault Flag
    uint16_t overPeakCurrent:1;     // 5  internal CMPSS Over Current Fault Flag
    uint16_t overLoad:1;            // 6  Over Load Error
    uint16_t motorLostPhase:1;      // 7  Motor Lost Phase

    uint16_t currentUnbalance:1;    // 8  Motor Phase Current imbalance
    uint16_t motorStall:1;          // 9  Motor Stall
    uint16_t startupFailed:1;       // 10 Startup failed
    uint16_t overSpeed:1;           // 11 Motor Over Speed

    uint16_t communication:1;       // 12 Communication lost
    uint16_t gateDriver:1;          // 13 Gate driver
    uint16_t currentOffset:1;       // 14 Current offset check
    uint16_t voltageOffset:1;       // 15 voltage offset check
} FAULT_MTR_BITS;


typedef union _FAULT_MTR_REG_t
{
    uint16_t        all;
    FAULT_MTR_BITS  bit;
}FAULT_MTR_REG_t;


typedef struct _MOTOR_SetVars_t_
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

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
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

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    float32_t V_decoup_sf;              // P-127
#endif  // _HSWFREQ_EN & MOTOR1_DECOUP

    CONTROL_TYPES_t controlTypes;       // P-128
    CONTROL_FUNCS_t controlFuncs;       // P-129
#endif  // _HSWFREQ_EN & GUI_SCI_EN

    FAULT_MTR_REG_t faultMtrMask;       // P-130

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    int16_t  overTemperatureMotor;      // P-131
    int16_t  overTemperatureModule;     // P-132
#endif  // _HSWFREQ_EN & GUI_SCI_EN

#if !defined(_HSWFREQ_EN)
    uint16_t RsOnlineWaitTimeSet;       // P-133
    uint16_t RsOnlineWorkTimeSet;       // P-134
#endif  // _HSWFREQ_EN

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    uint16_t sampleTrigDelay;           // P-135
    uint16_t dsscMinDuration;           // P-136
    uint16_t dsscSampleDelay;           // P-137
#endif  // _HSWFREQ_EN & GUI_SCI_EN

    uint16_t startupTimeDelay;          // P-138
    uint16_t stopWaitTimeSet;           // P-139

    uint16_t restartWaitTimeSet;        // P-140
    uint16_t restartTimesSet;           // P-141

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
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

    uint16_t bootChargeTimeSet;

    uint16_t dataPrmsLength;
    uint16_t dataPrmsIndexPrev;
    uint16_t dataPrmsIndexUse;
    uint16_t dataPrmsIndexRead;
    uint16_t dataPrmsIndexSave;
    uint16_t dataPrmsNumValue;
    uint16_t dataPrmsFlashStatus;
    uint16_t dataPrmsFlashFlag;

    uint16_t codeUpdateLength;
    uint16_t codeUpdateIndex;
    uint16_t codeUpdateStatus;
    uint16_t codeUpdateFlag;
#endif  // _HSWFREQ_EN & GUI_SCI_EN

    uint16_t  dacCMPValH;
    uint16_t  dacCMPValL;

    float32_t Kp_spd_set;               // calculated
    float32_t Ki_spd_set;               // calculated

    float32_t Kp_Iq_set;                // calculated
    float32_t Ki_Iq_set;                // calculated

    float32_t Kp_Id_set;                // calculated
    float32_t Ki_Id_set;                // calculated

    float32_t Kp_spd_slope_sf;          // calculated
    float32_t Ki_spd_slope_sf;          // calculated

    float32_t Kp_spd;
    float32_t Ki_spd;

    float32_t Kp_Iq;
    float32_t Ki_Iq;

    float32_t Kp_Id;
    float32_t Ki_Id;

#if defined(MOTOR1_POWCTRL)
    float32_t Kp_pow_set;
    float32_t Ki_pow_set;

    float32_t Kp_pow;
    float32_t Ki_pow;
#endif  // MOTOR1_POWCTRL

    float32_t maxCurrent_A;         // the maximum/rated current value of the motor, A
    float32_t IdInj_A;              // injection current on d-axis
    float32_t IqInj_A;              // injection current on q-axis

    float32_t Ls_d_comp_H;
    float32_t Ls_q_comp_H;

    float32_t voltage_sf;           // defines the voltage scale factor for the system
    float32_t current_sf;           // defines the current scale factor for the system
    float32_t currentInv_sf;        // defines the current invert coefficient

    float32_t maxAccel_Hzps;        // defines the maximum acceleration for the speed profiles, Hz/sec
    float32_t pwmPeriod_usec;       // defines the Pulse Width Modulation(PWM) period, usec
    float32_t ctrlPeriod_sec;       // defines the controller execution period, sec
    float32_t ctrlFreq_Hz;          // defines the controller frequency, Hz
    float32_t estFreq_Hz;           // defines the estimator frequency, Hz

#if !defined(_HSWFREQ_EN) && defined(GUI_SCI_EN)
    float32_t hz2Rpm_sf;            // defines convert Hz to rpm coefficient

    float32_t windingTemp_sfInv;    // Temperature coefficient of the material
    float32_t windingOverTempMax;   // motor over temperature threshold
    float32_t windingNormTempMin;   // motor temperature is normal threshold

    uint16_t  motorOverTempTimeSet; // motor over temperature time threshold

    int16_t   tempModuleCoef;       // module temperature coefficient
    int16_t   tempModuleOffset;     // module temperature offset

    int16_t   tempMotorCoef;        // motor temperature coefficient
    int16_t   tempMotorOffset;      // motor temperature offset
#endif  // _HSWFREQ_EN & GUI_SCI_EN
} MOTOR_SetVars_t;


//! \brief Defines the MOTOR_SetVars_t handle
//!
typedef struct _MOTOR_SetVars_t_ *MOTORSETS_Handle;

extern volatile MOTORSETS_Handle motorSetHandle_M1;
extern MOTOR_SetVars_t motorSetVars_M1;


typedef struct _MOTOR_CtrlVars_t_
{
    bool flagCmdRpmOrHz;                // true-rpm, false-Hz
    bool flagEnableGuiControl;          // 1-enable control the motor with GUI
    bool flagEnableLoadPrms;            // 1-enable load parameters from flash
    bool flagEnableUpdatePrms;          // 1-enable update parameters
    bool flagStatusLoadPrms;            // 0-Failed, 1-Success
    bool flagStatusUpdatePrms;          // 0-Failed, 1-Success

    bool flagEnableRunMotor;            // 0-stop, 1-run
    bool flagIdentifyStatus;
    bool flagEnableSpeedCtrl;
    bool flagEnableTorqueCtrl;
    bool flagEnablePowerCtrl;

    MOTOR_CtrlMode_e controlCmdRecv;    // received running command
    MOTOR_CtrlMode_e controlCmdUse;     // used running command
    MOTOR_CtrlMode_e controlCmdPrev;    // previous running command
    MOTOR_Status_e   controlStatus;     // running command
    MOTOR_Model_e    motorModel;        // the motor model
    uint16_t         motorCtrlStates;   // motor control states

    CONTROL_TYPES_t controlTypes;       // Estimator, CurrentShunt, SVM... mode
    CONTROL_FUNCS_t controlFuncs;       // RsRecalculate, RsOnline, MTPA... enable

    FAULT_MTR_REG_t faultMotor;         // fault

    int16_t   temperatureModule;        // module temperature
    int16_t   temperatureMotor;         // motor temperature

    uint16_t  accelerationSetTime;      // Acceleration time
    uint16_t  updatePrmsDelayTime;      // Update control parameters wait time

    float32_t IqSet_A;                  // Torque current setting value, A
    float32_t torqueSet_Nm;             // Torque setting value, N.m
    float32_t powerSet_W;               // Power setting value, W

    float32_t speedRef_Hz;              // Speed target value, Hz
    float32_t accelerationMax_Hzps;     // Acceleration value, Hz/s

    float32_t speedSet_Hz;              // Speed target value, Hz
    float32_t accelerationSet_Hzps;     // Acceleration value, Hz/s

    float32_t speedSet_rpm;             // Speed target value, rpm
    float32_t accelerationSet_rpmps;    // Acceleration, rpm/s

    float32_t rpm2Hz_sf;                // Convert rpm to Hz
    float32_t hz2Rpm_sf;                // convert Hz to rpm
    float32_t windingTempRs;            // motor temperature calculated by RsOnline
    float32_t windingTempFilter;        // filtering motor temperature
}MOTOR_CtrlVars_t;


//! \brief Defines the MOTOR_CtrlVars_t handle
//!
typedef struct _MOTOR_CtrlVars_t_ *MOTORCTRL_Handle;


extern MOTOR_CtrlVars_t motorCtrlVars_M1;


typedef struct _MOTOR_Vars_t_
{
    bool flagEnableRunAndIdentify;
    bool flagRunIdentAndOnLine;
    bool flagMotorIdentified;

    bool flagSetupController;
    bool flagEnableSpeedCtrl;
    bool flagEnableCurrentCtrl;

    bool enableSpeedCtrl;
    bool enableCurrentCtrl;

    bool flagEnableRestart;
    bool flagEnableAlignment;
    bool flagEnableOffsetCalc;

    bool flagEnableRsRecalc;
    bool flagEnableRsOnLine;
    bool flagEnableMTPA;
    bool flagEnableFWC;
    bool flagEnableDecoupleCtrl;
    bool flagEnablePirsControl;
    bool flagEnableVibComp;
    bool flagEnableForceAngle;
    bool flagEnableBrakingStop;
    bool flagEnableFlyingStart;
    bool flagEnableSSIPD;
    bool flagEnableIPDHFI;
    bool flagEnableLsUpdate;
    bool flagEnableOVM;

    bool flagStateFlyingStart;
    bool flagStartRsOnLine;
    bool flagBrakeDone;
    bool flagStartFWC;

    bool flagClearFaults;
    bool flagVIrmsCal;

    bool flagUpdateMTPAParams;
    bool flagPhaseAdjustEnable;

    bool flagInitializeDone;
    bool flagEnableTuneController;      // true-enable, false-disable

#if defined(MOTOR1_FAST)

#if defined(MOTOR1_FILTERIS)
    bool flagEnableFilterIs;
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    bool flagEnableFilterVs;
#endif  // MOTOR1_FILTERVS
    bool flagEnableMotorIdentify;
    bool flagEnablePowerWarp;
    bool flagBypassLockRotor;
#endif  // MOTOR1_FAST

    FAULT_MTR_REG_t faultMtrNow;
    FAULT_MTR_REG_t faultMtrUse;
    FAULT_MTR_REG_t faultMtrMask;
    FAULT_MTR_REG_t faultMtrPrev;

    MOTOR_Status_e controlStatus;

#if defined(MOTOR1_FAST)
    EST_State_e estState;
    EST_Traj_State_e trajState;
#endif  // MOTOR1_FAST

    Estimator_Type_e  estimatorType;
    CurrentShunt_Type_e currentSenType;
    SVM_Mode_e svmMode;
    Braking_Mode_e brakingMode;
    Operation_Mode_e operationMode;
    ESTIMATOR_Mode_e estimatorMode;
    FlyingStart_Mode_e flyingStartMode;
    RsOnline_Mode_e RsOnlineMode;
    CurrentSense_Dir_e CurrentSenDir;
    Startup_Mode_e startupMode;

    uint16_t overCurrentTimesCnt;
    uint16_t overVoltageTimeCnt;
    uint16_t underVoltageTimeCnt;

    uint16_t motorStallTimeCnt;
    uint16_t startupFailTimeCnt;

    uint16_t bootChargeTimeCnt;
    uint16_t stopWaitTimeCnt;
    uint16_t restartTimesCnt;
    uint16_t startSumTimesCnt;

    uint16_t overSpeedTimeCnt;
    uint16_t overLoadTimeCnt;
    uint16_t unbalanceTimeCnt;
    uint16_t lostPhaseTimeCnt;

    uint16_t  motorOverTempTimeCnt;

    uint16_t VIrmsIsrSet;
    uint16_t VIrmsIsrCnt;

#ifdef BRAKE_ENABLE
    uint16_t brakingTimeDelay;
    uint16_t brakingTimeCnt;
#endif  // BRAKE_ENABLE

    uint16_t stateRunTimeCnt;
    uint16_t counterSpeed;
    uint16_t numCtrlTicksPerSpeedTick;

#if defined(MOTOR1_POWCTRL)
    bool flagEnablePowerCtrl;
    bool enablePowerCtrl;

    uint16_t counterPower;
    uint16_t numCtrlTicksPerPowerTick;
#endif  // MOTOR1_POWCTRL

#if defined(MOTOR1_FAST)
    uint16_t RsOnlineTimeCnt;
#endif  // MOTOR1_FAST

#if defined(SAFETY_ENABLE)
    uint16_t safetyEnableFlag;
    uint16_t safetyFaultFlag;
    uint16_t safetyStatus;
#endif  // SAFETY_ENABLE

    uint32_t ISRCount;

    uint16_t  numPolePairs;         // the number of pole pairs for the motor
    MOTOR_Type_e  motor_type;       // the motor type

    HAL_ADCData_t adcData;
    HAL_PWMData_t pwmData;

    float32_t Rs_Ohm;               // the stator stator resistance of the motor, Ohm
    float32_t Ls_d_H;               // the direct stator inductance of the motor, H
    float32_t Ls_q_H;               // the quadrature stator inductance of the motor, H
    float32_t flux_VpHz;            // the rated flux of the motor, Hz
    float32_t flux_Wb;
    float32_t Rr_Ohm;               // the rotor resistance of the motor, Ohm
    float32_t magneticCurrent_A;

    float32_t RoverL_rps;
    float32_t RsOnLine_Ohm;

    float32_t direction;                    // 1.0f->forward, -1.0f->reserve

    float32_t accelerationMax_Hzps;
    float32_t accelerationSet_Hzps;

    float32_t speed_rpm;                    // CVA-10, Speed feedback value, rpm
    float32_t speedAbs_Hz;
    float32_t speedFilter_Hz;

    float32_t speedSet_Hz;                   // Speed target value, Hz
    float32_t speedRef_Hz;                  // Speed target value, Hz
    float32_t speed_int_Hz;                 // Speed reference value, Hz

    // the speed from PLL module
    float32_t speedPLL_Hz;

    // the speed from EST module
    float32_t speedEST_Hz;

    float32_t speed_Hz;                     //  Speed feedback value, Hz

    // the rotor angle from FOC modules
    float32_t angleFOC_rad;

    // the rotor angle from EST modules
    float32_t angleEST_rad;

    // the rotor angle from PLL modules
    float32_t anglePLL_rad;

    // the rotor angle from Generator modules
    float32_t angleGen_rad;

    float32_t anglePLLDelayed_sf;
    float32_t angleESTDelayed_sf;

    // the rotor angle from FOC modules
    float32_t angleFOCAdj_rad;

    // the rotor angle from FOC modules
    float32_t anglePhaseAdj_rad;

    // the rotor angle for braking
    float32_t angleBrake_rad;

    float32_t angleFWC_rad;

    float32_t angleCurrent_rad;

#if defined(MOTOR1_FAST)
    // the rotor angle compensation value
    float32_t angleESTCOMP_rad;
#endif  // MOTOR1_FAST || MOTOR2_FAST

    float32_t Is_A;
    float32_t Vs_V;
    float32_t maxVsMag_pu;
    float32_t VsRef_pu;
    float32_t VsRef_V;
    float32_t VsMax_V;              // the maximum stator voltage magnitude, V
    float32_t oneOverDcBus_invV;    // the DC Bus inverse, 1/V

    float32_t IdRated_A;
    float32_t IsRef_A;
    float32_t IsSet_A;
    float32_t Is_ffwd_A;
    float32_t IdInj_A;
    float32_t IqInj_A;
    float32_t frswPos_sf;
    float32_t torqueRef_Nm;

    float32_t unbalanceRatio;
    float32_t VIrmsIsrScale;
    float32_t IrmsCalSF;
    float32_t power_sf;

    float32_t Irms_A[3];
    float32_t IrmsCalSum[3];
    float32_t IrmsPrdSum[3];

    float32_t torque_Nm;

    float32_t powerActive_W;
    float32_t powerMotor_W;
#if defined(MOTOR1_POWCTRL)
    float32_t powerRef_W;
    float32_t IsPower_A;
#endif  // MOTOR1_POWCTRL

#if defined(MOTOR1_ESMO)
    // the rotor angle from SMO modules
#if defined(ESMO_DEBUG)
    float32_t angleSMO_rad;
#endif  //ESMO_DEBUG

    // the rotor angle delay compensation value
    float32_t anglePLLComp_rad;
#endif  // MOTOR1_ESMO

    MATH_Vec2 Iab_A;                // the alpha/beta current values, A

    // d&q axis current are converter from 3-phase sampling input current of motor
    MATH_Vec2 Idq_in_A;             // CV2-4,5

    // the reference current on d&q rotation axis
    MATH_Vec2 IdqRef_A;             // CV2-2,3

    // the reference output current on d&q rotation axis
    MATH_Vec2 Idq_out_A;

    MATH_Vec2 Vab_V;                // the alpha/beta voltage values, V

    MATH_Vec2 Vdq_V;                // the d/q axis voltage values, V

    MATH_Vec2 Eab_V;                // the alpha/beta back EMF voltage values, V

    MATH_Vec2 Vab_out_V;            // the output control voltage on alpha&beta axis

    MATH_Vec2 Vdq_out_V;            // the output control voltage on d&q axis

    MATH_Vec2 Vdq_ffwd_V;           // the output offset voltage on d&q axis

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    //!< the reference current on d&q rotation axis
    MATH_Vec2 Idq_set_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    // the handle for the hardware abstraction layer to motor control
    MOTORCTRL_Handle motorCtrlHandle;

    MOTORSETS_Handle motorSetsHandle;

    userParams_Handle userParamsHandle;

    HAL_MTR_Handle halMtrHandle;

#if defined(MOTOR1_FAST)
    EST_InputData_t estInputData;

    EST_OutputData_t estOutputData;

    // the handle for the estimator
    EST_Handle    estHandle;

    // the handle for the voltage Clarke transform
    CLARKE_Handle clarkeHandle_V;
#endif  // MOTOR1_FAST || MOTOR2_FAST

    // the handle for the current Clarke transform
    CLARKE_Handle clarkeHandle_I;

    // the handle for the inverse Park transform
    IPARK_Handle  iparkHandle_V;

    // the handle for the Park object
    PARK_Handle   parkHandle_I;

    // the handle for the Park object
    PARK_Handle   parkHandle_V;

    // the handle for the Id PI controller
    PI_Handle     piHandle_Id;

    // the handle for the Iq PI controller
    PI_Handle     piHandle_Iq;

    // the handle for the speed PI controller
    PI_Handle     piHandle_spd;

#if defined(MOTOR1_POWCTRL)
    // the handle for the power PI controller
    PI_Handle     piHandle_pow;
#endif  // MOTOR1_POWCTRL

    // the handle for the speed reference trajectory
    TRAJ_Handle  trajHandle_spd;

    // the handle for the space vector generator
    SVGEN_Handle  svgenHandle;

#if defined(MOTOR1_DCLINKSS)    // Single shunt
    //!< the handle for single-shunt current reconstruction
    DCLINK_SS_Handle dclinkHandle;
#endif  // MOTOR1_DCLINKSS      // single shunt

#if defined(MOTOR1_ESMO)
    //!< the handle for the speedfr object
    SPDFR_Handle spdfrHandle;

    //!< the handle for the esmo object
    ESMO_Handle esmoHandle;
#endif  // MOTOR1_ESMO

#if defined(MOTOR1_FILTERIS)
    // the current filter pole location, rad/sec
    float32_t filterIsPole_rps;

    // first order current filter handle
    FILTER_FO_Handle filterHandle_Is[3];

    //!< the current values
    MATH_Vec3 adcIs_A;
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    // the voltage filter pole location, rad/sec
    float32_t filterVsPole_rps;

    // first order voltage filter handle
    FILTER_FO_Handle filterHandle_Vs[3];

    //!< the voltage values
    MATH_Vec3 adcVs_V;
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // the space vector generator current object
    SVGENCURRENT_Handle svgencurrentHandle;
    MATH_Vec3 adcDataPrev;
    MATH_Vec3 pwmDataPrev;
    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle;
    SVGENCURRENT_VmidShunt_e midVolShunt;
#endif  // MOTOR1_OVM

#if defined(MOTOR1_FWC)
    // the handle for the fwc PI controller
    PI_Handle    piHandle_fwc;
#endif  // MOTOR1_FWC || MOTOR2_FWC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
    defined(MOTOR1_ESMO) || defined(MOTOR1_DCLINKSS)
    //!< the handles for Angle Generate for open loop control
    ANGLE_GEN_Handle angleGenHandle;
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT

#if defined(MOTOR1_MTPA)
    //!< the handle for the Maximum torque per ampere (MTPA)
    MTPA_Handle  mtpaHandle;

    float32_t angleMTPA_rad;
    float32_t mtpaKconst;
    float32_t LsOnline_d_H;
    float32_t LsOnline_q_H;
    float32_t fluxOnline_Wb;
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_VOLRECT)
    //!< the handle for the voltage reconstruct
    VOLREC_Handle volrecHandle;
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    //!< the handles for Vs per Freq for open loop control
    VS_FREQ_Handle VsFreqHandle;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(CMD_POT_EN)
    CMDPOT_Vars_t cmdPot;
#endif  // CMD_POT_EN

#if defined(CMD_CAP_EN)
    CMDCAP_Vars_t cmdCAP;
#endif // CMD_CAP_EN
}MOTOR_Vars_t;


//! \brief Defines the MOTOR_Vars_t handle
//!
typedef struct _MOTOR_Vars_t_ *MOTOR_Handle;

extern volatile MOTOR_Handle motorHandle_M1;
extern MOTOR_Vars_t motorVars_M1;

#if defined(SFRA_ENABLE)
extern float32_t   sfraNoiseId;
extern float32_t   sfraNoiseIq;
extern float32_t   sfraNoiseSpd;
extern float32_t   sfraNoiseOut;
extern float32_t   sfraNoiseFdb;
extern SFRA_TEST_e sfraTestLoop;
extern bool        sfraCollectStart;
#endif  // SFRA_ENABLE


#if defined(BENCHMARK_TEST)
#define SPEED_RECORD_INDEX_NUM  64

typedef struct _BMTEST_Vars_t_
{
    float32_t speedRef_Hz;
    float32_t speedMax_Hz;
    float32_t speedMin_Hz;
    float32_t speedDelta_Hz;

    float32_t speedRef_rpm;
    float32_t speedMax_rpm;
    float32_t speedMin_rpm;
    float32_t speedDelta_rpm;

    float32_t speedBuff_Hz[SPEED_RECORD_INDEX_NUM];

    uint16_t recordDataCount;

    uint16_t recordTicksCount;
    uint16_t recordTicksSet;

    bool flagEnableRecord;
    bool flagResetRecord;
}BMTEST_Vars_t;

extern BMTEST_Vars_t bmarkTestVars;

extern void recordSpeedData(MOTOR_Handle handle);
#endif  // BENCHMARK_TEST
//*****************************************************************************
// the function prototypes

//! \brief calculate motor over current threshold
//! \param[in]  handle   The motor control handle
extern void calcMotorOverCurrentThreshold(MOTOR_Handle handle);

//! \brief checks motor faults
//! \param[in]  handle   The motor control handle
extern void checkMotorFaults(MOTOR_Handle handle);

//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
extern void setupClarke_I(CLARKE_Handle handle, const uint16_t numCurrentSensors);

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
extern void setupClarke_V(CLARKE_Handle handle, const uint16_t numVoltageSensors);

//! \brief  Update the controllers
static inline void updateControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

#if defined(MOTOR1_FAST)
    if((obj->controlStatus == MOTOR_CTRL_RUN) && (obj->flagMotorIdentified == true))
    {
        // update the Id controller
        PI_setGains(obj->piHandle_Id, objSets->Kp_Id, objSets->Ki_Id);

        // update the Iq controller
        PI_setGains(obj->piHandle_Iq, objSets->Kp_Iq, objSets->Ki_Iq);

        // update the speed controller
        PI_setGains(obj->piHandle_spd, objSets->Kp_spd, objSets->Ki_spd);

#if defined(MOTOR1_POWCTRL)
        // update the power controller
        PI_setGains(obj->piHandle_pow, objSets->Kp_pow, objSets->Ki_pow);
#endif  // MOTOR1_POWCTRL
    }
#else   // !MOTOR1_FAST
    if(obj->controlStatus == MOTOR_CTRL_RUN)
    {
        // update the Id controller
        PI_setGains(obj->piHandle_Id, objSets->Kp_Id, objSets->Ki_Id);

        // update the Iq controller
        PI_setGains(obj->piHandle_Iq, objSets->Kp_Iq, objSets->Ki_Iq);

        // update the speed controller
        PI_setGains(obj->piHandle_spd, objSets->Kp_spd, objSets->Ki_spd);

#if defined(MOTOR1_POWCTRL)
        // update the power controller
        PI_setGains(obj->piHandle_pow, objSets->Kp_pow, objSets->Ki_pow);
#endif  // MOTOR1_POWCTRL
    }
#endif  // !MOTOR1_FAST
}


//! \brief  Get the controllers Parameters
static inline void getControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // Get the Id controller parameters
    objSets->Kp_Id = PI_getKp(obj->piHandle_Id);
    objSets->Ki_Id = PI_getKi(obj->piHandle_Id);

    // Get the Iq controller parameters
    objSets->Kp_Iq = PI_getKp(obj->piHandle_Iq);
    objSets->Ki_Iq = PI_getKi(obj->piHandle_Iq);

    // Get the speed controller parameters
    objSets->Kp_spd = PI_getKp(obj->piHandle_spd);
    objSets->Ki_spd = PI_getKi(obj->piHandle_spd);

    objSets->Kp_Id_set = objSets->Kp_Id;
    objSets->Ki_Id_set = objSets->Ki_Id;

    objSets->Kp_Iq_set = objSets->Kp_Iq;
    objSets->Ki_Iq_set = objSets->Ki_Iq;

    objSets->Kp_spd_set = objSets->Kp_spd;
    objSets->Ki_spd_set = objSets->Ki_spd;

#if defined(MOTOR1_POWCTRL)
    // Get the power controller parameters
    objSets->Kp_pow = PI_getKp(obj->piHandle_pow);
    objSets->Ki_pow = PI_getKi(obj->piHandle_pow);

    objSets->Kp_pow_set = objSets->Kp_pow;
    objSets->Ki_pow_set = objSets->Ki_pow;
#endif  // MOTOR1_POWCTRL
}

//! \brief  Sets up control parameters for stopping motor
extern void stopMotorControl(MOTOR_Handle handle);

//! \brief  Sets up control parameters for restarting motor
extern void restartMotorControl(MOTOR_Handle handle);

//! \brief  Resets motor control parameters for restarting motor
extern void resetMotorControl(MOTOR_Handle handle);

//! \brief  Tune the gains of the controllers according to the speed or load
extern void tuneControllerGains(MOTOR_Handle handle);

//! \brief  set the coefficient of the controllers gains
extern void setupControllerSF(MOTOR_Handle handle);

//! \brief  update the coefficient of the controllers gains
extern void updateControllerSF(MOTOR_Handle handle);

//! \brief  update the ESMO parameters
extern void updateESMOParameters(MOTOR_Handle handle);

//! \brief  update the FAST parameters
extern void updateFASTParameters(MOTOR_Handle handle);

//! \brief  Sets up the current controllers
extern void setupCurrentControllers(MOTOR_Handle handle);


//! \brief  Sets up the controllers
extern void setupControllers(MOTOR_Handle handle);

#if !defined(_HSWFREQ_EN) && !defined(_SIMPLE_FAST_LIB)
//! \brief  Collect the current and voltage data to calculate the RMS
extern void collectRMSData(MOTOR_Handle handle);

//! \brief  Calculate the RMS data
extern void calculateRMSData(MOTOR_Handle handle);
#endif  // !(_HSWFREQ_EN & _SIMPLE_FAST_LIB)

//! \brief run motor monitor in main loop timer
extern void runMotorMonitor(MOTOR_Handle handle);

#if defined(MOTOR1_FAST)
//! \brief Rs online calibration
extern void runRsOnLine(MOTOR_Handle handle);
#endif // MOTOR1_FAST || MOTOR2_FAST

//! \brief      Updates the global motor variables
//! \param[in]  estHandle   The estimator (EST) handle
extern void updateGlobalVariables(MOTOR_Handle handle);

//! \brief      Updates the FWC parameters
extern void updateFWCParams(MOTOR_Handle handle);

//! \brief      Updates the MTPA parameters
extern void updateMTPAParams(MOTOR_Handle handle);

#if defined(CMD_POT_EN)
//! \brief
extern void setExtCmdPotParams(MOTOR_Handle handle);

//! \brief
extern void updateExtCmdPotFreq(MOTOR_Handle handle);
#endif  // CMD_POT_EN

#if defined(CMD_CAP_EN)
//! \brief
extern void setExtCmdCapParams(MOTOR_Handle handle);

//! \brief
extern void updateExtCmdCapFreq(MOTOR_Handle handle, const uint32_t timeStamp);
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
//! \brief
extern void setExtCmdSwitchParams(MOTOR_Handle handle);

//! \brief
extern void updateCmdSwitch(MOTOR_Handle handle);

//! \brief
extern void outputCmdState(MOTOR_Handle handle);
#endif  // CMD_SWITCH_EN

#if defined(SFRA_ENABLE)
//------------------------------------------------------------------------------
// Using SFRA tool :
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
inline void injectSFRA(void)
{
    float32_t sfraNoiseInj_pu = 0.0f;

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;

    sfraNoiseInj_pu = SFRA_F32_inject(0.0f);

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseId = sfraNoiseInj_pu * USER_M1_ADC_FULL_SCALE_CURRENT_A;
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseIq = sfraNoiseInj_pu * USER_M1_ADC_FULL_SCALE_CURRENT_A;
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseSpd = sfraNoiseInj_pu * USER_MOTOR1_FREQ_MAX_Hz;
    }

    return;
}

//------------------------------------------------------------------------------
inline void collectSFRA(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseOut = objMtr->Vdq_out_V.value[0] * (1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V);
        sfraNoiseFdb = objMtr->Idq_in_A.value[0] * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseOut = objMtr->Vdq_out_V.value[1] * (1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V);
        sfraNoiseFdb = objMtr->Idq_in_A.value[1] * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseOut = objMtr->IsRef_A * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
        sfraNoiseFdb = objMtr->speed_Hz * (1.0f / USER_MOTOR1_FREQ_MAX_Hz);
    }

    SFRA_F32_collect(&sfraNoiseOut, &sfraNoiseFdb);

    return;
}
//------------------------------------------------------------------------------

#endif  // SFRA_ENABLE


// **************************************************************************
// the functions
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setMotor1Params(userParams_Handle handle,
                                 MOTORSETS_Handle motorSetHandle);

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

#endif // end of _MOTOR_COMMON_H_ definition
