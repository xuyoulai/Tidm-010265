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
// \file   /solutions/tida_010265_wminv/common/include/systemcontrol.h
//
// \brief  header file to be included in all labs
//         support for motor control with F28002x/F28003x/F280013x
//
//------------------------------------------------------------------------------


#ifndef SYSTEM_CONTROL_H
#define SYSTEM_CONTROL_H


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

#if defined(GUI_SCI_EN)
//
// Defines
//
typedef enum {
  OOB_Status_Idle = 0,          // the Idle
  OOB_Status_Start,             // the start
  OOB_Status_RunCW,             // the motor CW run 1s with 88rpm
  OOB_Status_RunCCW,            // the motor CW run 1s with 88rpm
  OOB_Status_ChkOOB,            //
  OOB_Status_GetOOB,            //
  OOB_Status_ChkWeight,         //
  OOB_Status_SpeedHold,         //
  OOB_Status_SpeedAvg,          //
  OOB_Status_SpeedAccel,        //
  OOB_Status_WeightAccel,       //
  OOB_Status_WeightHold,        //
  OOB_Status_WeightOOBEnd,      //
  OOB_Status_Calc,              //
  OOB_Status_Exit,              //
  OOB_Status_Finish             //
} OOB_Status_e;


//
// Typedefs
//
typedef struct _SYS_ctrlVars_t_
{

} SYS_CtrlVars_t;

//! \brief Defines the SYS_CtrlVars_t handle
//!
typedef struct _SYS_ctrlVars_t_ *SYSCTRL_Handle;


typedef struct _SYS_SetVars_t_
{

} SYS_SetVars_t;


//! \brief Defines the SYS_SetVars_t handle
//!
typedef struct _SYS_SetVars_t_ *SYSSET_Handle;

typedef struct _SYS_WashVars_t_
{
    float32_t oobSpeed_Hz;
    float32_t oobCalcCoef;
    float32_t oobAccel_Hzps;
    float32_t oobResultOrig;
    float32_t oobDeltaCoef;
    float32_t ooDeltaBase;
    float32_t oobCurrentCoef;

    float32_t weightSpeed_Hz;
    float32_t weightCalcCoef;
    float32_t weightAccel_Hzps;
    float32_t weightStartSpeed_Hz;
    float32_t weightFinishSpeed_Hz;

    float32_t activePower;
    float32_t activePowerAverage;
    float32_t torqueCurrentAverage;

    float32_t oobCalcValue;
    float32_t weightCalcValue;
    float32_t torqueCurrentSum_A;
    float32_t activePowerSum_W;
    float32_t speedRipple_rpm;

    float32_t speedCalCoef;
    float32_t currentCalcCoef;
    float32_t powerCalcCoef;

    bool oobCheckFlag;
    bool weightCheckFlag;
    OOB_Status_e oobStatus;

    uint16_t  oobHoldTimeSet;
    uint16_t  oobStopTimeSet;
    uint16_t  oobCalcLevel;

    uint16_t  weightAccelTimeSet;
    uint16_t  weightDataAquEnable;
    int16_t   weightSpeedLevel;
    int16_t   weightSpeedLevelCorr;

    uint32_t  oobTimeCount;
    uint32_t  weightTimeCount;
    uint32_t  timeCalcCoef;

    uint16_t  weightCalcBase;
    uint16_t  oobCalcBase;

    uint16_t  oobCheckTimeSet;          // P-116
    uint16_t  oobSpeedSet_rpm;          // P-117
    uint16_t  oobCalcCoefSet;           // P-118
    uint16_t  oobAccelSet_rpmps;        // P-119

    uint16_t  weightCheckTimeSet;       // P-120
    uint16_t  weightSpeedSet_rpm;       // P-121
    uint16_t  weightCalcCoefSet;        // P-122
    uint16_t  weightAccelSet_rpmps;     // P-123
} SYS_WashVars_t;


typedef struct _SYS_ParmVars_t_
{
    uint16_t supplierCode;
    uint16_t softwareVersion;
    uint16_t algorithmVersion;
    uint16_t hardwareVersion;
    uint16_t protocolVersion;

    uint16_t platformNumber;
    uint16_t volumeNumber;
    uint16_t motorNumber;
    MOTOR_Model_e motorModel;
} SYS_ParmVars_t;


//! \brief Defines the SYS_ParmVars_t handle
//!
typedef struct _SYS_ParmVars_t_ *SYSPARM_Handle;


// the modules


// the globals
extern SYS_CtrlVars_t sysCtrlVars;
extern SYS_SetVars_t  sysSetVars;
extern SYS_WashVars_t sysWashVars;
extern SYS_ParmVars_t sysParmVars;
#endif  // GUI_SCI_EN

// the functions


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

#endif // end of SYSTEM_CONTROL_H defines

//
// End of File
//
