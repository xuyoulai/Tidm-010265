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
//! \file   /solutions/tida_010265_wminv/common/include/guiprotocol.h
//!
//! \brief  header file to be included in all labs
//!         implement GU for motor control with F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


#ifndef GUI_PROTOCOL_H
#define GUI_PROTOCOL_H


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

#include <stdlib.h>

#include "motor_common.h"
#include "motor1_drive.h"

//
// Typedefs
//

typedef enum
{
    GUI_UINT      = 0,    // unsigned int (uint16_t)
    GUI_INT       = 1,    // int (int16_t)
    GUI_ULONG     = 2,    // unsigned long (uint32_t)
    GUI_LONG      = 3,    // long (int32_t)
    GUI_FLOAT     = 4     // float (float32_t)
} GUI_Data_Type_e;

typedef enum
{
    GUI_RO        = 0<<8,   //!< Read only
    GUI_WO        = 1<<8,   //!< Write only
    GUI_WR        = 2<<8    //!< Read and Write
} GUI_Data_WR_e;

typedef struct _GUI_ComVarsList_t_
{
    uint16_t    varsIndex;          // Index (16)
    uint16_t    varsSpecific;       // Data_Type_e (8) + Data_WR_e (4) + Data_Scale_e (4)
    float32_t   varsTxScaleCoef;    // scale for convert 16-bit to float or long
    float32_t   varsRxScaleCoef;    // scale for convert float or long to 16-bit
    void*       varsAddress;        // variable address
//  uint32_t    varsAddress;        // variable address
} GUI_ComVarsList_t;

//
// Defines
//
// Copy from LCINV*.xlsx
#define GUI_DATA_WO_START_INDEX                 (0U)
#define GUI_DATA_WO_END_INDEX                   (8U)
#define GUI_DATA_WO_LENGTH                      (9U)

#define GUI_DATA_WR_START_INDEX                 (9U)
#define GUI_DATA_WR_END_INDEX                   (160U)
#define GUI_DATA_WR_LENGTH                      (152U)

#define GUI_DATA_WS_START_INDEX                 (0U)
#define GUI_DATA_WS_END_INDEX                   (160U)
#define GUI_DATA_WS_LENGTH                      (161U)

#define GUI_DATA_RD_START_INDEX                 (161U)
#define GUI_DATA_RD_END_INDEX                   (195U)
#define GUI_DATA_RD_LENGTH                      (35U)

#define GUI_DATA_CTRL_START_INDEX               (9U)
#define GUI_DATA_CTRL_END_INDEX                 (150U)
#define GUI_DATA_CTRL_LENGTH                    (142U)

#define GUI_DATA_MOTOR_START_INDEX              (23U)
#define GUI_DATA_MOTOR_END_INDEX                (45U)
#define GUI_DATA_MOTOR_LENGTH                   (23U)

#define GUI_DATA_SYS_START_INDEX                (151U)
#define GUI_DATA_SYS_END_INDEX                  (160U)
#define GUI_DATA_SYS_LENGTH                     (10U)

#define GUI_DATA_MAX_INDEX                      (195U)
#define GUI_DATA_LIST_LENGTH                    (196U)

// variables index definitions (copy from .xlsx file)
#define  MOTORCTRLVARS_TORQUESET_NM_IND (0U)
#define  MOTORCTRLVARS_POWERSET_W_IND   (1U)
#define  MOTORCTRLVARS_IQSET_A_IND  (2U)
#define  MOTORCTRLVARS_SPEEDSET_HZ_IND  (3U)
#define  MOTORCTRLVARS_ACCELERATIONSET_HZPS_IND (4U)
#define  MOTORCTRLVARS_SPEEDSET_RPM_IND (5U)
#define  MOTORCTRLVARS_ACCELERATIONSET_RPMPS_IND    (6U)
#define  MOTORCTRLVARS_ACCELERATIONSETTIME_IND  (7U)
#define  MOTORCTRLVARS_CONTROLCMDRECV_IND   (8U)
#define  MOTORSETVARS_MOTORMODEL_IND    (9U)
#define  MOTORSETVARS_OVERCURRENTTIMESSET_IND   (10U)
#define  MOTORSETVARS_OVERLOADTIMESET_IND   (11U)
#define  MOTORSETVARS_MOTORSTALLTIMESET_IND (12U)
#define  MOTORSETVARS_VOLTAGEFAULTTIMESET_IND   (13U)
#define  MOTORSETVARS_STARTUPFAILTIMESET_IND    (14U)
#define  MOTORSETVARS_OVERSPEEDTIMESET_IND  (15U)
#define  MOTORSETVARS_UNBALANCETIMESET_IND  (16U)
#define  MOTORSETVARS_LOSTPHASETIMESET_IND  (17U)
#define  MOTORSETVARS_FLYINGSTARTTIMEDELAY_IND  (18U)
#define  MOTORSETVARS_ALIGNTIMEDELAY_IND    (19U)
#define  MOTORSETVARS_FORCERUNTIMEDELAY_IND (20U)
#define  MOTORSETVARS_CONTROLTICKSPWM_IND   (21U)
#define  MOTORSETVARS_SPEEDTICKSCONTROL_IND (22U)
#define  MOTORSETVARS_MOTOR_TYPE_IND    (23U)
#define  MOTORSETVARS_NUMPOLEPAIRS_IND  (24U)
#define  MOTORSETVARS_RS_OHM_IND    (25U)
#define  MOTORSETVARS_LS_D_H_IND    (26U)
#define  MOTORSETVARS_LS_Q_H_IND    (27U)
#define  MOTORSETVARS_FLUX_VPHZ_IND (28U)
#define  MOTORSETVARS_RR_OHM_IND    (29U)
#define  MOTORSETVARS_MAGNETICCURRENT_A_IND (30U)
#define  MOTORSETVARS_MAXCURRENTRESEST_A_IND    (31U)
#define  MOTORSETVARS_MAXCURRENTINDEST_A_IND    (32U)
#define  MOTORSETVARS_FLUXEXCFREQ_HZ_IND    (33U)
#define  MOTORSETVARS_FLUXFILTERCOEF_IND    (34U)
#define  MOTORSETVARS_SPEEDFILTERCOEF_IND   (35U)
#define  MOTORSETVARS_BEMFFILTERCOEF_IND    (36U)
#define  MOTORSETVARS_SPEEDPOLE_RPS_IND (37U)
#define  MOTORSETVARS_DIRECTIONPOLE_RPS_IND (38U)
#define  MOTORSETVARS_FLUXPOLE_RPS_IND  (39U)
#define  MOTORSETVARS_RSONLINE_RDELTA_OHM_IND   (40U)
#define  MOTORSETVARS_RSONLINE_ADELTA_RAD_IND   (41U)
#define  MOTORSETVARS_VOLTAGEFILTER_HZ_IND  (42U)
#define  MOTORSETVARS_VOLTAGESCALE_V_IND    (43U)
#define  MOTORSETVARS_CURRENTSCALE_A_IND    (44U)
#define  MOTORSETVARS_PWMCONTROL_KHZ_IND    (45U)
#define  MOTORSETVARS_RSONLINECURRENT_A_IND (46U)
#define  MOTORSETVARS_ANGLEESTDELAYED_SF_IND    (47U)
#define  MOTORSETVARS_MAXFREQUENCY_HZ_IND   (48U)
#define  MOTORSETVARS_MAXCURRENT_A_IND  (49U)
#define  MOTORSETVARS_MAXVOLTAGE_V_IND  (50U)
#define  MOTORSETVARS_MAXPEAKCURRENT_A_IND  (51U)
#define  MOTORSETVARS_MAXVSMAG_PU_IND   (52U)
#define  MOTORSETVARS_LS_D_ICOMP_COEF_IND   (53U)
#define  MOTORSETVARS_LS_Q_ICOMP_COEF_IND   (54U)
#define  MOTORSETVARS_LS_MIN_H_IND  (55U)
#define  MOTORSETVARS_POWERCTRLSET_W_IND    (56U)
#define  MOTORSETVARS_OVERCURRENT_A_IND (57U)
#define  MOTORSETVARS_OVERLOADSET_W_IND (58U)
#define  MOTORSETVARS_LOSTPHASESET_A_IND    (59U)
#define  MOTORSETVARS_UNBALANCERATIOSET_IND (60U)
#define  MOTORSETVARS_STALLCURRENTSET_A_IND (61U)
#define  MOTORSETVARS_SPEEDFAILMAXSET_HZ_IND    (62U)
#define  MOTORSETVARS_SPEEDFAILMINSET_HZ_IND    (63U)
#define  MOTORSETVARS_ISFAILEDCHEKSET_A_IND (64U)
#define  MOTORSETVARS_TOQUEFAILMINSET_NM_IND    (65U)
#define  MOTORSETVARS_OVERVOLTAGEFAULT_V_IND    (66U)
#define  MOTORSETVARS_OVERVOLTAGENORM_V_IND (67U)
#define  MOTORSETVARS_UNDERVOLTAGEFAULT_V_IND   (68U)
#define  MOTORSETVARS_UNDERVOLTAGENORM_V_IND    (69U)
#define  MOTORSETVARS_FLUXCURRENT_A_IND (70U)
#define  MOTORSETVARS_ALIGNCURRENT_A_IND    (71U)
#define  MOTORSETVARS_STARTCURRENT_A_IND    (72U)
#define  MOTORSETVARS_BRAKINGCURRENT_A_IND  (73U)
#define  MOTORSETVARS_ACCELSTART_HZPS_IND   (74U)
#define  MOTORSETVARS_ACCELSTOP_HZPS_IND    (75U)
#define  MOTORSETVARS_ACCELRUN_HZPS_IND (76U)
#define  MOTORSETVARS_SPEEDFLYINGSTART_HZ_IND   (77U)
#define  MOTORSETVARS_SPEEDFORCE_HZ_IND (78U)
#define  MOTORSETVARS_SPEEDSTART_HZ_IND (79U)
#define  MOTORSETVARS_VSREF_PU_IND  (80U)
#define  MOTORSETVARS_KP_FWC_IND    (81U)
#define  MOTORSETVARS_KI_FWC_IND    (82U)
#define  MOTORSETVARS_ANGLEFWCMAX_RAD_IND   (83U)
#define  MOTORSETVARS_GAIN_SPEED_HIGH_HZ_IND    (84U)
#define  MOTORSETVARS_GAIN_SPEED_LOW_HZ_IND (85U)
#define  MOTORSETVARS_KP_SPD_HIGH_SF_IND    (86U)
#define  MOTORSETVARS_KI_SPD_HIGH_SF_IND    (87U)
#define  MOTORSETVARS_KP_SPD_LOW_SF_IND (88U)
#define  MOTORSETVARS_KI_SPD_LOW_SF_IND (89U)
#define  MOTORSETVARS_KP_IQ_SF_IND  (90U)
#define  MOTORSETVARS_KI_IQ_SF_IND  (91U)
#define  MOTORSETVARS_KP_ID_SF_IND  (92U)
#define  MOTORSETVARS_KI_ID_SF_IND  (93U)
#define  MOTORSETVARS_KP_POW_SF_IND (94U)
#define  MOTORSETVARS_KI_POW_SF_IND (95U)
#define  MOTORSETVARS_KP_SPD_START_SF_IND   (96U)
#define  MOTORSETVARS_KI_SPD_START_SF_IND   (97U)
#define  MOTORSETVARS_ESMO_FAST_FSW_HZ_IND  (98U)
#define  MOTORSETVARS_ESMO_KSLIDEMAX_IND    (99U)
#define  MOTORSETVARS_ESMO_KSLIDEMIN_IND    (100U)
#define  MOTORSETVARS_ESMO_LPFFC_HZ_IND (101U)
#define  MOTORSETVARS_ESMO_FILTERFC_SF_IND  (102U)
#define  MOTORSETVARS_ESMO_E0_IND   (103U)
#define  MOTORSETVARS_ESMO_PLL_KPMAX_IND    (104U)
#define  MOTORSETVARS_ESMO_PLL_KPMIN_IND    (105U)
#define  MOTORSETVARS_ESMO_PLL_KPSF_IND (106U)
#define  MOTORSETVARS_ESMO_PLL_KI_IND   (107U)
#define  MOTORSETVARS_ANGLEPLLDELAYED_SF_IND    (108U)
#define  MOTORSETVARS_HFI_KSPD_IND  (109U)
#define  MOTORSETVARS_HFI_EXCMAG_COARSE_V_IND   (110U)
#define  MOTORSETVARS_HFI_EXCMAG_FINE_V_IND (111U)
#define  MOTORSETVARS_HFI_WAITTIME_COARSE_SEC_IND   (112U)
#define  MOTORSETVARS_HFI_WAITTIME_FINE_SEC_IND (113U)
#define  MOTORSETVARS_HFI_EXCFREQ_HZ_IND    (114U)
#define  MOTORSETVARS_HFI_LPFFCSPD_HZ_IND   (115U)
#define  MOTORSETVARS_HFI_HPFFCIQ_HZ_IND    (116U)
#define  MOTORSETVARS_HFI_IQMAXHFI_A_IND    (117U)
#define  MOTORSETVARS_HFI_IQMAXEST_A_IND    (118U)
#define  MOTORSETVARS_HFI_IQSLOPE_A_IND (119U)
#define  MOTORSETVARS_HFI_FREQLOW_HZ_IND    (120U)
#define  MOTORSETVARS_HFI_FREQHIGH_HZ_IND   (121U)
#define  MOTORSETVARS_PIR_SPD_F0_IND    (122U)
#define  MOTORSETVARS_PIR_SPD_FC_IND    (123U)
#define  MOTORSETVARS_PIR_SPD_K_IND (124U)
#define  MOTORSETVARS_PIR_IDQ_FC_IND    (125U)
#define  MOTORSETVARS_PIR_ID_K_IND  (126U)
#define  MOTORSETVARS_PIR_IQ_K_IND  (127U)
#define  MOTORSETVARS_VBC_ANGLE_DELTA_IND   (128U)
#define  MOTORSETVARS_VBC_IQ_AMP_IND    (129U)
#define  MOTORSETVARS_VBC_IQ_SF_IND (130U)
#define  MOTORSETVARS_VBC_FREQ_SF_IND   (131U)
#define  MOTORSETVARS_VBC_ANGLE_SF_IND  (132U)
#define  MOTORSETVARS_VBC_KI_SF_IND (133U)
#define  MOTORSETVARS_FILTERISFC_IND    (134U)
#define  MOTORSETVARS_FILTERVSFC_IND    (135U)
#define  MOTORSETVARS_V_DECOUP_SF_IND   (136U)
#define  MOTORSETVARS_IDINJ_A_IND   (137U)
#define  MOTORSETVARS_IQINJ_A_IND   (138U)
#define  MOTORSETVARS_CONTROLTYPES_IND  (139U)
#define  MOTORSETVARS_CONTROLFUNCS_IND  (140U)
#define  MOTORSETVARS_FAULTMTRMASK_ALL_IND  (141U)
#define  MOTORSETVARS_SAMPLETRIGDELAY_IND   (142U)
#define  MOTORSETVARS_OVERTEMPERATUREMOTOR_IND  (143U)
#define  MOTORSETVARS_OVERTEMPERATUREMODULE_IND (144U)
#define  MOTORSETVARS_RSONLINEWAITTIMESET_IND   (145U)
#define  MOTORSETVARS_RSONLINEWORKTIMESET_IND   (146U)
#define  MOTORSETVARS_STARTUPTIMEDELAY_IND  (147U)
#define  MOTORSETVARS_STOPWAITTIMESET_IND   (148U)
#define  MOTORSETVARS_RESTARTWAITTIMESET_IND    (149U)
#define  MOTORSETVARS_RESTARTTIMESSET_IND   (150U)
#define  SYSWASHVARS_OOBCHECKTIMESET_IND    (151U)
#define  SYSWASHVARS_OOBSPEEDSET_RPM_IND    (152U)
#define  SYSWASHVARS_OOBCALCCOEFSET_IND (153U)
#define  SYSWASHVARS_OOBACCELSET_RPMPS_IND  (154U)
#define  SYSWASHVARS_OOBCALCBASE_IND    (155U)
#define  SYSWASHVARS_WEIGHTCHECKTIMESET_IND (156U)
#define  SYSWASHVARS_WEIGHTSPEEDSET_RPM_IND (157U)
#define  SYSWASHVARS_WEIGHTCALCCOEFSET_IND  (158U)
#define  SYSWASHVARS_WEIGHTACCELSET_RPMPS_IND   (159U)
#define  SYSWASHVARS_WEIGHTCALCBASE_IND (160U)
#define  MOTORCTRLVARS_MOTORCTRLSTATES_IND  (161U)
#define  MOTORCTRLVARS_FAULTMOTOR_ALL_IND   (162U)
#define  MOTORVARS_SPEEDPLL_HZ_IND  (163U)
#define  MOTORVARS_SPEEDEST_HZ_IND  (164U)
#define  MOTORVARS_SPEED_HZ_IND (165U)
#define  MOTORVARS_ANGLEFOC_RAD_IND (166U)
#define  MOTORVARS_ANGLEEST_RAD_IND (167U)
#define  MOTORVARS_ANGLEPLL_RAD_IND (168U)
#define  MOTORVARS_SPEED_RPM_IND    (169U)
#define  MOTORVARS_ADCDATA_I_A_VALUE0_IND   (170U)
#define  MOTORVARS_ADCDATA_I_A_VALUE1_IND   (171U)
#define  MOTORVARS_ADCDATA_I_A_VALUE2_IND   (172U)
#define  MOTORVARS_ADCDATA_V_V_VALUE0_IND   (173U)
#define  MOTORVARS_ADCDATA_V_V_VALUE1_IND   (174U)
#define  MOTORVARS_ADCDATA_V_V_VALUE2_IND   (175U)
#define  MOTORVARS_ADCDATA_VDCBUS_V_IND (176U)
#define  MOTORVARS_IDQ_IN_A_VALUE0_IND  (177U)
#define  MOTORVARS_IDQ_IN_A_VALUE1_IND  (178U)
#define  MOTORVARS_VDQ_OUT_V_VALUE0_IND (179U)
#define  MOTORVARS_VDQ_OUT_V_VALUE1_IND (180U)
#define  MOTORVARS_IRMS_A0_IND  (181U)
#define  MOTORVARS_IRMS_A1_IND  (182U)
#define  MOTORVARS_IRMS_A2_IND  (183U)
#define  MOTORVARS_TORQUE_NM_IND    (184U)
#define  MOTORVARS_POWERACTIVE_W_IND    (185U)
#define  MOTORVARS_RSONLINE_OHM_IND (186U)
#define  MOTORVARS_IDQREF_A_VALUE0_IND  (187U)
#define  MOTORVARS_IDQREF_A_VALUE1_IND  (188U)
#define  MOTORCTRLVARS_TEMPERATUREMODULE_IND    (189U)
#define  MOTORCTRLVARS_TEMPERATUREMOTOR_IND (190U)
#define  SYSWASHVARS_OOBCALCVALUE_IND   (191U)
#define  SYSWASHVARS_WEIGHTCALCVALUE_IND    (192U)
#define  SYSWASHVARS_TORQUECURRENTSUM_A_IND (193U)
#define  SYSWASHVARS_ACTIVEPOWERSUM_W_IND   (194U)
#define  SYSWASHVARS_SPEEDRIPPLE_RPM_IND    (195U)

//
// the globals
//
extern const GUI_ComVarsList_t guiComVarsList[GUI_DATA_LIST_LENGTH];

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

#endif // end of GUI_PROTOCOL_H defines

//
// End of File
//
