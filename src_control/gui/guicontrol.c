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
//! \file   /solutions/tida_010265_wminv/common/source/guicontrol.c
//!
//! \brief  This project is used to implement GUI for motor control
//!         F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


//
// Includes
//
#include "user.h"
#include "hal.h"

#include "motor_common.h"
#include "motor1_drive.h"

#include "guidatalist.h"
#include "guicontrol.h"

#include "controlparameters.h"
#include "systemcontrol.h"

//#####END_INTFAST#####
#if defined(BUADTUNE_EN)
#include "baudratetune.h"
#endif  // BUADTUNE_EN

//
// Defines
//


//
// Typedefs
//


//
// Globals
//
#if defined(GUI_SCI_EN)
#pragma SET_DATA_SECTION ("gui_data")

uint16_t guiRxDataFifo[GUI_RX_BUFF_LENGTH];     // receiving data
uint16_t guiRxDataBuf[GUI_RX_BUFF_LENGTH];      // dealing data

uint16_t guiTxDataFifo[GUI_TX_BUFF_LENGTH];     // ready to send
uint16_t guiTxDataBuf[GUI_TX_BUFF_LENGTH];      // transmitting data

GUI_RTx_Status_e guiRxStatus;
GUI_RTx_Status_e guiTxStatus;

GUI_Command_e guiRxCommand;
GUI_Command_e guiTxCommand;

bool guiEnableTRx;
uint16_t guiTRxWaitTimeCnt;
uint16_t guiTRxWaitTimeSet;

uint16_t guiRxWaitTimeCnt;
uint16_t guiTxWaitTimeCnt;
uint16_t guiRxWaitTimeSet;
uint16_t guiTxWaitTimeSet;

uint16_t guiRxFailedTimesCnt;
uint16_t guiTxFailedTimesCnt;

uint16_t RxDataIndex;
uint16_t TxDataIndex;

uint16_t RxDataValue;
int16_t  TxDataValue;

uint16_t guiRxDataIndex[3];
uint16_t guiTxDataIndex[3];

uint16_t guiRxListIndex;
uint16_t guiTxListIndex;

uint16_t guiRxDataChkSum;
uint16_t guiTxDataChkSum;


//
// Functions
//

//
// GUI_updateTestTxData
//
#if defined(GUI_SCI_TEST)
#pragma CODE_SECTION(GUI_updateTestTxData, "ctrlfuncs")
void GUI_updateTestTxData(void)
{
    uint16_t i = 0;
    uint16_t TxDataChkSum = 0x0000;

    GUI_RTx_Status_e txStatus = guiTxStatus;

    switch(txStatus)
    {
        case GUI_RTX_IDLE:
            break;
        case GUI_RTX_GOING:
            // Wait send the data
            break;
        case GUI_RTX_SUCCESS:
        {
            guiTxStatus = GUI_RTX_IDLE;

#if !defined(GUI_SCI_AI_TEST)
            guiTxCommand = guiRxCommand;
            guiTxDataBuf[0]  = (uint16_t)((guiTxCommand) & 0x00FF);
            guiTxDataBuf[1]  = (uint16_t)((guiTxTestCount>>24) & 0x00FF);
            guiTxDataBuf[2]  = (uint16_t)((guiTxTestCount>>16) & 0x00FF);
            guiTxDataBuf[3]  = (uint16_t)((guiTxTestCount>>8 ) & 0x00FF);
            guiTxDataBuf[4]  = (uint16_t)( guiTxTestCount      & 0x00FF);
            guiTxDataBuf[5]  = 5;
            guiTxDataBuf[6]  = 6;
            guiTxDataBuf[7]  = 7;
            guiTxDataBuf[8]  = 8;
            guiTxDataBuf[9]  = 9;
            guiTxDataBuf[10] = 10;

#else   // GUI_SCI_AI_TEST
            if(guiTxDataType == 0)
            {
                guiTxMtrCount++;
                guiTxDataType = 1;

                guiTxDataBuf[0] = guiTxMtrDataID;
                guiTxDataBuf[1]  = (uint16_t)((guiTxMtrCount>>24) & 0x00FF);
                guiTxDataBuf[2]  = (uint16_t)((guiTxMtrCount>>16) & 0x00FF);
                guiTxDataBuf[3]  = (uint16_t)((guiTxMtrCount>>8 ) & 0x00FF);
                guiTxDataBuf[4]  = (uint16_t)( guiTxMtrCount      & 0x00FF);
                guiTxDataBuf[5]  = 5;
            }
            else
            {
                guiTxVibCount++;
                guiTxDataType = 0;

                guiTxDataBuf[0] = guiTxVibDataID;
                guiTxDataBuf[1]  = (uint16_t)((guiTxVibCount>>24) & 0x00FF);
                guiTxDataBuf[2]  = (uint16_t)((guiTxVibCount>>16) & 0x00FF);
                guiTxDataBuf[3]  = (uint16_t)((guiTxVibCount>>8 ) & 0x00FF);
                guiTxDataBuf[4]  = (uint16_t)( guiTxVibCount      & 0x00FF);
                guiTxDataBuf[5]  = 50;
            }

            guiTxDataBuf[6]  = 6;
            guiTxDataBuf[7]  = 7;
            guiTxDataBuf[8]  = 8;
            guiTxDataBuf[9]  = 9;
            guiTxDataBuf[10] = 10;
            guiTxDataBuf[11] = 11;
            guiTxDataBuf[12] = 12;
            guiTxDataBuf[13] = 13;
#endif  // GUI_SCI_AI_TEST

            for(i = 0; i < (GUI_TX_BUFF_LENGTH - 1); i++)
            {
                TxDataChkSum += guiTxDataBuf[i];
            }

            TxDataChkSum &= 0x00FF;
            guiTxDataBuf[GUI_TX_BUFF_LENGTH - 1] = TxDataChkSum;
        }
            break;
        case GUI_RTX_FAILED:
            guiTxFailedTimesCnt++;
            guiTxStatus = GUI_RTX_IDLE;
            break;
        default:
            break;
    } // switch(guiTxStatus)
}   // GUI_updateTestTxData
#endif  // GUI_SCI_TEST & _F28003x

//
// GUI_updateTransmitData
//
#pragma CODE_SECTION(GUI_updateTransmitData, "ctrlfuncs")
void GUI_updateTransmitData(void)
{
    uint16_t i = 0;

    switch(guiTxStatus)
    {
        case GUI_RTX_IDLE:
            break;
        case GUI_RTX_GOING:
            // Wait send the data
            break;
        case GUI_RTX_SUCCESS:
        {
            guiTxStatus = GUI_RTX_IDLE;

            if((motorCtrlVars_M1.faultMotor.all != 0x0000) && (guiRxCommand >= GUI_MTR_ID_RS))
            {
                guiTxCommand = GUI_M_TEST_DGF;
            }
            else
            {
                guiTxCommand = guiRxCommand;
            }

            guiTxDataBuf[0] = (uint16_t)(guiTxCommand) & 0x00FF;

            switch(guiTxCommand)
            {
                case GUI_IDLE_BR:
                {
                    // 0xA0[0]
                    // +0xC0/0xCC [1]
                    // + 0xAA[2]~[10]
#if defined(BUADTUNE_EN)
                    if(flagBaudRateTuned >= 3)
                    {
                        guiTxDataBuf[1] = 0xC0;
                    }
                    else
                    {
                        guiTxDataBuf[1] = 0xAA;
                    }
#else  // !BUADTUNE_EN
                    guiTxDataBuf[1] = 0xC0;
#endif  // !BUADTUNE_EN

//                    guiTxDataBuf[2]  = 0;
//                    guiTxDataBuf[3]  = 0;
//                    guiTxDataBuf[4]  = 0;
//                    guiTxDataBuf[5]  = 0;
//                    guiTxDataBuf[6]  = 0;
//                    guiTxDataBuf[7]  = 0;
//                    guiTxDataBuf[8]  = 0;
//                    guiTxDataBuf[9]  = 0;
//                    guiTxDataBuf[10] = 0;
                }
                    break;
                case GUI_PING_CHK:
                {
                    // 0xA1[0] + Running Status[1]
                    // + Software Version [2] [3]
                    // + Algorithm [4]
                    // + Data[5]~[10](=0, reserve)
                    guiTxDataBuf[1] = motorCtrlVars_M1.motorCtrlStates & 0x00FF;
                    guiTxDataBuf[2] = (sysParmVars.softwareVersion>>8) & 0x00FF;
                    guiTxDataBuf[3] = sysParmVars.softwareVersion & 0x00FF;
                    guiTxDataBuf[4] = sysParmVars.algorithmVersion & 0x00FF;
                    guiTxDataBuf[5] = sysParmVars.hardwareVersion & 0x00FF;
//                    guiTxDataBuf[6] = 0;
//                    guiTxDataBuf[7] = 0;
//                    guiTxDataBuf[8] = 0;
//                    guiTxDataBuf[9] = 0;
//                    guiTxDataBuf[10] = 0;
                }
                    break;
                case GUI_RESET_PRMS:
                {
                    // 0xA7[0] + Running Status[1]
                    // + Reset state[2]
                    // + Data[3]~[10](=0, reserve)
                    guiTxDataBuf[1] = motorCtrlVars_M1.motorCtrlStates & 0x00FF;

                    guiTxDataBuf[2] = guiTxDataIndex[0];
                    TxDataValue = GUI_readTxDataList(guiTxDataBuf[2]);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    guiTxDataBuf[5] = guiTxDataIndex[1];
                    TxDataValue = GUI_readTxDataList(guiTxDataBuf[5]);
                    guiTxDataBuf[6] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[7] = TxDataValue & 0x00FF;

                    guiTxDataBuf[8] = guiTxDataIndex[2];
                    TxDataValue = GUI_readTxDataList(guiTxDataBuf[8]);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_CODE_UPDATE:
                {
                    // 0xA3[0]
                    // + Code index[1][2]
                    // + Code[3][4]
                    // + Code[5][6]
                    // + Code[7][8]
                    // + Code[9][10]
                }
                    break;
                case GUI_M_TEMP_CHK:  // 1
                {
                    // 0xA2[0] + Running Status[1]
                    // + temperature (motor)[2]
                    // + temperature (IPM) [3]
                    // + Data[4]~[10](=0, reserve)
                    guiTxDataBuf[1] = motorCtrlVars_M1.motorCtrlStates & 0x00FF;
                    guiTxDataBuf[2] = motorCtrlVars_M1.temperatureMotor & 0x00FF;
                    guiTxDataBuf[3] = motorCtrlVars_M1.temperatureModule & 0x00FF;
//                    guiTxDataBuf[4] = 0;
//                    guiTxDataBuf[5] = 0;
//                    guiTxDataBuf[6] = 0;
//                    guiTxDataBuf[7] = 0;
//                    guiTxDataBuf[8] = 0;
//                    guiTxDataBuf[9] = 0;
//                    guiTxDataBuf[10] = 0;
                }
                    break;
                case GUI_MTR_ID_RS:
                {
                    // 0xA4[0] + Running Status[1]
                    // + Motor ID state[2]
                    // + Rs [3][4
                    // + Ld [7][8]]
                    // + Lq [7][8]
                    // + Flux[9][10]
                    guiTxDataBuf[1] = motorCtrlVars_M1.motorCtrlStates & 0x00FF;

#if defined(MOTOR1_FAST)
                    guiTxDataBuf[2] = motorVars_M1.estState & 0x00FF;
#else   //!MOTOR1_FAST
                    guiTxDataBuf[2] = 0;
#endif  //!MOTOR1_FAST

                    TxDataValue = GUI_readDataFloat32(MOTORSETVARS_RS_OHM_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORSETVARS_LS_D_H_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORSETVARS_LS_Q_H_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORSETVARS_FLUX_VPHZ_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_RUN_REQ_DATA:
                {
                    // 0xA5[0] + Running Status[1]
                    // + motor speed [2][3]
                    // + TX Data1_Index[4]
                    // + TX Data1[5][6]
                    // + TX Data2_Index[7]
                    // + TX Data2[8][9]
                    // + Data[10] (Reserve, 0)

                    guiTxDataBuf[1] = motorCtrlVars_M1.motorCtrlStates & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[2] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[3] = TxDataValue & 0x00FF;

                    guiTxDataBuf[4] = guiTxDataIndex[0];
                    TxDataValue = GUI_readTxDataList(guiTxDataBuf[4]);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    guiTxDataBuf[7] = guiTxDataIndex[1];
                    TxDataValue = GUI_readTxDataList(guiTxDataBuf[7]);
                    guiTxDataBuf[8] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[9] = TxDataValue & 0x00FF;

                    guiTxDataBuf[10] = 0;       // Reserve
                }
                    break;
                case GUI_RUN_SET_PRMS:
                {
                    // 0xA6[0] + Running Status[1]
                    // + motor frequency[2] [3]
                    // + rotor angle [4][5]
                    // + dc bus voltage [6][7]
                    // + TX Data1_Index[8]
                    // + TX Data1[9][10]
                    guiTxDataBuf[1] = motorCtrlVars_M1.motorCtrlStates & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[2] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[3] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[4] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[5] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_VDCBUS_V_IND);
                    guiTxDataBuf[6] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[7] = TxDataValue & 0x00FF;

                    guiTxDataBuf[8] = guiTxDataIndex[0];
                    TxDataValue = GUI_readTxDataList(guiTxDataBuf[8]);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DG1:
                {
                    // 0xA8[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + current A [5][6]
                    // + current B [7][8]
                    // + current C [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_I_A_VALUE0_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_I_A_VALUE1_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_I_A_VALUE2_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DG2:
                {
                    // 0xA9[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + voltage A [5][6]
                    // + voltage B [7][8]
                    // + voltage C [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_V_V_VALUE0_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_V_V_VALUE1_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_V_V_VALUE2_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DG3:
                {
                    // 0xAA[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + dc bus voltage [5][6]
                    // + d-axis current [7][8]
                    // + q-axis current [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_VDCBUS_V_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_IDQ_IN_A_VALUE0_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_IDQ_IN_A_VALUE1_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DG4:
                {
#if !defined(RSONLINE_TEST)
                    // 0xAB[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + torque [5][6]
                    // + power [7][8]
                    // + temperature (motor) [9]
                    // + temperature (IPM) [10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_TORQUE_NM_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_POWERACTIVE_W_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_RSONLINE_OHM_IND);
                    guiTxDataBuf[9]  = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
#else   // RSONLINE_TEST
                    // 0xAB[0]
                    // + motor speed [1][2]
                    // + Id_in [3][4]
                    // + Iq_in [5][6]
                    // + RsOnline [7][8]
                    // + winding A temperature (motor) [9]
                    // + winding B temperature [10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_IDQ_IN_A_VALUE0_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_IDQ_IN_A_VALUE1_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_RSONLINE_OHM_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
#endif  // RSONLINE_TEST
                }
                    break;
                case GUI_M_TEST_DG5:
                {
                    // 0xAC[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + FAST angel  [5][6]
                    // + eSMO angle  [7][8]
                    // + eSMO speed  [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEEDEST_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEEST_RAD_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEPLL_RAD_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEEDPLL_HZ_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DG6:
                {
                    // 0xAD[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + OOB Calculation [5][6]
                    // + Torque Sum [7][8]
                    // + Speed Ripple [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(SYSWASHVARS_OOBCALCVALUE_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(SYSWASHVARS_TORQUECURRENTSUM_A_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(SYSWASHVARS_SPEEDRIPPLE_RPM_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DG7:
                {
                    // 0xAE[0]
                    // + motor speed [1][2]
                    // + rotor angle [3][4]
                    // + Weight Calculation [5][6]
                    // + Torque Sum [7][8]
                    // + Power Sum [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ANGLEFOC_RAD_IND);
                    guiTxDataBuf[3] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[4] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(SYSWASHVARS_WEIGHTCALCVALUE_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(SYSWASHVARS_TORQUECURRENTSUM_A_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(SYSWASHVARS_ACTIVEPOWERSUM_W_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                case GUI_M_TEST_DGF:
                {
                    // 0xAF[0]
                    // + motor speed [1][2]
                    // + fault [3][4]
                    // + current A [5][6]
                    // + current B [7][8]
                    // + current C [9][10]
                    TxDataValue = GUI_readDataFloat32(MOTORVARS_SPEED_HZ_IND);
                    guiTxDataBuf[1] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[2] = TxDataValue & 0x00FF;

                    guiTxDataBuf[3] = (motorCtrlVars_M1.faultMotor.all>>8) & 0x00FF;
                    guiTxDataBuf[4] = motorCtrlVars_M1.faultMotor.all & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_I_A_VALUE0_IND);
                    guiTxDataBuf[5] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[6] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_I_A_VALUE1_IND);
                    guiTxDataBuf[7] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[8] = TxDataValue & 0x00FF;

                    TxDataValue = GUI_readDataFloat32(MOTORVARS_ADCDATA_I_A_VALUE2_IND);
                    guiTxDataBuf[9] = (TxDataValue>>8) & 0x00FF;
                    guiTxDataBuf[10] = TxDataValue & 0x00FF;
                }
                    break;
                default:
                    break;
            }   // switch(guiTxCommand)


            uint16_t TxDataChkSum = 0x0000;

            for(i = 0; i < (GUI_TX_BUFF_LENGTH - 1); i++)
            {
                TxDataChkSum += guiTxDataBuf[i];
            }

            TxDataChkSum &= 0x00FF;
            guiTxDataBuf[GUI_TX_BUFF_LENGTH - 1] = TxDataChkSum;
        }
            break;
        case GUI_RTX_FAILED:
            guiTxFailedTimesCnt++;
            guiTxStatus = GUI_RTX_IDLE;
            break;
        default:
            break;
    }   // switch(guiTxStatus)

    return;
}   // GUI_updateTransmitData()

//
// TODO: GUI_readTxDataList
//
#pragma CODE_SECTION(GUI_readTxDataList, "ctrlfuncs")
int16_t GUI_readTxDataList(uint16_t comTxIndex)
{
    int16_t comTxData = 0;

    uint16_t txListIndex = guiComVarsList[comTxIndex].varsIndex;

    if(txListIndex == comTxIndex)
    {
        uint16_t dataSpecs = guiComVarsList[comTxIndex].varsSpecific;

        GUI_Data_Type_e dataType = (GUI_Data_Type_e)(dataSpecs & 0x00FF);

        float32_t dataTxScaleCoef = guiComVarsList[comTxIndex].varsTxScaleCoef;

        switch(dataType)
        {
            case GUI_UINT:      // unsigned int (uint16_t)
            {
                uint16_t *ptrDataUint16 = guiComVarsList[comTxIndex].varsAddress;
                comTxData = (uint16_t)(*ptrDataUint16);
            }
                break;
            case GUI_INT:       // int (int16_t)
            {
                int16_t *ptrDataInt16 = guiComVarsList[comTxIndex].varsAddress;
                comTxData = (int16_t)(*ptrDataInt16);
            }
                break;

            case GUI_ULONG:     // unsigned long (uint32_t)
            {
                uint32_t *ptrDataUint32 = guiComVarsList[comTxIndex].varsAddress;
                uint32_t txDataUint32 = *ptrDataUint32;
                comTxData = (uint16_t)((float32_t)txDataUint32 * dataTxScaleCoef);
            }
                break;

            case GUI_LONG:      // long (int32_t)
            {
                int32_t *ptrDataInt32 = guiComVarsList[comTxIndex].varsAddress;
                int32_t txDataInt32 = *ptrDataInt32;
                comTxData = (int16_t)((float32_t)txDataInt32 * dataTxScaleCoef);
            }
                break;

            case GUI_FLOAT:     // float (float32_t)
            {
                float32_t *ptrDataFloat32 = guiComVarsList[comTxIndex].varsAddress;
                float32_t txDataFloat32 = *ptrDataFloat32;
                comTxData = (int16_t)(txDataFloat32 * dataTxScaleCoef);
            }
                break;
            default:
                break;
        }
    }

    return (comTxData);
}   // GUI_readTxDataList


//
// call the function in 1ms control loop
//
#pragma CODE_SECTION(GUI_clearCtrlFaults, "sysfuncs");
void GUI_clearCtrlFaults(void)
{
//    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)mtrHandle;
//    MOTOR_CtrlVars_t *objCtrl = (MOTOR_CtrlVars_t *)(motorVars_M1.motorCtrlHandle);
    if(motorCtrlVars_M1.controlCmdUse != motorCtrlVars_M1.controlCmdPrev)
    {
        motorCtrlVars_M1.controlCmdPrev = motorCtrlVars_M1.controlCmdUse;

        if(motorCtrlVars_M1.faultMotor.all != 0x0000)
        {
            motorVars_M1.flagClearFaults = 1;

            motorVars_M1.faultMtrNow.all &= MTR_FAULT_CLEAR;
            motorVars_M1.faultMtrUse.all = 0x0000;
        }
    }

    return;
}

#pragma CODE_SECTION(GUI_updateCtrlSate, "sysfuncs");
void GUI_updateCtrlSate(void)
{
    motorCtrlVars_M1.controlCmdUse = motorCtrlVars_M1.controlCmdRecv;

    switch(motorCtrlVars_M1.controlCmdUse)
    {
        case MOTOR_CTRL_IDLE_STATE:     //!< (0000), idle state for waiting command
        {
            motorCtrlVars_M1.flagIdentifyStatus = false;
            motorCtrlVars_M1.controlCmdPrev = motorCtrlVars_M1.controlCmdUse;
            // No any additional action
        }
            break;
        case MOTOR_CTRL_FREE_STOP:      //!< (0001), Stop the motor with free mode
        {
            motorCtrlVars_M1.flagIdentifyStatus = false;
            motorCtrlVars_M1.flagEnableRunMotor = false;
            motorCtrlVars_M1.controlCmdPrev = motorCtrlVars_M1.controlCmdUse;

        }
            break;
        case MOTOR_CTRL_BRAKE_STOP:      //!< (0010), Stop the motor with braking mode
        {
            motorCtrlVars_M1.flagIdentifyStatus = false;
            motorCtrlVars_M1.flagEnableRunMotor = false;
            motorCtrlVars_M1.controlCmdPrev = motorCtrlVars_M1.controlCmdUse;
        }
            break;
        case MOTOR_CTRL_URGENT_STOP:    //!< (0011), urgent stop
        {
            motorCtrlVars_M1.flagIdentifyStatus = false;
            motorCtrlVars_M1.flagEnableRunMotor = false;
            motorCtrlVars_M1.controlCmdPrev = motorCtrlVars_M1.controlCmdUse;
        }
            break;
        case MOTOR_CTRL_SPEED_CW:        //!< (0100), CW spin with speed closed-loop (Hz)
        {
            GUI_clearCtrlFaults();

            motorCtrlVars_M1.flagEnableRunMotor = true;
            motorCtrlVars_M1.flagEnableSpeedCtrl = true;
#if defined(MOTOR1_POWCTRL)
            motorCtrlVars_M1.flagEnablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL

            motorCtrlVars_M1.speedRef_Hz = motorCtrlVars_M1.speedSet_Hz;
            motorCtrlVars_M1.accelerationMax_Hzps = motorCtrlVars_M1.accelerationSet_Hzps;

            motorSetVars_M1.maxCurrent_A = motorSetVars_M1.maxCurrentSet_A;
        }
            break;
        case MOTOR_CTRL_SPEED_CCW:       //!< (0101), CCW spin with speed closed-loop (Hz)
        {
            GUI_clearCtrlFaults();

            motorCtrlVars_M1.flagEnableRunMotor = true;
            motorCtrlVars_M1.flagEnableSpeedCtrl = true;
#if defined(MOTOR1_POWCTRL)
            motorCtrlVars_M1.flagEnablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL

            motorCtrlVars_M1.speedRef_Hz = -motorCtrlVars_M1.speedSet_Hz;
            motorCtrlVars_M1.accelerationMax_Hzps = motorCtrlVars_M1.accelerationSet_Hzps;

            motorSetVars_M1.maxCurrent_A = motorSetVars_M1.maxCurrentSet_A;
        }
            break;
        case MOTOR_CTRL_TORQUE_CW:      //!< (0110), CW spin with torque control
        {
            GUI_clearCtrlFaults();

            motorCtrlVars_M1.flagEnableRunMotor = true;
            motorCtrlVars_M1.flagEnableSpeedCtrl = true;
#if defined(MOTOR1_POWCTRL)
            motorCtrlVars_M1.flagEnablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL

            motorCtrlVars_M1.speedRef_Hz = motorSetVars_M1.maxFrequency_Hz;
            motorSetVars_M1.maxCurrent_A = motorCtrlVars_M1.IqSet_A;
        }
            break;
        case MOTOR_CTRL_TORQUE_CCW:     //!< (0111), CCW spin with torque control
        {
            GUI_clearCtrlFaults();

            motorCtrlVars_M1.flagEnableRunMotor = true;
            motorCtrlVars_M1.flagEnableSpeedCtrl = true;
#if defined(MOTOR1_POWCTRL)
            motorCtrlVars_M1.flagEnablePowerCtrl = false;
#endif  // MOTOR1_POWCTRL

            motorCtrlVars_M1.speedRef_Hz = -motorSetVars_M1.maxFrequency_Hz;
            motorSetVars_M1.maxCurrent_A = motorCtrlVars_M1.IqSet_A;
        }
            break;
        case MOTOR_CTRL_POWER_CW:       //!< (1000), CW spin with speed closed-loop (rpm)
        {
            GUI_clearCtrlFaults();

            motorCtrlVars_M1.flagEnableRunMotor = true;
            motorCtrlVars_M1.flagEnableSpeedCtrl = true;

#if defined(MOTOR1_POWCTRL)
            motorCtrlVars_M1.flagEnablePowerCtrl = true;
            motorVars_M1.powerRef_W = motorCtrlVars_M1.powerSet_W;
            motorCtrlVars_M1.speedRef_Hz = motorSetVars_M1.maxFrequency_Hz;
#endif  // MOTOR1_POWCTRL

            motorSetVars_M1.maxCurrent_A = motorSetVars_M1.maxCurrentSet_A;
        }
            break;
        case MOTOR_CTRL_POWER_CCW:      //!< (1001), CCW spin with speed closed-loop (rpm)
        {
            GUI_clearCtrlFaults();

            motorCtrlVars_M1.flagEnableRunMotor = true;
            motorCtrlVars_M1.flagEnableSpeedCtrl = true;

#if defined(MOTOR1_POWCTRL)
            motorCtrlVars_M1.flagEnablePowerCtrl = true;
            motorCtrlVars_M1.speedRef_Hz = -motorSetVars_M1.maxFrequency_Hz;
#endif  // MOTOR1_POWCTRL

            motorSetVars_M1.maxCurrent_A = motorSetVars_M1.maxCurrentSet_A;
        }
            break;
        case MOTOR_CTRL_ID_AUTO:        //!< (1010), Motor parameters identification
        {
#if defined(MOTOR1_FAST)
            GUI_clearCtrlFaults();

            if(motorCtrlVars_M1.flagIdentifyStatus == false)
            {
                motorVars_M1.flagEnableMotorIdentify = true;
                motorCtrlVars_M1.flagIdentifyStatus = true;
                motorCtrlVars_M1.flagEnableRunMotor = true;
                motorCtrlVars_M1.controlStatus = MOTOR_ID_START;
            }
            else if(motorVars_M1.flagEnableMotorIdentify == false)
            {
                if(motorVars_M1.flagMotorIdentified == true)
                {
                    motorCtrlVars_M1.flagEnableRunMotor = false;
                    motorCtrlVars_M1.controlStatus = MOTOR_ID_DONE;
                }
                else
                {
                    motorCtrlVars_M1.controlStatus = MOTOR_ID_RUN;
                }
            }
#endif  //MOTOR1_FAST
        }
            break;
        case MOTOR_CTRL_ID_SET:         //!< (1011), Motor parameters identification
        {
#if defined(MOTOR1_FAST)
            GUI_clearCtrlFaults();

            if(motorCtrlVars_M1.flagIdentifyStatus == false)
            {
                motorVars_M1.flagEnableMotorIdentify = true;
                motorCtrlVars_M1.flagIdentifyStatus = true;
                motorCtrlVars_M1.flagEnableRunMotor = true;
                motorCtrlVars_M1.controlStatus = MOTOR_ID_START;
            }
            else if(motorVars_M1.flagEnableMotorIdentify == false)
            {
                if(motorVars_M1.flagMotorIdentified == true)
                {
                    motorCtrlVars_M1.flagEnableRunMotor = false;
                    motorCtrlVars_M1.controlStatus = MOTOR_ID_DONE;
                }
                else
                {
                    motorCtrlVars_M1.controlStatus = MOTOR_ID_RUN;
                }
            }
#endif  //MOTOR1_FAST
        }
            break;
        case MOTOR_CTRL_RS_AUTO:        //!< (1100), Rs recalculation
        {
#if defined(MOTOR1_FAST)
            GUI_clearCtrlFaults();

            if(motorCtrlVars_M1.flagIdentifyStatus == false)
            {
                motorVars_M1.flagEnableRsRecalc = true;
                motorCtrlVars_M1.flagIdentifyStatus = true;
                motorCtrlVars_M1.flagEnableRunMotor = true;
                motorCtrlVars_M1.controlStatus = MOTOR_RS_START;
            }
            else if(motorVars_M1.flagEnableMotorIdentify == false)
            {
                if(motorVars_M1.estState == EST_STATE_RS)
                {
                    motorCtrlVars_M1.flagEnableRunMotor = false;
                    motorCtrlVars_M1.controlStatus = MOTOR_RS_DONE;
                }
                else
                {
                    motorCtrlVars_M1.controlStatus = MOTOR_RS_RUN;
                }
            }
#endif  //MOTOR1_FAST
        }
            break;
        case MOTOR_CTRL_RS_SET:         //!< (1101), Rs recalculation
        {
#if defined(MOTOR1_FAST)
            GUI_clearCtrlFaults();

            if(motorCtrlVars_M1.flagIdentifyStatus == false)
            {
                motorVars_M1.flagEnableRsRecalc = true;
                motorCtrlVars_M1.flagIdentifyStatus = true;
                motorCtrlVars_M1.flagEnableRunMotor = true;
                motorCtrlVars_M1.controlStatus = MOTOR_RS_START;
            }
            else if(motorVars_M1.flagEnableMotorIdentify == false)
            {
                if(motorVars_M1.estState == EST_STATE_RS)
                {
                    motorCtrlVars_M1.flagEnableRunMotor = false;
                    motorCtrlVars_M1.controlStatus = MOTOR_RS_DONE;
                }
                else
                {
                    motorCtrlVars_M1.controlStatus = MOTOR_RS_RUN;
                }
            }
#endif  //MOTOR1_FAST
        }
            break;
        case MOTOR_CTRL_PRMS_RESET:     //!< (1110), Reset control parameters
        {
            // Reset motor control parameters
        }
            break;
        case MOTOR_CTRL_DEBUG_CHECK:    //!< (1111), debug monitor
        {
            if(motorCtrlVars_M1.controlCmdRecv <= MOTOR_CTRL_URGENT_STOP)
            {
                motorCtrlVars_M1.flagEnableRunMotor = false;        // Stop the motor
            }
        }
            break;
        default:
            // No any action
            break;
    }

    if(motorCtrlVars_M1.flagEnableGuiControl == true)
    {
        // flagEnableRunMotor->flagEnableRunAndIdentify->flagRunIdentAndOnLine->Enable start
        if((motorCtrlVars_M1.flagEnableRunMotor == true) && (motorVars_M1.faultMtrUse.all == 0))
        {
            motorVars_M1.flagEnableRunAndIdentify = true;
            motorVars_M1.flagEnableSpeedCtrl = motorCtrlVars_M1.flagEnableSpeedCtrl;

#if defined(MOTOR1_POWCTRL)
            motorVars_M1.flagEnablePowerCtrl = motorCtrlVars_M1.flagEnablePowerCtrl;
#endif  // MOTOR1_POWCTRL

            motorVars_M1.speedSet_Hz = motorCtrlVars_M1.speedRef_Hz;
            motorVars_M1.accelerationSet_Hzps = motorCtrlVars_M1.accelerationMax_Hzps;
        }
        else
        {
            motorVars_M1.flagEnableRunAndIdentify = false;
            motorVars_M1.speedSet_Hz = 0.0f;
        }
    }

    if(motorVars_M1.controlStatus <= MOTOR_BRAKE_RUN)
    {
        motorCtrlVars_M1.controlStatus = motorVars_M1.controlStatus;
    }

    // Update motor control status
    uint16_t ctrlStates = motorCtrlVars_M1.controlStatus;

    if(motorCtrlVars_M1.flagStatusUpdatePrms == true)
    {
        ctrlStates |=  MOTOR_PRMS_UPDATE_DONE;
    }

    if(motorVars_M1.flagStartRsOnLine == true)
    {
        ctrlStates |=  MOTOR_RSONLINE_RUN_ON;
    }

    if(motorVars_M1.flagStartFWC == true)
    {
        ctrlStates |=  MOTOR_FWC_RUN_ON;
    }

    motorCtrlVars_M1.motorCtrlStates = ctrlStates;
    motorCtrlVars_M1.faultMotor.all = motorVars_M1.faultMtrUse.all;

    return;
}

//
// Update received data in the background loop
// always call  this function when the background is free
//
#pragma CODE_SECTION(GUI_updateReceiveData, "sysfuncs");
void GUI_updateReceiveData(void)
{
    GUI_RTx_Status_e rxStatus = guiRxStatus;

    switch(rxStatus)
    {
        case GUI_RTX_IDLE:
            break;
        case GUI_RTX_GOING:
            // Wait
            break;
        case GUI_RTX_SUCCESS:
            guiRxStatus = GUI_RTX_IDLE;
            guiRxCommand = (GUI_Command_e)(guiRxDataBuf[0]);

            if(guiRxCommand == GUI_CODE_UPDATE)
            {
                // 0xA3[0]
                // + Code index[1][2]
                // + Code[3][4]
                // + Code[5][6]
                // + Code[7][8]
                // + Code[9][10]
                // Add code update function in here
            }
            else
            {
                GUI_dealReceiveData();
            }

            guiRxFailedTimesCnt = 0;

        case GUI_RTX_FAILED:
            guiRxStatus = GUI_RTX_IDLE;
            guiRxFailedTimesCnt++;

            if(guiRxFailedTimesCnt > GUI_RX_MAX_WAIT_TIMES)
            {
                guiEnableTRx = 0;
                guiRxFailedTimesCnt = 0;
                GUI_setupUart();

#if defined(BUADTUNE_EN)
                flagBaudRateTuned = 0;
#endif  // BUADTUNE_EN
            }

            // This time received data is not right, wait new correct data
            break;
        default:
            break;
    }

    return;
}   // GUI_updateReceiveData()

#pragma CODE_SECTION(GUI_dealReceiveData, "sysfuncs");
void GUI_dealReceiveData(void)
{
    switch(guiRxCommand)
    {
        case GUI_IDLE_BR:
        {
            // 0xA0[0]
            //+0xC0/0xCC
            //+ 0xAA[2]~[10]
#if defined(BUADTUNE_EN)
            if(flagBaudRateTuned == 2)
            {
                flagBaudRateTuned = 3;
            }
            else if(flagBaudRateTuned == 4)
            {
                flagBaudRateTuned = 0;
            }
#endif  // BUADTUNE_EN

        }
            break;
        case GUI_PING_CHK:
        {
            // 0xA1[0]
            // + Command[1]
            // + Platform Number[2]
            // + Volume Number[3]
            // + Motor Number[4]
            // + Data[5]~[10](=0, reserve)
            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);
            sysParmVars.platformNumber = guiRxDataBuf[2] & 0x00FF;
            sysParmVars.volumeNumber = guiRxDataBuf[3] & 0x00FF;
            sysParmVars.motorNumber = guiRxDataBuf[4] & 0x00FF;
        }
            break;
        case GUI_RESET_PRMS:
        {
            // 0xA7[0]
            // + Command[1]
            // + Reset mode[2]
            // + Data[3]~[14](=0, reserve)
            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);
            guiTxDataIndex[0] = guiRxDataBuf[3] & 0x00FF;
            guiTxDataIndex[1] = guiRxDataBuf[4] & 0x00FF;
            guiTxDataIndex[2] = guiRxDataBuf[5] & 0x00FF;

            motorCtrlVars_M1.updatePrmsDelayTime = UPDATE_PRMS_WAIT_TIME_SET;
            motorCtrlVars_M1.flagEnableUpdatePrms = true;
        }
            break;
        case GUI_CODE_UPDATE:
        {
            // 0xA3[0]
            // + Code index[1][2]
            // + Code[3][4]
            // + Code[5][6]
            // + Code[7][8]
            // + Code[9][10]
            // Add code update function in here
            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);
        }
            break;
        case GUI_M_TEMP_CHK:
        {
            // 0xA2[0]
            // + Command[1]
            // + Data[2]~[10](=0, reserve)
            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);
        }
            break;
        case GUI_MTR_ID_RS:
        {
            // 0xA4[0] + Command[1]
            // + Pole pairs[2]
            // + Maximum Current[3]~[4]
            // + Rs Current[5][6]
            // + Ind Current[7][8]
            // + Flux EST frequency[9][10]
            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);
        }
            break;
        case GUI_RUN_REQ_DATA:
        {
            // 0xA5[0] + Command[1]
            // + motor speed [2][3]
            // + acceleration[4][5]
            // + TX Data1 Index [6]
            // + TX Data1 [7][8]
            // + RX Data1 Index[9]
            // + RX Data2 Index[10]

            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);

            RxDataValue = ((guiRxDataBuf[2]<<8) & 0xFF00) + (guiRxDataBuf[3] & 0x00FF);
            GUI_writeDataFloat32(MOTORCTRLVARS_SPEEDSET_HZ_IND, RxDataValue);

            RxDataValue = ((guiRxDataBuf[4]<<8) & 0xFF00) + (guiRxDataBuf[5] & 0x00FF);
            GUI_writeDataFloat32(MOTORCTRLVARS_ACCELERATIONSET_HZPS_IND, RxDataValue);

            guiRxDataIndex[0] = guiRxDataBuf[6] & 0x00FF;
            RxDataValue = ((guiRxDataBuf[7]<<8) & 0xFF00) + (guiRxDataBuf[8] & 0x00FF);
            GUI_writeRxDataList(guiRxDataIndex[0], RxDataValue);

            if((guiRxDataIndex[0] == MOTORSETVARS_CONTROLTYPES_IND) ||
                    (guiRxDataIndex[0] == MOTORSETVARS_CONTROLFUNCS_IND))
            {
                GUI_updateCtrlSettings();
            }

            guiTxDataIndex[0] = guiRxDataBuf[9] & 0x00FF;
            guiTxDataIndex[1] = guiRxDataBuf[10] & 0x00FF;
        }
            break;
        case GUI_RUN_SET_PRMS:
        {
            // 0xA6[0] + Command[1]
            // + motor speed [2][3]
            // + TX Data1 Index [4]
            // + TX Data1 [5][6]
            // + TX Data2 Index[7]
            // + TX Data2 [8][9]
            // + RX Data1 Index[10]

            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);

            RxDataValue = ((guiRxDataBuf[2]<<8) & 0xFF00) + (guiRxDataBuf[3] & 0x00FF);
            GUI_writeDataFloat32(MOTORCTRLVARS_SPEEDSET_HZ_IND, RxDataValue);

            guiRxDataIndex[0] = guiRxDataBuf[4] & 0x00FF;
            RxDataValue = ((guiRxDataBuf[5]<<8) & 0xFF00) + (guiRxDataBuf[6] & 0x00FF);
            GUI_writeRxDataList(guiRxDataIndex[0], RxDataValue);

            if((guiRxDataIndex[0] == MOTORSETVARS_CONTROLTYPES_IND) ||
                    (guiRxDataIndex[0] == MOTORSETVARS_CONTROLFUNCS_IND))
            {
                GUI_updateCtrlSettings();
            }

            if((guiRxDataIndex[0] >= GUI_DATA_MOTOR_START_INDEX) &&
                    (guiRxDataIndex[0] <= GUI_DATA_MOTOR_END_INDEX))
            {
                motorCtrlVars_M1.updatePrmsDelayTime = UPDATE_PRMS_DELAY_TIME_SET;
                motorCtrlVars_M1.flagEnableUpdatePrms = true;
            }

            guiRxDataIndex[1] = guiRxDataBuf[7] & 0x00FF;
            RxDataValue = ((guiRxDataBuf[8]<<8) & 0xFF00) + (guiRxDataBuf[9] & 0x00FF);
            GUI_writeRxDataList(guiRxDataIndex[1], RxDataValue);

            if((guiRxDataIndex[1] == MOTORSETVARS_CONTROLTYPES_IND) ||
                    (guiRxDataIndex[1] == MOTORSETVARS_CONTROLFUNCS_IND))
            {
                GUI_updateCtrlSettings();
            }

            if((guiRxDataIndex[0] >= GUI_DATA_MOTOR_START_INDEX) &&
                    (guiRxDataIndex[0] <= GUI_DATA_MOTOR_END_INDEX))
            {
                motorCtrlVars_M1.updatePrmsDelayTime = UPDATE_PRMS_DELAY_TIME_SET;
                motorCtrlVars_M1.flagEnableUpdatePrms = true;
            }

            guiTxDataIndex[0] = guiRxDataBuf[10] & 0x00FF;
        }
            break;
        case GUI_M_TEST_DG1:
        case GUI_M_TEST_DG2:
        case GUI_M_TEST_DG3:
        case GUI_M_TEST_DG4:
        case GUI_M_TEST_DG5:
        case GUI_M_TEST_DG6:
        case GUI_M_TEST_DG7:
        case GUI_M_TEST_DGF:
        {
            // 0xA8~0xAF[0] + Command[1]
            // + Data1_Index[2]
            // + Data1[3][4]
            // + Data2_Index[5]
            // + Data2[6][7]
            // + Data3_Index[8]
            // + Data3[9][10]
            motorCtrlVars_M1.controlCmdRecv = (MOTOR_CtrlMode_e)(guiRxDataBuf[1] & 0x0F);

            RxDataIndex = guiRxDataBuf[2] & 0x00FF;
            RxDataValue = ((guiRxDataBuf[3]<<8) & 0xFF00) + (guiRxDataBuf[4] & 0x00FF);
            GUI_writeRxDataList(RxDataIndex, RxDataValue);

            RxDataIndex = guiRxDataBuf[5] & 0x00FF;
            RxDataValue = ((guiRxDataBuf[6]<<8) & 0xFF00) + (guiRxDataBuf[7] & 0x00FF);
            GUI_writeRxDataList(RxDataIndex, RxDataValue);

            RxDataIndex = guiRxDataBuf[8] & 0x00FF;
            RxDataValue = ((guiRxDataBuf[9]<<8) & 0xFF00) + (guiRxDataBuf[10] & 0x00FF);
            GUI_writeRxDataList(RxDataIndex, RxDataValue);
        }
            break;
        default:
        break;
    }

    return;
}

#pragma CODE_SECTION(GUI_updateCtrlSettings, "sysfuncs");
void GUI_updateCtrlSettings(void)
{
    if(motorCtrlVars_M1.controlTypes.all != motorSetVars_M1.controlTypes.all)
    {

        motorVars_M1.currentSenType  = motorSetVars_M1.controlTypes.bit.currentSenType;
        motorVars_M1.svmMode         = motorSetVars_M1.controlTypes.bit.svmMode;
        motorVars_M1.brakingMode     = motorSetVars_M1.controlTypes.bit.brakingMode;
        motorVars_M1.operationMode   = motorSetVars_M1.controlTypes.bit.operationMode;
        motorVars_M1.estimatorMode   = motorSetVars_M1.controlTypes.bit.estimatorMode;
        motorVars_M1.flyingStartMode = motorSetVars_M1.controlTypes.bit.flyingStartMode;
        motorVars_M1.RsOnlineMode    = motorSetVars_M1.controlTypes.bit.RsOnlineMode;
        motorVars_M1.CurrentSenDir   = motorSetVars_M1.controlTypes.bit.currentSenDir;
        motorVars_M1.startupMode     = motorSetVars_M1.controlTypes.bit.startupMode;

        motorCtrlVars_M1.controlTypes.all = motorSetVars_M1.controlTypes.all;
    }
    else if(motorCtrlVars_M1.controlFuncs.all != motorSetVars_M1.controlFuncs.all)
    {

        motorVars_M1.flagEnableRsRecalc     = motorSetVars_M1.controlFuncs.bit.enableRsRecalculate;
        motorVars_M1.flagEnableRsOnLine     = motorSetVars_M1.controlFuncs.bit.enableRsOnline;
        motorVars_M1.flagEnableMTPA         = motorSetVars_M1.controlFuncs.bit.enableMTPA;
        motorVars_M1.flagEnableFWC          = motorSetVars_M1.controlFuncs.bit.enableFieldWeakening;
        motorVars_M1.flagEnableDecoupleCtrl = motorSetVars_M1.controlFuncs.bit.enableDecoupleCtrl;
        motorVars_M1.flagEnablePirsControl  = motorSetVars_M1.controlFuncs.bit.enablePirsControl;
        motorVars_M1.flagEnableVibComp      = motorSetVars_M1.controlFuncs.bit.enableVibComp;
        motorVars_M1.flagEnableForceAngle   = motorSetVars_M1.controlFuncs.bit.enableForceAngle;
        motorVars_M1.flagEnableBrakingStop  = motorSetVars_M1.controlFuncs.bit.enableBrakingStop;
        motorVars_M1.flagEnableFlyingStart  = motorSetVars_M1.controlFuncs.bit.enableFlyingStart;
        motorVars_M1.flagEnableSSIPD        = motorSetVars_M1.controlFuncs.bit.enableSSIPD;
        motorVars_M1.flagEnableIPDHFI       = motorSetVars_M1.controlFuncs.bit.enableIPDHFI;
        motorVars_M1.flagEnableLsUpdate     = motorSetVars_M1.controlFuncs.bit.enableLsUpdate;
        motorVars_M1.flagEnableOVM          = motorSetVars_M1.controlFuncs.bit.enableOverModulation;

        if(motorSetVars_M1.controlFuncs.bit.enablePrmsUpdateSave == 1)
        {
            if(motorCtrlVars_M1.controlFuncs.bit.enablePrmsUpdateSave == 0)
            {
                motorSetVars_M1.dataPrmsFlashFlag = 1;
                motorCtrlVars_M1.updatePrmsDelayTime = UPDATE_PRMS_WAIT_TIME_SET;
            }
        }
        else if(motorCtrlVars_M1.controlStatus == MOTOR_PRMS_STORE)
        {
            motorCtrlVars_M1.controlStatus = MOTOR_STOP_IDLE;
        }
        else
        {
            motorCtrlVars_M1.flagStatusUpdatePrms = false;
        }

        motorCtrlVars_M1.controlFuncs.all = motorSetVars_M1.controlFuncs.all;
    }

    return;
}

#pragma CODE_SECTION(GUI_initCtrlSettings, "sysfuncs");
void GUI_initCtrlSettings(void)
{
    motorSetVars_M1.controlTypes.bit.estimatorType   = motorVars_M1.estimatorType;
    motorSetVars_M1.controlTypes.bit.currentSenType  = motorVars_M1.currentSenType;
    motorSetVars_M1.controlTypes.bit.svmMode         = motorVars_M1.svmMode;
    motorSetVars_M1.controlTypes.bit.brakingMode     = motorVars_M1.brakingMode;
    motorSetVars_M1.controlTypes.bit.operationMode   = motorVars_M1.operationMode;
    motorSetVars_M1.controlTypes.bit.estimatorMode   = motorVars_M1.estimatorMode;
    motorSetVars_M1.controlTypes.bit.flyingStartMode = motorVars_M1.flyingStartMode;
    motorSetVars_M1.controlTypes.bit.RsOnlineMode    = motorVars_M1.RsOnlineMode;
    motorSetVars_M1.controlTypes.bit.currentSenDir   = motorVars_M1.CurrentSenDir;
    motorSetVars_M1.controlTypes.bit.startupMode     = motorVars_M1.startupMode;

    motorCtrlVars_M1.controlTypes.all = motorSetVars_M1.controlTypes.all;

    motorSetVars_M1.controlFuncs.bit.enableRsRecalculate  = motorVars_M1.flagEnableRsRecalc;
    motorSetVars_M1.controlFuncs.bit.enableRsOnline       = motorVars_M1.flagEnableRsOnLine;
    motorSetVars_M1.controlFuncs.bit.enableMTPA           = motorVars_M1.flagEnableMTPA;
    motorSetVars_M1.controlFuncs.bit.enableFieldWeakening = motorVars_M1.flagEnableFWC;
    motorSetVars_M1.controlFuncs.bit.enableDecoupleCtrl   = motorVars_M1.flagEnableDecoupleCtrl;
    motorSetVars_M1.controlFuncs.bit.enablePirsControl    = motorVars_M1.flagEnablePirsControl;
    motorSetVars_M1.controlFuncs.bit.enableVibComp        = motorVars_M1.flagEnableVibComp;
    motorSetVars_M1.controlFuncs.bit.enableForceAngle     = motorVars_M1.flagEnableForceAngle;
    motorSetVars_M1.controlFuncs.bit.enableBrakingStop    = motorVars_M1.flagEnableBrakingStop;
    motorSetVars_M1.controlFuncs.bit.enableFlyingStart    = motorVars_M1.flagEnableFlyingStart;
    motorSetVars_M1.controlFuncs.bit.enableSSIPD          = motorVars_M1.flagEnableSSIPD;
    motorSetVars_M1.controlFuncs.bit.enableIPDHFI         = motorVars_M1.flagEnableIPDHFI;
    motorSetVars_M1.controlFuncs.bit.enableLsUpdate       = motorVars_M1.flagEnableLsUpdate;
    motorSetVars_M1.controlFuncs.bit.enableOverModulation = motorVars_M1.flagEnableOVM;

    motorSetVars_M1.controlFuncs.bit.enablePrmsUpdateSave = 0;

    motorCtrlVars_M1.controlFuncs.all = motorSetVars_M1.controlFuncs.all;

    return;
}

//
//
//
#pragma CODE_SECTION(GUI_writeRxDataList, "sysfuncs");

void GUI_writeRxDataList(uint16_t comRxIndex, int16_t comRxData)
{
    uint16_t listIndex = guiComVarsList[comRxIndex].varsIndex;

    if(comRxIndex == listIndex)
    {
        uint16_t dataSpecs = guiComVarsList[comRxIndex].varsSpecific;

        GUI_Data_Type_e dataType = (GUI_Data_Type_e)(dataSpecs & 0x00FF);
        GUI_Data_WR_e dataWR = (GUI_Data_WR_e)(dataSpecs & 0x0F00);

        if(dataWR >= GUI_WO)
        {
            float32_t dataScaleCoef = guiComVarsList[comRxIndex].varsRxScaleCoef;

            switch(dataType)
            {
                case GUI_UINT:      // unsigned int (uint16_t)
                {
                    uint16_t *ptrDataUint16 = guiComVarsList[comRxIndex].varsAddress;
                    *ptrDataUint16 = comRxData;
                }
                    break;
                case GUI_INT:       // int (int16_t)
                {
                    int16_t *ptrDataInt16 = guiComVarsList[comRxIndex].varsAddress;
                    *ptrDataInt16 = (int16_t)comRxData;
                }
                    break;

                case GUI_ULONG:     // unsigned long (uint32_t)
                {
                    uint32_t *ptrDataUint32 = guiComVarsList[guiRxListIndex].varsAddress;
                    *ptrDataUint32 = ((uint32_t)comRxData) * ((uint32_t)dataScaleCoef);
                }
                    break;

                case GUI_LONG:      // long (int32_t)
                {
                    int32_t *ptrDataInt32 = guiComVarsList[comRxIndex].varsAddress;
                    *ptrDataInt32 = ((int32_t)comRxData) * ((int32_t)dataScaleCoef);
                }
                    break;

                case GUI_FLOAT:     // float (float32_t)
                {
                    float32_t *ptrDataFloat32 = guiComVarsList[comRxIndex].varsAddress;
                    *ptrDataFloat32 =((float32_t)comRxData) * dataScaleCoef;
                }
                    break;
                default:
                    break;
            }
        }
    }

    return;
}   // GUI_writeRxDataList


//
// call the function in 1ms control loop
//
#pragma CODE_SECTION(GUI_initCtrlParms, "sysfuncs");

void GUI_initCtrlParms(void)
{
    motorCtrlVars_M1.accelerationMax_Hzps = USER_MOTOR1_ACCEL_RUN_Hzps;

    motorCtrlVars_M1.flagCmdRpmOrHz = false;            // the speed command is rpm
    motorCtrlVars_M1.flagEnableGuiControl = false;

    motorCtrlVars_M1.rpm2Hz_sf = userParams_M1.motor_numPolePairs / 60.0f;
    motorCtrlVars_M1.hz2Rpm_sf = 60.0f / userParams_M1.motor_numPolePairs;

    return;
}

//
//
//
#pragma CODE_SECTION(GUI_initUartParams, "guifuncs");

void GUI_initUartParams(void)
{
    guiEnableTRx = false;
    guiTRxWaitTimeCnt = 0;
    // USER_M1_PWM_FREQ_kHz = 10.0f -> 100us
    guiTRxWaitTimeSet = (uint16_t)GUI_TRX_WAIT_TIME_SET;

    guiRxStatus = GUI_RTX_IDLE;
    guiTxStatus = GUI_RTX_IDLE;

    guiRxListIndex = 0;
    guiTxListIndex = 0;

    guiRxWaitTimeCnt = 0;
    guiTxWaitTimeCnt = 0;
    guiRxWaitTimeSet = GUI_RX_WAIT_MAX_TIME;
    guiTxWaitTimeSet = GUI_TX_WAIT_MAX_TIME;

    guiRxFailedTimesCnt = 0;
    guiTxFailedTimesCnt = 0;

    guiRxDataChkSum = 0;
    guiTxDataChkSum = 0;

    guiRxDataIndex[0] = MOTORCTRLVARS_SPEEDSET_RPM_IND;
    guiRxDataIndex[1] = MOTORCTRLVARS_SPEEDSET_RPM_IND;
    guiRxDataIndex[2] = MOTORCTRLVARS_SPEEDSET_RPM_IND;

    guiTxDataIndex[0] = MOTORVARS_ADCDATA_VDCBUS_V_IND;
    guiTxDataIndex[1] = MOTORVARS_ADCDATA_VDCBUS_V_IND;
    guiTxDataIndex[2] = MOTORVARS_ADCDATA_VDCBUS_V_IND;


    motorCtrlVars_M1.controlCmdRecv = MOTOR_CTRL_IDLE_STATE;
    motorCtrlVars_M1.controlCmdUse = MOTOR_CTRL_IDLE_STATE;

    motorCtrlVars_M1.torqueSet_Nm = USER_MOTOR1_TORQUE_SET_NM;
    motorCtrlVars_M1.powerSet_W = USER_MOTOR1_POWWR_SET_W;
    motorCtrlVars_M1.IqSet_A = USER_MOTOR1_IS_Q_SET_A;
    motorCtrlVars_M1.speedSet_Hz = USER_MOTOR1_SPEED_SET_Hz;
    motorCtrlVars_M1.accelerationSet_Hzps = USER_MOTOR1_ACCEL_SET_Hzps;

    return;
}

//
//
//
#pragma CODE_SECTION(GUI_setupUart, "guifuncs");

void GUI_setupUart(void)
{
    // Initialize SCIC and its FIFO.
    SCI_performSoftwareReset(GUI_SCI_BASE);

    // Configure SCIC for echoback.
    SCI_setConfig(GUI_SCI_BASE, DEVICE_LSPCLK_FREQ, GUI_BAUD_RATE,
                        ( SCI_CONFIG_WLEN_8 |
                          SCI_CONFIG_STOP_ONE |
                          SCI_CONFIG_PAR_NONE ) );

    SCI_enableModule(GUI_SCI_BASE);
    SCI_resetChannels(GUI_SCI_BASE);
    SCI_enableFIFO(GUI_SCI_BASE);

    // RX and TX FIFO Interrupts Enabled
    SCI_enableInterrupt(GUI_SCI_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));
    SCI_disableInterrupt(GUI_SCI_BASE, SCI_INT_RXERR);

    SCI_clearInterruptStatus(GUI_SCI_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

    // The transmit FIFO generates an interrupt when FIFO status
    // bits are less than or equal to 2 out of 16 words
    // The receive FIFO generates an interrupt when FIFO status
    // bits are greater than equal to 2 out of 16 words

#if !defined(GUI_SCI_AI_TEST)
    SCI_setFIFOInterruptLevel(GUI_SCI_BASE, SCI_FIFO_TX12, SCI_FIFO_RX12);
#else   // GUI_SCI_AI_TEST
    SCI_setFIFOInterruptLevel(GUI_SCI_BASE, SCI_FIFO_TX15, SCI_FIFO_RX15);
#endif  // GUI_SCI_AI_TEST

    SCI_performSoftwareReset(GUI_SCI_BASE);
    SCI_resetRxFIFO(GUI_SCI_BASE);
    SCI_resetTxFIFO(GUI_SCI_BASE);

    return;
}
#endif  // GUI_SCI_EN & _F28003x
//
// End of File
//
