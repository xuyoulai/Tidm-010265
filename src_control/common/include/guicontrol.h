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
//! \file   /solutions/tida_010265_wminv/common/include/guicontrol.h
//!
//! \brief  header file to be included in all labs
//!         implement GU for motor control with F28002x/F28003x/F280013x
//!
//------------------------------------------------------------------------------


#ifndef GUI_CONTROL_H
#define GUI_CONTROL_H


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

#include "motor_common.h"
#include "guidatalist.h"

//
// Defines
//

#define GUI_RTX_TEST_ENABLE         1
#define GUI_RTX_TEST_DISABLE        0

#define GUI_RTX_TEST                GUI_RTX_TEST_DISABLE

#if !defined(GUI_SCI_AI_TEST)
#define GUI_RX_DATA_LENGTH          (12U)
#define GUI_TX_DATA_LENGTH          (12U)

#define GUI_RX_BUFF_LENGTH          (12U)
#define GUI_TX_BUFF_LENGTH          (12U)
#else   // GUI_SCI_AI_TEST
#define GUI_RX_DATA_LENGTH          (15U)
#define GUI_TX_DATA_LENGTH          (15U)

#define GUI_RX_BUFF_LENGTH          (15U)
#define GUI_TX_BUFF_LENGTH          (15U)
#endif  // GUI_SCI_AI_TEST

#define GUI_RX_WAIT_MAX_TIME        36000              // 10(ISRs)= 3s
#define GUI_TX_WAIT_MAX_TIME        36000              // 10(ISRs)= 3s

#define GUI_RX_BYTE_BITS            (10U)
#define GUI_TX_BYTE_BITS            (10U)

#define GUI_RX_FRAME_BITS           (GUI_RX_DATA_LENGTH * GUI_RX_BYTE_BITS)
#define GUI_TX_FRAME_BITS           (GUI_TX_DATA_LENGTH * GUI_TX_BYTE_BITS)

#define GUI_TX_WAIT_TIME_OFFSET     (1.05f)    // at least 1.0f
#define GUI_RX_MAX_WAIT_TIMES       (20U)

#define GUI_TRX_WAIT_TIME_CAL       ((((float32_t)GUI_TX_FRAME_BITS) / USER_M1_ISR_PERIOD_sec) / ((float32_t)GUI_BAUD_RATE))

#define GUI_TRX_WAIT_TIME_SET       (GUI_TRX_WAIT_TIME_CAL + GUI_TX_WAIT_TIME_OFFSET)


//
// Typedefs
//
typedef enum
{
    GUI_IDLE_BR       = 0xA0,
    GUI_PING_CHK      = 0xA1,
    GUI_RESET_PRMS    = 0xA2,
    GUI_CODE_UPDATE   = 0xA3,
    GUI_M_TEMP_CHK    = 0xA4,
    GUI_MTR_ID_RS     = 0xA5,
    GUI_RUN_REQ_DATA  = 0xA6,
    GUI_RUN_SET_PRMS  = 0xA7,
    GUI_M_TEST_DG1    = 0xA8,
    GUI_M_TEST_DG2    = 0xA9,
    GUI_M_TEST_DG3    = 0xAA,
    GUI_M_TEST_DG4    = 0xAB,
    GUI_M_TEST_DG5    = 0xAC,
    GUI_M_TEST_DG6    = 0xAD,
    GUI_M_TEST_DG7    = 0xAE,
    GUI_M_TEST_DGF    = 0xAF
} GUI_Command_e;

typedef enum
{
    GUI_RTX_IDLE        = 0,    // Idle, wait receive/transmit
    GUI_RTX_GOING       = 1,    // In receive/transmit process
    GUI_RTX_SUCCESS     = 2,    // receive/transmit
    GUI_RTX_FAILED      = 3     // receive/transmit Failed
} GUI_RTx_Status_e;


//
// the modules
//

//
// the globals
//
#if defined(GUI_SCI_EN)

extern uint16_t RxDataIndex;
extern uint16_t TxDataIndex;

extern uint16_t RxDataValue;
extern int16_t TxDataValue;

extern uint16_t guiRxDataIndex[3];
extern uint16_t guiTxDataIndex[3];

extern uint16_t guiRxListIndex;
extern uint16_t guiTxListIndex;

extern uint16_t guiRxDataChkSum;
extern uint16_t guiTxDataChkSum;

extern uint16_t guiRxDataFifo[GUI_RX_BUFF_LENGTH];     // receiving data
extern uint16_t guiRxDataBuf[GUI_RX_BUFF_LENGTH];      // dealing data

extern uint16_t guiTxDataFifo[GUI_TX_BUFF_LENGTH];     // ready to send
extern uint16_t guiTxDataBuf[GUI_TX_BUFF_LENGTH];      // transmitting data


extern GUI_RTx_Status_e guiRxStatus;
extern GUI_RTx_Status_e guiTxStatus;

extern GUI_Command_e guiRxCommand;
extern GUI_Command_e guiTxCommand;

extern uint16_t guiRxDataChkSum;
extern uint16_t guiTxDataChkSum;

extern bool guiEnableTRx;
extern uint16_t guiTRxWaitTimeCnt;
extern uint16_t guiTRxWaitTimeSet;

extern uint16_t guiRxWaitTimeCnt;
extern uint16_t guiTxWaitTimeCnt;
extern uint16_t guiRxWaitTimeSet;
extern uint16_t guiTxWaitTimeSet;

extern uint16_t guiRxFailedTimesCnt;
extern uint16_t guiTxFailedTimesCnt;

extern uint16_t guiRxDataIndex[3];
extern uint16_t guiTxDataIndex[3];

extern uint16_t guiRxListIndex;
extern uint16_t guiTxListIndex;


// the functions
extern void GUI_initUartParams(void);
extern void GUI_setupUart(void);

extern void GUI_initCtrlParms(void);
extern void GUI_updateCtrlSate(void);
extern void GUI_clearCtrlFaults(void);

// the functions
extern void GUI_updateReceiveData(void);
extern void GUI_dealReceiveData(void);
extern void GUI_updateCtrlSettings(void);
extern void GUI_updateTransmitData(void);
extern void GUI_initCtrlSettings(void);


extern void GUI_writeRxDataList(uint16_t comRxIndex, int16_t comRxData);
extern int16_t GUI_readTxDataList(uint16_t comTxIndex);

inline void GUI_writeDataFloat32(uint16_t index, int16_t writeData)
{
    float32_t dataRxScaleCoef = guiComVarsList[index].varsRxScaleCoef;

    float32_t *ptrDataFloat32 = guiComVarsList[index].varsAddress;
    *ptrDataFloat32 = (((float32_t)writeData) * dataRxScaleCoef);
}   // GUI_writeDataFloat32


inline int16_t GUI_readDataFloat32(uint16_t index)
{
    float32_t *ptrDataFloat32 = guiComVarsList[index].varsAddress;

    float32_t txDataFloat32 = *ptrDataFloat32;
    float32_t dataTxScaleCoef = guiComVarsList[index].varsTxScaleCoef;
    int16_t readTxData = (int16_t)(txDataFloat32 * dataTxScaleCoef);

    return(readTxData);
}   // GUI_readDataFloat32


inline void GUI_writeTxData(void)
{
    guiTxWaitTimeCnt++;

    if(SCI_getTxFIFOStatus(GUI_SCI_BASE) == SCI_FIFO_TX0)
    {
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[0]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[1]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[2]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[3]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[4]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[5]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[6]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[7]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[8]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[9]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[10]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[11]);

#if defined(GUI_SCI_AI_TEST)
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[12]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[13]);
        SCI_writeCharNonBlocking(GUI_SCI_BASE, guiTxDataBuf[14]);
#endif  // GUI_SCI_TEST

        SCI_clearInterruptStatus(GUI_SCI_BASE, SCI_INT_TXFF);

        guiTxStatus = GUI_RTX_SUCCESS;
        guiTxWaitTimeCnt = 0;

    }
    else if(guiTxWaitTimeCnt > guiTxWaitTimeSet)
    {
        guiTxStatus = GUI_RTX_FAILED;
        guiTxWaitTimeCnt = 0;
        guiEnableTRx = false;

        SCI_resetTxFIFO(GUI_SCI_BASE);
    }
}


inline void GUI_readRxData(void)
{
    guiRxWaitTimeCnt++;

#if !defined(GUI_SCI_AI_TEST)
    if(SCI_getRxFIFOStatus(GUI_SCI_BASE) == SCI_FIFO_RX12)
#else   // GUI_SCI_AI_TEST
    if(SCI_getRxFIFOStatus(GUI_SCI_BASE) == SCI_FIFO_RX15)
#endif  // GUI_SCI_AI_TEST
    {
        guiRxDataBuf[0]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum = guiRxDataBuf[0];

        guiRxDataBuf[1]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[1];

        guiRxDataBuf[2]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[2];

        guiRxDataBuf[3]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[3];

        guiRxDataBuf[4]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[4];

        guiRxDataBuf[5]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[5];

        guiRxDataBuf[6]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[6];

        guiRxDataBuf[7]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[7];

        guiRxDataBuf[8]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[8];

        guiRxDataBuf[9]  = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[9];

        guiRxDataBuf[10] = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[10];

#if !defined(GUI_SCI_AI_TEST)
        guiRxDataBuf[11] = SCI_readCharBlockingFIFO(GUI_SCI_BASE);

        guiRxDataChkSum &=0x00FF;

        if((guiRxDataChkSum == guiRxDataBuf[11]) && ((guiRxDataBuf[0] & 0xF0) == 0xA0))
        {
            guiEnableTRx = true;
            guiRxStatus = GUI_RTX_SUCCESS;
            guiRxWaitTimeCnt = 0;
        }
#else   // GUI_SCI_AI_TEST
        guiRxDataBuf[11] = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[11];

        guiRxDataBuf[12] = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[12];

        guiRxDataBuf[13] = SCI_readCharBlockingFIFO(GUI_SCI_BASE);
        guiRxDataChkSum += guiRxDataBuf[13];

        guiRxDataBuf[14] = SCI_readCharBlockingFIFO(GUI_SCI_BASE);

        guiRxDataChkSum &=0x00FF;

        if((guiRxDataChkSum == guiRxDataBuf[14]) && ((guiRxDataBuf[0] & 0xFF) == guiTxMtrDataID))
        {
            guiEnableTRx = true;
            guiRxStatus = GUI_RTX_SUCCESS;
            guiRxWaitTimeCnt = 0;
        }
#endif  // GUI_SCI_AI_TEST

        SCI_clearOverflowStatus(GUI_SCI_BASE);
        SCI_clearInterruptStatus(GUI_SCI_BASE, SCI_INT_RXFF);

    }
    else if(guiRxWaitTimeCnt > guiRxWaitTimeSet)
    {
        guiRxStatus = GUI_RTX_FAILED;
        guiRxWaitTimeCnt = 0;

        // Reset the RX
        SCI_resetRxFIFO(GUI_SCI_BASE);
        SCI_enableFIFO(GUI_SCI_BASE);
    }
}
#endif  // GUI_SCI_EN

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

#endif // end of GUI_CONTROL_H defines

//
// End of File
//
