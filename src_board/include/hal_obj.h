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
//! \file   \solutions\universal_motorcontrol_lab\f280013x\drivers\include\hal_obj.h
//! \brief  Defines the structures for the HAL object
//!
//------------------------------------------------------------------------------

#ifndef HAL_OBJ_H
#define HAL_OBJ_H


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
//! \defgroup HAL HALL_OBJ
//! @{
//
//*****************************************************************************

// drivers
#include "device.h"

// modules
#include "hal_data.h"


// platforms

// Defines for I2C
#define I2C_SLAVE_ADDRESS           0x50

//!  \brief  Defines the pulse width modulation digital-to-analog (PWMDAC) handle
//!
#define   PWMDAC_Handle                             PWM_Handle

//!  \brief  Links the PWMDAC_setPeriod() function to the PWM_setPeriod() function
//!
#define   PWMDAC_getPeriod                          EPWM_getTimeBasePeriod

//!  \brief  Links the PWMDAC_init() function to the PWM_init() function
//!
#define   PWMDAC_init                               PWM_init

//! \brief Enumeration to define the pulse width modulation digital-to-analog (PWM) numbers
//!
typedef enum
{
  PWMDAC_NUMBER_1 = 0,
  PWMDAC_NUMBER_2 = 1,
  PWMDAC_NUMBER_3 = 2
} PWMDAC_Number_e;

//! \brief Enumeration for the GPIO for SPI_CS
//!
typedef enum
{
  SPI_CS_NSC = 0,    // Not a device connected to the SPI
  SPI_CS_DRV = 1,    // A DRV device is connecting to the SPI
  SPI_CS_DAC = 2     // A DAC device is connecting to the SPI
} SPI_CS_SEL_e;

//! \brief      Defines the HAL object
//!
typedef struct _HAL_MTR_Obj_
{
  bool           flagEnablePWM;         //<! the pwm enable flag

  MotorNum_e     motorNum;

  SPI_CS_SEL_e   selectSPICS;           //!< the selection of SPICS pin

  uint16_t       numCurrentSensors;     //!< the number of current sensors
  uint16_t       numVoltageSensors;     //!< the number of voltage sensors

  uint16_t       pwmPeriodCycles;
  uint16_t       numPWMTicksPerISRTick;

  uint32_t       pwmHandle[3];          //<! the PWM handles

#if defined(MOTOR1_DCLINKSS)    // Single shunt
  uint32_t       cmpssHandle[1];     //!< the CMPSS handle
#else   // !(MOTOR1_DCLINKSS)   // 2/3 shunt
  uint32_t       cmpssHandle[3];     //!< the CMPSS handle
#endif  // !(MOTOR1_DCLINKSS)   // 2/3 shunt

#if defined(HVMTRPFC_REV1P1)
  uint32_t       gateEnableGPIO;
  // HVMTRPFC_REV1P1
#elif defined(BSXL8323RH_REVB)
  uint32_t       gateEnableGPIO;
  uint32_t       gateGainGPIO;
  // BSXL8323RH_REVB
#elif defined(DRV8329AEVM_REVA)
  uint32_t       gateEnableGPIO;
  uint32_t       gateSleepGPIO;
  // DRV8329AEVM_REVA
#endif  // HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA

#if defined(CMD_CAP_EN)
  uint32_t       capHandle;           //<! the CAP handles
#endif  // CMD_CAP_EN

} HAL_MTR_Obj;

//! \brief      Defines the HAL_MTR handle
//! \details    The HAL_MTR handle is a pointer to a HAL_MTR object.  In all
//!             HAL_MTR functions, the HAL_MTR handle is passed so that the
//!             function knows what peripherals are to be accessed.
//!
typedef struct _HAL_MTR_Obj_ *HAL_MTR_Handle;


//! \brief      Defines the hardware abstraction layer (HAL) data
//! \details    The HAL object contains all handles to peripherals.  When accessing a
//!             peripheral on a processor, use a HAL function along with the HAL handle
//!             for that processor to access its peripherals.
//!
typedef struct _HAL_Obj_
{
  uint32_t       adcHandle[2];      //!< the ADC handles
  uint32_t       adcResult[2];      //!< the ADC results

  uint32_t       timerHandle[3];    //<! the timer handles
  uint32_t       sciHandle[2];      //!< the SCI handle, SCIA & SCIC
  uint32_t       linHandle;         //!< the LIN handle
  uint32_t       i2cHandle;         //!< the I2C handle

  uint32_t       spiHandle[1];      //!< the SPI handle, SPIA

  HAL_MTR_Handle mtrHandle;         //!< the motor control interface handle

#if defined(EPWMDAC_MODE)
  // RC(2.2k/220nF) or RC(1k/470nF) on PWM output pin
#if defined(WMINVBRD_REV1P0) || defined(HVMTRPFC_REV1P1) || \
    defined(BSXL8323RH_REVB) || defined(DRV8329AEVM_REVA)
  uint32_t       pwmDACHandle[3];   //<! the PWMDAC handles
  // (WMINVBRD_REV1P0 | HVMTRPFC_REV1P1 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#else   // !(HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#error EPWMDAC is not supported on this kit!
#endif  // !(HVMTRPFC_REV1P1 | WMINVBRD_REV1P0 | BSXL8323RH_REVB | DRV8329AEVM_REVA)
#endif  // EPWMDAC_MODE

} HAL_Obj;

//! \brief      Defines the HAL handle
//! \details    The HAL handle is a pointer to a HAL object.  In all HAL functions
//!             the HAL handle is passed so that the function knows what peripherals
//!             are to be accessed.
//!
typedef struct _HAL_Obj_ *HAL_Handle;


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of HAL_OBJ_H definition

