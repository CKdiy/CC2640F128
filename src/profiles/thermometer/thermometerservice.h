/******************************************************************************

 @file  thermometerservice.h

 @brief This file contains the Thermometer service definitions and prototypes
        prototypes.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2011-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

#ifndef THERMOMETERSERVICE_H
#define THERMOMETERSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Thermometer Service Parameters
#define THERMOMETER_TEMP                      0
#define THERMOMETER_TEMP_CHAR_CFG             1
#define THERMOMETER_TYPE                      2
#define THERMOMETER_INTERVAL                  3
#define THERMOMETER_INTERVAL_CHAR_CFG         4
#define THERMOMETER_IMEAS_CHAR_CFG            5
#define THERMOMETER_IRANGE                    6

// Position  in attribute array
#define THERMOMETER_TEMP_VALUE_POS            2
#define THERMOMETER_TEMP_CHAR_CONFIG_POS      3
#define THERMOMETER_IMEAS_VALUE_POS           7
#define THERMOMETER_IMEAS_CHAR_CONFIG_POS     8
#define THERMOMETER_INTERVAL_VALUE_POS        10
#define THERMOMETER_INTERVAL_CHAR_CONFIG_POS  11

  // Length of bytes
#define THERMOMETER_INTERVAL_LEN              2
#define THERMOMETER_TYPE_LEN                  1
#define THERMOMETER_IRANGE_LEN                (2 + 2) // low + high

// Maximum length of thermometer
// measurement characteristic
#define THERMOMETER_MEAS_MAX                  (ATT_MTU_SIZE -5)

// Values for flags
#define THERMOMETER_FLAGS_CELCIUS             0x00
#define THERMOMETER_FLAGS_FARENHEIT           0x01
#define THERMOMETER_FLAGS_TIMESTAMP           0x02
#define THERMOMETER_FLAGS_TYPE                0x04

// Values for sensor location
#define THERMOMETER_TYPE_ARMPIT               0x01
#define THERMOMETER_TYPE_BODY                 0x02
#define THERMOMETER_TYPE_EAR                  0x03
#define THERMOMETER_TYPE_FINGER               0x04
#define THERMOMETER_TYPE_GASTRO               0x05
#define THERMOMETER_TYPE_MOUTH                0x06
#define THERMOMETER_TYPE_RECTUM               0x07
#define THERMOMETER_TYPE_TOE                  0x08
#define THERMOMETER_TYPE_TYMPNUM              0x09

// Thermometer Service bit fields
#define THERMOMETER_SERVICE                   0x00000001

// Callback events
#define THERMOMETER_TEMP_IND_ENABLED          1
#define THERMOMETER_TEMP_IND_DISABLED         2
#define THERMOMETER_IMEAS_NOTI_ENABLED        3
#define THERMOMETER_IMEAS_NOTI_DISABLED       4
#define THERMOMETER_INTERVAL_IND_ENABLED      5
#define THERMOMETER_INTERVAL_IND_DISABLED     6
#define THERMOMETER_INTERVAL_SET              7
#define THERMOMETER_TESTCMD_C                 8
#define THERMOMETER_TESTCMD_F                 9

/*********************************************************************
 * TYPEDEFS
 */

// Thermometer Service callback function
typedef void (*thermometerServiceCB_t)(uint8 event);

// Thermometer Interval Range
typedef struct
{
  uint16 low;
  uint16 high;
} thermometerIRange_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*
 * Thermometer_AddService- Initializes the Thermometer service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
extern bStatus_t Thermometer_AddService(uint32 services);

/*
 * Thermometer_Register - Register a callback function with the
 *          Thermometer Service
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Thermometer_Register(thermometerServiceCB_t pfnServiceCB);

/*
 * Thermometer_SetParameter - Set a Thermometer parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len   - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Thermometer_SetParameter(uint8 param, uint8 len,
                                          void *value);

/*
 * Thermometer_GetParameter - Get a Thermometer parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Thermometer_GetParameter(uint8 param, void *value);

/*********************************************************************
 * @fn          Thermometer_TempIndicate
 *
 * @brief       Send a notification containing a thermometer
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 * @param       taskId - application's taskId
 *
 * @return      Success or Failure
 */
extern bStatus_t Thermometer_TempIndicate(uint16 connHandle,
                                          attHandleValueInd_t *pNoti,
                                          uint8 taskId);

/*********************************************************************
 * @fn          Thermometer_IntervalIndicate
 *
 * @brief       Send a interval change indication
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 * @param       taskId - application's taskId
 *
 * @return      Success or Failure
 */
extern bStatus_t Thermometer_IntervalIndicate(uint16 connHandle,
                                              attHandleValueInd_t *pNoti,
                                              uint8 taskId);

/*********************************************************************
 * @fn          Thermometer_IMeasNotify
 *
 * @brief       Send a intermediate temperature notification
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t Thermometer_IMeasNotify(uint16 connHandle,
                                         attHandleValueNoti_t *pNoti);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* THERMOMETERSERVICE_H */
