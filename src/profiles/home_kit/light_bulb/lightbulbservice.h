/******************************************************************************

 @file  lightbulbservice.h

 @brief This file contains the HAP Lightbulb service definitions and prototypes
        prototypes.

        This code is written by reference to HAP specifications from
        Apple Inc.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2015-2018, Texas Instruments Incorporated
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

#ifndef LIGHTBULBSERVICE_H
#define LIGHTBULBSERVICE_H

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

// HAP Lightbulb Service UUID (Bluetooth base UUID)
#define HAP_LIGHTBULB_SERV_UUID         0xFED2

// HAP Lightbulb Characteristic UUIDs (HAP base UUIDs)
#define HAP_LIGHTBULB_ON_UUID           0x25
#define HAP_SERV_INSTANCE_UUID          0x51

// Length of HAP instance characteristic. The value of an instance
// characteristic is its instance ID encoded as a UTF-8 string. Instance
// IDs start at 1 and have a maximum of (2^64)-1. A maximum of (2^16)-1,
// 65536, is used instead for now.
#define HAP_INSTANCE_CHAR_LEN           6

// Pairing features supported by the accessory
#define HAP_FEATURE_PAIR_SETUP          0x00
#define HAP_FEATURE_MFI_PAIR_SETUP      0x01

// Pairing Service Get/Set Parameters
#define HAP_PAIRING_FEATURES            1

// Callback events
#define HAP_PAIRING_NOTI_ENABLED        1
#define HAP_PAIRING_NOTI_DISABLED       2

/*********************************************************************
 * TYPEDEFS
 */

// Lightbulb Service callback function
typedef void (*lightbulbServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
typedef void (*lightbulbChange_t)( uint8 paramID );

typedef struct
{
  lightbulbChange_t        pfnLightbulbChange;  // Called when characteristic value changes
} lightbulbCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      Lightbulb_AddService
 *
 * @brief   Initializes the HAP Lightbulb service by registering GATT
 *          attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t Lightbulb_AddService(void);

/*********************************************************************
 * @fn      Lightbulb_Register
 *
 * @brief   Register a callback function with the HAP Lightbulb Serivce.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Lightbulb_Register(lightbulbServiceCB_t pfnServiceCB);

/*********************************************************************
 * @fn      Lightbulb_SetParameter
 *
 * @brief   Set an HAP Lightbulb Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Lightbulb_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      Lightbulb_GetParameter
 *
 * @brief   Get an HAP Lightbulb parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Lightbulb_GetParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* LIGHTBULBSERVICE_H */
