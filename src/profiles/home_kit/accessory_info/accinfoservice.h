/******************************************************************************

 @file  accinfoservice.h

 @brief This file contains the HAP Accessory Information service definitions
        and prototypes.

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

#ifndef ACCINFOSERVICE_H
#define ACCINFOSERVICE_H

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

// HAP Access Information Service UUID (Bluetooth base UUID)
#define HAP_ACCINFO_SERV_UUID           0xFED3

// HAP Access Information Characteristic UUIDs (HAP base UUIDs)
#define HAP_ACCINFO_IDENTIFY_UUID       0x14
#define HAP_ACCINFO_MANUFACT_UUID       0x20
#define HAP_ACCINFO_MODEL_UUID          0x21
#define HAP_ACCINFO_NAME_UUID           0x23
#define HAP_ACCINFO_SERIAL_NO_UUID      0x30
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

  // HAP Pairing Service UUID (Bluetooth base UUID)
#define HAP_PAIRING_SERV_UUID           0xFED4

// HAP Pairing Characteristic UUIDs (HAP base UUIDs)
#define HAP_PAIR_SETUP_UUID             0x4C
#define HAP_PAIR_VERIFY_UUID            0x4E
#define HAP_PAIR_FEATURES_UUID          0x4F
#define HAP_PAIR_PAIRINGS_UUID          0x50
#define HAP_SERV_INSTANCE_UUID          0x51

/*********************************************************************
 * TYPEDEFS
 */

// Accessory Information Service callback function
typedef void (*accInfoServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      AccInfo_AddService
 *
 * @brief   Initializes the HAP Accessory Information service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t AccInfo_AddService(void);

/*********************************************************************
 * @fn      AccInfo_Register
 *
 * @brief   Register a callback function with the HAP Accessory Information
 *          Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void AccInfo_Register(accInfoServiceCB_t pfnServiceCB);

/*********************************************************************
 * @fn      AccInfo_SetParameter
 *
 * @brief   Set an HAP Accessory Information Service parameter.
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
extern bStatus_t AccInfo_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      AccInfo_GetParameter
 *
 * @brief   Get an HAP Accessory Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t AccInfo_GetParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ACCINFOSERVICE_H */
