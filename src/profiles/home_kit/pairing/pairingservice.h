/******************************************************************************

 @file  pairingservice.h

 @brief This file contains the HAP Pairing service definitions and prototypes
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

#ifndef PAIRINGSERVICE_H
#define PAIRINGSERVICE_H

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

// HAP Pairing Service UUID (Bluetooth base UUID)
#define HAP_PAIRING_SERV_UUID           0xFED4

// HAP Pairing Characteristic UUIDs (HAP base UUIDs)
#define HAP_PAIR_SETUP_UUID             0x4C
#define HAP_PAIR_VERIFY_UUID            0x4E
#define HAP_PAIR_FEATURES_UUID          0x4F
#define HAP_PAIR_PAIRINGS_UUID          0x50
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

// Pairing Service callback function
typedef void (*pairingServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

// Bluetooth base 128-bit UUID: 00000000-0000-1000-8000-00805F9B34FB
#define BT_BASE_UUID_128( uuid )  0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, \
                                  0x00, 0x80, 0x00, 0x10, 0x00, 0x00, \
                                  LO_UINT16( uuid ), HI_UINT16( uuid ), 0x00, 0x00

// HAP base 128-bit UUID: 00000000-0000-1000-8000-0026BB765291
#define HAP_BASE_UUID_128( uuid ) 0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, \
                                  0x00, 0x80, 0x00, 0x10, 0x00, 0x00, \
                                  LO_UINT16( uuid ), HI_UINT16( uuid ), 0x00, 0x00

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      Pairing_AddService
 *
 * @brief   Initializes the HAP Pairing service by registering GATT
 *          attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t Pairing_AddService(void);

/*********************************************************************
 * @fn      Pairing_Register
 *
 * @brief   Register a callback function with the HAP Pairing Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Pairing_Register(pairingServiceCB_t pfnServiceCB);

/*********************************************************************
 * @fn      Pairing_SetParameter
 *
 * @brief   Set an HAP Pairing Service parameter.
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
extern bStatus_t Pairing_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      Pairing_GetParameter
 *
 * @brief   Get an HAP Pairing parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Pairing_GetParameter(uint8_t param, void *value);

/*********************************************************************
 * @fn      Pairing_ConvertUUIDto16
 *
 * @brief   Convert a 16 or 128-bit UUID to 16-bit UUID. Simply, the 2 byte
 *          Attribute UUID represents the x's in the following:
 *          0000xxxx-0000-1000-8000-00805F9B34FB
 *
 * @param   pUUID - pointer to 16 or 128-bit UUID
 *
 * @return  16-bit UUID.
 */
extern uint16_t Pairing_ConvertUUIDto16(gattAttrType_t *pUuid);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PAIRINGSERVICE_H */
