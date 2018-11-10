/******************************************************************************

 @file  simple_np_uuid.h

 @brief This file contains the UUID management for the
        Simple BLE Peripheral sample application, for use with the
        CC2650 Bluetooth Low Energy Protocol Stack.

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

#ifndef SIMPLENP_UUID_H
#define SIMPLENP_UUID_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "gatt_profile_uuid.h"
#include "gatt_uuid.h"
#include "simple_gatt_profile.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * MACROS
 */

 // Bluetooth base 128-bit UUID: 00000000-0000-1000-8000-00805F9B34FB
#define BT_BASE_UUID_128(uuid) 0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, \
                               0x00, 0x80, 0x00, 0x10, 0x00, 0x00, \
                               LO_UINT16( uuid ), HI_UINT16( uuid ), 0x00, 0x00

// Vendor base 128-bit UUID: 00000000-0000-1000-8000-0026BB765291
#define VENDOR_BASE_1_UUID_128(uuid) 0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, \
                                     0x00, 0x80, 0x00, 0x10, 0x00, 0x00, \
                                LO_UINT16( uuid ), HI_UINT16( uuid ), 0x00, 0x00

/*********************************************************************
 * TYPDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */

extern const uint8_t *SNP_findUUIDRec(uint8_t *pUUID, uint8_t len,
                                      uint8_t *newLen);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLENP_UUID_H */
