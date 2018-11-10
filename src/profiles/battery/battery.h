/******************************************************************************

 @file  battery.h

 @brief This file contains Battery Profile header file.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2009-2018, Texas Instruments Incorporated
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

#ifndef BATTERY_H
#define BATTERY_H

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

// Profile Parameters
#define BATTERY_ATTR_LEVEL            1  // RW uint8 - Profile Attribute value
#define BATTERY_ATTR_STATE            2  // RW uint8 - Profile Attribute value

// Profile UUIDs
#define BATTERY_LEVEL_UUID            0xFFB1
#define BATTERY_STATE_UUID            0xFFB2

// Battery Service UUID
#define BATTERY_SERVICE_UUID          0xFFB0

// Battery Level Max
#define BATTERY_LEVEL_MAX             100

// Battery States
#define BATTERY_STATE_NOT_PRESENT             0
#define BATTERY_STATE_NOT_USED                1
#define BATTERY_STATE_CHARGING                2
#define BATTERY_STATE_RECHARGE_COMPLETE       3
#define BATTERY_STATE_DISCHARGING             4
#define BATTERY_STATE_CRITICAL_REPLACE_NOW    5
#define BATTERY_STATE_CRITICAL_RECHARGE_NOW   6
#define BATTERY_STATE_RESERVED                7

// Proximity Profile Services bit fields
#define BATTERY_SERVICE                       0x00000001

/*********************************************************************
 * TYPEDEFS
 */

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
 *      Initializes the Batter service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
bStatus_t Battery_AddService( uint32 services );

/*
 * Battery_SetParameter - Set a Battery Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Battery_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Battery_GetParameter - Get a Battery Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Battery_GetParameter( uint8 param, void *value );





/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
