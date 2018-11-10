/******************************************************************************

 @file  lightservice.h

 @brief This file contains the Lights GATT profile definitions and prototypes
        prototypes.

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

#ifndef LIGHTSERVICE_H
#define LIGHTSERVICE_H

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
#define LIGHTSPROFILE_RED                   0  // RW uint8 - Profile Characteristic 1 value
#define LIGHTSPROFILE_GREEN                 1  // RW uint8 - Profile Characteristic 2 value
#define LIGHTSPROFILE_BLUE                  2  // RW uint8 - Profile Characteristic 3 value
#define LIGHTSPROFILE_WHITE                 3  // RW uint8 - Profile Characteristic 4 value
#define LIGHTSPROFILE_RGBW                  4  // RW uint8 - Profile Characteristic 4 value

// Lights Profile Service UUID
#define LIGHTSPROFILE_SERV_UUID             0xFFB0

// Key Pressed UUID
#define LIGHTSPROFILE_RED_UUID              0xFFB1
#define LIGHTSPROFILE_GREEN_UUID            0xFFB2
#define LIGHTSPROFILE_BLUE_UUID             0xFFB3
#define LIGHTSPROFILE_WHITE_UUID            0xFFB4
#define LIGHTSPROFILE_RGBW_UUID             0xFFB5

// Lights Keys Profile Services bit fields
#define LIGHTSPROFILE_SERVICE               0x00000001

// Length of Characteristic 5 in bytes
#define LIGHTSPROFILE_RGBW_LEN              4

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef NULL_OK void (*lightsProfileChange_t)(uint8 paramID);

typedef struct
{
  lightsProfileChange_t        pfnLightsProfileChange;  // Called when characteristic value changes
} lightsProfileCBs_t;


/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Lights_addService- Initializes the Lights GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 */

extern bStatus_t Lights_addService(void);

/*
 * Lights_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Lights_registerAppCBs(lightsProfileCBs_t *appCallbacks);

/*
 * Lights_setParameter - Set a Lights GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Lights_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * Lights_setParameter - Get a Lights GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Lights_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* LIGHTSERVICE_H */
