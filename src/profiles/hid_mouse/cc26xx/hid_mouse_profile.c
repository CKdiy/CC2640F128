/******************************************************************************

 @file  hid_mouse_profile.c

 @brief This file contains the HID Mouse profile sample GATT service profile
        for use with the BLE sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "hid_mouse_profile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        14

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// HIDMouse Service
CONST uint8 hidMouseServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HIDMOUSE_SERV_UUID), HI_UINT16(HIDMOUSE_SERV_UUID)
};

// HIDMouse Data
CONST uint8 hidMouseDataUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HIDMOUSE_DATA_UUID), HI_UINT16(HIDMOUSE_DATA_UUID)
};
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint16 hidMouseConnHandle = 0;

/*********************************************************************
 * Profile Attributes - variables
 */

// HID Mouse Service attribute
static CONST gattAttrType_t hidMouseService = { ATT_BT_UUID_SIZE, hidMouseServUUID };

// HID Mouse Data Characteristic Properties
static uint8 hidMouseDataCharProps = GATT_PROP_NOTIFY;

// HID Mouse Data Characteristic Value
// First byte indicates if buttons is pressed or released
// Second byte represent dX data
// Third byte represent dY data
// Fourth byte represent dZ data
static uint8 hidMouseData[4] = { 0x00, 0x00, 0x00 , 0x00};

// HID Mouse Data Characteristic Configuration
static uint16 hidMouseDataCharConfig = GATT_CLIENT_CFG_NOTIFY;

// HID Mouse Data Characteristic User Description
static uint8 hidMouseCharDataUserDesp[14] = "Mouse Data";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t hidMouseAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // Simple Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, 	/* 	type 		*/
    GATT_PERMIT_READ,                         	/* 	permissions     */
    0,                                        	/* 	handle 		*/
    (uint8 *)&hidMouseService            	/* 	pValue 		*/
  },

    // Mouse Data Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidMouseDataCharProps
    },

      // Mouse DataCharacteristic Value
      {
        { ATT_BT_UUID_SIZE, hidMouseDataUUID },
        0,
        0,
        hidMouseData
      },

      // Mouse Data Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&hidMouseDataCharConfig
      },

      // Mouse Data Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        hidMouseCharDataUserDesp
      },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t hidMouse_ReadAttrCB(uint16_t connHandle,
                                     gattAttribute_t *pAttr, uint8_t *pValue,
                                     uint16_t *pLen, uint16_t offset,
                                     uint16_t maxLen, uint8_t method);
static bStatus_t hidMouse_WriteAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t len, uint16_t offset,
                                      uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// HID Mouse Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t hidMouseCBs =
{
  hidMouse_ReadAttrCB,  // Read callback function pointer
  hidMouse_WriteAttrCB, // Write callback function pointer
  NULL                  // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


 /*********************************************************************
 * @fn      HidMouse_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t HidMouse_AddService(uint32 services)
{
  uint8 status = SUCCESS;

  if (services & HIDMOUSE_SERVICE)
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(hidMouseAttrTbl,
                                         GATT_NUM_ATTRS(hidMouseAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &hidMouseCBs);
  }

  return (status);
}

/*********************************************************************
 * @fn      HidMouse_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
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
bStatus_t HidMouse_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;
  const uint8 MOUSE_LEN = 4;

  switch (param)
  {
    case HIDMOUSE_DATA:
      if (len == MOUSE_LEN)
      {
        memcpy(hidMouseData, value, MOUSE_LEN);

        // Send notification if enabled
        if ((hidMouseDataCharConfig == GATT_CLIENT_CFG_NOTIFY) &&
             (hidMouseConnHandle != INVALID_CONNHANDLE))
        {
          gattAttribute_t *attr;

          // Find the characteristic value attribute
          attr = GATTServApp_FindAttr(hidMouseAttrTbl,
                                       GATT_NUM_ATTRS(hidMouseAttrTbl),
                                       hidMouseData);
          if (attr != NULL)
          {
            attHandleValueNoti_t notify;

            // Send the notification
            notify.handle = attr->handle;
            notify.len = MOUSE_LEN;
            memcpy(notify.value, hidMouseData, MOUSE_LEN);

            ret = GATT_Notification(hidMouseConnHandle, &notify, FALSE);
          }
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          hidMouse_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t hidMouse_ReadAttrCB(uint16_t connHandle,
                                     gattAttribute_t *pAttr, uint8_t *pValue,
                                     uint16_t *pLen, uint16_t offset,
                                     uint16_t maxLen, uint8_t method)
{
  // Should never get here! (characteristic does not have read permissions)
  return (ATT_ERR_ATTR_NOT_FOUND);
}

/*********************************************************************
 * @fn      hidMouse_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t hidMouse_WriteAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr, uint8_t *pValue,
                                      uint16_t len, uint16_t offset,
                                      uint8_t method)
{
  // Should never get here! (characteristic does does not have write permissions)
  return (ATT_ERR_ATTR_NOT_FOUND);
}


/*********************************************************************
*********************************************************************/
