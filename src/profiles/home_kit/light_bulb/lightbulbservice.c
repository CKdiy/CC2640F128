/******************************************************************************

 @file  lightbulbservice.c

 @brief This file contains the HAP Lightbulb service.

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

/*********************************************************************
 * INCLUDES
 */

#include <string.h>
#include <xdc/std.h>
#include <stdbool.h>
#include <driverlib/aon_batmon.h>

#include "onboard.h"

#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "pairingservice.h"
#include "lightbulbservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERV_INSTANCE_POS       2
#define ON_POS                  5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Lightbulb service UUID
CONST uint8_t lightbulbServUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(HAP_LIGHTBULB_SERV_UUID)
};

// Service instance characteristic UUID
extern CONST uint8_t servInstanceUUID[];
//CONST uint8_t servInstanceUUID[ATT_BT_UUID_SIZE] =
//{
  //HAP_BASE_UUID_128(HAP_SERV_INSTANCE_UUID)
//};

// On characteristic UUID
CONST uint8_t onUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LIGHTBULB_ON_UUID)
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

// Application callback
static lightbulbServiceCB_t lightbulbServiceCB = NULL;

static lightbulbCBs_t *lightbulbAppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Lightbulb Service attribute
static CONST gattAttrType_t lightbulbService = { ATT_UUID_SIZE, lightbulbServUUID };

// Service instance characteristic
static uint8_t servInstanceProps = GATT_PROP_READ;

// Service instance ID represented as a UTF-8 string
static uint8 servInstance[HAP_INSTANCE_CHAR_LEN+1];

// On characteristic properties
static uint8_t onProps = GATT_PROP_READ | GATT_PROP_WRITE; // Paired Read/Write

// Identify attribute
static uint8_t on = 0;

// Characteristic User Description (characteristic instance ID represented as a
// UTF-8 string and delimited with a semicolon, e.g. 2;)
static uint8 onUserDesc[HAP_INSTANCE_CHAR_LEN+1];

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t lightbulbAttrTbl[] =
{
  // Lightbulb Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&lightbulbService              /* pValue */
  },

    // Service instance characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      (uint8 *)&servInstanceProps
    },

      // Service instance characteristic
      {
        { ATT_UUID_SIZE, servInstanceUUID },
        GATT_PERMIT_READ,
        0,
        servInstance
      },

    // on characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &onProps
    },

      // Identify characteristic
      {
        { ATT_UUID_SIZE, onUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, // Paired Read/Write
        0,
        &on
      },

      // Identify characteristic user description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        onUserDesc
      }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t lightbulbReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint8_t *pLen, uint16_t offset,
                                     uint8_t maxLen, uint8 method );
static bStatus_t lightbulbWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint8_t len, uint16_t offset,
                                      uint8 method );

static void lightbulbSetCharInstID(uint8_t *pInst, uint8_t size, uint8_t pos);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Lightbulb Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t lightbulbCBs =
{
  lightbulbReadAttrCB,  // Read callback function pointer
  lightbulbWriteAttrCB, // Write callback function pointer
  NULL                  // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Lightbulb_AddService
 *
 * @brief   Initializes the HAP Lightbulb Service by registering GATT
 *          attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Lightbulb_AddService(void)
{
  uint8_t status;

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(lightbulbAttrTbl,
                                       GATT_NUM_ATTRS(lightbulbAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &lightbulbCBs);

  // Setup the service instance ID
  memset(servInstance, 0, sizeof(servInstance));
  _itoa(lightbulbAttrTbl[SERV_INSTANCE_POS].handle, servInstance, 10);

  // Setup the characteristic instance IDs
  lightbulbSetCharInstID(onUserDesc, sizeof(onUserDesc), ON_POS);

  return (status);
}

/*********************************************************************
 * @fn      Lightbulb_Register
 *
 * @brief   Register a callback function with the HAP Lightbulb Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Lightbulb_Register(lightbulbServiceCB_t pfnServiceCB)
{
  lightbulbServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Lightbulb_SetParameter
 *
 * @brief   Set an HAP Lightbulb Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Lightbulb_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HAP_LIGHTBULB_ON_UUID:
      if (len == sizeof(uint8_t))
      {
        on = *((uint8_t* )value);
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
 * @fn      Lightbulb_GetParameter
 *
 * @brief   Get an HAP Lightbulb Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Lightbulb_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HAP_LIGHTBULB_ON_UUID:
      *((uint8_t *)value) = on;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          lightbulbReadAttrCB
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
static bStatus_t lightbulbReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint8_t *pLen, uint16_t offset,
                                     uint8_t maxLen, uint8 method)
{
  bStatus_t status = SUCCESS;
  uint16_t uuid = Pairing_ConvertUUIDto16(&pAttr->type);

  switch (uuid)
  {
    case HAP_LIGHTBULB_ON_UUID:
      *pLen = 1;
      pValue[0] = on;
      break;

    case HAP_SERV_INSTANCE_UUID:
      {
        uint8 len = strlen((char *)(pAttr->pValue)); // Could be a long attribute

        // If the value offset of the Read Blob Request is greater than the
        // length of the attribute value, an Error Response shall be sent with
        // the error code Invalid Offset.
        if (offset <= len)
        {
          // If the value offset is equal than the length of the attribute
          // value, then the length of the part attribute value shall be zero.
          if (offset == len)
          {
            len = 0;
          }
          else
          {
            // If the attribute value is longer than (Value Offset + maxLen)
            // then maxLen octets from Value Offset shall be included in
            // this response.
            if (len > (offset + maxLen))
            {
              len = maxLen;
            }
            else
            {
              len -= offset;
            }
          }

          *pLen = len;
          VOID memcpy(pValue, &(pAttr->pValue[offset]), len);
        }
        else
        {
          status = ATT_ERR_INVALID_OFFSET;
        }
      }
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return (status);
}

/*********************************************************************
 * @fn      lightbulbWriteAttrCB
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
static bStatus_t lightbulbWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint8_t len, uint16_t offset,
                                      uint8 method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint16_t uuid = Pairing_ConvertUUIDto16(&pAttr->type);

  switch (uuid)
  {
    case HAP_LIGHTBULB_ON_UUID:
      on = pValue[0];
      notifyApp = HAP_LIGHTBULB_ON_UUID;
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && lightbulbAppCBs && lightbulbAppCBs->pfnLightbulbChange )
  {
    lightbulbAppCBs->pfnLightbulbChange( notifyApp );
  }

  return (status);
}

/*********************************************************************
 * @fn      lightbulbSetCharInstID
 *
 * @brief   Set the characteristic instance ID for a given Characteristic
 *          User Description attribute. The attribute's value will be set
 *          to its attribute handle represented as a UTF-8 string and
 *          delimited with a semicolon, e.g. 2;.
 *
 * @param   pInst - pointer to characteristic instance ID to be set
 * @param   size - size of characteristic instance ID
 * @param   pos - position of characteristic instance ID in database
 *
 * @return  none
 */
static void lightbulbSetCharInstID(uint8_t *pInst, uint8_t size, uint8_t pos)
{
  // Initialize it
  memset(pInst, 0, size);

  // Use attribute's handle as its instance ID
  _itoa(lightbulbAttrTbl[pos].handle, pInst, 10);

  // Delimit it with a semicolon (;)
  pInst[strlen((char *)pInst)] = ';';
}

/*********************************************************************
 * @fn      lightbulbRegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t lightbulbRegisterAppCBs( lightbulbCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    lightbulbAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
*********************************************************************/
