/******************************************************************************

 @file  lightservice.c

 @brief This file contains the Lights GATT profile sample GATT service profile
        for use with the BLE sample application.

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
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "string.h"

#include "lightservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Lights GATT Profile Service UUID: 0xFFB0
CONST uint8 lightsProfileServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LIGHTSPROFILE_SERV_UUID), HI_UINT16(LIGHTSPROFILE_SERV_UUID)
};

// Characteristic Red UUID: 0xFFB1
CONST uint8 lightsProfileRedUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LIGHTSPROFILE_RED_UUID), HI_UINT16(LIGHTSPROFILE_RED_UUID)
};

// Characteristic Green UUID: 0xFFB2
CONST uint8 lightsProfileGreenUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LIGHTSPROFILE_GREEN_UUID), HI_UINT16(LIGHTSPROFILE_GREEN_UUID)
};

// Characteristic Blue UUID: 0xFFB3
CONST uint8 lightsProfileBlueUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LIGHTSPROFILE_BLUE_UUID), HI_UINT16(LIGHTSPROFILE_BLUE_UUID)
};

// Characteristic White UUID: 0xFFB4
CONST uint8 lightsProfileWhiteUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LIGHTSPROFILE_WHITE_UUID), HI_UINT16(LIGHTSPROFILE_WHITE_UUID)
};

// Characteristic RGBW UUID: 0xFFB5
CONST uint8 lightsProfileRGBWUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LIGHTSPROFILE_RGBW_UUID), HI_UINT16(LIGHTSPROFILE_RGBW_UUID)
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

static lightsProfileCBs_t *lights_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Lights Profile Service attribute
static CONST gattAttrType_t lightsProfileService = { ATT_BT_UUID_SIZE, lightsProfileServUUID };

// Lights Profile Red Properties
static uint8 lightsProfileRedProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic Red Value: see lightsProfileRGBW[0]
// Lights Profile Characteristic Red User Description
static uint8 lightsProfileRedUserDesp[17] = "Red";

// Lights Profile Green Properties
static uint8 lightsProfileGreenProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic Green Value: see lightsProfileRGBW[1]
// Lights Profile Characteristic Green User Description
static uint8 lightsProfileGreenUserDesp[17] = "Green";

// Lights Profile Blue Properties
static uint8 lightsProfileBlueProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic Blue Value; see lightsProfileRGBW[2]
// Lights Profile Characteristic Blue User Description
static uint8 lightsProfileBlueUserDesp[17] = "Blue";

// Lights Profile White Properties
static uint8 lightsProfileWhiteProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic White Value: see lightsProfileRGBW[3]
// Lights Profile Characteristic White User Description
static uint8 lightsProfileWhiteUserDesp[17] = "White";


// Lights Profile Characteristic RGBW Properties
static uint8 lightsProfileRGBWProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic RGBW Value
static uint8 lightsProfileRGBW[LIGHTSPROFILE_RGBW_LEN] = { 0, 0, 0, 0 };

// Lights Profile Characteristic 5 User Description
static uint8 lightsProfileRGBWUserDesp[17] = "RGBW 4 bytes";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t lightsProfileAttrTbl[] =
{
  // Lights Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&lightsProfileService            /* pValue */
  },

    // Characteristic Red Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &lightsProfileRedProps
    },

      // Characteristic Value Red
      {
        { ATT_BT_UUID_SIZE, lightsProfileRedUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &lightsProfileRGBW[0]
      },

      // Characteristic Red User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lightsProfileRedUserDesp
      },


    // Characteristic Green Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &lightsProfileGreenProps
    },

      // Characteristic Value Green
      {
        { ATT_BT_UUID_SIZE, lightsProfileGreenUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &lightsProfileRGBW[1]
      },

      // Characteristic Green User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lightsProfileGreenUserDesp
      },


    // Characteristic Blue Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &lightsProfileBlueProps
    },

      // Characteristic Value Blue
      {
        { ATT_BT_UUID_SIZE, lightsProfileBlueUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &lightsProfileRGBW[2]
      },

      // Characteristic Blue User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lightsProfileBlueUserDesp
      },


    // Characteristic White Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &lightsProfileWhiteProps
    },

      // Characteristic Value White
      {
        { ATT_BT_UUID_SIZE, lightsProfileWhiteUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &lightsProfileRGBW[3]
      },

      // Characteristic White User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lightsProfileWhiteUserDesp
      },

    // Characteristic 5 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &lightsProfileRGBWProps
    },

      // Characteristic Value RGBW
      {
        { ATT_BT_UUID_SIZE, lightsProfileRGBWUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        lightsProfileRGBW
      },

      // Characteristic 5 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lightsProfileRGBWUserDesp
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t lights_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen,
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);
static bStatus_t lights_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Lights Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t lightsProfileCBs =
{
  lights_ReadAttrCB,  // Read callback function pointer
  lights_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Lights_addService
 *
 * @brief   Initializes the Lights Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Lights_addService(void)
{

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(lightsProfileAttrTbl,
                                     GATT_NUM_ATTRS(lightsProfileAttrTbl),
                                     GATT_MAX_ENCRYPT_KEY_SIZE,
                                     &lightsProfileCBs);
}


/*********************************************************************
 * @fn      Lights_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Lights_registerAppCBs(lightsProfileCBs_t *appCallbacks)
{
  if (appCallbacks)
  {
    lights_AppCBs = appCallbacks;

    return (SUCCESS);
  }
  else
  {
    return (bleAlreadyInRequestedMode);
  }
}


/*********************************************************************
 * @fn      Lights_setParameter
 *
 * @brief   Set a Lights Profile parameter.
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
bStatus_t Lights_setParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case LIGHTSPROFILE_RED:
      if (len == sizeof(uint8))
      {
        lightsProfileRGBW[0] = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LIGHTSPROFILE_GREEN:
      if (len == sizeof(uint8))
      {
        lightsProfileRGBW[1] = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LIGHTSPROFILE_BLUE:
      if (len == sizeof(uint8))
      {
        lightsProfileRGBW[2] = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LIGHTSPROFILE_WHITE:
      if (len == sizeof(uint8))
      {
        lightsProfileRGBW[3] = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LIGHTSPROFILE_RGBW:
      if (len == LIGHTSPROFILE_RGBW_LEN)
      {
        VOID memcpy(lightsProfileRGBW, value, LIGHTSPROFILE_RGBW_LEN);
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
 * @fn      Lights_getParameter
 *
 * @brief   Get a Lights Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Lights_getParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case LIGHTSPROFILE_RED:
      *((uint8*)value) = lightsProfileRGBW[0];
      break;

    case LIGHTSPROFILE_GREEN:
      *((uint8*)value) = lightsProfileRGBW[1];
      break;

    case LIGHTSPROFILE_BLUE:
      *((uint8*)value) = lightsProfileRGBW[2] ;
      break;

    case LIGHTSPROFILE_WHITE:
      *((uint8*)value) = lightsProfileRGBW[3];
      break;

    case LIGHTSPROFILE_RGBW:
      VOID memcpy( value, lightsProfileRGBW, LIGHTSPROFILE_RGBW_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          lights_ReadAttrCB
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
 * @return      Success or Failure
 */
static bStatus_t lights_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen,
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      case LIGHTSPROFILE_RED_UUID:
      case LIGHTSPROFILE_GREEN_UUID:
      case LIGHTSPROFILE_BLUE_UUID:
      case LIGHTSPROFILE_WHITE_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;

      case LIGHTSPROFILE_RGBW_UUID:
        *pLen = LIGHTSPROFILE_RGBW_LEN;
        VOID memcpy(pValue, pAttr->pValue, LIGHTSPROFILE_RGBW_LEN);
        break;

      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return (status);
}

/*********************************************************************
 * @fn      lights_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr      - pointer to attribute
 * @param   pValue     - pointer to data to be written
 * @param   len        - length of data
 * @param   offset     - offset of the first octet to be written
 * @param   method     - type of write message
 *
 * @return  Success or Failure
 */
static bStatus_t lights_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

    switch (uuid)
    {
      case LIGHTSPROFILE_RED_UUID:
      case LIGHTSPROFILE_GREEN_UUID:
      case LIGHTSPROFILE_BLUE_UUID:
      case LIGHTSPROFILE_WHITE_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if (offset == 0)
        {
          if (len != 1)
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        // Write the value
        if (status == SUCCESS)
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          if (pAttr->pValue == &lightsProfileRGBW[0])
          {
            notifyApp = LIGHTSPROFILE_RED;
          }

          if (pAttr->pValue == &lightsProfileRGBW[1])
          {
            notifyApp = LIGHTSPROFILE_GREEN;
          }

          if (pAttr->pValue == &lightsProfileRGBW[2])
          {
            notifyApp = LIGHTSPROFILE_BLUE;
          }

          if (pAttr->pValue == &lightsProfileRGBW[3])
          {
            notifyApp = LIGHTSPROFILE_WHITE;
          }
        }

        break;

    case LIGHTSPROFILE_RGBW_UUID:
      if (offset == 0)
      {
        if (len != LIGHTSPROFILE_RGBW_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      if (status == SUCCESS)
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;
        memcpy( pCurValue, pValue, LIGHTSPROFILE_RGBW_LEN );

        if( (int)pAttr->pValue == (int)&lightsProfileRGBW )
        {
          notifyApp = LIGHTSPROFILE_RGBW;
        }
      }
      break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                offset, GATT_CLIENT_CFG_NOTIFY);
        break;

      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify
  // application of change
  if ((notifyApp != 0xFF ) && lights_AppCBs &&
       lights_AppCBs->pfnLightsProfileChange)
  {
    lights_AppCBs->pfnLightsProfileChange(notifyApp);
  }

  return (status);
}


/*********************************************************************
*********************************************************************/
