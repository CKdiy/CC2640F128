/******************************************************************************

 @file  battery.c

 @brief Battery Profile

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "osal.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "battery.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        7

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Battery Service UUID
CONST uint8 battServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATTERY_SERVICE_UUID), HI_UINT16(BATTERY_SERVICE_UUID)
};

// Battery Level UUID
CONST uint8 battLevelUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATTERY_LEVEL_UUID), HI_UINT16(BATTERY_LEVEL_UUID)
};

// Battery State UUID
CONST uint8 battStateUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATTERY_STATE_UUID), HI_UINT16(BATTERY_STATE_UUID)
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

/*********************************************************************
 * Profile Attributes - variables
 */

// Battery Service attribute
static CONST gattAttrType_t battService = { ATT_BT_UUID_SIZE, battServiceUUID };


// Battery Level Characteristic Properties
static uint8 batLevelCharProps = GATT_PROP_READ;

// Battery Level Characteristic Value
static uint8 batLevel = 100;

// Battery Level Characteristic User Description
static uint8 batLevelCharUserDesp[14] = "Battery Level";


// Battery State Characteristic Properties
static uint8 batStateCharProps = GATT_PROP_READ;

// Battery State Characteristic
static uint8 batState = BATTERY_STATE_NOT_PRESENT;

// Battery State Characteristic User Description
static uint8 batStateCharUserDesp[14] = "Battery State";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t batteryAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // Battery Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&battService                /* pValue */
  },

    // Battery Level Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &batLevelCharProps
    },

      // Battery Level Characteristic Value
      {
        { ATT_BT_UUID_SIZE, battLevelUUID },
        GATT_PERMIT_READ,
        0,
        (uint8*)&batLevel
      },

      // Battery Level Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        batLevelCharUserDesp
      },

    // Battery State Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &batStateCharProps
    },

      // Battery State Characteristic Value
      {
        { ATT_BT_UUID_SIZE, battStateUUID },
        GATT_PERMIT_READ,
        0,
        (uint8*)&batState
      },

      // Battery State Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        batStateCharUserDesp
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t bat_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint16 *pLen, uint16 offset,
                                 uint16 maxLen, uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Battery Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t  batCBs =
{
  bat_ReadAttrCB, // Read callback function pointer
  NULL,           // Write callback function pointer
  NULL            // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Battery_AddService
 *
 * @brief   Initializes the Batter service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  success or failure
 */
bStatus_t Battery_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  if ( services & BATTERY_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( batteryAttrTbl,
                                          GATT_NUM_ATTRS( batteryAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &batCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      Battery_SetParameter
 *
 * @brief   Set a Battery Profile parameter.
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
bStatus_t Battery_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
     case BATTERY_ATTR_LEVEL:
      if ( (len == sizeof ( uint8 )) && ((*((uint8*)value) <= BATTERY_LEVEL_MAX)) )
      {
        batLevel = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BATTERY_ATTR_STATE:
      if ( (len == sizeof ( uint8 )) && ((*((uint8*)value) < BATTERY_STATE_RESERVED)) )
      {
        batState = *((uint8*)value);
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

  return ( ret );
}

/*********************************************************************
 * @fn      Battery_GetParameter
 *
 * @brief   Get a Battery Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Battery_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BATTERY_ATTR_LEVEL:
      *((uint8*)value) = batLevel;
      break;

    case BATTERY_ATTR_STATE:
      *((uint8*)value) = batState;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}



/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * @fn          bat_ReadAttrCB
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
static bStatus_tbat_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint16 *pLen, uint16 offset,
                                uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  // 16-bit UUID
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" case;
      // gattserverapp handles those types for reads
      case BATTERY_LEVEL_UUID:
      case BATTERY_STATE_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    //128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}


/*********************************************************************
*********************************************************************/
