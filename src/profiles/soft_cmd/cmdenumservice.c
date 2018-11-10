/******************************************************************************

 @file  cmdenumservice.c

 @brief This file contains the Command Enumeration service.

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
#include "bcomdef.h"
#include "osal.h"
#include "hal_adc.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "cmdenumservice.h"

/*********************************************************************
 * MACROS
 */

// Default Command Enumeration bitmask
#define CMD_ENUM_DEFAULT_BITMASK    ((1 << CMD_ENUM_SOFT_CMD_0) | \
                                     (1 << CMD_ENUM_SOFT_CMD_1))

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Command Enumeration service
CONST uint8 cmdEnumServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CMD_ENUM_SERVICE_UUID), HI_UINT16(CMD_ENUM_SERVICE_UUID)
};

// Command Enumeration characteristic
CONST uint8 cmdEnumUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CMD_ENUM_UUID), HI_UINT16(CMD_ENUM_UUID)
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

// Command Enumeration Service attribute
static CONST gattAttrType_t cmdEnumService = { ATT_BT_UUID_SIZE, cmdEnumServUUID };

// Command Enumeration characteristic
static uint8 cmdEnumProps = GATT_PROP_READ;
static uint8 cmdEnum[CMD_ENUM_CHAR_LEN] =
{
  CMD_ENUM_PAGE_0,
  LO_UINT16(CMD_ENUM_DEFAULT_BITMASK),
  HI_UINT16(CMD_ENUM_DEFAULT_BITMASK)
};

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t cmdEnumAttrTbl[] =
{
  // Command Enumeration Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&cmdEnumService                  /* pValue */
  },

    // Command Enumeration Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &cmdEnumProps
    },

      // Command Enumeration
      {
        { ATT_BT_UUID_SIZE, cmdEnumUUID },
        GATT_PERMIT_READ,
        0,
        cmdEnum
      }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t cmdEnumReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint16 *pLen, uint16 offset,
                                    uint16 maxLen, uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Command Enumeration Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t cmdEnumCBs =
{
  cmdEnumReadAttrCB, // Read callback function pointer
  NULL,              // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      CmdEnum_AddService
 *
 * @brief   Initializes the Command Enumeration Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t CmdEnum_AddService( void )
{
  uint8 status = SUCCESS;

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( cmdEnumAttrTbl,
                                        GATT_NUM_ATTRS( cmdEnumAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &cmdEnumCBs );

  return ( status );
}

/*********************************************************************
 * @fn      CmdEnum_SetParameter
 *
 * @brief   Set a Command Enumeration Service parameter.
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
bStatus_t CmdEnum_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case CMD_ENUM_UUID:
      osal_memcpy( cmdEnum, value, CMD_ENUM_CHAR_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      CmdEnum_GetParameter
 *
 * @brief   Get a Command Enumeration Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CmdEnum_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CMD_ENUM_UUID:
      osal_memcpy( value, cmdEnum, CMD_ENUM_CHAR_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}


/*********************************************************************
 * @fn          cmdEnumReadAttrCB
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
static bStatus_t cmdEnumReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint16 *pLen, uint16 offset,
                                    uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1] );

  // Command enumeration characteristic
  if ( uuid == CMD_ENUM_UUID )
  {
    *pLen = CMD_ENUM_CHAR_LEN;
    osal_memcpy( pValue, pAttr->pValue, CMD_ENUM_CHAR_LEN );
  }
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return ( status );
}


/*********************************************************************
*********************************************************************/
