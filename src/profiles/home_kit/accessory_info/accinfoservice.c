/******************************************************************************

 @file  accinfoservice.c

 @brief This file contains the HAP Accessory Information service.

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
#include "accinfoservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERV_INSTANCE_POS       2
#define IDENTIFY_POS            5
#define MANUFACT_POS            8
#define MODEL_POS               11
#define NAME_POS                14
#define SERIAL_NO_POS           17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Access Information service UUID
CONST uint8_t accInfoServUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(HAP_ACCINFO_SERV_UUID)
};

// Service instance characteristic UUID
extern CONST uint8_t servInstanceUUID[];
//CONST uint8_t servInstanceUUID[ATT_BT_UUID_SIZE] =
//{
  //HAP_BASE_UUID_128(HAP_SERV_INSTANCE_UUID)
//};

// Identify characteristic UUID
CONST uint8_t identifyUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ACCINFO_IDENTIFY_UUID)
};

// Manufacturer characteristic UUID
CONST uint8_t manufactUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ACCINFO_MANUFACT_UUID)
};

// Model characteristic UUID
CONST uint8_t modelUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ACCINFO_MODEL_UUID)
};

// Name characteristic UUID
CONST uint8_t nameUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ACCINFO_NAME_UUID)
};

// Serial Number characteristic UUID
CONST uint8_t serialNoUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ACCINFO_SERIAL_NO_UUID)
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
static accInfoServiceCB_t accInfoServiceCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Accessory Information Service attribute
static CONST gattAttrType_t accInfoService = { ATT_UUID_SIZE, accInfoServUUID };

// Service instance characteristic
static uint8_t servInstanceProps = GATT_PROP_READ;

// Service instance ID represented as a UTF-8 string
static uint8 servInstance[HAP_INSTANCE_CHAR_LEN+1];

// Identify characteristic properties
static uint8_t identifyProps = GATT_PROP_WRITE; // Paired Write

// Identify attribute
static uint8_t identify = 0;

// Characteristic User Description (characteristic instance ID represented as a
// UTF-8 string and delimited with a semicolon, e.g. 2;)
static uint8 identifyUserDesc[HAP_INSTANCE_CHAR_LEN+1];

// Manufacturer characteristic properties
static uint8_t manufactProps = GATT_PROP_READ; // Paired Read

// Manufacturer attribute
static uint8 manufacturer[] = "TI Inc.";

// Characteristic User Description (characteristic instance ID represented as a
// UTF-8 string and delimited with a semicolon, e.g. 2;)
static uint8 manufactUserDesc[HAP_INSTANCE_CHAR_LEN+1];

// Model characteristic properties
static uint8_t modelProps = GATT_PROP_READ; // Paired Read

// Model attribute
static uint8_t model[] = "Model S";

// Characteristic User Description (characteristic instance ID represented as a
// UTF-8 string and delimited with a semicolon, e.g. 2;)
static uint8 modelUserDesc[HAP_INSTANCE_CHAR_LEN+1];

// Name characteristic properties
static uint8_t nameProps = GATT_PROP_READ; // Paired Read

// Name attribute
static uint8 name[] = "Accessory Y";

// Characteristic User Description (characteristic instance ID represented as a
// UTF-8 string and delimited with a semicolon, e.g. 2;)
static uint8 nameUserDesc[HAP_INSTANCE_CHAR_LEN+1];

// Serial Number characteristic properties
static uint8_t serialNoProps = GATT_PROP_READ; // Paired Read

// Serial Number attribute
static uint8 serialNo[] = "123456789";

// Characteristic User Description (characteristic instance ID represented as a
// UTF-8 string and delimited with a semicolon, e.g. 2;)
static uint8 serialNoUserDesc[HAP_INSTANCE_CHAR_LEN+1];

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t accInfoAttrTbl[] =
{
  // Accessory Information Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&accInfoService                /* pValue */
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

    // Identify characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &identifyProps
    },

      // Identify characteristic
      {
        { ATT_UUID_SIZE, identifyUUID },
        GATT_PERMIT_WRITE, // Paired Write
        0,
        &identify
      },

      // Identify characteristic user description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        identifyUserDesc
      },

    // Manufacturer characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &manufactProps
    },

      // Manufacturer characteristic
      {
        { ATT_UUID_SIZE, manufactUUID },
        GATT_PERMIT_READ, // Paired Read
        0,
        manufacturer
      },

      // Manufacturer characteristic user description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        manufactUserDesc
      },

    // Model characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &modelProps
    },

      // Model characteristic
      {
        { ATT_UUID_SIZE, modelUUID },
        GATT_PERMIT_READ, // Paired Read
        0,
        model
      },

      // Model characteristic user description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        modelUserDesc
      },

    // Name characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &nameProps
    },

      // Name characteristic
      {
        { ATT_UUID_SIZE, nameUUID },
        GATT_PERMIT_READ, // Paired Read
        0,
        name
      },

      // Name characteristic user description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        nameUserDesc
      },

    // Serial Number characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &serialNoProps
    },

      // Serial Number characteristic
      {
        { ATT_UUID_SIZE, serialNoUUID },
        GATT_PERMIT_READ, // Paired Read
        0,
        serialNo
      },

      // Serial Number characteristic user description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        serialNoUserDesc
      }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t accInfoReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint8_t *pLen, uint16_t offset,
                                   uint8_t maxLen, uint8 method );
static bStatus_t accInfoWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint8_t len, uint16_t offset,
                                    uint8 method );

static void accInfoSetCharInstID(uint8_t *pInst, uint8_t size, uint8_t pos);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Accessory Information Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t accInfoCBs =
{
  accInfoReadAttrCB,  // Read callback function pointer
  accInfoWriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AccInfo_AddService
 *
 * @brief   Initializes the HAP Accessory Information Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t AccInfo_AddService(void)
{
  uint8_t status;

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(accInfoAttrTbl,
                                       GATT_NUM_ATTRS(accInfoAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &accInfoCBs);

  // Setup the service instance ID
  memset(servInstance, 0, sizeof(servInstance));
  _itoa(accInfoAttrTbl[SERV_INSTANCE_POS].handle, servInstance, 10);

  // Setup the characteristic instance IDs
  accInfoSetCharInstID(identifyUserDesc, sizeof(identifyUserDesc), IDENTIFY_POS);
  accInfoSetCharInstID(manufactUserDesc, sizeof(manufactUserDesc), MANUFACT_POS);
  accInfoSetCharInstID(modelUserDesc, sizeof(modelUserDesc), MODEL_POS);
  accInfoSetCharInstID(nameUserDesc, sizeof(nameUserDesc), NAME_POS);
  accInfoSetCharInstID(serialNoUserDesc, sizeof(serialNoUserDesc), SERIAL_NO_POS);

  return (status);
}

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
extern void AccInfo_Register(accInfoServiceCB_t pfnServiceCB)
{
  accInfoServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      AccInfo_SetParameter
 *
 * @brief   Set an HAP Pairing Service parameter.
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
bStatus_t AccInfo_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HAP_ACCINFO_MANUFACT_UUID:
      if (len == sizeof(uint8_t))
      {
        manufacturer[0] = *((uint8_t* )value);
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
 * @fn      AccInfo_GetParameter
 *
 * @brief   Get an HAP Pairing Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t AccInfo_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case HAP_ACCINFO_MANUFACT_UUID:
      *((uint8_t *)value) = manufacturer[0];
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          accInfoReadAttrCB
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
static bStatus_t accInfoReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint8_t *pLen, uint16_t offset,
                                   uint8_t maxLen, uint8 method)
{
  bStatus_t status = SUCCESS;
  uint16_t uuid = Pairing_ConvertUUIDto16(&pAttr->type);

  switch (uuid)
  {
    case HAP_ACCINFO_MANUFACT_UUID:
      *pLen = 2;
      pValue[0] = 'P';
      pValue[1] = 'S';
      break;

    case HAP_ACCINFO_MODEL_UUID:
      *pLen = 2;
      pValue[0] = 'P';
      pValue[1] = 'V';
      break;

    case HAP_ACCINFO_NAME_UUID:
      *pLen = 1;
      pValue[0] = name[0];
      break;

    case HAP_ACCINFO_SERIAL_NO_UUID:
      *pLen = 2;
      pValue[0] = 'P';
      pValue[1] = 'L';
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
 * @fn      accInfoWriteAttrCB
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
static bStatus_t accInfoWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint8_t len, uint16_t offset,
                                    uint8 method)
{
  bStatus_t status = SUCCESS;
  uint16_t uuid = Pairing_ConvertUUIDto16(&pAttr->type);

  switch (uuid)
  {
    case HAP_ACCINFO_IDENTIFY_UUID:
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return (status);
}

/*********************************************************************
 * @fn      accInfoSetCharInstID
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
static void accInfoSetCharInstID(uint8_t *pInst, uint8_t size, uint8_t pos)
{
  // Initialize it
  memset(pInst, 0, size);

  // Use attribute's handle as its instance ID
  _itoa(accInfoAttrTbl[pos].handle, pInst, 10);

  // Delimit it with a semicolon (;)
  pInst[strlen((char *)pInst)] = ';';
}

/*********************************************************************
*********************************************************************/
