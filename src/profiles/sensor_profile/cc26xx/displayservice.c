/******************************************************************************

 @file  displayservice.c

 @brief Display Service for SensoprTag LCD DevPack

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

#ifdef INCLUDE_DISPLAY
/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "string.h"
#include "displayservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Attribute names
#ifdef USER_DESCRIPTION
#define DISPLAY_DATA_DESCR        "Display Data"
#define DISPLAY_CONF_DESCR        "Display Control"
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8_t sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(DISPLAY_SERV_UUID),
};

// Characteristic UUID: data
static CONST uint8_t displayDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(DISPLAY_DATA_UUID),
};

// Characteristic UUID: control
static CONST uint8_t displayControlUUID[TI_UUID_SIZE] =
{
  TI_UUID(DISPLAY_CONF_UUID),
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

static sensorCBs_t *sensor_AppCBs = NULL;
static uint8_t currentConfLen;
static uint8_t currentDataLen;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

// Characteristic Value: data
static uint8_t displayData[DISPLAY_BUFFER_LEN] = {
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'
};

// Characteristic Properties: data
static uint8_t displayDataProps = GATT_PROP_WRITE;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t displayDataUserDescr[] = DISPLAY_DATA_DESCR;
#endif

// Characteristic Properties: configuration
static uint8_t displayControlProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t displayControl[DISPLAY_CONF_LEN] = {0, 0, 0 };

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t displayControlUserDescr[] = DISPLAY_ADDR_DESCR;
#endif

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&sensorService                 /* pValue */
  },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &displayDataProps
    },

      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, displayDataUUID },
        GATT_PERMIT_WRITE,
        0,
        displayData
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        displayDataUserDescr
      },
#endif
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &displayControlProps
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, displayControlUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t*)&displayControl
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        displayControlUserDescr
      },
#endif
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen,
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);
static bStatus_t sensor_checkRange(uint8_t *pValue, uint8_t len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Display Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
static CONST gattServiceCBs_t sensorCBs =
{
  sensor_ReadAttrCB,  // Read callback function pointer
  sensor_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Display_addService
 *
 * @brief   Initializes the Sensor Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Display_addService(void)
{
    currentConfLen = DISPLAY_CONF_LEN;
    currentDataLen = DISPLAY_DATA_LEN;

    // Register GATT attribute list and CBs with GATT Server App
    return GATTServApp_RegisterService( sensorAttrTable,
                                       GATT_NUM_ATTRS (sensorAttrTable),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &sensorCBs );
}


/*********************************************************************
 * @fn      Display_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Display_registerAppCBs(sensorCBs_t *appCallbacks)
{
  if (sensor_AppCBs == NULL)
  {
    if (appCallbacks != NULL)
    {
      sensor_AppCBs = appCallbacks;
    }

    return (SUCCESS);
  }

  return (bleAlreadyInRequestedMode);
}


/*********************************************************************
 * @fn      Display_getParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Display_getParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case DISPLAY_DATA:
      memcpy(value, displayData, currentDataLen + 1);
      break;

   case DISPLAY_CONF:
      memcpy(value, displayControl, currentConfLen);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      Display_setParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Display_setParameter(uint8_t param, uint8_t len, void *value)
{
    bStatus_t ret = SUCCESS;

    switch (param)
    {
    case DISPLAY_DATA:
        if (len <= DISPLAY_DATA_LEN)
        {
            memcpy(displayData, value , len);
            displayData[len] = '\0';
            currentDataLen = len;
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;

    case DISPLAY_CONF:
        if (len <= DISPLAY_CONF_LEN)
        {
            ret = sensor_checkRange(value, len);

            if (ret == SUCCESS)
            {

                memcpy(displayControl, value , len);
                currentConfLen = len;
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
 * @fn          sensor_ReadAttrCB
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
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen,
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method)
{
    uint16_t uuid;
    bStatus_t status = SUCCESS;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if (offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
        // Invalid handle
        *pLen = 0;
        return ATT_ERR_INVALID_HANDLE;
    }

    switch (uuid)
    {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case DISPLAY_CONF_UUID:
        *pLen = currentConfLen;
        memcpy(pValue, pAttr->pValue, *pLen);
        break;

    default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }

    return (status);
}

/*********************************************************************
 * @fn      sensor_WriteAttrCB
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
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t notifyApp = 0xFF;
    uint16_t uuid;

    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
        // Invalid handle
        return ATT_ERR_INVALID_HANDLE;
    }

    switch (uuid)
    {
    case DISPLAY_DATA_UUID:
        // Validate the value
        // Make sure it's not a blob oper
        if (offset == 0)
        {
            if (len > DISPLAY_DATA_LEN )
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
            memcpy(pAttr->pValue, pValue, len);
            pAttr->pValue[len] = '\0';
            currentDataLen = len;

            // Execute the control sequence
            if (pAttr->pValue == (uint8_t*)&displayData )
            {
                notifyApp = DISPLAY_DATA;
            }
        }
        break;

    case DISPLAY_CONF_UUID:
        // Validate the value
        // Make sure it's not a blob oper
        if (offset == 0)
        {
            if (len > DISPLAY_CONF_LEN )
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
            status = sensor_checkRange(pValue, len);

            if (status == SUCCESS)
            {
                memcpy(pAttr->pValue, pValue, len);
                currentConfLen = len;

                // Execute the control sequence
                if (pAttr->pValue == (uint8_t*)&displayControl )
                {
                    notifyApp = DISPLAY_CONF;
                }
            }
        }
        break;


    default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }

    // If a characteristic value changed then callback function
    // to notify application of change
    if ((notifyApp != 0xFF ) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange)
    {
        sensor_AppCBs->pfnSensorChange(notifyApp);
    }

    return (status);
}

/*********************************************************************
 * @fn      sensor_checkRange
 *
 * @brief   Validate configuration data, in this case verify that line and
 *          column are within range and that the command is valid.
 *
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 *
 * @return  SUCCESS, or bleInvalidRange
 */
static bStatus_t sensor_checkRange(uint8_t *pValue, uint8_t len)
{
    if (pValue[DISPLAY_CMD_OFFSET] > DISPLAY_CONF_MAX)
    {
        return bleInvalidRange;
    }

    if ((len >= 2) && (pValue[DISPLAY_LINE_OFFSET] >= DISPLAY_MAX_LINES))
    {
        return bleInvalidRange;
    }

    if ((len == 3) && (pValue[DISPLAY_COL_OFFSET] >= DISPLAY_MAX_COLUMNS))
    {
        return bleInvalidRange;
    }

    return SUCCESS;
}
#endif // INCLUDE_DISPLAY
/*********************************************************************
*********************************************************************/
