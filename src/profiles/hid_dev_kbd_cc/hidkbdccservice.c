/******************************************************************************

 @file  hidkbdccservice.c

 @brief This file contains the HID service for a keyboard and Consumer
        Control.

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
#include "bcomdef.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "gattservapp.h"
#include "hidkbdccservice.h"
#include "hiddev.h"
#include "battservice.h"

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
// HID service
CONST uint8 hidServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HID_SERV_UUID), HI_UINT16(HID_SERV_UUID)
};

// HID Boot Keyboard Input Report characteristic
CONST uint8 hidBootKeyInputUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BOOT_KEY_INPUT_UUID), HI_UINT16(BOOT_KEY_INPUT_UUID)
};

// HID Boot Keyboard Output Report characteristic
CONST uint8 hidBootKeyOutputUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BOOT_KEY_OUTPUT_UUID), HI_UINT16(BOOT_KEY_OUTPUT_UUID)
};

// HID Information characteristic
CONST uint8 hidInfoUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HID_INFORMATION_UUID), HI_UINT16(HID_INFORMATION_UUID)
};

// HID Report Map characteristic
CONST uint8 hidReportMapUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(REPORT_MAP_UUID), HI_UINT16(REPORT_MAP_UUID)
};

// HID Control Point characteristic
CONST uint8 hidControlPointUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HID_CTRL_PT_UUID), HI_UINT16(HID_CTRL_PT_UUID)
};

// HID Report characteristic
CONST uint8 hidReportUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(REPORT_UUID), HI_UINT16(REPORT_UUID)
};

// HID Protocol Mode characteristic
CONST uint8 hidProtocolModeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PROTOCOL_MODE_UUID), HI_UINT16(PROTOCOL_MODE_UUID)
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

// HID Information characteristic value
static CONST uint8 hidInfo[HID_INFORMATION_LEN] =
{
  LO_UINT16(0x0111), HI_UINT16(0x0111),           // bcdHID (USB HID version)
  0x00,                                           // bCountryCode
  HID_KBD_FLAGS                                   // Flags
};

// HID Report Map characteristic value
// Keyboard report descriptor
static CONST uint8 hidReportMap[] =
{
  0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,  // USAGE (Keyboard)
  0xA1, 0x01,  // COLLECTION (Application)
  0x85, 0x01,  // REPORT_ID (1)
               //
  0x05, 0x07,  //   USAGE_PAGE (Key Codes)
  0x19, 0xE0,  //   USAGE_MIN (224)
  0x29, 0xE7,  //   USAGE_MAX (231)
  0x15, 0x00,  //   LOGICAL_MINIMUM (0)
  0x25, 0x01,  //   LOGICAL_MAXIMUM (1)
               //
               //   Modifier byte
  0x95, 0x08,  //     REPORT_COUNT (8)
  0x75, 0x01,  //     REPORT_SIZE (1)
  0x81, 0x02,  //     INPUT (Data, Variable, Absolute)
               //
               //   Reserved byte
  0x95, 0x01,  //     REPORT_COUNT (1)
  0x75, 0x08,  //     REPORT_SIZE (8)
  0x81, 0x01,  //     INPUT (Constant)
               //
               //   LED report
  0x95, 0x05,  //     REPORT_COUNT (5)
  0x75, 0x01,  //     REPORT_SIZE (1)
  0x05, 0x08,  //     USAGE_PAGE (LEDs)
  0x19, 0x01,  //     USAGE_MIN (1)
  0x29, 0x05,  //     USAGE_MAX (5)
  0x91, 0x02,  //     OUTPUT (Data, Variable, Absolute)
               //
               //   LED report padding
  0x95, 0x01,  //     REPORT_COUNT (1)
  0x75, 0x03,  //     REPORT_SIZE (3)
  0x91, 0x01,  //     OUTPUT (Constant)
               //
               //   Key arrays (6 bytes)
  0x95, 0x06,  //     REPORT_COUNT (6)
  0x75, 0x08,  //     REPORT_SIZE (8)
  0x15, 0x00,  //     LOGICAL_MINIMUM (0)
  0x25, 0x65,  //     LOGICAL_MAXIMUM (101)
  0x05, 0x07,  //     USAGE_PAGE (Key Codes)
  0x19, 0x00,  //     USAGE_MIN (0)
  0x29, 0x65,  //     USAGE_MAX (101)
  0x81, 0x00,  //     INPUT (Data, Array)
               //
  0xC0,        // END_COLLECTION
               //
  0x05, 0x0c,  // USAGE_PAGE (Consumer Devices)
  0x09, 0x01,  // USAGE (Consumer Control)
  0xa1, 0x01,  // COLLECTION (Application)
  0x85, 0x02,  // REPORT_ID (2)
               //
  0x09, 0x30,  //   USAGE (Power)
  0x09, 0xCD,  //   USAGE (Play/Pause)
  0x09, 0xB7,  //   USAGE (Stop)
  0x09, 0xB5,  //   USAGE (Skip track)
  0x09, 0xB6,  //   USAGE (Previous track)
  0x09, 0xB3,  //   USAGE (Fast forward)
  0x09, 0xB4,  //   USAGE (Rewind)
  0x09, 0xB2,  //   USAGE (Record)
  0x09, 0xE9,  //   USAGE (Volume up)
  0x09, 0xEA,  //   USAGE (Volume down)
  0x09, 0xE2,  //   USAGE (Mute)
  0x15, 0x01,  //   LOGICAL_MINIMUM (1)
  0x25, 0x0B,  //   LOGICAL_MAXIMUM (11)
  0x95, 0x01,  //   REPORT_COUNT (1)
  0x75, 0x08,  //   REPORT_SIZE (8)
  0x81, 0x00,  //   INPUT (Data,Ary,Abs)
               //
  0xC0         // END_COLLECTION
};

// HID report map length
uint8 hidReportMapLen = sizeof(hidReportMap);

// HID report mapping table
static hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

/*********************************************************************
 * Profile Attributes - variables
 */

// HID Service attribute
static CONST gattAttrType_t hidService = { ATT_BT_UUID_SIZE, hidServUUID };

// Include attribute (Battery service)
static uint16 include = GATT_INVALID_HANDLE;

// HID Information characteristic
static uint8 hidInfoProps = GATT_PROP_READ;

// HID Control Point characteristic
static uint8 hidControlPointProps = GATT_PROP_WRITE_NO_RSP;
static uint8 hidControlPoint;

// HID Protocol Mode characteristic
static uint8 hidProtocolModeProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
uint8 hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID Report Map characteristic
static uint8 hidReportMapProps = GATT_PROP_READ;

// HID External Report Reference Descriptor
static uint8 hidExtReportRefDesc[ATT_BT_UUID_SIZE] =
             { LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID) };

// HID Report characteristic, key input
static uint8 hidReportKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportKeyIn;
static gattCharCfg_t *hidReportKeyInClientCharCfg;

// HID Report Reference characteristic descriptor, key input
static uint8 hidReportRefKeyIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, LED output
static uint8 hidReportLedOutProps = GATT_PROP_READ  |
                                    GATT_PROP_WRITE |
                                    GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportLedOut;

// HID Report Reference characteristic descriptor, LED output
static uint8 hidReportRefLedOut[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT };

// HID Boot Keyboard Input Report
static uint8 hidReportBootKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportBootKeyIn;
static gattCharCfg_t *hidReportBootKeyInClientCharCfg;

// HID Boot Keyboard Output Report
static uint8 hidReportBootKeyOutProps = GATT_PROP_READ  |
                                        GATT_PROP_WRITE |
                                        GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportBootKeyOut;

// HID Report characteristic, consumer control input
static uint8 hidReportCCInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportCCIn;
static gattCharCfg_t *hidReportCCInClientCharCfg;

// HID Report Reference characteristic descriptor, consumer control input
static uint8 hidReportRefCCIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t hidAttrTbl[] =
{
  // HID Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *) &hidService                     /* pValue */
  },

    // Included service (battery)
    {
      { ATT_BT_UUID_SIZE, includeUUID },
      GATT_PERMIT_READ,
      0,
      (uint8 *)&include
    },

    // HID Information characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidInfoProps
    },

      // HID Information characteristic
      {
        { ATT_BT_UUID_SIZE, hidInfoUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8 *) hidInfo
      },

    // HID Control Point characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidControlPointProps
    },

      // HID Control Point characteristic
      {
        { ATT_BT_UUID_SIZE, hidControlPointUUID },
        GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidControlPoint
      },

    // HID Protocol Mode characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidProtocolModeProps
    },

      // HID Protocol Mode characteristic
      {
        { ATT_BT_UUID_SIZE, hidProtocolModeUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidProtocolMode
      },

    // HID Report Map characteristic declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidReportMapProps
    },

      // HID Report Map characteristic
      {
        { ATT_BT_UUID_SIZE, hidReportMapUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8 *) hidReportMap
      },

    // HID External Report Reference Descriptor
    {
      { ATT_BT_UUID_SIZE, extReportRefUUID },
      GATT_PERMIT_READ,
      0,
      hidExtReportRefDesc
    },

    // HID Report characteristic declaration, key input
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidReportKeyInProps
    },

      // HID Report characteristic, key input
      {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportKeyIn
      },

      // HID Report characteristic client characteristic configuration, key input
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8 *) &hidReportKeyInClientCharCfg
      },

      // HID Report Reference characteristic descriptor, key input
      {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefKeyIn
      },

    // HID Report characteristic, LED output declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidReportLedOutProps
    },

      // HID Report characteristic, LED output
      {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidReportLedOut
      },

      // HID Report Reference characteristic descriptor, LED output
      {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefLedOut
      },

    // HID Boot Keyboard Input Report declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidReportBootKeyInProps
    },

      // HID Boot Keyboard Input Report
      {
        { ATT_BT_UUID_SIZE, hidBootKeyInputUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportBootKeyIn
      },

      // HID Boot Keyboard Input Report characteristic client characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8 *) &hidReportBootKeyInClientCharCfg
      },

    // HID Boot Keyboard Output Report declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidReportBootKeyOutProps
    },

      // HID Boot Keyboard Output Report
      {
        { ATT_BT_UUID_SIZE, hidBootKeyOutputUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidReportBootKeyOut
      },

    // HID Report characteristic declaration, consumer control
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &hidReportCCInProps
    },

      // HID Report characteristic, consumer control
      {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportCCIn
      },

      // HID Report characteristic client characteristic configuration, consumer control
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8 *) &hidReportCCInClientCharCfg
      },

     // HID Report Reference characteristic descriptor, consumer control
     {
       { ATT_BT_UUID_SIZE, reportRefUUID },
       GATT_PERMIT_READ,
       0,
       hidReportRefCCIn
     }
};

// Attribute index enumeration-- these indexes match array elements above
enum
{
  HID_SERVICE_IDX,                // HID Service
  HID_INCLUDED_SERVICE_IDX,       // Included Service (battery)
  HID_INFO_DECL_IDX,              // HID Information characteristic declaration
  HID_INFO_IDX,                   // HID Information characteristic
  HID_CONTROL_POINT_DECL_IDX,     // HID Control Point characteristic declaration
  HID_CONTROL_POINT_IDX,          // HID Control Point characteristic
  HID_PROTOCOL_MODE_DECL_IDX,     // HID Protocol Mode characteristic declaration
  HID_PROTOCOL_MODE_IDX,          // HID Protocol Mode characteristic
  HID_REPORT_MAP_DECL_IDX,        // HID Report Map characteristic declaration
  HID_REPORT_MAP_IDX,             // HID Report Map characteristic
  HID_EXT_REPORT_REF_DESC_IDX,    // HID External Report Reference Descriptor
  HID_REPORT_KEY_IN_DECL_IDX,     // HID Report characteristic declaration, key input
  HID_REPORT_KEY_IN_IDX,          // HID Report characteristic, key input
  HID_REPORT_KEY_IN_CCCD_IDX,     // HID Report characteristic client characteristic configuration, key input
  HID_REPORT_REF_KEY_IN_IDX,      // HID Report Reference characteristic descriptor, key input
  HID_REPORT_LED_OUT_DECL_IDX,    // HID Report characteristic, LED output declaration
  HID_REPORT_LED_OUT_IDX,         // HID Report characteristic, LED output
  HID_REPORT_REF_LED_OUT_IDX,     // HID Report Reference characteristic descriptor, LED output
  HID_BOOT_KEY_IN_DECL_IDX,       // HID Boot Keyboard Input Report declaration
  HID_BOOT_KEY_IN_IDX,            // HID Boot Keyboard Input Report
  HID_BOOT_KEY_IN_CCCD_IDX,       // HID Boot Keyboard Input Report characteristic client characteristic configuration
  HID_BOOT_KEY_OUT_DECL_IDX,      // HID Boot Keyboard Output Report declaration
  HID_BOOT_KEY_OUT_IDX,           // HID Boot Keyboard Output Report
  HID_REPORT_CC_IN_DECL_IDX,      // HID Report characteristic declaration, consumer control
  HID_REPORT_CC_IN_IDX,           // HID Report characteristic, consumer control
  HID_REPORT_CC_IN_CCCD_IDX,      // HID Report characteristic client characteristic configuration, consumer control
  HID_REPORT_REF_CC_IN_IDX,       // HID Report Reference characteristic descriptor, consumer control
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t hidKbdCBs =
{
  HidDev_ReadAttrCB,  // Read callback function pointer
  HidDev_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidKbdCC_AddService
 *
 * @brief   Initializes the HID Service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   none
 *
 * @return  Success or Failure
 */
bStatus_t HidKbdCC_AddService(void)
{
  uint8 status = SUCCESS;

  // Allocate Client Charateristic Configuration tables.
  hidReportKeyInClientCharCfg = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                              linkDBNumConns);

  if (hidReportKeyInClientCharCfg == NULL)
  {
    return bleMemAllocError;
  }

  hidReportBootKeyInClientCharCfg = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                                  linkDBNumConns);
  if (hidReportBootKeyInClientCharCfg == NULL)
  {
    ICall_free(hidReportKeyInClientCharCfg);

    return bleMemAllocError;
  }

  hidReportCCInClientCharCfg = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                             linkDBNumConns);
  if (hidReportCCInClientCharCfg == NULL)
  {
    ICall_free(hidReportKeyInClientCharCfg);
    ICall_free(hidReportBootKeyInClientCharCfg);

    return bleMemAllocError;
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportKeyInClientCharCfg);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportBootKeyInClientCharCfg);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportCCInClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(hidAttrTbl, GATT_NUM_ATTRS(hidAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE, &hidKbdCBs);

  // Set up included service
  Batt_GetParameter(BATT_PARAM_SERVICE_HANDLE,
                    &GATT_INCLUDED_HANDLE(hidAttrTbl, HID_INCLUDED_SERVICE_IDX));

  // Construct map of reports to characteristic handles
  // Each report is uniquely identified via its ID and type

  // Key input report
  hidRptMap[0].id = hidReportRefKeyIn[0];
  hidRptMap[0].type = hidReportRefKeyIn[1];
  hidRptMap[0].handle = hidAttrTbl[HID_REPORT_KEY_IN_IDX].handle;
  hidRptMap[0].pCccdAttr = &hidAttrTbl[HID_REPORT_KEY_IN_CCCD_IDX];
  hidRptMap[0].mode = HID_PROTOCOL_MODE_REPORT;

  // LED output report
  hidRptMap[1].id = hidReportRefLedOut[0];
  hidRptMap[1].type = hidReportRefLedOut[1];
  hidRptMap[1].handle = hidAttrTbl[HID_REPORT_LED_OUT_IDX].handle;
  hidRptMap[1].pCccdAttr = NULL;
  hidRptMap[1].mode = HID_PROTOCOL_MODE_REPORT;

  // Boot keyboard input report
  // Use same ID and type as key input report
  hidRptMap[2].id = hidReportRefKeyIn[0];
  hidRptMap[2].type = hidReportRefKeyIn[1];
  hidRptMap[2].handle = hidAttrTbl[HID_BOOT_KEY_IN_IDX].handle;
  hidRptMap[2].pCccdAttr = &hidAttrTbl[HID_BOOT_KEY_IN_CCCD_IDX];
  hidRptMap[2].mode = HID_PROTOCOL_MODE_BOOT;

  // Boot keyboard output report
  // Use same ID and type as LED output report
  hidRptMap[3].id = hidReportRefLedOut[0];
  hidRptMap[3].type = hidReportRefLedOut[1];
  hidRptMap[3].handle = hidAttrTbl[HID_BOOT_KEY_OUT_IDX].handle;
  hidRptMap[3].pCccdAttr = NULL;
  hidRptMap[3].mode = HID_PROTOCOL_MODE_BOOT;

  // Consumer Control input report
  hidRptMap[4].id = hidReportRefCCIn[0];
  hidRptMap[4].type = hidReportRefCCIn[1];
  hidRptMap[4].handle = hidAttrTbl[HID_REPORT_CC_IN_IDX].handle;
  hidRptMap[4].pCccdAttr = &hidAttrTbl[HID_REPORT_CC_IN_CCCD_IDX];
  hidRptMap[4].mode = HID_PROTOCOL_MODE_REPORT;

  // Battery level input report
  VOID Batt_GetParameter(BATT_PARAM_BATT_LEVEL_IN_REPORT,
                         &(hidRptMap[2]));

  // Setup report ID map
  HidDev_RegisterReports(HID_NUM_REPORTS, hidRptMap);

  return status;
}

/*********************************************************************
 * @fn      HidKbd_SetParameter
 *
 * @brief   Set a HID Kbd parameter.
 *
 * @param   id     - HID report ID.
 * @param   type   - HID report type.
 * @param   uuid   - attribute uuid.
 * @param   len    - length of data to right.
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  GATT status code.
 */
uint8 HidKbd_SetParameter(uint8 id, uint8 type, uint16 uuid, uint8 len,
                          void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (uuid)
  {
    case REPORT_UUID:
      if (type ==  HID_REPORT_TYPE_OUTPUT)
      {
        if (len == 1)
        {
          hidReportLedOut = *((uint8 *)pValue);
        }
        else
        {
          ret = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        ret = ATT_ERR_ATTR_NOT_FOUND;
      }
      break;

    case BOOT_KEY_OUTPUT_UUID:
      if (len == 1)
      {
        hidReportBootKeyOut = *((uint8 *)pValue);
      }
      else
      {
        ret = ATT_ERR_INVALID_VALUE_SIZE;
      }
      break;

    default:
      // Ignore the request
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      HidKbd_GetParameter
 *
 * @brief   Get a HID Kbd parameter.
 *
 * @param   id     - HID report ID.
 * @param   type   - HID report type.
 * @param   uuid   - attribute uuid.
 * @param   pLen   - length of data to be read
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  GATT status code.
 */
uint8 HidKbd_GetParameter(uint8 id, uint8 type, uint16 uuid, uint8 *pLen,
                          void *pValue)
{
  switch (uuid)
  {
    case REPORT_UUID:
      if (type ==  HID_REPORT_TYPE_OUTPUT)
      {
        *((uint8 *)pValue) = hidReportLedOut;
        *pLen = 1;
      }
      else
      {
        *pLen = 0;
      }
      break;

    case BOOT_KEY_OUTPUT_UUID:
      *((uint8 *)pValue) = hidReportBootKeyOut;
      *pLen = 1;
      break;

    default:
      *pLen = 0;
      break;
  }

  return (SUCCESS);
}

/*********************************************************************
*********************************************************************/
