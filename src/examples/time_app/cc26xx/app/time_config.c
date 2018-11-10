/******************************************************************************

 @file  time_config.c

 @brief Time characteristic configuration routines.

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
#include <ti/mw/display/Display.h>
#include "board.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "time_clock.h"
#include "bletime.h"

/*********************************************************************
 * MACROS
 */

// Used to determine the end of Time_configList[]
#define TIME_CONFIG_MAX      (sizeof(Time_configList) / sizeof(uint8_t))

/*********************************************************************
 * CONSTANTS
 */

// Array of handle cache indexes.  This list determines the
// characteristics that are read or written during configuration.
const uint8 Time_configList[] =
{
  HDL_CURR_TIME_CT_TIME_START,            // Current time
  HDL_CURR_TIME_LOC_INFO,                 // Local time info
  HDL_CURR_TIME_REF_INFO,                 // Reference time info
  HDL_DST_CHG_TIME_DST,                   // Time with DST
  HDL_NWA_NWA_START,                      // NwA
  HDL_BATT_LEVEL_START,                   // Battery level
  HDL_PAS_RINGER_START,                   // Ringer setting start handle
  HDL_REF_TIME_UPD_STATE,                 // Time update state
  HDL_CURR_TIME_CT_TIME_CCCD,             // Current time CCCD
  HDL_NWA_NWA_CCCD,                       // NwA CCCD
  HDL_ALERT_NTF_UNREAD_CCCD,              // Unread alert status CCCD
  HDL_ALERT_NTF_NEW_CCCD,                 // New alert CCCD
  HDL_BATT_LEVEL_CCCD,                    // Battery level CCCD
  HDL_PAS_ALERT_CCCD,                     // Alert status CCCD
  HDL_PAS_RINGER_CCCD,                    // Ringer setting CCCD

  // these characteristics are configured at connection setup
  HDL_PAS_ALERT_START,                    // Alert status start handle
  HDL_ALERT_NTF_NEW_CAT,                  // Supported New Alert Category
  HDL_ALERT_NTF_UNREAD_CAT,               // Supported Unread Alert Category
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_CTRL                      // Alert notification control point
};

// start index of alert notification control point in config list
#define TIMEAPP_ALERT_NTF_CTRL_START    18

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Connection handle.
uint16_t Time_connHandle = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

// Display Interface
extern Display_Handle dispHandle;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Time_configNext()
 *
 * @brief   Perform the characteristic configuration read or
 *          write procedure.
 *
 * @param   state - Configuration state.
 *
 * @return  New configuration state.
 */
uint8_t Time_configNext(uint8_t state)
{
  bool read;
  uint8_t value0, value1;
  static uint8_t alertNtfCtrlCmd;

  // Find next non-zero cached handle of interest
  while (state < TIME_CONFIG_MAX &&
          Time_handleCache[Time_configList[state]] == 0)
  {
    state++;
  }

  // Return if reached end of list
  if (state >= TIME_CONFIG_MAX)
  {
    return TIME_CONFIG_CMPL;
  }

  // Determine what to do with characteristic
  switch (Time_configList[state])
  {
    // Read these characteristics
    case HDL_CURR_TIME_LOC_INFO:
    case HDL_CURR_TIME_REF_INFO:
    case HDL_DST_CHG_TIME_DST:
    case HDL_NWA_NWA_START:
    case HDL_CURR_TIME_CT_TIME_START:
    case HDL_ALERT_NTF_NEW_CAT:
    case HDL_ALERT_NTF_UNREAD_CAT:
    case HDL_PAS_ALERT_START:
    case HDL_PAS_RINGER_START:
    case HDL_REF_TIME_UPD_STATE:
      read = TRUE;
      break;

    // Set notification for these characteristics
    case HDL_CURR_TIME_CT_TIME_CCCD:
    case HDL_ALERT_NTF_UNREAD_CCCD:
    case HDL_ALERT_NTF_NEW_CCCD:
    case HDL_BATT_LEVEL_CCCD:
    case HDL_PAS_ALERT_CCCD:
    case HDL_PAS_RINGER_CCCD:
      read = FALSE;
      value0 = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
      value1 = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
      break;

    // Set indication for these characteristics
    case HDL_NWA_NWA_CCCD:
      read = FALSE;
      value0 = LO_UINT16(GATT_CLIENT_CFG_INDICATE);
      value1 = HI_UINT16(GATT_CLIENT_CFG_INDICATE);
      break;

    // Alert notification control point
    case HDL_ALERT_NTF_CTRL:
      // initialize control point command
      if (state == TIMEAPP_ALERT_NTF_CTRL_START)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_ENABLE_NEW;
      }

      read = FALSE;
      value0 = alertNtfCtrlCmd;
      value1 = ALERT_NOTIF_CAT_ALL;

      // set next command to send
      if (alertNtfCtrlCmd == ALERT_NOTIF_ENABLE_NEW)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_NOTIFY_NEW;
      }
      else if (alertNtfCtrlCmd == ALERT_NOTIF_NOTIFY_NEW)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_ENABLE_UNREAD;
      }
      else if (alertNtfCtrlCmd == ALERT_NOTIF_ENABLE_UNREAD)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_NOTIFY_UNREAD;
      }
      break;

    default:
      return state;
  }

  // Do a GATT read or write
  if (read)
  {
    attReadReq_t  readReq;

    readReq.handle = Time_handleCache[Time_configList[state]];

    // Send the read request
    GATT_ReadCharValue(Time_connHandle, &readReq, ICall_getEntityId());
  }
  else
  {
    attWriteReq_t writeReq;

    writeReq.pValue = GATT_bm_alloc(Time_connHandle, ATT_WRITE_REQ, 2, NULL);
    if (writeReq.pValue != NULL)
    {
      writeReq.len = 2;
      writeReq.pValue[0] = value0;
      writeReq.pValue[1] = value1;
      writeReq.sig = 0;
      writeReq.cmd = 0;

      writeReq.handle = Time_handleCache[Time_configList[state]];

      // Send the read request
      if (GATT_WriteCharValue(Time_connHandle, &writeReq,
                              ICall_getEntityId()) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);
      }
    }
  }

  return state;
}

/*********************************************************************
 * @fn      Time_configGattMsg()
 *
 * @brief   Handle GATT messages for characteristic configuration.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New configuration state.
 */
uint8_t Time_configGattMsg(uint8_t state, gattMsgEvent_t *pMsg)
{
  // Return complete if state is beyond or at the end of list
  if (state >= TIME_CONFIG_MAX)
  {
    return TIME_CONFIG_CMPL;
  }

  if ((pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP) &&
       (pMsg->hdr.status == SUCCESS))
  {
    // Process response
    switch (Time_configList[state])
    {
      case HDL_CURR_TIME_CT_TIME_START:
        // Set clock to time read from time server
        Time_clockSet(pMsg->msg.readRsp.pValue);
        break;

      case HDL_CURR_TIME_LOC_INFO:
        break;

      case HDL_CURR_TIME_REF_INFO:
        break;

      case HDL_DST_CHG_TIME_DST:
        break;

      case HDL_NWA_NWA_START:
        // Display network availability state
        if (pMsg->msg.readRsp.pValue[0] == 1)
        {
          Display_print0(dispHandle, 0, 0, "Network: Yes");
        }
        else
        {
          Display_print0(dispHandle, 0, 0, "Network: None");
        }
        break;

      case HDL_BATT_LEVEL_START:
        // Display battery level
        Display_print1(dispHandle, 1, 0, "Battery%% %d", pMsg->msg.readRsp.pValue[0]);
        break;

      case HDL_CURR_TIME_CT_TIME_CCCD:
        break;

      case HDL_ALERT_NTF_NEW_CAT:
        break;

      case HDL_ALERT_NTF_UNREAD_CAT:
        break;

      case HDL_ALERT_NTF_UNREAD_CCCD:
        break;

      case HDL_ALERT_NTF_NEW_CCCD:
        break;

      case HDL_NWA_NWA_CCCD:
        break;

      case HDL_PAS_ALERT_START:
        // Display phone alert status
        Display_print1(dispHandle, 1, 0, "Phone Alert: %X", pMsg->msg.readRsp.pValue[0]);
        break;

      case HDL_PAS_RINGER_START:
        // Display ringer state
        if (pMsg->msg.readRsp.pValue[0] == 0)
        {
          Display_print0(dispHandle, 1, 0, "Ringer Off");
        }
        else
        {
          Display_print0(dispHandle, 1, 0, "Ringer On");
        }
        break;

      default:
        break;
    }
  }

  return Time_configNext(state + 1);
}


/*********************************************************************
*********************************************************************/
