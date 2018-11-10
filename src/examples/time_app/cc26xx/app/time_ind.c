/******************************************************************************

 @file  time_ind.c

 @brief Time indication and notification handling routines.

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
#include "string.h"
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

// Maximum category ID value
#define ALERT_CAT_ID_MAX            9

// Parse major category
#define ALERT_MAJOR_CAT(x)          ((x) >> 3)

// Parse subcategory
#define ALERT_SUBCAT(x)             ((x) & 0x07)

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

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

static const char *timeAppAlertCatStr[] =
{
  "New Alert:",      //  Simple Alert
  "New Email:",      //  Email
  "New News:",       //  News
  "New Call:",       //  Call
  "New Missed:",     //  Missed call
  "New SMS:",        //  SMS
  "New Vmail:",      //  Voice mail
  "New Sched:",      //  Schedule
  "New E!!!:",       //  High Prioritized Alert
  "New IM:"          //  Instant Message
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Time_displayAlert(uint8_t *pValue, uint8_t len);

/*********************************************************************
 * @fn      Time_indGattMsg
 *
 * @brief   Handle indications and notifications.
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void Time_indGattMsg(gattMsgEvent_t *pMsg)
{
  uint8_t i;

  // Look up the handle in the handle cache
  for (i = 0; i < HDL_CACHE_LEN; i++)
  {
    if (pMsg->msg.handleValueNoti.handle == Time_handleCache[i])
    {
      break;
    }
  }

  // Perform processing for this handle
  switch (i)
  {
    case HDL_CURR_TIME_CT_TIME_START:
      // Set clock to time read from time server
      Time_clockSet(pMsg->msg.handleValueNoti.pValue);
      break;

    case HDL_NWA_NWA_START:
      // Display network availability state
      if (pMsg->msg.handleValueInd.pValue[0] == 1)
      {
        Display_print0(dispHandle, 0, 0, "Network: Yes");
      }
      else
      {
        Display_print0(dispHandle, 0, 0, "Network: None");
      }
      break;

    case HDL_ALERT_NTF_UNREAD_START:
      // Display unread message alert
      {
        uint8_t *p = pMsg->msg.handleValueNoti.pValue;
        uint8_t len = pMsg->msg.handleValueNoti.len;

        if (p[0] <= ALERT_CAT_ID_MAX && len == 2)
        {
          Display_print2(dispHandle, 0, 0, "%s %d", (uint32_t)timeAppAlertCatStr[p[0]], p[1]);
          Display_clearLine(dispHandle, 1);
        }
      }
      break;

    case HDL_ALERT_NTF_NEW_START:
      // Display incoming message
      Time_displayAlert(pMsg->msg.handleValueNoti.pValue,
                          pMsg->msg.handleValueNoti.len);
      break;

    case HDL_BATT_LEVEL_START:
      // Display battery level
      Display_print1(dispHandle, 1, 0, "Battery%% %d", pMsg->msg.handleValueNoti.pValue[0]);
      break;

    case HDL_PAS_ALERT_START:
      // Display phone alert status
      Display_print1(dispHandle, 0, 0, "Phone Alert: %X", pMsg->msg.handleValueNoti.pValue[0]);
      break;

    case HDL_PAS_RINGER_START:
      // Display ringer state
      if (pMsg->msg.handleValueNoti.pValue[0] == 0)
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

  // Send confirm for indication
  if (pMsg->method == ATT_HANDLE_VALUE_IND)
  {
    ATT_HandleValueCfm(pMsg->connHandle);
  }
}

/*********************************************************************
 * @fn      Time_displayAlert
 *
 * @brief   Display the alert.
 *
 * @param   pValue - Pointer to buffer containing a new incoming alert
 *                   characteristic.
 * @param   len - length of pValue.
 *
 * @return  none
 */
static void Time_displayAlert(uint8_t *pValue, uint8_t len)
{
  char *pBuf;

  // Verify alert category and length
  if ( pValue[0] <= ALERT_CAT_ID_MAX && len >= 2 )
  {
    // Write category and number of unread to first line
    Display_print2(dispHandle, 0, 0, "%s %d", (uint32_t)timeAppAlertCatStr[pValue[0]], pValue[1]);
    pValue += 2;
    len -= 2;

    // Write alert text to second line with guaranteed null terminator
    if ( pBuf = ICall_malloc(len + 1) )
    {
      memcpy(pBuf, pValue, len);
      pBuf[len] = '\0';
      Display_print0(dispHandle, 1, 0, pBuf);
      ICall_free(pBuf);
    }
  }
}


/*********************************************************************
*********************************************************************/
