/******************************************************************************

 @file  glucose_ind.c

 @brief Glucose Collector App indication and notification handling routines for
        use with the CC26xx Bluetooth Low Energy Protocol Stack.

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
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include "string.h"
#include "bcomdef.h"

#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "glucservice.h"
#include "glucose_collector.h"
#include <ti/mw/display/Display.h>
#include "util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define STR_MG_PER_DL "mg/dL: %d"
#define STR_MMOL_PER_L "mmol/L: %d"

/*********************************************************************
 * TYPEDEFS
 */

// Data in a glucose measurement as defined in the profile
typedef struct {
  uint8_t flags;
  uint16_t seqNum;
  uint8_t baseTime[7];
  int16 timeOffset;
  uint16_t concentration;
  uint8_t typeSampleLocation;
  uint16_t sensorStatus;
} glucoseMeas_t;

// Context data as defined in profile
typedef struct {
  uint8_t flags;
  uint16_t seqNum;
  uint8_t extendedFlags;
  uint8_t carboId;
  uint16_t carboVal;
  uint8_t mealVal;
  uint8_t TesterHealthVal;
  uint16_t exerciseDuration;
  uint8_t exerciseIntensity;
  uint8_t medId;
  uint16_t medVal;
  uint16_t HbA1cVal;
} glucoseContext_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
// Clock instances for internal periodic events.
extern Clock_Struct procTimeoutClock;

// Display Interface
extern Display_Handle dispHandle;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// For test purposes
static glucoseMeas_t glucoseMeas;
static glucoseContext_t glucoseContext;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      glucoseIndGattMsg
 *
 * @brief   Handle indications and notifications.
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void glucoseIndGattMsg(gattMsgEvent_t *pMsg)
{
  uint8_t i;

  // Look up the handle in the handle cache
  for (i = 0; i < HDL_CACHE_LEN; i++)
  {
    if (pMsg->msg.handleValueInd.handle == glucoseHdlCache[i])
    {
      break;
    }
  }

  // Perform processing for this handle
  switch (i)
  {
    case HDL_GLUCOSE_START:
      {
        uint8_t *p = pMsg->msg.handleValueNoti.pValue;

        // restart procedure timer
        if (glucCollWritePending == true)
        {
          // start procedure timer
          Util_stopClock(&procTimeoutClock);
          Util_startClock(&procTimeoutClock);
        }

        memset(&glucoseMeas, 0, sizeof(glucoseMeas));

        // Flags
        glucoseMeas.flags = *p++;

        // Sequence number
        glucoseMeas.seqNum = BUILD_UINT16(p[0], p[1]);
        Display_print1(dispHandle, 0, 0, "SeqNum: %d", glucoseMeas.seqNum);
        p += 2;

        // Base time
        memcpy(glucoseMeas.baseTime, p, 7);
        p += 7;

        // Time offset;
        if (glucoseMeas.flags & GLUCOSE_MEAS_FLAG_TIME_OFFSET)
        {
          glucoseMeas.timeOffset = BUILD_UINT16(p[0], p[1]);
          p += 2;
        }

        // Glucose concentration
        if(glucoseMeas.flags & GLUCOSE_MEAS_FLAG_CONCENTRATION)
        {
          glucoseMeas.concentration = BUILD_UINT16(p[0], p[1]);

          if(glucoseMeas.flags & GLUCOSE_MEAS_FLAG_UNITS)
          {
            Display_print1(dispHandle, 1, 0, STR_MMOL_PER_L, glucoseMeas.concentration);
          }
          else
          {
            Display_print1(dispHandle, 1, 0, STR_MG_PER_DL, glucoseMeas.concentration);
          }

          p += 2;

          // Type sample location
          glucoseMeas.typeSampleLocation = *p++;
        }

        // Sensor status annunciation
        if (glucoseMeas.flags & GLUCOSE_MEAS_FLAG_STATUS_ANNUNCIATION)
        {
          glucoseMeas.sensorStatus = BUILD_UINT16(p[0], p[1]);
          p += 2;
        }
      }
      break;

    case HDL_GLUCOSE_CONTEXT_START:
      {
        uint8_t *p = pMsg->msg.handleValueNoti.pValue;

        // restart procedure timer
        if (glucCollWritePending == true)
        {
          // start procedure timer
          Util_stopClock(&procTimeoutClock);
          Util_startClock(&procTimeoutClock);
        }

        memset(&glucoseContext, 0, sizeof(glucoseContext));

        // Flags
        glucoseContext.flags = *p++;

        // Sequence number
        glucoseContext.seqNum = BUILD_UINT16(p[0], p[1]);
        p += 2;

        // Extended flags
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_EXTENDED)
        {
          glucoseContext.extendedFlags = *p++;
        }

        // Carbohydrate
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_CARBO)
        {
          // carbohydrate ID
          glucoseContext.carboId = *p++;

          // Carbohydrate
          glucoseContext.carboVal = BUILD_UINT16(p[0], p[1]);
          p += 2;
        }

        // Meal
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_MEAL)
        {
          glucoseContext.mealVal = *p++;
        }

        // Tester health
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_TESTER_HEALTH)
        {
          glucoseContext.TesterHealthVal = *p++;
        }

        // Exercise
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_EXERCISE)
        {
          // Duration
          glucoseContext.exerciseDuration = BUILD_UINT16(p[0], p[1]);
          p += 2;

          // Intensity
          glucoseContext.exerciseIntensity = *p++;
        }

        // Medication
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_MEDICATION)
        {
          // Medication ID
          glucoseContext.medId = *p++;

          // Medication
          glucoseContext.medVal = BUILD_UINT16(p[0], p[1]);
          p += 2;
        }

        // HbA1c
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_HbA1c)
        {
          glucoseContext.HbA1cVal = BUILD_UINT16(p[0], p[1]);

          Display_print1(dispHandle, 2, 0, "HbA1c: %d", glucoseContext.HbA1cVal);
          p += 2;
        }
      }
      break;

    case HDL_GLUCOSE_CTL_PNT_START:
      {
        uint8_t *pValue = pMsg->msg.handleValueInd.pValue;

        // stop procedure timer
        Util_stopClock(&procTimeoutClock);

        if(pValue[0] == CTL_PNT_OP_NUM_RSP)
        {
          if(pMsg->msg.handleValueInd.len >= 3)
          {
            Display_print0(dispHandle, 0, 0, "Matching ");
            Display_print0(dispHandle, 1, 0, "Records:");
            Display_print1(dispHandle, 2, 0, "%d", BUILD_UINT16(pValue[2], pValue[3]));
          }
        }
        else if(pValue[0] == CTL_PNT_OP_REQ_RSP && glucCollClearPending)
        {
          glucCollClearPending = false;

          if(pMsg->msg.handleValueInd.len >= 3)
          {
            switch(pValue[3])
            {
              case CTL_PNT_RSP_SUCCESS:
                Display_print0(dispHandle, 0, 0, "Records");
                Display_print0(dispHandle, 1, 0, "Cleared");
                Display_clearLine(dispHandle, 2);
                break;

              case CTL_PNT_RSP_NO_RECORDS:
                Display_print0(dispHandle, 0, 0, "No Matching");
                Display_print0(dispHandle, 1, 0, "Records");
                Display_print0(dispHandle, 2, 0, "to Delete");
                break;

              default:
                Display_print0(dispHandle, 0, 0, "Error:");
                Display_print1(dispHandle, 1, 0, "%d", pValue[3]);
                Display_clearLine(dispHandle, 2);
                break;
            }
          }
        }
        else if(pValue[0] == CTL_PNT_OP_REQ_RSP)
        {
          if(pMsg->msg.handleValueInd.len >= 3)
          {
            switch(pValue[3])
            {
              case CTL_PNT_RSP_SUCCESS:
                break;

              case CTL_PNT_RSP_NO_RECORDS:
                Display_print0(dispHandle, 0, 0, "No Matching");
                Display_print0(dispHandle, 1, 0, "Records");
                Display_print0(dispHandle, 2, 0, "Found");
                break;

              default:
                Display_print0(dispHandle, 0, 0, "Error:");
                Display_print1(dispHandle, 1, 0, "%d", pValue[3]);
                Display_clearLine(dispHandle, 2);
                break;
            }
          }
        }
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
 *********************************************************************/
