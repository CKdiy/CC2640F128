/******************************************************************************

 @file  glucose_ctlpnt.c

 @brief Glucose Collector indication and notification handling routines for use
        with the CC26xx Bluetooth Low Energy Protocol Stack.

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

#include "bcomdef.h"
#include "gatt.h"
#include "glucservice.h"
#include "glucose_collector.h"
#include <ti/mw/display/Display.h>
#include "utc_clock.h"
#include "util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define GLUCOSE_CTL_PNT_LEN         2
#define GLUCOSE_CTL_PNT_FILTER_LEN  17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern Clock_Struct procTimeoutClock;

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
 * @fn      glucoseCtlPntWrite
 *
 * @brief   Write Control Point Requests
 *
 * @param   opcode - control point opcode
 *          oper - control point operator
 *
 * @return  status of write
 */

uint8_t glucoseCtlPntWrite(uint8_t opcode, uint8_t oper)
{
  attWriteReq_t writeReq;
  uint8_t status;

  writeReq.pValue = GATT_bm_alloc(glucCollConnHandle, ATT_WRITE_REQ,
                                  GLUCOSE_CTL_PNT_LEN, NULL);
  if (writeReq.pValue != NULL)
  {
    writeReq.pValue[0] = opcode;
    writeReq.pValue[1] = oper;

    writeReq.len = GLUCOSE_CTL_PNT_LEN;
    writeReq.sig = 0;
    writeReq.cmd = 0;

    writeReq.handle = glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START];

    // Send the write request
    status = GATT_WriteCharValue(glucCollConnHandle, &writeReq, glucCollTaskId);
    if (status != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);
    }
  }
  else
  {
    status = bleMemAllocError;
  }

  return status;
}

/*********************************************************************
 * @fn      glucoseCtlPntWriteFilter
 *
 * @brief   Write Control Point Filter Requests
 *
 * @param   opcode - control point opcode
 * @param   oper - control point operator
 * @param   filterType - control point filter type
 * @param   param1 - first filter
 * @param   param2 - second filter (if applicable), otherwise NULL

 *
 * @return  status of write
 */

uint8_t glucoseCtlPntWriteFilter(uint8_t opcode, uint8_t oper, uint8_t filterType,
                                 void* param1, void* param2)
{
  attWriteReq_t writeReq;
  uint8_t status;

  writeReq.pValue = GATT_bm_alloc(glucCollConnHandle, ATT_WRITE_REQ,
                                  GLUCOSE_CTL_PNT_FILTER_LEN, NULL);
  if (writeReq.pValue != NULL)
  {
    UTCTimeStruct *time1, *time2;
    uint16 *seqNum1, *seqNum2;

    uint8_t *p = writeReq.pValue;

    *p++ = opcode;
    *p++ = oper;

    // The operator will tells us whether to include the filters or not
    // Note day and month are converted to date time struct values
    switch(oper)
    {
      case CTL_PNT_OPER_LESS_EQUAL:
      case CTL_PNT_OPER_GREATER_EQUAL:
        *p++ = filterType;

        if (filterType == CTL_PNT_FILTER_SEQNUM)
        {
          seqNum1 = param1;
          *p++ = LO_UINT16(*seqNum1);
          *p++ = HI_UINT16(*seqNum1);
        }
        else
        {
          time1 = param1;
          *p++ = LO_UINT16(time1->year);
          *p++ = HI_UINT16(time1->year);
          *p++ = (time1->month + 1);
          *p++ = (time1->day + 1);
          *p++ = time1->hour;
          *p++ = time1->minutes;
          *p++ = time1->seconds;
        }
        break;

      case CTL_PNT_OPER_RANGE:
        *p++ = filterType;

        if (filterType == CTL_PNT_FILTER_SEQNUM)
        {
          seqNum1 = param1;
          seqNum2 = param2;
          *p++ = LO_UINT16(*seqNum1);
          *p++ = HI_UINT16(*seqNum1);
          *p++ = LO_UINT16(*seqNum2);
          *p++ = HI_UINT16(*seqNum2);
        }
        else
        {
          time1 = param1;
          time2 = param2;
          *p++ = LO_UINT16(time1->year);
          *p++ = HI_UINT16(time1->year);
          *p++ = (time1->month + 1);
          *p++ = (time1->day + 1);
          *p++ = time1->hour;
          *p++ = time1->minutes;
          *p++ = time1->seconds;
          *p++ = LO_UINT16(time2->year);
          *p++ = HI_UINT16(time2->year);
          *p++ = (time2->month + 1);
          *p++ = (time2->day + 1);
          *p++ = time2->hour;
          *p++ = time2->minutes;
          *p++ = time2->seconds;
        }
        break;

      default:
        break;
    }

    writeReq.len = (p - writeReq.pValue);
    writeReq.sig = 0;
    writeReq.cmd = 0;

    writeReq.handle = glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START];

    status = GATT_WriteCharValue(glucCollConnHandle, &writeReq, glucCollTaskId);
    if (status != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);
    }
  }
  else
  {
    status = bleMemAllocError;
  }

  return status;
}

/*********************************************************************
 * @fn      glucoseCtlPntGattMsg()
 *
 * @brief   Handle GATT messages for control point operations.
 *
 * @param   pMsg - GATT message.
 *
 * @return  None.
 */
void glucoseCtlPntGattMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_ERROR_RSP)
  {
     attErrorRsp_t *pRsp = &pMsg->msg.errorRsp;

     glucCollClearPending = false;
     Display_print0(dispHandle, 0, 0, "Write Error");
     Display_print1(dispHandle, 1, 0, "Handle: %d", pRsp->handle);
     Display_print1(dispHandle, 2, 0, "errCode:  %d", pRsp->errCode);
  }
  else if (pMsg->method == ATT_WRITE_RSP)
  {
    // start procedure timer
    Util_stopClock(&procTimeoutClock);
    Util_startClock(&procTimeoutClock);
  }

  glucCollWritePending = false;
}

/*********************************************************************
 *********************************************************************/
