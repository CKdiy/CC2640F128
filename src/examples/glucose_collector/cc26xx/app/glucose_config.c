/******************************************************************************

 @file  glucose_config.c

 @brief Glucose Collector App characteristic configuration routines for use
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
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "glucservice.h"
#include "glucose_collector.h"

/*********************************************************************
 * MACROS
 */

// Used to determine the end of glucoseConfigList[]
#define GLUCOSE_CONFIG_MAX      (sizeof(glucoseConfigList) / sizeof(uint8_t))

/*********************************************************************
 * CONSTANTS
 */

// Array of handle cache indexes.  This list determines the
// characteristics that are read or written during configuration.
const uint8_t glucoseConfigList[] =
{
  HDL_DEVINFO_SYSTEM_ID,
  HDL_DEVINFO_MANUFACTURER_NAME,
  HDL_DEVINFO_MODEL_NUM,
  HDL_GLUCOSE_FEATURE,
  HDL_GLUCOSE_MEAS_CCCD,
  HDL_GLUCOSE_CONTEXT_CCCD,
  HDL_GLUCOSE_CTL_PNT_CCCD
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern bool glucCollCharHdls;

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
 * @fn      glucoseConfigNext()
 *
 * @brief   Perform the characteristic configuration read or
 *          write procedure.
 *
 * @param   state - Configuration state.
 *
 * @return  New configuration state.
 */
uint8_t glucoseConfigNext(uint8_t state)
{
  bool read;
  uint16_t charCfg;

  // Find next non-zero cached handle of interest
  while (state < GLUCOSE_CONFIG_MAX &&
         glucoseHdlCache[glucoseConfigList[state]] == 0)
  {
    state++;
  }

  // Return if reached end of list
  if (state >= GLUCOSE_CONFIG_MAX)
  {
    glucCollCharHdls = true;

    return GLUCOSE_CONFIG_CMPL;
  }

  // Determine what to do with characteristic
  switch (glucoseConfigList[state])
  {
    // Read these characteristics
    case HDL_DEVINFO_SYSTEM_ID:
    case HDL_DEVINFO_MANUFACTURER_NAME:
    case HDL_DEVINFO_MODEL_NUM:
    case HDL_GLUCOSE_FEATURE:
      read = TRUE;
      break;

    // Set notification for these characteristics
    case HDL_GLUCOSE_MEAS_CCCD:
    case HDL_GLUCOSE_CONTEXT_CCCD:
      read = FALSE;
      charCfg = GATT_CLIENT_CFG_NOTIFY;
      break;

    // Set indication for these characteristics
    case HDL_GLUCOSE_CTL_PNT_CCCD:
      read = FALSE;
      charCfg = GATT_CLIENT_CFG_INDICATE;
      break;

    default:
      return state;
  }

  // Do a GATT read or write
  if (read)
  {
    attReadReq_t readReq;

    readReq.handle = glucoseHdlCache[glucoseConfigList[state]];

    // Send the read request
    GATT_ReadCharValue(glucCollConnHandle, &readReq, glucCollTaskId);
  }
  else
  {
    attWriteReq_t writeReq;

    writeReq.pValue = GATT_bm_alloc(glucCollConnHandle, ATT_WRITE_REQ, 2, NULL);
    if (writeReq.pValue != NULL)
    {
      writeReq.len = 2;
      writeReq.pValue[0] = LO_UINT16(charCfg);
      writeReq.pValue[1] = HI_UINT16(charCfg);
      writeReq.sig = 0;
      writeReq.cmd = 0;

      writeReq.handle = glucoseHdlCache[glucoseConfigList[state]];

      // Send the write request
      if (GATT_WriteCharValue(glucCollConnHandle, &writeReq,
                              glucCollTaskId) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&writeReq, ATT_WRITE_REQ);
      }
    }
  }

  return state;
}

/*********************************************************************
 * @fn      glucoseConfigGattMsg()
   *
 * @brief   Handle GATT messages for characteristic configuration.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New configuration state.
 */
uint8_t glucoseConfigGattMsg(uint8_t state, gattMsgEvent_t *pMsg)
{
  if ((pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP) &&
      (pMsg->hdr.status == SUCCESS))
  {
    // Process response
    switch (glucoseConfigList[state])
    {
      case HDL_GLUCOSE_MEAS_CCCD:
        break;

      case HDL_GLUCOSE_CONTEXT_CCCD:
        break;

      case HDL_GLUCOSE_CTL_PNT_CCCD:
        break;

      case HDL_GLUCOSE_FEATURE:
        glucoseFeatures = BUILD_UINT16(pMsg->msg.readRsp.pValue[0],
                                       pMsg->msg.readRsp.pValue[1]);
        break;

      case HDL_DEVINFO_SYSTEM_ID:
        break;

      case HDL_DEVINFO_MANUFACTURER_NAME:
        break;

      case HDL_DEVINFO_MODEL_NUM:
        break;

      default:
        break;
    }
  }

  return glucoseConfigNext(state + 1);
}

/*********************************************************************
 *********************************************************************/
