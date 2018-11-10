/******************************************************************************

 @file  broadcaster.c

 @brief GAP Broadcaster Role for RTOS Applications

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
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <driverlib/ioc.h>

#include "gap.h"
#include "gatt.h"
#include "linkdb.h"
#include "util.h"

#include "gattservapp.h"
#include "broadcaster.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Profile Events
#define START_ADVERTISING_EVT         0x0001

#define DEFAULT_ADVERT_OFF_TIME       30000   // 30 seconds

// Task configuration
#define GAPROLE_TASK_PRIORITY         3

#ifndef GAPROLE_TASK_STACK_SIZE
#define GAPROLE_TASK_STACK_SIZE       400
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startAdvClock;

// Task pending events
static uint16_t events = 0;

// Task setup
Task_Struct gapRoleTask;
Char gapRoleTaskStack[GAPROLE_TASK_STACK_SIZE];

static gaprole_States_t gapRole_state;

/*********************************************************************
 * Profile Parameters - reference GAPROLE_PROFILE_PARAMETERS for
 * descriptions
 */

static uint8_t  gapRole_profileRole;
static uint8_t  gapRole_bdAddr[B_ADDR_LEN];
static uint8_t  gapRole_AdvEnabled = TRUE;
static uint16_t gapRole_AdvertOffTime = DEFAULT_ADVERT_OFF_TIME;
static uint8_t  gapRole_AdvertDataLen = 3;
static uint8_t  gapRole_AdvertData[B_MAX_ADV_LEN] =
{
  0x02,             // length of this data
  GAP_ADTYPE_FLAGS, // AD Type = Flags
  // BR/EDR not supported
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
static uint8_t  gapRole_ScanRspDataLen = 0;
static uint8_t  gapRole_ScanRspData[B_MAX_ADV_LEN] = {0};
static uint8_t  gapRole_AdvEventType;
static uint8_t  gapRole_AdvDirectType;
static uint8_t  gapRole_AdvDirectAddr[B_ADDR_LEN] = {0};
static uint8_t  gapRole_AdvChanMap;
static uint8_t  gapRole_AdvFilterPolicy;

// Application callbacks
static gapRolesCBs_t *pGapRoles_AppCGs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

/*********************************************************************
 * Profile Attributes - Table
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gapRole_init(void);
static void gapRole_taskFxn(UArg a0, UArg a1);

static void gapRole_processStackMsg(ICall_Hdr *pMsg);
static void gapRole_processGAPMsg(gapEventHdr_t *pMsg);
static void gapRole_SetupGAP(void);

static void gapRole_setEvent(uint32_t event);

/*********************************************************************
 * CALLBACKS
 */
void gapRole_clockHandler(UArg a0);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @brief   Set a GAP Role parameter.
 *
 * Public function defined in broadcaster.h.
 */
bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case GAPROLE_ADVERT_ENABLED:
      if (len == sizeof(uint8_t))
      {
        uint8_t oldAdvEnabled = gapRole_AdvEnabled;
        gapRole_AdvEnabled = *((uint8_t*)pValue);

        if ((oldAdvEnabled) && (gapRole_AdvEnabled == FALSE))
        {
          // Turn off Advertising
          if (gapRole_state == GAPROLE_ADVERTISING)
          {
            VOID GAP_EndDiscoverable(selfEntity);
          }
        }
        else if ((oldAdvEnabled == FALSE) && (gapRole_AdvEnabled))
        {
          // Turn on Advertising
          if ((gapRole_state == GAPROLE_STARTED)
              || (gapRole_state == GAPROLE_WAITING))
          {
            gapRole_setEvent(START_ADVERTISING_EVT);
          }
        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADVERT_OFF_TIME:
      if (len == sizeof (uint16_t))
      {
        gapRole_AdvertOffTime = *((uint16_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADVERT_DATA:
      if (len <= B_MAX_ADV_LEN)
      {
        VOID memset(gapRole_AdvertData, 0, B_MAX_ADV_LEN);
        VOID memcpy(gapRole_AdvertData, pValue, len);
        gapRole_AdvertDataLen = len;

        // Update the advertising data.
        ret = GAP_UpdateAdvertisingData(selfEntity, TRUE, gapRole_AdvertDataLen,
                                        gapRole_AdvertData);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_SCAN_RSP_DATA:
      if (len <= B_MAX_ADV_LEN)
      {
        VOID memset(gapRole_ScanRspData, 0, B_MAX_ADV_LEN);
        VOID memcpy(gapRole_ScanRspData, pValue, len);
        gapRole_ScanRspDataLen = len;

        // Update the scan response data.
        ret = GAP_UpdateAdvertisingData(selfEntity, FALSE,
                                        gapRole_ScanRspDataLen,
                                        gapRole_ScanRspData);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_EVENT_TYPE:
      if ((len == sizeof (uint8_t)) &&
          (*((uint8_t*)pValue) <= GAP_ADTYPE_ADV_LDC_DIRECT_IND))
      {
        gapRole_AdvEventType = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_DIRECT_TYPE:
      if ((len == sizeof (uint8_t)) &&
          (*((uint8_t*)pValue) <= ADDRMODE_PRIVATE_RESOLVE))
      {
        gapRole_AdvDirectType = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_DIRECT_ADDR:
      if (len == B_ADDR_LEN)
      {
        VOID memcpy(gapRole_AdvDirectAddr, pValue, B_ADDR_LEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_CHANNEL_MAP:
      if ((len == sizeof (uint8_t)) && (*((uint8_t*)pValue) <= 0x07))
      {
        gapRole_AdvChanMap = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPROLE_ADV_FILTER_POLICY:
      if ((len == sizeof (uint8_t)) &&
          (*((uint8_t*)pValue) <= GAP_FILTER_POLICY_WHITE))
      {
        gapRole_AdvFilterPolicy = *((uint8_t*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if ((param < TGAP_PARAMID_MAX) && (len == sizeof (uint16_t)))
      {
        ret = GAP_SetParamValue(param, *((uint16_t*)pValue));
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Get a GAP Role parameter.
 *
 * Public function defined in broadcaster.h.
 */
bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case GAPROLE_PROFILEROLE:
      *((uint8_t*)pValue) = gapRole_profileRole;
      break;

    case GAPROLE_BD_ADDR:
      VOID memcpy(pValue, gapRole_bdAddr, B_ADDR_LEN);
      break;

    case GAPROLE_ADVERT_ENABLED:
      *((uint8_t*)pValue) = gapRole_AdvEnabled;
      break;

    case GAPROLE_ADVERT_OFF_TIME:
      *((uint16_t*)pValue) = gapRole_AdvertOffTime;
      break;

    case GAPROLE_ADVERT_DATA:
      VOID memcpy(pValue , gapRole_AdvertData, gapRole_AdvertDataLen);
      break;

    case GAPROLE_SCAN_RSP_DATA:
      VOID memcpy(pValue, gapRole_ScanRspData, gapRole_ScanRspDataLen) ;
      break;

    case GAPROLE_ADV_EVENT_TYPE:
      *((uint8_t*)pValue) = gapRole_AdvEventType;
      break;

    case GAPROLE_ADV_DIRECT_TYPE:
      *((uint8_t*)pValue) = gapRole_AdvDirectType;
      break;

    case GAPROLE_ADV_DIRECT_ADDR:
      VOID memcpy(pValue, gapRole_AdvDirectAddr, B_ADDR_LEN) ;
      break;

    case GAPROLE_ADV_CHANNEL_MAP:
      *((uint8_t*)pValue) = gapRole_AdvChanMap;
      break;

    case GAPROLE_ADV_FILTER_POLICY:
      *((uint8_t*)pValue) = gapRole_AdvFilterPolicy;
      break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if (param < TGAP_PARAMID_MAX)
      {
        *((uint16_t*)pValue) = GAP_GetParamValue(param);
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return (ret);
}

/*********************************************************************
 * @brief   Does the device initialization.
 *
 * Public function defined in broadcaster.h.
 */
bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks)
{
  if (gapRole_state == GAPROLE_INIT)
  {
    // Clear all of the Application callbacks
    if (pAppCallbacks)
    {
      pGapRoles_AppCGs = pAppCallbacks;
    }

    // Start the GAP
    gapRole_SetupGAP();

    return (SUCCESS);
  }
  else
  {
    return (bleAlreadyInRequestedMode);
  }
}

/*********************************************************************
 * @fn      GAPRole_createTask
 *
 * @brief   Task creation function for the GAP Broadcaster Role.
 *
 * @param   none
 *
 * @return  none
 */
void GAPRole_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gapRoleTaskStack;
  taskParams.stackSize = GAPROLE_TASK_STACK_SIZE;
  taskParams.priority = GAPROLE_TASK_PRIORITY;

  Task_construct(&gapRoleTask, gapRole_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * @fn      gapRole_init
 *
 * @brief   Initialization function for the GAP Role Task.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_init(void)
{
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  gapRole_state = GAPROLE_INIT;

  // Setup timers as one-shot timers
  Util_constructClock(&startAdvClock, gapRole_clockHandler,
                      0, 0, false, START_ADVERTISING_EVT);

  // Initialize the Profile Advertising and Connection Parameters
  gapRole_profileRole = GAP_PROFILE_BROADCASTER;

  gapRole_AdvEventType = GAP_ADTYPE_ADV_NONCONN_IND;
  gapRole_AdvDirectType = ADDRMODE_PUBLIC;
  gapRole_AdvChanMap = GAP_ADVCHAN_ALL;
  gapRole_AdvFilterPolicy = GAP_FILTER_POLICY_ALL;
}

/*********************************************************************
 * @fn      gapRole_taskFxn
 *
 * @brief   Task entry point for the GAP Peripheral Role.
 *
 * @param   a0 - first argument
 * @param   a1 - second argument
 *
 * @return  none
 */
static void gapRole_taskFxn(UArg a0, UArg a1)
{
  // Initialize profile
  gapRole_init();

  // Profile main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          gapRole_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    if (events & START_ADVERTISING_EVT)
    {
      events &= ~START_ADVERTISING_EVT;

      if (gapRole_AdvEnabled)
      {
        gapAdvertisingParams_t params;

        // Setup advertisement parameters
        params.eventType = gapRole_AdvEventType;
        params.initiatorAddrType = gapRole_AdvDirectType;
        VOID memcpy(params.initiatorAddr, gapRole_AdvDirectAddr, B_ADDR_LEN);
        params.channelMap = gapRole_AdvChanMap;
        params.filterPolicy = gapRole_AdvFilterPolicy;

        if (GAP_MakeDiscoverable(selfEntity, &params) != SUCCESS)
        {
          gapRole_state = GAPROLE_ERROR;

          // Notify the application with the new state change
          if (pGapRoles_AppCGs && pGapRoles_AppCGs->pfnStateChange)
          {
            pGapRoles_AppCGs->pfnStateChange(gapRole_state);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      gapRole_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void gapRole_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      gapRole_processGAPMsg((gapEventHdr_t *)pMsg);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      gapRole_processGAPMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void gapRole_processGAPMsg(gapEventHdr_t *pMsg)
{
  uint8_t notify = FALSE;   // State changed notify the app? (default no)

  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
        bStatus_t stat = pPkt->hdr.status;

        if (stat == SUCCESS)
        {
          // Save off the information
          VOID memcpy(gapRole_bdAddr, pPkt->devAddr, B_ADDR_LEN);

          gapRole_state = GAPROLE_STARTED;

          // Update the advertising data
          stat = GAP_UpdateAdvertisingData(selfEntity, TRUE,
                                            gapRole_AdvertDataLen,
                                            gapRole_AdvertData);
        }

        if (stat != SUCCESS)
        {
          gapRole_state = GAPROLE_ERROR;
        }

        notify = TRUE;
      }
      break;

    case GAP_ADV_DATA_UPDATE_DONE_EVENT:
      {
        gapAdvDataUpdateEvent_t *pPkt = (gapAdvDataUpdateEvent_t *)pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          if (pPkt->adType)
          {
            // Setup the Response Data
            pPkt->hdr.status = GAP_UpdateAdvertisingData(selfEntity,
                              FALSE, gapRole_ScanRspDataLen, gapRole_ScanRspData);
          }
          else if ((gapRole_state != GAPROLE_ADVERTISING) &&
                    (Util_isActive(&startAdvClock) == FALSE))
          {
            // Start advertising
            gapRole_setEvent(START_ADVERTISING_EVT);
          }
        }

        if (pPkt->hdr.status != SUCCESS)
        {
          // Set into Error state
          gapRole_state = GAPROLE_ERROR;
          notify = TRUE;
        }
      }
      break;

    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    case GAP_END_DISCOVERABLE_DONE_EVENT:
      {
        gapMakeDiscoverableRspEvent_t *pPkt = (gapMakeDiscoverableRspEvent_t *)pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          if (pMsg->opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
          {
            gapRole_state = GAPROLE_ADVERTISING;
          }
          else // GAP_END_DISCOVERABLE_DONE_EVENT
          {

            if (gapRole_AdvertOffTime != 0)
            {
              if ((gapRole_AdvEnabled))
              {
                Util_restartClock(&startAdvClock, gapRole_AdvertOffTime);
              }
            }
            else
            {
              // Since gapRole_AdvertOffTime is set to 0, the device should not
              // automatically become discoverable again after a period of time.
              // Set enabler to FALSE; device will become discoverable again when
              // this value gets set to TRUE
              gapRole_AdvEnabled = FALSE;
            }

            // In the Advertising Off period
            gapRole_state = GAPROLE_WAITING;
          }
        }
        else
        {
          gapRole_state = GAPROLE_ERROR;
        }
        notify = TRUE;
      }
      break;

    default:
      break;
  }

  if (notify == TRUE)
  {
    // Notify the application
    if (pGapRoles_AppCGs && pGapRoles_AppCGs->pfnStateChange)
    {
      pGapRoles_AppCGs->pfnStateChange(gapRole_state);
    }
  }
}

/*********************************************************************
 * @fn      gapRole_SetupGAP
 *
 * @brief   Call the GAP Device Initialization function using the
 *          Profile Parameters.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_SetupGAP(void)
{
  VOID GAP_DeviceInit(selfEntity, gapRole_profileRole, 0, NULL, NULL, NULL);
}

/*********************************************************************
 * @fn      gapRole_setEvent
 *
 * @brief   Set an event
 *
 * @param   event - event to be set
 *
 * @return  none
 */
static void gapRole_setEvent(uint32_t event)
{
  events |= event;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      gapRole_clockHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - event
 *
 * @return  none
 */
void gapRole_clockHandler(UArg a0)
{
  gapRole_setEvent(a0);
}

/*********************************************************************
*********************************************************************/
