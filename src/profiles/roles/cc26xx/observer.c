/******************************************************************************

 @file  observer.c

 @brief GAP Observer Role for RTOS Applications

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
#include "util.h"

#include "gattservapp.h"
#include "observer.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Task configuration
#define GAPOBSERVERROLE_TASK_PRIORITY     3

#ifndef GAPOBSERVERROLE_TASK_STACK_SIZE
#define GAPOBSERVERROLE_TASK_STACK_SIZE   400
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
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

// Task setup
Task_Struct gapObserverRoleTask;
Char gapObserverRoleTaskStack[GAPOBSERVERROLE_TASK_STACK_SIZE];

// App callbacks
static gapObserverRoleCB_t *pGapObserverRoleCB;

/*********************************************************************
 * Profile Parameters - reference GAPCENTRALROLE_PROFILE_PARAMETERS for
 * descriptions
 */

static uint8_t gapObserverRoleBdAddr[B_ADDR_LEN];
static uint8_t gapObserverRoleMaxScanRes = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gapObserverRole_init(void);
static void gapObserverRole_taskFxn(UArg a0, UArg a1);

static uint8_t gapObserverRole_processStackMsg(ICall_Hdr *pMsg);
static uint8_t gapObserverRole_ProcessGAPMsg(gapEventHdr_t *pMsg);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief   Start the device in Observer role.  This function is typically
 *          called once during system startup.
 *
 * Public function defined in observer.h.
 */
bStatus_t GAPObserverRole_StartDevice(gapObserverRoleCB_t *pAppCallbacks)
{
  if (pAppCallbacks)
  {
    pGapObserverRoleCB = pAppCallbacks;
  }

  return GAP_DeviceInit(selfEntity, GAP_PROFILE_OBSERVER,
                        gapObserverRoleMaxScanRes, NULL, NULL, NULL);
}

/**
 * @brief   Set a parameter in the Observer Profile.
 *
 * Public function defined in observer.h.
 */
bStatus_t GAPObserverRole_SetParameter(uint16_t param, uint8_t len, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GAPOBSERVERROLE_MAX_SCAN_RES:
      if (len == sizeof (uint8_t))
      {
        gapObserverRoleMaxScanRes = *((uint8_t*)pValue);
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

  return ret;
}

/**
 * @brief   Get a parameter in the Observer Profile.
 *
 * Public function defined in observer.h.
 */
bStatus_t GAPObserverRole_GetParameter(uint16_t param, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GAPOBSERVERROLE_BD_ADDR:
      VOID osal_memcpy(pValue, gapObserverRoleBdAddr, B_ADDR_LEN) ;
      break;

    case GAPOBSERVERROLE_MAX_SCAN_RES:
      *((uint8_t*)pValue) = gapObserverRoleMaxScanRes;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ret;
}

/**
 * @brief   Start a device discovery scan.
 *
 * Public function defined in observer.h.
 */
bStatus_t GAPObserverRole_StartDiscovery(uint8_t mode, uint8_t activeScan,
                                         uint8_t whiteList)
{
  gapDevDiscReq_t params;

  params.taskID = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG,
                                            selfEntity);
  params.mode = mode;
  params.activeScan = activeScan;
  params.whiteList = whiteList;

  return GAP_DeviceDiscoveryRequest(&params);
}

/**
 * @brief   Cancel a device discovery scan.
 *
 * Public function defined in observer.h.
 */
bStatus_t GAPObserverRole_CancelDiscovery(void)
{
  return GAP_DeviceDiscoveryCancel(selfEntity);
}

/**
 * @brief   Task creation function for the GAP Observer Role.
 *
 * @param   none
 *
 * @return  none
 */
void GAPObserverRole_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gapObserverRoleTaskStack;
  taskParams.stackSize = GAPOBSERVERROLE_TASK_STACK_SIZE;
  taskParams.priority = GAPOBSERVERROLE_TASK_PRIORITY;

  Task_construct(&gapObserverRoleTask, gapObserverRole_taskFxn, &taskParams, NULL);
}

/**
 * @brief   Observer Profile Task initialization function.
 *
 * @param   none
 *
 * @return  void
 */
void gapObserverRole_init(void)
{
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
}

/**
 * @brief   Task entry point for the GAP Observer Role.
 *
 * @param   a0 - first argument
 * @param   a1 - second argument
 *
 * @return  none
 */
static void gapObserverRole_taskFxn(UArg a0, UArg a1)
{
  // Initialize profile
  gapObserverRole_init();

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
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          safeToDealloc = gapObserverRole_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      gapObserverRole_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming GAP message, FALSE otherwise.
 */
static uint8_t gapObserverRole_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      safeToDealloc = gapObserverRole_ProcessGAPMsg((gapEventHdr_t *) pMsg);
      break;

    default:
      break;
  }

  return ( safeToDealloc );
}

/*********************************************************************
 * @fn      gapObserverRole_ProcessGAPMsg
 *
 * @brief   Process an incoming task message from GAP.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming GAP message, FALSE otherwise.
 */
static uint8_t gapObserverRole_ProcessGAPMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *) pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          // Save off the information
          VOID memcpy(gapObserverRoleBdAddr, pPkt->devAddr, B_ADDR_LEN);
        }
      }
      break;

    default:
      break;
  }

  // Pass event to app
  if (pGapObserverRoleCB && pGapObserverRoleCB->eventCB)
  {
    return (pGapObserverRoleCB->eventCB((gapObserverRoleEvent_t *)pMsg));
  }

  return (TRUE);
}

/*********************************************************************
*********************************************************************/
