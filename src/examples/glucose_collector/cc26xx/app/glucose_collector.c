/******************************************************************************

 @file  glucose_collector.c

 @brief This file contains the Glucose Collector sample application for use
        with the CC26xx Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2010-2018, Texas Instruments Incorporated
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
/*********************************************************************
 * INCLUDES
 */

#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "gatt.h"
#include "gatt_profile_uuid.h"
#include "central.h"
#include "gapbondmgr.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "glucservice.h"
#include "osal_snv.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"
#include "icall_apimsg.h"
#include "utc_clock.h"

#include "glucose_collector.h"


/*********************************************************************
 * CONSTANTS
 */

// Task configuration
#define GLUCOLL_TASK_PRIORITY                     1
#define GLUCOLL_TASK_STACK_SIZE                   644

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header
  uint8_t *pData;  // Event data
} glucCollEvt_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Set to TRUE to filter record requests, FALSE to request/delete all
#define GLUCOSE_FILTER_ENABLED                FALSE

// Filter by time or sequence number: CTL_PNT_FILTER_SEQNUM or CTL_PNT_FILTER_TIME
#define DEFAULT_FILTER_TYPE                   CTL_PNT_FILTER_SEQNUM

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection
// is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE //FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_DISPLAY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Task ID for internal task/event processing
uint8_t glucCollTaskId;

// Connection handle of current connection
uint16_t glucCollConnHandle = GAP_CONNHANDLE_INIT;

uint16_t glucoseFeatures = 0;

// Discovered characteristic handles
bool glucCollCharHdls = false;

bool glucCollWritePending = false;

bool glucCollClearPending = false;

// Clock instances for internal periodic events.
Clock_Struct procTimeoutClock;

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

// Clock instances for internal periodic events.
static Clock_Struct discoveryClock;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration
static Task_Struct glucCollCentralTask;
static Char glucCollCentralTaskStack[GLUCOLL_TASK_STACK_SIZE];

#if GLUCOSE_FILTER_ENABLED
#if DEFAULT_FILTER_TYPE == CTL_PNT_FILTER_TIME
// These filters should return the first two records in the sample app only.
// Note dates are in UTC time; day and month start at 0
static UTCTimeStruct filterTime1 = {0,0,0,0,0,2011};
static UTCTimeStruct filterTime2 = {0,0,0,27,1,2011};
static void *pFilter1 = &filterTime1;
static void *pFilter2 = &filterTime2;
#else
// Sequence number filters
static uint16_t filterSeqNum1 = 1;
static uint16_t filterSeqnum2 = 10;
static void *pFilter1 = &filterSeqNum1;
static void *pFilter2 = &filterSeqnum2;
#endif
#endif // GLUCOSE_FILTER_ENABLED

// GAP GATT Attributes
static const uint8_t glucCollDeviceName[GAP_DEVICE_NAME_LEN] = "Glucose Collector";

// Number of scan results and scan result index
static uint8_t glucCollScanRes;
static uint8_t glucCollScanIdx;

// Scan result list
static gapDevRec_t glucCollDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8_t glucCollScanning = FALSE;

// Application state
static uint8_t glucCollState = BLE_STATE_IDLE;

// Discovery state
static uint8_t glucCollDiscState = DISC_IDLE;

// Characteristic configuration state
static uint8_t glucCollConfigState = GLUCOSE_CONFIG_START;

// TRUE if pairing started
static uint8_t glucCollPairingStarted = FALSE;

// TRUE if discovery postponed due to pairing
static uint8_t glucCollDiscPostponed = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void glucCollCentral_Init();
static void glucCollCentral_processAppMsg(glucCollEvt_t *pMsg);
static uint8_t glucCollCentral_enqueueMsg(uint8_t event, uint8_t state,
                                          uint8_t *pData);
static void glucCollCentral_keyChangeHandler(uint8_t keys);
static void glucCollCentral_taskFxn(UArg a0, UArg a1);
static void glucCollCentral_processGATTMsg(gattMsgEvent_t *pMsg);

static uint8_t glucCollCentral_eventCB(gapCentralRoleEvent_t *pEvent);

static uint8_t glucCollCentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void glucCollCentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs);
static void glucCollCentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);
static void glucCollCentral_processPasscode (uint16_t connHandle,
                                             uint8_t uiInputs,
                                             uint8_t uiOutputs);
static void glucCollCentral_processPairState(uint16_t connHandle, uint8_t state,
                                             uint8_t status);
static void glucCollCentral_handleKeys(uint8_t shift, uint8_t keys);
static void glucCollCentral_processStackMsg(ICall_Hdr *pMsg);
static void glucCollCentral_startDiscovery(void);
static bool glucCollCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                        uint8_t dataLen);
static void glucCollCentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void glucCollCentral_clockHandler(UArg arg);

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t glucCollRoleCB =
{
  glucCollCentral_eventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t glucCollBondCB =
{
  (pfnPasscodeCB_t)glucCollCentral_passcodeCB,
  glucCollCentral_pairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      glucCollCentral_createTask
 *
 * @brief   Task creation function for the Glucose collector.
 *
 * @param   none
 *
 * @return  none
 */
void glucCollCentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = glucCollCentralTaskStack;
  taskParams.stackSize = GLUCOLL_TASK_STACK_SIZE;
  taskParams.priority = GLUCOLL_TASK_PRIORITY;

  Task_construct(&glucCollCentralTask, glucCollCentral_taskFxn,
                 &taskParams, NULL);
}

/*********************************************************************
 * @fn      glucCollCentral_Init
 *
 * @brief   Initialization function for the Glucose Collector App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 *
 * @return  none
 */
static void glucCollCentral_Init()
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);

  // Save the taskId
  glucCollTaskId = selfEntity;

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Create clock objects for discovery and procedure timeout
  Util_constructClock(&discoveryClock, glucCollCentral_clockHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false,
                      GLUCOLL_START_DISCOVERY_EVT);

  Util_constructClock(&procTimeoutClock, glucCollCentral_clockHandler,
                      GLUCOSE_PROCEDURE_TIMEOUT, 0, false,
                      GLUCOLL_PROCEDURE_TIMEOUT_EVT);

  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter (GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                 &scanRes);
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (uint8_t *)glucCollDeviceName);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(glucCollTaskId);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Register for all key events - This app will handle all key events
  Board_initKeys(glucCollCentral_keyChangeHandler);

  // Register with bond manager after starting device
  GAPBondMgr_Register((gapBondCBs_t *)&glucCollBondCB);

  // Start the Device
  VOID GAPCentralRole_StartDevice((gapCentralRoleCB_t *)&glucCollRoleCB);
}

/*********************************************************************
 * @fn     glucCollCentral_taskFxn
 *
 * @brief   Application task entry point for the Glucose collector.
 *
 * @param   a0, a1 - not used
 *
 * @return  none
 */
static void glucCollCentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  glucCollCentral_Init();

  // Application main loop
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
          glucCollCentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        glucCollEvt_t *pMsg = (glucCollEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          glucCollCentral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }

      if (events)
      {
        if (events & GLUCOLL_PROCEDURE_TIMEOUT_EVT)
        {
          events &= ~GLUCOLL_PROCEDURE_TIMEOUT_EVT;

          if (glucCollState == BLE_STATE_CONNECTED)
          {
            // disconnect
            glucCollState = BLE_STATE_DISCONNECTING;

            GAPCentralRole_TerminateLink(glucCollConnHandle);

            Display_print0(dispHandle, 0, 0, "Timeout");
            Display_clearLines(dispHandle, 1, 2);
          }
        }

        if (events & GLUCOLL_START_DISCOVERY_EVT)
        {
          events &= ~GLUCOLL_START_DISCOVERY_EVT;

          if (glucCollPairingStarted)
          {
            // Postpone discovery until pairing completes
            glucCollDiscPostponed = TRUE;
          }
          else
          {
            glucCollCentral_startDiscovery();
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      glucCollCentral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void glucCollCentral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      glucCollCentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void glucCollCentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      glucCollCentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      glucCollCentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      glucCollCentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void glucCollCentral_processAppMsg(glucCollEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case GLUCOLL_STATE_CHANGE_EVT:
      glucCollCentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case GLUCOLL_KEY_CHANGE_EVT:
      glucCollCentral_handleKeys(0, pMsg->hdr.state);
      break;

    case GLUCOLL_PASSCODE_NEEDED_EVT:
      {
        glucCollCentral_processPasscode(glucCollConnHandle, pMsg->pData[0], pMsg->pData[1]);

        // Free data.
        ICall_free(pMsg->pData);
      }
      break;

    case GLUCOLL_PAIRING_STATE_EVT:
      {
        glucCollCentral_processPairState(glucCollConnHandle, pMsg->hdr.state, pMsg->pData[0]);

        // Free data.
        ICall_free(pMsg->pData);
      }
      break;

    default:
      // Do nothing.
      break;
  }
}
/*********************************************************************
 * @fn      glucCollCentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events.
 *
 * @return  none
 */
static void glucCollCentral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_UP)
  {
    // Start or stop discovery
    if (glucCollState != BLE_STATE_CONNECTED)
    {
      if (!glucCollScanning)
      {
        glucCollScanning = TRUE;
        glucCollScanRes = 0;

        Display_print0(dispHandle, 0, 0, "Discovering...");
        Display_clearLines(dispHandle, 1, 2);

        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
    else if (glucCollState == BLE_STATE_CONNECTED &&
             glucCollCharHdls == true &&
             glucCollWritePending == false)
    {
      uint8_t status;
#if GLUCOSE_FILTER_ENABLED
     // Request number of records
      status = glucoseCtlPntWriteFilter(CTL_PNT_OP_GET_NUM, CTL_PNT_OPER_RANGE,
                                        DEFAULT_FILTER_TYPE, pFilter1, pFilter2);
#else
      // Request number of records
      status = glucoseCtlPntWrite(CTL_PNT_OP_GET_NUM, CTL_PNT_OPER_ALL);
#endif

      if(status == 0)
      {
        glucCollWritePending = true;
      }
    }

    return;
  }

  if (keys & KEY_LEFT)
  {
    // Display discovery results
    if (glucCollState != BLE_STATE_CONNECTED && !glucCollScanning &&
        glucCollScanRes > 0)
    {
        // Increment index of current result (with wraparound)
        glucCollScanIdx++;

        if (glucCollScanIdx >= glucCollScanRes)
        {
          glucCollScanIdx = 0;
        }

        Display_print1(dispHandle, 0, 0, "Device %d", glucCollScanIdx + 1);
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(glucCollDevList[glucCollScanIdx].addr));
        Display_clearLine(dispHandle, 2);
    }
    else if (glucCollState == BLE_STATE_CONNECTED)
    {
      if(glucoseCtlPntWrite(CTL_PNT_OP_ABORT, CTL_PNT_OPER_NULL) == 0)
      {
        glucCollWritePending = false;
      }
    }

    return;
  }

  if (keys & KEY_RIGHT)
  {
    // Request all records
    if (glucCollState == BLE_STATE_CONNECTED &&
        glucCollCharHdls == true &&
        glucCollWritePending == false)
    {
      uint8_t status;
#if GLUCOSE_FILTER_ENABLED
      status = glucoseCtlPntWriteFilter(CTL_PNT_OP_REQ, CTL_PNT_OPER_RANGE,
                                        DEFAULT_FILTER_TYPE, pFilter1, pFilter2);
#else
      status = glucoseCtlPntWrite(CTL_PNT_OP_REQ, CTL_PNT_OPER_ALL);
#endif

      if(status == SUCCESS)
      {
        glucCollWritePending = true;
      }
    }

    return;
  }

  if (keys & KEY_SELECT)
  {
    uint8_t addrType;
    uint8_t *peerAddr;

    // Connect or disconnect
    if (glucCollState == BLE_STATE_IDLE)
    {
      // if there is a scan result
      if (glucCollScanRes > 0)
      {
        // connect to current device in scan result
        peerAddr = glucCollDevList[glucCollScanIdx].addr;
        addrType = glucCollDevList[glucCollScanIdx].addrType;

        glucCollState = BLE_STATE_CONNECTING;

        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                     DEFAULT_LINK_WHITE_LIST,
                                     addrType, peerAddr);

        Display_print0(dispHandle, 0, 0, "Connecting");
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(peerAddr));
        Display_clearLine(dispHandle, 2);
      }
    }
    else if (glucCollState == BLE_STATE_CONNECTING ||
             glucCollState == BLE_STATE_CONNECTED)
    {
      // disconnect
      glucCollState = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink(glucCollConnHandle);

      Display_print0(dispHandle, 0, 0, "Disconnecting");
      Display_clearLines(dispHandle, 1, 2);
    }

    return;
  }

  if (keys & KEY_DOWN)
  {
    // Clear stored records
    if (glucCollState == BLE_STATE_CONNECTED &&
        glucCollCharHdls == true &&
        glucCollWritePending == false)
    {
      uint8_t status;
#if GLUCOSE_FILTER_ENABLED
      status = glucoseCtlPntWriteFilter(CTL_PNT_OP_CLR, CTL_PNT_OPER_RANGE,
                                        DEFAULT_FILTER_TYPE, pFilter1, pFilter2);
#else
      status = glucoseCtlPntWrite(CTL_PNT_OP_CLR, CTL_PNT_OPER_ALL);
#endif

      if(status == 0)
      {
        glucCollWritePending = true;
        glucCollClearPending = true;
      }
    }
    else if (glucCollState != BLE_STATE_CONNECTED)
    {
      // erase all bonds
      GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
      Display_print0(dispHandle, 0, 0, "Erasing bonds");
      Display_clearLines(dispHandle, 1, 2);

      // initiate service discovery again
      glucCollCharHdls = false;
    }
  }
}

/*********************************************************************
 * @fn     glucCollCentral_processGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @param   pMsg - pointer to Gatt Message Event
 *
 * @return  none
 */
static void glucCollCentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_HANDLE_VALUE_NOTI ||
      pMsg->method == ATT_HANDLE_VALUE_IND)
  {
    glucoseIndGattMsg(pMsg);
  }
  else if (pMsg->method == ATT_READ_RSP ||
           pMsg->method == ATT_WRITE_RSP)
  {
    if(glucCollCharHdls == true)
    {
       glucoseCtlPntGattMsg(pMsg);
    }
    else
    {
      glucCollConfigState = glucoseConfigGattMsg (glucCollConfigState, pMsg);
    }
  }
  else if(glucCollDiscState != DISC_IDLE)
  {
    glucCollDiscState = glucoseDiscGattMsg(glucCollDiscState, pMsg);

    if (glucCollDiscState == DISC_IDLE)
    {
      // Start characteristic configuration
      glucCollConfigState = glucoseConfigNext(GLUCOSE_CONFIG_START);
    }
  }
  else if (pMsg->method == ATT_ERROR_RSP)
  {
    if(glucCollCharHdls == true)
    {
       glucoseCtlPntGattMsg(pMsg);
    }
    else
    {
      glucCollConfigState = glucoseConfigGattMsg (glucCollConfigState, pMsg);
    }
  }

  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      glucCollCentral_processRoleEvent
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t glucCollCentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        Display_print0(dispHandle, 0, 0, "Gluc. Collector");
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (glucCollCentral_findSvcUuid(GLUCOSE_SERV_UUID,
                                          pEvent->deviceInfo.pEvtData,
                                          pEvent->deviceInfo.dataLen))
          {
            glucCollCentral_addDeviceInfo(pEvent->deviceInfo.addr,
                                          pEvent->deviceInfo.addrType);
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        glucCollScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          glucCollScanRes = pEvent->discCmpl.numDevs;
          memcpy(glucCollDevList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * pEvent->discCmpl.numDevs));
        }

        Display_print1(dispHandle, 0, 0, "Devices Found %d", glucCollScanRes);

        if (glucCollScanRes > 0)
        {
          Display_print0(dispHandle, 1, 0, "<- To Select");
        }

        // initialize scan index to last device
        glucCollScanIdx = glucCollScanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          glucCollState = BLE_STATE_CONNECTED;
          glucCollConnHandle = pEvent->linkCmpl.connectionHandle;

          // If service discovery not performed initiate service discovery
          if (glucCollCharHdls == false)
          {
            // start procedure timer
            Util_stopClock(&discoveryClock);
            Util_startClock(&discoveryClock);
          }

          Display_print0(dispHandle, 0, 0, "Connected");
          Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
        }
        else
        {
          glucCollState = BLE_STATE_IDLE;
          glucCollConnHandle = GAP_CONNHANDLE_INIT;
          glucCollDiscState = DISC_IDLE;

          Display_print0(dispHandle, 0, 0, "Connect Failed");
          Display_print1(dispHandle, 1, 0, "Reason: %d", pEvent->gap.hdr.status);
          Display_clearLine(dispHandle, 2);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        glucCollState = BLE_STATE_IDLE;
        glucCollConnHandle = GAP_CONNHANDLE_INIT;
        glucCollDiscState = DISC_IDLE;
        glucCollPairingStarted = false;
        glucCollDiscPostponed = false;
        glucCollClearPending = false;

        // stop procedure timer
        Util_stopClock(&procTimeoutClock);

        Display_print0(dispHandle, 0, 0, "Disconnected");
        Display_print1(dispHandle, 1, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 2);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print0(dispHandle, 0, 0, "Param Update");
        Display_clearLines(dispHandle, 1, 2);
      }
      break;

    default:
      break;
  }

  return (TRUE);
}

/*********************************************************************
 * @fn      glucCollCentral_processPairState
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void glucCollCentral_processPairState(uint16_t connHandle, uint8_t state,
                                             uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    glucCollPairingStarted = true;

    Display_print0(dispHandle, 0, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    glucCollPairingStarted = false;

    if (status == SUCCESS)
    {
      // If discovery was postponed start discovery
      if (glucCollDiscPostponed &&  glucCollCharHdls == false)
      {
        glucCollDiscPostponed = false;

        // Set START_DISCOVERY event
        events |= GLUCOLL_START_DISCOVERY_EVT;

        // Wake up the application thread when it waits for clock event
        Semaphore_post(sem);
      }

      Display_print0(dispHandle, 0, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 0, 0, "Pairing fail %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 0, 0, "Bonding success");
    }
  }
}

/*********************************************************************
 * @fn      glucCollCentral_processPasscode
 *
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode
 * @param   uiOutputs  - display passcode
 *
 * @return  none
 */
static void glucCollCentral_processPasscode(uint16_t connHandle,
                                            uint8_t uiInputs,
                                            uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Is the callback to get the passcode from or display it to the user?
  if (uiInputs != 0)
  {
    // Passcode must be entered by the user but use the default passcode for now
    passcode = DEFAULT_PASSCODE;
  }
  else
  {
    // Create random passcode
    passcode = Util_GetTRNG();
  }

  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      glucCollCentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @param   none
 *
 * @return  none
 */
static void glucCollCentral_startDiscovery(void)
{
  glucCollDiscState = glucoseDiscStart();
}

/*********************************************************************
 * @fn      glucCollCentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @param   uuid     - service UUID
 * @param   pData    - device info data
 * @param   dataLen  - device info data length
 *
 * @return  TRUE if service UUID found
 */
static bool glucCollCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                        uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if (adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE)
      {
        pData++;
        adLen--;

        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if (pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid))
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      glucCollCentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @param   pAddr    - device address
 * @param   addrType - device address type
 *
 * @return  none
 */
static void glucCollCentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (glucCollScanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < glucCollScanRes; i++)
    {
      if (memcmp(pAddr, glucCollDevList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(glucCollDevList[glucCollScanRes].addr, pAddr, B_ADDR_LEN);
    glucCollDevList[glucCollScanRes].addrType = addrType;

    // Increment scan result count
    glucCollScanRes++;
  }
}

/*********************************************************************
 * @fn      glucCollCentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - input key
 *
 * @return  none
 */
void glucCollCentral_keyChangeHandler(uint8_t keys)
{
  glucCollCentral_enqueueMsg(GLUCOLL_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      glucCollCentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t glucCollCentral_enqueueMsg(uint8_t event, uint8_t state,
                                          uint8_t *pData)
{
  glucCollEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(glucCollEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      glucCollCentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t glucCollCentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (glucCollCentral_enqueueMsg(GLUCOLL_STATE_CHANGE_EVT,
                                 SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      glucCollCentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void glucCollCentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  uint8_t *psEvt;

  // Allocate message data.
  if ((psEvt = ICall_malloc(sizeof(uint8_t))))
  {
    *psEvt = status;

    // Queue the event.
    glucCollCentral_enqueueMsg(GLUCOLL_PAIRING_STATE_EVT, state, psEvt);
  }
}

/*********************************************************************
 * @fn      glucCollCentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - device address
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode
 * @param   uiOutputs  - display passcode
 *
 * @return  none
 */
static void glucCollCentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pcEvt;

  // Allocate message data.
  if ((pcEvt = ICall_malloc(sizeof(uint8_t) * 2)))
  {
    pcEvt[0] = uiInputs;
    pcEvt[1] = uiOutputs;

    // Queue the event.
    glucCollCentral_enqueueMsg(GLUCOLL_PASSCODE_NEEDED_EVT, 0, pcEvt);
  }
}

/*********************************************************************
*********************************************************************/
