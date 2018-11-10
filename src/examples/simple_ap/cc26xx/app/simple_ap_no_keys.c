/******************************************************************************

 @file  simple_ap_no_keys.c

 @brief This file contains the host chip sample application for use with the
        CC2650 Bluetooth Low Energy Simple Network Processor.

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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/drivers/UART.h>

#ifdef SNP_LOCAL
#include <ti/sap/snp_rpc_synchro.h>
#endif //SNP_LOCAL

#include <ti/sap/snp.h>
#include <ti/sap/sap.h>
#include "hal_defs.h"

#ifdef FEATURE_OAD
#include "oad_sap.h"
#include "oad_target.h"
#include <ti/sbl/sbl.h>
#include "ext_flash_layout.h"
#endif //FEATURE_OAD

#include "util.h"
#include <ti/mw/display/Display.h>
#include "simple_gatt_profile.h"
#include "simple_ap.h"

#include "board.h"

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             SAP_GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 6=7.5ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6

// Maximum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     80
// White List Filter Policy, not used by default.
#define DEFAULT_WHITELIST_FILTER_POLICY       SAP_WHITELIST_DISABLE

/**
 * AP States.
 */
typedef enum
{
  AP_RESET,
  AP_IDLE,
  AP_START_ADV,
  AP_CONNECTED,
  AP_CANCEL_ADV,
  AP_OAD
} ap_States_t;

// Task configuration
#define AP_TASK_PRIORITY                     1

#ifndef AP_TASK_STACK_SIZE
#define AP_TASK_STACK_SIZE                   712
#endif

// Application Events
// Note:  Event_Id_30 and Event_Id_31 are reserved for the SNP when SNP_LOCAL
// is defined
#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication
#define AP_EVT_ADV_ENB                       Event_Id_01     // Advertisement Enable
#define AP_EVT_ADV_END                       Event_Id_02     // Advertisement Ended
#define AP_EVT_CONN_EST                      Event_Id_03     // Connection Established Event
#define AP_EVT_CONN_TERM                     Event_Id_04     // Connection Terminated Event
#define AP_EVT_SECURITY                      Event_Id_05     // Security state event
#define AP_EVT_START_PERIODIC_CLOCK          Event_Id_06     // Start the periodic clock Event
#define AP_EVT_OAD_ID_REQ                    Event_Id_07     // OAD Write Identify Request
#define AP_EVT_OAD_BLOCK_REQ                 Event_Id_08     // OAD Write Block Request
#define AP_EVT_OAD_SNP_IMAGE                 Event_Id_09     // OAD SNP Image received
#define AP_ERROR                             Event_Id_29     // Error

// How often to perform periodic event (in msec)
#define AP_PERIODIC_EVT_PERIOD               5000

#define AP_DEFAULT_CONN_HANDLE               0xFFFF

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D
#define TI_ST_DEVICE_ID                       0x03
#define TI_ST_KEY_DATA_ID                     0x00

#if defined (CC2650STK)
#define LOCAL_INTERFACE_ID      Board_UART0
#define RESET_PIN_ID            Board_DP0
#define BL_PIN_ID               Board_MRDY
#elif defined (CC2650_LAUNCHXL) && defined (LAUNCHPAD_SBL)
#define LOCAL_INTERFACE_ID      Board_UART0
#define RESET_PIN_ID            Board_DIO25_ANALOG
#define BL_PIN_ID               Board_MRDY
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;


/*********************************************************************
 * LOCAL VARIABLES
 */

// Used to block SNP calls during a synchronous transaction.
Event_Handle apEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Task configuration
Task_Struct apTask;
Char apTaskStack[AP_TASK_STACK_SIZE];

// NP Parameters for opening serial port to SNP
SAP_Params sapParams;

#if FEATURE_OAD
// Event data from OAD profile.
Queue_Struct oadQ;
Queue_Handle hOadQ;
#endif //FEATURE_OAD

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0xb,   // length of this data
  SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'C', 'C', '2', '6', '5', '0', ' ',
  'S', 'A', 'P',

  // connection interval range
  0x05,   // length of this data
  0x12, //GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  0x0A, //GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  SAP_GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | SAP_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // Manufacturer specific advertising data
  0x06,
  0xFF, //GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16(TI_COMPANY_ID),
  HI_UINT16(TI_COMPANY_ID),
  TI_ST_DEVICE_ID,
  TI_ST_KEY_DATA_ID,
  0x00                                    // Key state
};

#ifdef FEATURE_OAD
// Event data from OAD profile.
static oadTargetWrite_t oadWriteEventData;
#endif //FEATURE_OAD

// Characteristic 3 Update string
static uint8 char3 = 3;

// Characteristic 4
static uint8 char4 = 4;

// Security event
snpSecurityEvt_t securityEvt;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void AP_init( void );
static void AP_taskFxn(UArg a0, UArg a1);
static void AP_initServices(void);
static void AP_clockHandler(UArg arg);

static void AP_asyncCB(uint8_t cmd1, void *pParams);
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param);

static void AP_SPWriteCB(uint8_t charID);
static void AP_SPcccdCB(uint8_t charID, uint16_t value);

#ifdef FEATURE_OAD
static void AP_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                 uint8_t *pData, uint16_t dataLen);

static void AP_updateSNP(void);
#endif //FEATURE_OAD

/*********************************************************************
 * PROFILE CALLBACKS
 */

#ifdef FEATURE_OAD
static oadTargetCBs_t AP_oadCBs =
{
  AP_processOadWriteCB          // Write Callback.  Mandatory.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AP_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void AP_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = apTaskStack;
  taskParams.stackSize = AP_TASK_STACK_SIZE;
  taskParams.priority = AP_TASK_PRIORITY;

  Task_construct(&apTask, AP_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      AP_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void AP_init(void)
{
  // Create RTOS Event
  apEvent = Event_create(NULL, NULL);

  // Register to receive notifications from Simple Profile if characteristics
  // have been written to
  SimpleProfile_RegisterAppCB(AP_SPWriteCB, AP_SPcccdCB);

#ifdef FEATURE_OAD
  // Construct Msg Queue for OAD Writes
  hOadQ = Util_constructQueue(&oadQ);

  // Register to receive CB when OAD Profile characteristics have been written
  // to
  OAD_register((oadTargetCBs_t *)&AP_oadCBs);
#endif //FEATURE_OAD

  // Create periodic clock for internal periodic events.
  Util_constructClock(&periodicClock, AP_clockHandler,
                      0, AP_PERIODIC_EVT_PERIOD, false, 0);

}

/*********************************************************************
 * @fn      AP_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void AP_taskFxn(UArg a0, UArg a1)
{
  UInt events;
  ap_States_t state = AP_RESET;
  uint8 enableAdv = 1;
  uint8 disableAdv = 0;

#ifdef FEATURE_OAD
  uint8 firstRST = TRUE;
#endif //FEATURE_OAD

  // Initialize application
  AP_init();

  // Application main loop
  for (;;)
  {
    switch(state)
    {
      case AP_RESET:
        {
          // Initialize UART port parameters within NP parameters
#ifdef SNP_LOCAL
          sapParams.port.local.syncHandle = &apEvent;
          SAP_initParams(SAP_PORT_LOCAL,&sapParams);
#else //!SNP_LOCAL
          SAP_initParams(SAP_PORT_REMOTE_UART,&sapParams);

#ifdef POWER_SAVING
          sapParams.port.remote.mrdyPinID = Board_MRDY;
          sapParams.port.remote.srdyPinID = Board_SRDY;
#endif //POWER_SAVING
#endif //SNP_LOCAL

          // Setup NP module
          SAP_open(&sapParams);

          // Register Application thread's callback to receive asynchronous
          // requests from the NP.
          SAP_setAsyncCB(AP_asyncCB);

#ifndef SNP_LOCAL
          // Reset the NP, and await a powerup indication.
          // Clear any pending power indications received prior to this reset
          // call
          SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_PUI, 100000);

#ifdef FEATURE_OAD
          if (firstRST)
          {
            // Assuming that at SAP start up that SNP is already running
            SAP_reset();
          }
          else
          {
            // Reflashed SNP after OAD. Perform hard reset
            SBL_resetTarget(RESET_PIN_ID, BL_PIN_ID);
          }
#else
          SAP_reset();
#endif //FEATURE_OAD
          SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_PUI, BIOS_WAIT_FOREVER);
#endif //SNP_LOCAL
          // Read BD ADDR
          SAP_setParam(SAP_PARAM_HCI,SNP_HCI_OPCODE_READ_BDADDR,0,NULL);

          // Setup Services - Service creation is blocking so no need to pend
          AP_initServices();

          state = AP_START_ADV;
        }
        break;

      case AP_START_ADV:
        {
          // Set advertising data.
          SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_NOTCONN, sizeof(advertData),
                       advertData);

          // Set Scan Response data.
          SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_SCANRSP, sizeof(scanRspData),
                       scanRspData);

          // Enable Advertising and await NP response
          SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);
          SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_ADV_ENB, BIOS_WAIT_FOREVER);

          // Wait for connection
          events = SIMPLE_AP_PEND(apEvent, AP_EVT_ADV_END, AP_EVT_CONN_EST,
                            BIOS_WAIT_FOREVER);

          state = AP_CONNECTED;
        }
        break;

      case AP_CONNECTED:

        // Events that can happen during connection - Client Disconnection
        //                                          - Client OAD
        //                                          - Updated Char 4 value
        //                                          - Security/Bonding
        events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_OAD_ID_REQ +
                          AP_EVT_CONN_TERM + AP_EVT_SECURITY +
                          AP_EVT_START_PERIODIC_CLOCK, BIOS_WAIT_FOREVER);

        if ( events & AP_EVT_CONN_TERM )
        {
          // Stop periodic clock
          if (Util_isActive(&periodicClock))
          {
            Util_stopClock(&periodicClock);
          }

          // Client has disconnected from server
          state = AP_IDLE;
        }
        else if (events & AP_EVT_SECURITY)
        {
          if (securityEvt.state != securityEvt.status == SNP_SUCCESS)
          {
            uint8_t setWhiteList = DEFAULT_WHITELIST_FILTER_POLICY;

            if (setWhiteList == SAP_WHITELIST_ENABLE)
            {
              // Enabling White List Filtering on the bonded device if
              // configured to do so.
              SAP_setParam(SAP_PARAM_WHITELIST, 0, sizeof(uint8_t),
                           &setWhiteList);
            }
          }
        }
        else if (events & AP_EVT_START_PERIODIC_CLOCK)
        {
          // Start periodic clock.
          Util_startClock(&periodicClock);
        }
#ifdef FEATURE_OAD
        else if ( events & AP_EVT_OAD_ID_REQ )
        {
          // Dequeue OAD write message and handle
          if (!Queue_empty(hOadQ))
          {
                 oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);
                 OAD_imgIdentifyWrite(oadWriteEvt->connHandle,
                                      oadWriteEvt->pData);

                 // Free record and enter OAD state
                 free(oadWriteEvt);
                 state = AP_OAD;
          }
        }
#endif //FEATURE_OAD
        break;

#ifdef FEATURE_OAD
      case AP_OAD:
        events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_OAD_BLOCK_REQ +
                          AP_EVT_CONN_TERM + AP_EVT_OAD_ID_REQ,
                          BIOS_WAIT_FOREVER);

        if ( events & AP_EVT_OAD_BLOCK_REQ )
        {
          while ( !Queue_empty(hOadQ) )
          {
            // Dequeue OAD write message and handle
            oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);
            if (oadWriteEvt != NULL)
            {
              OAD_imgBlockWrite(oadWriteEvt->connHandle,
                                oadWriteEvt->pData);

              // Free record
              free(oadWriteEvt);

              events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_OAD_SNP_IMAGE,
                                BIOS_NO_WAIT);

              if (events & AP_EVT_OAD_SNP_IMAGE)
              {
                // Reset SNP and re-register SAP services
                firstRST = FALSE;
                state = AP_RESET;
                break;
              }
            }
          }
        }
        else if ( events & AP_EVT_OAD_ID_REQ )
        {
          // If OAD'ing multiple images the next ID REQ will come while still
          // in AP_OAD state

          // Dequeue OAD write message and handle
		  if (!Queue_empty(hOadQ))
		  {
            oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);
            if (oadWriteEvt != NULL)
            {
              OAD_imgIdentifyWrite(oadWriteEvt->connHandle,
                                 oadWriteEvt->pData);

              // Free record
              free(oadWriteEvt);
		    }
          }
        }
        else if ( events & AP_EVT_CONN_TERM )
        {
          // Stop periodic clock
          if (Util_isActive(&periodicClock))
          {
            Util_stopClock(&periodicClock);
          }

          // Client has disconnected from server
          state = AP_IDLE;
        }
        break;
#endif //FEATURE_OAD

      case AP_CANCEL_ADV:
        // Cancel Advertisement
        SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv);

        SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_ADV_END, BIOS_WAIT_FOREVER);

        state = AP_IDLE;
        break;


      case AP_IDLE:
        // Automatically re-enter advertisement state
        state = AP_START_ADV;
        break;

      default:
        break;
    }
  }
}

/*********************************************************************
 * @fn      AP_initServices
 *
 * @brief   Configure SNP and register services.
 *
 * @param   None.
 *
 * @return  None.
 */
static void AP_initServices(void)
{
  //////////////////////// SNP Service Addition and Registry! ////////////////////////
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue5 = 5;

    SimpleProfile_SetParameter(SP_CHAR1_ID, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SP_CHAR2_ID, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SP_CHAR3_ID, sizeof(char3),
                               &char3);
    SimpleProfile_SetParameter(SP_CHAR4_ID, sizeof(char4),
                               &char4);
    SimpleProfile_SetParameter(SP_CHAR5_ID, sizeof(uint8_t),
                               &charValue5);
  }

  // Add the SimpleProfile Service to the SNP.
  SimpleProfile_AddService();

#if FEATURE_OAD
  // Add the OAD Profile Service to the SNP
  OAD_addService();
#endif //FEATURE_OAD

  ////////////////////////      Set event callback      ////////////////////////
  SAP_registerEventCB(AP_processSNPEventCB, 0xFFFF);
}

/*
 * This is a callback operating in the NPI task.
 * These are events this application has registered for.
 *
 */
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
  switch(event)
  {
    case SNP_CONN_EST_EVT:
      {
        // Notify state machine of established connection
        Event_post(apEvent, AP_EVT_CONN_EST);
      }
      break;

    case SNP_CONN_TERM_EVT:
      {
        // Notify state machine of disconnection event
        Event_post(apEvent, AP_EVT_CONN_TERM);
      }
      break;

    case SNP_ADV_STARTED_EVT:
      {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *)param;
        if ( advEvt->status == SNP_SUCCESS )
        {
          // Notify state machine of Advertisement Enabled
          Event_post(apEvent, AP_EVT_ADV_ENB);
        }
        else
        {
          Event_post(apEvent, AP_ERROR);
        }
      }
      break;

    case SNP_ADV_ENDED_EVT:
      {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *)param;
        if ( advEvt->status == SNP_SUCCESS )
        {
          // Notify state machine of Advertisement Disabled
          Event_post(apEvent, AP_EVT_ADV_END);
        }
      }
      break;

    case SNP_SECURITY_EVT:
      {
        snpSecurityEvt_t * pEvt = (snpSecurityEvt_t * )param;

        // Only update and send on the last expected security message
        if ((pEvt->state == SNP_GAPBOND_PAIRING_STATE_COMPLETE &&
            pEvt->status != SNP_SUCCESS) ||
            pEvt->state == SNP_GAPBOND_PAIRING_STATE_BOND_SAVED ||
            pEvt->state == SNP_GAPBOND_PAIRING_STATE_BONDED)
        {
          securityEvt.state = pEvt->state;
          securityEvt.status = pEvt->status;

          Event_post(apEvent, AP_EVT_SECURITY);
        }

      }
      break;

    default:
      break;
  }
}

/*
 * This is a callback operating in the NPI task.
 * These are Asynchronous indications.
 *
 */
static void AP_asyncCB(uint8_t cmd1, void *pParams)
{
  switch(SNP_GET_OPCODE_HDR_CMD1(cmd1))
  {
    case SNP_DEVICE_GRP:
      {
        switch (cmd1)
        {
          case SNP_POWER_UP_IND:
            // Notify state machine of Power Up Indication
            Event_post(apEvent, AP_EVT_PUI);
            break;

          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

static void AP_clockHandler(UArg arg)
{
  // Set parameter value of char4. If notifications or indications have
  // been enabled, the profile will send them.
  SimpleProfile_SetParameter(SP_CHAR4_ID,sizeof(char4),&char4);
}

static void AP_SPWriteCB(uint8_t charID)
{
  switch( SP_ID_CHAR(charID) )
  {
    case SP_CHAR1:
      switch( SP_ID_CHARHTYPE(charID) )
      {
        case SP_VALUE:
          // Do nothing for now
          break;
        default:
          // Should not receive other types
          break;
      }
      break;
    case SP_CHAR3:
      switch( SP_ID_CHARHTYPE(charID) )
      {
        case SP_VALUE:
          // Do nothing for now
          break;
        default:
          // Should not receive other types
          break;
      }
      break;
    default:
      // Other Characteristics not writable
      break;
  }
}


static void AP_SPcccdCB(uint8_t charID, uint16_t value)
{
  switch( SP_ID_CHAR(charID) )
  {
    case SP_CHAR4:
      switch( SP_ID_CHARHTYPE(charID) )
      {
        case SP_CCCD:
          // If indication or notification flags are set start periodic
          // clock that will write to characteristic 4
          if ( value & ( SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE ) )
          {
            /*
             * Note: when SNP_LOCAL is defined, starting a clock with 0 duration
             * before the first expiry to begin periodic notifications will
             * execute its callback before SNP can update its CCCD value.  To
             * preclude this race condition, the clock should be synchronized
             * with an event.
             */
            Event_post(apEvent, AP_EVT_START_PERIODIC_CLOCK);
          }
          else
          {
            // Flags are not set so make sure clock has stopped and clear
            // appropriate fields of LCD
            Util_stopClock(&periodicClock);
          }
          break;

        default:
          // Should not receive other types
          break;
      }
      break;

    default:
      // Other Characteristics do not contain CCCD
      break;
  }
}

#ifdef FEATURE_OAD
void AP_processOadWriteCB(uint8_t event, uint16_t connHandle,
                          uint8_t *pData, uint16_t dataLen )
{
  if (event == OAD_IMAGE_COMPLETE )
  {
    // Perform SNP Update
    AP_updateSNP();

    // Notify App State machine
    Event_post(apEvent, AP_EVT_OAD_SNP_IMAGE);
  }
  else
  {
    oadTargetWrite_t *oadWriteEvt;
    oadWriteEvt = (oadTargetWrite_t *)malloc( sizeof(oadTargetWrite_t) +
                                              dataLen );

    if ( oadWriteEvt != NULL )
    {
      oadWriteEvt->event = event;
      oadWriteEvt->connHandle = connHandle;

      oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
      memcpy(oadWriteEvt->pData, pData, dataLen);

      if ( event == OAD_WRITE_IDENTIFY_REQ )
      {
        Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);
        Event_post(apEvent, AP_EVT_OAD_ID_REQ);
      }
      else if ( event == OAD_WRITE_BLOCK_REQ )
      {
        Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);
        Event_post(apEvent, AP_EVT_OAD_BLOCK_REQ);
      }
      else
      {
        free(oadWriteEvt);
      }
    }
  }
}

void AP_updateSNP(void)
{
  SBL_Params params;
  SBL_Image image;

  // Close NP so SBL can use serial port
  SAP_close();

  // Initialize SBL Params and open port to target device
  SBL_initParams(&params);
  params.resetPinID = RESET_PIN_ID;
  params.blPinID = BL_PIN_ID;
  params.targetInterface = SBL_DEV_INTERFACE_UART;
  params.localInterfaceID = Board_UART0;

  // If SBL cannot be opened the process cannot proceed
  if(SBL_SUCCESS == SBL_open(&params))
  {
    // Reset target and force into SBL code
    SBL_openTarget();

    // Flash new image to target
    image.imgType = SBL_IMAGE_TYPE_EXT;
    image.imgInfoLocAddr = EFL_IMAGE_INFO_ADDR_BLE;
    image.imgLocAddr = EFL_ADDR_IMAGE_BLE;
    image.imgTargetAddr = SNP_IMAGE_START;
    SBL_writeImage(&image);

    // Close SBL port to target device
    SBL_close();
  }
}
#endif //FEATURE_OAD

/*********************************************************************
*********************************************************************/
