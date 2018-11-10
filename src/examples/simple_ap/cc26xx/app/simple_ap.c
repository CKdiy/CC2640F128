/******************************************************************************

 @file  simple_ap.c

 @brief This file contains the host chip sample application for use with the
        CC2650 Bluetooth Low Energy Simple Network
        Processor.

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
#include "util.h"

#include "simple_gatt_profile.h"
#include "simple_ap.h"

#include <ti/mw/display/Display.h>
#include "board_key.h"
#include "board.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             SAP_GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

#define DEFAULT_SECURITY_IOCAPS               SAP_DISPLAY_YES_NO

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
#ifdef SNP_LOCAL
#define AP_TASK_STACK_SIZE                   780
#else // !SNP_LOCAL
#define AP_TASK_STACK_SIZE                   712
#endif // SNP_LOCAL
#endif // AP_TASK_STACK_SIZE

// Application Events
// Note:  Event_Id_30 and Event_Id_31 are reserved for the SNP when SNP_LOCAL
// is defined
#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication
#define AP_EVT_ADV_ENB                       Event_Id_01     // Advertisement Enable
#define AP_EVT_ADV_END                       Event_Id_02     // Advertisement Ended
#define AP_EVT_CONN_EST                      Event_Id_03     // Connection Established Event
#define AP_EVT_CONN_TERM                     Event_Id_04     // Connection Terminated Event
#define AP_EVT_AUTHENTICATION                Event_Id_05     // Authentication IO Event
#define AP_EVT_SECURITY                      Event_Id_06     // Security State event
#define AP_EVT_START_PERIODIC_CLOCK          Event_Id_07     // Start the periodic clock Event
#define AP_EVT_BUTTON_SELECT                 Event_Id_24     // SELECT Button Press
#define AP_EVT_BUTTON_UP                     Event_Id_25     // UP Button Press
#define AP_EVT_BUTTON_DOWN                   Event_Id_26     // DOWN Button Press
#define AP_EVT_BUTTON_LEFT                   Event_Id_27     // LEFT Button Press
#define AP_EVT_BUTTON_RIGHT                  Event_Id_28     // RIGHT Button Press
#define AP_ERROR                             Event_Id_29     // Error


// How often to perform periodic event (in msec)
#define AP_PERIODIC_EVT_PERIOD               5000

#define AP_DEFAULT_CONN_HANDLE               0xFFFF

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                        0x000D
#define TI_ST_DEVICE_ID                      0x03
#define TI_ST_KEY_DATA_ID                    0x00

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

// SAP Parameters for opening serial port to SNP
SAP_Params sapParams;

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

// Connection Handle - only one device currently allowed to connect to SNP
static uint16_t connHandle = AP_DEFAULT_CONN_HANDLE;

// BD Addr of the NWP
static char nwpstr[] =  "NWP:  0xFFFFFFFFFFFF";
#define nwpstrIDX       8

// BD Addr of peer device in connection
static char peerstr[] = "Peer: 0xFFFFFFFFFFFF";
#define peerstrIDX       8

// Characteristic 3 Update string
static uint8 char3 = 3;
static char char3str[12];

// Characteristic 4
static uint8 char4 = 4;
static char char4str[12];

// Authenticaion data
snpAuthenticationEvt_t authEvt;

// Security event
snpSecurityEvt_t securityEvt;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void AP_init( void );
static void AP_taskFxn(UArg a0, UArg a1);
static void AP_initServices(void);
#if defined(TI_DRIVERS_LCD_INCLUDED)
static void AP_setSecurityReqs(void);
#endif //TI_DRIVERS_LCD_INCLUDED
static void AP_keyHandler(uint8 keys);
static void AP_clockHandler(UArg arg);

static void AP_asyncCB(uint8_t cmd1, void *pParams);
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param);

static void AP_SPWriteCB(uint8_t charID);
static void AP_SPcccdCB(uint8_t charID, uint16_t value);

static void AP_ClearLCD(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AP_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
static void AP_convertBdAddr2Str(char *str, uint8_t *pAddr)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--)
  {
    *str++ = hex[*--pAddr >> 4];
    *str++ = hex[*pAddr & 0x0F];
  }

  return;
}

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
#ifndef SNP_LOCAL
  // Create event for task synchronization.
  // Note: when SNP_LOCAL is defined, Event is created by SNP when registering
  // with ICall.
  apEvent = Event_create(NULL, NULL);
#endif //!SNP_LOcAL

  // Turn on LCD.
  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Register Key Handler
  Board_initKeys(AP_keyHandler);

  // Write to the LCD.
  Display_print0(dispHandle, 0, 0, "Application Processor");
  Display_print0(dispHandle, 1, 0, "Initializing"); // State


  // Register to receive notifications from Simple Profile if characteristics
  // have been written to
  SimpleProfile_RegisterAppCB(AP_SPWriteCB,AP_SPcccdCB);

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
  UInt events = 0;
  ap_States_t state = AP_RESET;
  uint8 enableAdv = SAP_ADV_STATE_ENABLE;
  uint8 disableAdv = SAP_ADV_STATE_DISABLE;

  // Initialize application
  AP_init();

  // Application main loop
  for (;;)
  {
    switch(state)
    {
      case AP_RESET:
        {
          // Initialize UART port parameters within SAP parameters
#ifdef SNP_LOCAL
          sapParams.port.local.syncHandle = &apEvent;
          SAP_initParams(SAP_PORT_LOCAL,&sapParams);
#else //!SNP_LOCAL
          SAP_initParams(SAP_PORT_REMOTE_UART,&sapParams);
#ifdef POWER_SAVING
          sapParams.port.remote.mrdyPinID = Board_MRDY; //SRF06 LED3 / RF1.2
          sapParams.port.remote.srdyPinID = Board_SRDY; //SRF06 LED4 / RF1.4
#endif //POWER_SAVING
#endif //SNP_LOCAL

          // Setup NP module
          SAP_open(&sapParams);

          // Register Application thread's callback to receive asynchronous requests from the NP.
          SAP_setAsyncCB(AP_asyncCB);

#ifndef SNP_LOCAL
          // Reset the NP, and await a powerup indication.
          // Clear any pending power indications received prior to this reset
          // call
          SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_PUI, 100000);

          // Reset the SNP.
          // Note: when SNP_LOCAL is defined, this will reset SAP too.
          SAP_reset();

          // Pend on powerup indication.
          SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_PUI, BIOS_WAIT_FOREVER);
#endif //SNP_LOCAL

          // Read BD ADDR
          SAP_setParam(SAP_PARAM_HCI,SNP_HCI_OPCODE_READ_BDADDR,0,NULL);

          // Setup Services - Service creation is blocking so no need to pend
          AP_initServices();

#if defined(TI_DRIVERS_LCD_INCLUDED)
          // Customize Security parameters for this device.
          AP_setSecurityReqs();
#endif //TI_DRIVERS_LCD_INCLUDED

          state = AP_START_ADV;
        }
        break;

      case AP_START_ADV:
        {
          AP_ClearLCD();
          Display_print0(dispHandle, 1, 0, "Advertising");
          Display_print0(dispHandle, 2, 0, nwpstr);

          // Set advertising data.
          SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_NOTCONN, sizeof(advertData),
                      advertData);

          // Set Scan Response data.
          SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_SCANRSP, sizeof(scanRspData),
                      scanRspData);

          // Enable Advertising and await NP response
          SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);

          events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_ADV_ENB +
                              AP_EVT_CONN_EST, BIOS_WAIT_FOREVER);

          if (!(events & AP_EVT_CONN_EST))
          {
            // Wait for connection or button press to cancel advertisement
            events = SIMPLE_AP_PEND(apEvent, AP_EVT_ADV_END,
                              AP_EVT_BUTTON_LEFT + AP_EVT_CONN_EST,
                              BIOS_WAIT_FOREVER);
          }

          state = ( events & AP_EVT_CONN_EST ) ? AP_CONNECTED : AP_CANCEL_ADV;
        }
        break;

      case AP_CONNECTED:
        // Update State and Characteristic values on LCD
        Display_print0(dispHandle, 1, 0, "Connected");
        Display_print0(dispHandle, 3, 0, peerstr);
        System_sprintf(char3str,"Char3: %d",char3);
        Display_print0(dispHandle, 5, 0, char3str);

        // Events that can happen during connection - Client Disconnection
        //                                          - AP Disconnection
        //                                          - Updated Char 4 value
        //                                          - Security/Bonding
        events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_BUTTON_LEFT +
                          AP_EVT_BUTTON_DOWN + AP_EVT_BUTTON_UP +
                          AP_EVT_CONN_TERM + AP_EVT_AUTHENTICATION +
                          AP_EVT_SECURITY + AP_EVT_START_PERIODIC_CLOCK,
                          BIOS_WAIT_FOREVER);

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
        else if ( events & AP_EVT_AUTHENTICATION )
        {
          if (authEvt.numCmp == 0)
          {
            // Passkey
            if (authEvt.display == TRUE)
            {
              //Display passkey "000000"
              Display_print0(dispHandle, 4, 0, "Passkey: 000000");

              SAP_setAuthenticationRsp(0);
            }
          }
          else
          {
            uint8_t numCmpResult;

            // Numeric Comparison
            Display_print1(dispHandle, 4, 0, "NumCmp: %d", authEvt.numCmp);

            // Pend until a left or right button press is detected
            events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_BUTTON_LEFT +
                              AP_EVT_BUTTON_RIGHT + AP_EVT_SECURITY,
                              BIOS_WAIT_FOREVER);


            if (events & (AP_EVT_BUTTON_LEFT | AP_EVT_BUTTON_RIGHT))
            {
              numCmpResult = (events & AP_EVT_BUTTON_LEFT) ? TRUE : FALSE;

              SAP_setAuthenticationRsp(numCmpResult);
            }
          }
        }
        else if ( events & AP_EVT_BUTTON_LEFT )
        {
          // Terminate Connection
          SAP_setParam(SAP_PARAM_CONN, SAP_CONN_STATE, sizeof(connHandle),
                      (uint8_t *)&connHandle);

          // Pend on connection termination
          SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_CONN_TERM, BIOS_WAIT_FOREVER);

          // Move to idle state.
          state = AP_IDLE;
        }
        else if ( events & (AP_EVT_BUTTON_UP + AP_EVT_BUTTON_DOWN) )
        {
          // Update LCD with new value if periodic clock is running
          if ( Util_isActive(&periodicClock) )
          {
            System_sprintf(char4str,"Char4: %d",char4);
            Display_print0(dispHandle, 6, 0, char4str);
          }
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
        break;

      case AP_CANCEL_ADV:
        Display_print0(dispHandle, 1, 0, "Advtsment Cancelled");

        // Cancel Advertisement
        SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv);

        SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_ADV_END, BIOS_WAIT_FOREVER);

        state = AP_IDLE;
        break;

      case AP_IDLE:
        AP_ClearLCD();
        Display_print0(dispHandle, 1, 0, "Idle");

        // Key Press triggers state change from idle, or an Advertising Event
        events = SIMPLE_AP_PEND(apEvent, AP_NONE, AP_EVT_BUTTON_RIGHT +
                          AP_EVT_ADV_ENB, BIOS_WAIT_FOREVER);

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
  ///////////////////// SNP Service Addition and Registry! /////////////////////
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

  ////////////////////////      Set event callback      ////////////////////////
  SAP_registerEventCB(AP_processSNPEventCB, 0xFFFF);

}

#if defined(TI_DRIVERS_LCD_INCLUDED)
/*********************************************************************
 * @fn      AP_setSecurityReqs
 *
 * @brief   Configure security capabilities of this device.
 *
 * @param   None.
 *
 * @return  None.
 */
static void AP_setSecurityReqs(void)
{
  uint8_t ioCaps = DEFAULT_SECURITY_IOCAPS;

  /*
   * Default Security parameters enable Bonding and no MITM protection.
   * MITM protection is determined by the IO Capabilities.
   * By default IO Capabilities are No Input No Output and MITM is disabled.
   *
   * Default behavior upon connection is to wait for a security request from
   * the Central device.
   */

  // Set the SAP IO Capabilities
  SAP_setParam(SAP_PARAM_SECURITY, SAP_SECURITY_IOCAPS, sizeof(uint8_t), &ioCaps);
}
#endif //TI_DRIVERS_LCD_INCLUDED

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
        snpConnEstEvt_t * connEstEvt = (snpConnEstEvt_t *)param;

        // Update Peer Addr String
        connHandle = connEstEvt->connHandle;
        AP_convertBdAddr2Str(&peerstr[peerstrIDX],connEstEvt->pAddr);

        // Notify state machine of established connection
        Event_post(apEvent, AP_EVT_CONN_EST);
      }
      break;

    case SNP_CONN_TERM_EVT:
      {
        connHandle = AP_DEFAULT_CONN_HANDLE;
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
    case SNP_AUTHENTICATION_EVT:
      {
        authEvt = *(snpAuthenticationEvt_t *)param;

        // Notify state machine of Authentication event.
        Event_post(apEvent, AP_EVT_AUTHENTICATION);
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

        Display_clearLine(dispHandle, 4);
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

          case SNP_HCI_CMD_RSP:
            {
              snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *)pParams;
              switch ( hciRsp->opcode )
              {
                case SNP_HCI_OPCODE_READ_BDADDR:
                  // Update NWP Addr String
                  AP_convertBdAddr2Str(&nwpstr[nwpstrIDX],hciRsp->pData);
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
      break;
    default:
      break;
  }
}

/*********************************************************************
 * @fn      AP_keyHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bit mask for which keys were pressed
 *
 * @return  none
 */
void AP_keyHandler(uint8_t keys)
{
  if (keys & KEY_UP)
  {
    Event_post(apEvent, AP_EVT_BUTTON_UP);

    // Increment Characteristic 4
    // Cannot update LCD within key handler
    char4++;
  }

  if (keys & KEY_LEFT)
  {
    Event_post(apEvent, AP_EVT_BUTTON_LEFT);
  }

  if (keys & KEY_RIGHT)
  {
    Event_post(apEvent, AP_EVT_BUTTON_RIGHT);
  }

  if (keys & KEY_SELECT)
  {
    Event_post(apEvent, AP_EVT_BUTTON_SELECT);
  }

  if (keys & KEY_DOWN)
  {
    Event_post(apEvent, AP_EVT_BUTTON_DOWN);

    // Decrement Characteristic 4
    // Cannot update LCD within key handler
    char4--;
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
          // Update LCD with new value of Characteristic 3
          SimpleProfile_GetParameter(charID,&char3);
          Display_print1(dispHandle, 5, 0, "Char3: %d", char3);
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

            Display_print0(dispHandle, 7, 0, "Update Enabled");

            // Update LCD with current value of Characteristic 4
            System_sprintf(char4str,"Char4: %d",char4);
            Display_print0(dispHandle, 6, 0, char4str);
          }
          else
          {
            // Flags are not set so make sure clock has stopped and clear
            // appropriate fields of LCD
            Util_stopClock(&periodicClock);
            Display_clearLines(dispHandle, 6, 7);
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

static void AP_ClearLCD(void)
{
  Display_print0(dispHandle, 0, 0, "Application Processor");
  Display_clearLines(dispHandle, 1, 7);
}
/*********************************************************************
*********************************************************************/
