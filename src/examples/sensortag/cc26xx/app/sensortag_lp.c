/******************************************************************************

 @file  sensortag_lp.c

 @brief This file is the SensorTag application's main body.

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


/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <ICall.h>

#include "gatt.h"
#include "gatt_profile_uuid.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "osal_snv.h"
#include "util.h"
#include "icall_apimsg.h"

#include "board.h"
#include "ExtFlash.h"

#include "sensortag.h"
#include "sensortag_revision.h"
#include "devinfoservice.h"
#include "simplekeys.h"
#include "st_util.h"

#include "displayservice.h"
#include "sensortag_register.h"

#ifdef FACTORY_IMAGE
#include "sensortag_factoryreset.h"
#endif

// On-board devices
#include "sensortag_keys.h"
#include "sensortag_io.h"
#include "sensortag_batt.h"
#include "sensortag_oad.h"
#include "sensortag_conn_ctrl.h"

// Booster Pack devices
#include "sensortag_display.h"

/*******************************************************************************
 * CONSTANTS
 */

// How often to perform periodic event (in milliseconds)
#define ST_PERIODIC_EVT_PERIOD               1000

// What is the advertising interval when device is discoverable
// (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100

// Whether to enable automatic parameter update request when a
// connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         1

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D
#define TI_ST_DEVICE_ID                       0x03
#define TI_ST_KEY_DATA_ID                     0x00

// Length of board address
#define B_ADDR_STR_LEN                        15

#if defined (PLUS_BROADCASTER)
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// Task configuration
#define ST_TASK_PRIORITY                      1

#ifndef ST_TASK_STACK_SIZE
// Stack size may be overridden by project settings
#define ST_TASK_STACK_SIZE                    700
#endif

// Internal Events for RTOS application
#define ST_STATE_CHANGE_EVT                   0x0001
#define ST_CHAR_CHANGE_EVT                    0x0002
#define ST_PERIODIC_EVT                       0x0004
#define SBP_OAD_WRITE_EVT                     0x0008

// Misc.
#define INVALID_CONNHANDLE                    0xFFFF
#define TEST_INDICATION_BLINKS                5  // Number of blinks
#define OAD_PACKET_SIZE                       18
#define KEY_STATE_OFFSET                      13 // Offset in advertising data

/*******************************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  uint8_t event;  // Which profile's event
  uint8_t serviceID; // New status
  uint8_t paramID;
} stEvt_t;

/*******************************************************************************
 * GLOBAL VARIABLES
 */
// Profile state and parameters
gaprole_States_t gapProfileState = GAPROLE_INIT;

// Semaphore globally used to post events to the application thread
ICall_Semaphore sem;

// Entity ID globally used to check for source and/or destination of messages
ICall_EntityID selfEntityMain;

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Task configuration
static Task_Struct sensorTagTask;
static Char sensorTagTaskStack[ST_TASK_STACK_SIZE];

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// self-test result
static uint8_t selfTestMap;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
#ifdef CC1350_LAUNCHXL
  'C', 'C', '1', '3', '5', '0', ' ',
#else
  'C', 'C', '2', '6', '5', '0', ' ',
#endif
  'L', 'a', 'u',  'n',  'c',  'h',  'P',  'a',  'd',
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID),

  // Manufacturer specific advertising data
  0x06,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16(TI_COMPANY_ID),
  HI_UINT16(TI_COMPANY_ID),
  TI_ST_DEVICE_ID,
  TI_ST_KEY_DATA_ID,
  0x00                                    // Key state
};

// Device information parameters
#ifdef CC1350_LAUNCHXL
static const uint8_t devInfoModelNumber[] = "CC1350 LaunchPad";
#else
static const uint8_t devInfoModelNumber[] = "CC2650 LaunchPad";
#endif
static const uint8_t devInfoNA[] =          "N.A.";
static const uint8_t devInfoFirmwareRev[] = FW_VERSION_STR;
static const uint8_t devInfoMfrName[] =     "Texas Instruments";
static const uint8_t *devInfoHardwareRev =  devInfoNA;

// GAP GATT Attributes
static const uint8_t *attDeviceName = devInfoModelNumber;

// Pins that are actively used by the application
static PIN_Config SensortagAppPinTable[] =
{
    Board_GLED  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_RLED  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_BTN1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
    Board_BTN2  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */

    PIN_TERMINATE
};

/*******************************************************************************
 * LOCAL FUNCTIONS
 */

static void SensorTag_init(void);
static void SensorTag_taskFxn(UArg a0, UArg a1);
static void SensorTag_processStackMsg(ICall_Hdr *pMsg);
static void SensorTag_processGATTMsg(gattMsgEvent_t *pMsg);
static void SensorTag_processAppMsg(stEvt_t *pMsg);
static void SensorTag_processStateChangeEvt(gaprole_States_t newState) ;
static void SensorTag_processCharValueChangeEvt(uint8_t serviceID, uint8_t paramID) ;
static void SensorTag_performPeriodicTask(void);
static void SensorTag_stateChangeCB(gaprole_States_t newState);
static void SensorTag_resetAllModules(void);
static void SensorTag_clockHandler(UArg arg);
static void SensorTag_enqueueMsg(uint8_t event, uint8_t serviceID, uint8_t paramID);
static void SensorTag_callback(PIN_Handle handle, PIN_Id pinId);
static void SensorTag_setDeviceInfo(void);

/*******************************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t sensorTag_gapRoleCBs =
{
  SensorTag_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t sensorTag_bondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

/*******************************************************************************
 * @fn      SensorTag_createTask
 *
 * @brief   Task creation function for the SensorTag.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTag_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sensorTagTaskStack;
  taskParams.stackSize = ST_TASK_STACK_SIZE;
  taskParams.priority = ST_TASK_PRIORITY;

  Task_construct(&sensorTagTask, SensorTag_taskFxn, &taskParams, NULL);
}

/*******************************************************************************
 * @fn      SensorTag_testResult
 *
 * @brief   Get result of self-test
 *
 * @param   none
 *
 * @return  bitmap with result of self-test
 */
uint8_t SensorTag_testResult(void)
{
    return selfTestMap;
}

/*******************************************************************************
 * @fn      SensorTag_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (i.e. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTag_init(void)
{
  // Handling of buttons, LED, relay
  hGpioPin = PIN_open(&pinGpioState, SensortagAppPinTable);
  PIN_registerIntCb(hGpioPin, SensorTag_callback);

  // ***************************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ***************************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntityMain, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SensorTag_clockHandler,
                      ST_PERIODIC_EVT_PERIOD, 0, false, ST_PERIODIC_EVT);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void*)attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = FALSE;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes

  // Add device info service.
  DevInfo_AddService();

  // Add application specific device information
  SensorTag_setDeviceInfo();

  // Add battery monitor
  SensorTagBatt_init();

  // Power on self-test for flash
  if (ExtFlash_test())
  {
    selfTestMap = 1;
  }
  else
  {
    selfTestMap = 0;
  }

  if (selfTestMap == 1)
  {
    SensorTagIO_blinkLed(IOID_GREEN_LED, TEST_INDICATION_BLINKS);
  }
  else
  {
    SensorTagIO_blinkLed(IOID_RED_LED, TEST_INDICATION_BLINKS);
  }


#ifdef FACTORY_IMAGE
  // Check if a factory image exists and apply current image if necessary
  if (!SensorTagFactoryReset_hasImage())
  {
      if (!SensorTagFactoryReset_storeCurrentImage())
      {
          // Failure to store factory image should be reported as flash failure
          selfTestMap = 0;
      }
  }
#endif

  // Auxiliary services
  SensorTagKeys_init();                           // Simple Keys
  SensorTagIO_init();                             // IO (LED+buzzer+self test)
  SensorTagRegister_init();                       // Register Service
  SensorTagOad_init();                            // Over the Air Download

  // Booster Pack devices
  SensorTagDisplay_init();                        // Display DevPack

  // Start the Device
  GAPRole_StartDevice(&sensorTag_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&sensorTag_bondMgrCBs);

  // Enable interrupt handling for keys and relay
  PIN_registerIntCb(hGpioPin, SensorTag_callback);
}

/*******************************************************************************
 * @fn      SensorTag_taskFxn
 *
 * @brief   Application task entry point for the SensorTag
 *
 * @param   a0, a1 (not used)
 *
 * @return  none
 */
static void SensorTag_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SensorTag_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signalled when a
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
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntityMain))
        {
          // Process inter-task message
          SensorTag_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        stEvt_t *pMsg = (stEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SensorTag_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }

      // Process new data if available
      SensorTagKeys_processEvent();
      SensorTagBatt_processSensorEvent();
    }

    if (!!(events & ST_PERIODIC_EVT))
    {
      events &= ~ST_PERIODIC_EVT;

      if (gapProfileState == GAPROLE_CONNECTED
          || gapProfileState == GAPROLE_ADVERTISING)
      {
        Util_startClock(&periodicClock);
      }

      // Perform periodic application task
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        SensorTag_performPeriodicTask();
      }

      // Blink green LED when advertising
      if (gapProfileState == GAPROLE_ADVERTISING)
      {
        SensorTagIO_blinkLed(IOID_GREEN_LED, 1);
        SensorTagDisplay_showBatteryLevel();
      }
    }

    // OAD event queue
    SensorTagOad_processEvent();
  } // task loop
}


/*******************************************************************************
 * @fn      SensorTag_setDeviceInfo
 *
 * @brief   Set application specific Device Information
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTag_setDeviceInfo(void)
{
  DevInfo_SetParameter(DEVINFO_MODEL_NUMBER, sizeof(devInfoModelNumber),
                       (void*)devInfoModelNumber);
  DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, sizeof(devInfoNA),
                       (void*)devInfoNA);
  DevInfo_SetParameter(DEVINFO_SOFTWARE_REV, sizeof(devInfoNA),
                       (void*)devInfoNA);
  DevInfo_SetParameter(DEVINFO_FIRMWARE_REV, sizeof(devInfoFirmwareRev),
                       (void*)devInfoFirmwareRev);
  DevInfo_SetParameter(DEVINFO_HARDWARE_REV, sizeof(devInfoHardwareRev),
                       (void*)devInfoHardwareRev);
  DevInfo_SetParameter(DEVINFO_MANUFACTURER_NAME, sizeof(devInfoMfrName),
                       (void*)devInfoMfrName);
}

/*******************************************************************************
 * @fn      SensorTag_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SensorTag_processAppMsg(stEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case ST_STATE_CHANGE_EVT:
      SensorTag_processStateChangeEvt((gaprole_States_t)pMsg->serviceID);
      break;

    case ST_CHAR_CHANGE_EVT:
      SensorTag_processCharValueChangeEvt(pMsg->serviceID, pMsg->paramID);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*******************************************************************************
 * @fn      SensorTag_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SensorTag_stateChangeCB(gaprole_States_t newState)
{
  SensorTag_enqueueMsg(ST_STATE_CHANGE_EVT, newState, NULL);
}

/*******************************************************************************
 * @fn      SensorTag_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SensorTag_processStateChangeEvt(gaprole_States_t newState)
{
  switch (newState)
  {
  case GAPROLE_STARTED:
    {
      uint8_t ownAddress[B_ADDR_LEN];
      uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

      SensorTagIO_blinkLed(IOID_GREEN_LED, 5);

      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

      // use 6 bytes of device address for 8 bytes of system ID value
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];

      // set middle bytes to zero
      systemId[4] = 0x00;
      systemId[3] = 0x00;

      // shift three bytes up
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];

      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
    }
    break;

  case GAPROLE_ADVERTISING:
    // Start the clock
    if (!Util_isActive(&periodicClock))
    {
      Util_startClock(&periodicClock);
    }

    // Make sure key presses are not stuck
    SensorTag_updateAdvertisingData(0);
    break;

  case GAPROLE_CONNECTED:
    {
      // Start the clock
      if (!Util_isActive(&periodicClock))
      {
        Util_startClock(&periodicClock);
      }

      // Turn of LEDs and buzzer
      PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_OFF);
      PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);

      SensorTagConnectionControl_update();
    }
    break;

  case GAPROLE_CONNECTED_ADV:
    break;

  case GAPROLE_WAITING:
  case GAPROLE_WAITING_AFTER_TIMEOUT:
    SensorTag_resetAllModules();
    break;

  case GAPROLE_ERROR:
    SensorTag_resetAllModules();
    PIN_setOutputValue(hGpioPin,Board_LED1, Board_LED_ON);
    break;

  default:
    break;
  }

  gapProfileState = newState;
  SensorTagDisplay_showStatus();
}

/*******************************************************************************
 * @fn      SensorTag_charValueChangeCB
 *
 * @brief   Callback from Sensor Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
void SensorTag_charValueChangeCB(uint8_t serviceID, uint8_t paramID)
{
  SensorTag_enqueueMsg(ST_CHAR_CHANGE_EVT, serviceID, paramID);
}

/*******************************************************************************
 * @fn      SensorTag_processCharValueChangeEvt
 *
 * @brief   Process pending Profile characteristic value change
 *          events. The events are generated by the network task (BLE)
 *
 * @param   serviceID - ID of the affected service
 * @param   paramID - ID of the affected parameter
 *
 * @return  none
 */
static void SensorTag_processCharValueChangeEvt(uint8_t serviceID,
                                                uint8_t paramID)
{
  switch (serviceID)
  {
  case SERVICE_ID_IO:
    SensorTagIO_processCharChangeEvt(paramID);
    break;

  case SERVICE_ID_BATT:
    SensorTagBatt_processCharChangeEvt(paramID);
    break;

  case SERVICE_ID_REGISTER:
    SensorTagRegister_processCharChangeEvt(paramID);
    break;

  case SERVICE_ID_CC:
    SensorTagConnControl_processCharChangeEvt(paramID);
    break;

  case SERVICE_ID_DISPLAY:
    SensorTagDisplay_processCharChangeEvt(paramID);
    break;

  default:
    break;
  }
}

/*******************************************************************************
 * @fn      SensorTag_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SensorTag_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      SensorTag_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    default:
      // do nothing
      break;
  }
}

/*******************************************************************************
 * @fn      SensorTag_processGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void SensorTag_processGATTMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*******************************************************************************
 * @fn      SensorTag_performPeriodicTask
 *
 * @brief   Perform a periodic application task.
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTag_performPeriodicTask(void)
{
  SensorTagRegister_update();
}

/*******************************************************************************
 * @fn      SensorTag_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void SensorTag_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*******************************************************************************
 * @fn      SensorTag_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   serviceID - service identifier
 * @param   paramID - parameter identifier
 *
 * @return  none
 */
static void SensorTag_enqueueMsg(uint8_t event, uint8_t serviceID, uint8_t paramID)
{
  stEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(stEvt_t))))
  {
    pMsg->event = event;
    pMsg->serviceID = serviceID;
    pMsg->paramID = paramID;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8_t*)pMsg);
  }
}


/*********************************************************************
 * @fn      SensorTag_resetAllModules
 *
 * @brief   Reset all modules, typically when a connection is terminated.
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTag_resetAllModules(void)
{
  SensorTagIO_reset();
  SensorTagRegister_reset();
}

/*!*****************************************************************************
 *  @fn         SensorTag_callback
 *
 *  Interrupt service routine for buttons
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void SensorTag_callback(PIN_Handle handle, PIN_Id pinId)
{
  switch (pinId)
  {
  case Board_BTN1:
    SensorTagKeys_processKeyLeft();
    break;

  case Board_BTN2:
    SensorTagKeys_processKeyRight();
    break;

  default:
    /* Do nothing */
    break;
  }
}

/*******************************************************************************
 * @fn      SensorTag_updateAdvertisingData
 *
 * @brief   Update the advertising data with the latest key press status
 *
 * @return  none
 */
void SensorTag_updateAdvertisingData(uint8_t keyStatus)
{
  // Record key state in advertising data
  advertData[KEY_STATE_OFFSET] = keyStatus;
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
}

/*******************************************************************************
*******************************************************************************/
