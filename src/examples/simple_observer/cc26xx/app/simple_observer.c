/******************************************************************************

 @file  simple_observer.c

 @brief This file contains the Simple BLE Observer sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

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

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"

#include "observer.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "simple_observer.h"
#include "tagmgr.h"
#include <inc/hw_types.h>
#include <inc/hw_fcfg1.h>
#include "mems.h"
#include "userprocessmgr.h"
#include "sx1278_lora.h"
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION_10ms            10
#define DEFAULT_SCAN_DURATION_100ms           100


// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// Task configuration
#define SBO_TASK_PRIORITY                     1

#ifndef SBO_TASK_STACK_SIZE
#define SBO_TASK_STACK_SIZE                   1024
#endif

// Internal Events for RTOS application
#define SBO_KEY_CHANGE_EVT                    0x0001
#define SBO_STATE_CHANGE_EVT                  0x0002
#define SBO_MEMS_ACTIVE_EVT                   0x0004
#define SBO_LORA_STATUSPIN_EVT                0x0008
#define SBP_PERIODIC_EVT                      0x0010

//User Send Interval Time
#define DEFAULT_USER_TX_INTERVAL_TIME         5
#define DEFAULT_USER_MEMS_ACTIVE_TIME         3 
#define DEFAULT_USER_MEMS_NOACTIVE_TIME       13  
// 1000 ms
#define RCOSC_CALIBRATION_PERIOD              1000
#define RCOSC_CALIBRATION_PERIOD_3s           3000
#define RCOSC_CALIBRATION_PERIOD_15s          15000
#define DEFAULT_RFTRANSMIT_LEN                45
#define DEFAULT_RFRXTIMOUT_TIME                2
#define DEFAULT_RFTXTIMOUT_TIME                2
#define DEFAULT_MAX_RFSENDTAG_NUM              4
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sboEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

static userTxStruct userTxInf;
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

// Clock object used to signal timeout
static Clock_Struct keyChangeClock;

// Clock instances for internal periodic events.
static Clock_Struct userProcessClock;
// Power Notify Object for wake-up callbacks
Power_NotifyObj injectCalibrationPowerNotifyObj;

static uint8_t isEnabled = FALSE;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

static uint8 rfstatus;
volatile tUserProcessStates userProcess_State;
static tUserProcessMode userProcessMode;
volatile tUserProcessMgr userProcessMgr;
tUserNvramInf   userNvramInf;
// Task configuration
Task_Struct sboTask;
Char sboTaskStack[SBO_TASK_STACK_SIZE];

//Watchdog_Params params;
//Watchdog_Handle watchdog;

// events flag for internal application events.
static uint16_t events;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];
static tagInfStruct devInfList[DEFAULT_MAX_SCAN_RES];
static observerInfStruct tagInf_t;
static tagInfStruct userTxList[DEFAULT_MAX_SCAN_RES];
static uint8_t rfRxTxBuf[DEFAULT_RFTRANSMIT_LEN];
const uint8_t weiXinUuid[6]={0xFD,0xA5,0x06,0x93,0xA4,0xE2};
const uint8_t lrtag_preFix[2] = {0xFE, 0xBE};
const uint8_t lrtag_sufFix[2] = {0x0D, 0x0A};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLEObserver_init(void);
static void SimpleBLEObserver_taskFxn(UArg a0, UArg a1);
static void getMacAddress(uint8_t *mac_address);
static void SimpleBLEObserver_addDeviceInfo_Ex(uint8 *pAddr, uint8 *pData, uint8 datalen, uint8 rssi);

static void SimpleBLEObserver_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLEObserver_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLEObserver_processAppMsg(sboEvt_t *pMsg);
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent);

static uint8_t SimpleBLEObserver_eventCB(gapObserverRoleEvent_t *pEvent);

static uint8_t SimpleBLEObserver_enqueueMsg(uint8_t event, uint8_t status,
                                            uint8_t *pData);

void SimpleBLEObserver_initKeys(void);

void SimpleBLEObserver_keyChangeHandler(uint8 keys);

void SimpleBLEObserver_memsActiveHandler(uint8 pins);
void SimpleBLEObserver_loraStatusHandler(uint8 pins);

static void SimpleBLEObserver_userclockHandler(UArg arg);

static uint8_t rcosc_injectCalibrationPostNotify(uint8_t eventType,
                                                 uint32_t *eventArg,
                                                 uint32_t *clientArg);
static void SimpleBLEObserver_performPeriodicTask(void);
static void SimpleBLEObserver_sleepModelTask(void);
static bool UserProcess_LoraInf_Get(void);
static void UserProcess_LoraInf_Send(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB =
{
  SimpleBLEObserver_eventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      RCOSC_enableCalibration
 *
 * @brief   enable calibration.  calibration timer will start immediately.
 *
 * @param   none
 *
 * @return  none
 */
void RCOSC_enableCalibration(void)
{
  if (!isEnabled)
  {
    isEnabled = TRUE;

    // Set device's Sleep Clock Accuracy
    HCI_EXT_SetSCACmd(500);

    // Receive callback when device wakes up from Standby Mode.
    Power_registerNotify(&injectCalibrationPowerNotifyObj, PowerCC26XX_AWAKE_STANDBY,
                         (Power_NotifyFxn)rcosc_injectCalibrationPostNotify,
                         NULL);
  }
}

/*********************************************************************
 * @fn      rcosc_injectCalibrationPostNotify
 *
 * @brief   Callback for Power module state change events.
 *
 * @param   eventType - The state change.
 * @param   clientArg - Not used.
 *
 * @return  Power_NOTIFYDONE
 */
static uint8_t rcosc_injectCalibrationPostNotify(uint8_t eventType,
                                                 uint32_t *eventArg,
                                                 uint32_t *clientArg)
{
  // If clock is active at time of wake up,
//  if (Util_isActive(&userProcessClock))
//  {
//    // Stop injection of calibration - the wakeup has automatically done this.
//    Util_stopClock(&userProcessClock);
//  }

  // Restart the clock in case delta between now and next wake up is greater
  // than one second.
  //Util_startClock(&userProcessClock);
  
  return Power_NOTIFYDONE;
}

/*********************************************************************
 * @fn      SimpleBLEObserver_createTask
 *
 * @brief   Task creation function for the Simple BLE Observer.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sboTaskStack;
  taskParams.stackSize = SBO_TASK_STACK_SIZE;
  taskParams.priority = SBO_TASK_PRIORITY;

  Task_construct(&sboTask, SimpleBLEObserver_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEObserver_init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_init(void)
{  
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  
  Board_initKeys(SimpleBLEObserver_keyChangeHandler);

  //获取当前设备的Mac地址，作为设备唯一识别ID
  getMacAddress(&userTxInf.devId[0]);
  //config user inf
  userTxInf.status       = 0;
  userTxInf.txTagNum     = 0;
  userTxInf.tagInfBuf_t  = (tagInfStruct *)userTxList;
  userTxInf.interval[0]  = DEFAULT_USER_TX_INTERVAL_TIME >> 8;
  userTxInf.interval[1]  = DEFAULT_USER_TX_INTERVAL_TIME & (0xFF);
  tagInf_t.index         = 0;
  tagInf_t.tagNum        = 0;
  tagInf_t.tagInfBuf_t   = (tagInfStruct *)devInfList;
  userProcess_State      = USER_PROCESS_IDLE;
  userProcessMode        = USER_PROCESS_IDLE_MODE;

  userProcessMgr.abNormalScanTime = 0;
  userProcessMgr.memsActiveFlg = 0;
  userProcessMgr.clockCounter  = 0;
  userProcessMode = USER_PROCESS_IDLE_MODE;
  userProcessMgr.rftxtimeout = 0;
  userProcessMgr.rfrxtimeout = 0;
  
  userNvramInf.sleeptime = RCOSC_CALIBRATION_PERIOD/1000;
  userNvramInf.txinterval= DEFAULT_USER_TX_INTERVAL_TIME;
  
  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter(GAPOBSERVERROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                 &scanRes );
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION_100ms);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION_100ms);
  
  // add a white list entry
  if(DEFAULT_DISCOVERY_WHITE_LIST)
  {
 	//HCI_LE_AddWhiteListCmd(HCI_PUBLIC_DEVICE_ADDRESS,&bdAdder);
  }
  
  // Start the Device
  VOID GAPObserverRole_StartDevice((gapObserverRoleCB_t *)&simpleBLERoleCB);

  //Display_print0(dispHandle, 0, 0, "BLE Observer");
}

/*********************************************************************
 * @fn      SimpleBLEObserver_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Observer.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEObserver_taskFxn(UArg a0, UArg a1)
{
  uint8 res; 
  // Initialize application
  SimpleBLEObserver_init();

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
          SimpleBLEObserver_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sboEvt_t *pMsg = (sboEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLEObserver_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }
		
	switch(sx1278Lora_Process())
	{
		case RFLR_STATE_TX_DONE:
			sx1278Lora_EntryRx();
			sx1278Lora_SetRFStatus(RFLR_STATE_RX_RUNNING);						
			userProcessMgr.rftxtimeout = 0;
			break;
							
		case RFLR_STATE_RX_DONE:
			if(UserProcess_LoraInf_Get())
			{
			    sx1278Lora_SetOpMode( RFLR_OPMODE_STANDBY );
				Task_sleep(5*1000/Clock_tickPeriod);
				sx1278Lora_SetOpMode(RFLR_OPMODE_SLEEP);	
				sx1278Lora_SetRFStatus(RFLR_STATE_SLEEP);	
				sx1278_LowPowerMgr();
				userProcessMgr.rfrxtimeout = 0;
				userProcessMgr.clockCounter = 0;
				Util_restartClock(&userProcessClock,RCOSC_CALIBRATION_PERIOD);
			}
			break;
						  
		default:
			break;
	}

	switch(userProcessMode)
	{		
		case USER_PROCESS_NORAMAL_MODE:
		 	{
			    sx1278Init();
				sx1278Lora_SetOpMode(RFLR_OPMODE_SLEEP);
				sx1278_LowPowerMgr();
				sx1278Lora_SetRFStatus(RFLR_STATE_SLEEP);

			    res = MemsOpen();
			  	if(!res)
				{
				  /* reset or enter into  ABNORMAL_MODE*/
				  HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
				}
				
				MemsLowPwMgr();
				
				res = Mems_ActivePin_Enable(SimpleBLEObserver_memsActiveHandler);
				if(!res)
				{
				  HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
				}  
				
				// Setup GAP
  				GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION_100ms);
  				GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION_100ms);
				
#ifdef USE_RCOSC
	RCOSC_enableCalibration();
#endif // USE_RCOSC  	
	
				Util_constructClock(&userProcessClock, SimpleBLEObserver_userclockHandler,
                                            RCOSC_CALIBRATION_PERIOD, 0, false, SBP_PERIODIC_EVT);
				
				userProcessMode = USER_PROCESS_ACTIVE_MODE;
				
				Util_startClock(&userProcessClock);
		 	}	  
	  		break;
			
	    case USER_PROCESS_ABNORMAL_MODE:
		  	{
		  		;
		  	}
			break;
			
	    case USER_PROCESS_ACTIVE_MODE:
		  	{	
				rfstatus = sx1278Lora_GetRFStatus();
				
				if (events & SBP_PERIODIC_EVT)
				{
					tagInf_t.index ++;
	  
					events &= ~SBP_PERIODIC_EVT;
	  
					Util_startClock(&userProcessClock);

					// Perform periodic application task
					SimpleBLEObserver_performPeriodicTask();
				}
			  			  			  
				if((RFLR_STATE_TX_RUNNING != rfstatus) && (RFLR_STATE_RX_RUNNING != rfstatus) && (RFLR_STATE_RX_DONE != rfstatus))
				{
			  		if(userProcessMgr.memsNoActiveCounter > DEFAULT_USER_MEMS_NOACTIVE_TIME)
					{
						if( RFLR_STATE_SLEEP != rfstatus )
						{
							sx1278_OutputLowPw();
							sx1278Lora_SetOpMode(RFLR_OPMODE_SLEEP);
							sx1278_LowPowerMgr();
						}
						
						userProcessMgr.memsActiveFlg = FALSE;
						userProcessMode = USER_PROCESS_SLEEP_MODE;
						userProcessMgr.memsActiveCounter = 0;
						tagInf_t.index = 0;
						userProcessMgr.clockCounter = 0;
						userTxInf.txTagNum = 0;
						userProcessMgr.memsNoActiveCounter = 0;
						Util_stopClock(&userProcessClock);						
						Util_restartClock(&userProcessClock,RCOSC_CALIBRATION_PERIOD_3s);
					}
				}																									
		  	}
			break;
			
	    case USER_PROCESS_SLEEP_MODE:
		  	{
			  	if (events & SBP_PERIODIC_EVT)
   			 	{	  
      				events &= ~SBP_PERIODIC_EVT;
	  
      				Util_startClock(&userProcessClock);

      				// Perform periodic application task
	  				SimpleBLEObserver_sleepModelTask();
    			}
			  
				if(userProcessMgr.wakeUpSourse & 0x04)
				{
				  	if(userProcessMgr.memsActiveFlg == TRUE)
					{
						userProcessMgr.memsActiveFlg = FALSE;
						res = Mems_ActivePin_Enable(SimpleBLEObserver_memsActiveHandler);
						if(!res)
						{
							HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);							
						}						
					}
					userProcessMgr.wakeUpSourse = 0; 
					Util_stopClock(&userProcessClock);	
					Util_restartClock(&userProcessClock,RCOSC_CALIBRATION_PERIOD);
					userProcessMode = USER_PROCESS_ACTIVE_MODE;				  				
				}		  		
			}
			break;	
			
		default:
	 		break;
	}  
  }
  
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEObserver_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLEObserver_processRoleEvent((gapObserverRoleEvent_t *)pMsg);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processAppMsg(sboEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBO_STATE_CHANGE_EVT:
      SimpleBLEObserver_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBO_KEY_CHANGE_EVT:
      SimpleBLEObserver_handleKeys(0, pMsg->hdr.state);
	  userProcessMgr.wakeUpSourse |= 0x04;
      break;
	  
  	case SBO_MEMS_ACTIVE_EVT:
	  userProcessMgr.memsActiveFlg = TRUE;
	  userProcessMgr.wakeUpSourse |= 0x02;
	  Mems_ActivePin_Disable();
	  break;
	  
  	case  SBO_LORA_STATUSPIN_EVT:
      break; 
	
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleBLEObserver_handleKeys(uint8 shift, uint8 keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_UP)
  {
    return;
  }

  if (keys & KEY_LEFT)
  {
    return;
  }
  
  if(keys & KEY_SOS)
  {
  	userTxInf.status |= 1<<3;
	userProcessMgr.clockCounter = 4;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processRoleEvent
 *
 * @brief   Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent)
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
		if(USER_PROCESS_IDLE == userProcess_State)
		{
			//设备上电初始化完成即开启扫描,扫描时间10ms
			GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      		DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      		DEFAULT_DISCOVERY_WHITE_LIST );
		}
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
	  {
		
		if(pEvent->deviceInfo.eventType != GAP_ADRPT_SCAN_RSP)
		{
			SimpleBLEObserver_addDeviceInfo_Ex(pEvent->deviceInfo.addr,
 										    pEvent->deviceInfo.pEvtData,
 										   	pEvent->deviceInfo.dataLen,
 										   	pEvent->deviceInfo.rssi);
		}
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
      	// discovery complete
		switch(userProcess_State)
		{
			case USER_PROCESS_IDLE :
			{
				if(tagInf_t.tagNum < 1)
				{
				    GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      				DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      				DEFAULT_DISCOVERY_WHITE_LIST );
								
					userProcessMgr.abNormalScanTime ++;
					userProcess_State = USER_PROCESS_READY;
				}
				else
				{
					tagInf_t.tagNum = 0;
					userProcess_State = USER_PROCESS_RUNNING;
					userProcessMode   = USER_PROCESS_NORAMAL_MODE;
				}		 
			}
			  break; 
			  
			case USER_PROCESS_RUNNING :
			{	  
			  	if(tagInf_t.tagNum  == 0)
				{
					HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);		
				}
				else
				{				
					//涮选出本次搜索rssi最大设备并暂存到用户区
 					if(userTxInf.txTagNum >= DEFAULT_MAX_RFSENDTAG_NUM)
 		    			return;  

 					for(; tagInf_t.tagNum >0; tagInf_t.tagNum --)
 					{
 						if(tagInf_t.tagNum  -1 == 0)
 						{
 							memcpy(&userTxInf.tagInfBuf_t[userTxInf.txTagNum], &tagInf_t.tagInfBuf_t[tagInf_t.tagNum -1], sizeof(tagInfStruct));	
 							continue;
 						}
 						else
 						{
 			  				if(tagInf_t.tagInfBuf_t[tagInf_t.tagNum -1].rssi < tagInf_t.tagInfBuf_t[tagInf_t.tagNum -2].rssi)
 								memcpy(&tagInf_t.tagInfBuf_t[tagInf_t.tagNum -2], &tagInf_t.tagInfBuf_t[tagInf_t.tagNum -1], sizeof(tagInfStruct));							
 						}		  
 					}		
					userTxInf.txTagNum ++;
				}
			}
			break;
			  
			case USER_PROCESS_READY: 
			{
				if(tagInf_t.tagNum < 1 )
				{
					if(userProcessMgr.abNormalScanTime > 1)
					{
						userProcessMode = USER_PROCESS_ABNORMAL_MODE;
						userProcessMgr.abNormalScanTime = 0;
					}
					else if(userProcessMgr.abNormalScanTime == 1)
					{
						Task_sleep(10*1000/Clock_tickPeriod);
				    	userProcessMgr.abNormalScanTime ++;
						//Scan Again
						GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      				DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      			 	DEFAULT_DISCOVERY_WHITE_LIST );					
					}
				}
				else
				{
					tagInf_t.tagNum = 0;
					userProcess_State = USER_PROCESS_RUNNING;
					userProcessMode   = USER_PROCESS_NORAMAL_MODE;		
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
 * @fn      SimpleBLEObserver_eventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLEObserver_eventCB(gapObserverRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLEObserver_enqueueMsg(SBO_STATE_CHANGE_EVT,
                                   SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLEObserver_addDeviceInfo_Ex
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLEObserver_addDeviceInfo_Ex(uint8 *pAddr, uint8 *pData, uint8 datalen, uint8 rssi)
{
	uint8 i;
 	uint8 majoroffset;
 	uint8 uuidoffset;
	uint8 tagNum;
 	uint8 *ptr;
 	
 	if((pData == NULL) && (pAddr == NULL))
 		return;
 	
 	if(BEELINKER_ADVDATA_LEN != datalen)
 		return;

 	ptr = pData; 
 	uuidoffset = BEELINKER_ADVUUID_OFFSET;
 	if(memcmp(&ptr[uuidoffset], weiXinUuid, sizeof(weiXinUuid)) != 0)
 	  	return;
 
	tagNum = tagInf_t.tagNum ;
   	// If result count not at max
   	if ( tagNum < DEFAULT_MAX_SCAN_RES )
   	{
		// Check if device is already in scan results
		for ( i = 0; i < tagNum; i++ )
		{
			if (memcmp(pAddr, devList[i].addr, B_ADDR_LEN) == 0)
			{
				return;
			}
		}
		// Add addr to scan result list
		memcpy(devList[tagNum].addr, pAddr, B_ADDR_LEN );

		majoroffset = BEELINKER_ADVMAJOR_OFFSET; 
		memcpy((void *)(&tagInf_t.tagInfBuf_t[tagNum].major[0]), (void *)&ptr[majoroffset], sizeof(uint32));

		tagInf_t.tagInfBuf_t[tagNum].tagIndex = tagInf_t.index;
		
		tagInf_t.tagInfBuf_t[tagNum].rssi     = 0xFF - rssi;
 		
		// Increment scan result count
		tagInf_t.tagNum ++;
	}
}

/*********************************************************************
 * @fn      SimpleBLEObserver_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys pressed
 *
 * @return  none
 */
void SimpleBLEObserver_keyChangeHandler(uint8 keys)
{
  SimpleBLEObserver_enqueueMsg(SBO_KEY_CHANGE_EVT, keys, NULL);
}

void SimpleBLEObserver_memsActiveHandler(uint8 pins)
{
  SimpleBLEObserver_enqueueMsg(SBO_MEMS_ACTIVE_EVT, pins, NULL);
}

void SimpleBLEObserver_loraStatusHandler(uint8 pins)
{
  SimpleBLEObserver_enqueueMsg(SBO_LORA_STATUSPIN_EVT, pins, NULL);
  Semaphore_post(sem);
}
/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEObserver_userclockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
  
  PowerCC26XX_injectCalibration(); 
}
/*********************************************************************
 * @fn      SimpleBLEObserver_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLEObserver_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sboEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(sboEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

static void SimpleBLEObserver_performPeriodicTask(void)
{
   	uint8 res = 0; 
	
	userProcessMgr.clockCounter ++;
	
	if(userProcessMgr.clockCounter == userNvramInf.txinterval)
	{
		sx1278_StatusPin_Disable();
							
		if( RFLR_STATE_SLEEP == rfstatus)
		{
			sx1278_OutputLowPw();
			UserProcess_LoraInf_Send();
		}
		userTxInf.status &= ~(1<<3); 
	}
	else if(userProcessMgr.clockCounter < userNvramInf.txinterval)
	{
		GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST );	
	}

	if(userProcessMgr.clockCounter >= userNvramInf.txinterval)
	{
		if(userProcessMgr.memsActiveFlg == TRUE)
		{
			userProcessMgr.memsActiveFlg = FALSE;
			res = Mems_ActivePin_Enable(SimpleBLEObserver_memsActiveHandler);
			if(!res)
			{
				HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
			}						
		}	
	}	
	
	if(RFLR_STATE_RX_RUNNING == rfstatus)
	{
		userProcessMgr.rfrxtimeout ++;
		/* 接收超时，强制进入Sleep模式 */
		if( userProcessMgr.rfrxtimeout >= DEFAULT_RFRXTIMOUT_TIME)
		{
			sx1278Lora_SetOpMode( RFLR_OPMODE_STANDBY );
			Task_sleep(5*1000/Clock_tickPeriod);
			sx1278Lora_SetOpMode(RFLR_OPMODE_SLEEP);
			sx1278Lora_SetRFStatus(RFLR_STATE_SLEEP);	
			sx1278_LowPowerMgr();
			serProcessMgr.clockCounter = 0;
			userProcessMgr.rfrxtimeout = 0;
		}
	}
	else if(RFLR_STATE_TX_RUNNING == rfstatus)
	{	 
		userProcessMgr.rftxtimeout ++;
		if( userProcessMgr.rftxtimeout > DEFAULT_RFTXTIMOUT_TIME)
		{
			HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
		}
	}
	
	if(userProcessMgr.memsActiveFlg == FALSE)
		userProcessMgr.memsNoActiveCounter ++;
	else 
		userProcessMgr.memsNoActiveCounter = 0;
}

static void SimpleBLEObserver_sleepModelTask(void)
{
  	uint8 res = 0; 
	
	if(userProcessMgr.memsActiveFlg == TRUE)
	{
		userProcessMgr.memsActiveCounter ++;
		userProcessMgr.memsActiveFlg = FALSE;
		res = Mems_ActivePin_Enable(SimpleBLEObserver_memsActiveHandler);
		if(!res)
		{
			HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
		}
						
		if(userProcessMgr.memsActiveCounter >= DEFAULT_USER_MEMS_ACTIVE_TIME)
		{
			userProcessMgr.memsActiveFlg = FALSE;
			userProcessMgr.memsActiveCounter = 0;			
			userProcessMode = USER_PROCESS_ACTIVE_MODE;
			Util_stopClock(&userProcessClock);	
			Util_restartClock(&userProcessClock,RCOSC_CALIBRATION_PERIOD);	
		}				
	}
//	else
//		userProcessMgr.memsActiveCounter = 0;
}

#define LORATAT_MACADDRE_LEN    6

static bool UserProcess_LoraInf_Get(void)
{
	uint8_t size;
	uint8_t len;
	uint8_t crc;
	uint8_t *ptr;
	uint8_t tmpCnt;
	uint8_t cmd;
	uint8_t i = 0;
	
	ptr = sx1278Lora_GetRxData(&size);
	
	if(ptr == NULL)
		return FALSE;
	
	while(i < size)
	{
		if(!memcmp(&ptr[i], lrtag_preFix, sizeof(lrtag_preFix)))
			break;
		i ++;		
	}
	
	tmpCnt = i + sizeof(lrtag_preFix);
	
	cmd = ptr[tmpCnt];
	
	len = ptr[tmpCnt + 1];
	
	if ((size - i) < sizeof(lrtag_preFix) + sizeof(cmd) + sizeof(len) + len + sizeof(crc) + sizeof(lrtag_sufFix)) // PreFix + LEN + CMD + CRC + SufFix    
		return FALSE;
	
	for(i=0; i<len + sizeof(uint16_t); i++)
		crc += ptr[tmpCnt + i];
		  
	 if(crc != ptr[tmpCnt + sizeof(uint16_t) + len])
		return FALSE;

	memset(rfRxTxBuf, 0, DEFAULT_RFTRANSMIT_LEN);
	memcpy(rfRxTxBuf, &ptr[tmpCnt+sizeof(uint16_t)], len);
	
	switch(cmd)
	{
		case TYPE_LORATAUPRESP:
			if(!memcmp(rfRxTxBuf, &userTxInf.devId[0], LORATAT_MACADDRE_LEN))
			{
			  	Board_LedCtrl(Board_LED_ON);
				Task_sleep(50*1000/Clock_tickPeriod);
				Board_LedCtrl(Board_LED_OFF);
			}
			break;
			
		default:
			break;
	}
	return TRUE;
}

static void UserProcess_LoraInf_Send(void)
{
	uint8_t *ptr;
	uint8_t crc;
	uint8_t len;
	uint8_t offset;
	uint8_t i;
	
	ptr = rfRxTxBuf;
	crc = 0;
	offset = 0;
	
	memcpy(ptr, lrtag_preFix, sizeof(lrtag_preFix));
	ptr += sizeof(lrtag_preFix);
	
	*ptr = TYPE_LORATAGUP;
	crc += *ptr++;
	
	*ptr = userTxInf.txTagNum * 7 + 10;//7 = sizeof(tagInfStruct) 
	crc += *ptr++;
	
	memcpy(ptr, &userTxInf.devId[0], 10);
	for(i=0; i<10; i++)
		crc += *ptr++;
	
	if(userTxInf.txTagNum > 0)
	{
	   	for(i=0; i<userTxInf.txTagNum; i++)
		{
			userTxInf.tagInfBuf_t[i].tagIndex = ntohs(userTxInf.tagInfBuf_t[i].tagIndex); 
			memcpy(ptr + offset, &userTxInf.tagInfBuf_t[i].tagIndex, 7);
			offset += 7;	
		}		
		
		for(i=0; i<userTxInf.txTagNum * 7; i++)				    
			crc += *ptr++;	
				
		userTxInf.txTagNum = 0;
	}
	
	*ptr++ = crc;
	
	memcpy(ptr, lrtag_sufFix, sizeof(lrtag_sufFix));
	ptr += sizeof(lrtag_sufFix);
	
	len = ptr - rfRxTxBuf;
	if(len <= DEFAULT_RFTRANSMIT_LEN)
	{
		sx1278Lora_EntryTx();
		Task_sleep(5*1000/Clock_tickPeriod);
	  	sx1278Lora_SetRFStatus(RFLR_STATE_TX_RUNNING);
		sx1278_StatusPin_Enable(SimpleBLEObserver_loraStatusHandler);
		sx1278Lora_RFSendBuf(rfRxTxBuf, len);
	}	  	 	
}

/*********************************************************************
 * @fn      getMacAddress
 *
 * @brief  获取本设备的MAC地址
 *
 * @param   mac_address - MAC地址存储首地址
 *
 * @return  none
 */
static void getMacAddress(uint8_t *mac_address)  
{      
	uint32_t mac0 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);    
	uint32_t mac1 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);    
   
	*mac_address++ = HI_UINT16(mac1);  
	*mac_address++ = LO_UINT16(mac1);  
	*mac_address++ = BREAK_UINT32(mac0, 3);  
	*mac_address++ = BREAK_UINT32(mac0, 2);  
	*mac_address++ = BREAK_UINT32(mac0, 1);  
	*mac_address++ = BREAK_UINT32(mac0, 0);  
}
/*********************************************************************
*********************************************************************/
