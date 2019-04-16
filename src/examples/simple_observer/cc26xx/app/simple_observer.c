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
#include <stdlib.h>
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
#include <ti/drivers/Watchdog.h>
#include <inc/hw_wdt.h>
#include "snv.h"
#include "auxadc.h"
#include "crc.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  30

// Scan duration in ms
#define DEFAULT_SCAN_DURATION_500ms           500

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
#define SBP_LORAUP_EVT                        0x0020
#define SBP_LORAUPDELAY_EVT                   0x0040

//User Send Interval Time
#define DEFAULT_USER_TX_INTERVAL_TIME         4
#define DEFAULT_USER_MEMS_ACTIVE_TIME         3 
#define DEFAULT_USER_MEMS_NOACTIVE_TIME       2  
// 1000 ms
#define RCOSC_CALIBRATION_PERIOD_400ms        400
#define RCOSC_CALIBRATION_PERIOD              1000
#define RCOSC_CALIBRATION_PERIOD_3s           3000
#define RCOSC_CALIBRATION_PERIOD_30s          30000
#define RCOSC_CALIBRATION_PERIOD_160ms        160  
#define DEFAULT_RFTRANSMIT_LEN                55
#define DEFAULT_RFRXTIMOUT_TIME                1
#define DEFAULT_RFTXTIMOUT_TIME                1
#define DEFAULT_SOSTICK_NUM                    12 
#define DEFAULT_SINGLESTORMAX_NUM              4   
#define DEFAULT_UPBLEINFMAX_NUM                16   
#define DEFAULT_PERDRVFAILMAX_TIME             4 
#define DEFAULT_CHANNELCHECK_TIME              4    
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
static Clock_Struct userProcessClock;
static Clock_Struct loraUpClock;
static Clock_Struct loraUpDelayClock;

// Power Notify Object for wake-up callbacks
Power_NotifyObj injectCalibrationPowerNotifyObj;
static uint8_t ledStatus;
static uint8_t isEnabled = FALSE;
static uint8_t rcoscTimeTick;
static uint8_t scanTagNum; 
static uint8_t loraUpPktLen; 
static uint16_t bleScanTimeTick; 
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

static UserProcessMode userProcessMode;
volatile UserProcessMgr_t userProcessMgr;
UserTimeSeries_t UserTimeSeries;
// Task configuration
Task_Struct sboTask;
Char sboTaskStack[SBO_TASK_STACK_SIZE];

//Watchdog_Params params;
Watchdog_Handle watchdog;

// events flag for internal application events.
static uint16_t events;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];
static tagInfStruct devInfList[DEFAULT_MAX_SCAN_RES];
static observerInfStruct tagInf_t;
static tagInfStruct userTxList[16];
static uint8_t rfRxTxBuf[DEFAULT_RFTRANSMIT_LEN];
const uint8_t weiXinUuid[6]={0xFD,0xA5,0x06,0x93,0xA4,0xE2};
LoRaSettings_t *userLoraPara = NULL;
extern SX1276_t  SX1278;
snv_driverfailure_t snv_DriverFailure;
const uint32_t default_loraChannel[2]={499300000,496330000};
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


static void SimpleBLEObserver_userClockHandler(UArg arg);

static uint8_t rcosc_injectCalibrationPostNotify(uint8_t eventType,
                                                 uint32_t *eventArg,
                                                 uint32_t *clientArg);
static void SimpleBLEObserver_performPeriodicTask(void);
static void SimpleBLEObserver_sleepModelTask(void);

static void UserProcess_GetLoraUp_Pkt(void);
static bool UserProcess_LoraInf_Get(void);
static void UserProcess_LoraInf_Send(uint8_t *buf, uint8_t len);
static void SimpleBLEObserver_loraStatusTask(uint8_t rxtimeout, uint8_t txtimeout);
void wdtInitFxn(void);
void TagPara_Get(void);
void Voltage_Check(void);
void SleepToActive_Ready(void);
void ActiveToSleep_Ready(void);
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
	uint16_t crc;  
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  
  Board_initKeys(SimpleBLEObserver_keyChangeHandler);
  
#ifdef IWDG_ENABLE
  wdtInitFxn();
#endif
  
  //获取当前设备的Mac地址，作为设备唯一识别ID
  getMacAddress(rfRxTxBuf);
  crc = crc16(0, rfRxTxBuf, B_ADDR_LEN);
  userTxInf.devId[0] = ( crc >> 8 ) & 0xFF;
  userTxInf.devId[1] = crc & 0xFF;
  
  Nvram_Init();
  
  //config user inf
  userTxInf.tagInfBuf_t  = (tagInfStruct *)userTxList;
  tagInf_t.tagNum        = 0;
  tagInf_t.tagInfBuf_t   = (tagInfStruct *)devInfList;
  userProcessMode        = USER_PROCESS_IDLE_MODE;

  userProcessMgr.memsActiveFlg = 0;
  userProcessMgr.wakeUpSourse = WAKEUP_SOURSE_RTC;
  userProcessMode = USER_PROCESS_IDLE_MODE; 
  userProcessMgr.rftxtimeout = 0;
  userProcessMgr.rfrxtimeout = 0;
  
  Voltage_Check();
  TagPara_Get();

  if( (UserTimeSeries.txinterval == 0) || (UserTimeSeries.txinterval == 0xFFFF) )
    UserTimeSeries.txinterval = DEFAULT_USER_TX_INTERVAL_TIME; //4s
  
  if( (UserTimeSeries.sleepDelay == 0) || (UserTimeSeries.sleepDelay == 0xFF))
    UserTimeSeries.sleepDelay = 4; //4s
  
  UserTimeSeries.memsNoActiveTime = UserTimeSeries.txinterval * DEFAULT_USER_MEMS_NOACTIVE_TIME + UserTimeSeries.sleepDelay;
  //BLE的扫描次数不得大于发送间隔
  if( UserTimeSeries.txinterval < UserTimeSeries.scanTimes)
    UserTimeSeries.scanTimes = UserTimeSeries.txinterval;
	
  rcoscTimeTick = 0;
  scanTagNum = 0;
  bleScanTimeTick = 0;

  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter(GAPOBSERVERROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                 &scanRes );
  }
	
  // Setup GAP
  if( UserTimeSeries.timeForscanning == 0)
    UserTimeSeries.timeForscanning = DEFAULT_SCAN_DURATION_500ms;
  
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION_500ms);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION_500ms);
  
  // add a white list entry
  if(DEFAULT_DISCOVERY_WHITE_LIST)
  {
 	//HCI_LE_AddWhiteListCmd(HCI_PUBLIC_DEVICE_ADDRESS,&bdAdder);
  }
  
  // Start the Device
  VOID GAPObserverRole_StartDevice((gapObserverRoleCB_t *)&simpleBLERoleCB);
  
 #ifdef USE_RCOSC
	RCOSC_enableCalibration();
#endif // USE_RCOSC  	
	
  /* Base timer */
  if( USER_PROCESS_LOWVOLTAGE_MODE == userProcessMode)
  {
      Util_constructClock(&userProcessClock, SimpleBLEObserver_userClockHandler,
                          RCOSC_CALIBRATION_PERIOD_400ms, 0, false, SBP_PERIODIC_EVT);  
  }
  else
  {
      Util_constructClock(&userProcessClock, SimpleBLEObserver_userClockHandler,
                          RCOSC_CALIBRATION_PERIOD, 0, false, SBP_PERIODIC_EVT); 
  }
  
  Util_startClock(&userProcessClock);
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
	
	if (events & SBP_LORAUPDELAY_EVT)
	{	
		events &= ~SBP_LORAUPDELAY_EVT;
	  	
		UserProcess_LoraInf_Send(rfRxTxBuf, loraUpPktLen);
		
		if( USER_PROCESS_LOWVOLTAGE_MODE == userProcessMode)
		{
		 	Util_stopClock(&userProcessClock);	 
			Util_stopClock(&loraUpClock);	
			Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD_400ms);
		}
	}
	else if(events & SBP_LORAUP_EVT)
	{
		events &= ~SBP_LORAUP_EVT;
					
		UserProcess_GetLoraUp_Pkt();
		
		if( loraUpPktLen != 0 )
		{	
		    if( UserTimeSeries.channelCheckTime == 0 )
				Util_restartClock(&loraUpDelayClock, 10);
			else 
			{
			    Util_stopClock(&loraUpDelayClock);
			    UserTimeSeries.channelCheckTime = 0;
			}
		}
		
		Voltage_Check();	
		
		if( 3 == userTxInf.device_up_inf.bit_t.acflag )
			userTxInf.device_up_inf.bit_t.acflag = 0;
		else
			userTxInf.device_up_inf.bit_t.acflag ++;
										
		Util_startClock(&loraUpClock);	
					
		userTxInf.device_up_inf.bit_t.beaconNum_1 = 0;
		userTxInf.device_up_inf.bit_t.beaconNum_2 = 0;
		userTxInf.device_up_inf.bit_t.beaconNum_3 = 0;
		userTxInf.device_up_inf.bit_t.beaconNum_4 = 0;
		bleScanTimeTick = 0;
		rcoscTimeTick   = 0;
					
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
	
#ifdef IWDG_ENABLE 
	Watchdog_clear(watchdog);
#endif
	
	switch(userProcessMode)
	{		
		case USER_PROCESS_NORAMAL_MODE:
		 	{
				if( sx1278Init() )
				{
					sx1278_SetLoraPara(userLoraPara);
					sx1278_SetSleep();
					sx1278_LowPowerMgr();
				}
				else
				{
					snv_DriverFailure.lora ++;
					Ble_WriteNv_Inf(BLE_NVID_DRFLG_START, (uint8_t *)&snv_DriverFailure.mems);
					/* reset or enter into  ABNORMAL_MODE*/
					HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);					
				}

			    res = MemsOpen();
			  	if(!res)
				{
				    snv_DriverFailure.mems ++;
				    Ble_WriteNv_Inf(BLE_NVID_DRFLG_START, (uint8_t *)&snv_DriverFailure.mems);
				    /* reset or enter into  ABNORMAL_MODE*/
				    HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
				}
				
				MemsLowPwMgr();
				
				Mems_ActivePin_Enable(SimpleBLEObserver_memsActiveHandler);
	
				GAP_SetParamValue(TGAP_GEN_DISC_SCAN, UserTimeSeries.timeForscanning);
				GAP_SetParamValue(TGAP_LIM_DISC_SCAN, UserTimeSeries.timeForscanning);
				
				/* Lora Up Timer */
				Util_constructClock(&loraUpClock, SimpleBLEObserver_userClockHandler,
                                            UserTimeSeries.txinterval*RCOSC_CALIBRATION_PERIOD, 0, false, SBP_LORAUP_EVT);		
				
				/* Lora Up Delay Timer */
				Util_constructClock(&loraUpDelayClock, SimpleBLEObserver_userClockHandler,
                                            RCOSC_CALIBRATION_PERIOD, 0, false, SBP_LORAUPDELAY_EVT);	
				
				Util_stopClock(&userProcessClock);	
				Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD);
				/* Start Timer */
				Util_startClock(&loraUpClock);
				userProcessMode = USER_PROCESS_ACTIVE_MODE;
		 	}	  
	  		break;
			
	    case USER_PROCESS_ABNORMAL_MODE:
		  	{
		  		if (events & SBP_PERIODIC_EVT)
				{	
					events &= ~SBP_PERIODIC_EVT;
					GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                                    DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                                    DEFAULT_DISCOVERY_WHITE_LIST );	
					Util_startClock(&userProcessClock);
				}
		  	}
			break;
			
	    case USER_PROCESS_ACTIVE_MODE:
		  	{				
				if (events & SBP_PERIODIC_EVT)
				{	
					events &= ~SBP_PERIODIC_EVT;

					// Perform periodic application task
					SimpleBLEObserver_performPeriodicTask();
					
					Util_startClock(&userProcessClock);
				}
			  	
				if(userProcessMgr.sosstatustick == 0)
				{
					if((RF_IDLE == SX1278.State))
					{
			  			if(userProcessMgr.memsNoActiveCounter > UserTimeSeries.memsNoActiveTime + UserTimeSeries.sleepDelay )
						{			
							ActiveToSleep_Ready();
						}
					}																									
		  		}
			}
			break;
			
	    case USER_PROCESS_SLEEP_MODE:
		  	{	 	
			  	if (events & SBP_PERIODIC_EVT)
   			 	{	  
      				events &= ~SBP_PERIODIC_EVT;

      				// Perform periodic application task
	  				SimpleBLEObserver_sleepModelTask();
					
					Util_startClock(&userProcessClock);
    			}
				
				if( userProcessMgr.wakeUpSourse & WAKEUP_SOURSE_KEY )
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
					SleepToActive_Ready();
				}		  		
			}
			break;
			
	    case USER_PROCESS_PERIPHERALDRVFAIL_MODE:
			{
		  		Board_LedCtrl(Board_LED_ON);
				userProcessMode = USER_PROCESS_IDLE_MODE;
			}
			break;
			
	    case USER_PROCESS_LOWVOLTAGE_MODE:
		  {
			if(events & SBP_PERIODIC_EVT)
			{
				events &= ~SBP_PERIODIC_EVT;
					
				Util_startClock(&userProcessClock);		
				
				if( ledStatus & 0x01)
				  ledStatus &= ~(1<<0);
				else
				  ledStatus |= 1<<0;
				
				Board_LedCtrl(ledStatus);
			}
		  }
		  break;
		  
		default:
	 		break;
	}  
  }
  
}

void ActiveToSleep_Ready(void)
{
	bleScanTimeTick = 0;
	userProcessMgr.memsActiveFlg = FALSE;
	userProcessMgr.memsActiveCounter = 0;
	userProcessMgr.memsNoActiveCounter = 0;
	Util_stopClock(&loraUpClock);	
	Util_stopClock(&userProcessClock);	
	Util_restartClock(&loraUpClock, UserTimeSeries.txinterval_S*RCOSC_CALIBRATION_PERIOD);
	Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD_3s);	
	userProcessMode = USER_PROCESS_SLEEP_MODE;
}

void SleepToActive_Ready(void)
{
	userProcessMgr.wakeUpSourse = WAKEUP_SOURSE_RTC; 
	userProcessMgr.memsActiveCounter = 0;
	Util_stopClock(&userProcessClock);	
	Util_stopClock(&loraUpClock);	
	Util_restartClock(&loraUpClock, UserTimeSeries.txinterval*RCOSC_CALIBRATION_PERIOD);
	Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD);
	userProcessMode = USER_PROCESS_ACTIVE_MODE;	
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
	  userProcessMgr.wakeUpSourse |= WAKEUP_SOURSE_KEY;
      break;
	  
  	case SBO_MEMS_ACTIVE_EVT:
	  userProcessMgr.memsActiveFlg = TRUE;
	  userProcessMgr.wakeUpSourse |= WAKEUP_SOURSE_MEMS;
	  Mems_ActivePin_Disable();
	  break;
	  
  	case  SBO_LORA_STATUSPIN_EVT:
	  {
	  	if( RF_TX_RUNNING == SX1278.State )
		{
			sx1278_TxDoneCallback();
			userProcessMgr.rftxtimeout = 0;
			sx1278_EnterRx();
		}
		else if( RF_RX_RUNNING == SX1278.State)
		{
			sx1278_ReadRxPkt();
			if(UserProcess_LoraInf_Get())
			{
			  	sx1278_SetStby();
				Task_sleep(5*1000/Clock_tickPeriod);
				sx1278_SetSleep();
				sx1278_LowPowerMgr();
				userProcessMgr.rfrxtimeout = 0;
			}	
		}	  
	  }
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
	userProcessMgr.sosstatustick = 1;
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
  uint8_t i=0,j=0;
  
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
		if( (USER_PROCESS_PERIPHERALDRVFAIL_MODE != userProcessMode) &&
		    (USER_PROCESS_LOWVOLTAGE_MODE != userProcessMode))
		{
	    	//设备上电初始化完成即开启扫描,扫描时间500ms
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
		  if( USER_PROCESS_IDLE_MODE == userProcessMode)
		  {
			if( 0 == tagInf_t.tagNum )
			{
				userProcessMode = USER_PROCESS_ABNORMAL_MODE; 
				Util_stopClock(&userProcessClock);	
				Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD_30s);
				sx1278Init();
				sx1278_SetSleep();
				sx1278_LowPowerMgr();			
			}
		    else
			{
				userProcessMode = USER_PROCESS_NORAMAL_MODE;	
				tagInf_t.tagNum = 0;
			}
		  }
		  else if( USER_PROCESS_ABNORMAL_MODE == userProcessMode)
		  {
		  	if( tagInf_t.tagNum > 0)
			{
				sx1278_OutputLowPw();
				userProcessMode = USER_PROCESS_NORAMAL_MODE;
			}	  
		  }
		  else
		  {
			  if( 0 == tagInf_t.tagNum )
				return;
			  
			  //根据Rssi大小排序
			  for(i=0; i<tagInf_t.tagNum - 1; i++)
			  {
				  for(j=0; j<tagInf_t.tagNum-i-1; j++)
				  {
					  if(tagInf_t.tagInfBuf_t[j].rssi > tagInf_t.tagInfBuf_t[j+1].rssi)
					  {
						  memcpy(&tagInf_t.tagInfBuf_t[tagInf_t.tagNum-1], &tagInf_t.tagInfBuf_t[j], sizeof(tagInfStruct));
						  memcpy(&tagInf_t.tagInfBuf_t[j], &tagInf_t.tagInfBuf_t[j+1], sizeof(tagInfStruct));
						  memcpy(&tagInf_t.tagInfBuf_t[j+1], &tagInf_t.tagInfBuf_t[tagInf_t.tagNum-1], sizeof(tagInfStruct));															
					  }												
				  }				
			  }
					
			  if(tagInf_t.tagNum > DEFAULT_SINGLESTORMAX_NUM)
				  tagInf_t.tagNum = DEFAULT_SINGLESTORMAX_NUM;
					
			  if(scanTagNum + tagInf_t.tagNum > DEFAULT_UPBLEINFMAX_NUM)
				  return;
					
			  for(i=0; i<tagInf_t.tagNum; i++)
			  {
				  memcpy(&userTxInf.tagInfBuf_t[scanTagNum + i], &tagInf_t.tagInfBuf_t[i], sizeof(tagInfStruct)); 
			  }
					
			  scanTagNum += tagInf_t.tagNum; 
					
			  if(rcoscTimeTick > 0)
			  {
				  switch(rcoscTimeTick)
				  {
					  case 1: userTxInf.device_up_inf.bit_t.beaconNum_1 = tagInf_t.tagNum;
					  break;
							 
					  case 2: userTxInf.device_up_inf.bit_t.beaconNum_2 = tagInf_t.tagNum;
					  break;
							
					  case 3: userTxInf.device_up_inf.bit_t.beaconNum_3 = tagInf_t.tagNum;
					  break;
							
					  case 4: userTxInf.device_up_inf.bit_t.beaconNum_4 = tagInf_t.tagNum;
					  break;
							
					  default:
					  break;
				  }  
			  }
			  tagInf_t.tagNum = 0;
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
 	uint8 minoroffset;
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

		minoroffset = BEELINKER_ADVMINOR_OFFSET; 
		memcpy((void *)(&tagInf_t.tagInfBuf_t[tagNum].minor[0]), (void *)&ptr[minoroffset], sizeof(uint16));
		
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
static void SimpleBLEObserver_userClockHandler(UArg arg)
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

static void SimpleBLEObserver_loraStatusTask(uint8_t rxtimeout, uint8_t txtimeout)
{
	if(RF_RX_RUNNING == SX1278.State)
	{
		userProcessMgr.rfrxtimeout ++;
		/* 接收超时，强制进入Sleep模式 */
		if( userProcessMgr.rfrxtimeout >= rxtimeout)
		{
			sx1278_SetStby();
			Task_sleep(5*1000/Clock_tickPeriod);
			sx1278_SetSleep();
			sx1278_LowPowerMgr();
			userProcessMgr.rfrxtimeout = 0;
		}
	}
	else if(RF_TX_RUNNING == SX1278.State)
	{	 
		userProcessMgr.rftxtimeout ++;
		if( userProcessMgr.rftxtimeout > txtimeout)
		{
			HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
		}
	}
}

static void SimpleBLEObserver_performPeriodicTask(void)
{	
	bleScanTimeTick ++;
	if( bleScanTimeTick > UserTimeSeries.txinterval - UserTimeSeries.scanTimes)
	{
    	GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                    DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                    DEFAULT_DISCOVERY_WHITE_LIST );		
    	if( rcoscTimeTick == UserTimeSeries.scanTimes)
        	rcoscTimeTick = 0;
		
    	rcoscTimeTick ++;
	}
	
	SimpleBLEObserver_loraStatusTask(DEFAULT_RFRXTIMOUT_TIME, DEFAULT_RFTXTIMOUT_TIME);
	
    if(userProcessMgr.sosstatustick !=0)
    {
        userProcessMgr.sosstatustick ++;
        if(userProcessMgr.sosstatustick > DEFAULT_SOSTICK_NUM)
		{
            userProcessMgr.sosstatustick  = 0;
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
	
	bleScanTimeTick ++;
	if( bleScanTimeTick == UserTimeSeries.txinterval_S/3 - 1)
	{
		GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST );		
		
		if( rcoscTimeTick == UserTimeSeries.scanTimes)
		  	rcoscTimeTick = 0;
		
    	rcoscTimeTick ++;   
	}	
	
	SimpleBLEObserver_loraStatusTask(DEFAULT_RFRXTIMOUT_TIME-1, DEFAULT_RFTXTIMOUT_TIME-1);
	
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
			SleepToActive_Ready();
		}				
	}
	else
		userProcessMgr.memsActiveCounter = 0;
}

#define LORATAT_MACADDRE_LEN    6

static bool UserProcess_LoraInf_Get(void)
{
	uint8_t size;
	uint8_t crc;
	uint8_t *ptr;
	uint8_t tmpCnt;
	uint8_t fix;
	uint8_t i = 0;
	uint8_t res;
	payload_inf_n payload_inf;
	  
	ptr = sx1278_GetRxData(&size);
	
	if(ptr == NULL)
		return FALSE;
	
	fix = LORATAG_INFRX_FIX;
	while(i < size)
	{
		if(!memcmp(&ptr[i], &fix, sizeof(uint8_t)))
			break;
		i ++;		
	}
	
	tmpCnt = i + sizeof(uint8_t);
	
	payload_inf.payLoadInf = ptr[tmpCnt];
		
	if ((size - i) < sizeof(uint8_t) + sizeof(payload_inf) + payload_inf.bit_t.pkt_len + sizeof(crc))  
		return FALSE;
	
	for(i=0; i<payload_inf.bit_t.pkt_len + sizeof(payload_inf); i++)
		crc += ptr[tmpCnt + i];
		  
	if(crc != ptr[tmpCnt + sizeof(payload_inf) + payload_inf.bit_t.pkt_len])
		return FALSE;

	memset(rfRxTxBuf, 0, DEFAULT_RFTRANSMIT_LEN);
	memcpy(rfRxTxBuf, &ptr[tmpCnt + sizeof(payload_inf)], payload_inf.bit_t.pkt_len);
	
	res = 0;
	
	if( memcmp(rfRxTxBuf, &userTxInf.devId[0], sizeof(uint16_t)) != 0 )
		return FALSE;
	
	switch(payload_inf.bit_t.pkt_type)
	{
		case TYPE_LORATAGUP:		
			res = 1;			
			break;
			
		case TYPE_TAGPARACONFIG:			  
		  	res = Ble_WriteNv_Inf(BLE_NVID_DEVINF_START, rfRxTxBuf+sizeof(uint16_t));	
			if(res == 0)
				HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);  
		  	break;
			
		default:
			break;
	}
	
	if(res)
	{
		Board_LedCtrl(Board_LED_ON);
		Task_sleep(50*1000/Clock_tickPeriod);
		Board_LedCtrl(Board_LED_OFF);		
		return TRUE;
	}
	
	return FALSE;
}

static void UserProcess_GetLoraUp_Pkt(void)
{
	uint8_t *ptr;
	uint8_t crc;
	uint8_t bleinf_len;
	uint8_t i;
	uint16_t res = 0;
	
	ptr = rfRxTxBuf;
	crc = 0;
	bleinf_len = 0;
	
	userTxInf.loratag_pkt_hdr.pre = LORATAG_INFTX_FIX;
	 
	bleinf_len  =(userTxInf.device_up_inf.bit_t.beaconNum_1 + userTxInf.device_up_inf.bit_t.beaconNum_2 +
				   userTxInf.device_up_inf.bit_t.beaconNum_3 + userTxInf.device_up_inf.bit_t.beaconNum_4) * sizeof(tagInfStruct); 
	
	userTxInf.loratag_pkt_hdr.payload_inf.bit_t.pkt_type = TYPE_LORATAGUP;
	userTxInf.loratag_pkt_hdr.payload_inf.bit_t.pkt_len  = bleinf_len + sizeof(uint16_t) + sizeof(uint16_t);
    
	res |= userTxInf.device_up_inf.bit_t.vbat << 15; 
	if( userProcessMgr.sosstatustick != 0)
		res |= 1<< 14;
	
	res |= userTxInf.device_up_inf.bit_t.acflag << 12; 
	
	res |=  userTxInf.device_up_inf.bit_t.beaconNum_1 << 0;
	res |=  userTxInf.device_up_inf.bit_t.beaconNum_2 << 3;
	res |=  userTxInf.device_up_inf.bit_t.beaconNum_3 << 6;
	res |=  userTxInf.device_up_inf.bit_t.beaconNum_4 << 9;
	res = ntohs(res);
	memcpy(ptr, &userTxInf.loratag_pkt_hdr.pre, 4); //sizeof(loratag_pkt_hdr_t) + ID 

	ptr ++;
	for(i =0; i<3; i++)
		crc += *ptr++; 
	
	memcpy(ptr, &res, sizeof(res));
	for(i =0; i<2; i++)
		crc += *ptr++; 
	
	memcpy(ptr, &userTxInf.tagInfBuf_t[0].rssi, bleinf_len);
	for(i =0; i<bleinf_len; i++)
		crc += *ptr++; 
		
	*ptr++ = crc;
	
	loraUpPktLen = ptr - rfRxTxBuf;

	if(loraUpPktLen > DEFAULT_RFTRANSMIT_LEN)
	 	loraUpPktLen = 0; 
}

static void UserProcess_LoraInf_Send(uint8_t *buf, uint8_t len)
{
  	uint8_t i;
	
	if( SX1278.State == RF_IDLE )	 		
		sx1278_OutputLowPw();
			
	if( sx1278IsChannelFree( SX1278.Modem, SX1278.LoRa.Channel, -100) ) 
	{
	  	sx1278_StatusPin_Disable();
		sx1278_SendBuf(buf, len);
		sx1278_StatusPin_Enable(SimpleBLEObserver_loraStatusHandler);
		scanTagNum = 0;
		UserTimeSeries.channelCheckTime = 0;
	}
	else
	{
	  	UserTimeSeries.channelCheckTime ++;
		if( (UserTimeSeries.channelCheckTime == (DEFAULT_CHANNELCHECK_TIME + 1) )
		    || (UserTimeSeries.channelCheckTime == (DEFAULT_CHANNELCHECK_TIME*2 + 1)) ) //准备调频检测
		{
		  	if( SX1278.LoRa.Channel == default_loraChannel[0] )
				SX1278.LoRa.Channel =default_loraChannel[1];
			else
			    SX1278.LoRa.Channel = default_loraChannel[0];
			
			sx1278_SetRFChannel(SX1278.LoRa.Channel);		  
		}
		sx1278_LowPowerMgr();
				
		if( UserTimeSeries.channelCheckTime > DEFAULT_CHANNELCHECK_TIME)
			i = UserTimeSeries.channelCheckTime - DEFAULT_CHANNELCHECK_TIME;
		else 
		    i = UserTimeSeries.channelCheckTime;
		
		if( i < DEFAULT_CHANNELCHECK_TIME + 1 )
			Util_restartClock(&loraUpDelayClock, RCOSC_CALIBRATION_PERIOD_160ms*i);
	}
}

void Voltage_Check(void)
{
	uint8_t val; 
	  
	val = adc_OneShot_Read();
  	if( 0 == val )
		userTxInf.device_up_inf.bit_t.vbat = 1;
  	else if( 1 == val )
		userTxInf.device_up_inf.bit_t.vbat = 0;
 	else
		userProcessMode = USER_PROCESS_LOWVOLTAGE_MODE;
}

void TagPara_Get(void)
{
	uint8_t buf[10];
	uint32_t crc; 
	int8_t res;
	
	snv_device_inf_t *ptr = (snv_device_inf_t *)buf;
	
	if( Ble_ReadNv_Inf( BLE_NVID_DEVINF_START, (uint8_t *)ptr) == 0 )
	{
	  	crc = crc32(0, &buf[ sizeof(crc) ], sizeof(lora_para_n) + sizeof(tag_para_n) + sizeof(ble_para_n));
		if( crc == ptr->crc32)
		{
			userLoraPara = (LoRaSettings_t *)rfRxTxBuf;
			
			/******************* LORA Para ***************/
			if(ptr->loraPara_u.bit_t.power == 1)
			{
				userLoraPara->Power = 20;
			}
			
			if( ptr->loraPara_u.bit_t.rate == 1)
			{
				userLoraPara->Coderate = 2;
			}
			
			if( ptr->loraPara_u.bit_t.channel == 1)
			{
				userLoraPara->Channel = 470000000;
			}
			
			/* Test Para */
			userLoraPara->Bandwidth             = 7;
			userLoraPara->Sf                    = 9;
			userLoraPara->PreambleLen           = 8;
			userLoraPara->LowDatarateOptimize   = FALSE;
			userLoraPara->FixLen                = 0;
			userLoraPara->PayloadLen            = 64;
			userLoraPara->CrcOn                 = TRUE;
			userLoraPara->FreqHopOn             = FALSE;
			userLoraPara->HopPeriod             = 0;
			userLoraPara->IqInverted            = TRUE;
			userLoraPara->RxContinuous          = TRUE;
			/******************* END LORA Para ***********/
			
			/*********** TAG Para ************************/
			ptr->tagPara_u.tagPara = ntohs(ptr->tagPara_u.tagPara);
			res = ptr->tagPara_u.bit_t.sleep_delay;	
			switch(res)
			{
				case 0 :  UserTimeSeries.sleepDelay = 4; //4s
				break;
					
				case 1 :  UserTimeSeries.sleepDelay = 8; //8s
				break;
					
				case 2 :  UserTimeSeries.sleepDelay = 20; //20s
				break;
					
				case 3 :  UserTimeSeries.sleepDelay = 60; //60s
				break;
				    
				default:  UserTimeSeries.sleepDelay = 4; //4s 
				break;
			}
			
			res = ptr->tagPara_u.bit_t.scan_num ;
			if( ( res < 4) && ( res >= 0))
			{
				UserTimeSeries.scanTimes = res + 1;
			}
			else
			{
				UserTimeSeries.scanTimes = 4;
			}
			
			res = ptr->tagPara_u.bit_t.up_interval ;
			if( ( res >= 0 ) && ( res < 10))
			{
				UserTimeSeries.txinterval = res + 1; //s
			}
			else 
			{
			  	switch( res )
				{
					case 10 :  UserTimeSeries.txinterval = 60; //1min
					break;
					
					case 11 :  UserTimeSeries.txinterval = 120; //2min
					break;
					
					case 12 :  UserTimeSeries.txinterval = 180; //3min
					break;
					
					case 13 :  UserTimeSeries.txinterval = 300; //5min
					break;
					
					case 14 :  UserTimeSeries.txinterval = 480; //8min
					break;
					
					case 15 :  UserTimeSeries.txinterval = 600; //10min
					break;					
				    
					default:  UserTimeSeries.txinterval = 4; //4s 
					break;
				}
			}
			
			res = ptr->tagPara_u.bit_t.up_interval_s ;
			if( ( res >= 0 ) && ( res < 6 ) )
			{
				UserTimeSeries.txinterval_S = ( res + 1 ) * 30; // 30s ~ 3min	
			}
			else if( ( res >= 6 ) && ( res < 12 ))
			{
				UserTimeSeries.txinterval_S = ( res - 5 ) * 300; // 5min ~ 30min
			}
			else
			{
				UserTimeSeries.txinterval_S = 3600;    //1h
			}
			/*********** END TAG Para ********************/
									
			/*********************** BLE Para ************/
			if( ( ptr->blePara_u.blePara >0 ) && ( ptr->blePara_u.blePara < 10 ))
			{
				UserTimeSeries.timeForscanning = 100 * ptr->blePara_u.blePara;
			}
			else
			{
				UserTimeSeries.timeForscanning = 50;
			}
			/*******************END BLE Para *************/
		}
		else
			userLoraPara = NULL;  
	}
	
	if( Ble_ReadNv_Inf( BLE_NVID_DRFLG_START, (uint8_t *)&snv_DriverFailure.crc32) == 0 )
	{
		crc = crc32(0, (uint8_t *)&snv_DriverFailure.mems, sizeof(uint16_t));	  
		if( crc == snv_DriverFailure.crc32)
		{	
			if( (snv_DriverFailure.lora > DEFAULT_PERDRVFAILMAX_TIME) ||
			 
			  	(snv_DriverFailure.mems > DEFAULT_PERDRVFAILMAX_TIME))
		 	{
		      	userProcessMode = USER_PROCESS_PERIPHERALDRVFAIL_MODE;	
		  	}	
		}
	}
}

#ifdef IWDG_ENABLE 
void wdtCallback(UArg handle) 
{
	Watchdog_clear((Watchdog_Handle)handle);
}

void wdtInitFxn(void) 
{
	Watchdog_Params wp;
	
	Watchdog_init();
	Watchdog_Params_init(&wp);
	wp.callbackFxn    = (Watchdog_Callback)wdtCallback;
	wp.debugStallMode = Watchdog_DEBUG_STALL_ON;
	wp.resetMode      = Watchdog_RESET_ON;
 
	watchdog = Watchdog_open(CC2650_WATCHDOG0, &wp);
	Watchdog_setReload(watchdog, 1500000); // 1sec (WDT runs always at 48MHz/32)
}
#endif

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
