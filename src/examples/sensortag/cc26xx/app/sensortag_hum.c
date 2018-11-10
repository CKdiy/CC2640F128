/******************************************************************************

 @file  sensortag_hum.c

 @brief This file contains the Sensor Tag sample application,
        Humidity part, for use with the TI Bluetooth Low
        Energy Protocol Stack.

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

#ifndef EXCLUDE_HUM
/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "board.h"
#include "string.h"

#include "humidityservice.h"
#include "sensortag_hum.h"
#include "SensorHdc1000.h"
#include "sensortag.h"
#include "SensorTagTest.h"
#include "SensorUtil.h"

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   1000

// Time start measurement and data ready
#define HUM_DELAY_PERIOD        15

// Length of the data for this sensor
#define SENSOR_DATA_LEN         HUMIDITY_DATA_LEN

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600

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
static ICall_EntityID sensorSelfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sensorSem;

// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTaskFxn(UArg a0, UArg a1);
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      SensorTagHum_createTask
 *
 * @brief   Task creation function for the SensorTag
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagHum_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

/*********************************************************************
 * @fn      SensorTagHum
 *
 * @brief   Initialization function for the SensorTag humidity sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagHum_init(void)
{
  // Add service
  Humidity_addService();

  // Register callbacks with profile
  Humidity_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  SensorTagHum_reset();
  initCharacteristicValue(SENSOR_PERI, SENSOR_DEFAULT_PERIOD
                          / SENSOR_PERIOD_RESOLUTION, sizeof(uint8_t));

  // Initialize the driver
  SensorHdc1000_init();
}

/*********************************************************************
 * @fn      SensorTagHum_processCharChangeEvt
 *
 * @brief   SensorTag Humidity event handling
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagHum_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  bool accepted;
  
  accepted = false;
  switch (paramID)
  {
  case  SENSOR_CONF:
    if ((SensorTag_testResult() & SENSOR_HUM_TEST_BM) == 0)
    {
      sensorConfig = ST_CFG_ERROR;
    }

    if (sensorConfig != ST_CFG_ERROR)
    {
      Humidity_getParameter(SENSOR_CONF, &newValue);

      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        accepted = true;
        
        // Reset characteristics
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);

        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);
      }
      else if (newValue == ST_CFG_SENSOR_ENABLE)
      {
        accepted = true;
        
        // Activate task
        Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
      }
    }
    
    if (accepted)
    {
      // Accept the new value
      sensorConfig = newValue;
    }
    else
    {
      // Restore the previous value
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(sensorConfig));
    }

    break;

  case SENSOR_PERI:
    Humidity_getParameter(SENSOR_PERI, &newValue);
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION;
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagHum_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagHum_reset(void)
{
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof(uint8_t));
}

/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the humidity readout task
 *
 * @param   a0 - not used
 *
 * @param   a1 - not used
 *
 * @return  none
 */
static void sensorTaskFxn(UArg a0, UArg a1)
{
  typedef union {
    struct {
      uint16_t rawTemp, rawHum;
    } v;
    uint8_t a[SENSOR_DATA_LEN];
  } Data_t;

  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);

  // Task loop
  while (true)
  {
    if (sensorConfig == ST_CFG_SENSOR_ENABLE)
    {
      Data_t data;

      // 1. Start temperature measurement
      SensorHdc1000_start();
      DELAY_MS(HUM_DELAY_PERIOD);

      // 2. Read data
      SensorHdc1000_read(&data.v.rawTemp, &data.v.rawHum);

      // 3. Send data
      Humidity_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, data.a);

      // 4. Wait until next cycle
      DELAY_MS(sensorPeriod - HUM_DELAY_PERIOD);
    }
    else
    {
      DELAY_MS(SENSOR_DEFAULT_PERIOD);
    }
  }
}

/*********************************************************************
 * @fn      sensorConfigChangeCB
 *
 * @brief   Callback from Humidity Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_HUM, paramID);
}

/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  Humidity_setParameter(paramID, paramLen, data);
}
#endif // EXCLUDE_HUM

/*********************************************************************
*********************************************************************/

