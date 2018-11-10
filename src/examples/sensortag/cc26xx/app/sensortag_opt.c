/******************************************************************************

 @file  sensortag_opt.c

 @brief This file contains the Sensor Tag sample application,
        Optical sensor, for use with the TI Bluetooth Low
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

#ifndef EXCLUDE_OPT
/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gattservapp.h"

#include "opticservice.h"
#include "sensortag_opt.h"
#include "SensorOpt3001.h"
#include "SensorTagTest.h"
#include "board.h"
#include "util.h"
#include "string.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   800

// Length of the data for this sensor
#define SENSOR_DATA_LEN         OPTIC_DATA_LEN

// Event flag for this sensor
#define SENSOR_EVT              ST_OPTIC_SENSOR_EVT


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
static Clock_Struct periodicClock;

// Parameters
static uint8_t sensorConfig;
static uint16_t sensorPeriod;
static bool sensorReadScheduled;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen);
static void SensorTagOpt_clockHandler(UArg arg);

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
 * @fn      SensorTagOpt_init
 *
 * @brief   Initialize scheduler for light sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagOpt_init(void)
{
  // Add service
  Optic_addService();

  // Register callbacks with profile
  Optic_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  sensorReadScheduled = false;
  SensorTagOpt_reset();
  initCharacteristicValue(SENSOR_PERI,
                          SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                          sizeof(uint8_t));

  // Initialize the driver
  SensorOpt3001_init();
  SensorOpt3001_enable(false);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SensorTagOpt_clockHandler,
                      100, sensorPeriod, false, 0);
}

/*********************************************************************
 * @fn      SensorTagOpt_processCharChangeEvt
 *
 * @brief   SensorTag Light Sensor event handling
 *
 * @param   paramID - parameter identifier
 *
 * @return  none
 */
void SensorTagOpt_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  bool accepted;

  accepted = false;
  switch (paramID)
  {
  case SENSOR_CONF:
    if ((SensorTag_testResult() & SENSOR_OPT_TEST_BM) == 0)
    {
      sensorConfig = ST_CFG_ERROR;
    }

    if (sensorConfig != ST_CFG_ERROR)
    {
      Optic_getParameter(SENSOR_CONF, &newValue);

      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (sensorConfig != ST_CFG_SENSOR_DISABLE)
        {
          accepted = true;
          Util_stopClock(&periodicClock);
          SensorOpt3001_enable(false); // Disable the sensor
          initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
        }
      }
      else
      {
        if (sensorConfig == ST_CFG_SENSOR_DISABLE)
        {
          accepted = true;
          SensorOpt3001_enable(true);
          Util_startClock(&periodicClock);
        }
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
    Optic_getParameter(SENSOR_PERI, &newValue);
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION;
    Util_rescheduleClock(&periodicClock,sensorPeriod);
    break;

  default:
    // Should not get here
    break;
  }
}


/*********************************************************************
 * @fn      SensorTagOpt_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagOpt_reset(void)
{
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, ST_CFG_SENSOR_DISABLE,
                          sizeof(uint8_t));

  if ((SensorTag_testResult() & SENSOR_OPT_TEST_BM) > 0)
  {
    SensorOpt3001_enable(false); // Disable the sensor
  }
}


/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      SensorTagOpt_processSensorEvent
 *
 * @brief   SensorTag Light Meter event processor.
 *
 * @return  none
 */
void SensorTagOpt_processSensorEvent(void)
{
  if (sensorReadScheduled)
  {
    uint16_t data;

    SensorOpt3001_read(&data);
    Optic_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, &data);
    sensorReadScheduled = false;
  }
}


/*********************************************************************
 * @fn      SensorTagOpt_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void SensorTagOpt_clockHandler(UArg arg)
{
  if (sensorConfig == ST_CFG_SENSOR_ENABLE)
  {
    // Wake up the application.
    sensorReadScheduled = true;
    Semaphore_post(sem);
  }
}


/*********************************************************************
 * @fn      sensorConfigChangeCB
 *
 * @brief   Callback from Optical Service indicating a configuration change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_OPT, paramID);
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
  Optic_setParameter(paramID, paramLen, data);
}
#endif // EXCLUDE_OPT

/*********************************************************************
*********************************************************************/

