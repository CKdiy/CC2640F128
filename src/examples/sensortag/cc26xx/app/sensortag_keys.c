/******************************************************************************

 @file  sensortag_keys.c

 @brief This file contains the Sensor Tag sample application,
        Keys part, for use with the TI Bluetooth Low
        Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2018, Texas Instruments Incorporated
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

#ifndef EXCLUDE_KEYS

/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "sensortag_keys.h"
#include "sensortag_io.h"
#include "ioservice.h"
#include "sensortag_factoryreset.h"
#include "board.h"
#include "peripheral.h"
#include "simplekeys.h"
#include "sensortag_audio.h"
#include "util.h"

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

/*********************************************************************
 * MACROS
 */
// Adaptation for LaunchPad
#ifndef Board_KEY_RIGHT
#define Board_KEY_RIGHT         Board_BTN2
#endif

#ifndef Board_KEY_LEFT
#define Board_KEY_LEFT          Board_BTN1
#endif

/*********************************************************************
 * CONSTANTS
 */
#define SK_KEY_REED             0x04
#define SK_PUSH_KEYS            (SK_KEY_LEFT | SK_KEY_RIGHT)

// Key press time-outs (seconds)
#define POWER_PRESS_PERIOD      3
#define RESET_PRESS_PERIOD      6

// Events
#define SK_EVT_FACTORY_RESET    0x01
#define SK_EVT_DISCONNECT       0x02

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
static uint8_t keys;
static uint16_t keyLeftTimer;
static uint16_t keyRightTimer;
static Clock_Struct periodicClock;
static uint8_t event;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processGapStateChange(void);
static void SensorTagKeys_clockHandler(UArg arg);
#ifdef Board_RELAY
static void configureRelay(void);
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagKeys_init
 *
 * @brief   Initialization function for the SensorTag keys
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_init(void)
{
  // Add service
  SK_AddService(GATT_ALL_SERVICES);

  // Initialize the module state variables
  SensorTagKeys_reset();

  // Create one-shot clock for key press timing (tick per second)
  Util_constructClock(&periodicClock, SensorTagKeys_clockHandler,
                      100, 1000, false, 0);
#ifdef Board_RELAY
  // Detect relay type and configure the input
  configureRelay();
#endif
}

/*********************************************************************
 * @fn      SensorTagKeys_processKeyRight
 *
 * @brief   Interrupt handler for BUTTON 1(right)
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_processKeyRight(void)
{
  if (PIN_getInputValue(Board_KEY_RIGHT))
  {
    keys &= ~SK_KEY_RIGHT;
  }
  else
  {
    keys |= SK_KEY_RIGHT;
  }

  // Wake up the application thread
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SensorTagKeys_processKeyLeft
 *
 * @brief   Interrupt handler for BUTTON 2 (left)
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_processKeyLeft(void)
{
  if (PIN_getInputValue(Board_KEY_LEFT))
  {
    keys &= ~SK_KEY_LEFT;
  }
  else
  {
    keys |= SK_KEY_LEFT;
  }

  // Wake up the application thread
  Semaphore_post(sem);
}

#ifdef Board_RELAY
/*********************************************************************
 * @fn      SensorTagKeys_processRelay
 *
 * @brief   Interrupt service routine for Reed relay
 *          (from PCB rev 1.5.1 Hall Effect sensor)
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_processRelay(void)
{
  if (PIN_getInputValue(Board_RELAY))
  {
    keys |= SK_KEY_REED;
  }
  else
  {
    keys &= ~SK_KEY_REED;
  }

  // Wake up the application thread
  Semaphore_post(sem);
}
#endif // Board_RELAY

/*********************************************************************
 * @fn      SensorTagKeys_processEvent
 *
 * @brief   SensorTag Keys event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_processEvent(void)
{
  static uint8_t current_keys = 0;

  // Factory reset by six second simultaneous key press
  if (event & SK_EVT_FACTORY_RESET)
  {
    event &= ~SK_EVT_FACTORY_RESET;

    // Indicate that we're entering factory reset
    SensorTagIO_blinkLed(IOID_RED_LED, 10);

    // Apply factory image and reboot
    SensorTagFactoryReset_applyFactoryImage();
  }

  // Disconnect on three seconds press on the power switch (right key)
  if (event & SK_EVT_DISCONNECT)
  {
    event &= ~SK_EVT_DISCONNECT;
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      processGapStateChange();
    }
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  if (current_keys != keys)
  {
    SK_SetParameter(SK_KEY_ATTR, sizeof(uint8_t), &keys);

    // Have keys been released?
    if ((current_keys & SK_KEY_LEFT)!=0 && (keys & SK_KEY_LEFT)==0)
    {
      keyLeftTimer = 0;
    }

    if ((current_keys & SK_KEY_RIGHT)!=0 && (keys & SK_KEY_RIGHT)==0)
    {
      keyRightTimer = 0;
    }

    // Insert key state into advertising data
    if (gapProfileState == GAPROLE_ADVERTISING)
    {
      SensorTag_updateAdvertisingData(keys);
    }

    // Check if right key was pressed
    if ((current_keys & SK_KEY_RIGHT)!=0 && (keys & SK_KEY_RIGHT)==0)
    {
      if (gapProfileState != GAPROLE_CONNECTED)
      {
        // Not connected; change state immediately (power/right button)
        processGapStateChange();
      }
    }

#ifndef EXCLUDE_AUDIO
    // Check left key press
    if ((current_keys & SK_KEY_LEFT)==0 && (keys & SK_KEY_LEFT)!=0)
    {
      SensorTagAudio_enableStreaming(true);
    }

    // Check left key release
    if ((current_keys & SK_KEY_LEFT)!=0 && (keys & SK_KEY_LEFT)==0)
    {
      SensorTagAudio_enableStreaming(false);
    }

#endif // EXCLUDE_AUDIO

    // Has a key been pressed ?
    if ((keys & SK_PUSH_KEYS) && (current_keys == 0))
    {
      if (!Util_isActive(&periodicClock))
      {
        Util_startClock(&periodicClock);
        keyRightTimer = 0;
        keyLeftTimer = 0;
      }
    }
  }

  current_keys = keys;
}

/*********************************************************************
 * @fn      SensorTagKeys_reset
 *
 * @brief   Reset key state to 'not pressed'
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_reset(void)
{
  keyLeftTimer = 0;
  keyRightTimer = 0;
  keys = 0;
  event = 0;
  SK_SetParameter(SK_KEY_ATTR, sizeof(uint8_t), &keys);
}

/*********************************************************************
 * @fn      SensorTagKeys_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void SensorTagKeys_clockHandler(UArg arg)
{
  // Are both keys pressed?
  if (keys & SK_KEY_RIGHT)
  {
    keyRightTimer++;
  }

  if (keys & SK_KEY_LEFT)
  {
    keyLeftTimer++;
  }

  // Both keys have been pressed for 6 seconds -> restore factory image
  if (keyLeftTimer >= RESET_PRESS_PERIOD && keyRightTimer >= RESET_PRESS_PERIOD)
  {
    // Stop the clock
    if (Util_isActive(&periodicClock))
    {
      Util_stopClock(&periodicClock);
      keyLeftTimer = 0;
      keyRightTimer = 0;

      // set event flag and wake up the application thread
      event |= SK_EVT_FACTORY_RESET;
      Semaphore_post(sem);
    }
  }
  // Right key (POWER) pressed for three seconds, disconnect if connected
  else if (keyRightTimer >= POWER_PRESS_PERIOD && keyLeftTimer == 0)
  {
    // Stop the clock
    if (Util_isActive(&periodicClock))
    {
      Util_stopClock(&periodicClock);
      keyRightTimer = 0;

      // set event flag and wake up the application thread
      event |= SK_EVT_DISCONNECT;
      Semaphore_post(sem);
    }
  }
  else if (keyLeftTimer == 0 && keyRightTimer == 0)
  {
    // Stop the clock
    if (Util_isActive(&periodicClock))
    {
      Util_stopClock(&periodicClock);
    }
  }
}

/*********************************************************************
 * @fn      processGapStateChange
 *
 * @brief   Change the GAP state.
 *          1. Connected -> disconnect and start advertising
 *          2. Advertising -> stop advertising
 *          3. Disconnected/not advertising -> start advertising
 *
 * @param   none
 *
 * @return  none
 */
static void processGapStateChange(void)
{
  if (gapProfileState != GAPROLE_CONNECTED)
  {
    uint8_t current_adv_enabled_status;
    uint8_t new_adv_enabled_status;

    // Find the current GAP advertising status
    GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);

    if (current_adv_enabled_status == FALSE)
    {
      new_adv_enabled_status = TRUE;
    }
    else
    {
      new_adv_enabled_status = FALSE;
    }

    // Change the GAP advertisement status to opposite of current status
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &new_adv_enabled_status);
  }

  if (gapProfileState == GAPROLE_CONNECTED)
  {
    uint8_t adv_enabled = TRUE;

    // Disconnect
    GAPRole_TerminateConnection();

    // Start advertising
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_enabled);
  }
}

#ifdef Board_RELAY
/*********************************************************************
 * @fn      configureRelay
 *
 * @brief   Re-configure IO-pin if Hall Effect Sensor detected.
 *
 * @descr   SensorTags with PCB revision 1.5.0 and higher are equipped
 *          with a Hall Effect Sensor instead of a Reed Relay. The application
 *          needs to take this into account because the polarity of Hall
 *          Effect Sensor is active low, whereas the Reed Relay is active high.
 *
 * @return  none
 */
static void configureRelay(void)
{
  uint_t value = PIN_getInputValue(Board_RELAY);

  if (value > 0) // Pin is high, assume Hall Effect Sensor
  {
    PIN_Config pinCfg;
    PIN_Config pinCfgBm;

    // Reconfigure as pull-up + inverted output. Turn off hysteresis.
    pinCfg = PIN_ID(Board_RELAY) | PIN_NOPULL | PIN_INV_INOUT;
    pinCfgBm = PIN_BM_PULLING | PIN_BM_INV_INOUT | PIN_BM_HYSTERESIS;

    PIN_setConfig(hGpioPin, pinCfgBm, pinCfg);
  }
  // else: Pin is low. Assume Reed Relay - no reconfiguration required
}
#endif // Board_RELAY

#endif // EXCLUDE_KEYS

/*********************************************************************
*********************************************************************/
