/******************************************************************************

 @file  sensortag_light.c

 @brief This file contains the SensorTag LED Light sub-application.

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
#ifdef INCLUDE_LIGHT

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>

#ifdef POWER_SAVING
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#endif

#include "board.h"
#include "bcomdef.h"
#include "sensortag.h"
#include "devpk_light.h"
#include "lightservice.h"
#include "st_util.h"
#include "sensortag_light.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * Profile callbacks
 */
static void lightChangeCB(uint8_t paramID);

/*********************************************************************
 * LOCAL VARIABLES
 */

// LED intensity values
static uint8_t lvalRed;
static uint8_t lvalGreen;
static uint8_t lvalBlue;
static uint8_t lvalWhite;

// Light Profile Callbacks
static lightsProfileCBs_t bleLight_LightsProfileCBs =
{
    lightChangeCB  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagLight_init
 *
 * @brief   Initialize the LED Lights module
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagLight_init(void)
{
  Lights_addService();
  Lights_registerAppCBs(&bleLight_LightsProfileCBs);

  devpkLightOpen();
  SensorTagLight_reset();
}


/*********************************************************************
 * @fn      SensorTagLight_reset
 *
 * @brief   Reset all sensors, typically when a connection is intentionally
 *          terminated.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagLight_reset(void)
{
  uint8_t rgbwVal[LIGHTSPROFILE_RGBW_LEN] = {0,0,0,0};

  lvalRed = 0;
  lvalBlue = 0;
  lvalGreen = 0;
  lvalWhite = 0;
  Lights_setParameter(LIGHTSPROFILE_RGBW, LIGHTSPROFILE_RGBW_LEN, &rgbwVal);

  devpkSetLight(Board_DEVPK_LIGHT_RED, lvalRed);
  devpkSetLight(Board_DEVPK_LIGHT_GREEN, lvalGreen);
  devpkSetLight(Board_DEVPK_LIGHT_BLUE, lvalBlue);
  devpkSetLight(Board_DEVPK_LIGHT_WHITE, lvalWhite);
}


/*********************************************************************
 * @fn      SensorTagLight_processCharChangeEvt
 *
 * @brief   Callback from the profile indicating a data change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
void SensorTagLight_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newVal;

  switch (paramID)
  {
    case LIGHTSPROFILE_RED:
      Lights_getParameter(LIGHTSPROFILE_RED, &newVal);
      lvalRed = newVal;
      break;

    case LIGHTSPROFILE_GREEN:
      Lights_getParameter(LIGHTSPROFILE_GREEN, &newVal);
      lvalGreen = newVal;
      break;

    case LIGHTSPROFILE_BLUE:
      Lights_getParameter(LIGHTSPROFILE_BLUE, &newVal);
      lvalBlue = newVal;
      break;

    case LIGHTSPROFILE_WHITE:
      Lights_getParameter(LIGHTSPROFILE_WHITE, &newVal);
      lvalWhite = newVal;
      break;

    case LIGHTSPROFILE_RGBW:
      {
        uint8_t rgbwVal[LIGHTSPROFILE_RGBW_LEN];

        Lights_getParameter(LIGHTSPROFILE_RGBW, &rgbwVal);
        lvalRed = rgbwVal[0];
        lvalGreen = rgbwVal[1];
        lvalBlue = rgbwVal[2];
        lvalWhite = rgbwVal[3];
      }
      break;
    default:
      // should not reach here!
      break;
  }

  // Apply the new values
  devpkSetLight(Board_DEVPK_LIGHT_RED, lvalRed);
  devpkSetLight(Board_DEVPK_LIGHT_GREEN, lvalGreen);
  devpkSetLight(Board_DEVPK_LIGHT_BLUE, lvalBlue);
  devpkSetLight(Board_DEVPK_LIGHT_WHITE, lvalWhite);

#ifdef POWER_SAVING
  // Make sure the power domains are on if any light is active
  if (lvalRed>0 || lvalGreen>0 || lvalBlue>0 || lvalWhite>0)
  {
    /* Set constraints for Standby and Idle mode */
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
  }
  else
  {
    /* Release constraints for Standby and Idle mode */
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
    Power_releaseConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
  }
#endif // POWER_SAVING
}

/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      lightChangeCB
 *
 * @brief   Callback from Light service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void lightChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_LIGHT, paramID);
}

#endif // #ifdef INCLUDE_LIGHT

/*********************************************************************
*********************************************************************/
