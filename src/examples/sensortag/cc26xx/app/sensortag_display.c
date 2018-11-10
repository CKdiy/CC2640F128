/******************************************************************************

 @file  sensortag_display.c

 @brief This file contains the Sensor Tag sample application, LCD support.

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
#ifdef INCLUDE_DISPLAY

/*********************************************************************
 * INCLUDES
 */
#include "stdio.h"

// Common
#include "gatt.h"
#include "gattservapp.h"
#include "displayservice.h"
#include "devinfoservice.h"
#include "util.h"

// SensorTag
#include "sensortag.h"
#include "sensortag_display.h"

// Middleware
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplaySharp.h>

// DriverLib
#include <driverlib/aon_batmon.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Display usage
#define DISPLAY_LINE_DEV_INFO      0
#define DISPLAY_LINE_BOARD_ADDR    1
#define DISPLAY_LINE_FWREV         2
#define DISPLAY_LINE_DEFAULT       7
#define DISPLAY_LINE_BATTERY       10
#define DISPLAY_LINE_GAP           11

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint8_t line;
  uint8_t column;
} CursorPos_t;

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
static CursorPos_t pos;
static Display_Handle handle;
static DisplaySharpColor_t color=
{
    .bg = ClrBlack,
    .fg = ClrWhite,
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void displayChangeCB(uint8_t newParamID);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorTag_displayCBs =
{
    displayChangeCB,               // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagDisplay_init
 *
 * @brief   Initialization function for the SensorTag Display
 *
 */
void SensorTagDisplay_init(void)
{
    uint8_t config[DISPLAY_CONF_LEN];

    // Add service
    Display_addService();
    Display_registerAppCBs(&sensorTag_displayCBs);

    // Initialize the service
    config[DISPLAY_CMD_OFFSET] = DISPLAY_CONF_MOV;
    config[DISPLAY_LINE_OFFSET] = DISPLAY_LINE_DEFAULT;
    config[DISPLAY_COL_OFFSET] = 1;
    Display_setParameter(DISPLAY_CONF, sizeof(config), config);

    // Store active "cursor" position
    pos.line = config[DISPLAY_LINE_OFFSET];
    pos.column = config[DISPLAY_COL_OFFSET];

    // Initialize LCD
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_NONE;
    handle = Display_open(Display_Type_LCD, &params);
}


/*********************************************************************
 * @fn      SensorTagDisplay_processCharChangeEvt
 *
 * @brief   Process a change in the IO characteristics
 *
 * @return  none
 */
void SensorTagDisplay_processCharChangeEvt(uint8_t paramID)
{
    if (paramID == DISPLAY_CONF)
    {
        uint8_t config[DISPLAY_CONF_LEN];

        Display_getParameter(DISPLAY_CONF, config);

        // Execute the control sequence
        switch (config[DISPLAY_CMD_OFFSET])
        {
        case DISPLAY_CONF_OFF:
            if (handle != NULL)
            {
                // NB! Has no effect on SensorTag because LCD_ENABLE pin is
                // shared with UART_TX, causing the LCD to get enabled again
                // as soon as the PINs used by the LCD are released.
                Display_clear(handle);
                Display_print0(handle, DISPLAY_LINE_DEFAULT, 2, "Display off");

                Display_close(handle);
                handle = NULL;
            }
            break;

        case DISPLAY_CONF_ON:
            // Initialize LCD
            if (handle == NULL)
            {
                Display_Params params;
                Display_Params_init(&params);
                params.lineClearMode = DISPLAY_CLEAR_NONE;
                handle = Display_open(Display_Type_LCD, &params);

                // Update status
                SensorTagDisplay_showStatus();
            }
            break;

        case DISPLAY_CONF_CLR:
            // Clear display
            if (handle != NULL)
            {
                Display_clear(handle);
            }
            break;

        case DISPLAY_CONF_CLR_LINE:
            // Clear line
            pos.line = config[DISPLAY_LINE_OFFSET];
            if (handle != NULL)
            {
                Display_clearLines(handle, pos.line, pos.line);
            }
            break;

        case DISPLAY_CONF_INV:
            if (handle != NULL)
            {
                uint8_t tmp;

                // Swap foreground and background colors
                tmp = color.fg;
                color.fg = color.bg;
                color.bg = tmp;

                Display_control(handle, DISPLAYSHARP_CMD_SET_COLORS, &color);

                // Update status
                SensorTagDisplay_showStatus();
            }
            break;

        case DISPLAY_CONF_MOV:
            // Store active "cursor" position
            pos.line = config[DISPLAY_LINE_OFFSET];
            pos.column = config[DISPLAY_COL_OFFSET];

            break;

        default:
            break;
        }
    }

    if (paramID == DISPLAY_DATA && handle != NULL)
    {
        uint8_t data[DISPLAY_BUFFER_LEN];

        Display_getParameter(DISPLAY_DATA, data);
        Display_print0(handle, pos.line, pos.column, (const char*)data);
    }
}

/*******************************************************************************
 * @fn      SensorTagDisplay_showStatus
 *
 * @brief   Display main status parameters on the display.
 *
 *          - device name
 *          - device address
 *          - firmware revision
 *          - GAP connection status
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagDisplay_showStatus(void)
{
    if (handle != NULL)
    {
        uint8_t buf[24];
        char *gapStatusStr;

        // Make sure display is blank
        Display_clear(handle);

        // Line 0: device info
        DevInfo_GetParameter(DEVINFO_MODEL_NUMBER, buf);
        Display_print0(handle, DISPLAY_LINE_DEV_INFO, 0, (const char*)buf);

        // Line 1: board address
        GAPRole_GetParameter(GAPROLE_BD_ADDR, buf);
        Display_print0(handle, DISPLAY_LINE_BOARD_ADDR,
                       1, Util_convertBdAddr2Str(buf));

        // Line 2: firmware revision
        DevInfo_GetParameter(DEVINFO_FIRMWARE_REV, buf);
        Display_print0(handle, DISPLAY_LINE_FWREV, 1, "FW rev.");
        buf[5] = '\0'; // Display max 5 characters (leave out date part)
        Display_print0(handle, DISPLAY_LINE_FWREV, 10, (const char*)buf);

        // Line 11: GAP connection status
        switch (gapProfileState)
        {
        case GAPROLE_INIT:
            gapStatusStr = "Init";
            break;
        case GAPROLE_STARTED:
            gapStatusStr = "Started";
            break;
        case GAPROLE_ADVERTISING:
            gapStatusStr = "Advertising";
            break;
        case GAPROLE_ADVERTISING_NONCONN:
            gapStatusStr = "Advertising-nc";
            break;
        case GAPROLE_WAITING:
            gapStatusStr = "Waiting";
            break;
        case GAPROLE_WAITING_AFTER_TIMEOUT:
            gapStatusStr = "Waiting-tout";
            break;
        case GAPROLE_CONNECTED:
            gapStatusStr = "Connected";
            break;
        case GAPROLE_CONNECTED_ADV:
            gapStatusStr = "Connected-adv";
            break;
        case GAPROLE_ERROR:
            gapStatusStr = "Error";
            break;
        default:
            gapStatusStr = "Unknown";
            break;
        }
        Display_print0(handle, DISPLAY_LINE_GAP, 2, gapStatusStr);
    }
}


/*******************************************************************************
 * @fn      SensorTagDisplay_showBatteryLevel
 *
 * @brief   Display the battery voltage in millivolts
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagDisplay_showBatteryLevel(void)
{
    if (AONBatMonNewBatteryMeasureReady())
    {
        uint32_t batt;

        // Battery monitor (bit 10:8 - integer, but 7:0 fraction)
        batt = AONBatMonBatteryVoltageGet();

        batt = (batt * 125) >> 5;
        Display_print1(handle, 10, 1, "Batt: %d mV", batt);
    }
}

/*******************************************************************************
 * @fn      SensorTagDisplay_suspend
 *
 * @brief   Releases the SPI driver for use by other devices. Power to the LCD
 *          is retained.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagDisplay_suspend(void)
{
    Display_clear(handle);
    Display_print0(handle, DISPLAY_LINE_DEFAULT, 2, "Display off");

    Display_control(handle, DISPLAY_CMD_TRANSPORT_CLOSE, NULL);
}

/*******************************************************************************
 * @fn      SensorTagDisplay_resume
 *
 * @brief   Reconnects the SPI driver for use by this devices.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagDisplay_resume(void)
{
    Display_control(handle, DISPLAY_CMD_TRANSPORT_OPEN, NULL);
    Display_clear(handle);
    Display_print0(handle, DISPLAY_LINE_DEFAULT, 2, "Display on");
}

/*******************************************************************************
* Private functions
*/

/*******************************************************************************
 * @fn      displayChangeCB
 *
 * @brief   Callback from IO service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void displayChangeCB(uint8_t newParamID)
{
    // Wake up the application thread
    SensorTag_charValueChangeCB(SERVICE_ID_DISPLAY, newParamID);
}
#endif // INCLUDE_DISPLAY

/*******************************************************************************
*******************************************************************************/
