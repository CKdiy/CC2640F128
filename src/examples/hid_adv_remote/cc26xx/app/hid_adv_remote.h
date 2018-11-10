/******************************************************************************

 @file  hid_adv_remote.h

 @brief This file contains the HID Advanced Remote Application header file file
        file.

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

#ifndef HIDADVREMOTE_H
#define HIDADVREMOTE_H

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 ******************************************************************************/

/******************************************************************************
 * CONSTANTS
 ******************************************************************************/


// no key ID
#define KEY_NONE                            0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN             8
#define HID_CC_IN_RPT_LEN                   1

// Time between each repeat key event
#define HAR_KEY_REPEAT_INTERVAL             120

// Time to wait for switch debounce
#define HAR_KEY_DEBOUNCE_TIME               25

// Time for repeating key clock to expire
#define HAR_KEY_POLL_RATE                   20

// HID Consumer Control keycodes
// (based on the HID Report Map characteristic value)
#define HID_CC_RPT_POWER                    1
#define HID_CC_RPT_PLAY_PAUSE               2
#define HID_CC_RPT_STOP                     3
#define HID_CC_RPT_SCAN_NEXT_TRK            4
#define HID_CC_RPT_SCAN_PREV_TRK            5
#define HID_CC_RPT_FAST_FWD                 6
#define HID_CC_RPT_REWIND                   7
#define HID_CC_RPT_RECORD                   8
#define HID_CC_RPT_VOLUME_UP                9
#define HID_CC_RPT_VOLUME_DOWN              10
#define HID_CC_RPT_MUTE                     11

/******************************************************************************
 * MACROS
 ******************************************************************************/
// Delay
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/

/******************************************************************************
 * Task creation function for the HID Advanced Remote
 */
extern void HIDAdvRemote_createTask(void);

/******************************************************************************
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // HIDADVREMOTE_H
