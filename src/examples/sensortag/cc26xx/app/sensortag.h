/******************************************************************************

 @file  sensortag.h

 @brief This file contains the SensorTag application's definitions and
        prototypes.

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

#ifndef SENSORTAG_H
#define SENSORTAG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ICall.h"
#include "peripheral.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>

/*********************************************************************
 * CONSTANTS
 */

// Service ID's for internal application use
#define SERVICE_ID_TMP       0x01
#define SERVICE_ID_OPT       0x02
#define SERVICE_ID_MOV       0x04
#define SERVICE_ID_HUM       0x05
#define SERVICE_ID_BAR       0x06
#define SERVICE_ID_IO        0x07
#define SERVICE_ID_KEYS      0x08
#define SERVICE_ID_CC        0x09
#define SERVICE_ID_DISPLAY   0x0A
#define SERVICE_ID_LIGHT     0x0B
#define SERVICE_ID_REGISTER  0x0C
#define SERVICE_ID_BATT      0x0D

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * VARIABLES
 */
extern ICall_Semaphore sem;
extern gaprole_States_t gapProfileState;
extern ICall_EntityID selfEntityMain;
extern PIN_State pinGpioState;
extern PIN_Handle hGpioPin;


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for SensorTag
 */
extern void SensorTag_createTask(void);

/*
 * Function to call when a characteristic value has changed
 */
extern void SensorTag_charValueChangeCB(uint8_t sensorID, uint8_t paramID);

/*
 * Function to load the factory image and reboot it
 */
extern void SensorTag_applyFactoryImage(void);

/*
 * Update the advertising data with the latest key press status
 */
extern void SensorTag_updateAdvertisingData(uint8_t keyStatus);

/*
 * Return the self-test result
 */
extern uint8_t SensorTag_testResult(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAG_H */
