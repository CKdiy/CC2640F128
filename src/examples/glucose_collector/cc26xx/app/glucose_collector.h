/******************************************************************************

 @file  glucose_collector.h

 @brief This file contains the Glucose Collector sample application definitions
        and prototypes.

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

#ifndef GLUCOSECOLLECTOR_H
#define GLUCOSECOLLECTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Glucose App discovery states
enum
{
  DISC_IDLE = 0x00,                       // Idle state

  DISC_GLUCOSE_START = 0x10,            // Current glucose service
  DISC_GLUCOSE_SVC,                     // Discover service
  DISC_GLUCOSE_CHAR,                    // Discover all characteristics
  DISC_GLUCOSE_CCCD,                    // Discover glucose CCCD
  DISC_GLUCOSE_CONTEXT_CCCD,            // Discover context CCCD
  DISC_GLUCOSE_CTL_PNT_CCCD,            // Discover record control point CCCD

  DISC_DEVINFO_START = 0x20,
  DISC_DEVINFO_SVC,
  DISC_DEVINFO_CHAR,

  DISC_FAILED = 0xFF                      // Discovery failed
};


// Glucose handle cache indexes
enum
{
  HDL_GLUCOSE_START,
  HDL_GLUCOSE_END,
  HDL_GLUCOSE_MEAS_CCCD,
  HDL_GLUCOSE_CONTEXT_START,
  HDL_GLUCOSE_CONTEXT_END,
  HDL_GLUCOSE_CONTEXT_CCCD,
  HDL_GLUCOSE_FEATURE,
  HDL_GLUCOSE_CTL_PNT_START,
  HDL_GLUCOSE_CTL_PNT_END,
  HDL_GLUCOSE_CTL_PNT_CCCD,
  HDL_DEVINFO_SYSTEM_ID,
  HDL_DEVINFO_MODEL_NUM,
  HDL_DEVINFO_MANUFACTURER_NAME,

  HDL_CACHE_LEN
};

// Configuration states
#define GLUCOSE_CONFIG_START                      0x00
#define GLUCOSE_CONFIG_CMPL                       0xFF

// Glucose Collector Task Events
#define GLUCOLL_START_DISCOVERY_EVT               0x0001
#define GLUCOLL_PAIRING_STATE_EVT                 0x0002
#define GLUCOLL_PASSCODE_NEEDED_EVT               0x0004
#define GLUCOLL_RSSI_READ_EVT                     0x0008
#define GLUCOLL_KEY_CHANGE_EVT                    0x0010
#define GLUCOLL_STATE_CHANGE_EVT                  0x0020
#define GLUCOLL_PROCEDURE_TIMEOUT_EVT             0x0040

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task ID
extern uint8 glucCollTaskId;

// Connection handle
extern uint16 glucCollConnHandle;

// Features
extern uint16 glucoseFeatures;

// Handle cache
extern uint16 glucoseHdlCache[HDL_CACHE_LEN];

// control point write in progress
extern bool glucCollWritePending;

// control point clear in progress
extern bool glucCollClearPending;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for Glucose collector.
 */
extern  void glucCollCentral_createTask(void);

/*
 * Glucose service discovery functions
 */
extern uint8 glucoseDiscStart(void);
extern uint8 glucoseDiscGattMsg(uint8 state, gattMsgEvent_t *pMsg);

/*
 * Glucose characteristic configuration functions
 */
extern uint8 glucoseConfigNext(uint8 state);
extern uint8 glucoseConfigGattMsg(uint8 state, gattMsgEvent_t *pMsg);

/*
 * Glucose indication and notification handling functions
 */
extern void glucoseIndGattMsg(gattMsgEvent_t *pMsg);

/*
 * Glucose control point functions
 */
extern uint8 glucoseCtlPntWrite(uint8 opcode, uint8 oper);
extern uint8 glucoseCtlPntWriteFilter(uint8 opcode, uint8 oper,
                                      uint8 filterType, void* param1,
                                      void* param2);
extern void glucoseCtlPntGattMsg(gattMsgEvent_t *pMsg);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GLUCOSECOLLECTOR_H */
