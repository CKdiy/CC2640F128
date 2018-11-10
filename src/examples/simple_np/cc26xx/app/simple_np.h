/******************************************************************************

 @file  simple_np.h

 @brief This file contains the Simple Network processor header application for
        use with the CC2650 Bluetooth Low Energy
        Protocol Stack.

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

#ifndef SIMPLENP_H
#define SIMPLENP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <ti/sysbios/knl/Event.h>
#include "icall.h"
#include "util.h"


#include "npi_data.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/
extern ICall_EntityID snp_selfEntity;

/*********************************************************************
 * CONSTANTS
 */
#define SNP_VERSION                           0x0110  //<! Version 1.1

// Internal Events for RTOS application
#define SNP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SNP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SNP_ALL_EVENTS                        (SNP_ICALL_EVT | SNP_QUEUE_EVT)

#define SNP_STATE_CHANGE_EVT                  0x0001
#define SNP_CHAR_CHANGE_EVT                   0x0002
#define SNP_CHAR_READ_EVT                     0x0004
#define SNP_NEW_AP_MSG_EVT                    0x0010

/*-------------------------------------------------------------------
* TYPEDEFS - Initialization and Configuration
*/

// App event passed from profiles.
typedef struct
{
  uint8_t     event;     // Which profile's event
  uint8_t     status;    // New status
  _npiFrame_t *pNpiMsg;  // Message Received from the Application processor
} snp_Evt_t;

/*********************************************************************
 * MACROS
 */
#ifdef SWO_DEBUG
//! \brief Macro to write the software output debug port
#define ITM_Port32(n) (*((volatile unsigned int *)(CPU_ITM_BASE+4*n)))
#endif

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple Peripheral Network Processor.
 */
extern void SNP_createTask(void);
extern void SNP_init(Event_Handle *event);
extern void SNP_replyToHost_send(uint8_t type, uint16_t opcode, uint8_t *status,
                                 uint16_t sizeParam, uint8_t *param);
extern void SNP_replyToHostValue_send(uint8_t type, uint16_t opcode,
                                      uint8_t *status, uint16_t sizeParam,
                                      uint8_t *param, uint16_t sizeParam2,
                                      uint8_t *param2D);
void   SNP_eventToHost_send(uint16_t event, uint8_t *status, uint16_t sizeParam,
                            uint8_t *param);
extern void SNP_stateChangeCB(uint8_t newState);
void   SNP_enqueueMsg(uint8_t event, uint8_t status);

void SNP_processEvents(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLENP_H */
