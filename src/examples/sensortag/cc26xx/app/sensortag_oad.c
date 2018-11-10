/******************************************************************************

 @file  sensortag_oad.c

 @brief This file is the SensorTag OAD sub-application.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2016-2018, Texas Instruments Incorporated
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

#ifndef EXCLUDE_OAD
/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "string.h"
#include <ICall.h>
#include "util.h"
#include "sensortag.h"
#include "sensortag_conn_ctrl.h"
#include "sensortag_display.h"
#include "oad_target.h"
#include "oad.h"

/*******************************************************************************
 * CONSTANTS
 */

// Misc.
#define OAD_PACKET_SIZE                       18

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */

static void SensorTag_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                        uint8_t *pData);

/*******************************************************************************
 * PROFILE CALLBACKS
 */

static gapRolesParamUpdateCB_t paramUpdateCB =
{
  SensorTagConnControl_paramUpdateCB
};

static oadTargetCBs_t sensorTag_oadCBs =
{
  (oadWriteCB_t) SensorTag_processOadWriteCB // Write Callback
};


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

/*******************************************************************************
 * @fn      SensorTagOad_init
 *
 * @brief
 * @param   none
 *
 * @return  none
 */
void SensorTagOad_init(void)
{
  // Register connection parameter update
  GAPRole_RegisterAppCBs(&paramUpdateCB);
  SensorTagConnectionControl_init();              // Connection control to
                                                  // support OAD for iOs/Android
  OAD_addService();                               // OAD Profile
  OAD_register((oadTargetCBs_t *)&sensorTag_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
}

/*******************************************************************************
 * @fn      SensorTagOad_processEvent
 *
 * @brief   Process the write events to the OAD characteristics
 *
 * @param   a0, a1 (not used)
 *
 * @return  none
 */
void SensorTagOad_processEvent(void)
{
    while (!Queue_empty(hOadQ))
    {
        oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

        // Identify new image.
        if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
        {
            // Suspend display to allow operations on the external flash
            SensorTagDisplay_suspend();

            // Request image identification
            OAD_imgIdentifyWrite(oadWriteEvt->connHandle,
                                 oadWriteEvt->pData);
        }
        // Write a next block request.
        else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
        {
            OAD_imgBlockWrite(oadWriteEvt->connHandle,
                              oadWriteEvt->pData);
        }

        // Free buffer.
        ICall_free(oadWriteEvt);
    }
}

/*******************************************************************************
 * @fn      SensorTag_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
static void SensorTag_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc(sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if (oadWriteEvt != NULL)
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif // EXCLUDE_OAD

/*******************************************************************************
*******************************************************************************/
