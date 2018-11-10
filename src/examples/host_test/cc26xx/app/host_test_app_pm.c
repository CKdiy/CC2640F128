/******************************************************************************

 @file  host_test_app_pm.c

 @brief This file contains the HostTest sample application for use with the
        CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2013-2018, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/UART.h> //For UART application

#include <icall.h>
#include <string.h>

#include "hal_types.h"
#include "hci_tl.h"
#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#include "gapbondmgr.h"
#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include <ti/mw/display/Display.h>

#include "npi_pm.h"
#include <inc/hw_types.h>
#include "hal_defs.h"

#if defined ( GATT_TEST ) || defined ( GATT_QUAL )
  #include "gatttest.h"
#endif

#include "host_test_app.h"

/*********************************************************************
 * CONSTANTS
 */

// LE Event Lengths
#define HCI_CMD_COMPLETE_EVENT_LEN              3
#define HCI_CMD_VS_COMPLETE_EVENT_LEN           2
#define HCI_CMD_STATUS_EVENT_LEN                4

// Task configuration
#define HTA_TASK_PRIORITY                       1
#define HTA_TASK_STACK_SIZE                     644

// NPI Events
#define NPI_WRITECALLBACK_EVENT                 0x0002
#define NPI_READ_EVENT                          0x0004
#define NPI_MRDY_EVENT                          0x0008

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for UART TX messages
static Queue_Struct uartTx;
static Queue_Handle uartTxQueue;

// Task configuration
Task_Struct htaTask;
Char htaTaskStack[HTA_TASK_STACK_SIZE];

#if !defined ( GATT_DB_OFF_CHIP )
  static uint8 deviceName[GAP_DEVICE_NAME_LEN] = { 0 };
  static uint16 appearance = 17;
#endif

// Pointer to last transmitted message
npiPkt_t *pMsgLastBuf = NULL;

// events for application
static uint16_t events = NPI_WRITECALLBACK_EVENT;

// Stack build revision
buildRevision_t buildRev;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void HostTestApp_init(void);
static void HostTestApp_taskFxn(UArg a0, UArg a1);

static void HostTestApp_processGapEvent(ICall_HciExtEvt *pMsg);

//RTOS queue functions
static void HostTestApp_sendMsg(npiPkt_t *pMsgBuf);

//UART callback
Void HostTestApp_writeCallBack(int size);
Void HostTestApp_readCallBack(int size);

static void sendCommandCompleteEvent(uint8 eventCode, uint16 opcode,
                                     uint8 numParam, uint8 *param);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HostTestApp_createTask
 *
 * @brief   Task creation function for the Host Test App.
 *
 * @param   none
 *
 * @return  none
 */
void HostTestApp_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = htaTaskStack;
  taskParams.stackSize = HTA_TASK_STACK_SIZE;
  taskParams.priority = HTA_TASK_PRIORITY;

  Task_construct(&htaTask, HostTestApp_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HostTestApp_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   none
 *
 * @return  none
 */
static void HostTestApp_init(void)
{
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);

  // Create an RTOS queue for messages from stack to be sent to UART
  uartTxQueue = Util_constructQueue(&uartTx);

  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Register for unwanted HCI messages
  GAP_RegisterForMsgs(selfEntity);

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Get build revision
  VOID Util_buildRevision(&buildRev);

#if !defined ( GATT_DB_OFF_CHIP )

  #if defined ( GATT_QUAL )
    VOID GATTQual_AddService( GATT_ALL_SERVICES ); // Includes GAP and GATT Services
  #else
    // Add our services to GATT Server
    VOID GGS_AddService( GATT_ALL_SERVICES );
    VOID GATTServApp_AddService( GATT_ALL_SERVICES );
    #if defined ( GATT_TEST )
      VOID GATTTest_AddService( GATT_ALL_SERVICES );
    #endif
  #endif

  // Set device name
  if ((buildRev.hostInfo & CENTRAL_CFG) && (buildRev.hostInfo & PERIPHERAL_CFG))
  {
    memcpy(deviceName, "TI BLE All", 10);
  }
  else if (buildRev.hostInfo & CENTRAL_CFG)
  {
    memcpy(deviceName, "TI BLE Central", 14);
  }
  else if (buildRev.hostInfo & PERIPHERAL_CFG)
  {
    memcpy(deviceName, "TI BLE Peripheral",  17);
  }
  else
  {
    memcpy(deviceName, "TI BLE Unknown",  14);
  }

  VOID GGS_SetParameter(GGS_DEVICE_NAME_ATT, strlen((char *)deviceName), deviceName);
  VOID GGS_SetParameter(GGS_APPEARANCE_ATT, sizeof(uint16), (void*)&appearance);

#endif // GATT_DB_OFF_CHIP

#ifdef TI_DRIVERS_UART_INCLUDED
  //Initialize the NPI layer
  NPI_InitInterface(UART_INTERFACE, &HostTestApp_writeCallBack,
                    &HostTestApp_readCallBack);
#elif TI_DRIVERS_SPI_DMA_INCLUDED
  //Initialize the NPI layer
  NPI_InitInterface(SPI_INTERFACE, &HostTestApp_writeCallBack,
                    &HostTestApp_readCallBack);
#endif

  //Register application with NPI for stack messages to be sent to UART
  NPI_RegisterTask(selfEntity);

  Display_print0(dispHandle, 0, 0, "TI BLEv2.0");
  Display_print0(dispHandle, 1, 0, "HostTestApp");

  // Display Host build configuration
  if ((buildRev.hostInfo & CENTRAL_CFG) && (buildRev.hostInfo & PERIPHERAL_CFG))
  {
    Display_print0(dispHandle, 2, 0, "All");
  }
  else if ((buildRev.hostInfo & CENTRAL_CFG) &&
           (buildRev.hostInfo & BROADCASTER_CFG))
  {
    Display_print0(dispHandle, 2, 0, "Cent+Bcast");
  }
  else if ((buildRev.hostInfo & PERIPHERAL_CFG) &&
           (buildRev.hostInfo & OBSERVER_CFG))
  {
    Display_print0(dispHandle, 2, 0, "Peri+Observ");
  }
  else if (buildRev.hostInfo & CENTRAL_CFG)
  {
    Display_print0(dispHandle, 2, 0, "Central");
  }
  else if (buildRev.hostInfo & PERIPHERAL_CFG)
  {
    Display_print0(dispHandle, 2, 0, "Peripheral");
  }
  else
  {
    Display_print1(dispHandle, 2, 0, "Unknown build cfg %d", buildRev.hostInfo);
  }
}

/*********************************************************************
 * @fn      HostTestApp_taskFxn
 *
 * @brief   Application task entry point for the Host Test App.
 *
 * @param   none
 *
 * @return  none
 */
static void HostTestApp_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  HostTestApp_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        bool dealloc = true;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process incoming messages
          switch (pMsg->hdr.event)
          {
            case HCI_GAP_EVENT_EVENT:
              HostTestApp_processGapEvent(pMsg);
              break;

            case HCI_ACL_DATA_PACKET:
            case HCI_SCO_DATA_PACKET:
            case HCI_EVENT_PACKET:
              {
                HostTestApp_sendMsg((npiPkt_t *)pMsg);
                dealloc = false;
              }
              break;

            default:
              break;
          }
        }

        if (dealloc == true)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    if (events & NPI_READ_EVENT )
    {
      events &= ~NPI_READ_EVENT;
      NPI_serialPacketParser(); //Call the parser..
    }

    if (events & NPI_MRDY_EVENT )
    {
      // This read is initiated to read again and again till you run out of
      // bytes. This read was initiated in the callback but is actually done in
      // this application thread
      events &= ~NPI_MRDY_EVENT;
#ifdef POWER_SAVING
      NPI_MrdyEvent();
#endif
    }

    // Check if write is complete and free the last written buffer
    if(events & NPI_WRITECALLBACK_EVENT)
    {
      if (pMsgLastBuf)
      {
        ICall_freeMsg(pMsgLastBuf);  //Free the NPI/OSAL buffer which was sent
        pMsgLastBuf = NULL;          //Need to have this to avoid memory leak
      }

      // If queue is not empty, we did add some frames to it
      if (!Queue_empty(uartTxQueue))
      {
        pMsgLastBuf = (npiPkt_t *)Util_dequeueMsg(uartTxQueue);
        if (pMsgLastBuf)
        {
          // send dequeued element to UART
          events &= ~NPI_WRITECALLBACK_EVENT;
          NPI_WriteTransport(pMsgLastBuf->pData, pMsgLastBuf->pktLen);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      HostTestApp_processGapEvent
 *
 * @brief   Process an incoming GAP Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HostTestApp_processGapEvent(ICall_HciExtEvt *pMsg)
{
  switch(pMsg->hdr.status)
  {
    case HCI_COMMAND_COMPLETE_EVENT_CODE:
      {
        hciEvt_CmdComplete_t *pkt = (hciEvt_CmdComplete_t *)pMsg;

        if (lastAppOpcodeSent == pkt->cmdOpcode)
        {
          // app processes this as it was embedded msg to stack

          // Reset last opcode sent
          lastAppOpcodeSent = 0xFFFF;
        }
        else
        {
          osal_msg_hdr_t *msgHdr;
          uint8 len;

          msgHdr = (osal_msg_hdr_t *)pMsg;
          msgHdr--; // Backup to the msg header

          len = (uint8)(msgHdr->len - sizeof ( hciEvt_CmdComplete_t ));

          sendCommandCompleteEvent(HCI_COMMAND_COMPLETE_EVENT_CODE,
                                   pkt->cmdOpcode, len, pkt->pReturnParam);
        }
      }
      break;

    case HCI_DISCONNECTION_COMPLETE_EVENT_CODE:
      break;

    case HCI_COMMAND_STATUS_EVENT_CODE:
      {
        hciEvt_CommandStatus_t *pkt = (hciEvt_CommandStatus_t *)pMsg;

        if (lastAppOpcodeSent == pkt->cmdOpcode)
        {
          // app processes this as it was embedded msg to stack

          // Reset last opcode sent
          lastAppOpcodeSent = 0xFFFF;
        }
        // else - drop it for now
      }
      break;

    case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
      {
        AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
      }
      break;

    case HCI_LE_EVENT_CODE:
      break;

    case HCI_VE_EVENT_CODE:
      {
        hciEvt_VSCmdComplete_t *pkt = (hciEvt_VSCmdComplete_t *)pMsg;

        if (lastAppOpcodeSent == pkt->cmdOpcode)
        {
          // app processes this as it was embedded msg to stack

          // Reset last opcode sent
          lastAppOpcodeSent = 0xFFFF;
        }
        else
        {
          sendCommandCompleteEvent(HCI_VE_EVENT_CODE, pkt->cmdOpcode,
                                   pkt->length, pkt->pEventParam);
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      HostTestApp_writeCallBack
 *
 * @brief   Callback for TX ISR
 *
 * @param   none
 *
 * @return  none
 */
Void HostTestApp_writeCallBack(int size)
{
  events |= NPI_WRITECALLBACK_EVENT;
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HostTestApp_readCallBack
 *
 * @brief   Callback for RX ISR
 *
 * @param   none
 *
 * @return  none
 */
Void HostTestApp_readCallBack(int size)
{
  if ( size == 0xFF )
  {
    events |= NPI_MRDY_EVENT;
  }
  else
  {
    events |= NPI_READ_EVENT;
  }

  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HostTestApp_sendMsg
 *
 * @brief   Sends packet to UART else queues it if not able to do so
 *
 * @param   none
 *
 * @return  none
 */
static void HostTestApp_sendMsg(npiPkt_t *pMsgBuf)
{
  if ((events & NPI_WRITECALLBACK_EVENT) && (Queue_empty(uartTxQueue)))
  {
    events &= ~NPI_WRITECALLBACK_EVENT;

    if (pMsgLastBuf)
    {
      ICall_freeMsg(pMsgLastBuf);  //Free the NPI/OSAL buffer which was sent
      pMsgLastBuf = NULL;          //Need to have this to avoid memory leak
    }

    pMsgLastBuf = pMsgBuf;
    NPI_WriteTransport(pMsgBuf->pData, pMsgBuf->pktLen);
    return;
  }

  // Enqueue the message
  Util_enqueueMsg(uartTxQueue, sem, (uint8*)pMsgBuf);
}

//*****************************************************************************
// the function prototypes

/*******************************************************************************
 * This generic function sends a Command Complete or a Vendor Specific Command
 * Complete Event to the Host.
 *
 */
static void sendCommandCompleteEvent(uint8_t eventCode, uint16_t opcode,
                                     uint8_t numParam, uint8_t *param)
{
  npiPkt_t *msg;
  uint8_t   totalLength;
  uint8_t   txLen = 0; // Length to transmit

  // The initial length will be:
  // OSAL message header(4) - not part of packet sent to HCI Host!
  // Minimum Event Data: Packet Type(1) + Event Code(1) + Length(1)
  // Return Parameters (0..N)
  totalLength = sizeof(npiPkt_t) + HCI_EVENT_MIN_LENGTH + numParam;

  // adjust the size of the event packet based on event code
  // Note: If not a vendor specific event, then the event includes:
  //       Command Complete Data: Number of HCI Commands Allowed(1) + Command Opcode(2)
  // Note: If a vendor specific event, then the event includes:
  //       Vendor Specific Command Complete Data: Vendor Specific Event Opcode(2)
  totalLength += ( (eventCode != HCI_VE_EVENT_CODE)  ?
                   HCI_CMD_COMPLETE_EVENT_LEN        :
                   HCI_CMD_VS_COMPLETE_EVENT_LEN );

  // allocate memory for OSAL hdr + packet
  msg = (npiPkt_t *)ICall_allocMsg(totalLength);
  if (msg)
  {
    // OSAL message event, status, and pointer to packet
    msg->hdr.event  = HCI_EVENT_PACKET;
    msg->hdr.status = 0xFF;
    msg->pData      = (uint8*)(msg+1);

    // fill in Command Complete Event data
    msg->pData[0] = HCI_EVENT_PACKET;
    msg->pData[1] = eventCode;

    txLen += 2;

    // check if this isn't a vendor specific event
    if ( eventCode != HCI_VE_EVENT_CODE )
    {
      msg->pData[2] = numParam + HCI_CMD_COMPLETE_EVENT_LEN;
      msg->pData[3] = 1;// hciCtrlCmdToken;     // event parameter 1
      msg->pData[4] = LO_UINT16( opcode ); // event parameter 2
      msg->pData[5] = HI_UINT16( opcode ); // event parameter 2

      txLen += 4;

      // remaining event parameters
      (void)memcpy(&msg->pData[6], param, numParam);

      txLen += numParam;
    }
    else // it is a vendor specific event
    {
      // less one byte as number of complete packets not used in vendor specific event
      msg->pData[2] = numParam + HCI_CMD_VS_COMPLETE_EVENT_LEN;
      msg->pData[3] = param[0];            // event parameter 0: event opcode LSB
      msg->pData[4] = param[1];            // event parameter 1: event opcode MSB
      msg->pData[5] = param[2];            // event parameter 2: status
      msg->pData[6] = LO_UINT16( opcode ); // event parameter 3: command opcode LSB
      msg->pData[7] = HI_UINT16( opcode ); // event parameter 3: command opcode MSB

      txLen += 6;

      // remaining event parameters
      // Note: The event opcode and status were already placed in the msg packet.
      (void)memcpy(&msg->pData[8], &param[3], numParam-HCI_EVENT_MIN_LENGTH);

      txLen += (numParam-HCI_EVENT_MIN_LENGTH);
    }

    msg->pktLen = txLen;

    HostTestApp_sendMsg((npiPkt_t *)msg);
  }
}

/*******************************************************************************
 * This generic function sends a Command Complete or a Vendor Specific Command
 * Complete Event to the Host.
 *
 */
void sendCommandStatusEvent(uint8_t eventCode, uint16_t status, uint16_t opcode)
{
  npiPkt_t *msg;
  uint8_t   totalLength;

  // The initial length will be:
  // OSAL message header(4) - not part of packet sent to HCI Host!
  // Minimum Event Data: Packet Type(1) + Event Code(1) + Length(1)
  // Command Status Event Data: Status (1) + Num HCI Cmd Pkt (1) + Cmd Opcode (2)
  totalLength = sizeof(npiPkt_t)     +
                HCI_EVENT_MIN_LENGTH +
                HCI_CMD_STATUS_EVENT_LEN;

  // allocate memory for OSAL hdr + packet
  msg = (npiPkt_t *)ICall_allocMsg(totalLength);
  if (msg)
  {
    // OSAL message event, status, and pointer to packet
    msg->hdr.event  = HCI_EVENT_PACKET;
    msg->hdr.status = 0xFF;

    // fill in length and data pointer
    msg->pktLen = HCI_EVENT_MIN_LENGTH + HCI_CMD_STATUS_EVENT_LEN;
    msg->pData  = (uint8*)(msg+1);

    // fill in Command Complete Event data
    msg->pData[0] = HCI_EVENT_PACKET;
    msg->pData[1] = eventCode;
    msg->pData[2] = HCI_CMD_STATUS_EVENT_LEN;
    msg->pData[3] = status;
    msg->pData[4] = 1;                 // number of HCI command packets
    msg->pData[5] = LO_UINT16(opcode); // opcode (LSB)
    msg->pData[6] = HI_UINT16(opcode); // opcode (MSB)

    HostTestApp_sendMsg((npiPkt_t *)msg);
  }
}

/*********************************************************************
*********************************************************************/
