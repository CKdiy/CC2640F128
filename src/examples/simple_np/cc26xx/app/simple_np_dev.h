/******************************************************************************

 @file  simple_np_dev.h

 @brief This file contains the parsing of Device related command for the
        Simple BLE Peripheral sample application, for use with the
        CC2650 Bluetooth Low Energy Protocol Stack.

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
/**
@page DEV  API: device related

  @section dev_api Device API summary
   commands Acronyms                              | Origin  | Type        | TL Cmd0                 | TL Cmd1                 | Parameter structure
--------------------------------------------------|---------|-------------|-------------------------|-------------------------|---------------------
   \ref SNP_powerUp_ind                           | NP      | Async       | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_START_ADV_REQ      |  |
   [SNP_maskEvent_req  ] (\ref SNP_maskEvent)     | AP      | Sync Req    | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_MASK_EVT_REQ       | #snpMaskEventReq_t
   [SNP_maskEvent_cnf  ] (\ref SNP_maskEvent)     | NP      | Sync Rsp    | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_MASK_EVENT_RSP     | #snpMaskEventRsp_t
   [SNP_getRevision_req] (\ref SNP_getRevision)   | AP      | Sync Req    | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_GET_REVISION_REQ   |  |
   [SNP_getRevision_rsp] (\ref SNP_getRevision)   | NP      | Sync Rsp    | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_GET_REVISION_RSP   | #snpGetRevisionRsp_t
   [SNP_HCI_req] (\ref SNP_HCI)                   | AP      | Async       | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_HCI_CMD_REQ        | #snpHciCmdReq_t
   [SNP_HCI_rsp] (\ref SNP_HCI)                   | NP      | Async       | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_HCI_CMD_RSP        | #snpHciCmdRsp_t
   [SNP_event_ind] (\ref SNP_Event_ind)           | NP      | Async       | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_EVENT_IND          | #snpEvt_t
   [SNP_getStatus_req] (\ref SNP_getStatus)       | AP      | Sync Req    | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_GET_STATUS_RSP     |  |
   [SNP_getStatus_rsp] (\ref SNP_getStatus)       | NP      | Sync Rsp    | #SNP_NPI_ASYNC_RSP_TYPE | #SNP_TEST_RSP           | #snpGetStatusCmdRsp_t
   [SNP_getRand_req] (\ref SNP_getRand)           | AP      | Sync Req    | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_GET_RAND_REQ       |  |
   [SNP_getRand_rsp] (\ref SNP_getRand)           | AP      | Sync Rsp    | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_GET_RAND_RSP       | #snpGetRandRsp_t
   [SNP_test_req  (debug only)] (\ref SNP_test)   | AP      | Async       | #SNP_NPI_ASYNC_REQ_TYPE | #SNP_TEST_REQ           |  |
   [SNP_test_rsp  (debug only)] (\ref SNP_test)   | NP      | Async       | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_TEST_RSP           |  #snpTestCmdRsp_t
   [invalid cmd] (\ref Sync_error)                | NP      | Sync Rsp    | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_SYNC_ERROR_CMD_IND |  |

  all those command have some parameters. those parameters are can be set using teh indicated packed structure.
  note that all parameter/structure field should be in little-endian format (LSB first).
  for UUID parameters, take a look at @ref UUID_desc

  @section SNP_powerUp_ind SNP power up Indication
  This event is send by the AP once the device has powered up.
  This event will be received if the device reset unexpectedly.
  One reception of this event the AP should consider that the NP lost any previous configuration.

  When this event is received, the following services are initialized and ready to be used:
  - GAP service
  - GATT service
  - Device Info Service.

  This command doesn't have any parameters

  @section SNP_maskEvent SNP mask Event
  This command enable the AP to mask some events. By default, all events are enabled.
  Purpose is to limit the number of possible wake up condition of the AP and reduce power consumption as much as possible.

  All those events can be triggered asynchronously, either due to an action from the AP, or due to an action of the remote peer, or timer expiration.

  This request takes as parameter the structure #snpMaskEventReq_t.

  Possible Events are listed here: @ref SNP_EVENT , @see SNP_Event_ind

  @section SNP_Event_ind SNP Event indication
  This indication is send by the SNP to signal an event.
  Events can be masked by using the SNP_maskEvent API ([masking event] (\ref SNP_maskEvent)) (RFU).
  Only one event will be indicated at a time per packet. if several events occurs, they will each be encapsulated in their own TL packet.

  Each event has a 2-bytes Event type.
  along with this type, some events might have some parameters associated with them.

  events parameters are mapped on the @ref snpEvt_t structure, which is a union of #snpConnEstEvt_t, #snpConnTermEvt_t , #snpUpdateConnParamEvt_t , #snpAdvStatusEvt_t and #snpATTMTUSizeEvt_t

  Possible Events are listed here: @ref SNP_EVENT

  The table bellow list the events and the structure used to MAP the parameter of those events:

   Event                     |  Parameter structure
---------------------------- | --------------------
  SNP_CONN_EST_EVT           | #snpEvt_t + #snpConnEstEvt_t
  SNP_CONN_TERM_EVT          | #snpEvt_t + #snpConnTermEvt_t
  SNP_CONN_PARAM_UPDATED_EVT | #snpEvt_t + #snpUpdateConnParamEvt_t
  SNP_ADV_STARTED_EVT        | #snpEvt_t + #snpAdvStatusEvt_t
  SNP_ADV_ENDED_EVT          | #snpEvt_t + #snpAdvStatusEvt_t
  SNP_ATT_MTU_EVT            | #snpEvt_t + #snpATTMTUSizeEvt_t
  SNP_ERROR_EVT              | #snpEvt_t + #snpConnEstEvt_t

  @subsection SNP_event_MTU about ATT MTU size
  The default ATT_MTU size is set to 23 bytes. this implies that no fragmentation is done at HCI level.
  The GATT client can request an ATT_MTU_EXCHANGE method to change the maximum possible ATT MTU size.
  The SNP is configure to manage ATT MTU size up to 251 Bytes.
  If this update occurs, the corresponding event will be send by the SNP. if this event is not received, the AP must assume that ATT MTU size is 23.

  @subsection SNP_event_Error about the error event indication
  This error event occurs if something unexpected occurs while SNP is running some operation.
  The list of all possible SNP errors can be found here \ref SNP_ERRORS

  @section SNP_getRevision Get SNP Revision Command
  This is use to get the current revision of the SNP API. Along as the full stack revision number as define in the HCI vendor guide.

  SNP_getRevision_req does not take any parameters.

  SNP_getRevision_rsp parameters are mapped on structure #snpGetRevisionRsp_t.

  @section SNP_getStatus  Get SNP Status Command
  This is use to get the current status of the SNP.

  SNP_getStatus_req does not take any parameters.

  SNP_getStatus_rsp parameters are mapped on structure #snpGetStatusCmdRsp_t.

  @section SNP_getRand  Get Random Number Command
  This is use to get a 32 bit random number generated by the SNP TRNG.

  SNP_getRand_req does not take any parameters.

  SNP_getRand_rsp parameters are mapped on structure #snpGetRandRsp_t.

  @section SNP_test   SNP test Command
  Return the heap usage of the SNP.
  DEBUG use only, this command may not be present in the final release.

  SNP_test_req does not take any parameters.

  SNP_test_rsp parameters are mapped on structure #snpTestCmdRsp_t.

  @section SNP_HCI   SNP encapsulated HCI Command
  This is sending a HCI command. Only the HCI command listed [here] (\ref SNP_ALLOWED_HCI) are supported:

  This SNP_HCI_req command takes the 2 bytes HCI opcode, as define in the TI HCI vendor guide.
  Then the parameters as defined for this command in the TI HCI vendor guide.
  Those parameters can be mapped on the structure #snpHciCmdReq_t

  SNP_test_rsp parameters are mapped on structure #snpHciCmdRsp_t.

  @section Sync_error   SNP synchronous invalid command indication
  This indication packet will be send over TL if a unknown synchronous packet is send to the TL.
  An unknown command send as a synchronous packet will prevent any other command to be send.
  Therefore this indication is send back.

  This indication has no parameter.

  another asynchronous error event will also be send to signal this error, it will have the opcode of the faulting command as a parameter. see \ref SNP_Event_ind.

*/



#ifndef SIMPLENP_DEV_H
#define SIMPLENP_DEV_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/**********************************************************************
* TYPEDEFS - Initialization and Configuration
*/

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/**
 *  @fn          SNP_getRev
 *
 *  @brief       send back to the revision of the SNP.
 *
 *  @param[in]   None
 *  @param[out]  Msg      : structure containing the response
 *
 *  @return  None.
 */
void SNP_getRev(snpGetRevisionRsp_t *Msg);

/**
 *  @fn          SNP_getRand
 *
 *  @brief       send to the AP a random number generated by TRNG.
 *
 *  @param[in]   None
 *  @param[out]  pRsp      : structure containing the response
 *
 *  @return  None.
 */
void SNP_getRand(snpGetRandRsp_t *pRsp);

/**
 *  @fn          SNP_maskEvt
 *
 * @brief        mask event, prevent them to be send to the AP when they occurred..
 *
 *  @param[in]   pReq      : Msg from NPI
 *  @param[out]  pRsp      : structure containing the response
 *
 *  @return  None.
 */
void SNP_maskEvt(snpMaskEventReq_t* pReq, snpMaskEventRsp_t *pRsp);

/**
 *  @fn          SNP_getStatus
 *
 * @brief        send back to the status of the SNP.
 *
 *  @param[in]   None
 *  @param[out]  pRsp : structure containing the response.
 *
 *  @return  None.
 */
void SNP_getStatus(snpGetStatusCmdRsp_t *pRsp);

/**
 *  @fn          SNP_executeTestCmd
 *
 * @brief        send back to the AP the HEAP usage.
 *
 *  @param[in]   None
 *  @param[out]  pRsp : structure containing the response.
 *
 *  @return  None.
 */
void SNP_executeTestCmd(snpTestCmdRsp_t *pRsp);

/**
 *  @fn          SNP_executeHCIcmd
 *
 *  @brief       call a HCI command
 *
 *  @param[in]   pReq     :HCI request structure, see #snpHciCmdReq_t
 *  @param[in]   dataLen   :Len of variable length parameter field in Bytes
 *  @param[out]  None
 *
 *  @return  status of the command.
 */
uint8_t SNP_executeHCIcmd(snpHciCmdReq_t *pReq, uint16_t dataLen);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLENP_DEV_H */
