/******************************************************************************

 @file  simple_np_gap.h

 @brief This file contains the parsing of GAP related command for the
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
@page GAP  API: GAP related

 @section gap_api GAP API summary

 commands Acronyms                                        | Origin | Type     | TL Cmd0                 | TL Cmd1                          | Parameter structure
--------------------------------------------------------- | ------ |--------- |------------------------ |--------------------------------- | --------------------
 [SNP_startAdv_req]   (\ref SNP_startAdv)                 | AP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_POWER_UP_IND                | #snpStartAdvReq_t
 [SNP_setAdvData_req] (\ref SNP_setAdvData )              | AP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_SET_ADV_DATA_REQ            | #snpSetAdvDataReq_t
 [SNP_setAdvData_rsp] (\ref SNP_setAdvData )              | NP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_SET_ADV_DATA_CNF            | #snpSetAdvDataCnf_t
 [SNP_stopAdv_req]    (\ref SNP_stopAdv)                  | AP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_STOP_ADV_REQ                |  |
 [SNP_updateConnParam_req] (\ref SNP_updateConnParam )    | AP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_UPDATE_CONN_PARAM_REQ       | #snpUpdateConnParamReq_t
 [SNP_updateConnParam_rsp] (\ref SNP_updateConnParam )    | NP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_UPDATE_CONN_PARAM_CNF       | #snpUpdateConnParamCnf_t
 [SNP_terminateConn_req]   (\ref SNP_terminateConn)       | AP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_TERMINATE_CONN_REQ          | #snpTermConnReq_t
 [SNP_setGAPparam_req] (\ref SNP_setGAPparam )            | AP     | Sync Req | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_SET_GAP_PARAM_REQ           | #snpSetGapParamReq_t
 [SNP_setGAPparam_rsp] (\ref SNP_setGAPparam )            | NP     | Sync Rsp | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_SET_GAP_PARAM_REQ           | #snpSetGapParamRsp_t
 [SNP_getGAPparam_req] (\ref SNP_getGAPparam )            | AP     | Sync Req | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_GET_GAP_PARAM_REQ           | #snpGetGapParamReq_t
 [SNP_getGAPparam_rsp] (\ref SNP_getGAPparam )            | NP     | Sync Rsp | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_GET_GAP_PARAM_REQ           | #snpGetGapParamRsp_t
 [SNP_setSecurityParams_req] (\ref SNP_setSecurityParam ) | AP     | Sync Req | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_SET_SECURITY_PARAM_REQ      | #snpSetSecParamReq_t
 [SNP_setSecurityParams_rsp] (\ref SNP_setSecurityParam ) | NP     | Sync Rsp | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_SET_SECURITY_PARAM_REQ      | #snpSetSecParamRsp_t
 [SNP_processSetWhiteList_req] (\ref SNP_setWhiteList )   | AP     | Sync Req | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_SET_WHITE_LIST_POLICY_REQ   | #snpSetWhiteListReq_t
 [SNP_processSetWhiteList_rsp] (\ref SNP_setWhiteList )   | NP     | Sync Rsp | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_SET_WHITE_LIST_POLICY_REQ   | #snpSetWhiteListRsp_t
 [SNP_sendSecurityRequest_req] (\ref SNP_sendSecurityReq) | AP     | Async    | #SNP_NPI_ASYNC_CMD_TYPE | #SNP_SEND_SECURITY_REQUEST_REQ   |  |
 [SNP_setAuthenticationData_req] (\ref SNP_setAuthData)   | AP     | Sync Req | #SNP_NPI_SYNC_REQ_TYPE  | #SNP_SET_AUTHENTICATION_DATA_REQ | #snpSetAuthDataReq_t
 [SNP_setAuthenticationData_rsp] (\ref SNP_setAuthData)   | NP     | Sync Rsp | #SNP_NPI_SYNC_RSP_TYPE  | #SNP_SET_AUTHENTICATION_DATA_REQ | #snpSetAuthDataRsp_t
all those command have some parameters. those parameters are can be set using the indicated packed structure.
note that all parameter/structure field should be in little-endian format (LSB first).
for UUID parameters, take a look at @ref UUID_desc


  @section SNP_startAdv Start Advertisement
    Start advertising on all 3 channels, every 'interval' , for 'timeout' .

   This request takes as parameter the structure #snpStartAdvReq_t.

   if a timeout value different than 0 is used, then the following GAP timer will be update :
       - TGAP_GEN_DISC_ADV_INT_MIN : general mode advertisement duration.
       - TGAP_LIM_ADV_TIMEOUT: limited mode advertisement duration.

   if a timeout value equal 0 is used, the value of the GAP timer will be used:
       - TGAP_GEN_DISC_ADV_INT_MIN : default 0ms, means infinite general mode advertisement duration.
       - TGAP_LIM_ADV_TIMEOUT: default 180s, limited mode advertisement duration.
   if those value has been change using the  @ref SNP_setGAPparam  request, then set value will be used.

   if a interval value different than 0 is used, then the following GAP parameter will be update :
       - TGAP_GEN_DISC_ADV_INT_MIN and TGAP_GEN_DISC_ADV_INT_MAX : general mode advertisement duration.
       - TGAP_LIM_DISC_ADV_INT_MIN and TGAP_LIM_DISC_ADV_INT_MAX: limited mode advertisement interval.

   if a interval value equal 0 is used, the value of the GAP parameter will be used:
       - TGAP_GEN_DISC_ADV_INT_MIN and TGAP_GEN_DISC_ADV_INT_MAX : default 100ms, means infinite general mode advertisement duration.
       - TGAP_LIM_DISC_ADV_INT_MIN and TGAP_LIM_DISC_ADV_INT_MAX: default 100ms, limited mode advertisement duration.
   if those value has been change using the  @ref SNP_setGAPparam  request, then set value will be used.

   The timeout is in ms.
   The interval in in multiple of 625us, meaning an interval value of 160 will lead to a advertisement every 100ms.
   Since for now the SNP support only one connection, advertisement done while in a connection will only be non-connectable advertisement.

   Also, when the connection is terminated, if bit 2 of snpStartAdvReq_t::behavior field is set, advertisement will start automatically, even if it was not adv during the connection, or the advertisement was explicitly stopped before.

   When a connection is established, if bit 1 of snpStartAdvReq_t::behavior field is set, advertisement will continue automatically.
   Interval of advertising during a connection is ruled by another set of parameter: TGAP_CONN_ADV_INT_MIN and TGAP_CONN_ADV_INT_MAX.
   By default, those parameters are set to 1280ms. they can be change by using the @ref SNP_setGAPparam  request.

   When advertisement is starting, an event is send by the SNP (@ref SNP_Event_ind).

   Limitation:
    - Advertisement is always performed on all 3 advertisement channels.
    - The address type is always static (no private or public address).
    - The advertisement cannot be directed to a particular address.
    - In order to change the type of an ongoing advertisement, it must be stopped first.

    - This event is send by the AP once the device has powered up.
    - This event will be received if the device reset unexpectedly.
    - One reception of this event the AP should consider that the NP lost any previous configuration.

 @section SNP_setAdvData Set Advertisement Data
    update the raw information data of either the scan response or the advertisement information.

    There are 2 buffers for the advertisement data:
      - one for the non-connected state (device is NOT in a connection)
      - one the the connected state (device is in a connection)

    When NOT in a connection, if advertisement is requested, the advertisement data store in the non-connected state buffer will be advertise.

    When in a connection, if advertisement is requested, the advertisement data store in the connected state buffer will be advertise.
    If the connected state buffer has not been set (empty), then the adv data of the non-connected mode will be used automatically.
    This way, if the user does not care about differentiating adv data in a connected mode or in a non connected mode,
    he does not have to set data for the connected mode.

    The SNP_setAdvData_req request takes as parameter the structure #snpSetAdvDataReq_t .

    The SNP_setAdvData_rsp response uses as parameter the structure #snpSetAdvDataCnf_t .

    The maximum advertisement size is for now 31 Bytes.

    The default advertising data is the following value  :
    \snippet simpleNP_GAP.c Default adv data

    The default scanning response data is the following value  :
    \snippet simpleNP_GAP.c Default Scan rsp data

    Limitation:
    No advertisement filters policy available for now.


 @section SNP_stopAdv Stop Advertisement
   Stop advertising.
    When advertisement stop, an event is send by the SNP (@ref SNP_Event_ind).

    This request doesn't take any parameters.

 @section SNP_updateConnParam Update connection parameters
   Update the connection parameters.
   A confirmation packet will be reply to this request: SNP_updateConnParam_Rsp.
   This rsp will return the status of the request: ble_pending, SNP_FAILURE, invalid range.
   Changing the connection parameters is performed (or not) by the master.
   Once the parameters has been updated and are active, an event will be send by the SNP (SNP_event_ind)

   This SNP_updateConnParam_req request takes as parameter the structure #snpUpdateConnParamReq_t .

   The SNP_updateConnParam_req response uses as parameter the structure #snpUpdateConnParamCnf_t .

 @section SNP_terminateConn Terminate Connection
   Terminate any ongoing connection.
   When advertisement stop, an event is send by the SNP (@ref SNP_Event_ind).

   This request takes as parameter the structure #snpTermConnReq_t.

   Limitation:
   Only one connection is enabled for first release. Therefore the Handle parameter will not be used by the SNP. For API consistency this parameter still needs to be present in the request.

 @section SNP_setGAPparam Set GAP parameters
    This API enables the update of GAP parameters.
    Available GAP parameters are listed in TI HCI vendor Guide, chapter 12.17.


    The SNP_setGAPparam_req request takes as parameter the structure #snpSetGapParamReq_t .

    The SNP_setGAPparam_rsp response uses as parameter the structure #snpSetGapParamRsp_t .

 @section SNP_getGAPparam Get GAP parameters
    This API enables the read GAP parameters.
    Available GAP parameters are listed in TI HCI vendor Guide, chapter 12.17.

    The SNP_getGAPparam_req request takes as parameter the structure #snpGetGapParamReq_t .

    The SNP_getGAPparam_rsp response uses as parameter the structure #snpGetGapParamRsp_t .

@section SNP_setSecurityParam Set Security parameters
    This API enables the configuration of Security parameters.

    The SNP_setSecurityParams_req takes as parameter the structure #snpSetSecParamReq_t .

    The SNP_setSecurityParams_rsp takes as parameter the structure #snpSetSecParamRsp_t .

@section SNP_setWhiteList set White List Filter Policy
    This API enables configuration of the White List Filter Policy.

    The SNP_processSetWhiteList_req takes as parameter the structure #snpSetWhiteListReq_t .

    The SNP_processSetWhiteList_rsp takes as parameter the structure #snpSetWhiteListRsp_t .

@section SNP_sendSecurityRequest_req Send Security Request
    This API sends a Security Request to the remote connected device.  As a Peripheral device, this sends a Slave Security Request.

    The SNP_sendSecurityRequest_req takes no parameters.

@section SNP_setAuthData Set Authentication Data
    This API sets the Authentication data input needed during a Security Pairing process which uses MITM.

    The SNP_setAuthenticationData_req takes as parameter the structure #snpSetAuthDataReq_t .

    the SNP_setAuthenticationData_rsp takes as parameter the structure #snpSetAuthDataReq_t .

 */
#ifndef SIMPLEBLEPROCESSOR_GAP_H
#define SIMPLEBLEPROCESSOR_GAP_H

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

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @defgroup SNP_GAP_API SPNP GAP Command Functions
 *
 * @{
 */

/*-------------------------------------------------------------------
 * FUNCTIONS - Initialization and Configuration
 */


/**
 *  @brief Initialize SPNP GAP internal state
 *
 *
 */

/**
 *  @fn         SNP_startAdv
 *
 *  @brief      start advertising
 *
 *  @param[in]  *cmdStartAdv :advertising control parameter structure
 *
 *  @return     status of the command.
 */
uint8_t SNP_startAdv(snpStartAdvReq_t* cmdStartAdv);

/**
 *  @fn      SNP_stopAdv
 *
 *  @brief   stop advertising
 *
 *  @return  status of the command.
 */
uint8_t SNP_stopAdv( void );

/**
 *  @fn         SNP_setAdvData
 *
 *  @brief      set advertising data
 *
 *  @param[in]  *cmdData :advertising control parameter structure
 *
 *  @return     status of the command.
 */
uint8_t SNP_setAdvData(snpSetAdvDataReq_t *cmdData, uint8_t len);
/**
 *  @fn         SNP_updateConnParam
 *
 *  @brief      set advertising data
 *
 *  @param[in]  *cmdData :connection parameter structure
 *
 *  @return     status of the command.
 */
uint8_t SNP_updateConnParam(snpUpdateConnParamReq_t *cmdData);

/**
 *  @fn         SNP_terminateConn
 *
 *  @brief      Terminate an ongoing connection
 *
 *  @param[in]  connHandle :handle of the connection to terminate
 *  @param[in]  option     :option of the termination
 *
 *  @return     status of the command.
 */
uint8_t SNP_terminateConn( snpTermConnReq_t* cmdStruct);

/**
 *  @fn         SNP_setGapParam
 *
 *  @brief      set a GAP parameter
 *
 *  @param[in]  cmdStruct :parameters for the command, #snpSetGapParamReq_t
 *
 *  @return     status of the command.
 */
uint8_t SNP_setGapParam( snpSetGapParamReq_t* cmdStruct);

/**
 *  @fn         SNP_getGapParam
 *
 *  @brief      get a GAP parameter
 *
 *  @param[in]  cmdStruct :parameters for the command, #snpGetGapParamReq_t
 *
 *  @return     status of the command.
 */
uint8_t SNP_getGapParam( snpGetGapParamReq_t* cmdStruct);

/**
 *  @fn         SNP_setSecurityParams
 *
 *  @brief      Set Security Parameters
 *
 *  @param[in]  pReq :Security parameter to set, #snpSetSecParamReq_t
 *
 *  @return     status of the command.
 */
uint8_t SNP_setSecurityParams(snpSetSecParamReq_t *pReq);

/**
*  @fn         SNP_setWhiteListFilterPolicy
*
*  @brief      Set White List Filter Policy
*
*  @param[in]  pReq :new Filter Policy
*
*  @return     status of the command.
*/
uint8_t SNP_setWhiteListFilterPolicy(snpSetWhiteListReq_t *pReq);

/**
*  @fn         SNP_sendSecurityRequest
*
*  @brief      Send a Security Request
*
*  @param[in]  none
*
*  @return     none.
*/
uint8_t SNP_sendSecurityRequest(void);

/**
 *  @fn         SNP_sendSecurityRequest
 *
 *  @brief      Set the authentication data for this authentication event
 *
 *  @param[in]  pReq :the Authentication data.
 *
 *  @return     none.
 */
uint8_t SNP_setAuthenticationData(snpSetAuthDataReq_t *pReq);
/**
 * @}
 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPROCESSOR_H */
