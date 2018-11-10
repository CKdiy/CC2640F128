/******************************************************************************

 @file  simple_np_gap.c

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include "hal_types.h"
#include "comdef.h"

#include <icall.h>

#include "gatt.h"

#include "gattservapp.h"
#include "inc/npi_task.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "hci_tl.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#include "snp.h"
#include "simple_np.h"
#include "simple_np_gap.h"
#include "simple_np_gatt.h"
#include "devinfoservice.h"

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     160

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define SNP_ADV_DATA_IN_CONNECTION            0x1
#define SNP_ADV_DATA_AFTER_CONNECTION         0x2

#define SNP_ADV_DATA_SCAN_RSP_IDX             0
#define SNP_ADV_DATA_NON_CONN_IDX             1
#define SNP_ADV_DATA_CONN_IDX                 2
#define SNP_ADV_DATA_MAX_IDX                  3

/*********************************************************************
 * TYPEDEFS
 */

// Passcode/Numeric Comparison display data structure
typedef struct
{
  uint16_t connHandle;
  uint8_t uiOutputs;
  uint32_t numComparison;
} pairDisplay_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Advertisement attribute
static uint8_t advBehavior = 0;
static uint8_t advReqAdvType = 0;
//! [Default Scan rsp data]
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

//! [Default Adv. data]
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // Local name
  0x04,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'N',
  'P'
};
//! [Default Adv. data]

//
//// GAP GATT Attributes
typedef struct snp_advData
{
  uint8_t length;
  uint8_t *pData;
} snp_advData_t;

// The following table contains pointer to adv. data buffer.
// adv data buffer must be created from Heap only, so they can be release
// when a update of the pointer is needed.
snp_advData_t advPtrTable[3] =
{
  {0, NULL},   // reserve for scan rsp
  {0, NULL},   // reserve for adv data  (while not in a connection)
  {0, NULL},   // reserve for adv data  (while in a connection)
};

// White List Filter Policy - default is not white list filtering.
uint8_t snp_whiteListFilterPolicy = GAP_FILTER_POLICY_ALL;

/*********************************************************************
 * EXTERN FUNCTIONS
 */

extern void SNP_resetGATT(uint16_t handle);

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SNP_paramUpdateCB(uint16_t connInterval, uint16_t connSlaveLatency,
                        uint16_t connTimeout);

static void SNP_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                           uint8_t uiInputs, uint8_t uiOutputs,
                           uint32_t numComparison);

static void SNP_securityStateCB(uint16_t connHandle, uint8_t state,
                                uint8_t status);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Role Callbacks
static  gapRolesCBs_t SNP_gapRoleCBs =
{
  (gapRolesStateNotify_t) SNP_stateChangeCB,               // Profile State Change Callbacks
};

static gapRolesParamUpdateCB_t paramUpdateCB =
{
  SNP_paramUpdateCB
};

// GAP Bond Manager Callbacks
static gapBondCBs_t SNP_BondMgrCBs =
{
  (pfnPasscodeCB_t)SNP_passcodeCB, // Passcode callback
  SNP_securityStateCB              // Pairing and Bonding state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************

 * @fn      SNP_initGAP
 *
 * @brief   Initial parameter needed for using SPNP GATT functionality
 *
 * @return  None.
 */
void SNP_initGAP(void)
{

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),
                         advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }
  // Initialize the Advertising data store table with a valid entry
  // for the non-connectable mode.
  {
    uint8_t *pDataPtr;
    advPtrTable[SNP_ADV_DATA_NON_CONN_IDX].pData = pDataPtr = ICall_malloc(sizeof(advertData));
    if(pDataPtr)
    {
      advPtrTable[SNP_ADV_DATA_NON_CONN_IDX].length = sizeof(advertData);
      memcpy(pDataPtr, advertData, sizeof(advertData));
    }
  }

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  GAPRole_RegisterAppCBs(&paramUpdateCB);

  /* Set up the GAP Bond Manager
   * Default behavior is No Input No Output IO Capabilities Just Works,
   * Secure Connections, and Bonding Enabled.  Only the Slave's Encryption Data
   * and the Master's Identity Data are distributed.
   */
  {
    uint8_t mode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t bonding = TRUE;
    uint8_t wlPolicy = TRUE;
    uint8_t lruPolicy = TRUE;
    // Only request Master's Encryption and Identity information.
    uint8_t keyDist = GAPBOND_KEYDIST_SENCKEY | GAPBOND_KEYDIST_MIDKEY;

    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(ioCap), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(bonding), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(mode), &mode);
    GAPBondMgr_SetParameter(GAPBOND_AUTO_SYNC_WL, sizeof(wlPolicy), &wlPolicy);
    GAPBondMgr_SetParameter(GAPBOND_KEY_DIST_LIST, sizeof(keyDist), &keyDist);
    GAPBondMgr_SetParameter(GAPBOND_LRU_BOND_REPLACEMENT, sizeof(lruPolicy),
                            &lruPolicy);
  }

  // Start Bond Manager
  VOID GAPBondMgr_Register(&SNP_BondMgrCBs);
}

/*********************************************************************
 * @fn      SNP_AdvStateChange
 *
 * @brief   Process a pending GAP Role state change event. This function
 *          will decide how and when to change Advertisement state and send
 *          event to AP accordingly.
 *
 * @param   newState      - new GAP ROLE state
 * @param   previousState - previous GAP ROLE state
 *
 * @return  None.
 */
void SNP_AdvStateChange(gaprole_States_t newState,
                        gaprole_States_t previousState)
{
  static uint8_t advCurrentState = false;
  static uint8_t advDbleStateChange = false;

  if(newState == GAPROLE_CONNECTED)
  {
    if(previousState == GAPROLE_CONNECTED_ADV)
    {
      //Adv. has been stop while in a connection.
      uint8_t status = SNP_SUCCESS;
      SNP_eventToHost_send(SNP_ADV_ENDED_EVT, &status, 0, NULL);
      advCurrentState = false;
    }
    else
    {
      uint8_t advertEnabled = false;
      // We just enter a connection
      if(advBehavior &  SNP_ADV_DATA_IN_CONNECTION)
      {
        // need to disable connect. adv and enable non-conn. adv.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        advertEnabled = true;
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        // Set it to true to be sure no adv event are send to the AP.
        advCurrentState = true;
      }
      else
      {
        // Need to Stop Advertisement and send the event to the application
        // processor
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        {
          //This is a normal non-conn advertisement start
          uint8_t status = SNP_SUCCESS;
          //Advertisement Ended due to the disconnection
          SNP_eventToHost_send(SNP_ADV_ENDED_EVT, &status, 0, NULL);
        }
        advCurrentState = false;
      }
    }
    // if connected adv data are available, use them
    if(advPtrTable[SNP_ADV_DATA_CONN_IDX].length)
    {
      // Use Adv. Data for connected mode
      GAPRole_SetParameter(GAPROLE_ADVERT_DATA,
                           advPtrTable[SNP_ADV_DATA_CONN_IDX].length,
                           advPtrTable[SNP_ADV_DATA_CONN_IDX].pData);
    }
  }
  else if(newState == GAPROLE_CONNECTED_ADV)
  {
    // This state can be reach only from the GAPROLE_CONNECTED state
    // Adv has started while in a connection.
    // check if it was on going before.
    if(advCurrentState == true)
    {
      //nothing to do, keep it for clarity, compiler will optimize it.
    }
    else
    {
      //request to start adv. was done from the startAdv API
      uint8_t status = SNP_SUCCESS;
      SNP_eventToHost_send(SNP_ADV_STARTED_EVT, &status, 0, NULL);
    }
    advCurrentState = true;
  }
  else if((newState == GAPROLE_WAITING) ||
          (newState == GAPROLE_WAITING_AFTER_TIMEOUT) ||
          (newState == GAPROLE_ADVERTISING_NONCONN))
  {
    // Check if we came from a connection or not
    // Check current Adv state, if currently
    if((previousState == GAPROLE_CONNECTED) ||
       (previousState == GAPROLE_CONNECTED_ADV))
    {
      // a connection just terminate.
      // check if adv. need to be started.
      if(advBehavior & SNP_ADV_DATA_AFTER_CONNECTION)
      {
        uint8_t advertEnabled;
        // check if we were currently advertising
        if(advCurrentState == true)
        {
          // Still advertising, since we came from a connection, non-conn adv
          // was used.
          if(advReqAdvType != GAP_ADTYPE_ADV_NONCONN_IND)
          {
            // the following double action will trigger two state changes:
            // to GAPROLE_WAITING and then to GAPROLE_ADVERTISING
            // In order to avoid false interpretation and sending wrong event
            // a static variable is used to keep this action in mind.
            advDbleStateChange = true;
            advertEnabled = false;
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            advertEnabled = true;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
          }
          else
          {
            // Nothing to be done, continue advertising with non-connect.
            // keep it for clarity, compiler will optimize it.
          }
        }
        else
        {
          advertEnabled = true;
          // Need to Switch  Adv.  ONM.
          if(advReqAdvType == GAP_ADTYPE_ADV_NONCONN_IND)
          {
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
          }
          else
          {
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
          }
        }
      }
      else
      {
        // No advertisement after a connection is terminated.
        // need to disable non-connect. adv and enable conn. adv.
        // Force both connectable and non-coonectable adv to stop.
        uint8_t advertEnabled = false;
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
      }
      if(advPtrTable[SNP_ADV_DATA_CONN_IDX].length)
      {
        // SNP was using adv data for connected mode, switch to use adv
        // Data for non-connected mode
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA,
                             advPtrTable[SNP_ADV_DATA_NON_CONN_IDX].length,
                             advPtrTable[SNP_ADV_DATA_NON_CONN_IDX].pData);
      }
    }
    else
    {
      // we were not in a connection, meaning we came here from an advertisement
      // state, or from waiting state.
      if(newState == GAPROLE_ADVERTISING_NONCONN)
      {
        if(advDbleStateChange == false)
        {
          //Advertisement has started.
          uint8_t status = SNP_SUCCESS;
          SNP_eventToHost_send(SNP_ADV_STARTED_EVT, &status, 0, NULL);
          advCurrentState = true;
        }
        else
        {
          advDbleStateChange = false;
        }
      }
      else if(advCurrentState == true)
      {
        if(previousState == GAPROLE_ADVERTISING)
        {
          // Advertisement has stopped, need to send event
          uint8_t status = SNP_SUCCESS;
          SNP_eventToHost_send(SNP_ADV_ENDED_EVT, &status, 0, NULL);
          advCurrentState = false;
          advDbleStateChange = false;
        }
        else if(advDbleStateChange == false)
        {
          // Advertisement has stopped, need to send event
          uint8_t status = SNP_SUCCESS;
          SNP_eventToHost_send(SNP_ADV_ENDED_EVT, &status, 0, NULL);
          advCurrentState = false;
        }
        else
        {
          // We are in a double state change, do nothing.
        }
      }
    }
  }
  else if(newState == GAPROLE_ADVERTISING)
  {
    // Advertising as started
    if(advCurrentState == false)
    {
      uint8_t status = SNP_SUCCESS;
      SNP_eventToHost_send(SNP_ADV_STARTED_EVT, &status, 0, NULL);
    }
    advCurrentState = true;
  }
}

/*********************************************************************
 * @fn      SNP_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
void SNP_processStateChangeEvt(gaprole_States_t newState)
{
  static bool firstConnFlag = false;
  static gaprole_States_t currentState = GAPROLE_STARTED;
  static uint16_t currentConnectHandle;


  switch (newState)
  {
    case GAPROLE_STARTED:
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &currentConnectHandle);
      break;

    case GAPROLE_ADVERTISING:
      break;

    case GAPROLE_ADVERTISING_NONCONN:
    /* After a connection is dropped a device will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
      {
        if(firstConnFlag)
        {
          // We were in a connection.
          uint8_t param[3];

          param[0] = LO_UINT16(currentConnectHandle);
          param[1] = HI_UINT16(currentConnectHandle);
          GAPRole_GetParameter(GAPROLE_CONN_TERM_REASON, &param[2]);

          //reset the GATT state for this connection handle
          SNP_resetGATT(*((uint16_t*) &param[0]));
          //Connection Ended
          SNP_eventToHost_send(SNP_CONN_TERM_EVT, NULL, sizeof(param), param);
        }
        // Reset flag for next connection.
        firstConnFlag = false;
      }
      break;
    case GAPROLE_CONNECTED:
      if(firstConnFlag == false)
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &currentConnectHandle);
        {
          uint8_t param[15];
          GAPRole_GetParameter(GAPROLE_CONNHANDLE, &param[0]);
          GAPRole_GetParameter(GAPROLE_CONN_INTERVAL, &param[2]);
          GAPRole_GetParameter(GAPROLE_CONN_LATENCY, &param[4]);
          GAPRole_GetParameter(GAPROLE_CONN_TIMEOUT, &param[6]);
          GAPRole_GetParameter(GAPROLE_BD_ADDR_TYPE, &param[8]);
          memcpy(&param[9], peerAddress, 6);
          //Advertisement Ended due to the connection
          SNP_eventToHost_send(SNP_CONN_EST_EVT,
                                NULL, sizeof(param), param);
        }

        // if 4.1 feature are enable on the controller,
        // then the adv needs to be forced to
        // be non-connectable, since peripheral.c does not support multiple
        // connection.
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this
        // state we will be turning advertising back on.
        firstConnFlag = true;
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:
      {
        if(firstConnFlag)
        {
          // We were in a connection.
          uint8_t param[3];

          param[0] = LO_UINT16(currentConnectHandle);
          param[1] = HI_UINT16(currentConnectHandle);
          GAPRole_GetParameter(GAPROLE_CONN_TERM_REASON, &param[2]);

          //reset the GATT state for this connection handle
          SNP_resetGATT(*((uint16_t*)&param[0]));
          //Connection Ended
          SNP_eventToHost_send(SNP_CONN_TERM_EVT, NULL, sizeof(param), param);

          // Reset flag for next connection.
          firstConnFlag = false;
        }
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      if(firstConnFlag)
      {
          uint8_t param[3];
          param[0] = LO_UINT16(currentConnectHandle);
          param[1] = HI_UINT16(currentConnectHandle);
          GAPRole_GetParameter(GAPROLE_CONN_TERM_REASON, &param[2]);

          //reset the GATT state for this connection handle
          SNP_resetGATT(*((uint16_t*) &param[0]));

          //Connection Ended
          SNP_eventToHost_send(SNP_CONN_TERM_EVT, NULL, sizeof(param), param);
      }
      // Reset flag for next connection.
      firstConnFlag = false;
     break;

    case GAPROLE_ERROR:
      break;

    default:
      break;
  }

  SNP_AdvStateChange(newState, currentState);
  currentState = newState;
}

/**
 *  SNP_startAdv
 *
 */
uint8_t SNP_startAdv(snpStartAdvReq_t *pReq)
{
  uint8_t status = SNP_SUCCESS;
  uint8_t value;
  VOID GAPRole_StartDevice(&SNP_gapRoleCBs);

  if(pReq->type == GAP_ADTYPE_ADV_HDC_DIRECT_IND)
  {
    status = SNP_INVALID_PARAMS;
  }
  else
  {
    //If currently advertising
    GAPRole_GetParameter(GAPROLE_STATE, &value);
    if((value == GAPROLE_ADVERTISING) ||
       (value == GAPROLE_ADVERTISING_NONCONN)||
       (value == GAPROLE_CONNECTED_ADV))
    {
      //Adv need to be stopped first...
      status = SNP_ALREADY_ADVERTISING;
    }
  }

  // If type of advertising is invalid or process is already occuring, exit
  // early.
  if (status)
  {
    return status;
  }

  // Duration of the advertisement
  // In order to simplify design, set the timeout and interval for both
  // limited and general mode. The mode is define by the Adflags advertisement
  // token.
  // User can still set them individually by using the 'setGapParam' command.
  if(pReq->timeout)
  {
    // Save the timeout in second for the limited mode.
    uint16_t temp = pReq->timeout/1000;
    status |= GAPRole_SetParameter(TGAP_LIM_ADV_TIMEOUT, sizeof(uint16_t),
                                    &temp);
    status |= GAPRole_SetParameter(TGAP_GEN_DISC_ADV_MIN, sizeof(uint16_t),
                                    &pReq->timeout);
  }

  // Set advertising interval
  if(pReq->interval)
  {
    status |= GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, pReq->interval);
    status |= GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, pReq->interval);
    status |= GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, pReq->interval);
    status |= GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, pReq->interval);
  }

  // Set filter policy
  status |= GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof(uint8_t),
                                 &snp_whiteListFilterPolicy);

  //Set Advertise Type
  status |= GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t),
                                 &pReq->type);

  if(status)
  {
    //Error management
    return SNP_OUT_OF_RESOURCES ;
  }

  //Store behavior of the advertisement.
  advBehavior = (pReq->behavior) & 0x3;

  advReqAdvType = pReq->type;
  //Enable Advertising.
  if(GAP_ADTYPE_ADV_NONCONN_IND == pReq->type)
  {
    uint8_t start = TRUE;
    GAPRole_GetParameter(GAPROLE_STATE, &value);
    if(value == GAPROLE_CONNECTED)
    {
      if(advPtrTable[SNP_ADV_DATA_CONN_IDX].length)
      {
        // SNP was using adv data for connected mode, switch to use adv
        // Data for non-connected mode
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA,
                             advPtrTable[SNP_ADV_DATA_NON_CONN_IDX].length,
                             advPtrTable[SNP_ADV_DATA_NON_CONN_IDX].pData);
      }
    }
    status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                         &start);
  }
  else
  {
    uint8_t start = TRUE;
    GAPRole_GetParameter(GAPROLE_STATE, &value);
    if(value == GAPROLE_CONNECTED)
    {
      return SNP_CMD_REJECTED;
    }
    status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &start);
  }
  return status;
}


/**
 *  @fn      SNP_stopAdv
 *
 */
uint8_t SNP_stopAdv(void)
{
  uint8_t value = GAPROLE_INIT;
  uint8_t status = SNP_SUCCESS;

    //If adv was going on, wait for it to stop before sending the reply.
  GAPRole_GetParameter(GAPROLE_STATE, &value);
  if((value != GAPROLE_ADVERTISING) &&
     (value != GAPROLE_ADVERTISING_NONCONN) &&
     (value != GAPROLE_CONNECTED_ADV))
  {
    status = SNP_NOT_ADVERTISING;
    SNP_eventToHost_send(SNP_ADV_ENDED_EVT, &status, 0, NULL);
  }
  else
  {
    //Disable Advertising.
    uint8_t start = FALSE;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &start);
    GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                         &start);
  }

  return status;
}

/**
 *  SNP_setAdvData
 *
 */
uint8_t SNP_setAdvData(snpSetAdvDataReq_t *pReq, uint8_t len)
{
  uint8_t status = 0;
  uint8_t *pDataPtr;

  //Device must be started, or the set adv command will failed.
  VOID GAPRole_StartDevice(&SNP_gapRoleCBs);

  if(pReq->type < SNP_ADV_DATA_MAX_IDX || len > B_MAX_ADV_LEN)
  {
    pDataPtr = advPtrTable[pReq->type].pData;
    if(pDataPtr)
    {
      ICall_free(advPtrTable[pReq->type].pData);
    }
    advPtrTable[pReq->type].pData = pDataPtr = ICall_malloc(len);
    if(pDataPtr)
    {
      advPtrTable[pReq->type].length = len;
      memcpy(pDataPtr, pReq->pData, len);
      if(pReq->type == SNP_ADV_DATA_SCAN_RSP_IDX)
      {
        status = GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, len, pDataPtr);
      }
      else if(pReq->type == SNP_ADV_DATA_NON_CONN_IDX)
      {
        status = GAPRole_SetParameter(GAPROLE_ADVERT_DATA, len, pDataPtr);
      }
      else if(pReq->type == SNP_ADV_DATA_CONN_IDX)
      {
        uint8_t value;
        GAPRole_GetParameter(GAPROLE_STATE, &value);
        if(value == GAPROLE_CONNECTED_ADV)
        {
          status = GAPRole_SetParameter(GAPROLE_ADVERT_DATA, len, pDataPtr);
        }
      }
    }
    else
    {
      status = SNP_OUT_OF_RESOURCES;
    }
  }
  else
  {
    //Error, bad type
    status = SNP_INVALID_PARAMS;
  }
  return status;
}

/**
 *  SNP_updateConnParam
 *
 */
uint8_t SNP_updateConnParam(snpUpdateConnParamReq_t *pReq)
{
    uint8_t enableUpdateRequest = TRUE;
    uint8_t status;
    uint8_t value;

    status = GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    status |= GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &pReq->intervalMin);
    status |= GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &pReq->intervalMax);
    status |= GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &pReq->slaveLatency);
    status |= GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &pReq->supervisionTimeout);

    if(status == bleInvalidRange)
    {
      //Error management
      return SNP_INVALID_PARAMS;
    }
    else if(status)
    {
      // Mix of errors
      return SNP_FAILURE;
    }

    //check Status, only force update if in a connection.
    GAPRole_GetParameter(GAPROLE_STATE, &value);
    if((value == GAPROLE_CONNECTED) || (value == GAPROLE_CONNECTED_ADV))
    {
      if (GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ,
                                      sizeof(uint8_t),
                                      &enableUpdateRequest))
      {
        status = SNP_INVALID_PARAMS;
      }
    }
    else
    {
      status = SNP_NOT_CONNECTED;
    }
    return status;
}

/**
 *  SNP_terminateConn
 *
 */
uint8_t SNP_terminateConn(snpTermConnReq_t *pReq)
{
    uint8_t status = SNP_SUCCESS;

    if(pReq->option == SNP_GAP_TERM_CONN_IMMEDIATLY)
    {
      uint16_t value;
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &value);
      if(pReq->connHandle != value)
      {
        return SNP_INVALID_PARAMS;
      }
      else
      {
        HCI_EXT_DisconnectImmedCmd(value);
      }
    }
    else if(pReq->option == SNP_GAP_TERM_CONN_DEFAULT)
    {
      status = GAPRole_TerminateConnection();
    }
    else
    {
      return SNP_INVALID_PARAMS;
    }
    return status;
}

/**
 *  SNP_setGapParam
 *
 */
uint8_t SNP_setGapParam(snpSetGapParamReq_t *pReq)
{
    uint8_t status;

    if((pReq->paramId != TGAP_AUTH_TASK_ID) &&
       (pReq->paramId < TGAP_PARAMID_MAX))
    {
      status = GAP_SetParamValue(pReq->paramId, pReq->value);
    }
    else
    {
      status = SNP_INVALID_PARAMS;
    }
    return status;
}

/**
 *  SNP_getGapParam
 *
 */
uint8_t SNP_getGapParam(snpGetGapParamReq_t* pReq)
{
    uint8_t status;

    if(pReq->paramId != TGAP_AUTH_TASK_ID)
    {
      uint16_t value;
      value = GAP_GetParamValue(pReq->paramId);
      pReq->value = BUILD_UINT16( LO_UINT16(value),  HI_UINT16(value));
    }

    if(pReq->value != 0xFFFF)
    {
      status = SNP_SUCCESS;
    }
    else
    {
      status = SNP_INVALID_PARAMS;
    }
    return status;
}

/**
 *  @fn      SNP_paramUpdateCB
 *
 *  @brief   callback to be notify a update of the connection parameters
 *           has happened.
 *
 *  @param[in]   connInterval     : interval of the connection
 *  @param[in]  connSlaveLatency  : slave latency of the connection
 *  @param[in]  connTimeout       : supervision timeout of the connection
 *
 *  @return  status of the command.
 */
void SNP_paramUpdateCB(uint16_t connInterval,
                       uint16_t connSlaveLatency,
                       uint16_t connTimeout)
{
  uint16_t param[4];
  uint16_t temp;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &temp);

  param[0] = BUILD_UINT16(LO_UINT16(temp), HI_UINT16(temp));
  param[1] = BUILD_UINT16(LO_UINT16(connInterval), HI_UINT16(connInterval));
  param[2] = BUILD_UINT16(LO_UINT16(connSlaveLatency),
                          HI_UINT16(connSlaveLatency));
  param[3] = BUILD_UINT16(LO_UINT16(connTimeout), HI_UINT16(connTimeout));
  SNP_eventToHost_send(SNP_CONN_PARAM_UPDATED_EVT, NULL, sizeof(param),
                       (uint8_t*)param);
}

/**
 *  @fn      SNP_setSecurityParams
 *
 *  @brief   Set Security Parameters
 *
 *  @param[in]  pReq : Security parameter to set
 *
 *  @return  status of the command.
 */
uint8_t SNP_setSecurityParams(snpSetSecParamReq_t *pReq)
{
  uint16_t gapBondParamId;
  uint8_t len;
  uint8_t status = SNP_SUCCESS;

  switch(pReq->paramId)
  {
    case SNP_GAPBOND_PAIRING_MODE:
      gapBondParamId = GAPBOND_PAIRING_MODE;
      len = 1;
      break;

    case SNP_GAPBOND_IO_CAPABILITIES:
      {
        uint8_t mitm = (pReq->value != GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT) ? TRUE : FALSE;
        gapBondParamId = GAPBOND_IO_CAPABILITIES;
        len = 1;

        // MITM capabilities are infered from the IO Caps.
        status = GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, len, &mitm);
      }
      break;

    case SNP_GAPBOND_BONDING_ENABLED:
      gapBondParamId = GAPBOND_BONDING_ENABLED;
      len = 1;
      break;

    case SNP_GAPBOND_ERASE_ALLBONDS:
      gapBondParamId = GAPBOND_ERASE_ALLBONDS;
      len = 0;

      //Disable White List filtering
      snp_whiteListFilterPolicy = GAP_FILTER_POLICY_ALL;
      break;

    case SNP_GAPBOND_LRU_BOND_REPLACEMENT:
      gapBondParamId = GAPBOND_LRU_BOND_REPLACEMENT;
      len = 1;
      break;

    default:
      //Unknown parameter
      status = SNP_FAILURE;
      break;
  }

  if (status == SNP_SUCCESS)
  {
    status = GAPBondMgr_SetParameter(gapBondParamId, len, &pReq->value);
  }

  return status;
}

/**
*  @fn      SNP_setWhiteListFilterPolicy
*
*  @brief   Set White List Filter Policy
*
*  @param[in]  pReq : new Filter Policy
*
*  @return  status of the command.
*/
uint8_t SNP_setWhiteListFilterPolicy(snpSetWhiteListReq_t *pReq)
{
  uint8_t value;
  uint8_t numBonds;
  uint8_t status;

    // Get GAP State.
  GAPRole_GetParameter(GAPROLE_STATE, &value);

  // Get number of bonds.
  GAPBondMgr_GetParameter(GAPBOND_BOND_COUNT, &numBonds);

  // If Advertising or no bonds
  if (value == GAPROLE_ADVERTISING || value == GAPROLE_ADVERTISING_NONCONN ||
      numBonds == 0)
  {
    // Don't allow White List Filter Policy update as it cannot immediately take
    // effect.
    status = SNP_FAILURE;
  }
  else
  {
    // Update Filter Policy.
    snp_whiteListFilterPolicy = (pReq->useWhiteList == GAP_FILTER_POLICY_ALL) ?
                                 GAP_FILTER_POLICY_ALL :
                                 GAP_FILTER_POLICY_WHITE;

    status = SNP_SUCCESS;
  }

  return status;
}

/**
*  @fn      SNP_sendSecurityRequest
*
*  @brief   Send a Security Request
*
*  @param[in]  none
*
*  @return  status of the command.
*/
uint8_t SNP_sendSecurityRequest(void)
{
  uint8_t status = SNP_SUCCESS;
  uint8_t authReq;
  uint8_t param;

  // Set bonding requirements.
  GAPBondMgr_GetParameter(GAPBOND_BONDING_ENABLED, &authReq);

  // Bonding is bit 0.
  authReq = authReq ? SM_AUTH_STATE_BONDING : 0x00;

  // Get MITM protection requirements.
  GAPBondMgr_GetParameter(GAPBOND_MITM_PROTECTION, &param);

  // MITM is bit 2.
  authReq |= param ? SM_AUTH_STATE_AUTHENTICATED : 0x00;

  // Get status, Secure Connection might not be defined...
  GAPBondMgr_GetParameter(GAPBOND_SECURE_CONNECTION, &param);

  // SC is bit 3.
  authReq |= param ? SM_AUTH_STATE_SECURECONNECTION : 0x00;

  // Issue a Slave Security Request.
  status = GAP_SendSlaveSecurityRequest(0, authReq);

  if (status)
  {
   status = (status == bleNotConnected) ? SNP_NOT_CONNECTED : SNP_FAILURE;
  }

  return status;
}

/**
 *  @fn      SNP_sendSecurityRequest
 *
 *  @brief   Set the authentication data for this authentication event
 *
 *  @param[in]  pReq : the Authentication data.
 *
 *  @return  none.
 */
uint8_t SNP_setAuthenticationData(snpSetAuthDataReq_t *pReq)
{
  uint16_t temp;

  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &temp);

  return GAPBondMgr_PasscodeRsp(temp, SUCCESS, pReq->authData);
}

/*********************************************************************
 * @fn      SNP_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SNP_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                           uint8_t uiInputs, uint8_t uiOutputs,
                           uint32_t numComparison)
{
  uint8_t param[6];
  uint8_t *ptr;

  // Send an authentication data event to SAP
  param[0] = uiOutputs;
  param[1] = uiInputs;

  ptr = &param[2];
  UINT32_TO_BUF_LITTLE_ENDIAN(ptr, numComparison);

  SNP_eventToHost_send(SNP_AUTHENTICATION_EVT, NULL, sizeof(param),
                       (uint8_t*)param);
}

/*********************************************************************
 * @fn      SNP_securityStateCB
 *
 * @brief   Pair state callback.
 *
 * @return  none
 */
static void SNP_securityStateCB(uint16_t connHandle, uint8_t state,
                                uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_COMPLETE ||
      state == GAPBOND_PAIRING_STATE_BONDED ||
      state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    uint8_t param[2];

    switch ( state )
    {
      case GAPBOND_PAIRING_STATE_COMPLETE:
        param[0] = SNP_GAPBOND_PAIRING_STATE_COMPLETE;
        break;

      case GAPBOND_PAIRING_STATE_BONDED:
        param[0] = SNP_GAPBOND_PAIRING_STATE_BONDED;
        break;

      case GAPBOND_PAIRING_STATE_BOND_SAVED:
        param[0] = SNP_GAPBOND_PAIRING_STATE_BOND_SAVED;
        break;
    }

    param[1] = status;

    // Send Security Event to SAP
    SNP_eventToHost_send(SNP_SECURITY_EVT, NULL, sizeof(param),
                         param);
  }
}


/*********************************************************************
*********************************************************************/
