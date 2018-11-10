/******************************************************************************

 @file  simple_np_dev.c

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hal_types.h"
#include "comdef.h"

#include <icall.h>

#include "gatt.h"
#include "gap.h"

#include "gattservapp.h"
#include "npi_task.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "hci_tl.h"
#include "npi_data.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#include "snp.h"
#include "simple_np.h"
#include "simple_np_gap.h"
#include "simple_np_gatt.h"
#include "simple_np_gatt_internal.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

typedef struct snp_HCIoperation
{
  uint8_t  validity;
  uint16_t opcode;      //!< method of the ongoing ATT operation.
}snp_HCIoperation_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

uint16_t snp_gattEventMask = 0;

/*********************************************************************
 * EXTERN VARIABLES
 */

extern snp_HCIoperation_t  snp_HCIstore;

/*********************************************************************
 * SPNP TEST SERVICE
 */

/*********************************************************************
 * Extern FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SNP_ReverseBytes(uint8_t *buf, uint8_t len);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 *  SNP_executeTestCmd
 */
void SNP_executeTestCmd(snpTestCmdRsp_t *pRsp)
{
#ifdef HEAPMGR_METRICS
  {
    extern uint16_t heapmgrMemAlo;
    extern uint16_t heapmgrMemMax;
    pRsp->memAlo = heapmgrMemAlo;
    pRsp->memMax = heapmgrMemMax;
    pRsp->memSize = HEAPMGR_SIZE;
  }
#else
  {
    pRsp->memAlo = 0;
    pRsp->memMax = 0;
    pRsp->memSize = 0;
  }
#endif
}

/**
 *  SNP_getStatus
 */
void SNP_getStatus(snpGetStatusCmdRsp_t *pMsg)
{
  extern snp_ATToperation_t  snp_ATTstore;
  GAPRole_GetParameter(GAPROLE_STATE, &pMsg->gapRoleStatus);
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &pMsg->advStatus);
  if(pMsg->advStatus == FALSE)
  {
    // Connectable Advertisement not enabled, Check of non-connectable adv
    // enabled
    GAPRole_GetParameter(GAPROLE_ADV_NONCONN_ENABLED, &pMsg->advStatus);
  }

  pMsg->ATTstatus = snp_ATTstore.validity;
  if(snp_ATTstore.validity)
  {
    pMsg->ATTmethod = snp_ATTstore.method;
  }
  else
  {
    pMsg->ATTmethod = 0;
  }
}

/**
 *  SNP_getRev
 *
 */
void SNP_getRev(snpGetRevisionRsp_t *pRsp)
{
  ICall_BuildRevision buildRev = {0};
  pRsp->snpVer = BUILD_UINT16(HI_UINT16(SNP_VERSION), LO_UINT16(SNP_VERSION));
  pRsp->status = Util_buildRevision(&buildRev);

  // Stack revision
  // Byte 0: Major
  // Byte 1: Minor
  // Byte 2: Patch
  pRsp->stackBuildVer[0] = BREAK_UINT32(buildRev.stackVersion, 0);
  pRsp->stackBuildVer[1] = BREAK_UINT32(buildRev.stackVersion, 1);
  pRsp->stackBuildVer[2] = BREAK_UINT32(buildRev.stackVersion, 2);

  // Build revision
  pRsp->stackBuildVer[3] = LO_UINT16(buildRev.buildVersion);
  pRsp->stackBuildVer[4] = HI_UINT16(buildRev.buildVersion);

  // Stack info (Byte 5)
  pRsp->stackBuildVer[5] = buildRev.stackInfo;

  // Controller info - part 1 (Byte 6)
  pRsp->stackBuildVer[6] = LO_UINT16(buildRev.ctrlInfo);

  // Controller info - part 2 (Byte 7)
  pRsp->stackBuildVer[7] = 0; // reserved

  // Host info - part 1 (Byte 8)
  pRsp->stackBuildVer[8] = LO_UINT16(buildRev.hostInfo);

  // Host info - part 2 (Byte 9)
  pRsp->stackBuildVer[9] = 0; // reserved

}

/**
 *  SNP_getRand
 *
 */
void SNP_getRand(snpGetRandRsp_t *pRsp)
{
  pRsp->rand = Util_GetTRNG();
}

/**
 *   SNP_maskEvt
 */
void SNP_maskEvt(snpMaskEventReq_t *pReq, snpMaskEventRsp_t *pRsp)
{
  snp_gattEventMask = pReq->eventMask;
}


/**
 *  SNP_executeHCIcmd
 */
uint8_t SNP_executeHCIcmd(snpHciCmdReq_t *pReq, uint16_t dataLen)
{
  bStatus_t stat;
  uint16_t opcode =  pReq->opcode;
  uint8_t *pData = (uint8_t*)&pReq->pData;
  uint8_t status = blePending;

  if(snp_HCIstore.validity)
  {
    return SNP_CMD_ALREADY_IN_PROGRESS;
  }

  switch (opcode)
  {
    case SNP_HCI_OPCODE_EXT_RESET_SYSTEM:
    {
      stat =  HCI_EXT_ResetSystemCmd(LL_EXT_RESET_SYSTEM_HARD);

      if(stat == SUCCESS)
      {
        status = SNP_SUCCESS;
      }
      else
      {
        status = bleIncorrectMode;
      }
    }
    break;

    case SNP_HCI_OPCODE_READ_BDADDR:
    {
      stat = HCI_ReadBDADDRCmd();
      if(stat == SUCCESS)
      {
        // Set state to wait for the HCI event with the address
        snp_HCIstore.validity = TRUE;
        snp_HCIstore.opcode = opcode;
        status = blePending;
      }
      else
      {
        status = stat;
      }
    }
    break;

    case SNP_HCI_OPCODE_EXT_SET_BDADDR:
      {
        if(dataLen != B_ADDR_LEN)
        {
          status = SNP_INVALID_PARAMS;
        }
        else
        {
          stat = HCI_EXT_SetBDADDRCmd(pData);
          if(stat == SUCCESS)
          {
            // Set state to wait for the HCI event with the address
            snp_HCIstore.validity = TRUE;
            snp_HCIstore.opcode = opcode;
            status = blePending;
          }
          else
          {
            status = stat;
          }
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_SET_TX_POWER:
      {
        if(dataLen != sizeof(uint8_t))
        {
          status = SNP_INVALID_PARAMS;
        }
        else
        {
          stat = HCI_EXT_SetTxPowerCmd(pData[0]);
          if(stat == SUCCESS)
          {
            // Set state to wait for the HCI event with the address
            snp_HCIstore.validity = TRUE;
            snp_HCIstore.opcode = opcode;
            status = blePending;
          }
          else
          {
            status = stat;
          }
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_SET_SCA:
      if(dataLen != sizeof(uint16_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_SetSCACmd((pData[0] &0xFF) +  (pData[1] << 8));
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_MODEM_TEST_TX:
      if(dataLen != 2*sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_ModemTestTxCmd(pData[0], pData[1]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_MODEM_HOP_TEST_TX:
      stat = HCI_EXT_ModemHopTestTxCmd();
      if(stat == SUCCESS)
      {
        // Set state to wait for the HCI event with the address
        snp_HCIstore.validity = TRUE;
        snp_HCIstore.opcode = opcode;
        status = blePending;
      }
      else
      {
        status = stat;
      }
      break;

    case SNP_HCI_OPCODE_EXT_MODEM_TEST_RX:
      if(dataLen != sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_ModemTestRxCmd(pData[0]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_END_MODEM_TEST:
      stat = HCI_EXT_EndModemTestCmd();
      if(stat == SUCCESS)
      {
        // Set state to wait for the HCI event with the address
        snp_HCIstore.validity = TRUE;
        snp_HCIstore.opcode = opcode;
        status = blePending;
      }
      else
      {
        status = stat;
      }
      break;

    case SNP_HCI_OPCODE_EXT_ENABLE_PTM:
      stat = HCI_EXT_EnablePTMCmd();
      if(stat == SUCCESS)
      {
        status = SNP_SUCCESS;
      }
      else
      {
        status = stat;
      }
      break;

    case SNP_HCI_OPCODE_EXT_SET_MAX_DTM_TX_POWER:
      if(dataLen != sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_SetMaxDtmTxPowerCmd(pData[0]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_READ_RSSI:
      if(dataLen != sizeof(uint16_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_ReadRssiCmd((pData[0] &0xFF) +  (pData[1] << 8));
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_LE_RECEIVER_TEST:
      if(dataLen != sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_LE_ReceiverTestCmd(pData[0]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_LE_TRANSMITTER_TEST:
      if(dataLen != 3 * sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_LE_TransmitterTestCmd(pData[0], pData[1], pData[2]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_LE_TEST_END:
      stat = HCI_LE_TestEndCmd();
      if(stat == SUCCESS)
      {
        // Set state to wait for the HCI event with the address
        snp_HCIstore.validity = TRUE;
        snp_HCIstore.opcode = opcode;
        status = blePending;
      }
      else
      {
        status = stat;
      }
      break;

    case SNP_HCI_OPCODE_EXT_PER:
      if(dataLen != (sizeof(uint8_t) + sizeof(uint16_t)))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_PacketErrorRateCmd(pData[0] +  (pData[1] << 8),
                                          pData[2]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_DECRYPT:
      if(dataLen != (KEYLEN + KEYLEN))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        // reverse byte order of key (MSB..LSB required)
        SNP_ReverseBytes(&pData[0], KEYLEN);

        // reverse byte order of encText (MSB..LSB required)
        SNP_ReverseBytes(&pData[KEYLEN], KEYLEN);

        stat = HCI_EXT_DecryptCmd(&pData[0], &pData[KEYLEN]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_LE_ENCRYPT:
      if(dataLen != (KEYLEN + KEYLEN))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        // reverse byte order of key (MSB..LSB required)
        SNP_ReverseBytes(&pData[0], KEYLEN);

        // reverse byte order of encText (MSB..LSB required)
        SNP_ReverseBytes(&pData[KEYLEN], KEYLEN);

        stat = HCI_LE_EncryptCmd(&pData[0], &pData[KEYLEN]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_OVERRIDE_SL:
      if(dataLen != sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {

        stat = HCI_EXT_SetSlaveLatencyOverrideCmd(pData[0]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_SET_FAST_TX_RESP_TIME:
      if(dataLen != sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_SetFastTxResponseTimeCmd(pData[0]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_ONE_PKT_PER_EVT:
      if(dataLen != sizeof(uint8_t))
      {
        status = SNP_INVALID_PARAMS;
      }
      else
      {
        stat = HCI_EXT_OnePktPerEvtCmd(pData[0]);
        if(stat == SUCCESS)
        {
          // Set state to wait for the HCI event with the address
          snp_HCIstore.validity = TRUE;
          snp_HCIstore.opcode = opcode;
          status = blePending;
        }
        else
        {
          status = stat;
        }
      }
      break;

    case SNP_HCI_OPCODE_EXT_GET_CONNECTION_INFO:
      stat = HCI_EXT_GetConnInfoCmd(NULL, NULL, NULL);
      if(stat == SUCCESS)
      {
        // Set state to wait for the HCI event with the address
        snp_HCIstore.validity = TRUE;
        snp_HCIstore.opcode = opcode;
        status = blePending;
      }
      else
      {
        status = stat;
      }
      break;
  default:
      status = SNP_HCI_CMD_UNKNOWN;
      break;
  }
   return status;
}

/*******************************************************************************
 * This function is used to reverse the order of the bytes in an array in place.
 */
static void SNP_ReverseBytes(uint8_t *buf, uint8_t len)
{
  uint8_t temp;
  uint8_t index = (uint8_t)(len - 1);
  uint8_t i;

  // adjust length as only half the operations are needed
  len >>= 1;

  // reverse the order of the bytes
  for (i=0; i<len; i++)
  {
    temp           = buf[i];
    buf[i]         = buf[index - i];
    buf[index - i] = temp;
  }

  return;
}
