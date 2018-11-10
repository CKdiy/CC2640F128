/******************************************************************************

 @file  glucose_discovery.c

 @brief Glucose Collector App service and characteristic discovery routines for
        use with the CC26xx Bluetooth Low Energy Protocol Stack.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "glucose_collector.h"
#include "glucservice.h"
#include "devinfoservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Length of Characteristic declaration + handle with 16 bit UUID
#define CHAR_DESC_HDL_UUID16_LEN        7

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Attribute handle cache
uint16_t glucoseHdlCache[HDL_CACHE_LEN];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Attribute handles used during discovery
static uint16_t glucoseSvcStartHdl;
static uint16_t glucoseSvcEndHdl;
static uint8_t  glucoseEndHdlIdx;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8_t GlucoseDisc(uint8_t state, gattMsgEvent_t *pMsg);
static uint8_t GlucoseDevInfoDisc(uint8_t state, gattMsgEvent_t *pMsg);

/*********************************************************************
 * @fn      glucoseDiscStart()
 *
 * @brief   Start service discovery.
 *
 * @param   none
 *
 * @return  New discovery state.
 */
uint8_t glucoseDiscStart(void)
{
  // Clear handle cache
  memset(glucoseHdlCache, 0, sizeof(glucoseHdlCache));

  // Start discovery with first service
  return glucoseDiscGattMsg(DISC_GLUCOSE_START, NULL);
}

/*********************************************************************
 * @fn      glucoseDiscGattMsg()
 *
 * @brief   Handle GATT messages for characteristic discovery.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
uint8_t glucoseDiscGattMsg(uint8_t state, gattMsgEvent_t *pMsg)
{
  // Execute discovery function for service
  do
  {
    switch (state & 0xF0)
    {
      // Current glucose service
      case DISC_GLUCOSE_START:
        state = GlucoseDisc(state, pMsg);

        if(state == DISC_IDLE)
        {
          state = DISC_DEVINFO_START;
        }
        break;

      case DISC_DEVINFO_START:
        state = GlucoseDevInfoDisc(state, pMsg);
        break;

      default:
        break;
    }
  } while ((state != 0) && ((state & 0x0F) == 0));

  return state;
}

/*********************************************************************
 * @fn      GlucoseDisc
 *
 * @brief   Current glucose service and characteristic discovery.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8_t GlucoseDisc(uint8_t state, gattMsgEvent_t *pMsg)
{
  uint8_t newState = state;

  switch (state)
  {
    case DISC_GLUCOSE_START:
      {
        uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(GLUCOSE_SERV_UUID),
                                           HI_UINT16(GLUCOSE_SERV_UUID) };

        // Initialize service discovery variables
        glucoseSvcStartHdl = glucoseSvcEndHdl = 0;
        glucoseEndHdlIdx = 0;

        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID(glucCollConnHandle, uuid,
                                      ATT_BT_UUID_SIZE, glucCollTaskId);

        newState = DISC_GLUCOSE_SVC;
      }
      break;

    case DISC_GLUCOSE_SVC:
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
        glucoseSvcStartHdl =
          ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        glucoseSvcEndHdl =
          ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }

      // If procedure complete
      if ((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  &&
           pMsg->hdr.status == bleProcedureComplete) ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        // If service found
        if (glucoseSvcStartHdl != 0)
        {
          // Discover all characteristics
          GATT_DiscAllChars(glucCollConnHandle, glucoseSvcStartHdl,
                            glucoseSvcEndHdl, glucCollTaskId);

          newState = DISC_GLUCOSE_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }
      break;

    case DISC_GLUCOSE_CHAR:
      {
        uint8_t   i;
        uint8_t   *p;
        uint16_t  handle;
        uint16_t  uuid;

        // Characteristics found
        if (pMsg->method == ATT_READ_BY_TYPE_RSP &&
            pMsg->msg.readByTypeRsp.numPairs > 0 &&
            pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN)
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.pDataList;

          for (i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i--)
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);

            // If looking for end handle
            if (glucoseEndHdlIdx != 0)
            {
              // End handle is one less than handle of characteristic declaration
              glucoseHdlCache[glucoseEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              glucoseEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch (uuid)
            {
              case GLUCOSE_MEAS_UUID:
                glucoseHdlCache[HDL_GLUCOSE_START] = handle;
                glucoseEndHdlIdx = HDL_GLUCOSE_END;
                break;

              case GLUCOSE_CONTEXT_UUID:
                glucoseHdlCache[HDL_GLUCOSE_CONTEXT_START] = handle;
                glucoseEndHdlIdx = HDL_GLUCOSE_CONTEXT_END;
                break;

              case RECORD_CTRL_PT_UUID:
                glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START] = handle;
                glucoseEndHdlIdx = HDL_GLUCOSE_CTL_PNT_END;
                break;

              case GLUCOSE_FEATURE_UUID:
                glucoseHdlCache[HDL_GLUCOSE_FEATURE] = handle;
                break;

              default:
                break;
            }

            p += CHAR_DESC_HDL_UUID16_LEN;
          }
        }

        // If procedure complete
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP  &&
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          // Special case of end handle at end of service
          if (glucoseEndHdlIdx != 0)
          {
            glucoseHdlCache[glucoseEndHdlIdx] = glucoseSvcEndHdl;
            glucoseEndHdlIdx = 0;
          }

          // If didn't find glucose characteristic
          if (glucoseHdlCache[HDL_GLUCOSE_START] == 0)
          {
            newState = DISC_FAILED;
          }
          else if (glucoseHdlCache[HDL_GLUCOSE_START] <
                   glucoseHdlCache[HDL_GLUCOSE_END])
          {
            // Discover characteristic descriptors
            GATT_DiscAllCharDescs(glucCollConnHandle,
                                  glucoseHdlCache[HDL_GLUCOSE_START] + 1,
                                  glucoseHdlCache[HDL_GLUCOSE_END],
                                  glucCollTaskId);

            newState = DISC_GLUCOSE_CCCD;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }
      break;

    case DISC_GLUCOSE_CCCD:
      {
        uint8_t i;

        // Characteristic descriptors found
        if (pMsg->method == ATT_FIND_INFO_RSP &&
            pMsg->msg.findInfoRsp.numInfo > 0 &&
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE)
        {
          attFindInfoRsp_t *pRsp = &(pMsg->msg.findInfoRsp);

          // For each handle/uuid pair
          for (i = 0; i < pRsp->numInfo; i++)
          {
            // Look for CCCD
            if ( ATT_BT_PAIR_UUID( pRsp->pInfo, i ) == GATT_CLIENT_CHAR_CFG_UUID )
            {
              // CCCD found
              glucoseHdlCache[HDL_GLUCOSE_MEAS_CCCD] =
                ATT_BT_PAIR_HANDLE( pRsp->pInfo, i );

              break;
            }
          }
        }

        // If procedure complete
        if ((pMsg->method == ATT_FIND_INFO_RSP  &&
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          // If CCCD found
          if (glucoseHdlCache[HDL_GLUCOSE_MEAS_CCCD] != 0)
          {
            // Should we look for unread category status CCCD
            if (glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START] <
                glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_END])
            {
              // Discover unread category status characteristic descriptors
              GATT_DiscAllCharDescs(glucCollConnHandle,
                                    glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START] + 1,
                                    glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_END],
                                    glucCollTaskId);

              newState = DISC_GLUCOSE_CTL_PNT_CCCD;
            }
            else
            {
              // Missing required characteristic
              newState = DISC_FAILED;
            }
          }
          else
          {
            // Missing required characteristic descriptor
            glucoseHdlCache[HDL_GLUCOSE_MEAS_CCCD] = 0;
            newState = DISC_FAILED;
          }
        }
      }
      break;

   case DISC_GLUCOSE_CTL_PNT_CCCD:
      {
        uint8_t i;

        // Characteristic descriptors found
        if (pMsg->method == ATT_FIND_INFO_RSP &&
            pMsg->msg.findInfoRsp.numInfo > 0 &&
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE)
        {
          attFindInfoRsp_t *pRsp = &(pMsg->msg.findInfoRsp);

          // For each handle/uuid pair
          for (i = 0; i < pRsp->numInfo; i++)
          {
            // Look for CCCD
            if ( ATT_BT_PAIR_UUID( pRsp->pInfo, i ) == GATT_CLIENT_CHAR_CFG_UUID )
            {
              // CCCD found
              glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_CCCD] =
                ATT_BT_PAIR_HANDLE( pRsp->pInfo, i );

              break;
            }
          }
        }

        // If procedure complete
        if ((pMsg->method == ATT_FIND_INFO_RSP  &&
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          // If CCCD found
          if (glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_CCCD] != 0)
          {
            // Should we look for unread category status CCCD
            if (glucoseHdlCache[HDL_GLUCOSE_CONTEXT_START] <
                glucoseHdlCache[HDL_GLUCOSE_CONTEXT_END])
            {
              // Discover unread category status characteristic descriptors
              GATT_DiscAllCharDescs(glucCollConnHandle,
                                    glucoseHdlCache[HDL_GLUCOSE_CONTEXT_START] + 1,
                                    glucoseHdlCache[HDL_GLUCOSE_CONTEXT_END],
                                    glucCollTaskId);

              newState = DISC_GLUCOSE_CONTEXT_CCCD;
            }
            else
            {
              // Done
              newState = DISC_IDLE;
            }
          }
          else
          {
            // Missing required characteristic descriptor
            glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_CCCD] = 0;
            newState = DISC_FAILED;
          }
        }
      }
      break;

   case DISC_GLUCOSE_CONTEXT_CCCD:
      {
        uint8_t i;

        // Characteristic descriptors found
        if (pMsg->method == ATT_FIND_INFO_RSP &&
            pMsg->msg.findInfoRsp.numInfo > 0 &&
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE)
        {
          attFindInfoRsp_t *pRsp = &(pMsg->msg.findInfoRsp);

          // For each handle/uuid pair
          for (i = 0; i < pRsp->numInfo; i++)
          {
            // Look for CCCD
            if ( ATT_BT_PAIR_UUID( pRsp->pInfo, i ) == GATT_CLIENT_CHAR_CFG_UUID )
            {
              // CCCD found
              glucoseHdlCache[HDL_GLUCOSE_CONTEXT_CCCD] =
                ATT_BT_PAIR_HANDLE( pRsp->pInfo, i );

              break;
            }
          }
        }

        // If procedure complete
        if ((pMsg->method == ATT_FIND_INFO_RSP  &&
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          newState = DISC_IDLE;
        }
      }
      break;

    default:
      break;
  }

  return newState;
}

/*********************************************************************
 * @fn      GlucoseDevInfoDisc
 *
 * @brief   Current glucose service and characteristic discovery.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8_t GlucoseDevInfoDisc(uint8_t state, gattMsgEvent_t *pMsg)
{
  uint8_t newState = state;

  switch (state)
  {
    case DISC_DEVINFO_START:
      {
        uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(DEVINFO_SERV_UUID),
                                           HI_UINT16(DEVINFO_SERV_UUID) };

        // Initialize service discovery variables
        glucoseSvcStartHdl = glucoseSvcEndHdl = 0;
        glucoseEndHdlIdx = 0;

        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID(glucCollConnHandle, uuid,
                                      ATT_BT_UUID_SIZE, glucCollTaskId);

        newState = DISC_DEVINFO_SVC;
      }
      break;

    case DISC_DEVINFO_SVC:
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
        glucoseSvcStartHdl =
          ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        glucoseSvcEndHdl =
          ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }

      // If procedure complete
      if ((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  &&
           pMsg->hdr.status == bleProcedureComplete) ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        // If service found
        if (glucoseSvcStartHdl != 0)
        {
          // Discover all characteristics
          GATT_DiscAllChars(glucCollConnHandle, glucoseSvcStartHdl,
                            glucoseSvcEndHdl, glucCollTaskId);

          newState = DISC_DEVINFO_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }
      break;

    case DISC_DEVINFO_CHAR:
      {
        uint8_t   i;
        uint8_t   *p;
        uint16_t  handle;
        uint16_t  uuid;

        // Characteristics found
        if (pMsg->method == ATT_READ_BY_TYPE_RSP &&
            pMsg->msg.readByTypeRsp.numPairs > 0 &&
            pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN)
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.pDataList;

          for (i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i--)
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);

            // If UUID is of interest, store handle
            switch (uuid)
            {
              case MANUFACTURER_NAME_UUID:
                glucoseHdlCache[HDL_DEVINFO_MANUFACTURER_NAME] = handle;
                break;

              case SYSTEM_ID_UUID:
                glucoseHdlCache[HDL_DEVINFO_SYSTEM_ID] = handle;
                break;

              case MODEL_NUMBER_UUID:
                glucoseHdlCache[HDL_DEVINFO_MODEL_NUM] = handle;
                break;

              default:
                break;
            }

            p += CHAR_DESC_HDL_UUID16_LEN;
          }
        }

        // If procedure complete
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP  &&
             pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
          // If didn't find required device info
          if (glucoseHdlCache[HDL_DEVINFO_MANUFACTURER_NAME] == 0 ||
              glucoseHdlCache[HDL_DEVINFO_SYSTEM_ID] == 0 ||
              glucoseHdlCache[HDL_DEVINFO_MODEL_NUM] == 0)
          {
            newState = DISC_FAILED;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }
      break;

    default:
      break;
  }

  return newState;
}


/*********************************************************************
 *********************************************************************/
