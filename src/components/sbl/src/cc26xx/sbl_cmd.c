/******************************************************************************

 @file  sbl_cmd.c

 @brief This file contains SBL commands

 Group: WCS, LPC, BTS
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
#include <xdc/std.h>

#include "sbl.h"
#include "sbl_cmd.h"
#include "sbl_tl.h"

/*********************************************************************
 * CONSTANTS
 */

#define SBL_MAX_BYTES_PER_TRANSFER   252
   
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SBL_CMD_uint32MSB(uint32_t src, uint8_t *pDst);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint8_t SBL_CMD_ping(void)
{
  return SBL_TL_sendCmd(SBL_CMD_PING, NULL, 0);
}
   
uint8_t SBL_CMD_download(uint32_t addr, uint32_t size)
{
  uint8_t payload[8];
  
  // Check input arguments
  if (size & 0x03)
  {
    // NEED TO CHECK: addr in range of device
    // CHECKING: byte count must be multiple of 4
    return SBL_FAILURE;
  }

  // Initialize payload - MSB order
  SBL_CMD_uint32MSB(addr, &payload[0]);
  SBL_CMD_uint32MSB(size, &payload[4]);

  return SBL_TL_sendCmd(SBL_CMD_DOWNLOAD, payload, sizeof(payload));
}

uint8_t SBL_CMD_getStatus(void)
{
  uint8_t rsp[1];
  uint16_t rspLen = 0;
  
  // Send Get Status command
  if (SBL_TL_sendCmd(SBL_CMD_GET_STATUS, NULL, 0) == SBL_FAILURE)
  {
    return SBL_FAILURE;
  }
  
  // Get the status response from target
  SBL_TL_getRsp(rsp, sizeof(rsp), &rspLen);
  
  return rsp[0];
}

uint8_t SBL_CMD_sendData(uint8_t *pData, uint16_t len)
{
  // Check input arguments
  if (len > SBL_MAX_BYTES_PER_TRANSFER)
  {
    // Length of bytes excess maximum allowed per transfer
    return SBL_FAILURE;
  }

  // Send command
  return SBL_TL_sendCmd(SBL_CMD_SEND_DATA, pData, len);
}

uint8_t SBL_CMD_reset(void)
{
  return SBL_TL_sendCmd(SBL_CMD_RESET, NULL, 0);
}

uint8_t SBL_CMD_sectorErase(uint32_t addr)
{
  uint8_t payload[4];
  
  // Initialize payload - MSB order
  SBL_CMD_uint32MSB(addr, &payload[0]);

  // Send command
  return SBL_TL_sendCmd(SBL_CMD_SECTOR_ERASE, payload, sizeof(payload));
}

uint8_t SBL_CMD_crc32(uint32_t addr, uint32_t size, uint32_t readRepeatCnt)
{
  uint8_t payload[12];

  // Initialize payload - MSB order
  SBL_CMD_uint32MSB(addr, &payload[0]);
  SBL_CMD_uint32MSB(size, &payload[4]);
  SBL_CMD_uint32MSB(readRepeatCnt, &payload[8]);

  return SBL_TL_sendCmd(SBL_CMD_CRC32, payload, sizeof(payload));
}

uint32_t SBL_CMD_getChipID(void)
{
  uint8_t rsp[4];
  uint16_t rspLen = 0;
  uint32_t chipID = 0;
  
  // Send Get Chip ID command
  if (SBL_TL_sendCmd(SBL_CMD_GET_CHIP_ID, NULL, 0) == SBL_FAILURE)
  {
    return SBL_FAILURE;
  }
  
  // Get the status response from target
  SBL_TL_getRsp(rsp, sizeof(rsp), &rspLen);
  
  // Reverse MSB order of response to get Chip ID
  SBL_CMD_uint32MSB(chipID, rsp);
  
  return chipID;
}

uint8_t SBL_CMD_bankErase(void)
{
  return SBL_TL_sendCmd(SBL_CMD_BANK_ERASE, NULL, 0);
}
   
uint8_t SBL_CMD_setCCFG(uint32_t fieldID, uint32_t fieldValue)
{
  uint8_t payload[8];

  // Initialize payload - MSB order
  SBL_CMD_uint32MSB(fieldID, &payload[0]);
  SBL_CMD_uint32MSB(fieldValue, &payload[4]);

  return SBL_TL_sendCmd(SBL_CMD_SET_CCFG, payload, sizeof(payload));
}

void SBL_CMD_uint32MSB(uint32_t src, uint8_t *pDst)
{
  // MSB first
  pDst[0] = (uint8_t)(src >> 24);
  pDst[1] = (uint8_t)(src >> 16);
  pDst[2] = (uint8_t)(src >> 8);
  pDst[3] = (uint8_t)(src >> 0);
}
