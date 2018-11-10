/******************************************************************************

 @file  bim_main.c

 @brief This module contains the definitions for the main functionality of a
        Boot  Image Manager.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2012-2018, Texas Instruments Incorporated
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

/* -----------------------------------------------------------------------------
 *                                          Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_flash.h"
#include <driverlib/pwr_ctrl.h>
#include "hal_types.h"
#include "oad_target.h"

#include <inc/hw_device.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/aon_event.h>
#include <driverlib/osc.h>
/* -----------------------------------------------------------------------------
 *                                          Constants
 * -----------------------------------------------------------------------------
 */

#define BIM_IMG_A_PAGE        OAD_IMG_A_PAGE
#define BIM_IMG_A_OSET        OAD_IMG_A_OSET
#define BIM_IMG_A_AREA        OAD_IMG_A_AREA

#define BIM_IMG_B_PAGE        OAD_IMG_B_PAGE
#define BIM_IMG_B_OSET        OAD_IMG_B_OSET
#define BIM_IMG_B_AREA        OAD_IMG_B_AREA

#define BIM_CRC_OSET          OAD_IMG_CRC_OSET
#define BIM_HDR_OSET          OAD_IMG_HDR_OSET

/* -----------------------------------------------------------------------------
 *                                          Externs
 * -----------------------------------------------------------------------------
 */

void trimDevice(void);

/* -----------------------------------------------------------------------------
 *                                          Typedefs
 * -----------------------------------------------------------------------------
 */
/*
typedef struct {
  // User-defined Image Version Number - default logic uses simple a '<'
  // comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;
*/
/* -----------------------------------------------------------------------------
 *                                       Global Variables
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *                                       Local Variables
 * -----------------------------------------------------------------------------
 */

__no_init uint8 pgBuf[HAL_FLASH_PAGE_SIZE];

void halSleepExec(void);
static uint16 crc16(uint16 crc, uint8 val);

/*******************************************************************************
 * @fn          halSleepExec
 *
 * @brief       This function puts the device to sleep.
 *
 * @param       None.
 *
 * @return      None.
 */
void halSleepExec(void)
{
  AONIOCFreezeEnable();
  SysCtrlAonSync();
  PRCMPowerDomainOff(PRCM_DOMAIN_CPU);

  //Propogate changes to AON.
  PRCMLoadSet();

  VIMSModeSet(VIMS_BASE, VIMS_MODE_OFF);

  PRCMCacheRetentionDisable();

  SysCtrlAonSync();

  // Sleep.
  PRCMDeepSleep();

  // Reset
  OADTarget_systemReset();
}

/*******************************************************************************
 * @fn          crcCalc
 *
 * @brief       Run the CRC16 Polynomial calculation over the image specified.
 *
 * @param       page   - Flash page on which to beginning the CRC calculation.
 *
 * @param       offset - offset of first byte of image within first flash page
 *                       of the image.
 * @return      The CRC16 calculated.
 */
static uint16 crcCalc(uint8 page, uint16 offset)
{
  uint16 crc = 0;

  // Read first page of the image into the buffer.
  OADTarget_readFlash(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);

  const img_hdr_t *pImgHdr = (const img_hdr_t *)(pgBuf + offset + BIM_HDR_OSET);

  uint8 pageBeg = page;
  uint8 pageEnd = pImgHdr->len / (OAD_FLASH_PAGE_MULT);

  // Find ending offset on last page.
  uint16 osetEnd = ((pImgHdr->len + (offset / HAL_FLASH_WORD_SIZE)) -
                    (pageEnd * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))) *
                    HAL_FLASH_WORD_SIZE;

  // Set pageEnd to the end page of the OAD range.
  pageEnd += pageBeg;

  // Read over image pages.
  for (uint8_t pageIdx = pageBeg; pageIdx <= pageEnd; pageIdx++)
  {
    // Read over all flash words in a page, excluding the CRC section of the
    // first page and all bytes after the remainder bytes on the last page.
    for (uint16_t oset = (pageIdx == pageBeg ? offset + 4 : 0);
         oset < HAL_FLASH_PAGE_SIZE && (pageIdx < pageEnd || oset < osetEnd);
         oset++)
    {
      crc = crc16(crc, pgBuf[oset]);
    }

    // Read the next page into the buffer.
    if (pageIdx != pageEnd)
    {
      OADTarget_readFlash(pageIdx + 1, 0, pgBuf, HAL_FLASH_PAGE_SIZE);
    }
  }

  // IAR note explains that poly must be run with value zero for each byte of crc.
  crc = crc16(crc, 0);
  crc = crc16(crc, 0);

  return crc;
}

/*******************************************************************************
 * @fn          crcCheck
 *
 * @brief       Calculate the image CRC and set it ready-to-run if it is good.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beginning the CRC calculation.
 *
 * @param       offset - offset into page at which the image starts.
 *
 * output parameters
 *
 * None.
 *
 * @return      None, but no return from this function if the CRC check is good.
 */
static uint8 crcCheck(uint8 page, uint16 offset, uint16 *crc)
{
  HAL_BOARD_INIT();

  // Calculate CRC and compare with original output.
  if (crc[0] == crcCalc(page, offset))
  {
    // Set shadow equal to the original CRC output.
    crc[1] = crc[0];

    // Write CRC shadow to flash.
    OADTarget_writeFlash(page, offset + BIM_CRC_OSET, (uint8 *)crc, 4);

    // Allow branch to application.
    return 1;
  }

  // Image denied!
  return 0;
}

/*******************************************************************************
 * @fn          crc16
 *
 * @brief       Run the CRC16 Polynomial calculation over the byte parameter.
 *
 * input parameters
 *
 * @param       crc - Running CRC calculated so far.
 * @param       val - Value on which to run the CRC16.
 *
 * output parameters
 *
 * None.
 *
 * @return      crc - Updated for the run.
 */
static uint16 crc16(uint16 crc, uint8 val)
{
  const uint16 poly = 0x1021;
  uint8 cnt;

  for (cnt = 0; cnt < 8; cnt++, val <<= 1)
  {
    uint8 msb = (crc & 0x8000) ? 1 : 0;

    crc <<= 1;

    if (val & 0x80)
    {
      crc |= 0x0001;
    }

    if (msb)
    {
      crc ^= poly;
    }
  }

  return crc;
}

/*******************************************************************************
 * @fn          main
 *
 * @brief       C-code main function.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void main(void)
{
  uint16 crc[2];

  // Read Image B's CRC values from flash.  This is done first to prevent
  // Image A from ever invalidating itself.
  OADTarget_readFlash(BIM_IMG_B_PAGE, BIM_IMG_B_OSET + BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1] || crcCheck(BIM_IMG_B_PAGE, BIM_IMG_B_OSET, crc) )
    {
      // Load address of label __iar_program_start from the fixed location
      // of the image's reset vector,
      asm(" MOV R0, #0x9010 ");
      asm(" LDR R1, [R0, #0x4] ");

      // Reset the stack pointer,
      asm(" LDR SP, [R0, #0x0] ");

      // And jump.
      asm(" BX R1 ");

      OADTarget_systemReset();  // Should not get here.
    }
  }

#if !defined(FEATURE_FIXED_IMAGE)
  // Read Image A's CRC values from flash.
  OADTarget_readFlash(BIM_IMG_A_PAGE, BIM_IMG_A_OSET + BIM_CRC_OSET, (uint8 *)crc, 4);
  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1] || crcCheck(BIM_IMG_A_PAGE, BIM_IMG_A_OSET, crc))
#endif //FEATURE_FIXED_IMAGE
    {
      // Load address of label __iar_program_start from the fixed location
      // of the image's reset vector,
      asm(" MOV R0, #0x0610 ");
      asm(" LDR R1, [R0, #0x4] ");

      // Reset the stack pointer,
      asm(" LDR SP, [R0, #0x0] ");

      // And jump.
      asm(" BX R1 ");

      OADTarget_systemReset();  // Should not get here.
    }
#if !defined(FEATURE_FIXED_IMAGE)
  }
#endif //FEATURE_FIXED_IMAGE

  // Neither image is ready to run.  Go to sleep.
  halSleepExec();

  OADTarget_systemReset();  // Should not get here.
}


/**************************************************************************************************
*/
