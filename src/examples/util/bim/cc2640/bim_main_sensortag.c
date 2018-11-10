/******************************************************************************

 @file  bim_main_sensortag.c

 @brief This module contains the implementation of a
        Boot Image Manager for SensorTag2

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2014-2018, Texas Instruments Incorporated
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
#include "bsp.h"
#include "bsp_led.h"

#include "ext_flash.h"
#include "ext_flash_layout.h"
#include <driverlib/vims.h>
#include "string.h"
#include <inc/hw_aon_rtc.h>

/* -----------------------------------------------------------------------------
 *                                       Constants
 * -----------------------------------------------------------------------------
 */
#define RESET_VECTOR_OFFSET   0x14      // Determined by application linker file

#define CRC_OFFSET            0x00

#define FLASH_WORD_SIZE       4
#define FLASH_PAGE_SIZE       0x1000
#define APP_IMAGE_START_MAX   0x2000

/* -----------------------------------------------------------------------------
 *                                       Type defintions
 * -----------------------------------------------------------------------------
 */
typedef void (*voidfunc_t)(void);

/* -----------------------------------------------------------------------------
 *                                       Local Variables
 * -----------------------------------------------------------------------------
 */
static uint8_t pgBuf[EXT_FLASH_PAGE_SIZE];
static ExtImageInfo_t hdrTmp;
static uint16_t crcBim;
static uint32_t startAddress;

/* -----------------------------------------------------------------------------
 *                                       Function prototypes
 * -----------------------------------------------------------------------------
 */
static uint16_t extImgCalcCrc(uint8_t page);

static bool extImgIsValid(void);
static bool extImgLoad(void);
static bool extImgUpdated(void);
static void checkKeyExceptions(void);
static bool extInfoLoad(void);

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
 *******************************************************************************
 */
void main(void)
{
  voidfunc_t pFnStartApplication;

  // Initialise board
  bspInit(BSP_CLK_SPD_48MHZ);
  bspLedInit(BSP_LED_ALL);

  // Check keys: backdoor (right button) + factory reset (both buttons)
  checkKeyExceptions();

  // Set green LED before FW gets updated and application start
  bspLedSet(BSP_LED_2);

  // Safe start-up values
  crcBim = 0;
  startAddress = APP_IMAGE_START;

  // Check external flash image validity
  if (extImgIsValid())
  {
    // Check if external image is newer than current image
    if (extImgUpdated())
    {
      // Boot from serial flash
      if (!extImgLoad())
      {
        // Lit the red LED to indicate failure
        bspLedSet(BSP_LED_1);
        // Serious trouble - the red LED stays on
        // TBD: blink for a while - then enter low power mode
        while(1);
      }
    }
  }

  // Start application
  // pull address of label __iar_program_start from the
  // fixed location of the image's interrupt vectors and jump.
  memcpy(&pFnStartApplication,(void*)(startAddress + RESET_VECTOR_OFFSET),
             sizeof(pFnStartApplication));

  // Sanity check to avoid execution in empty flash
  if (pFnStartApplication != (voidfunc_t)(0xFFFFFFFF))
  {
    pFnStartApplication();
  }

  // Should never get here
  bspLedSet(BSP_LED_1);
  while(1);
}


/*******************************************************************************
 * @fn          extImgIsValid
 *
 * @brief       Check if the image stored on external flash is valid.
 *
 * @return      true if valid
 ******************************************************************************/
static bool extImgIsValid(void)
{
  bool ret;
  uint16_t page;
  size_t addr;

  if (!extFlashOpen())
  {
    bspLedSet(BSP_LED_1);
    return false;
  }

  // Read external image header
  extInfoLoad();

  // Only used for CRC check
  addr = (hdrTmp.addr * EFL_OAD_ADDR_RESOLUTION);
  page = (addr) / FLASH_PAGE_SIZE;

  ret = false;

  // Do CRC check first
  crcBim = extImgCalcCrc(page);

  if (crcBim == hdrTmp.crc[0] && hdrTmp.uid[0] != 0xFF && hdrTmp.uid[1] != 0xFF)
  {
    uint32_t nBytes;

    nBytes = (hdrTmp.len * FLASH_WORD_SIZE);

    ret = nBytes >= 0x1000 || nBytes < 0x20000;
    if (ret)
    {
      if (addr>APP_IMAGE_START && addr<=APP_IMAGE_START_MAX)
      {
        startAddress = addr;
      }
    }
  }

  extFlashClose();

  return ret;
}


/*******************************************************************************
 * @fn          extImgUpdated
 *
 * @brief       Check if the image stored on external flash is newer than
 *              the current image. This function assumes that the external
 *              image has been validated.
 *
 * @return      true if valid
 ******************************************************************************/
static bool extImgUpdated(void)
{
  bool ret;

  if (!extFlashOpen())
  {
    bspLedSet(BSP_LED_1);
    return false;
  }

  // Consider a new image if BIM has not yet calculated CRC
  ret = hdrTmp.crc[1] == 0xFFFF;

  if (ret)
  {
    // Write the calculated image back to external flash
    hdrTmp.crc[1] = crcBim;
    extFlashWrite(EFL_IMAGE_INFO_ADDR_APP+2,sizeof(hdrTmp.crc[1]),
                  (uint8_t*)&hdrTmp.crc[1]);
  }

  extFlashClose();

  return ret;
}

/*******************************************************************************
 * @fn          extImgLoad
 *
 * @brief       Load image from external flash and store it in on-chip flash
 *
 * @return      true if valid
 *******************************************************************************
 */
static bool extImgLoad(void)
{
  bool ret;

  size_t addr;
  size_t nPages;
  size_t nBytes;

  // Check if the image only contains the BIM
  nBytes = hdrTmp.len * FLASH_WORD_SIZE;
  nPages = nBytes / EXT_FLASH_PAGE_SIZE;
  addr = hdrTmp.addr * EFL_OAD_ADDR_RESOLUTION;
  if (addr == 0)
  {
    nPages -= (APP_IMAGE_START/FLASH_PAGE_SIZE);
    if (nPages == 0)
    {
      // BIM only, nothing to do
      return true;
    }
  }

  ret = extFlashOpen();

  if (ret)
  {
    uint16_t firstPage;

    // Fetch and program all pages
    addr = startAddress;
    firstPage = startAddress / FLASH_PAGE_SIZE;

    // Disable cache
    ROM_VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);

    // Wait for disabling to be complete
    while (VIMSModeGet( VIMS_BASE ) != VIMS_MODE_DISABLED);

    for (uint16_t pg = firstPage; pg < (nPages + firstPage) && ret; pg++)
    {
      // Get external flash page
      ret = extFlashRead(addr, EXT_FLASH_PAGE_SIZE, pgBuf);

      if (ret)
      {
        // Erase the page of internal flash
        HapiSectorErase(addr);

        // Write page of internal flash
        HapiProgramFlash(pgBuf, addr, EXT_FLASH_PAGE_SIZE);
      }

      addr += EXT_FLASH_PAGE_SIZE;
    }

    // enable iCache prefetching
    ROM_VIMSConfigure(VIMS_BASE, 1, 1);

    // Re-enable cache
    ROM_VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

    extFlashClose();
  }

  return ret;
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
 *******************************************************************************
 */
static uint16_t crc16(uint16_t crc, uint8_t val)
{
  const uint16_t poly = 0x1021;
  uint8_t cnt;

  for (cnt = 0; cnt < 8; cnt++, val <<= 1)
  {
    uint8_t msb = (crc & 0x8000) ? 1 : 0;

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
 * @fn          extImgCalcCrc
 *
 * @brief       Run the CRC16 Polynomial calculation over the image specified.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 ******************************************************************************/
static uint16_t extImgCalcCrc(uint8_t page)
{
  uint16_t crc = 0;
  size_t addr = page * FLASH_PAGE_SIZE;

  extFlashRead(addr, FLASH_PAGE_SIZE, pgBuf);

  const ExtImageInfo_t *pImgHdr = &hdrTmp;

  uint8_t pageBeg = page;
  uint8_t pageEnd = pImgHdr->len / (FLASH_PAGE_SIZE / FLASH_WORD_SIZE);
  uint16_t osetEnd = (pImgHdr->len -
                      (pageEnd * (FLASH_PAGE_SIZE / FLASH_WORD_SIZE)))
                       * FLASH_WORD_SIZE;

  // Set pageEnd to the end page of the OAD range.
  pageEnd += pageBeg;

  while(1)
  {
    uint16_t oset;

    for (oset = 0; oset < FLASH_PAGE_SIZE; oset++)
    {
      if ((page == pageBeg) && (oset == CRC_OFFSET))
      {
        // Skip the CRC and shadow.
        // Note: this increments by 3 because oset is incremented by 1
        // in each pass through the loop.
        oset += 3;
      }
      else if ((page == pageEnd) && (oset == osetEnd))
      {
        // IAR note explains that poly must be run with value zero for each byte
        crc = crc16(crc, 0);
        crc = crc16(crc, 0);

        return crc;
      }
      else
      {
        crc = crc16(crc, pgBuf[oset]);
      }
    }

    // Start CRC check on the next page.
    page++;
    addr = page * FLASH_PAGE_SIZE;
    extFlashRead(addr,FLASH_PAGE_SIZE,pgBuf);
  }
}

/*******************************************************************************
 * @fn          checkKeyExceptions
 *
 * @brief       Check the keys for determine emergency action:
 *              - right key only; backdoor
*               - left key; factory reset
 *
 * @return      None.
 *******************************************************************************
 */
static void checkKeyExceptions(void)
{
  uint32_t t = GPIOPinRead(BSP_KEY_ALL);

  // Backdoor = RIGHT KEY - allows connection via debugger
  if ( (t & (BSP_KEY_RIGHT | BSP_KEY_LEFT)) == BSP_KEY_LEFT)
  {
    while(1);
  }
}

/*******************************************************************************
 * @fn          extInfoLoad
 *
 * @brief       Load infromation about the image (stored in external flash)
 *
 * @return      true if flahs read succeeds
 *******************************************************************************
 */
static bool extInfoLoad(void)
{
  return extFlashRead(EFL_IMAGE_INFO_ADDR_APP,sizeof(ExtImageInfo_t),
                      (uint8_t*)&hdrTmp);
}


/*******************************************************************************
*/
