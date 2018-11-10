/*******************************************************************************

 @file  sensortag_factoryreset.c

 @brief This file is the SensorTag application's factory reset functionality.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 *******************************************************************************
 
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

 *******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 ******************************************************************************/
#ifndef EXCLUDE_FACTORY_RESET

/*******************************************************************************
 * INCLUDES
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

// Sensor devices
#include "st_util.h"
#include "sensortag_batt.h"
#include "SensorUtil.h"
#include "sensortag_io.h"

// Other devices
#include "ExtFlash.h"
#include "ext_flash_layout.h"

// Application
#include "sensortag_factoryreset.h"
#include "sensortag_display.h"

/*******************************************************************************
 * LOCAL CONSTANTS
 */
#define APP_START          0x1000
#define FLASH_VECTOR_TABLE APP_START + 0x10
#define RAM_START          0x20000000
#define RAM_END            0x20005000
#define FLASH_END          0x1FFFF
#define CCFG_OFFSET        0xFA8
#define BL_OFFSET          0x1F001

#define FLASH_BUF_SIZE     256

/*******************************************************************************
 * LOCAL VARIABLES
 */
static ExtImageInfo_t imgInfo;
static uint8_t buf[FLASH_BUF_SIZE]; // Buffer for external flash read/write

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static uint16_t crc16(uint16_t crc, uint8_t val);
static uint16_t calcImageCRC(void);

/*******************************************************************************
 * @fn      SensorTagFactoryReset_applyFactoryImage
 *
 * @brief   Load the factory image from external flash and reboot
 *
 * @return  none
 */
void SensorTagFactoryReset_applyFactoryImage(void)
{
    SensorTagDisplay_suspend();

    if (SensorTagFactoryReset_hasImage())
    {
        // Indicate that factory image is launched
#ifdef IOID_GREEN_LED
        PIN_setOutputValue(hGpioPin, IOID_GREEN_LED, Board_LED_ON);
#endif
        // Load and launch factory image; page 0 and 31 must be omitted
        ((void (*)(uint32_t, uint32_t, uint32_t))BL_OFFSET)
            (EFL_ADDR_RECOVERY + APP_START, // Location in external flash
             EFL_SIZE_RECOVERY - 0x2000,    // Length
             APP_START);                    // Location in internal flash
    }
    else
    {
        // Indicate that factory image launch failed
        PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_ON);
    }

    SensorTagDisplay_resume();
}

/*******************************************************************************
 * @fn      SensorTagFactoryReset_storeCurrentImage
 *
 * @brief   Save the current image to external flash as a factory image
 *
 * @return  none
 */
bool SensorTagFactoryReset_storeCurrentImage(void)
{
  bool success;

  success = ExtFlash_open();

  if (success)
  {
    uint32_t address;
    uint16_t imageCRC = 0;

    // Install factory image
    for (address=0; address<EFL_SIZE_RECOVERY && success; address+=EFL_PAGE_SIZE)
    {
        size_t offset;
        bool ledToggle;

        ledToggle = (address % (EFL_PAGE_SIZE * 2)) == 0;

        // LED on
        if (ledToggle)
        {
#ifdef IOID_GREEN_LED
            PIN_setOutputValue(hGpioPin, IOID_GREEN_LED, Board_LED_ON);
#endif
            PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_OFF);
        }

        // Erase the page
        ExtFlash_erase(EFL_ADDR_RECOVERY + address, EFL_PAGE_SIZE);

        for (offset=0; offset<EFL_PAGE_SIZE && success; offset+=sizeof(buf))
        {
            const uint8_t *pIntFlash;
            int i;

            // Copy from internal to external flash
            pIntFlash = (const uint8_t*)address + offset;
            memcpy(buf, pIntFlash, sizeof(buf));
            success = ExtFlash_write(EFL_ADDR_RECOVERY + address + offset,
                                    sizeof(buf), buf);

            if (success)
            {
                // Add CRC
                for (i = 0; i < sizeof(buf); i++)
                {
                    imageCRC = crc16(imageCRC, buf[i]);
                }
            }
        }

        // LED off
        if (ledToggle)
        {
#ifdef IOID_GREEN_LED
            PIN_setOutputValue(hGpioPin, IOID_GREEN_LED, Board_LED_OFF);
#endif
            PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_ON);
        }
    }

    if (success)
    {
        PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_OFF);

        imageCRC = crc16(imageCRC, 0);
        imageCRC = crc16(imageCRC, 0);

        // Erase mata-data page
        ExtFlash_erase(EFL_IMAGE_INFO_ADDR_FACTORY, EFL_PAGE_SIZE);

        // Populate meta-data
        imgInfo.crc[0] = imageCRC;
        imgInfo.crc[1]= 0xFFFF;
        imgInfo.addr = 0x0000;
        imgInfo.ver = 0;
        imgInfo.len = EFL_SIZE_RECOVERY / EFL_OAD_ADDR_RESOLUTION;
        imgInfo.imgType = EFL_OAD_IMG_TYPE_FACTORY;
        imgInfo.uid[0]= 'F';
        imgInfo.uid[1]= 'F';
        imgInfo.uid[2]= 'F';
        imgInfo.uid[3]= 'F';
        imgInfo.status = 0xFF;

        // Store CRC in the meta-data region for factory image
        ExtFlash_write(EFL_IMAGE_INFO_ADDR_FACTORY, sizeof(ExtImageInfo_t),
                       (uint8_t*)&imgInfo);
    }
    else
    {
        // Erase the meta-data to invalidate factory image
        ExtFlash_erase(EFL_IMAGE_INFO_ADDR_FACTORY, EFL_PAGE_SIZE);
    }

    ExtFlash_close();
  }

  return success;
}

/*******************************************************************************
 * @fn      SensorTagFactoryReset_hasImage
 *
 * @brief   Determine if the SensorTag has a pre-programmed factory image
 *          in external flash. Criteria for deciding if a factory image:
 *
 *          - valid reset vector and stack pointer in BIM (page 0)
 *          - valid reset vector and stack pointer in application (page 1)
 *          - check that first page of stack image has been programmed
 *
 * @return  none
 */
bool SensorTagFactoryReset_hasImage(void)
{
  bool valid;

  valid = ExtFlash_open();

  if (valid)
  {
    uint32_t instr[4];

    // Check vector table for BIM
    valid = ExtFlash_read(EFL_ADDR_RECOVERY, sizeof(instr), (uint8_t*)instr);
    if (valid)
    {
        // Initial stack pointer must point to RAM
        if (instr[0] < RAM_START || instr[0] > RAM_END)
        {
            valid = false;
        }

        // Reset vector must point to a flash location in page 0 or 31
        if (instr[1] < BL_OFFSET || instr[1] > BL_OFFSET + CCFG_OFFSET)
        {
            if (instr[1] >= APP_START)
            {
                valid = false;
            }
        }
    }

    if (valid)
    {
        // Check vector table for application
        valid = ExtFlash_read(EFL_ADDR_RECOVERY + FLASH_VECTOR_TABLE,
                              sizeof(instr), (uint8_t*)instr);
    }

    if (valid)
    {
        // Initial stack pointer must point to RAM
        if (instr[0] < RAM_START || instr[0] > RAM_END)
        {
            valid = false;
        }

        // Reset vector must point to flash
        if (instr[1] < APP_START || instr[1] > FLASH_END)
        {
            valid = false;
        }
    }

    if (valid)
    {
        uint16_t imageCRC;

        // Calculate CRC of factory image on external flash
        imageCRC = calcImageCRC();

        // Compare to information stored in meta-data
        ExtFlash_read(EFL_IMAGE_INFO_ADDR_FACTORY, sizeof(ExtImageInfo_t),
                       (uint8_t*)&imgInfo);

        valid = imageCRC == imgInfo.crc[0];
    }
    else
    {
        // Erase the meta-data to invalidate factory image
        ExtFlash_erase(EFL_IMAGE_INFO_ADDR_FACTORY, EFL_PAGE_SIZE);
    }

    ExtFlash_close();
  }

  return valid;
}

/*******************************************************************************
 * @fn      SensorTagFactoryReset_extFlashErase
 *
 * @brief   Erase the external flash
 *
 * @return  none
 */
void SensorTagFactoryReset_extFlashErase(void)
{
    SensorTagDisplay_suspend();

    if (ExtFlash_open())
    {
        uint32_t address;

        // Erase entire external flash
        for (address= 0; address<EFL_FLASH_SIZE; address+=EFL_PAGE_SIZE)
        {
            bool ledToggle;

            // Toggle both LEDs for each second page
            ledToggle = (address % (EFL_PAGE_SIZE * 2)) == 0;

            // LED on
            if (ledToggle)
            {
#ifdef IOID_GREEN_LED
                PIN_setOutputValue(hGpioPin, IOID_GREEN_LED, Board_LED_ON);
#endif
                PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_OFF);
            }

            // Erase the page
            ExtFlash_erase(address,EFL_PAGE_SIZE);

            // LED off
            if (ledToggle)
            {
                PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_ON);
#ifdef IOID_GREEN_LED
                PIN_setOutputValue(hGpioPin, IOID_GREEN_LED, Board_LED_OFF);
#endif
            }
        }
        PIN_setOutputValue(hGpioPin, IOID_RED_LED, Board_LED_OFF);

        ExtFlash_close();
    }

    SensorTagDisplay_resume();
}

/*******************************************************************************
 * @fn          crc16
 *
 * @brief       Run the CRC16 Polynomial calculation over the byte parameter.
 *
 * @param       crc - Running CRC calculated so far.
 * @param       val - Value on which to run the CRC16.
 *
 * @return      crc - Updated for the run.
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
 * @fn      calcImageCRC
 *
 * @brief   Calculate the CRC across the entire factory image
 *
 * @return  16 bit CRC
 */
static uint16_t calcImageCRC(void)
{
    uint16_t imageCRC = 0;
    uint32_t address;

    // Read across the whole flash
    for (address=0; address<EFL_SIZE_RECOVERY; address+=sizeof(buf))
    {
        int i;

        ExtFlash_read(EFL_ADDR_RECOVERY + address, sizeof(buf), buf);

        // Calculate CRC of word, byte by byte.
        for (i = 0; i < sizeof(buf); i++)
        {
            imageCRC = crc16(imageCRC, buf[i]);
        }
    }

    imageCRC = crc16(imageCRC, 0);
    imageCRC = crc16(imageCRC, 0);

    return imageCRC;
}

#endif // EXCLUDE_FACTORY_RESET

/*******************************************************************************
*******************************************************************************/
