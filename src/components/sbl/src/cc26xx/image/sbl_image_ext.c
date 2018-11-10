/******************************************************************************

 @file  sbl_image_ext.c

 @brief This file contains APIs for accessing SBL image for NP when image is
        contained in external flash

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
#include <stdbool.h>

#include "sbl.h"
#include "sbl_image.h"

#include <ti/mw/extflash/ExtFlash.h>
#include "ext_flash_layout.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8_t flashIsOpen = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint8_t SBL_IMG_open( void )
{  
  // Open External Flash
  if ( ExtFlash_open() )
  {
    flashIsOpen = 1;
    return SBL_SUCCESS;
  }
  
  return SBL_FAILURE;
}

bool SBL_IMG_isValid( uint32_t hdrAddr )
{
  ExtImageInfo_t tempHdr;
  
  // Verify external flash has been opened
  if ( flashIsOpen )
  {
    // Read image header from external flash
    ExtFlash_read(hdrAddr, sizeof(ExtImageInfo_t), (uint8_t*)&tempHdr);
    
    // Verify CRCs of image are valid
    if ( tempHdr.crc[0] != 0xFFFF && tempHdr.crc[0] != 0x0000 && 
         tempHdr.crc[0] == tempHdr.crc[1] && tempHdr.status == 0xFF )
    {
      return true;
    }
  }
  
  return false; 
}

uint32_t SBL_IMG_getSize( uint32_t hdrAddr )
{
  ExtImageInfo_t tempHdr;
  
  // Verify external flash has been opened
  if ( flashIsOpen )
  {
    // Read image header from external flash
    ExtFlash_read(hdrAddr, sizeof(ExtImageInfo_t), (uint8_t*)&tempHdr);
    
    // Length in header field is number of 4B blocks
    return tempHdr.len * 4;
  }
  
  return 0;
}

uint32_t SBL_IMG_getOffset( uint32_t hdrAddr )
{
  ExtImageInfo_t tempHdr;
  
  // Verify external flash has been opened
  if ( flashIsOpen )
  {
    // Read image header from external flash
    ExtFlash_read(hdrAddr, sizeof(ExtImageInfo_t), (uint8_t*)&tempHdr);
    
    // Offset field is given in number of 4B blocks
    return tempHdr.addr * 4;
  }
  
  return 0;
}

uint8_t SBL_IMG_read( uint32_t addr, uint8_t *pBuf, uint16_t len)
{
  ExtFlash_read(addr, len, pBuf);
  return SBL_SUCCESS;
}

void SBL_IMG_close( void )
{
  flashIsOpen = 0;
  
  // Close External Flash
  ExtFlash_open();
}

