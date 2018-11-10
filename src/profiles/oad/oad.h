/******************************************************************************

 @file  oad.h

 @brief This file contains OAD Profile header file.

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
#ifndef OAD_H
#define OAD_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "hal_aes.h"
#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */

#define OAD_SERVICE_UUID      0xFFC0
#define OAD_IMG_IDENTIFY_UUID 0xFFC1
#define OAD_IMG_BLOCK_UUID    0xFFC2

#define OAD_IMG_CRC_OSET      0x0000
#if defined FEATURE_OAD_SECURE
#define OAD_IMG_HDR_OSET      0x0000
#else  // crc0 is calculated and placed by the IAR linker at 0x0, so img_hdr_t is 2 bytes offset.
#define OAD_IMG_HDR_OSET      0x0002
#endif

#define OAD_CHAR_CNT          2

// OAD Characteristic Indices
#define OAD_CHAR_IMG_IDENTIFY 0
#define OAD_CHAR_IMG_BLOCK    1

// Image Identification size
#define OAD_IMG_ID_SIZE       4

// Image header size (version + length + image id size)
#define OAD_IMG_HDR_SIZE      ( 2 + 2 + OAD_IMG_ID_SIZE )

// The Image is transported in 16-byte blocks in order to avoid using blob operations.
#define OAD_BLOCK_SIZE        16
#define OAD_BLOCKS_PER_PAGE  (HAL_FLASH_PAGE_SIZE / OAD_BLOCK_SIZE)
#define OAD_BLOCK_MAX        (OAD_BLOCKS_PER_PAGE * OAD_IMG_D_AREA)

/*********************************************************************
 * MACROS
 */

// Macros to get Image ID (LSB) and Version Number
#define OAD_IMG_ID( ver )    ( (ver) & 0x01 )
#define OAD_VER_NUM( ver )   ( (ver) >> 0x01 )

// Macro to set Image Version
#if defined (HAL_IMAGE_A)
  #define OAD_IMG_VER( ver ) ( (uint16)( (ver) << 0x01 ) )            // Clear LSB for Image A
#else
  #define OAD_IMG_VER( ver ) ( (uint16)( ( (ver) << 0x01 ) | 0x01 ) ) // Set LSB for Image B
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * TYPEDEFS
 */

// The Image Header will not be encrypted, but it will be included in a Signature.
typedef struct {
#if defined FEATURE_OAD_SECURE
  // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
  // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
  uint16 crc0;       // CRC must not be 0x0000 or 0xFFFF.
#endif
  uint16 crc1;       // CRC-shadow must be 0xFFFF.
  // User-defined Image Version Number - default logic uses simple a '!=' comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;

#if defined FEATURE_OAD_SECURE
static_assert((sizeof(img_hdr_t) == 16), "Bad SBL_ADDR_AES_HDR definition.");
static_assert(((sizeof(img_hdr_t) % KEY_BLENGTH) == 0),
                      "img_hdr_t is not an even multiple of KEY_BLENGTH");
#endif

// The AES Header must be encrypted and the Signature must include the Image Header.
typedef struct {
  uint8 signature[KEY_BLENGTH];  // The AES-128 CBC-MAC signature.
  uint8 nonce12[12];             // The 12-byte Nonce for calculating the signature.
  uint8 spare[4];
} aes_hdr_t;

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OAD_H */
