/******************************************************************************

 @file  ECCROMCC26XX.c

 @brief This file contains the source for the ECC in ROM Driver.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
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

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "ecc/ECCROMCC26XX.h"

/*********************************************************************
 * CONSTANTS
 */

//*****************************************************************************
//
// Union for parameters that forces 32-bit alignment on the byte array.
//
//*****************************************************************************
typedef union {
    uint8_t     byte[32];
    uint32_t    word[32 / sizeof(uint32_t)];
} PKA_EccParam256;

PKA_EccParam256 K_mont        = { .byte = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                                            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                            0xfe, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00 }};

PKA_EccParam256 K2_mont       = { .byte = { 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff,
                                            0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                            0xfd, 0xff, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00 }};

const PKA_EccParam256 a_mont  = { .byte  = { 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                             0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x04, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff }};

const PKA_EccParam256 b_mont  = { .byte  = { 0xdf, 0xbd, 0xc4, 0x29, 0x62, 0xdf, 0x9c, 0xd8,
                                             0x90, 0x30, 0x84, 0x78, 0xcd, 0x05, 0xf0, 0xac,
                                             0xd6, 0x2e, 0x21, 0xf7, 0xab, 0x20, 0xa2, 0xe5,
                                             0x34, 0x48, 0x87, 0x04, 0x1d, 0x06, 0x30, 0xdc }};

const PKA_EccParam256 gx_mont = { .byte  = { 0x3c, 0x14, 0xa9, 0x18, 0xd4, 0x30, 0xe7, 0x79,
                                             0x01, 0xb6, 0xed, 0x5f, 0xfc, 0x95, 0xba, 0x75,
                                             0x10, 0x25, 0x62, 0x77, 0x2b, 0x73, 0xfb, 0x79,
                                             0xc6, 0x55, 0x37, 0xa5, 0x76, 0x5f, 0x90, 0x18 }};

const PKA_EccParam256 gx      = { .byte  = { 0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4,
                                             0xA0, 0x33, 0xeb, 0x2d, 0x81, 0x7d, 0x03, 0x77,
                                             0xf2, 0x40, 0xa4, 0x63, 0xe5, 0xe6, 0xbc, 0xf8,
                                             0x47, 0x42, 0x2c, 0xe1, 0xf2, 0xd1, 0x17, 0x6b }};

const PKA_EccParam256 gy      = { .byte  = { 0xf5, 0x51, 0xbf, 0x37, 0x68, 0x40, 0xb6, 0xcb,
                                             0xce, 0x5e, 0x31, 0x6b, 0x57, 0x33, 0xce, 0x2b,
                                             0x16, 0x9e, 0x0f, 0x7c, 0x4a, 0xeb, 0xe7, 0x8e,
                                             0x9b, 0x7f, 0x1a, 0xfe, 0xe2, 0x42, 0xe3, 0x4f }};

// ECC Window Size.  Determines speed and workzone size of ECC operations.
// Recommended setting is 3.
#define ECC_WINDOW_SIZE                3

// Key size in uint32_t blocks.
#define ECC_UINT32_BLK_LEN(len)        (((len) + 3) / 4)

// Offset of Key field
#define ECC_KEY_OFFSET                 4

// Offset of Key Length field
#define ECC_KEY_LEN_OFFSET             0

// Total buffer size
#define ECC_BUF_TOTAL_LEN(len)         ((len) + ECC_KEY_OFFSET)

// ROM addressed of library functions
#define IMPORTDATA_rom          0x10015a35
#define EXPORTOPERAND_rom       0x10015a11
#define zSUB_rom                0x10015d5d
#define mMULT_rom               0x10015f81
#define mSUB_rom                0x1001664d
#define mADD_rom                0x10016551
#define mSET_rom                0x10015a9d
#define IMPORTMODULUS_rom       0x100159ad
#define IMPORTOPERAND_rom       0x100159d1

/*********************************************************************
 * EXTERNS
 */

// ECC ROM key generation functions.
extern uint8_t eccRom_genKeys(uint32_t *, uint32_t *, uint32_t *, uint32_t *);
extern uint8_t eccRom_genSharedSecret(uint32_t *, uint32_t *, uint32_t *,
                                      uint32_t *, uint32_t *);

// ECC ROM global window size and workzone buffer.
extern uint8_t eccRom_windowSize;
extern uint32_t *eccRom_workzone;

// ECC ROM global parameters
extern uint32_t *eccRom_param_p;
extern uint32_t *eccRom_param_r;
extern uint32_t *eccRom_param_a;
extern uint32_t *eccRom_param_b;
extern uint32_t *eccRom_param_Gx;
extern uint32_t *eccRom_param_Gy;

// NIST P-256 Curves in ROM
// Note: these are actually strings
extern uint32_t NIST_Curve_P256_p;
extern uint32_t NIST_Curve_P256_r;
extern uint32_t NIST_Curve_P256_a;
extern uint32_t NIST_Curve_P256_b;
extern uint32_t NIST_Curve_P256_Gx;
extern uint32_t NIST_Curve_P256_Gy;

/*********************************************************************
 * LOCAL VARIABLES
 */

// ECC driver semaphore used to synchronize access.
static Semaphore_Handle ECC_semaphore;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ECC_initGlobals(ECCROMCC26XX_CurveParams *pCurve);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

// typedef for function pointer
typedef uint32_t   (* fn_OneOperand_t)(uint32_t *);
typedef uint32_t   (* fn_twoOperand_t)(uint32_t *, uint32_t *);
typedef uint32_t   (* fn_threeOperand_t)(uint32_t *, uint32_t *, uint32_t *);

typedef void       (* fn_OneOperandV_t)(uint32_t *);
typedef void       (* fn_twoOperandV_t)(uint32_t *, uint32_t *);
typedef void       (* fn_threeOperandV_t)(uint32_t *, uint32_t *, uint32_t *);

/*
 *  ======== ECCROMCC26XX_init ========
 */
void ECCROMCC26XX_init(void)
{
  static uint8_t isInit = 0;
  unsigned int key;

  // Enter critical section.
  key = Hwi_disable();

  if (!isInit)
  {
    Semaphore_Params semParams;

    // Only initialize once.
    isInit = 1;

    // Setup semaphore for sequencing accesses to ECC
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    ECC_semaphore = Semaphore_create(1, &semParams, NULL);
  }

  // Exit critical section.
  Hwi_restore(key);
}

/*
 *  ======== ECCROMCC26XX_Params_init ========
 */
void ECCROMCC26XX_Params_init(ECCROMCC26XX_Params *params)
{
  // Initialize Curve to NIST P-256 with window size 3 by default.
  params->curve.keyLen      = ECCROMCC26XX_NIST_P256_KEY_LEN_IN_BYTES;
  // Initialize the workzone length according to whether only keys are
  // generated or if sign and verify are used
  params->curve.workzoneLen = ECCROMCC26XX_NIST_P256_WORKZONE_LEN_IN_BYTES;
  params->curve.windowSize  = ECC_WINDOW_SIZE;
  params->curve.param_p     = &NIST_Curve_P256_p;
  params->curve.param_r     = &NIST_Curve_P256_r;
  params->curve.param_a     = &NIST_Curve_P256_a;
  params->curve.param_b     = &NIST_Curve_P256_b;
  params->curve.param_gx    = &NIST_Curve_P256_Gx;
  params->curve.param_gy    = &NIST_Curve_P256_Gy;

  // Initialize memory functions to NULL
  params->malloc            = NULL;
  params->free              = NULL;

  // Initialize timeout to block indefinitely
  params->timeout           = BIOS_WAIT_FOREVER;

  // Initialize status
  params->status            = ECCROMCC26XX_STATUS_SUCCESS;
}

/*
 *  ======== ECCROMCC26XX_genKeys ========
 */
int8_t ECCROMCC26XX_genKeys(uint8_t *privateKey, uint8_t *publicKeyX,
                            uint8_t *publicKeyY, ECCROMCC26XX_Params *params)
{
  int8_t  status;
  uint8_t *randStrBuf;
  uint8_t *pubKeyXBuf;
  uint8_t *pubKeyYBuf;

  // Check key buffers and params.
  if (privateKey == NULL || publicKeyX == NULL || publicKeyY == NULL ||
      params == NULL || params->malloc == NULL || params->free == NULL)
  {
    // Store status.
    if (params)
    {
      params->status = ECCROMCC26XX_STATUS_ILLEGAL_PARAM;
    }

    return ECCROMCC26XX_STATUS_ILLEGAL_PARAM;
  }

  // Pend on Semaphore.
  params->status = Semaphore_pend(ECC_semaphore, params->timeout);

  // If execution returned due to a timeout
  if (!params->status)
  {
    // Store status.
    params->status = ECCROMCC26XX_STATUS_TIMEOUT;

    return ECCROMCC26XX_STATUS_TIMEOUT;
  }

  // Allocate total memory for operation: workzone and 3 buffers
  if (!(eccRom_workzone = (uint32_t *)params->malloc(params->curve.workzoneLen + ECC_BUF_TOTAL_LEN(params->curve.keyLen) * 3)))
  {
    // Post Semaphore.
    Semaphore_post(ECC_semaphore);

    // Store status.
    params->status = ECCROMCC26XX_STATUS_MALLOC_FAIL;

    return ECCROMCC26XX_STATUS_MALLOC_FAIL;
  }

  // Split allocated memory into buffers
  randStrBuf = (uint8_t *) eccRom_workzone + params->curve.workzoneLen;
  pubKeyXBuf = randStrBuf + ECC_BUF_TOTAL_LEN(params->curve.keyLen);
  pubKeyYBuf = pubKeyXBuf + ECC_BUF_TOTAL_LEN(params->curve.keyLen);

  // Initialize ECC curve and globals.
  ECC_initGlobals(&params->curve);

  // Set length of keys in words in the first word of each buffer.
  *((uint32_t *)&randStrBuf[ECC_KEY_LEN_OFFSET]) = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));
  *((uint32_t *)&pubKeyXBuf[ECC_KEY_LEN_OFFSET]) = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));
  *((uint32_t *)&pubKeyYBuf[ECC_KEY_LEN_OFFSET]) = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));

  // Copy in random string at key start offset.
  memcpy(randStrBuf + ECC_KEY_OFFSET, privateKey, params->curve.keyLen);

  // Generate ECC private/public key pair.
  // Note: the random string is an exact copy of the private key.
  status = eccRom_genKeys((uint32_t *)randStrBuf,
                          (uint32_t *)randStrBuf,
                          (uint32_t *)pubKeyXBuf,
                          (uint32_t *)pubKeyYBuf);

  // Move ECC buffer values to client buffers.
  memcpy(publicKeyX, pubKeyXBuf + ECC_KEY_OFFSET, params->curve.keyLen);
  memcpy(publicKeyY, pubKeyYBuf + ECC_KEY_OFFSET, params->curve.keyLen);

  // zero out workzone and 3 buffers as a precautionary measure.
  memset(eccRom_workzone, 0x00, params->curve.workzoneLen + params->curve.keyLen * 3);

  // Free allocated memory.
  params->free((uint8_t *)eccRom_workzone);

  // Post Semaphore.
  Semaphore_post(ECC_semaphore);

  // Map success code.
  if (((uint8_t)status) == ECCROMCC26XX_STATUS_ECDH_KEYGEN_OK)
  {
    status = ECCROMCC26XX_STATUS_SUCCESS;
  }

  // Store status.
  params->status = status;

  return status;
}

/*
 *  ======== ECCROMCC26XX_genDHKey ========
 */
int8_t ECCROMCC26XX_genDHKey(uint8_t *privateKey, uint8_t *publicKeyX,
                             uint8_t *publicKeyY, uint8_t *dHKeyX,
                             uint8_t *dHKeyY, ECCROMCC26XX_Params *params)
{
  int8_t  status;
  uint8_t *privKeyBuf;
  uint8_t *pubKeyXBuf;
  uint8_t *pubKeyYBuf;
  uint8_t *DHKeyXBuf;
  uint8_t *DHKeyYBuf;

  // Check key buffers and params.
  if (privateKey == NULL || publicKeyX == NULL || publicKeyY == NULL ||
      dHKeyX == NULL || dHKeyY == NULL || params == NULL ||
      params->malloc == NULL || params->free == NULL)
  {
    // Store status.
    if (params)
    {
      params->status = ECCROMCC26XX_STATUS_ILLEGAL_PARAM;
    }

    return ECCROMCC26XX_STATUS_ILLEGAL_PARAM;
  }

  // Pend on Semaphore.
  params->status = Semaphore_pend(ECC_semaphore, params->timeout);

  // If execution returned due to a timeout then leave here
  if (!params->status)
  {
    // Store status.
    params->status = ECCROMCC26XX_STATUS_TIMEOUT;

    return ECCROMCC26XX_STATUS_TIMEOUT;
  }

  // Allocate total memory for operation: workzone and 5 key buffers.
  if (!(eccRom_workzone = (uint32_t *)params->malloc(params->curve.workzoneLen + ECC_BUF_TOTAL_LEN(params->curve.keyLen) * 5)))
  {
    // Post Semaphore.
    Semaphore_post(ECC_semaphore);

    // Store status.
    params->status = ECCROMCC26XX_STATUS_MALLOC_FAIL;

    return ECCROMCC26XX_STATUS_MALLOC_FAIL;
  }

  // Split allocated memory into buffers
  privKeyBuf = (uint8_t *)eccRom_workzone + params->curve.workzoneLen;
  pubKeyXBuf = privKeyBuf + ECC_BUF_TOTAL_LEN(params->curve.keyLen);
  pubKeyYBuf = pubKeyXBuf + ECC_BUF_TOTAL_LEN(params->curve.keyLen);
  DHKeyXBuf  = pubKeyYBuf + ECC_BUF_TOTAL_LEN(params->curve.keyLen);
  DHKeyYBuf  = DHKeyXBuf  + ECC_BUF_TOTAL_LEN(params->curve.keyLen);

  // Initialize ECC curve and globals.
  ECC_initGlobals(&params->curve);

  // Set length of keys in words in the first word of each buffer.
  *((uint32_t *)&privKeyBuf[ECC_KEY_LEN_OFFSET]) = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));
  *((uint32_t *)&pubKeyXBuf[ECC_KEY_LEN_OFFSET]) = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));
  *((uint32_t *)&pubKeyYBuf[ECC_KEY_LEN_OFFSET]) = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));
  *((uint32_t *)&DHKeyXBuf[ECC_KEY_LEN_OFFSET])  = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));
  *((uint32_t *)&DHKeyYBuf[ECC_KEY_LEN_OFFSET])  = (uint32_t)(ECC_UINT32_BLK_LEN(params->curve.keyLen));

  // Copy input keys into buffers.
  memcpy(privKeyBuf + ECC_KEY_OFFSET, privateKey, params->curve.keyLen);
  memcpy(pubKeyXBuf + ECC_KEY_OFFSET, publicKeyX, params->curve.keyLen);
  memcpy(pubKeyYBuf + ECC_KEY_OFFSET, publicKeyY, params->curve.keyLen);

  status = ECC_VerifyPublicKeyWeierstrass(params, pubKeyXBuf, pubKeyYBuf,
                                          params->curve.keyLen);

  if( status == ECCROMCC26XX_STATUS_SUCCESS)
  {
    // If the point is on curve generate shared key
    // Generate ECC Diffie-Hellman shared secret.
    status = eccRom_genSharedSecret((uint32_t *)privKeyBuf,
                                    (uint32_t *)pubKeyXBuf,
                                    (uint32_t *)pubKeyYBuf,
                                    (uint32_t *)DHKeyXBuf,
                                    (uint32_t *)DHKeyYBuf);

    // Move ECC buffer values to client buffers.
    memcpy(dHKeyX, DHKeyXBuf + ECC_KEY_OFFSET, params->curve.keyLen);
    memcpy(dHKeyY, DHKeyYBuf + ECC_KEY_OFFSET, params->curve.keyLen);
  }

  // zero out workzone and 5 buffers as a precautionary measure.
  memset(eccRom_workzone, 0x00, params->curve.workzoneLen + params->curve.keyLen * 5);

  // Free allocated memory.
  params->free((uint8_t *)eccRom_workzone);

  // Post Semaphore.
  Semaphore_post(ECC_semaphore);

  // Map success code.
  if (((uint8_t)status) == ECCROMCC26XX_STATUS_ECDH_COMMON_KEY_OK)
  {
    status = ECCROMCC26XX_STATUS_SUCCESS;
  }

  // Store status.
  params->status = status;

  return status;
}

/*
 *  ======== ECC_initGlobals ========
 */
static void ECC_initGlobals(ECCROMCC26XX_CurveParams *pCurve)
{
  // Store client parameters into ECC ROM parameters.
  eccRom_param_p  = pCurve->param_p;
  eccRom_param_r  = pCurve->param_r;
  eccRom_param_a  = pCurve->param_a;
  eccRom_param_b  = pCurve->param_b;
  eccRom_param_Gx = pCurve->param_gx;
  eccRom_param_Gy = pCurve->param_gy;

  // Initialize window size
  eccRom_windowSize = pCurve->windowSize;
}

/*
 *  ======== ECC_ArrayAllZeros ========
 */
bool ECC_ArrayAllZeros(const uint8_t *array, uint32_t arrayLength)
{
  uint32_t i;
  uint8_t arrayBits = 0;

  // We could speed things up by comparing word-wise rather than byte-wise.
  // However, this extra overhead is inconsequential compared to running an
  // actual PKA operation. Especially ECC operations.
  for (i = 0; i < arrayLength; i++)
  {
    arrayBits |= array[i];
  }

  if (arrayBits)
  {
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}

/*
 *  ======== ECC_cmpBytes ========
 */
bool ECC_cmpBytes(uint8_t *in1, uint8_t *in2, uint8_t len)
{
  uint8_t bCnt = 0;
  do
  {
    if(in1[bCnt] != in2[bCnt])
    {
      return false;
    }
    bCnt++;
  }while(bCnt < len);
  return true;
}

/*
 *  ======== ECC_VerifyPublicKeyWeierstrass ========
 */
int8_t ECC_VerifyPublicKeyWeierstrass(ECCROMCC26XX_Params *params, const uint8_t *curvePointX, const uint8_t *curvePointY, uint32_t length)
{
  uint32_t pkaResult;
  uint32_t len = length/sizeof(uint32_t), modBufLen = length/sizeof(uint32_t)+2; // Some operation required modulus form operator of 10 bytes length

  uint32_t * ECDSA_primeMod, * ECDSA_xImp_10Bt, * ECDSA_yImp_10Bt,
           * ECDSA_primeMod_10bt, * ECDSA_temp_buff, * ECDSA_temp_buff1,
           * ECDSA_gx_mont, * ECDSA_gy_mont, * ECDSA_gx_hat, * ECDSA_gy_hat,
           * ECDSA_x2_hat, * ECDSA_x3_hat, * ECDSA_xa_hat, * ECDSA_y2_hat;

  // Use allocated workzone instead of alllocating memory
  ECDSA_primeMod       = eccRom_workzone;
  ECDSA_xImp_10Bt      = (ECDSA_primeMod + modBufLen);
  ECDSA_yImp_10Bt      = (ECDSA_xImp_10Bt + modBufLen);
  ECDSA_primeMod_10bt  = (ECDSA_yImp_10Bt + modBufLen);
  ECDSA_gx_mont        = (ECDSA_primeMod_10bt + modBufLen);
  ECDSA_gy_mont        = (ECDSA_gx_mont + modBufLen);
  ECDSA_gx_hat         = (ECDSA_gy_mont + modBufLen);
  ECDSA_gy_hat         = (ECDSA_gx_hat + modBufLen);
  ECDSA_x2_hat         = (ECDSA_gy_hat + modBufLen);
  ECDSA_x3_hat         = (ECDSA_x2_hat + modBufLen);
  ECDSA_xa_hat         = (ECDSA_x3_hat + modBufLen);
  ECDSA_y2_hat         = (ECDSA_xa_hat + modBufLen);
  ECDSA_temp_buff      = (ECDSA_y2_hat + modBufLen);
  ECDSA_temp_buff1     = (ECDSA_temp_buff + modBufLen);

  memset(eccRom_workzone, 0x00, 16*(modBufLen)*sizeof(uint32_t));

  // Load curve order as modulus
  len = ((fn_twoOperand_t)(IMPORTMODULUS_rom))(ECDSA_primeMod, eccRom_param_p);
  if(len != length/sizeof(uint32_t)) return ECCROMCC26XX_STATUS_ILLEGAL_PARAM;

  // Set prime mod
  ((void (*)(uint32_t*, uint32_t))(mSET_rom))(ECDSA_primeMod, len);

  // Verify X != 0 (not point at infinity)
  if (ECC_ArrayAllZeros(curvePointX, length))
  {
    return ECCROMCC26XX_STATUS_ECDH_X_ZERO;
  }

  // Verify Y != 0 (not point at infinity)
  if (ECC_ArrayAllZeros(curvePointY, length))
  {
    return ECCROMCC26XX_STATUS_ECDH_Y_ZERO;
  }

  // Import  X point to operand format form for range check
  ((fn_twoOperand_t)(IMPORTDATA_rom))(ECDSA_xImp_10Bt, (uint32_t *)curvePointX);

  // Import  Y point to operand format form for range check
  ((fn_twoOperand_t)(IMPORTDATA_rom))(ECDSA_yImp_10Bt, (uint32_t *)curvePointY);

  // Import  P point to operand format form for range check
  ((fn_twoOperand_t)(IMPORTDATA_rom))(ECDSA_primeMod_10bt, (uint32_t *)params->curve.param_p);

  // Check X point range by subtarcting the X-P,
  // there should be borrow and len'th byte should be set to zero
  pkaResult = ((fn_threeOperand_t)(zSUB_rom))(ECDSA_temp_buff, ECDSA_xImp_10Bt, ECDSA_primeMod_10bt);
  if((pkaResult != 1) || (ECDSA_temp_buff[len] != 0))
  {
    return ECCROMCC26XX_STATUS_ECDH_X_LARGER_THAN_PRIME;
  }

  // Clear buffer
  memset(ECDSA_temp_buff, 0x00, (modBufLen*sizeof(uint32_t)));

  // Check Y point range Y <= [P -1]
  pkaResult = ((fn_threeOperand_t)(zSUB_rom))(ECDSA_temp_buff, ECDSA_yImp_10Bt, ECDSA_primeMod_10bt);
  if((pkaResult != 1) || (ECDSA_temp_buff[len] != 0))
  {
     return ECCROMCC26XX_STATUS_ECDH_Y_LARGER_THAN_PRIME;
  }

  // No need to compute the Montgomery constant
  // Covnert point to  [X, Y] Montgomery format
#ifdef DEBUG_ECDSA
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_gx_mont, (uint32_t *)&gx.word, (uint32_t *)&K2_mont.word);
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_gy_mont, (uint32_t *)&gy.word, (uint32_t *)&K2_mont.word);
#else
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_gx_mont, (uint32_t *)ECDSA_xImp_10Bt, (uint32_t *)&K2_mont.word);  // Both X and Y points are in operand format
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_gy_mont, (uint32_t *)ECDSA_yImp_10Bt, (uint32_t *)&K2_mont.word);
#endif // ifdef DEBUG_ECDSA

  // Conmpute x^2 hat
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_x2_hat, (uint32_t *)ECDSA_gx_mont, (uint32_t *)ECDSA_gx_mont);

  // Conmpute x^3 hat
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_x3_hat, (uint32_t *)ECDSA_x2_hat, (uint32_t *)ECDSA_gx_mont);

  // Conmpute xa_hat
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_xa_hat, (uint32_t *)ECDSA_gx_mont, (uint32_t *)&a_mont.word);

  // Conmpute x3pax_hat
  memset(ECDSA_temp_buff, 0x00, sizeof(uint32_t)*10);
  ((fn_threeOperandV_t)(mADD_rom))(ECDSA_temp_buff, (uint32_t *)ECDSA_x3_hat, (uint32_t *)ECDSA_xa_hat);

  // Conmpute x3paxpb_hat
  ((fn_threeOperandV_t)(mADD_rom))(ECDSA_temp_buff, (uint32_t *)ECDSA_temp_buff, (uint32_t *)&b_mont.word);

  // Conmpute y2_hat
  ((fn_threeOperandV_t)(mMULT_rom))(ECDSA_y2_hat, (uint32_t *)ECDSA_gy_mont, (uint32_t *)ECDSA_gy_mont);

  memset(ECDSA_temp_buff1, 0x00, modBufLen*sizeof(uint32_t));

  if(ECC_cmpBytes((uint8_t *)ECDSA_y2_hat, (uint8_t *)ECDSA_temp_buff, len) != true)
  {
    return ECCROMCC26XX_STATUS_ECDH_PT_CHECK_FAIL;
  }

  return ECCROMCC26XX_STATUS_SUCCESS;
}
