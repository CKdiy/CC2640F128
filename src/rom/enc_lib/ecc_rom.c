/******************************************************************************

 @file  ecc_rom.c

 @brief This is the implementation for the API to the ECC module built into
        ROM on the CC26xx.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2014-2018, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "ecc_rom.h"

/*********************************************************************
 * CONSTANTS
 */

// Low level types defined in ECC module.
#ifndef __LOWLEVELTYPES__
#define __LOWLEVELTYPES__
#ifndef UTYPES
#define UTYPES
  typedef unsigned long long int  u64;
  typedef unsigned long int       u32;
  typedef unsigned short int      u16;
  typedef unsigned char           u8;
#endif
#endif

#ifdef ECC_PRIME_NIST256_CURVE
//#define TEST_NIST256
//#define PARAM_P NIST256_p;
#define PARAM_P 0x10018b0c;

//#define PARAM_R NIST256_r;
#define PARAM_R 0x10018b30;

//#define PARAM_A NIST256_a;
#define PARAM_A 0x10018b54;

//#define PARAM_B NIST256_b;
#define PARAM_B 0x10018b78;

//#define PARAM_GX NIST256_Gx;
#define PARAM_GX 0x10018b9c;

//#define PARAM_GY NIST256_Gy;
#define PARAM_GY 0x10018bc0;

#endif

/*********************************************************************
 * TYPEDEFS
 */
typedef u8 (* ecc_keygen_t)(u32 *, u32 *,u32 *, u32 *);
ecc_keygen_t ecc_generatekey = (ecc_keygen_t)(0x10017dbd);

typedef u8 (* ecdsa_sign_t)(u32 *, u32 *,u32 *, u32 *, u32 *);
ecdsa_sign_t ecc_ecdsa_sign = (ecdsa_sign_t)(0x10017969);

typedef u8 (* ecdsa_verify_t)(u32 *, u32 *,u32 *, u32 *, u32 *);
ecdsa_verify_t ecc_ecdsa_verify = (ecdsa_verify_t)(0x10017b01);

typedef u8 (* ecdh_computeSharedSecret_t)(u32 *, u32 *,u32 *, u32 *, u32 *);
ecdh_computeSharedSecret_t ecdh_computeSharedSecret = (ecdh_computeSharedSecret_t)(0x10017ded);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * Function: ECC_initialize
 *
 * Description: pass pointer to ECC memory allocation to ECC engine.
 *
 * Parameters:
 *             workZone   - pointer to memory allocated for computations, input.
 *             windowSize - operation window, determines speed vs RAM tradeoff.
 *                          2 is the most RAM efficient and slowest.  5 is
 *                          quickest but with the greatest RAM tradeoff.
 *
 * Return:      None
 */
void ECC_initialize(uint32_t *pWorkzone, uint8_t windowSize)
{
  // Initialize curve parameters
  //data_p  = (u32 *)PARAM_P;
  *((u32 **)0x20004f48) = (u32 *)PARAM_P;

  //data_r  = (u32 *)PARAM_R;
  *((u32 **)0x20004f4c) = (u32 *)PARAM_R;

  //data_a  = (u32 *)PARAM_A;
  *((u32 **)0x20004f50) = (u32 *)PARAM_A;

  //data_b  = (u32 *)PARAM_B;
  *((u32 **)0x20004fa8) = (u32 *)PARAM_B;

  //data_Gx = (u32 *)PARAM_GX;
  *((u32 **)0x20004fa0) = (u32 *)PARAM_GX;

  //data_Gy = (u32 *)PARAM_GY;
  *((u32 **)0x20004fa4) = (u32 *)PARAM_GY;

  // Initialize window size
  //win = (u8) ECC_WINDOW_SIZE;
  *((u8 *)0x20004f40) = (u8) windowSize;

  // Initialize work zone
  //workzone = (u32 *) pWorkzone;
  *((u32 **)0x20004f44) = (u32 *) pWorkzone;
}

/*
 * Function: ECC_generateKey
 *
 * Description: generates a key. This is used for both ECDH and ECDSA.
 *
 * Parameters:
 *             randString  - random string, input.
 *
 *             privateKey  - the private key, output.
 *             publicKey_x - public key X-coordinate, output.
 *             publicKey_y - public key Y-coordinate, output.
 *
 * Return: Status
 */
uint8_t ECC_generateKey(uint32_t *randString, uint32_t *privateKey,
                        uint32_t *publicKey_x, uint32_t *publicKey_y)
{
  //return (uint8_t)ECC_keyGen((u32*)randString, (u32*)privateKey,
  //                           (u32*)publicKey_x, (u32*)publicKey_y);

  return (uint8_t)ecc_generatekey((u32*)randString, (u32*)privateKey,
                                  (u32*)publicKey_x, (u32*)publicKey_y);

}

/*
 * Function: ECC_ECDH_computeSharedSecret
 *
 * Description: compute the shared secret the private key and temporary public
 *              key.
 *
 * Parameters:
 *             privateKey      - private key, input.
 *             publicKey_x     - public key X-coordinate, input.
 *             publicKey_y     - public key Y-coordinate, input.
 *
 *             sharedSecret_x  - shared secret X-coordinate, output.
 *             sharedSecret_y  - shared secret Y-coordinate, output.
 *
 * Return:     Status
 */
uint8_t ECC_ECDH_computeSharedSecret(uint32_t *privateKey,
                                     uint32_t *publicKey_x,
                                     uint32_t *publicKey_y,
                                     uint32_t *sharedSecret_x,
                                     uint32_t *sharedSecret_y)
{
  //return (uint8_t)ECDH_commonKey((u32*)privateKey, (u32*)publicKey_x,
  //                               (u32*)publicKey_y, (u32*)sharedSecret_x,
  //                               (u32*)sharedSecret_y);

  return (uint8_t)ecdh_computeSharedSecret((u32*)privateKey,
                                           (u32*)publicKey_x,
                                           (u32*)publicKey_y,
                                           (u32*)sharedSecret_x,
                                           (u32*)sharedSecret_y);
}


/*********************************************************************
*********************************************************************/
