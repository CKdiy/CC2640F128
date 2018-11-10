/******************************************************************************

 @file  trng_api.h

 @brief Header for TRNG proxy for stack's interface to the TRNG driver.

 Group: WCS, LPC/BTS
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

#ifndef TRNG_API_H
#define TRNG_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */

#include <TRNGCC26XX.h>

extern uint32_t *trngDrvTblPtr;

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

// TRNG proxy index for TRNG driver API
#define TRNG_INIT               0
#define TRNG_OPEN               1
#define TRNG_GET_NUMBER         2

/*
** TRNG API Proxy
*/

#define TRNG_TABLE(index)      (*((uint32_t *)((uint32_t)trngDrvTblPtr + (uint32_t)((index)*4))))

#define TRNG_init              ((void     (*)(void))                                                                TRNG_TABLE(TRNG_INIT))
#define TRNG_open              ((void     (*)(uint8_t index))                                                       TRNG_TABLE(TRNG_OPEN))
#define TRNG_getNumber         ((uint32_t (*)(TRNGCC26XX_Handle handle, TRNGCC26XX_Params *params, int8_t *status)) TRNG_TABLE(TRNG_GET_NUMBER))

#ifdef __cplusplus
}
#endif

#endif /* TRNG_API_H */
