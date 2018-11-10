/******************************************************************************

 @file  oad_constants.h

 @brief This file contains OAD UUID and constants.

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
#ifndef OAD_CONSTANTS_H
#define OAD_CONSTANTS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */
#define OAD_SERVICE_UUID       0xFFC0
#define OAD_IMG_IDENTIFY_UUID  0xFFC1
#define OAD_IMG_BLOCK_UUID     0xFFC2
#define OAD_IMG_COUNT_UUID     0xFFC3
#define OAD_IMG_STATUS_UUID    0xFFC4

#define OAD_RESET_SERVICE_UUID 0xFFD0
#define OAD_RESET_CHAR_UUID    0xFFD1

// OAD Characteristic Indices
#define OAD_IDX_IMG_IDENTIFY   0
#define OAD_IDX_IMG_BLOCK      1
#define OAD_IDX_IMG_COUNT      2
#define OAD_IDX_IMG_STATUS     3

// Number of characteristics in the service
#define OAD_CHAR_CNT           4



#ifdef __cplusplus
}
#endif

#endif /* OAD_CONSTANTS_H */
