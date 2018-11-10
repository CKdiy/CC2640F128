/******************************************************************************

 @file  sensortag_factoryreset.h

 @brief This file is the SensorTag application's factory reset functionality
        functionality.

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

#ifndef SENSORTAGFACTORYRESET_H
#define SENSORTAGFACTORYRESET_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdbool.h>

#ifndef EXCLUDE_FACTORY_RESET
/*******************************************************************************
 * @fn      SensorTagFactoryReset_hasImage
 *
 * @brief   Determine if the SensorTag has a pre-programmed factory image
 *          in external flash. Criteria for deciding if a factory image is
 *          a sanity check on the vector table and the first instruction of
 *          the executable.
 *
 * @return  none
 */
extern bool SensorTagFactoryReset_hasImage(void);

/*******************************************************************************
 * @fn      SensorTagFactoryReset_applyFactoryImage
 *
 * @brief   Load the factory image from external flash and reboot
 *
 * @return  none
 */
extern void SensorTagFactoryReset_applyFactoryImage(void);

/*******************************************************************************
 * @fn      SensorTagFactoryReset_storeCurrentImage
 *
 * @brief   Save the current image to external flash as factory image
 *
 * @return  none
 */
extern bool SensorTagFactoryReset_storeCurrentImage(void);

/*******************************************************************************
 * @fn      SensorTagFactoryReset_extFlashErase
 *
 * @brief   Erase the external flash
 *
 * @return  none
 */
extern void SensorTagFactoryReset_extFlashErase(void);

#else

/* The Factory Reset functionality is not available */

#define SensorTagFactoryReset_hasImage() true
#define SensorTagFactoryReset_applyFactoryImage()
#define SensorTagFactoryReset_storeCurrentImage() true
#define SensorTagFactoryReset_extFlashErase()

#endif // EXCLUDE_FACTORY_RESET

/*******************************************************************************
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAGFACTORYRESET_H */
