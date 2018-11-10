/******************************************************************************

 @file  bsp.h

 @brief Board support package header file for CC2650 SensorTag.

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
#ifndef __BSP_H__
#define __BSP_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ioc.h"
#include "driverlib/ioc.h"
#include "driverlib/gpio.h"


/******************************************************************************
* DEFINES
*/

// Board LED defines
#define BSP_IOID_LED_1               IOID_10
#define BSP_IOID_LED_2               IOID_15

  // Board buzzer defines
#define BSP_IOID_BUZZER              IOID_21

// Board sensor power control defines
#define BSP_IOID_MPU_POWER           IOID_12
#define BSP_IOID_MIC_POWER           IOID_13

// Board key defines
#define BSP_IOID_KEY_LEFT            IOID_0
#define BSP_IOID_KEY_RIGHT           IOID_4

// Board external flash defines
#define BSP_IOID_FLASH_CS            IOID_14
#define BSP_SPI_MOSI                 IOID_19
#define BSP_SPI_MISO                 IOID_18
#define BSP_SPI_CLK_FLASH            IOID_17

// Board devpack defines (LCD etc.)
#define BSP_IOID_DEVPACK_CS          IOID_20
#define BSP_SPI_CLK_DEVPACK          IOID_17

#define BSP_IOID_DEVPK_LCD_DISP      IOID_29
#define BSP_IOID_DEVPK_LCD_EXTCOMIN  IOID_22
#define BSP_IOID_DEVPK_LCD_EXTMODE   IOID_28

// Board reed relay defines
#define BSP_IOID_REED_RELAY_INT      IOID_1

// Board sensor interface
#define BSP_IOID_MPU_INT             IOID_7

#define BSP_IOD_SDA                  IOID_5
#define BSP_IOD_SCL                  IOID_6

#define BSP_IOD_SDA_HP               IOID_8
#define BSP_IOD_SCL_HP               IOID_9

// Board Light DevPack interface
#define BSP_LIGHT_IOID_WHITE         IOID_27
#define BSP_LIGHT_IOID_GREEN         IOID_23
#define BSP_LIGHT_IOID_BLUE          IOID_24
#define BSP_LIGHT_IOID_RED           IOID_25

/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* #ifndef __BSP_H__ */
