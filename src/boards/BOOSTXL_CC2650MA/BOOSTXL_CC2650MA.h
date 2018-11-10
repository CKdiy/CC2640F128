/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       BOOSTXL_CC2650MA.h
 *
 *  @brief      CC2650 Booster Pack Board Specific header file.
 *
 *  NB! This is the board file for CC2650 Booster Pack
 *
 *  ============================================================================
 */
#ifndef __BOOSTXL_CC2650MA_BOARD_H__
#define __BOOSTXL_CC2650MA_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

#define CC2650M5A // Configures RF front-end

 /* Mapping of chip I/Os to DIO (Chip specific for the CC26xx/CC13xx familiy)
 */

#define DIO0       IOID_0
#define DIO1       IOID_1
#define DIO2       IOID_2
#define DIO3       IOID_3
#define DIO4       IOID_4
#define DIO5       IOID_5
#define DIO6       IOID_6
#define DIO7       IOID_7
#define DIO8       IOID_8
#define DIO9       IOID_9
#define DIO10      IOID_10
#define DIO11      IOID_11
#define DIO12      IOID_12
#define DIO13      IOID_13
#define DIO14      IOID_14

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

/* Mapping of DIOs to BoosterPack Connector Pins (reflecting the schematic of tbe BoosterPack)
*/

/* Connector J1 */
#define Board_BP_Pin_J1_2              DIO7
#define Board_BP_Pin_J1_3              DIO0
#define Board_BP_Pin_J1_4              DIO1
#define Board_BP_Pin_J1_5              DIO2
#define Board_BP_Pin_J1_6              DIO3
#define Board_BP_Pin_J1_7              DIO10
#define Board_BP_Pin_J1_8              DIO4
#define Board_BP_Pin_J1_9              DIO5
#define Board_BP_Pin_J1_10             DIO6

/* Connector J2 */
#define Board_BP_Pin_J2_19             DIO8
#define Board_BP_Pin_J2_18             DIO9          /* CS */
#define Board_BP_Pin_J2_17             IOID_UNUSED   /* NC */
#define Board_BP_Pin_J2_15             DIO11         /* MOSI */
#define Board_BP_Pin_J2_14             DIO12         /* MISO */
#define Board_BP_Pin_J2_13             DIO13
#define Board_BP_Pin_J2_12             DIO14
#define Board_BP_Pin_J2_11             IOID_UNUSED   /* NC */

/* Mapping of BoosterPack Connector Pins to BoosterPack Standard Functions (reflecting the BoosterPack Standard)
*/

/* Connector J1 */
#define Board_BP_AnalogIn_0            Board_BP_Pin_J1_2
#define Board_BP_UART_Rx               Board_BP_Pin_J1_3 /* To MCU on LaunchPad */
#define Board_BP_UART_Tx               Board_BP_Pin_J1_4 /* From MCU on LauchPad*/
#define Board_BP_GPIO_0                Board_BP_Pin_J1_5
#define Board_BP_AnalogIn_1            Board_BP_Pin_J1_6
#define Board_BP_SPI_CLK               Board_BP_Pin_J1_7
#define Board_BP_GPIO_1                Board_BP_Pin_J1_8
#define Board_BP_I2C_SCL               Board_BP_Pin_J1_9
#define Board_BP_I2C_SDA               Board_BP_Pin_J1_10

/* Connector J2 */
#define Board_BP_PWM_0                 Board_BP_Pin_J2_19
#define Board_BP_SPI_CS_Wireless       Board_BP_Pin_J2_18
#define Board_BP_GPIO_3                Board_BP_Pin_J2_17
#define Board_BP_SPI_MOSI              Board_BP_Pin_J2_15
#define Board_BP_SPI_MISO              Board_BP_Pin_J2_14
#define Board_BP_SPI_CS_Display        Board_BP_Pin_J2_13
#define Board_BP_SPI_CS_Other          Board_BP_Pin_J2_12
#define Board_BP_GPIO_2                Board_BP_Pin_J2_11


/* Mapping of application specific functionality of the BoosterPack to BoosterPack Pins (application dependent)
*/

/* On-board LEDs */
#define Board_GLED                     DIO2                      /* Green LED */
#define Board_RLED                     DIO4                      /* Red LED */

/* UART Board */
#define Board_UART_TX                  Board_BP_UART_Rx          /* RXD */
#define Board_UART_RX                  Board_BP_UART_Tx          /* TXD */

/* SPI Board */
#define Board_SPI0_MISO                Board_BP_SPI_MISO
#define Board_SPI0_MOSI                Board_BP_SPI_MOSI
#define Board_SPI0_CLK                 Board_BP_SPI_CLK
#define Board_SPI0_CS                  Board_BP_SPI_CS_Wireless

/* Power Management Board */
#define Board_SRDY                     Board_BP_Pin_J2_19
#define Board_MRDY                     Board_BP_Pin_J1_2

/* PWM outputs */
#define Board_PWMPIN0                  PIN_UNASSIGNED
#define Board_PWMPIN1                  PIN_UNASSIGNED
#define Board_PWMPIN2                  PIN_UNASSIGNED
#define Board_PWMPIN3                  PIN_UNASSIGNED
#define Board_PWMPIN4                  PIN_UNASSIGNED
#define Board_PWMPIN5                  PIN_UNASSIGNED
#define Board_PWMPIN6                  PIN_UNASSIGNED
#define Board_PWMPIN7                  PIN_UNASSIGNED

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic SPI instance identifiers */
#define Board_SPI0                  BOOSTXL_CC2650MA_SPI0
/* Generic UART instance identifiers */
#define Board_UART                  BOOSTXL_CC2650MA_UART0
/* Generic TRNG instance identiifer */
#define Board_TRNG                  BOOSTXL_CC2650MA_TRNG0
/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             BOOSTXL_CC2650MA_GPTIMER0A
#define Board_GPTIMER0B             BOOSTXL_CC2650MA_GPTIMER0B
#define Board_GPTIMER1A             BOOSTXL_CC2650MA_GPTIMER1A
#define Board_GPTIMER1B             BOOSTXL_CC2650MA_GPTIMER1B
#define Board_GPTIMER2A             BOOSTXL_CC2650MA_GPTIMER2A
#define Board_GPTIMER2B             BOOSTXL_CC2650MA_GPTIMER2B
#define Board_GPTIMER3A             BOOSTXL_CC2650MA_GPTIMER3A
#define Board_GPTIMER3B             BOOSTXL_CC2650MA_GPTIMER3B
/* Generic PWM instance identifiers */
#define Board_PWM0                  BOOSTXL_CC2650MA_PWM0
#define Board_PWM1                  BOOSTXL_CC2650MA_PWM1
#define Board_PWM2                  BOOSTXL_CC2650MA_PWM2
#define Board_PWM3                  BOOSTXL_CC2650MA_PWM3
#define Board_PWM4                  BOOSTXL_CC2650MA_PWM4
#define Board_PWM5                  BOOSTXL_CC2650MA_PWM5
#define Board_PWM6                  BOOSTXL_CC2650MA_PWM6
#define Board_PWM7                  BOOSTXL_CC2650MA_PWM7

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    BOOSTXL_CC2650MA_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 Booster Pack
 */
typedef enum BOOSTXL_CC2650MA_CryptoName {
    BOOSTXL_CC2650MA_CRYPTO0 = 0,

    BOOSTXL_CC2650MA_CRYPTOCOUNT
} BOOSTXL_CC2650MA_CryptoName;


/*!
 *  @def    BOOSTXL_CC2650MA_SPIName
 *  @brief  Enum of SPI names on the CC2650 Booster Pack
 */
typedef enum BOOSTXL_CC2650MA_SPIName {
    BOOSTXL_CC2650MA_SPI0 = 0,

    BOOSTXL_CC2650MA_SPICOUNT
} BOOSTXL_CC2650MA_SPIName;

/*!
 *  @def    BOOSTXL_CC2650MA_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum BOOSTXL_CC2650MA_TRNGName {
    BOOSTXL_CC2650MA_TRNG0 = 0,
    BOOSTXL_CC2650MA_TRNGCOUNT
} BOOSTXL_CC2650MA_TRNGName;

/*!
 *  @def    BOOSTXL_CC2650MA_UARTName
 *  @brief  Enum of UARTs on the CC2650 Booster Pack
 */
typedef enum BOOSTXL_CC2650MA_UARTName {
    BOOSTXL_CC2650MA_UART0 = 0,

    BOOSTXL_CC2650MA_UARTCOUNT
} BOOSTXL_CC2650MA_UARTName;

/*!
 *  @def    BOOSTXL_CC2650MA_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum BOOSTXL_CC2650MA_UdmaName {
    BOOSTXL_CC2650MA_UDMA0 = 0,

    BOOSTXL_CC2650MA_UDMACOUNT
} BOOSTXL_CC2650MA_UdmaName;

/*!
 *  @def    BOOSTXL_CC2650MA_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum BOOSTXL_CC2650MA_GPTimerName
{
    BOOSTXL_CC2650MA_GPTIMER0A = 0,
    BOOSTXL_CC2650MA_GPTIMER0B,
    BOOSTXL_CC2650MA_GPTIMER1A,
    BOOSTXL_CC2650MA_GPTIMER1B,
    BOOSTXL_CC2650MA_GPTIMER2A,
    BOOSTXL_CC2650MA_GPTIMER2B,
    BOOSTXL_CC2650MA_GPTIMER3A,
    BOOSTXL_CC2650MA_GPTIMER3B,
    BOOSTXL_CC2650MA_GPTIMERPARTSCOUNT
} BOOSTXL_CC2650MA_GPTimerName;

/*!
 *  @def    BOOSTXL_CC2650MA_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum BOOSTXL_CC2650MA_GPTimers
{
    BOOSTXL_CC2650MA_GPTIMER0 = 0,
    BOOSTXL_CC2650MA_GPTIMER1,
    BOOSTXL_CC2650MA_GPTIMER2,
    BOOSTXL_CC2650MA_GPTIMER3,
    BOOSTXL_CC2650MA_GPTIMERCOUNT
} BOOSTXL_CC2650MA_GPTimers;

/*!
 *  @def    BOOSTXL_CC2650MA_PWM
 *  @brief  Enum of PWM outputs on the board
 */
typedef enum BOOSTXL_CC2650MA_PWM
{
    BOOSTXL_CC2650MA_PWM0 = 0,
    BOOSTXL_CC2650MA_PWM1,
    BOOSTXL_CC2650MA_PWM2,
    BOOSTXL_CC2650MA_PWM3,
    BOOSTXL_CC2650MA_PWM4,
    BOOSTXL_CC2650MA_PWM5,
    BOOSTXL_CC2650MA_PWM6,
    BOOSTXL_CC2650MA_PWM7,
    BOOSTXL_CC2650MA_PWMCOUNT
} BOOSTXL_CC2650MA_PWM;

#ifdef __cplusplus
}
#endif

#endif /* __BOOSTXL_CC2650MA_BOARD_H__ */
