/*
 * Copyright (c) 2014-2016, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */

#include <xdc/std.h>
#include <stdbool.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>

#include "sbl_internal.h"
#include "sbl.h"

#include "board.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

//! \brief PIN Config for reset and bl signals without PIN IDs
static PIN_Config sblPinsCfg[] =
{
    PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

uint32_t rstPIN = (IOID_UNUSED & IOC_IOID_MASK);
uint32_t blPIN = (IOID_UNUSED & IOC_IOID_MASK);

//! \brief PIN State for reset and boot loader pins
static PIN_State sblPins;

//! \brief PIN Handles for reset and boot loader pins
PIN_Handle hsblPins = NULL;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/**
 * @fn      SBL_initPins
 *
 * @brief   set and open reset and bl pins.
 *
 * @param   params - pins to use
 *
 * @return  uint8_t - SBL_SUCCESS, SBL_FAILURE
 */
uint8_t SBL_initPins(SBL_Params *params)
{
  // Assign PIN IDs to reset and bl
  rstPIN = (params->resetPinID & IOC_IOID_MASK);
  blPIN = (params->blPinID & IOC_IOID_MASK);

  // Add PIN IDs to PIN Configuration
  sblPinsCfg[RST_PIN_IDX] |= rstPIN;
  sblPinsCfg[BL_PIN_IDX] |= blPIN;

  // Initialize SBL Pins
  hsblPins = PIN_open(&sblPins, sblPinsCfg);
  if (hsblPins == NULL)
  {
    return SBL_FAILURE;
  }

  return SBL_SUCCESS;
}

/**
 * @fn      SBL_closePins
 *
 * @brief   invalidate and close reset and bl pins.
 *
 * @param   params - pins to use
 *
 * @return  none
 */
void SBL_closePins(void)
{
  // Clear SBL PIN IDs
  rstPIN = (IOID_UNUSED & IOC_IOID_MASK); // Set to 0x000000FF
  blPIN = (IOID_UNUSED & IOC_IOID_MASK); // Set to 0x000000FF

  // Clear PIN IDs from PIN Configuration
  sblPinsCfg[RST_PIN_IDX] &= ~rstPIN;
  sblPinsCfg[BL_PIN_IDX] &= ~blPIN;

  // Close PIN Handle
  PIN_close(hsblPins);
  hsblPins = NULL;
}