/******************************************************************************

 @file  buzzer.c

 @brief Control of the buzzer of the keyfob board in the CC26xx.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2009-2018, Texas Instruments Incorporated
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

#include "hal_mcu.h"
#include "buzzer.h"

/*********************************************************************
 * @fn      Buzzer_init
 *
 * @brief   Initial the buzzer.
 *
 * @param   None.
 *
 * @return  None.
 */
void Buzzer_init(void)
{
  // To be implemented.
}

/*********************************************************************
 * @fn      Buzzer_start
 *
 * @brief   Start the buzzer at the given frequency
 *
 * @param   frequency - the frequency in Hertz to run the buzzer at.
 *
 * @return  1 - success. 0 - failure.
 */
uint8_t Buzzer_start(uint16_t frequency)
{
  Buzzer_init();

  // To be implemented.

  return 1;
}

/*********************************************************************
 * @fn      Buzzer_stop
 *
 * @brief   Stop the buzzer.
 *
 * @param   None.
 *
 * @return  None.
 */
void Buzzer_stop(void)
{
  // To be implemented.
}


/*********************************************************************
*********************************************************************/
