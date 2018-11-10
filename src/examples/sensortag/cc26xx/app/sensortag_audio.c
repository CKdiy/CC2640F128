/******************************************************************************

 @file  sensortag_audio.c

 @brief This file contains the Sensor Tag sample application,
        audio (voice control) sub-application.

 Group: WCS, BTS
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
#ifndef EXCLUDE_AUDIO

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/drivers/pdm/PDMCC26XX.h>

#include "hci.h"
#include "gatt.h"
#include "gattservapp.h"
#include "audio_profile.h"

#include "sensortag.h"
#include "sensortag_audio.h"
#include "board.h"
#include "util.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */

// Compatibility with HID Advanced Remote code
#define harPinHandle hGpioPin
#define HidDev_StartIdleTimer()
#define HidDev_StopIdleTimer()
#define Board_LED_R Board_LED1

/*********************************************************************
 * CONSTANTS
 */

// Internal Events for RTOS application
#define HAR_STATE_CHANGE_EVT                  0x0001
#define HAR_CONN_EVENT_EVT                    0x0002
#define HAR_START_STREAMING_EVT               0x0004
#define HAR_STOP_STREAMING_EVT                0x0008
#define HAR_SEND_STOP_CMD_EVT                 0x0010
#define HAR_PROCESS_PDM_DATA_EVT              0x0020
#define HAR_KEY_PRESS_EVT                     0x0040

#define BLE_AUDIO_CMD_STOP                    0x00
#define BLE_AUDIO_CMD_START                   0x04

#define RAS_DATA_TIC1_CMD                     0x01

#define HAR_AUDIO_MAX_ALLOC_BUF               10
#define HAR_MIC_KEY_RELEASE_TIME              500
#define HAR_STREAM_LIMIT_TIME                 30000

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// events flag for internal application events.
static uint16_t events;

// PDM parameters
static PDMCC26XX_Handle pdmHandle;

static void HIDAdvRemote_pdmCB(PDMCC26XX_Handle handle,
                              PDMCC26XX_StreamNotification *streamNotification);

static void *HIDAdvRemote_audioMalloc(uint_least16_t size);
static void HIDAdvRemote_audioFree(void *msg);

PDMCC26XX_Params pdmParams =
{
  .callbackFxn = HIDAdvRemote_pdmCB,
  .useDefaultFilter = true,
  .decimationFilter = NULL,
  .micGain = PDMCC26XX_GAIN_18,
  .micPowerActiveHigh = true,
  .applyCompression = true,
  .startupDelayWithClockInSamples = 512,
  .retBufSizeInBytes = BLEAUDIO_HDRSIZE + BLEAUDIO_BUFSIZE,
  .mallocFxn = (PDMCC26XX_MallocFxn) HIDAdvRemote_audioMalloc,
  .freeFxn = (PDMCC26XX_FreeFxn) HIDAdvRemote_audioFree,
  .custom = NULL
};

// Counter for number of PDM buffers
static uint8_t harAudioBufCount = 0;

// Mic key release timer
static Clock_Struct micKeyReleaseClock;

// Stream limit timer
static Clock_Struct streamLimitClock;

// Lock for PDM streaming
volatile static uint8_t pdmLock = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t HIDAdvRemote_transmitAudioStreamCmd(uint8_t cmd);
static void HIDAdvRemote_startStreamingVoice(void);
static void HIDAdvRemote_processPdmData(void);
static void HIDAdvRemote_transmitAudioFrame(uint8_t *buf);
static void HIDAdvRemote_stopStreamingVoice(void);
static void HIDAdvRemote_sendStopCmd(gaprole_States_t gapState);
static void HIDAdvRemote_finishStream(void);
static void HIDAdvRemote_micKeyReleaseCB(UArg a0);
static void HIDAdvRemote_signalStopStreamEvt(void);
static void HIDAdvRemote_streamLimitCB(UArg a0);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      SensorTagAudio_init
 *
 * @brief   Initialize audio interface
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagAudio_init(void)
{
  pdmHandle = NULL;

  // Initialize PDM Driver
  PDMCC26XX_init((PDMCC26XX_Handle) &(PDMCC26XX_config));

  // Add service
  Audio_AddService();

  // Initialize mic key release timer (500ms)
  Util_constructClock(&micKeyReleaseClock, HIDAdvRemote_micKeyReleaseCB,
                      HAR_MIC_KEY_RELEASE_TIME, 0, false, NULL);

  // Initialize stream limit timer (30 sec)
  Util_constructClock(&streamLimitClock, HIDAdvRemote_streamLimitCB,
                      HAR_STREAM_LIMIT_TIME, 0, false, NULL);

  // Initialize the module state variables
  SensorTagAudio_reset();
}

/*********************************************************************
 * @fn      SensorTagAudio_enableStreaming
 *
 * @brief   Control start and stop of streaming
 *
 * @param   enable - true if start, false if stop
 *
 * @return  none
 */
void SensorTagAudio_enableStreaming(bool enable)
{
    // set event
    if (enable)
    {
        events |= HAR_START_STREAMING_EVT;
    }
    else
    {
        events |= HAR_STOP_STREAMING_EVT;
    }
    // Simply post semaphore to make task run
    Semaphore_post(sem);
}


/*********************************************************************
 * @fn      SensorTagAudio_reset
 *
 * @brief   Reset characteristics and stop adudio streaming if started
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagAudio_reset(void)
{
    // Make sure audio streaming stops
    if (pdmHandle != NULL)
    {
        HIDAdvRemote_stopStreamingVoice();
    }

    events = 0;
}


/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      SensorTagAudio_processEvent
 *
 * @brief   SensorTag Audio event processor.
 *
 * @return  none
 */
void SensorTagAudio_processEvent(void)
{
    uint32_t hwiKey;
    volatile uint16_t localEvents = 0;
    volatile static uint8_t pdmStream = FALSE;

    // Handle events one at a time
    hwiKey = Hwi_disable();
    localEvents = events;
    events = 0;
    Hwi_restore(hwiKey);

    if (localEvents & HAR_START_STREAMING_EVT)
    {
        if ((!pdmStream) && (gapProfileState == GAPROLE_CONNECTED))
        {
            pdmStream = TRUE;
            HIDAdvRemote_startStreamingVoice();
        }
    }

    if (localEvents & HAR_PROCESS_PDM_DATA_EVT)
    {
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        if (pdmStream)
        {
          HIDAdvRemote_processPdmData();
        }
      }
      else
      {
        hwiKey = Hwi_disable();
        events |= HAR_STOP_STREAMING_EVT;
        Hwi_restore(hwiKey);
        Semaphore_post(sem);
      }
    }

    if (localEvents & HAR_STOP_STREAMING_EVT)
    {
      if (pdmStream)
      {
        HIDAdvRemote_stopStreamingVoice();
      }
    }

    if (localEvents & HAR_SEND_STOP_CMD_EVT)
    {
      HIDAdvRemote_sendStopCmd(gapProfileState);
      pdmStream = FALSE;
    }
}

/*********************************************************************
 * @fn      HIDAdvRemote_transmitAudioStreamCmd
 *
 * @brief   Transmits GATT Notification in order to start or stop stream
 *
 * @param   cmd - command to transmit
 *
 * @return  SUCCESS if successful, FAILURE if not
 */
static uint8_t HIDAdvRemote_transmitAudioStreamCmd(uint8_t cmd)
{
  return Audio_SetParameter(AUDIOPROFILE_START, AUDIOPROFILE_CMD_LEN, &cmd);
}

/*********************************************************************
 * @fn      HIDAdvRemote_audioMalloc
 *
 * @brief   allocates a audio block
 *
 * @param   size - size of the block in bytes
 *
 * @return  pointer to buffer if successful, NULL if failed
 */
static void *HIDAdvRemote_audioMalloc(uint_least16_t size)
{
  uint8_t *rtnAddr = NULL;
  uint_least32_t taskKey;
  uint8_t flag = 0;
  uint32_t hwiKey;

  /* Check number of simultaneous allocated buffers; do not allow more than
   * HAR_AUDIO_MAX_ALLOC_BUF simultaneous allocated buffers
   */
  taskKey = Task_disable();

  if (harAudioBufCount < HAR_AUDIO_MAX_ALLOC_BUF)
  {
    flag = 1;
    harAudioBufCount++;
  }
  else
  {
    // Reached maximum buffer size; must process data
    hwiKey = Hwi_disable();
    events |= HAR_PROCESS_PDM_DATA_EVT;
    Hwi_restore(hwiKey);
    Semaphore_post(sem);
  }

  Task_restore(taskKey);

  if (flag)
  {
    rtnAddr = (uint8_t *) ICall_malloc(size);
  }

  return rtnAddr;
}

/*********************************************************************
 * @fn      HIDAdvRemote_audioFree
 *
 * @brief   Frees an allocated audio block
 *
 * @param   msg - pointer to a memory block to free
 *
 * @return  None.
 */
static void HIDAdvRemote_audioFree(void *msg)
{
  uint8_t *pBuf = (uint8_t *)msg;
  uint_least32_t taskKey;

  taskKey = Task_disable();
  harAudioBufCount--;
  Task_restore(taskKey);

  ICall_free((void *) pBuf);
}

/*********************************************************************
 * @fn      HIDAdvRemote_startStreamingVoice
 *
 * @brief   Starts streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_startStreamingVoice(void)
{
  // LED on while streaming
  PIN_setOutputValue(harPinHandle, Board_LED_R, 1);

  // Stop HID Idle timer during stream
  HidDev_StopIdleTimer();

  // Increase TX power during stream
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  /* Send the start cmd. If it doesn't go through, return value is not SUCCESS,
   * and state should not be changed to STREAMING.
   */
  if (HIDAdvRemote_transmitAudioStreamCmd(BLE_AUDIO_CMD_START) == SUCCESS)
  {
    // Open PDM driver
    if (!pdmHandle)
    {
      pdmHandle = PDMCC26XX_open(&pdmParams);

      if (pdmHandle)
      {
        Util_restartClock(&streamLimitClock, HAR_STREAM_LIMIT_TIME);
        PDMCC26XX_startStream(pdmHandle);
      }
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_processPdmData
 *
 * @brief   Processes data from PDM driver
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_processPdmData(void)
{
  uint32_t hwiKey;
  static PDMCC26XX_BufferRequest bufferRequest;
  uint8_t *pAudioFrame = NULL;
  uint8_t tmpSeqNum;

  // Request new audio frame / buffer
  if (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest))
  {
    pAudioFrame = ((uint8 *) (bufferRequest.buffer));

    // First audio frame byte: 5 bits seq num, 3 bits data cmd
    tmpSeqNum = (((PDMCC26XX_pcmBuffer *)pAudioFrame)->metaData).seqNum;
    pAudioFrame[0] = (((tmpSeqNum % 32) << 3) | RAS_DATA_TIC1_CMD);

    // Transmit processed audio frame
    HIDAdvRemote_transmitAudioFrame(pAudioFrame);

    // Free audio frame
    HIDAdvRemote_audioFree(pAudioFrame);

    // Process additional audio frames if available
    hwiKey = Hwi_disable();
    if (harAudioBufCount)
    {
      events |= HAR_PROCESS_PDM_DATA_EVT;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
    }
    else
    {
      Hwi_restore(hwiKey);
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_transmitAudioFrame
 *
 * @brief   Transmits processed audio frame to connected device
 *
 * @param   buf - pointer to PDM buffer
 *
 * @return  None.
 */
static void HIDAdvRemote_transmitAudioFrame(uint8_t *buf)
{
  // Send 5 GATT notifications for every audio frame
  for (int i = 0; i < BLEAUDIO_NUM_NOT_PER_FRAME; )
  {
    if (Audio_SetParameter(AUDIOPROFILE_AUDIO, BLEAUDIO_NOTSIZE, buf) == SUCCESS)
    {
      // Move on to next section of audio frame
      buf += BLEAUDIO_NOTSIZE;
      i++;
    }
  }
}

/*********************************************************************
 * @fn      HIDAdvRemote_stopStreamingVoice
 *
 * @brief   Stops streaming audio to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_stopStreamingVoice(void)
{
  uint32_t hwiKey;

  if (pdmHandle)
  {
    // stop stream
    PDMCC26XX_stopStream(pdmHandle);

    // close driver
    PDMCC26XX_close(pdmHandle);
    pdmHandle = NULL;
  }

  hwiKey = Hwi_disable();
  events |= HAR_SEND_STOP_CMD_EVT;
  Hwi_restore(hwiKey);
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HIDAdvRemote_sendStopCmd
 *
 * @brief   Sends a stop command to connected device
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_sendStopCmd(gaprole_States_t gapState)
{
  uint32_t hwiKey;

  if (gapState == GAPROLE_CONNECTED)
  {
    if (HIDAdvRemote_transmitAudioStreamCmd(BLE_AUDIO_CMD_STOP) == SUCCESS)
    {
      HIDAdvRemote_finishStream();
    }
    else
    {
      hwiKey = Hwi_disable();
      events |= HAR_SEND_STOP_CMD_EVT;
      Hwi_restore(hwiKey);
      Semaphore_post(sem);
    }
  }
  else
  {
    HIDAdvRemote_finishStream();
  }
}


/*********************************************************************
 * @fn      HIDAdvRemote_finishStream
 *
 * @brief   Helper function that finishes process of streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_finishStream(void)
{
  uint32_t hwiKey;

  // LED off
  PIN_setOutputValue(harPinHandle, Board_LED_R, 0);

  // Restart HID Idle timer after stream
  HidDev_StartIdleTimer();

  // Reset TX power
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);

  // Clear event
  hwiKey = Hwi_disable();
  events &= ~HAR_SEND_STOP_CMD_EVT;
  Hwi_restore(hwiKey);

  // Temporarily disable streaming
  pdmLock = TRUE;
  Util_restartClock(&micKeyReleaseClock, HAR_MIC_KEY_RELEASE_TIME);
}

/*********************************************************************
 * @fn      HIDAdvRemote_pdmCB
 *
 * @brief   Callback function for the PDM driver
 *
 * @param   handle - handle of PDM object
 *          pStreamNotification - notification of available buffers
 *                                and potential errors
 *
 * @return  None.
 */
static void HIDAdvRemote_pdmCB(PDMCC26XX_Handle handle,
                              PDMCC26XX_StreamNotification *pStreamNotification)
{
  events |= HAR_PROCESS_PDM_DATA_EVT;

  if (pStreamNotification->status != PDMCC26XX_STREAM_BLOCK_READY)
  {
    events |= HAR_STOP_STREAMING_EVT;
  }

  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HIDAdvRemote_micKeyReleaseCB
 *
 * @brief   Enables voice streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_micKeyReleaseCB(UArg a0)
{
  // Unlock variable - allow PDM streaming to happen again
  pdmLock = FALSE;
}


/*********************************************************************
 * @fn      HIDAdvRemote_signalStopStreamEvt
 *
 * @brief   TODO
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_signalStopStreamEvt(void)
{
  uint32_t hwiKey;

  hwiKey = Hwi_disable();
  events |= HAR_STOP_STREAMING_EVT;
  Hwi_restore(hwiKey);
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HIDAdvRemote_streamLimitCB
 *
 * @brief   Limits time of voice streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void HIDAdvRemote_streamLimitCB(UArg a0)
{
  HIDAdvRemote_signalStopStreamEvt();
}

#endif // EXCLUDE_AUDIO

/*********************************************************************
*********************************************************************/
