/******************************************************************************

 @file  observer.h

 @brief TI BLE GAP Observer Role for RTOS Applications

        This GAP profile only discovers.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2011-2018, Texas Instruments Incorporated
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

#ifndef OBSERVER_H
#define OBSERVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "osal.h"
#include "gap.h"

/*********************************************************************
 * CONSTANTS
 */

/** @defgroup GAPOBSERVERROLE_PROFILE_PARAMETERS GAP Observer Role Parameters
 * @{
 */
#define GAPOBSERVERROLE_BD_ADDR        0x400  //!< Device's Address. Read Only. Size is uint8_t[B_ADDR_LEN]. This item is read from the controller.
#define GAPOBSERVERROLE_MAX_SCAN_RES   0x401  //!< Maximum number of discover scan results to receive. Default is 0 = unlimited.
/** @} End GAPOBSERVERROLE_PROFILE_PARAMETERS */

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/**
 * Observer Event Structure
 */
typedef union
{
  gapEventHdr_t             gap;                //!< GAP_MSG_EVENT and status.
  gapDeviceInitDoneEvent_t  initDone;           //!< GAP initialization done.
  gapDeviceInfoEvent_t      deviceInfo;         //!< Discovery device information event structure.
  gapDevDiscEvent_t         discCmpl;           //!< Discovery complete event structure.
} gapObserverRoleEvent_t;

/**
 * Observer Event Callback Function
 */
typedef uint8_t (*pfnGapObserverRoleEventCB_t)
(
  gapObserverRoleEvent_t *pEvent         //!< Pointer to event structure.
);

/**
 * Observer Callback Structure - must be setup by the application and used when
 *                               GAPObserverRole_StartDevice() is called.
 */
typedef struct
{
  pfnGapObserverRoleEventCB_t  eventCB;  //!< Event callback.
} gapObserverRoleCB_t;

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * Observer Profile Public APIs
 */

/**
 * @defgroup OBSERVER_PROFILE_API Observer Profile API Functions
 *
 * @{
 */

/**
 * @brief   Start the device in Observer role.  This function is typically
 *          called once during system startup.
 *
 * @param   pAppCallbacks - pointer to application callbacks
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          bleAlreadyInRequestedMode: Device already started.<BR>
 */
extern bStatus_t GAPObserverRole_StartDevice(gapObserverRoleCB_t *pAppCallbacks);

/**
 * @brief   Set a parameter in the Observer Profile.
 *
 * @param   param - profile parameter ID: @ref GAPOBSERVERROLE_PROFILE_PARAMETERS
 * @param   len - length of data to write
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type.
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          INVALIDPARAMETER: Invalid parameter ID.<BR>
 */
extern bStatus_t GAPObserverRole_SetParameter(uint16_t param, uint8_t len,
                                              void *pValue);
/**
 * @brief   Get a parameter in the Observer Profile.
 *
 * @param   param - profile parameter ID: @ref GAPOBSERVERROLE_PROFILE_PARAMETERS
 * @param   pValue - pointer to buffer to contain the read data
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          INVALIDPARAMETER: Invalid parameter ID.<BR>
 */
extern bStatus_t GAPObserverRole_GetParameter(uint16_t param, void *pValue);

/**
 * @brief   Start a device discovery scan.
 *
 * @param   mode - discovery mode: @ref GAP_DEVDISC_MODE_DEFINES
 * @param   activeScan - TRUE to perform active scan
 * @param   whiteList - TRUE to only scan for devices in the white list
 *
 * @return  SUCCESS: Discovery scan started.<BR>
 *          bleIncorrectMode: Invalid profile role.<BR>
 *          bleAlreadyInRequestedMode: Not available.<BR>
 */
extern bStatus_t GAPObserverRole_StartDiscovery(uint8_t mode, uint8_t activeScan,
                                                uint8_t whiteList);

/**
 * @brief   Cancel a device discovery scan.
 *
 * @return  SUCCESS: Cancel started.<BR>
 *          bleInvalidTaskID: Not the task that started discovery.<BR>
 *          bleIncorrectMode: Not in discovery mode.<BR>
 */
extern bStatus_t GAPObserverRole_CancelDiscovery(void);

/**
 * @}
 */

/*-------------------------------------------------------------------
 * TASK API - These functions must only be called by OSAL.
 */
/**
 * @brief       Task creation function for the GAP Observer Role.
 *
 * @param       none
 *
 * @return      none
 */
extern void GAPObserverRole_createTask(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_H */
