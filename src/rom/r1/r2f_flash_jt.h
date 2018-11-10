/******************************************************************************

 @file  r2f_flash_jt.h

 @brief This file contains the defines for every flash function call using the
        ROM-to-Flash Flash Jump Table.

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

#ifndef R2F_FLASH_JT_H
#define R2F_FLASH_JT_H

#if defined( ROM_BUILD ) //|| defined( FLASH_ROM_BUILD )

/*******************************************************************************
 * EXTERNS
 */

// ROM's RAM table for pointers to ICall functions and flash jump tables.
// Note: This linker imported symbol is treated as a variable by the compiler.
// 0: iCall Dispatch Function Pointer
// 1: iCall Enter Critical Section Function Pointer
// 2: iCall Leave Critical Section Function Pointer
// 3: R2F Flash Jump Table Pointer
// 4: R2R Flash Jump Table Pointer
extern uint32 RAM_BASE_ADDR[];

/*******************************************************************************
 * INCLUDES
 */

#include "hal_types.h"
#include "ll_config.h"
#include <driverlib/rom_crypto.h>
/*******************************************************************************
 * CONSTANTS
 */

// ROM's RAM table offset to R2F flash jump table pointer.
#define ROM_RAM_TABLE_R2F          3

// Defines used for the flash jump table routines that are not part of build.
// Note: Any change to this table must accompany a change to R2F_Flash_JT[]!
#define R2F_JT_LOCATION            (&RAM_BASE_ADDR[ROM_RAM_TABLE_R2F])

#define R2F_JT_BASE                (*((uint32 **)R2F_JT_LOCATION))
#define R2F_JT_OFFSET(index)       (*(R2F_JT_BASE+(index)))

// ROM-to-Flash Functions
#define MAP_osal_mem_alloc               ((void   *(*) (uint16))                                                 R2F_JT_OFFSET(0))
#define MAP_osal_mem_free                ((void    (*) (void *))                                                 R2F_JT_OFFSET(1))
#define MAP_osal_pwrmgr_task_state       ((uint8   (*) (uint8, uint8))                                           R2F_JT_OFFSET(2))
#define MAP_osal_msg_allocate            ((uint8  *(*) (uint16))                                                 R2F_JT_OFFSET(3))
#define MAP_osal_msg_send                ((uint8   (*) (uint8, uint8 *))                                         R2F_JT_OFFSET(4))
#define MAP_osal_set_event               ((uint8   (*) (uint8, uint16))                                          R2F_JT_OFFSET(5))
#define MAP_osal_memcpy                  ((void   *(*) (void *, const void *, uint32))                           R2F_JT_OFFSET(6))
#define MAP_osal_memset                  ((void   *(*) (void *, uint8, int))                                     R2F_JT_OFFSET(7))
#define MAP_osal_bm_alloc                ((void   *(*) (uint16))                                                 R2F_JT_OFFSET(8))
#define MAP_osal_bm_free                 ((void    (*) (void *))                                                 R2F_JT_OFFSET(9))
#define MAP_osal_bm_adjust_header        ((void   *(*) (void *, int16))                                          R2F_JT_OFFSET(10))
#define MAP_osal_start_timerEx           ((uint8   (*) (uint8, uint16, uint32))                                  R2F_JT_OFFSET(11))
#define MAP_osal_stop_timerEx            ((uint8   (*) (uint8, uint16))                                          R2F_JT_OFFSET(12))
#define MAP_osal_clear_event             ((uint8   (*) (uint8, uint16))                                          R2F_JT_OFFSET(13))
#define Onboard_soft_reset               ((void    (*) (void))                                                   R2F_JT_OFFSET(14))
#define MAP_IntMasterEnable              ((uint8   (*) (void))                                                   R2F_JT_OFFSET(15))
#define MAP_IntMasterDisable             ((uint8   (*) (void))                                                   R2F_JT_OFFSET(16))
#define MAP_IntEnable                    ((void    (*) (uint32))                                                 R2F_JT_OFFSET(17))
#define MAP_IntDisable                   ((void    (*) (uint32))                                                 R2F_JT_OFFSET(18))
#define halAssertHandler                 ((void    (*) (void))                                                   R2F_JT_OFFSET(19))
#define MAP_HalTRNG_InitTRNG             ((void    (*) (void))                                                   R2F_JT_OFFSET(20))
#define MAP_HalTRNG_GetTRNG              ((uint32  (*) (void))                                                   R2F_JT_OFFSET(21))
#define MAP_LL_PM_Init                   ((void    (*) (void))                                                   R2F_JT_OFFSET(22))
#define MAP_LL_PM_GetRfCoreState         ((uint8   (*) (void))                                                   R2F_JT_OFFSET(23))
#define MAP_LL_PM_StartRfTask            ((void    (*) (taskInfo_t *))                                           R2F_JT_OFFSET(24))
#define MAP_LL_PM_PowerOnReq             ((void    (*) (void))                                                   R2F_JT_OFFSET(25))
#define MAP_LL_PM_PowerCycleRadio        ((void    (*) (void))                                                   R2F_JT_OFFSET(26))
#define MAP_LL_PM_ForceSysBusThroughRF   ((void    (*) (void))                                                   R2F_JT_OFFSET(27))
#define MAP_LL_PM_Enter_AES              ((void    (*) (void))                                                   R2F_JT_OFFSET(28))
#define MAP_LL_PM_Exit_AES               ((void    (*) (void))                                                   R2F_JT_OFFSET(29))
#define MAP_LL_PM_PowerOnRfCore          ((void    (*) (void))                                                   R2F_JT_OFFSET(30))
#define MAP_LL_PM_PowerOffRfCore         ((void    (*) (void))                                                   R2F_JT_OFFSET(31))
#define MAP_LL_PM_StopCurTaskTimer       ((void    (*) (taskInfo_t *))                                           R2F_JT_OFFSET(32))
// ROM-to-RAM Data
#define hciTaskID                    (*(uint8 *)                                                                 R2F_JT_OFFSET(33))
#define hciL2capTaskID               (*(uint8 *)                                                                 R2F_JT_OFFSET(34))
#define hciGapTaskID                 (*(uint8 *)                                                                 R2F_JT_OFFSET(35))
#define hciSmpTaskID                 (*(uint8 *)                                                                 R2F_JT_OFFSET(36))
#define hciTestTaskID                (*(uint8 *)                                                                 R2F_JT_OFFSET(37))
#define llConfigTable                (*(llCfgTable_t *)                                                          R2F_JT_OFFSET(38))
//
#define ECC_initialize               ((void    (*) (uint32_t *, uint8_t))                                        R2F_JT_OFFSET(39))
#define ECC_generateKey              ((uint8_t (*) (uint32_t *, uint32_t *, uint32_t *, uint32_t *))             R2F_JT_OFFSET(40))
//#define ECC_ECDSA_sign             ((uint8_t (*) (uint32_t *, uint32_t *, uint32_t *, uint32_t *, uint32_t *)) R2F_JT_OFFSET(41))
#define ECC_ECDH_computeSharedSecret ((uint8_t (*) (uint32_t *, uint32_t *, uint32_t *, uint32_t *, uint32_t *)) R2F_JT_OFFSET(41))

#else

// Directly map MAP_ functions to their flash counterparts if not building ROM image
// regardless of whether the functions are present in the jump table above

#define MAP_osal_mem_alloc                      osal_mem_alloc
#define MAP_osal_mem_free                       osal_mem_free
#define MAP_osal_pwrmgr_task_state              osal_pwrmgr_task_state
#define MAP_osal_msg_allocate                   osal_msg_allocate
#define MAP_osal_msg_deallocate                 osal_msg_deallocate
#define MAP_osal_msg_send                       osal_msg_send
#define MAP_osal_msg_receive                    osal_msg_receive
#define MAP_osal_set_event                      osal_set_event
#define MAP_osal_memcpy                         osal_memcpy
#define MAP_osal_revmemcpy                      osal_revmemcpy
#define MAP_osal_memset                         osal_memset
#define MAP_osal_memcmp                         osal_memcmp
#define MAP_osal_memdup                         osal_memdup
#define MAP_osal_rand                           osal_rand
#define MAP_osal_isbufset                       osal_isbufset
#define MAP_osal_bm_alloc                       osal_bm_alloc
#define MAP_osal_bm_free                        osal_bm_free
#define MAP_osal_bm_adjust_header               osal_bm_adjust_header
#define MAP_osal_start_timerEx                  osal_start_timerEx
#define MAP_osal_stop_timerEx                   osal_stop_timerEx
#define MAP_osal_start_reload_timer             osal_start_reload_timer
#define MAP_osal_clear_event                    osal_clear_event
#define MAP_osal_CbTimerStart                   osal_CbTimerStart
#define MAP_osal_CbTimerStop                    osal_CbTimerStop
#define MAP_osal_CbTimerUpdate                  osal_CbTimerUpdate
#define MAP_osal_strlen                         osal_strlen
#define MAP_osal_buffer_uint32                  osal_buffer_uint32
#define MAP_osal_build_uint32                   osal_build_uint32
#define MAP_Onboard_soft_reset                  Onboard_soft_reset
#define MAP_IntMasterEnable                     IntMasterEnable
#define MAP_IntMasterDisable                    IntMasterDisable
#define MAP_IntEnable                           IntEnable
#define MAP_IntDisable                          IntDisable
#define MAP_halAssertHandler                    halAssertHandler
#define MAP_HalTRNG_InitTRNG                    HalTRNG_InitTRNG
#define MAP_HalTRNG_GetTRNG                     HalTRNG_GetTRNG
#define MAP_LL_PM_Init                          LL_PM_Init
#define MAP_LL_PM_GetRfCoreState                LL_PM_GetRfCoreState
#define MAP_LL_PM_StartRfTask                   LL_PM_StartRfTask
#define MAP_LL_PM_PowerOnReq                    LL_PM_PowerOnReq
#define MAP_LL_PM_PowerCycleRadio               LL_PM_PowerCycleRadio
#define MAP_LL_PM_ForceSysBusThroughRF          LL_PM_ForceSysBusThroughRF
#define MAP_LL_PM_Enter_AES                     LL_PM_Enter_AES
#define MAP_LL_PM_Exit_AES                      LL_PM_Exit_AES
#define MAP_LL_PM_PowerOnRfCore                 LL_PM_PowerOnRfCore
#define MAP_LL_PM_PowerOffRfCore                LL_PM_PowerOffRfCore
#define MAP_LL_PM_StopCurTaskTimer              LL_PM_StopCurTaskTimer

// The below defines are for functions which are not called out of ROM for R2
// but still use the MAP_ prefix in the source code for potential future ROM
#define MAP_LL_PM_RtcSynchToRAT                 LL_PM_RtcSynchToRAT
#define MAP_LL_PM_RtcSynchFromRAT               LL_PM_RtcSynchFromRAT
#define MAP_LL_PM_ShutdownFS                    LL_PM_ShutdownFS
#define MAP_LL_PM_EnablePMNotifyHandler         LL_PM_EnablePMNotifyHandler
#define MAP_LL_PM_TimeToNextRfEvent             LL_PM_TimeToNextRfEvent
#define MAP_LL_PM_SetTimerForWakeup             LL_PM_SetTimerForWakeup
#define MAP_LL_PM_SetPowerMgrRequirements       LL_PM_SetPowerMgrRequirements
#define MAP_LL_PM_ReleasePowerMgrRequirements   LL_PM_ReleasePowerMgrRequirements
#define MAP_LL_PM_PowerOnRfCoreOptimize         LL_PM_PowerOnRfCoreOptimize
#define MAP_LL_PM_PowerOnRfCoreOptimize         LL_PM_PowerOnRfCoreOptimize
#define MAP_LL_PM_SendWakeUpCmd                 LL_PM_SendWakeUpCmd
#define MAP_LL_PM_WakeupHandler                 LL_PM_WakeupHandler
#define MAP_LL_PM_PowerOffRfCoreOptimize        LL_PM_PowerOffRfCoreOptimize
#define MAP_LL_PM_RFStartupState_2              LL_PM_RFStartupState_2
#define MAP_LL_PM_RFStartupState_3              LL_PM_RFStartupState_3

#endif // ROM_BUILD


#endif /* R2F_FLASH_JT_H */
