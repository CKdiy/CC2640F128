/******************************************************************************

 @file  simple_np_vendor_uuid.h

 @brief This file contains the UUID management for the
        Simple BLE Peripheral sample application, for use with the
        CC2650 Bluetooth Low Energy Protocol Stack.

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

#ifndef SIMPLENP_VENDOR_UUID_H
#define SIMPLENP_VENDOR_UUID_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "simple_gatt_profile.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/** @defgroup HAP_BASE_UUID_GRP HAP predefined UUID
* @{
*/

// HAP Services short UUID (Bluetooth base UUID)
#define HAP_ACCINFO_SERV_UUID                 0xFED3
#define HAP_PAIRING_SERV_UUID                 0xFED4
#define HAP_LIGHTBULB_SERV_UUID               0xFED2
#define HAP_FAN_SERV_UUID                     0xFECB
#define HAP_GARAG_DOOR_SERV_UUID              0xFECE
#define HAP_LOCK_MGMT_SERV_UUID               0xFECF
#define HAP_LOCK_MECH_SERV_UUID               0xFED0
#define HAP_OUTLET_SERV_UUID                  0xFECC
#define HAP_SWITCH_SERV_UUID                  0xFECD
#define HAP_THERMOSTAT_SERV_UUID              0xFED1

// HAP Pairing Characteristic UUIDs (HAP base UUIDs)
#define HAP_PAIR_SETUP_UUID                   0x004C
#define HAP_PAIR_VERIFY_UUID                  0x004E
#define HAP_PAIR_FEATURES_UUID                0x004F
#define HAP_PAIR_PAIRINGS_UUID                0x0050
#define HAP_SERV_INSTANCE_UUID                0x0051

// HAP Characteristic short UUID ((HAP base UUIDs))
#define HAP_ADMIN_ONLY_UUID                   0x0001
#define HAP_AUDIO_FEEDBACK_UUID               0x0005
#define HAP_BRIGHTNESS_UUID                   0x0008
#define HAP_COOL_THRESH_UUID                  0x000D
#define HAP_CUR_DOOR_STATE_UUID               0x000E
#define HAP_CUR_HEAT_MODE_UUID                0x000F
#define HAP_CUR_REL_HUM_UUID                  0x0010
#define HAP_CUR_TEMP_UUID                     0x0011
#define HAP_FIRM_REV_UUID                     0x0052
#define HAP_HARD_REV_UUID                     0x0053
#define HAP_HEAT_THRESH_UUID                  0x0012
#define HAP_HUE_UUID                          0x0013
#define HAP_IDENTIFY_UUID                     0x0014
#define HAP_LOCK_MGNT_CTRL_POINT_UUID         0x0019
#define HAP_LOCK_MGNT_AUTO_SEC_TIMEOUT_UUID   0x001A
#define HAP_LOCK_LAST_KNOW_ACTION_UUID        0x001C
#define HAP_LOCK_CURRENT_STATE_UUID           0x001D
#define HAP_LOCK_TARGET_STATE_UUID            0x001E
#define HAP_LOG_UUID                          0x001F
#define HAP_MANUFACT_UUID                     0x0020
#define HAP_MODEL_UUID                        0x0021
#define HAP_MOTION_DETEC_UUID                 0x0022
#define HAP_NAME_UUID                         0x0023
#define HAP_OBSTRUCTION_DETECT_UUID           0x0024
#define HAP_ON_UUID                           0x0025
#define HAP_OUTLET_IN_USE_UUID                0x0026
#define HAP_ROT_DIR_UUID                      0x0028
#define HAP_ROT_SPEED_UUID                    0x0029
#define HAP_SATURATION_UUID                   0x002F
#define HAP_SERIAL_NO_UUID                    0x0030
#define HAP_SOFT_REV_UUID                     0x0054
#define HAP_TARGET_DOOR_STATE_UUID            0x0032
#define HAP_TARGET_HEAT_MODE_UUID             0x0033
#define HAP_TARGET_REL_HUM_UUID               0x0034
#define HAP_TARGET_HEAT_TEMP_UUID             0x0035
#define HAP_TEMP_UNIT_UUID                    0x0036
#define HAP_VERSION_UUID                      0x0037
/** @} End HAP_BASE_UUID_GRP */

/*********************************************************************
 * MACROS
 */

// HAP base 128-bit UUID: 00000000-0000-1000-8000-0026BB765291
#define HAP_BASE_UUID_128( uuid ) 0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, \
                                  0x00, 0x80, 0x00, 0x10, 0x00, 0x00, \
                                  LO_UINT16( uuid ), HI_UINT16( uuid ), 0x00, 0x00

// HAP base 128-bit UUID: 46DCFEF0-D281-1646-B5D9-6ABDD796939A
#define HAP_CHAR_ID_ATTR_UUID_128      0x9A, 0x93, 0x96, 0xD7, 0xBD, 0x6A, \
                                       0xD9, 0xB5, \
                                       0x16, 0x46, \
                                       0xD2, 0x81, \
                                       0xFE, 0xF0, \
                                       0x46, 0xDC

CONST uint8_t randomVendorUUIDTable[][ATT_UUID_SIZE] =
{
  {HAP_CHAR_ID_ATTR_UUID_128}
};

/*********************************************************************
 * TYPDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/

// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8_t simpleProfileServLongUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8_t simpleProfilechar1LongUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8_t simpleProfilechar2LongUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(SIMPLEPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8_t simpleProfilechar3LongUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(SIMPLEPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8_t simpleProfilechar4LongUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8_t simpleProfilechar5LongUUID[ATT_UUID_SIZE] =
{
  BT_BASE_UUID_128(SIMPLEPROFILE_CHAR5_UUID)
};

// HAP Lightbulb service UUID
CONST uint8_t HAP_lightbulbServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_LIGHTBULB_SERV_UUID ), HI_UINT16( HAP_LIGHTBULB_SERV_UUID )
};

/// HAP ON characteristic UUID
CONST uint8_t HAP_onUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ON_UUID)
};

// HAP HUE characteristic UUID
CONST uint8_t HAP_hueUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_HUE_UUID)
};

// HAP brightness characteristic UUID
CONST uint8_t HAP_brightnessUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_BRIGHTNESS_UUID)
};

// HAP saturation characteristic UUID
CONST uint8_t HAP_saturationUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_SATURATION_UUID)
};

// Access Information service UUID
CONST uint8_t HAP_accInfoServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_ACCINFO_SERV_UUID ), HI_UINT16( HAP_ACCINFO_SERV_UUID )
};

// fan service UUID
CONST uint8_t HAP_fanServiUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_FAN_SERV_UUID ), HI_UINT16( HAP_FAN_SERV_UUID )
};

// garage door service UUID
CONST uint8_t HAP_garageServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_GARAG_DOOR_SERV_UUID ), HI_UINT16( HAP_GARAG_DOOR_SERV_UUID )
};

// lock management service UUID
CONST uint8_t HAP_lockManagementServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_LOCK_MGMT_SERV_UUID ), HI_UINT16( HAP_LOCK_MGMT_SERV_UUID )
};

// lock mechanism service UUID
CONST uint8_t HAP_lockMechanismServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_LOCK_MECH_SERV_UUID ), HI_UINT16( HAP_LOCK_MECH_SERV_UUID )
};

// outlet service UUID
CONST uint8_t HAP_outletServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_OUTLET_SERV_UUID ), HI_UINT16( HAP_OUTLET_SERV_UUID )
};

// switch service UUID
CONST uint8_t HAP_switchServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_SWITCH_SERV_UUID ), HI_UINT16( HAP_SWITCH_SERV_UUID )
};

// thermostat service UUID
CONST uint8_t HAP_thermostatServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_THERMOSTAT_SERV_UUID ), HI_UINT16( HAP_THERMOSTAT_SERV_UUID )
};

// admin only characteristic UUID
CONST uint8_t HAP_adminUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ADMIN_ONLY_UUID)
};

// admin audio feedback characteristic UUID
CONST uint8_t HAP_audioUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_AUDIO_FEEDBACK_UUID)
};

// cool threshold feedback characteristic UUID
CONST uint8_t HAP_coolThreshUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_COOL_THRESH_UUID)
};

// current door state characteristic UUID
CONST uint8_t HAP_currentDoorStateUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_CUR_DOOR_STATE_UUID)
};

// current door state characteristic UUID
CONST uint8_t HAP_currentHeatModeUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_CUR_HEAT_MODE_UUID)
};

// current relative humidity characteristic UUID
CONST uint8_t HAP_currentRelativeHumidityUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_CUR_REL_HUM_UUID)
};

// current temperature characteristic UUID
CONST uint8_t HAP_currentTempThreshUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_CUR_TEMP_UUID)
};

// firmware revision characteristic UUID
CONST uint8_t HAP_firmRevUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_FIRM_REV_UUID)
};

// Hardware revision characteristic UUID
CONST uint8_t HAP_hardRevUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_HARD_REV_UUID)
};

// Heat threshold characteristic UUID
CONST uint8_t HAP_heatThreshUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_HEAT_THRESH_UUID)
};

// Identify characteristic UUID
CONST uint8_t HAP_identifyUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_IDENTIFY_UUID)
};

// lock management control point characteristic UUID
CONST uint8_t HAP_lockMgmtCtrlPointUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LOCK_MGNT_CTRL_POINT_UUID)
};

// lock management auto secure timeout characteristic UUID
CONST uint8_t HAP_lockMgmtAutoSecureTimeoutUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LOCK_MGNT_AUTO_SEC_TIMEOUT_UUID)
};

// lock last known action characteristic UUID
CONST uint8_t HAP_lockMgmtlastActionUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LOCK_LAST_KNOW_ACTION_UUID)
};

// lock current state characteristic UUID
CONST uint8_t HAP_lockCurrentStateUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LOCK_CURRENT_STATE_UUID)
};

// lock Target state characteristic UUID
CONST uint8_t HAP_lockTargetStateUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LOCK_TARGET_STATE_UUID)
};

// log characteristic UUID
CONST uint8_t HAP_logUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_LOG_UUID)
};

// Manufacturer characteristic UUID
CONST uint8_t HAP_manufactUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_MANUFACT_UUID)
};

// Model characteristic UUID
CONST uint8_t HAP_modelUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_MODEL_UUID)
};

// Motion detection characteristic UUID
CONST uint8_t HAP_motionDetectionUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_MOTION_DETEC_UUID)
};

// Obstruction detection characteristic UUID
CONST uint8_t HAP_obstructionDetectionUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_OBSTRUCTION_DETECT_UUID)
};

// outlet in use characteristic UUID
CONST uint8_t HAP_outletInUseUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_OUTLET_IN_USE_UUID)
};

// rotation direction characteristic UUID
CONST uint8_t HAP_rotDirUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ROT_DIR_UUID)
};

// rotation speed characteristic UUID
CONST uint8_t HAP_rotSpeedUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_ROT_SPEED_UUID)
};

// Serial Number characteristic UUID
CONST uint8_t HAP_serialNoUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_SERIAL_NO_UUID)
};

// software revision characteristic UUID
CONST uint8_t HAP_softRevUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_SOFT_REV_UUID)
};

// target door state characteristic UUID
CONST uint8_t HAP_targetDoorStateUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_TARGET_DOOR_STATE_UUID)
};

// target heap mode characteristic UUID
CONST uint8_t HAP_targetHeapModeUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_TARGET_HEAT_MODE_UUID)
};

// target relative humidity characteristic UUID
CONST uint8_t HAP_targetRelHumUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_TARGET_REL_HUM_UUID)
};

// target heap temp characteristic UUID
CONST uint8_t HAP_targetHeapTempUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_TARGET_HEAT_TEMP_UUID)
};

// temperature unit characteristic UUID
CONST uint8_t HAP_tempUnitUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_TEMP_UNIT_UUID)
};

// version characteristic UUID
CONST uint8_t HAP_versionUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_VERSION_UUID)
};


// Pairing service UUID
CONST uint8_t HAP_pairingServUUID[ATT_UUID_SIZE] =
{
  LO_UINT16( HAP_PAIRING_SERV_UUID ), HI_UINT16( HAP_PAIRING_SERV_UUID )
};

// HAP Service instance characteristic UUID
CONST uint8_t HAP_servInstanceUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_SERV_INSTANCE_UUID)
};

// HAP name UUID
CONST uint8_t HAP_nameUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_NAME_UUID)
};

// Pair-setup characteristic UUID
CONST uint8_t HAP_pairSetupUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_PAIR_SETUP_UUID)
};

// Pair-verify characteristic UUID
CONST uint8_t HAP_pairVerifyUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_PAIR_VERIFY_UUID)
};

// Features characteristic UUID
CONST uint8_t HAP_featuresUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_PAIR_FEATURES_UUID)
};

// Pairings characteristic UUID
CONST uint8_t HAP_pairingsUUID[ATT_UUID_SIZE] =
{
  HAP_BASE_UUID_128(HAP_PAIR_PAIRINGS_UUID)
};
#ifdef __cplusplus
}
#endif

#endif /* SIMPLENP_GATT_H */
