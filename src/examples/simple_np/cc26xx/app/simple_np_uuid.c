/******************************************************************************

 @file  simple_np_uuid.c

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include "hal_types.h"
#include "comdef.h"

#include <icall.h>

#include "gatt.h"

#include "gattservapp.h"
#include "inc/npi_task.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "hci_tl.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#include "snp.h"
#include "simple_np.h"
#include "simple_np_gap.h"
#include "simple_np_gatt.h"
#include "simple_np_uuid.h"
#include "simple_np_vendor_uuid.h"
#include "devinfoservice.h"
#include "gapgattserver.h"
#include "gatt_uuid.h"
#include "linkdb.h"

/*********************************************************************
 * CONSTANTS
 */
const uint8_t BT_BASE[] = {
  0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00,
  0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
  0x00, 0x00,  //Short UUID
  0x00, 0x00
};

// vendor base 128-bit UUID: 00000000-0000-1000-8000-0026BB765291
const uint8_t VENDOR_BASE_1[] = {
  0x91, 0x52, 0x76, 0xBB, 0x26, 0x00,
  0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
  0x00, 0x00, //Short UUID
  0x00, 0x00
};

// Device information service
extern CONST uint8_t devInfoServUUID[ATT_BT_UUID_SIZE];

// Link Loss Service UUID.
CONST uint8_t linkLossServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LINK_LOSS_SERV_UUID), HI_UINT16(LINK_LOSS_SERV_UUID)
};

// Immediate Alert Service UUID.
CONST uint8_t imAlertServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IMMEDIATE_ALERT_SERV_UUID), HI_UINT16(IMMEDIATE_ALERT_SERV_UUID)
};

// Tx Power Level Service UUID.
CONST uint8_t txPwrLevelServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(TX_PWR_LEVEL_SERV_UUID), HI_UINT16(TX_PWR_LEVEL_SERV_UUID)
};

// Alert Level Attribute UUID.
CONST uint8_t alertLevelUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ALERT_LEVEL_UUID), HI_UINT16(ALERT_LEVEL_UUID)
};

// Tx Power Level Attribute UUID.
CONST uint8_t txPwrLevelUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(TX_PWR_LEVEL_UUID), HI_UINT16(TX_PWR_LEVEL_UUID)
};

// Glucose service.
CONST uint8_t glucoseServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_SERV_UUID), HI_UINT16(GLUCOSE_SERV_UUID)
};

// Glucose characteristic.
CONST uint8_t glucoseMeasUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_MEAS_UUID), HI_UINT16(GLUCOSE_MEAS_UUID)
};

// Glucose Measurement Context.
CONST uint8_t glucoseContextUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_CONTEXT_UUID), HI_UINT16(GLUCOSE_CONTEXT_UUID)
};

// Glucose Feature.
CONST uint8_t glucoseFeatureUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GLUCOSE_FEATURE_UUID), HI_UINT16(GLUCOSE_FEATURE_UUID)
};

// Record Control Point.
CONST uint8_t recordControlPointUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RECORD_CTRL_PT_UUID), HI_UINT16(RECORD_CTRL_PT_UUID)
};

// Thermometer service
CONST uint8_t thermometerServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(THERMOMETER_SERV_UUID), HI_UINT16(THERMOMETER_SERV_UUID)
};

// Thermometer temperature characteristic
CONST uint8_t thermometerTempUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(TEMP_MEAS_UUID), HI_UINT16(TEMP_MEAS_UUID)
};

// Thermometer Site
CONST uint8_t thermometerTypeUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(TEMP_TYPE_UUID), HI_UINT16(TEMP_TYPE_UUID)
};

// Thermometer Immediate Measurement
CONST uint8_t thermometerImeasUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IMEDIATE_TEMP_UUID), HI_UINT16(IMEDIATE_TEMP_UUID)
};

// Thermometer Measurement Interval
CONST uint8_t thermometerIntervalUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MEAS_INTERVAL_UUID), HI_UINT16(MEAS_INTERVAL_UUID)
};

// Thermometer Test Commands
CONST uint8_t thermometerIRangeUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GATT_VALID_RANGE_UUID), HI_UINT16(GATT_VALID_RANGE_UUID)
};

// Heart rate service
CONST uint8_t heartRateServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_SERV_UUID), HI_UINT16(HEARTRATE_SERV_UUID)
};

// Heart rate measurement characteristic
CONST uint8_t heartRateMeasUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_MEAS_UUID), HI_UINT16(HEARTRATE_MEAS_UUID)
};

// Sensor location characteristic
CONST uint8_t heartRateSensLocUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BODY_SENSOR_LOC_UUID), HI_UINT16(BODY_SENSOR_LOC_UUID)
};

// Command characteristic
CONST uint8_t heartRateCommandUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HEARTRATE_CTRL_PT_UUID), HI_UINT16(HEARTRATE_CTRL_PT_UUID)
};
// Battery service
CONST uint8_t battServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATT_SERV_UUID), HI_UINT16(BATT_SERV_UUID)
};

// Battery level characteristic
CONST uint8_t battLevelUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID)
};

// BloodPressure service
CONST uint8_t bloodPressureServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLOODPRESSURE_SERV_UUID), HI_UINT16(BLOODPRESSURE_SERV_UUID)
};

// BloodPressure temperature characteristic
CONST uint8_t bloodPressureTempUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLOODPRESSURE_MEAS_UUID), HI_UINT16(BLOODPRESSURE_MEAS_UUID)
};

// BloodPressure Intermediate Cuff Pressure
CONST uint8_t bloodPressureImeasUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IMEDIATE_CUFF_PRESSURE_UUID), HI_UINT16(IMEDIATE_CUFF_PRESSURE_UUID)
};

// BloodPressure Feature
CONST uint8_t bpFeatureUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BLOODPRESSURE_FEATURE_UUID), HI_UINT16(BLOODPRESSURE_FEATURE_UUID)
};

// HID service
CONST uint8_t hidServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HID_SERV_UUID), HI_UINT16(HID_SERV_UUID)
};

// HID Boot Keyboard Input Report characteristic
CONST uint8_t hidBootKeyInputUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BOOT_KEY_INPUT_UUID), HI_UINT16(BOOT_KEY_INPUT_UUID)
};

// HID Boot Mouse Input Report characteristic
CONST uint8_t hidBootMouseInputUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BOOT_MOUSE_INPUT_UUID), HI_UINT16(BOOT_MOUSE_INPUT_UUID)
};

// HID Boot Keyboard Output Report characteristic
CONST uint8_t hidBootKeyOutputUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BOOT_KEY_OUTPUT_UUID), HI_UINT16(BOOT_KEY_OUTPUT_UUID)
};

// HID Information characteristic
CONST uint8_t hidInfoUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HID_INFORMATION_UUID), HI_UINT16(HID_INFORMATION_UUID)
};

// HID Report Map characteristic
CONST uint8_t hidReportMapUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(REPORT_MAP_UUID), HI_UINT16(REPORT_MAP_UUID)
};

// HID Control Point characteristic
CONST uint8_t hidControlPointUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HID_CTRL_PT_UUID), HI_UINT16(HID_CTRL_PT_UUID)
};

// HID Report characteristic
CONST uint8_t hidReportUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(REPORT_UUID), HI_UINT16(REPORT_UUID)
};

// HID Protocol Mode characteristic
CONST uint8_t hidProtocolModeUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PROTOCOL_MODE_UUID), HI_UINT16(PROTOCOL_MODE_UUID)
};

// Scan parameters service
CONST uint8_t scanParamServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SCAN_PARAM_SERV_UUID), HI_UINT16(SCAN_PARAM_SERV_UUID)
};

// Scan interval window characteristic
CONST uint8_t scanIntervalWindowUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SCAN_INTERVAL_WINDOW_UUID), HI_UINT16(SCAN_INTERVAL_WINDOW_UUID)
};

// Scan parameter refresh characteristic
CONST uint8_t scanParamRefreshUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SCAN_REFRESH_UUID), HI_UINT16(SCAN_REFRESH_UUID)
};
// RSC service
CONST uint8_t runningServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_SERV_UUID), HI_UINT16(RSC_SERV_UUID)
};

// RSC measurement characteristic
CONST uint8_t runningMeasUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_MEAS_UUID), HI_UINT16(RSC_MEAS_UUID)
};

// RSC feature characteristic
CONST uint8_t runningFeatureUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RSC_FEATURE_UUID), HI_UINT16(RSC_FEATURE_UUID)
};

// RSC sensor location characteristic
CONST uint8_t runningSensLocUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SENSOR_LOC_UUID), HI_UINT16(SENSOR_LOC_UUID)
};

// RSC command characteristic
CONST uint8_t runningCommandUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SC_CTRL_PT_UUID), HI_UINT16(SC_CTRL_PT_UUID)
};
// CSC service.
CONST uint8_t cyclingServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CSC_SERV_UUID), HI_UINT16(CSC_SERV_UUID)
};

// CSC measurement characteristic.
CONST uint8_t cyclingMeasUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CSC_MEAS_UUID), HI_UINT16(CSC_MEAS_UUID)
};

// CSC feature characteristic.
CONST uint8_t cyclingFeatureUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CSC_FEATURE_UUID), HI_UINT16(CSC_FEATURE_UUID)
};

// CSC sensor location characteristic.
CONST uint8_t cyclingSensLocUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SENSOR_LOC_UUID), HI_UINT16(SENSOR_LOC_UUID)
};

// CSC command characteristic.
CONST uint8_t cyclingCommandUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SC_CTRL_PT_UUID), HI_UINT16(SC_CTRL_PT_UUID)
};

// current time service.
CONST uint8_t currentTimeServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CURRENT_TIME_SERV_UUID), HI_UINT16(CURRENT_TIME_SERV_UUID)
};

// ref time service.
CONST uint8_t refTimeServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(REF_TIME_UPDATE_SERV_UUID), HI_UINT16(REF_TIME_UPDATE_SERV_UUID)
};

// next dst change  service.
CONST uint8_t nextDstChangeServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(NEXT_DST_CHANGE_SERV_UUID), HI_UINT16(NEXT_DST_CHANGE_SERV_UUID)
};

// nwa service.
CONST uint8_t nwaServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(NWA_SERV_UUID), HI_UINT16(NWA_SERV_UUID)
};
// phone alert service.
CONST uint8_t phoneAlertServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PHONE_ALERT_STS_SERV_UUID), HI_UINT16(PHONE_ALERT_STS_SERV_UUID)
};

// phone alert service.
CONST uint8_t alertNotifServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ALERT_NOTIF_SERV_UUID), HI_UINT16(ALERT_NOTIF_SERV_UUID)
};

// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8_t simpleProfileServUuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8_t simpleProfilechar1Uuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR1_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8_t simpleProfilechar2Uuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR2_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8_t simpleProfilechar3Uuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR3_UUID), HI_UINT16(SIMPLEPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8_t simpleProfilechar4Uuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8_t simpleProfilechar5Uuid[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR5_UUID), HI_UINT16(SIMPLEPROFILE_CHAR5_UUID)
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * SPNP TEST SERVICE
 */

/*********************************************************************
 * Extern FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
 /**
 *  @fn      SNP_checkBase
 *
 *  @brief   check if the long uuid match the base uuid.
 *           if so , return true.
 *
 *  @param[in]   base    :base UUID to check
 *  @param[in]   pUUID   :uuid to check
 *
 *  @return  TRUE if uuid and base match.
 */
bool SNP_checkBase(const uint8_t *pBase, uint8_t *pUUID)
{
  uint8_t i;
  bool res = TRUE;
  for(i = 0; i < 11; i++)
  {
    if(pBase[i] ^ pUUID[i])
    {
      res = FALSE;
      break;
    }
  }
  return res;
}

/**
 *  @fn          SNP_findCommonUUIDRec
 *
 *  @brief       check if the uuid is known and stored in flash, return it if so.
  *              only expect short UUID and return only short UUID
 *
 *  @param[in]   pUUID    :UUID of the service
 *  @param[in]   len      :length of the uuid (2 bytes expected)
 *
 *  @return      pointer to the UUID location in flash.
 */
const uint8_t *SNP_findCommonUUIDRec(uint8_t *pUUID, uint8_t len)
{
  const uint8_t *pRec = NULL;
  uint16_t uuid;
  if(len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uuid = BUILD_UINT16(pUUID[0], pUUID[1]);
  }
  else
  {
    return pRec;
  }
  switch (uuid)
  {
    /*** GATT Services ***/
    case IMMEDIATE_ALERT_SERV_UUID:
      pRec = imAlertServUuid;
      break;

    case LINK_LOSS_SERV_UUID:
      pRec = linkLossServUuid;
      break;

    case TX_PWR_LEVEL_SERV_UUID:
      pRec = txPwrLevelServUuid;
      break;

    case CURRENT_TIME_SERV_UUID:
      pRec = currentTimeServUuid;
      break;

    case REF_TIME_UPDATE_SERV_UUID:
      pRec = refTimeServUuid;
      break;

    case NEXT_DST_CHANGE_SERV_UUID:
      pRec = nextDstChangeServUuid;
      break;

    case GLUCOSE_SERV_UUID:
      pRec = glucoseServUuid;
      break;

    case THERMOMETER_SERV_UUID:
      pRec = thermometerServUuid;
      break;

    case DEVINFO_SERV_UUID:
      pRec = devInfoServUUID;
      break;

    case NWA_SERV_UUID:
      pRec = nwaServUuid;
      break;

    case HEARTRATE_SERV_UUID:
      pRec = heartRateServUuid;
      break;

    case PHONE_ALERT_STS_SERV_UUID:
      pRec = phoneAlertServUuid;
      break;

    case BATT_SERV_UUID:
      pRec = battServUuid;
      break;

    case BLOODPRESSURE_SERV_UUID:
      pRec = bloodPressureServUuid;
      break;

    case ALERT_NOTIF_SERV_UUID:
      pRec = alertNotifServUuid;
      break;

    case HID_SERV_UUID:
      pRec = hidServUuid;
      break;

    case SCAN_PARAM_SERV_UUID:
      pRec = scanParamServUuid;
      break;

    case RSC_SERV_UUID:
      pRec = runningServUuid;
      break;

    case CSC_SERV_UUID:
      pRec = cyclingServUuid;
      break;

    case CYCPWR_SERV_UUID:
      pRec = NULL;
      break;

    case LOC_NAV_SERV_UUID:
      pRec = NULL;
      break;

    case SIMPLEPROFILE_SERV_UUID:
      pRec = simpleProfileServUuid;
      break;

     /** GATT CHARACTERISTIC **/
    case ALERT_LEVEL_UUID:
      pRec = alertLevelUuid;
      break;

    case TX_PWR_LEVEL_UUID:
      pRec = txPwrLevelUuid;
      break;

    case GLUCOSE_MEAS_UUID:
      pRec = glucoseMeasUuid;
      break;

    case GLUCOSE_CONTEXT_UUID:
      pRec = glucoseContextUuid;
      break;

    case GLUCOSE_FEATURE_UUID:
      pRec = glucoseFeatureUuid;
      break;

    case RECORD_CTRL_PT_UUID:
      pRec = recordControlPointUuid;
      break;

    case TEMP_MEAS_UUID:
      pRec = thermometerTempUuid;
      break;

    case TEMP_TYPE_UUID:
      pRec = thermometerTypeUuid;
      break;

    case IMEDIATE_TEMP_UUID:
      pRec = thermometerImeasUuid;
      break;

    case MEAS_INTERVAL_UUID:
      pRec = thermometerIntervalUuid;
      break;

    case GATT_VALID_RANGE_UUID:
      pRec = thermometerIRangeUuid;
      break;

    case HEARTRATE_MEAS_UUID:
      pRec = heartRateMeasUuid;
      break;

    case BODY_SENSOR_LOC_UUID:
      pRec = heartRateSensLocUuid;
      break;

    case HEARTRATE_CTRL_PT_UUID:
      pRec = heartRateCommandUuid;
      break;

    case BATT_LEVEL_UUID:
      pRec = runningServUuid;
      break;

    case BLOODPRESSURE_MEAS_UUID:
      pRec = bloodPressureTempUuid;
      break;

    case BLOODPRESSURE_FEATURE_UUID:
      pRec = bpFeatureUuid;
      break;

    case IMEDIATE_CUFF_PRESSURE_UUID:
      pRec = bloodPressureImeasUuid;
      break;

    case BOOT_KEY_INPUT_UUID:
      pRec = hidBootKeyInputUuid;
      break;

    case BOOT_MOUSE_INPUT_UUID:
      pRec = hidBootMouseInputUuid;
      break;

    case BOOT_KEY_OUTPUT_UUID:
      pRec = hidBootKeyOutputUuid;
      break;

    case HID_INFORMATION_UUID:
      pRec = hidInfoUuid;
      break;

    case REPORT_MAP_UUID:
      pRec = hidReportMapUuid;
      break;

    case HID_CTRL_PT_UUID:
      pRec = hidControlPointUuid;
      break;

    case REPORT_UUID:
      pRec = hidReportUuid;
      break;

    case PROTOCOL_MODE_UUID:
      pRec = hidProtocolModeUuid;
      break;

    case SCAN_INTERVAL_WINDOW_UUID:
      pRec = scanIntervalWindowUuid;
      break;

    case SCAN_REFRESH_UUID:
      pRec = scanParamRefreshUuid;
      break;

    case RSC_MEAS_UUID:
      pRec = runningMeasUuid;
      break;

    case SC_CTRL_PT_UUID:
      pRec = runningCommandUuid;
      break;

    case CSC_MEAS_UUID:
      pRec = cyclingMeasUuid;
      break;

    case CSC_FEATURE_UUID:
      pRec = cyclingFeatureUuid;
      break;

    case SIMPLEPROFILE_CHAR1_UUID:
      pRec = simpleProfilechar1Uuid;
      break;

    case SIMPLEPROFILE_CHAR2_UUID:
      pRec = simpleProfilechar2Uuid;
      break;

    case SIMPLEPROFILE_CHAR3_UUID:
      pRec = simpleProfilechar3Uuid;
      break;

    case SIMPLEPROFILE_CHAR4_UUID:
      pRec = simpleProfilechar4Uuid;
      break;

    case SIMPLEPROFILE_CHAR5_UUID:
      pRec = simpleProfilechar5Uuid;
      break;

    default:
      break;

  }
  return pRec;
}

/**
 *  @fn      SNP_findVendorShortUUIDRec
 *
 *  @brief   check if the uuid is known and stored in flash, return it if so.
 *           This version return short UUID only, meaning it is a BT BASE UUID.
 *
 *
 *  @param[in]   pUUID    :UUID of the service
 *  @param[in]   len      :length of the uuid (2 or 16 bytes)
 *  @param[out]  pNewLen   :new length of the uuid (2 or 16 bytes)
 *
 *  @return  pointer to the long UUID location in flash.
 */
const uint8_t *SNP_findVendorShortUUIDRec( uint8_t *pUUID, uint8_t len,
                                           uint8_t *pNewLen )
{
  const uint8_t *pRec = NULL;
  uint16_t uuid;
  if(len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uuid = BUILD_UINT16(pUUID[0], pUUID[1]);
  }
  else if(len == ATT_UUID_SIZE)
  {
    //Check if the UUID is a vendor based service
    if((SNP_checkBase(BT_BASE, pUUID)) ||
       (SNP_checkBase(VENDOR_BASE_1, pUUID)))
    {
      uuid = BUILD_UINT16(pUUID[12], pUUID[13]);
    }
    else
    {
      return pRec;
    }
  }
  else
  {
    return pRec;
  }

  switch(uuid)
  {
    /*** GATT Services ***/
    case SIMPLEPROFILE_SERV_UUID:
      pRec = simpleProfileServUuid;
      break;

    case HAP_PAIRING_SERV_UUID:
      pRec = HAP_pairingServUUID;
      break;

    case HAP_LIGHTBULB_SERV_UUID:
      pRec = HAP_lightbulbServUUID;
      break;

    case HAP_ACCINFO_SERV_UUID:
      pRec = HAP_accInfoServUUID;
      break;

    case HAP_FAN_SERV_UUID:
      pRec = HAP_fanServiUUID;
      break;

    case HAP_GARAG_DOOR_SERV_UUID:
      pRec = HAP_garageServUUID;
      break;

    case HAP_LOCK_MGMT_SERV_UUID:
      pRec = HAP_lockManagementServUUID;
      break;

    case HAP_LOCK_MECH_SERV_UUID:
      pRec = HAP_lockMechanismServUUID;
      break;

    case HAP_OUTLET_SERV_UUID:
      pRec = HAP_outletServUUID;
      break;

    case HAP_SWITCH_SERV_UUID:
      pRec = HAP_switchServUUID;
      break;

    case HAP_THERMOSTAT_SERV_UUID:
      pRec = HAP_thermostatServUUID;
      break;

    default:
      break;

  }

  if(pRec)
  {
    *pNewLen = 2;
  }
  return pRec;

}

/**
 *  @fn          SNP_findVendorLongUUIDRec
 *
 *  @brief       check if the uuid know and store in flash, return it if so.
 *               only return long UUID location.
 *
 *
 *  @param[in]   pUUID    :UUID of the service
 *  @param[in]   len      :length of the uuid (2 or 16 bytes)
 *
 *  @return      pointer to the long UUID location in flash.
 */
const uint8_t *SNP_findVendorLongUUIDRec(uint8_t *pUUID, uint8_t len,
                                         uint8_t *pNewLen)
{
  const uint8_t *pRec = NULL;
  uint16_t uuid;
  if(len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uuid = BUILD_UINT16(pUUID[0], pUUID[1]);
  }
  else if(len == ATT_UUID_SIZE)
  {
    //Check if the UUID is a vendor based service
    if((SNP_checkBase(BT_BASE, pUUID)) ||
       (SNP_checkBase(VENDOR_BASE_1, pUUID)))
    {
      uuid = BUILD_UINT16(pUUID[12], pUUID[13]);
    }
    else
    {
      uint8_t index;

      // check for full random Long UUID not base on known vendor Base.
      for(index = 0; index < (sizeof(randomVendorUUIDTable) / ATT_UUID_SIZE);
          index++)
      {
        if(!memcmp(pUUID, &randomVendorUUIDTable[index], ATT_UUID_SIZE))
        {
          pRec = *(&randomVendorUUIDTable[index]);
          break;
        }
      }
      return pRec;
    }
  }
  else
  {
    return pRec;
  }

  switch (uuid)
  {
    case HAP_PAIR_SETUP_UUID:
      pRec = HAP_pairSetupUUID;
      break;

    case HAP_PAIR_VERIFY_UUID:
      pRec = HAP_pairVerifyUUID;
      break;

    case HAP_PAIR_FEATURES_UUID:
      pRec = HAP_featuresUUID;
      break;

    case HAP_PAIR_PAIRINGS_UUID:
      pRec = HAP_pairingsUUID;
      break;

    case HAP_ADMIN_ONLY_UUID:
      pRec = HAP_adminUUID;
      break;

    case HAP_AUDIO_FEEDBACK_UUID:
      pRec = HAP_audioUUID;
      break;

    case HAP_BRIGHTNESS_UUID:
      pRec = HAP_brightnessUUID;
      break;

    case HAP_COOL_THRESH_UUID:
      pRec = HAP_coolThreshUUID;
      break;

    case HAP_CUR_DOOR_STATE_UUID:
      pRec = HAP_currentDoorStateUUID;
      break;

    case HAP_CUR_HEAT_MODE_UUID:
      pRec = HAP_currentHeatModeUUID;
      break;

    case HAP_CUR_REL_HUM_UUID:
      pRec = HAP_currentRelativeHumidityUUID;
      break;

    case HAP_CUR_TEMP_UUID:
      pRec = HAP_currentTempThreshUUID;
      break;

    case HAP_FIRM_REV_UUID:
      pRec = HAP_firmRevUUID;
      break;
    case HAP_HARD_REV_UUID:
      pRec = HAP_hardRevUUID;
      break;

    case HAP_HEAT_THRESH_UUID:
      pRec = HAP_heatThreshUUID;
      break;

    case HAP_HUE_UUID:
      pRec = HAP_hueUUID;
      break;

    case HAP_IDENTIFY_UUID:
      pRec = HAP_identifyUUID;
      break;

    case HAP_LOCK_MGNT_CTRL_POINT_UUID:
      pRec = HAP_lockMgmtCtrlPointUUID;
      break;

    case HAP_LOCK_MGNT_AUTO_SEC_TIMEOUT_UUID:
      pRec = HAP_lockMgmtAutoSecureTimeoutUUID;
      break;

    case HAP_LOCK_LAST_KNOW_ACTION_UUID:
      pRec = HAP_lockMgmtlastActionUUID;
      break;

    case HAP_LOCK_CURRENT_STATE_UUID:
      pRec = HAP_lockCurrentStateUUID;
      break;

    case HAP_LOCK_TARGET_STATE_UUID:
      pRec = HAP_lockTargetStateUUID;
      break;

    case HAP_LOG_UUID:
      pRec = HAP_logUUID;
      break;

    case HAP_MANUFACT_UUID:
      pRec = HAP_manufactUUID;
      break;

    case HAP_MODEL_UUID:
      pRec = HAP_modelUUID;
      break;

    case HAP_MOTION_DETEC_UUID:
      pRec = HAP_motionDetectionUUID;
      break;

    case HAP_NAME_UUID:
      pRec = HAP_nameUUID;
      break;

    case HAP_OBSTRUCTION_DETECT_UUID:
      pRec = HAP_obstructionDetectionUUID;
      break;

    case HAP_ON_UUID:
      pRec = HAP_onUUID;
      break;

    case HAP_OUTLET_IN_USE_UUID:
      pRec = HAP_obstructionDetectionUUID;
      break;

    case HAP_ROT_DIR_UUID:
      pRec = HAP_rotDirUUID;
      break;

    case HAP_ROT_SPEED_UUID:
      pRec = HAP_rotSpeedUUID;
      break;

    case HAP_SATURATION_UUID:
      pRec = HAP_saturationUUID;
      break;

    case HAP_SERIAL_NO_UUID:
      pRec = HAP_serialNoUUID;
      break;

    case HAP_SOFT_REV_UUID:
      pRec = HAP_softRevUUID;
      break;

    case HAP_TARGET_DOOR_STATE_UUID:
      pRec = HAP_targetDoorStateUUID;
      break;

    case HAP_TARGET_HEAT_MODE_UUID:
      pRec = HAP_targetHeapModeUUID;
      break;
    case HAP_TARGET_REL_HUM_UUID:
      pRec = HAP_targetRelHumUUID;
      break;

    case HAP_TARGET_HEAT_TEMP_UUID:
      pRec = HAP_targetHeapTempUUID;
      break;

    case HAP_TEMP_UNIT_UUID:
      pRec = HAP_tempUnitUUID;
      break;

    case HAP_VERSION_UUID:
      pRec = HAP_versionUUID;
      break;

    case HAP_SERV_INSTANCE_UUID:
      pRec = HAP_servInstanceUUID;
      break;

    /*** TI simple Characteristic ***/
    case SIMPLEPROFILE_CHAR1_UUID:
      pRec = simpleProfilechar1LongUUID;
      break;

    case SIMPLEPROFILE_CHAR2_UUID:
      pRec = simpleProfilechar2LongUUID;
      break;

    case SIMPLEPROFILE_CHAR3_UUID:
      pRec = simpleProfilechar3LongUUID;
      break;

    case SIMPLEPROFILE_CHAR4_UUID:
      pRec = simpleProfilechar4LongUUID;
      break;

    case SIMPLEPROFILE_CHAR5_UUID:
      pRec = simpleProfilechar5LongUUID;
      break;

    default:
      break;

  }

  if(pRec)
  {
    *pNewLen = 16;
  }

  return pRec;
}

/*********************************************************************
 * @fn          SNP_findUUIDRec
 *
 *  @brief       check if the uuid know and store in flash, return it if so.
 *               only return long UUID location.
 *
 *
 *  @param[in]   pUUID    :UUID of the service
 *  @param[in]   len      :length of the uuid (2 or 16 bytes)
 *
 *  @return      pointer to the long UUID location in flash.
 */
const uint8_t *SNP_findUUIDRec(uint8_t *pUUID, uint8_t len, uint8_t *pNewLen)
{
  const uint8_t *pRec = NULL;
  uint8_t* pTemp;
  *pNewLen = len;
#ifndef SNP_NO_CONST_UUID
  //First Check   regular UUID
  pRec = GATT_FindUUIDRec(pUUID, len);
  if (!pRec)
  {
    pRec = SNP_findCommonUUIDRec(pUUID, len);
  }
  if (!pRec)
  {
    pRec = SNP_findVendorShortUUIDRec(pUUID, len, pNewLen);
  }
  if (!pRec)
  {
    pRec = SNP_findVendorLongUUIDRec(pUUID, len, pNewLen);
  }
#endif
  if (!pRec)
  {
    if(pTemp = ICall_malloc(len))
    {
      // Note: this alloc is not meant to be free, it must survive for the life
      // of the service.
      memcpy(pTemp, pUUID, len);
      pRec = pTemp;
    }
  }
  return pRec;
}
