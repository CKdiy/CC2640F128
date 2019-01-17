/*
 * snv.h
 */
#ifndef __SNV_H__
#define __SNV_H__

#include <xdc/std.h>
#include "crc.h"

/* Customer NV Items - Range 0x80 - 0x8F - This must match the number of Bonding entries */
#define BLE_NVID_CUST_START 0x80 //!< Start of the Customer's NV IDs
#define BLE_NVID_CUST_END   0x8F //!< End of the Customer's NV IDs

#define BLE_NVID_DEVINF_START 	BLE_NVID_CUST_START 
#define BLE_NVID_DRFLG_START    BLE_NVID_DEVINF_START + 1 

typedef struct
{
    uint8_t NVid;
    uint8_t size;
} block_info_t;

void Nvram_Init(void);
int Ble_ReadNv_Inf(uint8_t nvid, uint8_t *readbuf);
int Ble_WriteNv_Inf(uint8_t nvid, uint8_t *writebuf);

#endif  /* SNV_H */