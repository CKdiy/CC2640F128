/*
 * snv.c
 * Description: nvram managerment for system
 */

#include <string.h>
#include "osal_snv.h"
#include "snv.h"
#include "tagmgr.h"
#include "bcomdef.h"

static const block_info_t fb[] =
{
    /* block ID (must < 0xf)    size */
	{BLE_NVID_DEVINF_START,       9},
	{BLE_NVID_DRFLG_START,        6},
};

#define FB_SIZE         (sizeof(fb)/sizeof(block_info_t))

uint8_t nvBuf[10] = {0};

static int Nvram_Block_Init(uint8_t nvid, uint8_t len);
static int Nvram_Block_Check(uint8_t nvid, uint8_t len);
static int DevInf_Snv_Init(uint8_t len);

static int Ble_DrFailureNv_Init(uint8_t len);

/* Init nvram */
void Nvram_Init(void)
{  
	 uint8_t i;
	 
	 for(i=0; i<FB_SIZE; i++)
	 {
    	if (Nvram_Block_Check(fb[i].NVid, fb[i].size) != 0)
		{
			Nvram_Block_Init(fb[i].NVid, fb[i].size);
		}
	 }
}

static int Nvram_Block_Check(uint8_t nvid, uint8_t len)
{
    uint32_t	crc = 0;
	uint8_t		status;
	
    if(nvid > BLE_NVID_CUST_END)
	 	return -1;
	
	memset(nvBuf, 0, sizeof(nvBuf));
	
	status = osal_snv_read(nvid, len, (void *)nvBuf);
	if(status != SUCCESS)
	 	return -1;
	
	memcpy((void *)&crc, nvBuf, sizeof(crc));
    if (crc != crc32(0, &nvBuf[sizeof(crc)], len - sizeof(crc)))
        return -1;
    
    return 0;
}

static int Nvram_Block_Init(uint8_t nvid, uint8_t len)
{	
	if(nvid > BLE_NVID_CUST_END)
	 	return -1;
	
	if(BLE_NVID_DEVINF_START == nvid)
	{
		return DevInf_Snv_Init(len);
	}
	else if(BLE_NVID_DRFLG_START == nvid)
	{
	 	return Ble_DrFailureNv_Init(len); 
	}
	 
    return 0;
}

static int DevInf_Snv_Init(uint8_t len)
{
 	uint8_t		status;
	snv_device_inf_t *ptr = (snv_device_inf_t *)nvBuf;
	
	memset(nvBuf, 0, sizeof(nvBuf));
	/* LORA Para */
	ptr->loraPara_u.bit_t.power			= 1;
	ptr->loraPara_u.bit_t.rate  		= 1;
	ptr->loraPara_u.bit_t.channel	 	= 1;
	
	/* TAG Para */
	ptr->tagPara_u.bit_t.sos            = 0;
	ptr->tagPara_u.bit_t.sleep_delay    = 0;
	ptr->tagPara_u.bit_t.scan_num       = 3;
	ptr->tagPara_u.bit_t.res            = 0;
	ptr->tagPara_u.bit_t.up_interval    = 3;
	ptr->tagPara_u.bit_t.up_interval_s  = 8;
	ptr->tagPara_u.tagPara              = ntohs(ptr->tagPara_u.tagPara);
	
	/* BLE Para */
	ptr->blePara_u.bit_t.res            = 0;
	ptr->blePara_u.bit_t.scan_time      = 5;
	
	ptr->crc32 = crc32(0, (uint8_t *)(&ptr->loraPara_u), len - sizeof(uint32_t));
	
	status = osal_snv_write(BLE_NVID_DEVINF_START, len,  ptr);
	if(status != SUCCESS)
		return -1;
	
	return 0;
}

static int Ble_DrFailureNv_Init(uint8_t len)
{
 	uint8_t		status;
	snv_driverfailure_t *ptr = (snv_driverfailure_t *)nvBuf;  
	
	memset(nvBuf, 0, sizeof(nvBuf));
	ptr->lora   = 0;
	ptr->mems   = 0;
	
	ptr->crc32 = crc32(0, (uint8_t *)(&ptr->mems), len - sizeof(uint32_t));
	
	status = osal_snv_write(BLE_NVID_DRFLG_START, len,  ptr);
	if(status != SUCCESS)
		return -1;
	
	return 0;
}

int Ble_ReadNv_Inf(uint8_t nvid, uint8_t *readbuf)
{
	uint8_t		status;
	uint8_t     i;
	uint8_t     len;
	
	if(nvid > BLE_NVID_CUST_END)
		return -1;
	
	for(i=0; i<FB_SIZE; i++)
	{
		if(fb[i].NVid == nvid)
			len =  fb[i].size; 
	}
	
	status = osal_snv_read(nvid, len, (void *)readbuf);	
	if(status != SUCCESS)
		return -1;
	
	return 0;
}

int Ble_WriteNv_Inf(uint8_t nvid, uint8_t *writebuf)
{
	uint8_t		status;
	uint8_t     i;
	uint8_t     len;
	uint32_t	crc = 0;    
	
	if(nvid > BLE_NVID_CUST_END)
		return -1;
	
	for(i=0; i<FB_SIZE; i++)
	{
		if(fb[i].NVid == nvid)
			len =  fb[i].size; 
	}
	
	memset(nvBuf, 0, sizeof(nvBuf));
	
	crc = crc32(0, writebuf, len - sizeof(crc));
	memcpy(nvBuf, (void *)&crc, sizeof(crc));
	memcpy(nvBuf + sizeof(crc), writebuf, len - sizeof(crc));
	  
	status = osal_snv_write(nvid, len, (void *)nvBuf);	
	if(status != SUCCESS)
		return -1;
	
	return 0;
}