/**
  @headerfile: tagmgr.h
  $Date: 2018-10-18 $
  $Revision:    $
*/

#ifndef TAGMGR_H
#define TAGMGR_H

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"

/*********************************************************************
 * CONSTANTS
 */
#define BEELINKER_ADVDATA_LEN       30 
#define BEELINKER_ADVUUID_OFFSET    9
#define BEELINKER_ADVMAJOR_OFFSET   25
/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef union 
{
  	struct
	{
	  	uint8_t		pkt_len    		:6;      // Bit5~ Bit0  Payload length
		uint8_t		pkt_type		:2;      // Bit7~ Bit6  Package type     
    }bit_t; 
	
	uint8_t payLoadInf;
}payload_inf_n;

/**************************************
 * deviceinf union
 */
typedef union 
{
	struct
	{		
		uint8_t     beaconNum_1		:2;      // Bit2~ Bit0
		uint8_t     beaconNum_2     :2;      // Bit5~ Bit3
		uint8_t     beaconNum_3     :2;      // Bit8 ~Bit6
		uint8_t     beaconNum_4     :2;      // Bit11~ Bit9
		uint8_t     res    			:2;      // Bit13~ Bit12    reserve
		uint8_t     sos    			:1;      // Bit14   sos
		uint8_t     vbat    		:1;      // Bit15  Low voltage alarm
	}bit_t;

	uint16_t	tagPacketInf;
}device_up_inf_n;

/**
 *  lora union
 */
typedef union lora_para_n
{
	struct
	{
		uint8_t		power   		:8;      // Bit15~Bit8 Lora Power
		uint8_t     channel         :5;      // Bit4~Bit0  channel
		uint8_t     rate            :3;      // Bit7~Bit5  rate
	}bit_t;
	
	uint16_t    loraPara;
}lora_para_n;

/**
 *  tag union
 */
typedef union 
{
	struct
	{ 	    
		uint8_t		up_interval_s	:4;      // Bit3~Bit0 
		uint8_t     up_interval     :4;      // Bit7~Bit4    Reporting interval 
		uint8_t     res             :3;      // Bit10~Bit8   reserve
		uint8_t     scan_num        :2;      // Bit12~Bit11  scan times
		uint8_t     sleep_delay     :2;      // Bit14~Bit13 
		uint8_t		sos             :1;      // Bit15 sos: 0  enable 1 disable 
	}bit_t;
	
	uint16_t    tagPara;
}tag_para_n;

/**
 *  Ble parameter
 */
typedef union 
{
	struct
	{
		uint8_t		scan_time		:4;       // Bit3~Bit0   
		uint8_t		res	      		:4;       // Bit7~Bit4   reserve      
	}bit_t;
	
    uint8_t		blePara;
}ble_para_n;

/**
 * taginfo union
 */
typedef struct
{
	uint8_t rssi;	     	
	uint8_t minor[2];
}tagInfStruct;

/**
 *  Observer Structure
 */
typedef struct
{     
	tagInfStruct *tagInfBuf_t;  //缓存收到的所有tag
	uint16_t  index;	      
	uint8_t   tagNum;        //tagNum <= 8  
}observerInfStruct; 

/**
 *  Userinf Structure
 */
typedef struct
{   
    uint8_t  devId[6];          //设备地址 
	uint8_t  status;            // sos状态 & 电池电量 ？
    uint8_t  interval[2];	    //lora上传间隔时间,单位：秒
    uint8_t  txTagNum;          // txTagNum <= 4  上传Tag数量
	tagInfStruct *tagInfBuf_t;  // 单次上传的Tag信息
}userTxStruct;

typedef struct
{
	uint8_t           pre;
	payload_inf_n     payload_inf; 
}loratag_pkt_hdr_t;

typedef struct
{
	uint32_t   			crc32;
	lora_para_n			loraPara_u;
	tag_para_n			tagPara_u;
	ble_para_n    		blePara_u;
}snv_device_inf_t;

typedef struct
{
    uint32_t    crc32;
    uint8_t     mems;
    uint8_t     lora;
}snv_driverfailure_t;

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * Observer Profile Public APIs
 */


#endif /* TAGMGR_H */
