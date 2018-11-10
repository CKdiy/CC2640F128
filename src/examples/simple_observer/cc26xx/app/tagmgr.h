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

/**
 * taginfo Structure
 */
typedef struct
{
	uint8_t rssi;	
	uint8_t major[2];	     	
	uint8_t minor[2];
   	uint16_t tagIndex;      //tag接收序列
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
    uint8_t  interval;	        //lora上传间隔时间,单位：秒
    uint8_t  status;            // sos状态 & 电池电量 ？
    uint8_t  txTagNum;          // txTagNum <= 4  上传Tag数量
	tagInfStruct *tagInfBuf_t;  // 单次上传的Tag信息
}userTxStruct;

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
