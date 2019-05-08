/**
  @headerfile: userprocessmgr.h
  $Date: 2018-11-03 $
  $Revision:    $
*/

#ifndef USERPROCESSMGR_H
#define USERPROCESSMGR_H

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */
typedef enum
{
    USER_PROCESS_IDLE_MODE,
    USER_PROCESS_NORAMAL_MODE,
    USER_PROCESS_ABNORMAL_MODE,
    USER_PROCESS_ACTIVE_MODE,
    USER_PROCESS_SLEEP_MODE,
    USER_PROCESS_PERIPHERALDRVFAIL_MODE,
    USER_PROCESS_LOWVOLTAGE_MODE,
}UserProcessMode;

enum
{    
	TYPE_LORATAGUP          = 0x00,   
    
	TYPE_TAGPARACONFIG      = 0x01,
	
	TYPE_TAGPARAUP          = 0x02,
};

enum 
{    
	WAKEUP_SOURSE_RTC  = 0x01,     //default
    
	WAKEUP_SOURSE_MEMS = 0x02,
	
	WAKEUP_SOURSE_KEY  = 0x04,
	
	WAKEUP_SOURSE_LORA = 0x08,
};

typedef struct 
{
	uint8 memsActiveFlg;
	uint8 memsNoActiveCounter;
	uint8 memsActiveCounter;
	uint8 wakeUpSourse;      //value:0x01 RTC ,0x02 Mems ,0x04 key, 0x08 LoraTX/RX INT
	uint8 rftxtimeout;
	uint8 rfrxtimeout;
	uint8 sosstatustick;
	uint8 noacktimetick;
	uint8 memsinterrupt_status;
}UserProcessMgr_t;

typedef struct 
{
  	uint8  sleepDelay;
	uint8  scanTimes;
	uint16 timeForscanning;
	uint16 txinterval;
	uint16 txinterval_S;
	uint16 memsNoActiveTime;
	uint8  channelCheckTime;
}UserTimeSeries_t;

#define ntohs(x) (uint16_t)( ((uint16_t)(x<<8) & 0xFF00) | ((uint16_t)(x>>8) & 0x00FF) )
#define htons(x) (uint16_t)( ((uint16_t)(x<<8) & 0xFF00) | ((uint16_t)(x>>8) & 0x00FF) )

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * Observer Profile Public APIs
 */


#endif /* USERPROCESSMGR_H */
