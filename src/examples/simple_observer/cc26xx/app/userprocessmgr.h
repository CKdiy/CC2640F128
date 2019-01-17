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

/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
    USER_PROCESS_IDLE,
    USER_PROCESS_READY,
    USER_PROCESS_RUNNING,
    USER_PROCESS_TIMEOUT,
}tUserProcessStates;

typedef enum
{
	USER_PROCESS_IDLE_MODE,
	USER_PROCESS_NORAMAL_MODE,
    USER_PROCESS_ABNORMAL_MODE,
    USER_PROCESS_ACTIVE_MODE,
	USER_PROCESS_SLEEP_MODE,
}tUserProcessMode;

enum
{    
    TYPE_LORATAGUP          = 0x01,//¨¦¨ª¡¤Y?¡§¨¦?¡ä?LORATAG??
    
    TYPE_LORATAUPRESP       = 0x81,//¨ª?1????¡äLORATAG??
};

typedef struct 
{
	uint8 abNormalScanTime;	
	uint8 clockCounter;
	uint8 memsActiveFlg;
	uint8 memsNoActiveCounter;
	uint8 memsActiveCounter;
	uint8 wakeUpSourse;      //value:0x00 RTC ,0x02 Mems ,0x04 key, 0x08 LoraTX/RX INT
	uint8 rftxtimeout;
	uint8 rfrxtimeout;
	uint8 sosstatustick;
}tUserProcessMgr;

typedef struct 
{
	uint8 txinterval;
	uint8 sleeptime;
}tUserNvramInf;

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
