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

typedef struct 
{
	uint8 abNormalScanTime;	
	uint8 clockCounter;
	uint8 memsActiveFlg;
	uint8 memsNoActiveCounter;
	uint8 wakeUpFlg;
}tUserProcessMgr;

typedef struct 
{
	uint8 txinterval;
	uint8 sleeptime;
}tUserNvramInf;

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
