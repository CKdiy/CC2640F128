/**************************************************************************************************
  Filename:       MEMS.h
  Revised:        $Date: 2018-11-11  $
  Revision:       $Revision: ck  $

**************************************************************************************************/

#ifndef MEMS_H
#define MEMS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
  
/*********************************************************************
 * TYPEDEFS
 */
typedef void (*MemsActiveCB_t)(uint8_t pinID);

typedef enum
{
	Bit_RESET,
	Bit_SET,
}tBitAction;
/*********************************************************************
 * CONSTANTS
 */
 
#define  MEMS_SADW    0x4e 
#define  MEMS_SADR    0x4F

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

bool MemsOpen(void);
void MemsClose(void);
bool Mems_ActivePin_Enable(MemsActiveCB_t memsActiveCB);
void Mems_ActivePin_Disable(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SX1278_H */
