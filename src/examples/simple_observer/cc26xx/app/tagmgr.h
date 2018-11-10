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
   	uint16_t tagIndex;      //tag��������
}tagInfStruct;

/**
 *  Observer Structure
 */
typedef struct
{     
	tagInfStruct *tagInfBuf_t;  //�����յ�������tag
	uint16_t  index;	      
	uint8_t   tagNum;        //tagNum <= 8  
}observerInfStruct;

/**
 *  Userinf Structure
 */
typedef struct
{   
    uint8_t  devId[6];          //�豸��ַ 
    uint8_t  interval;	        //lora�ϴ����ʱ��,��λ����
    uint8_t  status;            // sos״̬ & ��ص��� ��
    uint8_t  txTagNum;          // txTagNum <= 4  �ϴ�Tag����
	tagInfStruct *tagInfBuf_t;  // �����ϴ���Tag��Ϣ
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
