/*
  Filename:       sx1278_hal.c
  Revised:        $Date: 2018-10-25  $
  Revision:       $Revision:  $
 */

#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include "Board.h"
#include "sx1278_hal.h"

/* defines */
#define SX1278BOARD_SPI     Board_SPI0

/* sx1278 pin table */
PIN_Config sx1278PinTable[] = {
    Board_SX1278_RST   | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  | PIN_PUSHPULL, 	// Enable 3V3 domain. Need to be high for sx1278 to work.
	Board_SX1278_CSN   | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  | PIN_PUSHPULL,
	PIN_TERMINATE                                                               // Terminate list
};

PIN_Config sx1278RFStatusPinTable[] = {
 	Board_SX1278_DIO0  | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN   | PIN_PULLUP, // sx1278 RF tx/rx status notification pin	 
	PIN_TERMINATE
};

/* sx1278 SPI Structure */
SPI_Handle sx1278Handle;

// Pointer to application callback
LoraRFStatusCB_t appLoraRFStatusHandler = NULL;

/* sx1278 PIN driver state object */
static PIN_State sx1278PinState;

/* sx1278 PIN driver handle */
static PIN_Handle sx1278Pin;
static PIN_Handle RFStatusPin; 
/* Default sx1278_SPI parameters structure */
const SPI_Params sx1278_SPI_defaultParams = {
    SPI_MODE_BLOCKING,  /* transferMode */
    SPI_WAIT_FOREVER,   /* transferTimeout */
    NULL,               /* transferCallbackFxn */
    SPI_MASTER,         /* mode */
    1000000,            /* bitRate */
    8,                  /* dataSize */
    SPI_POL0_PHA0,      /* frameFormat */
    NULL                /* custom */
};

/* sx1278 SPI functions */
static void Board_sx1278LoraStatusCallback(PIN_Handle hPin, PIN_Id pinId);
/*********************************************************************
 * @fn      Close_sx1278_SPI
 *
 * @brief   Close sx1278 SPI.
 *
 * @param   none
 *
 * @return  none
 */
void Close_sx1278_SPI(void)
{
	SPI_close(sx1278Handle);
}

/*********************************************************************
 * @fn      Open_sx1278_SPI
 *
 * @brief   Open sx1278 SPI.
 *
 * @param   none
 *
 * @return  TRUE if  sx1278_SPI Open successful
 */
bool Open_sx1278_SPI(void)
{  
	SPI_init();
	sx1278Handle = SPI_open(SX1278BOARD_SPI, (SPI_Params *)&sx1278_SPI_defaultParams);
	if(sx1278Handle == NULL)
	{
	  	//SPI open failed
		return FALSE;    
	}
		
	return TRUE; 
}

/*********************************************************************
 * @fn      sx1278_SPI_Write
 *
 * @brief   Write data to SPI.
 *
 * @param   none
 *
 * @return  TRUE if  Write successful
 */
bool sx1278_SPI_Write(const uint8_t *txbuf, size_t txlen)
{
    SPI_Transaction sx1278Transaction;
	
	sx1278Transaction.count  = txlen;
    sx1278Transaction.txBuf  = (void*)txbuf;
    sx1278Transaction.arg    = NULL;
    sx1278Transaction.rxBuf  = NULL;
	
    return SPI_transfer(sx1278Handle, &sx1278Transaction) ? 1 : 0;
}

/*********************************************************************
 * @fn      sx1278_SPI_Read
 *
 * @brief   Read data From SPI.
 *
 * @param   none
 *
 * @return  TRUE if  Read successful
 */
bool sx1278_SPI_Read(const uint8_t *rxbuf, size_t rxlen)
{
    SPI_Transaction sx1278Transaction;
	
	sx1278Transaction.count  = rxlen;
    sx1278Transaction.rxBuf  = (void*)rxbuf;
    sx1278Transaction.arg    = NULL;
    sx1278Transaction.txBuf  = NULL;
	
	return SPI_transfer(sx1278Handle, &sx1278Transaction) ? 1 : 0;
}

/*********************************************************************
 * @fn      Open_sx1278_PINs
 *
 * @brief   Open sx1278 PINs.
 *
 * @param   none
 *
 * @return  TRUE if  sx1278_PINs Open successful
 */
bool Open_sx1278_PINs(void) 
{
	sx1278Pin = PIN_open(&sx1278PinState, sx1278PinTable);
	
	if(sx1278Pin == NULL)
	  return FALSE;
	
	/* Delay ~10 ms for sx1278 to be powered up. */
    Task_sleep(10*1000/Clock_tickPeriod);
	
    /* Clear reset (set high)*/
    PIN_setOutputValue(sx1278Pin, Board_SX1278_RST, 1);
   
	return TRUE;
}

bool sx1278_StatusPin_Enable(LoraRFStatusCB_t loraRFStatusCB)
{
  	if(RFStatusPin)
   		PIN_close(RFStatusPin);
	
	RFStatusPin = PIN_open(&sx1278PinState, sx1278RFStatusPinTable);
	if(!RFStatusPin)
		return FALSE; 
	
	PIN_registerIntCb(RFStatusPin, Board_sx1278LoraStatusCallback);
	
	PIN_setConfig(RFStatusPin, PIN_BM_IRQ, Board_SX1278_DIO0 | PIN_IRQ_POSEDGE);
	
#ifdef POWER_SAVING
  	PIN_setConfig(RFStatusPin, PINCC26XX_BM_WAKEUP, Board_SX1278_DIO0 | PINCC26XX_WAKEUP_POSEDGE);
#endif
	
  	// Set the application callback
  	appLoraRFStatusHandler = loraRFStatusCB;
	
	return TRUE;
}

void sx1278_StatusPin_Disable(void)
{ 
    if(RFStatusPin)
    	PIN_close(RFStatusPin);
	
    sx1278RFStatusPinTable[0] =  Board_SX1278_DIO0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_OPENDRAIN;
	
	RFStatusPin = PIN_open(&sx1278PinState, sx1278RFStatusPinTable);
	if(!RFStatusPin)
	  while(1);	 
}

static void Board_sx1278LoraStatusCallback(PIN_Handle hPin, PIN_Id pinId)
{
	if (appLoraRFStatusHandler != NULL)
  	{
    	// Notify the application
		(*appLoraRFStatusHandler)(pinId);
	}
}

/*********************************************************************
 * @fn      Read_sx1278Dio0_Pin
 *
 * @brief   Read sx1278¡ªDio0 Status.
 *
 * @param   none
 *
 * @return Pin status 
 */
uint8_t Read_sx1278Dio0_Pin(void)
{
    return PIN_getInputValue(Board_SX1278_DIO0);
}

/*********************************************************************
 * @fn      Reset_sx1278
 *
 * @brief   Reset sx1278.
 *
 * @param   none
 *
 * @return none 
 */
void Reset_sx1278(void)
{
	PIN_setOutputValue(sx1278Pin, Board_SX1278_RST, 0);
}

/*********************************************************************
 * @fn      Enable_sx1278
 *
 * @brief   Enable sx1278.
 *
 * @param   none
 *
 * @return  none 
 */
void Enable_sx1278(void)
{
	PIN_setOutputValue(sx1278Pin, Board_SX1278_RST, 1);
}

void sx1278_SPI_CSN(bool enable)
{
	if(enable)
		PIN_setOutputValue(sx1278Pin, Board_SX1278_CSN, 1);
	else
	    PIN_setOutputValue(sx1278Pin, Board_SX1278_CSN, 0);
}
