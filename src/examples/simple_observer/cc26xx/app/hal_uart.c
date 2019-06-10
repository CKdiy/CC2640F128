/*
  Filename:       uart.c
  Revised:        $Date: 2019-03-13  $
  Revision:       $Revision:  $
 */
#ifndef DEMO

#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <string.h>
#include "Board.h"
#include "hal_uart.h"



static UART_Handle      uartHandle;

static uint8_t uart0RxBuff[UART0_RECEICE_BUFF_SIZE];
volatile uint8_t rx_buff_header,rx_buff_tailor;
/*********************************************************************
 * @fn      Close_uart0
 *
 * @brief   Close uart0.
 *
 * @param   none
 *
 * @return  none
 */
void Close_uart0(void)
{
	UART_close(uartHandle);
}

/*********************************************************************
 * @fn      Open_uart0
 *
 * @brief   Open uart0.
 *
 * @param   none
 *
 * @return  TRUE if  Uart0 Open successful
 */
bool Open_uart0(UART_Callback appuartCB)
{  
  	UART_Params      uartParams;
	
	Board_initUART();
	
	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.baudRate 			= 9600;
	uartParams.writeDataMode	 	= UART_DATA_BINARY;
	uartParams.readDataMode 		= UART_DATA_BINARY;
	uartParams.dataLength     		= UART_LEN_8;
	uartParams.stopBits          	= UART_STOP_ONE;
	uartParams.readMode             = UART_MODE_CALLBACK;
	uartParams.readCallback         = appuartCB;
	uartParams.readReturnMode 		= UART_RETURN_FULL;
	uartParams.readEcho 			= UART_ECHO_OFF;
	
	uartHandle = UART_open(Board_UART0, &uartParams);
	if( NULL == uartHandle )
	{
	  return FALSE;
	}
	else
	{
		UART_control(uartHandle, UART_CMD_RESERVED, NULL);
		UART_read(uartHandle, uart0RxBuff, UART0_RECEICE_BUFF_SIZE);
	}
	
	rx_buff_header = 0;
	rx_buff_tailor = 0;
	
	return TRUE; 
}

/*********************************************************************
 * @fn      Uart0_Write
 *
 * @brief   Write data to Uart.
 *
 * @param   none
 *
 * @return  TRUE if  Write successful
 */
bool Uart0_Write(const uint8_t *wbuf, size_t wlen)
{
	if(UART_write(uartHandle, wbuf, wlen) == UART_STATUS_ERROR)
	  return FALSE;
	
	return TRUE;
}

#endif 
