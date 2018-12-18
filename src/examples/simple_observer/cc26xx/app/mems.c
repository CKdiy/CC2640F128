/*
  Filename:       MEMS.c
  Revised:        $Date: 2018-11-11  $
  Revision:       $Revision: ck $
 */

#include <xdc/std.h>
#include <stdbool.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include "Board.h"
#include "mems.h"

#ifdef USE_ICALL
#include <icall.h>
#endif

/* Mems pin table */
PIN_Config memsSCLPinTable[] = {
	Board_I2C_SCL   | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  | PIN_OPENDRAIN | PIN_DRVSTR_MAX, 	   
	PIN_TERMINATE                                                               // Terminate list
};

PIN_Config memsSDAPinTable[] = {
	Board_I2C_SDA   | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  | PIN_OPENDRAIN,
	PIN_TERMINATE                                                               // Terminate list
};

PIN_Config memsActivePinTable[]={
   Board_I2C_INT    | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN   |  PIN_PULLUP,
   PIN_TERMINATE                                                                // Terminate list
};

// Pointer to application callback
MemsActiveCB_t appMemsActiveHandler = NULL;

//static I2C_Handle   memsHandle;
static PIN_Handle memsPinSCL;
static PIN_Handle memsPinSDA;
static PIN_Handle memsPinActive;

static PIN_State  memsSCLPinState; 
static PIN_State  memsSDAPinState; 
static PIN_State  memsActivePinState; 

uint8_t transBuf; 

/*********************Mems functions ******************************************/
void usDelay(volatile uint16_t delay);
static bool Mems_ReadReg(uint8_t Reg, uint8_t* Data);
static bool Mems_WriteReg(uint8_t Reg, uint8_t Data);
static void Board_MemsActivedCallback(PIN_Handle hPin, PIN_Id pinId);
/********************************************
* Mems functions *
********************************************/
bool MemsOpen(void)
{	
	memsPinSCL = PIN_open(&memsSCLPinState, memsSCLPinTable);
	if(!memsPinSCL)
		return FALSE;  
	
	memsPinSDA = PIN_open(&memsSDAPinState, memsSDAPinTable);
	if(!memsPinSDA)
	 	return FALSE; 
	
	usDelay(1);
	
	//read chip id
	Mems_ReadReg(0x01, &transBuf);
    if(transBuf != 0x13)
    	return FALSE;
	
	//Mode: normal
	transBuf = 0x35;
	Mems_WriteReg(0x11, transBuf);
	
	//ODR=125HZ
	transBuf = 0x07;
	Mems_WriteReg(0x10, transBuf);	
	
	//Set active_ths default:g_Rang +/-2g 
	//threshold of active interrupt=Active_th*K(mg)
	//K = 3.91(2g range)
	//K = 7.81(4g range)
	//K = 15.625(8g range)
	//K = 31.25(16g range)
	transBuf = 50;
	Mems_WriteReg(0x28, transBuf);
	
	//Enable active interrupt
	transBuf = 0x07;
	Mems_WriteReg(0x16, transBuf);			
	
	//mapping active interrupt to INT1
	transBuf = 0x04;
	Mems_WriteReg(0x19, transBuf);	
	
	return TRUE;
}

void MemsLowPwMode(void)
{
	//Mode: suspend
	transBuf = 0x35 | (1<<3);
	Mems_WriteReg(0x11, transBuf);
}

void MemsLowPwMgr(void)
{
    if(memsPinSCL)
		PIN_close(memsPinSCL);
	
	memsSCLPinTable[0] = Board_I2C_SCL | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP;
	memsPinSCL = PIN_open(&memsSCLPinState, memsSCLPinTable);
	
	if(memsPinSDA)
		PIN_close(memsPinSDA);
	
	memsSDAPinTable[0] = Board_I2C_SDA | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP;
	memsPinSDA = PIN_open(&memsSDAPinState, memsSDAPinTable);
}

/* 初始化完成后建议关闭，释放内存 */
void MemsClose(void)
{
    if(memsPinSCL)
		PIN_close(memsPinSCL);
	
	if(memsPinSDA)
		PIN_close(memsPinSDA);
}

/* 使能Mems的Actived中断 */
bool Mems_ActivePin_Enable(MemsActiveCB_t memsActiveCB)
{
	if(memsPinActive)
		PIN_close(memsPinActive);
	
	memsPinActive = PIN_open(&memsActivePinState, memsActivePinTable);
	if(!memsPinActive)
		return FALSE; 
	
	PIN_registerIntCb(memsPinActive, Board_MemsActivedCallback);
	
	PIN_setConfig(memsPinActive, PIN_BM_IRQ, Board_I2C_INT | PIN_IRQ_POSEDGE);
	
#ifdef POWER_SAVING
  	PIN_setConfig(memsPinActive, PINCC26XX_BM_WAKEUP, Board_I2C_INT | PINCC26XX_WAKEUP_POSEDGE);
#endif
	
  	// Set the application callback
  	appMemsActiveHandler = memsActiveCB;
	
	return TRUE;
}

void Mems_ActivePin_Disable(void)
{
   	if(memsPinActive)
   		PIN_close(memsPinActive);
	
	memsActivePinTable[0] =  Board_I2C_INT | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_OPENDRAIN;
	
	memsPinActive = PIN_open(&memsActivePinState, memsActivePinTable);
	if(!memsPinActive)
		while(1);
}

static void Board_MemsActivedCallback(PIN_Handle hPin, PIN_Id pinId)
{
	if (appMemsActiveHandler != NULL)
  	{
    	// Notify the application
		(*appMemsActiveHandler)(pinId);
	}
}

/********************************************
* IIC Functions * 
********************************************/
static void usDelay(volatile uint16_t delay)
{
	volatile uint16_t i;
	while(delay--)
	{
	  	i = 5;
		for(;i>0;i--);
	}
}

static void SDA_In(void)
{
	PIN_close(memsPinSDA);
	
	memsSDAPinTable[0] = Board_I2C_SDA   | PIN_INPUT_EN  |  PIN_NOPULL ;
	memsPinSDA = PIN_open(&memsSDAPinState, memsSDAPinTable); 
	if(!memsPinSDA)
		while(1);	 
}

static void SDA_Out(void)
{
  	PIN_close(memsPinSDA);
	
	memsSDAPinTable[0] = Board_I2C_SDA   | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH| PIN_OPENDRAIN| PIN_DRVSTR_MAX;
	memsPinSDA = PIN_open(&memsSDAPinState, memsSDAPinTable); 
	if(!memsPinSDA)
		while(1);	 
}

static void SDA_OutPutBit(tBitAction bit)
{
    if(bit == Bit_RESET)
        PIN_setOutputValue(memsPinSDA,Board_I2C_SDA,0);
    else
        PIN_setOutputValue(memsPinSDA,Board_I2C_SDA,1); 
}

static void SCL_OutPutBit(tBitAction bit)
{
    if(bit == Bit_RESET)
        PIN_setOutputValue(memsPinSCL,Board_I2C_SCL,0);
    else
        PIN_setOutputValue(memsPinSCL,Board_I2C_SCL,1);  
}

static uint8_t Read_SDA(void)
{
    return PIN_getInputValue(Board_I2C_SDA);            
}

void IIC_Start(void)
{
    SDA_OutPutBit(Bit_SET);
    usDelay(1);
    SCL_OutPutBit(Bit_SET);
    usDelay(2);
    
    SDA_OutPutBit(Bit_RESET);
    usDelay(2);
    
    /* 钳住I2C总线，准备发送或接收数据 */
    SCL_OutPutBit(Bit_RESET);
	usDelay(2);
}

void IIC_Stop(void)
{  
    SDA_OutPutBit(Bit_RESET);   
    usDelay(1);
    
    SCL_OutPutBit(Bit_SET); 
    usDelay(2);

    SDA_OutPutBit(Bit_SET);	 
    usDelay(2);							   	
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    
	usDelay(1);
	SDA_OutPutBit(Bit_SET);
	usDelay(1);
    SDA_In();
	
    SCL_OutPutBit(Bit_SET);    
    
    while(Read_SDA())
    {
        ucErrTime++;
        usDelay(1);
        if(ucErrTime > (20))
        {
            IIC_Stop();
            
            return 1;
        }
    }
    
    SCL_OutPutBit(Bit_RESET);
    usDelay(1);
    return 0;  
}

void IIC_Ack(void)
{
    SCL_OutPutBit(Bit_RESET);
    
    SDA_Out();
    
    SDA_OutPutBit(Bit_RESET);	 
    usDelay(2);
    
    SCL_OutPutBit(Bit_SET);
    usDelay(2);
    
    SCL_OutPutBit(Bit_RESET);
}
 
void IIC_NAck(void)
{
    SCL_OutPutBit(Bit_RESET);
    
    SDA_Out();
    SDA_OutPutBit(Bit_SET);    
    usDelay(2);
    
    SCL_OutPutBit(Bit_SET);   
    usDelay(2);
    
    SCL_OutPutBit(Bit_RESET);
}
  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;
    uint8_t bit;
      
    SCL_OutPutBit(Bit_RESET);
	
    for(t = 0; t < 8; t++)
    {              
        bit =(txd & 0x80) >> 7;
        
        if(bit == Bit_RESET)
            SDA_OutPutBit(Bit_RESET);
        else
            SDA_OutPutBit(Bit_SET);
        
        txd <<= 1; 	  
        //usDelay(2);
        
        SCL_OutPutBit(Bit_SET);
        usDelay(2); 
        
        SCL_OutPutBit(Bit_RESET);
        usDelay(2);
    }
} 
  
uint8_t IIC_Read_Byte(uint8_t ack)
{
	uint8_t i,receive = 0;
    
	SDA_OutPutBit(Bit_SET);
    SDA_In();
	
    SCL_OutPutBit(Bit_RESET);
    
    for(i=0;i<8;i++ )
    {
        SCL_OutPutBit(Bit_SET);
        usDelay(1);
        
        receive <<= 1;
        
        if(Read_SDA())
            receive ++;  
        
        SCL_OutPutBit(Bit_RESET);   
        usDelay(1); 
    }	
    
    if ( !ack )
        IIC_NAck();
    else
        IIC_Ack(); 
    
    return receive;
}

static bool Mems_ReadReg(uint8_t Reg, uint8_t* Data) 
{    
  	SDA_Out();
    IIC_Start();  
    /* 写指令 */    
    IIC_Send_Byte(MEMS_SADW);		 
    if( IIC_Wait_Ack() )  
        return FALSE;
	
	SDA_Out();
    IIC_Send_Byte(Reg);   		
    if( IIC_Wait_Ack() ) 
        return FALSE;
	
	SDA_Out();
    IIC_Start();  	 
    /* 读指令 */    
    IIC_Send_Byte(MEMS_SADR);	     	 
    if( IIC_Wait_Ack() )
        return FALSE;
    
	SDA_Out();
    *Data = IIC_Read_Byte(0);		    	   
    IIC_Stop();			        
    
    return TRUE;			 
}

static bool Mems_WriteReg(uint8_t Reg, uint8_t Data) 
{ 
  	SDA_Out();
    IIC_Start();  
    /* 写指令 */       
    IIC_Send_Byte(MEMS_SADW);     	
    if( IIC_Wait_Ack() )
        return FALSE; 
    
	SDA_Out();
    IIC_Send_Byte(Reg);           
    if( IIC_Wait_Ack() )
        return FALSE;  
    
	SDA_Out();
    IIC_Send_Byte(Data);     						   
    if( IIC_Wait_Ack() )
        return FALSE;
    
    IIC_Stop();			            
    
    return TRUE;
}