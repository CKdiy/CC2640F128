/**************************************************************************************************
  Filename:       sx1278_lora.h
  Revised:        $Date: 2018-10-27  $
  Revision:       $Revision: ck  $

**************************************************************************************************/

#ifndef SX1278_LORA_H
#define SX1278_LORA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "sx1278_hal.h"
#include "sx1276_regs_fsk.h"
#include "sx1276_regs_lora.h"
#include "radio.h"
#include <xdc/std.h>


/*!
 * RF packet definition
 */
#define RF_BUFFER_SIZE_MAX                          256
#define RF_BUFFER_SIZE                              256
  
#define RFLR_IRQFLAGS_ALL                           0xFF  

#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
	{ MODEM_FSK , REG_LNA                , 0x23 },\
	{ MODEM_FSK , REG_RXCONFIG           , 0x1E },\
	{ MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
	{ MODEM_FSK , REG_AFCFEI             , 0x01 },\
	{ MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
	{ MODEM_FSK , REG_OSC                , 0x07 },\
	{ MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
	{ MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
	{ MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
	{ MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
	{ MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
	{ MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
	{ MODEM_FSK , REG_IMAGECAL           , 0x02 },\
	{ MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
	{ MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
	{ MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}                                                     \
  
#define RF_MID_BAND_THRESH                          525000000
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625
#define RX_BUFFER_SIZE                              250

#define SF_12                                       12
	
typedef struct
{
	RadioModems_t 	Modem;
	uint8_t       	Addr;
	uint8_t       	value;
}RadioRegisters_t;

typedef struct
{
	int8_t   	Power;
	uint32_t 	Fdev;
	uint32_t 	Bandwidth;
	uint32_t 	BandwidthAfc;
	uint32_t 	Datarate;
	uint16_t 	PreambleLen;
	uint8_t     FixLen;
	uint8_t  	PayloadLen;
	uint8_t     CrcOn;
	uint8_t     IqInverted;
	uint8_t     RxContinuous;
	uint32_t 	TxTimeout;
}RadioFskSettings_t;

typedef struct
{
	uint8_t  	PreambleDetected;
	uint8_t  	SyncWordDetected;
	int8_t   	RssiValue;
	int32_t  	AfcValue;
	uint8_t  	RxGain;
	uint16_t 	Size;
	uint16_t 	NbBytes;
	uint8_t  	FifoThresh;
	uint8_t  	ChunkSize;
}RadioFskPacketHandler_t;
	
typedef struct
{
	uint32_t    	Channel;
	uint32_t 		Bandwidth;
	uint8_t   		Power;
	uint8_t         Sf;
	uint8_t     	LowDatarateOptimize;
	uint8_t  		Coderate;
	uint16_t 		PreambleLen;
	uint8_t    		FixLen;
	uint8_t  		PayloadLen;
	uint8_t     	CrcOn;
	uint8_t     	FreqHopOn;
	uint8_t  		HopPeriod;
	uint8_t     	IqInverted;
	uint8_t     	RxContinuous;
}LoRaSettings_t;

typedef struct
{
	RadioModems_t 	Modem;
	RadioState_t    State;
	LoRaSettings_t  LoRa;
	int16_t 		SnrValue;
	int16_t 		RssiValue;
	uint8_t 		Size;
}SX1276_t;

/*********************************************************************
 * FUNCTIONS
 */
/* Initializes the SX1278 */  
uint8_t sx1278Init(void);
void sx1278_SetLoraPara(LoRaSettings_t *ptr);
void sx1278_SetModem( RadioModems_t modem );
void sx1278_SetRFChannel( uint32_t freq );
void sx1278_SetOpMode(uint8_t opMode);
void sx1278_SendBuf(uint8_t *pkt, uint8_t pkt_size);
void sx1278_EnterRx(void);
int16_t sx1278_ReadRssi( RadioModems_t modem );
void sx1278_SetMaxPayloadLength( RadioModems_t modem, uint8_t max );
void sx1278_ReadRxPkt(void);
void *sx1278_GetRxData(uint8_t *size);
void sx1278_TxDoneCallback(void);
void sx1278_SetSleep(void);
void sx1278_SetStby(void);

#ifdef __cplusplus
}
#endif

#endif /* SX1278_LORA_H */
