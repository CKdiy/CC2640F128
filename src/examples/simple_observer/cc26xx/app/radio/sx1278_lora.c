/*
  Filename:       sx1278_lora.c
  Description:    Transplant official BSP
  Revised:        $Date: 2018-10-27  $
  Revision:       $Revision: ck $
 */

#include <string.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include "sx1278_lora.h"
   

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0

#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0 

#define REGVERSION_DEFAULT         0x12 
#define RF_DEFAULT_RXPACKET_LEN    1<<4

uint8_t LoraRxBuffer[RF_DEFAULT_RXPACKET_LEN];     

// Default settings
const LoRaSettings_t LoRaSettings =
{
    496330000,        // RFFrequency
	7,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    20,               // Power
    8,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	FALSE,            // LowDatarateOptimize
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
	8,                //PreambleLen
	0,                //FixLen
	64,               // PayloadLength (used for implicit header mode)
    TRUE,             // CrcOn [0: OFF, 1: ON]
    0,                // FreqHopOn [0: OFF, 1: ON]
	0,                // HopPeriod Hops every frequency hopping period symbols
	TRUE,             // IqInverted
    TRUE,             // RxSingleOn [0: Single , 1 Continuous]
};

//SX1278 LoRa registers variable 
SX1276_t    SX1278;

/*******************************************************************/
// sx1278 LORA Functions 

bool sx1278_GetLoRaOn(void);
void sx1278_WriteData(uint8_t addr, uint8_t data);
void sx1278_WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size);
void sx1278_ReadData(uint8_t addr, uint8_t *data);
void sx1278_ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size);
//******************************************************************************
/* sx1278Lora Initialize/configuration/control functions */

/**延时函数**/
void sx1278DelayMs(void)
{
	uint16_t i,j;
	
	for(i=0;i<300;i++)
	{
		for(j=0;j<300;j++);
	}
}

/* SX1278初始化 */
uint8_t sx1278Init(void)
{
  	uint8_t i,def_val;
	
	RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;
	
    if(!Open_sx1278_PINs())
	  return FALSE;
	
	if(!Open_sx1278_SPI())
	  return FALSE;
	
	// REMARK: See SX1276 datasheet for modified default values(0x12).
   	sx1278_ReadData( REG_LR_VERSION, &def_val );
	if(REGVERSION_DEFAULT == def_val)
	{
		sx1278_SetOpMode( RFLR_OPMODE_SLEEP );
		for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
		{
			sx1278_SetModem( RadioRegsInit[i].Modem );
			sx1278_WriteData( RadioRegsInit[i].Addr, RadioRegsInit[i].value );
		}
		
		sx1278_SetModem( MODEM_LORA );
		SX1278.Modem = MODEM_LORA;
		
		return TRUE;
	}
	
    return FALSE;		
}

/*  设置SX1278的LORA参数 */
void sx1278_SetLoraPara(LoRaSettings_t *ptr)
{
	uint8_t paConfig = 0;
	uint8_t paDac = 0,Sf;
	uint8_t power;
	uint8_t bandwidth;
	uint8_t res;
	
    if(ptr == NULL)
		ptr = (LoRaSettings_t *)&LoRaSettings;  
	
	sx1278_SetRFChannel( ptr->Channel );
	sx1278_SetModem( SX1278.Modem );
	
    sx1278_ReadData( REG_PACONFIG, &paConfig); 
    sx1278_ReadData( REG_PADAC, &paDac); 
    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
	
	power = ptr->Power;
	if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
		if( power > 17 )
        {
        	paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
		
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
	
	sx1278_WriteData( REG_PACONFIG, paConfig );
	sx1278_WriteData( REG_PADAC, paDac );
	SX1278.LoRa.Power = power;
	
	if(ptr->Sf < 6)	
	{
		ptr->Sf = 6;  	
	}
	else if(ptr->Sf >12)
	{
		ptr->Sf = 12;
	}
	
	Sf = ptr->Sf;
	
	SX1278.LoRa.Bandwidth 		= ptr->Bandwidth;
	SX1278.LoRa.Sf        		= ptr->Sf;
	SX1278.LoRa.Coderate  		= ptr->Coderate;
	SX1278.LoRa.PreambleLen 	= ptr->PreambleLen;
	SX1278.LoRa.FixLen      	= ptr->FixLen;
	SX1278.LoRa.FreqHopOn   	= ptr->FreqHopOn;
	SX1278.LoRa.HopPeriod   	= ptr->HopPeriod;
	SX1278.LoRa.CrcOn       	= ptr->CrcOn;
	SX1278.LoRa.IqInverted  	= ptr->IqInverted;
	SX1278.LoRa.RxContinuous 	= ptr->RxContinuous;
	
	bandwidth = SX1278.LoRa.Bandwidth;
	if( ( ( bandwidth == 7 ) && ( ( Sf == 11 ) || ( Sf == 12 ) ) ) ||
		( ( bandwidth == 8 ) && ( Sf == 12 ) ) )
	{
		SX1278.LoRa.LowDatarateOptimize = 0x01;
	}
	else
	{
		SX1278.LoRa.LowDatarateOptimize = 0x00;
	}
	
	if( SX1278.LoRa.FreqHopOn == TRUE )
	{
	    sx1278_ReadData( REG_LR_PLLHOP, &res);
		sx1278_WriteData( REG_LR_PLLHOP, ( res & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
		sx1278_WriteData( REG_LR_HOPPERIOD, SX1278.LoRa.HopPeriod );
	}
	
	sx1278_ReadData( REG_LR_MODEMCONFIG1, &res);
	sx1278_WriteData( REG_LR_MODEMCONFIG1, ( res & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK &
				   		RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( SX1278.LoRa.Bandwidth << 4 ) | ( SX1278.LoRa.Coderate << 1 ) | SX1278.LoRa.FixLen );
	
	sx1278_ReadData( REG_LR_MODEMCONFIG2, &res);
	sx1278_WriteData( REG_LR_MODEMCONFIG2, ( res & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
					 	( SX1278.LoRa.Sf << 4 ) | ( SX1278.LoRa.CrcOn  << 2 ) );	
	
	sx1278_ReadData( REG_LR_MODEMCONFIG3, &res);
	sx1278_WriteData( REG_LR_MODEMCONFIG3,( res & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
				 	  	( SX1278.LoRa.LowDatarateOptimize << 3 ) );	
	
	sx1278_WriteData( REG_LR_PREAMBLEMSB, ( SX1278.LoRa.PreambleLen >> 8 ) & 0x00FF );
	sx1278_WriteData( REG_LR_PREAMBLELSB, SX1278.LoRa.PreambleLen & 0xFF );

	if( ( bandwidth == 9 ) && ( SX1278.LoRa.Channel > RF_MID_BAND_THRESH ))
	{
		sx1278_WriteData( REG_LR_TEST36, 0x02 );
		sx1278_WriteData( REG_LR_TEST3A, 0x64 );
	}
	else if( bandwidth == 9 )
	{
		sx1278_WriteData( REG_LR_TEST36, 0x02 );
		sx1278_WriteData( REG_LR_TEST3A, 0x7F );
	}
	else
	{
		sx1278_WriteData( REG_LR_TEST36, 0x03 );
	}
	
	if( Sf == 6 )
	{
	  	sx1278_ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_WriteData( REG_LR_DETECTOPTIMIZE,( res & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF6 );
		
		sx1278_WriteData( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6 );
	}
	else
	{
	  	sx1278_ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_WriteData( REG_LR_DETECTOPTIMIZE,( res & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
		sx1278_WriteData( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
	}
}

/* 设置SX1278发送数据 */
void sx1278_SendBuf(uint8_t *pkt, uint8_t pkt_size)
{
	uint8_t res;   
  
	if( NULL == pkt )
	  return;
	
	sx1278_SetOpMode( RFLR_OPMODE_STANDBY );
	
	sx1278DelayMs();
	
	if( SX1278.LoRa.IqInverted == TRUE )
	{
	  	sx1278_ReadData( REG_LR_INVERTIQ, &res);
		sx1278_WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
		sx1278_WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
	}
	else
	{
	  	sx1278_ReadData( REG_LR_INVERTIQ, &res);
		sx1278_WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
	}

	sx1278_WriteData( REG_LR_PAYLOADLENGTH, pkt_size );
	sx1278_WriteData( REG_LR_FIFOTXBASEADDR, 0 );
	sx1278_WriteData( REG_LR_FIFOADDRPTR, 0 );
	
	sx1278_ReadData(REG_OPMODE, &res);
	if( ( res & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
	{
		sx1278_SetOpMode( RFLR_OPMODE_STANDBY );
		sx1278DelayMs();
	}
	
	sx1278_WriteBuf(0, pkt, pkt_size);
	
	sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
	sx1278_WriteData( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE |
						RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | 
						RFLR_IRQFLAGS_CADDONE |RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED );
	
	sx1278_ReadData( REG_DIOMAPPING1, &res);
	sx1278_WriteData( REG_DIOMAPPING1, ( res & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
	sx1278_ReadData( REG_DIOMAPPING2, &res);
	sx1278_WriteData( REG_DIOMAPPING2, ( res & RFLR_DIOMAPPING2_DIO4_MASK ) | RFLR_DIOMAPPING2_DIO4_00 );
	
    sx1278_SetOpMode( RF_OPMODE_TRANSMITTER );   
	
	sx1278_ReadData(REG_LR_IRQFLAGS, &res); 
	SX1278.State = RF_TX_RUNNING;
}

/* 设置SX1278进入接收模式 */
void sx1278_EnterRx(void)
{
    uint8_t rxContinuous = false;
	uint8_t res;
	
	if( SX1278.LoRa.IqInverted == TRUE )
	{
	    sx1278_ReadData( REG_LR_INVERTIQ, &res);
		sx1278_WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
	}
	else
	{
	    sx1278_ReadData( REG_LR_INVERTIQ, &res);
		sx1278_WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
	}
	
	if( SX1278.LoRa.Bandwidth < 9 )
	{
	  	sx1278_ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_WriteData( REG_LR_DETECTOPTIMIZE, res & 0x7F );
		sx1278_WriteData( REG_LR_TEST30, 0x00 );
		
		switch( SX1278.LoRa.Bandwidth )
		{
			case 0:                     sx1278_WriteData( REG_LR_TEST2F, 0x48 );
				sx1278_SetRFChannel(SX1278.LoRa.Channel + (uint32_t)(7.81e3 ));
			break;
			
			case 1:                     sx1278_WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_SetRFChannel(SX1278.LoRa.Channel + (uint32_t)(10.42e3 ));
			break;
			
			case 2:                     sx1278_WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_SetRFChannel(SX1278.LoRa.Channel + (uint32_t)(15.62e3 ));
			break;
			
			case 3:                     sx1278_WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_SetRFChannel(SX1278.LoRa.Channel + (uint32_t)(20.83e3 ));
			break;
			
			case 4:                     sx1278_WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_SetRFChannel(SX1278.LoRa.Channel + (uint32_t)(31.25e3 ));
			break;
			
			case 5:                     sx1278_WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_SetRFChannel(SX1278.LoRa.Channel + (uint32_t)(41.67e3 ));
			break;
			
			case 6:                     sx1278_WriteData( REG_LR_TEST2F, 0x40 );
				break;
				
			case 7:                     sx1278_WriteData( REG_LR_TEST2F, 0x40 );
				break;
				
			case 8:                     sx1278_WriteData( REG_LR_TEST2F, 0x40 );
				break;
		}
	}
	else
	{
	  	sx1278_ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_WriteData( REG_LR_DETECTOPTIMIZE, res | 0x80 );
	}
	
	sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
	rxContinuous = SX1278.LoRa.RxContinuous;
	if( SX1278.LoRa.FreqHopOn == TRUE )
	{
		sx1278_WriteData( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER |
						  RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_CADDETECTED );
		
		sx1278_ReadData( REG_DIOMAPPING1, &res);
		sx1278_WriteData( REG_DIOMAPPING1, ( res & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00);
		
		sx1278_ReadData( REG_DIOMAPPING2, &res);
		sx1278_WriteData( REG_DIOMAPPING2, ( res & RFLR_DIOMAPPING2_DIO4_MASK ) | RFLR_DIOMAPPING2_DIO4_10 );
	}
	else
	{
		sx1278_WriteData( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | 
						 RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED );
		
		sx1278_ReadData( REG_DIOMAPPING1, &res);
		sx1278_WriteData( REG_DIOMAPPING1, ( res & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
		
		sx1278_ReadData( REG_DIOMAPPING2, &res);
		sx1278_WriteData( REG_DIOMAPPING2, ( res & RFLR_DIOMAPPING2_DIO4_MASK ) | RFLR_DIOMAPPING2_DIO4_10 );
	}
	
	sx1278_SetOpMode( RFLR_OPMODE_STANDBY );
	sx1278DelayMs();
	
	sx1278_WriteData( REG_LR_FIFORXBASEADDR, 0 );
	sx1278_WriteData( REG_LR_FIFOADDRPTR, 0 );

    if( rxContinuous == TRUE )
    {
        sx1278_SetOpMode( RFLR_OPMODE_RECEIVER );
    }
    else
    {
        sx1278_SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
	
	sx1278_ReadData(REG_LR_IRQFLAGS, &res); 
	SX1278.State = RF_RX_RUNNING;
	SX1278.Size  = 0;
}

/* 设置SX1278调制模式 */
void sx1278_SetModem( RadioModems_t modem )
{
    uint8_t 	res;
	
    SX1278.Modem = modem;
	sx1278_SetOpMode( RF_OPMODE_SLEEP );
	sx1278_ReadData( REG_OPMODE, &res); 
	sx1278_WriteData( REG_OPMODE, ( res & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );
	sx1278_WriteData( REG_DIOMAPPING1, 0x00 );        
	sx1278_WriteData( REG_DIOMAPPING2, 0x00 );
}

/* 设置SX1278模式 */
void sx1278_SetOpMode(uint8_t opMode)
{
    uint8_t res;
	
	sx1278_ReadData( REG_OPMODE, &res);
		
    sx1278_WriteData(REG_LR_OPMODE, (res & RF_OPMODE_MASK) | opMode);    
}

/* 设置SX1278发射或接收频点 */
void sx1278_SetRFChannel( uint32_t freq )
{	
    SX1278.LoRa.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
	sx1278_WriteData( REG_LR_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ));
	sx1278_WriteData( REG_LR_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ));
	sx1278_WriteData( REG_LR_FRFLSB, ( uint8_t )( freq & 0xFF ));
}

/* 查询SX1278信道，是否空闲 */
uint8_t sx1278IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    int16_t rssi = 0;
    uint8_t i;
	  
    sx1278_SetModem( modem );
    sx1278_SetRFChannel( freq );
    sx1278_SetOpMode( RF_OPMODE_RECEIVER );
	
    for( i=0; i<10; i++ )
    {
        sx1278DelayMs();
        rssi += sx1278_ReadRssi( modem );
    }
	    
    sx1278_SetOpMode( RFLR_OPMODE_SLEEP );
	
    rssi = rssi/10;
	
    if( rssi > rssiThresh )
    {
        return FALSE;
    }
    return TRUE;
}

/* 读当前接收包的信号强度 */
int16_t sx1278_ReadRssi( RadioModems_t modem )
{
    uint8_t res;
    int16_t rssi = 0;
	
	sx1278_ReadData( REG_LR_RSSIVALUE, &res);	
	rssi = (int16_t)(RSSI_OFFSET_LF + res);
	
    return rssi;
}

/* 设置接收或发射，最大字节数 */
void sx1278_SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    sx1278_SetModem( modem );
	
    sx1278_WriteData( REG_LR_PAYLOADMAXLENGTH, max );
}

/* SX1278模块接收到数据，MCU读取其FIFO并打印，并解析RSSI,SNR */
void sx1278_ReadRxPkt(void)
{
	uint8_t irqFlags;
	int16_t rssi,snr;
	uint8_t	payload_len;	
	
	SX1278.Size = 0;
	sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );
	sx1278_ReadData( REG_LR_IRQFLAGS, &irqFlags);

	if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
	{
		sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
	}

	sx1278_ReadData( REG_LR_PKTSNRVALUE, (uint8_t *)&SX1278.SnrValue);
	if( SX1278.SnrValue & 0x80 ) 	
	{
		snr = ( ( ~SX1278.SnrValue + 1 ) & 0xFF ) >> 2;
		snr = -snr;
	}
	else
	{
		snr = ( SX1278.SnrValue & 0xFF ) >> 2;
	}
	SX1278.SnrValue = snr;
	
	sx1278_ReadData( REG_LR_PKTRSSIVALUE, (uint8_t *)&rssi);
	if( snr < 0 )
	{
		SX1278.RssiValue = (int16_t)(RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) + snr);
	}
	else
	{
		SX1278.RssiValue = (int16_t)(RSSI_OFFSET_LF + rssi + ( rssi >> 4 ));
	}

	sx1278_ReadData( REG_LR_RXNBBYTES, &payload_len);
	
	if(RF_DEFAULT_RXPACKET_LEN < payload_len)
	{
		payload_len = RF_DEFAULT_RXPACKET_LEN;
	}
	
	sx1278_ReadBuf(0, LoraRxBuffer, payload_len);
	SX1278.Size = payload_len;
}

/* 清除发送中断标志位 */
void sx1278_TxDoneCallback(void)
{
	sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
}

/* 设置SX1278进入Sleep模式 */
void sx1278_SetSleep(void)
{
	sx1278_SetOpMode( RFLR_OPMODE_SLEEP );
	SX1278.State = RF_IDLE;
}

/* 设置SX1278进入Standbay模式 */
void sx1278_SetStby(void)
{
	sx1278_SetOpMode( RFLR_OPMODE_STANDBY );
}

/* 获取SX1278接收到的数据 */
void *sx1278_GetRxData(uint8_t *size)
{
    if(size == NULL)
	  return NULL;
	
	if(RF_RX_RUNNING == SX1278.State)
	{
	 	if(SX1278.Size != 0)
		{
			*size = SX1278.Size;	
			SX1278.Size = 0;
		    return (void *)LoraRxBuffer;
		}	   
	}
	
	return NULL;
}

//******************************************************************************
/* sx1278 Read and write register functions */
void  sx1278_WriteData(uint8_t addr, uint8_t data)
{	
 	uint8_t wAddres;
	uint8_t wdata;
	
	wAddres = addr | 0x80; 
	wdata   = data;
	
	sx1278_SPI_CSN(FALSE);
 	sx1278_SPI_Write(&wAddres, 1);
	sx1278_SPI_Write(&wdata, 1);
	sx1278_SPI_CSN(TRUE);	
}

void  sx1278_WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size)
{
 	uint8_t wAddres;
	
    wAddres = addr | 0x80;
	
	sx1278_SPI_CSN(FALSE);
	sx1278_SPI_Write(&wAddres, 1);
	sx1278_SPI_Write(buf, size);
	sx1278_SPI_CSN(TRUE);	
}

void  sx1278_ReadData(uint8_t addr, uint8_t *data)
{   
  	uint8_t rAddres;
	
   	rAddres = addr & 0x7F;  
	
	sx1278_SPI_CSN(FALSE);
 	sx1278_SPI_Write(&rAddres, 1);	
 	sx1278_SPI_Read(data, 1);
	sx1278_SPI_CSN(TRUE);
}

void  sx1278_ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size)
{
  	uint8_t rAddres;
	
   	rAddres = addr & 0x7F; 
	
	sx1278_SPI_CSN(FALSE);
 	sx1278_SPI_Write(&rAddres, 1);
	sx1278_SPI_Read(buf, size);
	sx1278_SPI_CSN(TRUE);
}