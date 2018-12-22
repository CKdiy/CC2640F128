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

/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};

const double RssiOffsetLF[] =
{   // These values need to be specify in the Lab
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
};

const double RssiOffsetHF[] =
{   // These values need to be specify in the Lab
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
};

#define REGVERSION_DEFAULT   0x12 
#define RF_PACKET_LEN        1<<6

tRadioDriver *sx1278Radio = NULL;

uint8_t RFBuffer[40];     

static bool LoRaOn = FALSE;
static bool LoRaOnState = FALSE;
static uint8_t RFLRState = RFLR_STATE_IDLE;
//static int8_t RxPacketSnrEstimate;
//static double RxPacketRssiValue;
static uint16_t RxPacketSize = 0;

//SX1278 registers variable 
uint8_t SX1278Regs[0x70];


//Frequency hopping frequencies table
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
tLoRaSettings LoRaSettings =
{
    434000000,        // RFFrequency
    20,                // Power
    7,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    10,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    TRUE,             // CrcOn [0: OFF, 1: ON]
    FALSE,            // ImplicitHeaderOn [0: OFF, 1: ON]
    0,                // RxSingleOn [0: Continuous, 1 Single]
    0,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    64,              // PayloadLength (used for implicit header mode)
};

//SX1278 LoRa registers variable 
tSX1276LR* SX1278LR;

/*******************************************************************/
// sx1278 LORA Functions 
void sx1278_SetLoRaOn(bool enable);
bool sx1278_GetLoRaOn(void);
void sx1278_WriteData(uint8_t addr, uint8_t data);
void sx1278_WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size);
void sx1278_ReadData(uint8_t addr, uint8_t *data);
void sx1278_ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size);

void sx1278Lora_SetRFFrequency( uint32_t freq );
void sx1278Lora_SetOpMode(uint8_t opMode);
void sx1278Lora_SetSpreadingFactor( uint8_t factor );
void sx1278Lora_SetErrorCoding( uint8_t value );
void sx1278Lora_SetPacketCrcOn( bool enable );
void sx1278Lora_SetSignalBandwidth( uint8_t bw );
void sx1278Lora_SetImplicitHeaderOn( bool enable );
void sx1278Lora_SetSymbTimeout( uint16_t value );
void sx1278Lora_SetPayloadLength( uint8_t value );
void sx1278Lora_SetLowDatarateOptimize( bool enable );
void sx1278Lora_SetPAOutput( uint8_t outputPin );
void sx1278Lora_SetPa20dBm( bool enale );
void sx1278Lora_SetRFPower( int8_t power );
void sx1278Lora_SetRFMode(bool mode);
uint8_t sx1278Lora_GetOpMode( void );
bool sx1278Lora_RFSendBuf( uint8_t *txBuf, size_t txLen);
tRFLRStates  sx1278Lora_Process(void);
static void sx1278Lora_SetParameters(void);

//******************************************************************************
/* sx1278Lora Initialize/configuration/control functions */
void sx1278Init(void)
{
	SX1278LR = (tSX1276LR *)SX1278Regs;
	
    if(!Open_sx1278_PINs())
	  return;
	
	 if(!Open_sx1278_SPI())
	   while(1);
	
	LoRaOn = TRUE;
	sx1278_SetLoRaOn(LoRaOn);  
}

void sx1278_SetLoRaOn(bool enable)
{
	if(LoRaOnState == enable)
    	return;
    
    LoRaOnState = enable;
    LoRaOn      = enable;
	
	if( LoRaOn == TRUE )
    {
	  	
	    // REMARK: See SX1276 datasheet for modified default values(0x12).
   	 	sx1278_ReadData( REG_LR_VERSION, &SX1278LR->RegVersion );
	
			//Just for testing SPI
   		if(SX1278LR->RegVersion != REGVERSION_DEFAULT)
			return; 
		
    	sx1278Lora_SetOpMode(RFLR_OPMODE_SLEEP);
        
		Task_sleep(10*1000/Clock_tickPeriod);
		
		SX1278LR->RegTcxo = 0x09;   //USE TCXO
		
		sx1278_WriteData(REG_LR_TCXO, SX1278LR->RegTcxo);
		
        SX1278LR->RegOpMode = ( SX1278LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_LOWFREQUENCYMODEON;
		
		//select lora mode
		sx1278_WriteData(REG_LR_OPMODE, SX1278LR->RegOpMode);
        
		sx1278Lora_SetRFFrequency(LoRaSettings.RFFrequency);
		
		if(LoRaSettings.RFFrequency > 200000000)
		{
			sx1278Lora_SetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
    		sx1278Lora_SetPa20dBm( TRUE );
			LoRaSettings.Power = 20;
    		sx1278Lora_SetRFPower( LoRaSettings.Power );
		}
		else
		{
			sx1278Lora_SetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
    		sx1278Lora_SetPa20dBm( FALSE );
			LoRaSettings.Power = 14;
    		sx1278Lora_SetRFPower( LoRaSettings.Power );		
		}
		
		//RegOcp,Close Ocp		
		SX1278LR->RegOcp = 0x0B;
		sx1278_WriteData(REG_LR_OCP, SX1278LR->RegOcp);
		
		//RegLNA,High & LNA Enable
		SX1278LR->RegLna = 0x23;
		sx1278_WriteData(REG_LR_LNA, SX1278LR->RegLna);
		
		sx1278Lora_SetParameters();
		
		sx1278Lora_SetSymbTimeout( 0x0FC );	
		
		SX1278LR->RegPreambleMsb = 0;
		SX1278LR->RegPreambleLsb = 6;
		sx1278_WriteData(REG_LR_PREAMBLEMSB, SX1278LR->RegPreambleMsb);
		sx1278_WriteData(REG_LR_PREAMBLELSB, SX1278LR->RegPreambleLsb); 				
		
		//RegDioMapping2 DIO5=00, DIO4=01
		SX1278LR->RegDioMapping2 = 0x01;
		sx1278_WriteData(REG_LR_DIOMAPPING2, SX1278LR->RegDioMapping2);
		sx1278Lora_SetOpMode(RFLR_OPMODE_STANDBY);
    }
    else
    {
        sx1278Lora_SetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1278LR->RegOpMode = ( SX1278LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        sx1278_WriteData(REG_LR_OPMODE, SX1278LR->RegOpMode);
        
        sx1278Lora_SetOpMode( RFLR_OPMODE_STANDBY );  
    }
	
	RFLRState = RFLR_STATE_IDLE;
}

static void sx1278Lora_SetParameters(void)
{
	// SF6 only operates in implicit header mode.
	sx1278Lora_SetSpreadingFactor(LoRaSettings.SpreadingFactor); 
	
	sx1278Lora_SetErrorCoding( LoRaSettings.ErrorCoding );	
	sx1278Lora_SetPacketCrcOn( LoRaSettings.CrcOn );
	sx1278Lora_SetSignalBandwidth( LoRaSettings.SignalBw );
	sx1278Lora_SetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    sx1278Lora_SetPayloadLength( LoRaSettings.PayloadLength );
    sx1278Lora_SetLowDatarateOptimize( TRUE );	 
}

void sx1278Lora_EntryTx(void)
{	  
	sx1278Lora_SetOpMode( RFLR_OPMODE_STANDBY );
	if( LoRaSettings.FreqHopOn == true )
    {
    	SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278LR->RegHopPeriod = LoRaSettings.HopPeriod;

        sx1278_ReadData( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
        sx1278Lora_SetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    else
    {
        SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278LR->RegHopPeriod = 0;
    }
	
	sx1278_WriteData( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
	
	                                    // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // PllLock              Mode Ready
    SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    sx1278_WriteBuf( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
	
	//Clear all irq
	sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
    sx1278_WriteData( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );
	
	// Full buffer used for Tx
	SX1278LR->RegFifoTxBaseAddr = 0x00; 
    sx1278_WriteData( REG_LR_FIFOTXBASEADDR, SX1278LR->RegFifoTxBaseAddr );
	
	SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoTxBaseAddr;
    sx1278_WriteData( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );	
}

void sx1278Lora_EntryRx(void)
{
	sx1278Lora_SetOpMode( RFLR_OPMODE_STANDBY );

	SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                //RFLR_IRQFLAGS_RXDONE |
                                RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                RFLR_IRQFLAGS_VALIDHEADER |
                                RFLR_IRQFLAGS_TXDONE |
                                RFLR_IRQFLAGS_CADDONE |
                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                RFLR_IRQFLAGS_CADDETECTED;
	
	sx1278_WriteData( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );

	if( LoRaSettings.FreqHopOn == true )
	{
		SX1278LR->RegHopPeriod = LoRaSettings.HopPeriod;

		sx1278_ReadData( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
		sx1278Lora_SetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
	}
	else
	{
		SX1278LR->RegHopPeriod = 255;
	}
        
    sx1278_WriteData( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
                
                                   // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
    SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                   // CadDetected               ModeReady
    SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    sx1278_WriteBuf( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
    
    if( LoRaSettings.RxSingleOn == true ) // Rx single mode
    {
        sx1278Lora_SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
    else // Rx continuous mode
    {
		// Full buffer used for Rx
		SX1278LR->RegFifoRxBaseAddr = 0x00;
        SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxBaseAddr;
        sx1278_WriteData( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
            
        sx1278Lora_SetOpMode( RFLR_OPMODE_RECEIVER );
    }
    
	sx1278_ReadData(REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags);  
	
    memset( RFBuffer, 0, ( size_t )RF_PACKET_LEN );
}

void sx1278Lora_SetOpMode(uint8_t opMode)
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = TRUE;
    bool antennaSwitchTxOn = FALSE;

    opModePrev = SX1278LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if(opMode != opModePrev)
    {
        if(opMode&0x03 == RFLR_OPMODE_TRANSMITTER)
        {
            antennaSwitchTxOn = TRUE;
        }
        else
        {
            antennaSwitchTxOn = FALSE;
        }
        if(antennaSwitchTxOn != antennaSwitchTxOnPrev)
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
			
			// Antenna switch control
			if(antennaSwitchTxOn)
			{
			  //RX/TX 
			}
        }
        SX1278LR->RegOpMode = ( SX1278LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		
        sx1278_WriteData(REG_LR_OPMODE, SX1278LR->RegOpMode); 
    }
}

uint8_t sx1278Lora_GetOpMode( void )
{
    sx1278_ReadData( REG_LR_OPMODE, &SX1278LR->RegOpMode );
    
    return SX1278LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

void sx1278Lora_SetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1278LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1278LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1278LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    sx1278_WriteBuf( REG_LR_FRFMSB, &SX1278LR->RegFrfMsb, 3 );
}

void sx1278Lora_SetSpreadingFactor( uint8_t factor )
{
    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    sx1278_ReadData( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2 );    
    SX1278LR->RegModemConfig2 = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    sx1278_WriteData( REG_LR_MODEMCONFIG2, SX1278LR->RegModemConfig2 );  
    LoRaSettings.SpreadingFactor = factor;
}

void sx1278Lora_SetErrorCoding( uint8_t value )
{
    sx1278_ReadData( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    SX1278LR->RegModemConfig1 = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    sx1278_WriteData( REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}

void  sx1278Lora_SetPacketCrcOn( bool enable )
{
    sx1278_ReadData( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2 );
    SX1278LR->RegModemConfig2 = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    sx1278_WriteData( REG_LR_MODEMCONFIG2, SX1278LR->RegModemConfig2 );
    LoRaSettings.CrcOn = enable;
}

void sx1278Lora_SetSignalBandwidth( uint8_t bw )
{
    sx1278_ReadData( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    SX1278LR->RegModemConfig1 = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    sx1278_WriteData( REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

void sx1278Lora_SetImplicitHeaderOn( bool enable )
{
    sx1278_ReadData( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    SX1278LR->RegModemConfig1 = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    sx1278_WriteData( REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}

void sx1278Lora_SetSymbTimeout( uint16_t value )
{
    sx1278_ReadBuf( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2, 2 );

    SX1278LR->RegModemConfig2 = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1278LR->RegSymbTimeoutLsb = value & 0xFF;
    sx1278_WriteBuf( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2, 2 );
}

void sx1278Lora_SetPayloadLength( uint8_t value )
{
    SX1278LR->RegPayloadLength = value;
    sx1278_WriteData( REG_LR_PAYLOADLENGTH, SX1278LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}

void sx1278Lora_SetLowDatarateOptimize( bool enable )
{
    sx1278_ReadData( REG_LR_MODEMCONFIG3, &SX1278LR->RegModemConfig3 );
    SX1278LR->RegModemConfig3 = ( SX1278LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    sx1278_WriteData( REG_LR_MODEMCONFIG3, SX1278LR->RegModemConfig3 );
}

void sx1278Lora_SetPAOutput( uint8_t outputPin )
{
    sx1278_ReadData( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );
    SX1278LR->RegPaConfig = (SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    sx1278_WriteData( REG_LR_PACONFIG, SX1278LR->RegPaConfig );
}

void sx1278Lora_SetPa20dBm( bool enale )
{
    sx1278_ReadData( REG_LR_PADAC, &SX1278LR->RegPaDac );
    sx1278_ReadData( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );

    if( ( SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == TRUE )
        {
            SX1278LR->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1278LR->RegPaDac = 0x84;
    }
    sx1278_WriteData( REG_LR_PADAC, SX1278LR->RegPaDac );
}

void sx1278Lora_SetRFPower( int8_t power )
{
    sx1278_ReadData( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );
    sx1278_ReadData( REG_LR_PADAC, &SX1278LR->RegPaDac );
    
    if( ( SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278LR->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
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
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    sx1278_WriteData( REG_LR_PACONFIG, SX1278LR->RegPaConfig );
    LoRaSettings.Power = power;
}

bool sx1278Lora_RFSendBuf( uint8_t *txBuf, size_t txLen)
{
    if(txLen > RF_PACKET_LEN)
		return FALSE;	
		
    // Initializes the payload size
    SX1278LR->RegPayloadLength = txLen;
    sx1278_WriteData( REG_LR_PAYLOADLENGTH, SX1278LR->RegPayloadLength);
	
	// Write payload buffer to LORA modem
    sx1278_WriteBuf(0, txBuf, SX1278LR->RegPayloadLength);
   
    sx1278Lora_SetOpMode( RFLR_OPMODE_TRANSMITTER  );
			  
	sx1278_ReadData(REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags);
			  
    RFLRState = RFLR_STATE_TX_RUNNING;
	
	return TRUE;
}

tRFLRStates  sx1278Lora_Process(void)
{
    tRFLRStates result;
	
	switch(RFLRState)
	{
		case RFLR_STATE_IDLE:
			break;
			
		case RFLR_STATE_TX_INIT:
			sx1278Lora_EntryTx();
			RFLRState = RFLR_STATE_TX_RUNNING;
			result =  RFLR_STATE_TX_RUNNING;
		    break;
			
	  	case RFLR_STATE_TX_RUNNING:
		 	// TxDone
//			if(Read_sx1278Dio0_Pin())
//			{			
				sx1278_ReadData(REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags);
									
				if((SX1278LR->RegIrqFlags & RFLR_IRQFLAGS_TXDONE) == RFLR_IRQFLAGS_TXDONE)
				{ 	
					// Clear Irq
					sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
					sx1278Lora_SetOpMode( RFLR_OPMODE_STANDBY );
					RFLRState = RFLR_STATE_IDLE;
					result =  RFLR_STATE_TX_DONE;
				}
//			}
			else
				result = RFLR_STATE_TX_RUNNING; 
			break;
			
		case RFLR_STATE_RX_INIT:
			sx1278Lora_EntryRx();
			RFLRState = RFLR_STATE_RX_RUNNING;
			break;
		
		case RFLR_STATE_RX_RUNNING:
			// RxDone
//			if( Read_sx1278Dio0_Pin())
//			{				
				sx1278_ReadData(REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags);
						
				if((SX1278LR->RegIrqFlags & RFLR_IRQFLAGS_RXDONE) == RFLR_IRQFLAGS_RXDONE)
				{			
					// Clear Irq
					sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
					RFLRState = RFLR_STATE_RX_DONE;
				}
//			}
			else
				result = RFLR_STATE_RX_RUNNING;
			break;
			
		case RFLR_STATE_RX_DONE:
			sx1278_ReadData( REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags );
			if( ( SX1278LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
			{
				// Clear Irq
				sx1278_WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
				if( LoRaSettings.RxSingleOn == true ) // Rx single mode
				{
					RFLRState = RFLR_STATE_RX_INIT;
				}
				else
				{
					RFLRState = RFLR_STATE_RX_RUNNING;
				}
				break;
			}
        		
//			{
//				sx1278_1ReadData( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
//				if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
//				{
//					// Invert and divide by 4
//					RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
//					RxPacketSnrEstimate = -RxPacketSnrEstimate;
//				}
//				else
//				{
//					// Divide by 4
//					RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
//				}
//			}
        
//        	if( LoRaSettings.RFFrequency < 860000000 )  // LF
//        	{    
//            	if( RxPacketSnrEstimate < 0 )
//            	{
//                	RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
//            	}
//            	else
//            	{    
//                	sx1278_1ReadData( REG_LR_PKTRSSIVALUE, &SX1278LR_1->RegPktRssiValue );
//                	RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1278LR_1->RegPktRssiValue;
//            	}
//        	}
//        	else                                        // HF
//        	{    
//            	if( RxPacketSnrEstimate < 0 )
//            	{
//                	RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_HF + ( double )RxPacketSnrEstimate;
//           	 }
//            	else
//            	{    
//                	sx1278_1ReadData( REG_LR_PKTRSSIVALUE, &SX1278LR_1->RegPktRssiValue );
//               	 RxPacketRssiValue = RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1278LR_1->RegPktRssiValue;
//            	}
//        	}

       	  	if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        	{
            	SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxBaseAddr;
            	sx1278_WriteData( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );

           		if( LoRaSettings.ImplicitHeaderOn == true )
            	{
                	RxPacketSize = SX1278LR->RegPayloadLength;
               	 	sx1278_ReadBuf(0, RFBuffer, RxPacketSize );
           		}
           		else
           		{
                	sx1278_ReadData( REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes );
                	RxPacketSize = SX1278LR->RegNbRxBytes;
               	 	sx1278_ReadBuf(0, RFBuffer, RxPacketSize );
            	}
      	  	}
			else // Rx continuous mode
			{
            	sx1278_ReadData( REG_LR_FIFORXCURRENTADDR, &SX1278LR->RegFifoRxCurrentAddr );

            	if( LoRaSettings.ImplicitHeaderOn == true )
            	{
                	RxPacketSize = SX1278LR->RegPayloadLength;
                	SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
               	 	sx1278_WriteData( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
                	sx1278_ReadBuf(0, RFBuffer, RxPacketSize );
            	}
            	else
            	{
                	sx1278_ReadData( REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes );
                	RxPacketSize = SX1278LR->RegNbRxBytes;
                	SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
                	sx1278_WriteData( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
                	sx1278_ReadBuf(0, RFBuffer, RxPacketSize );
            	}
        	}
        
        	if( LoRaSettings.RxSingleOn == true ) // Rx single mode
			{
            	RFLRState = RFLR_STATE_RX_INIT;
        	}
       	 	else // Rx continuous mode
        	{
            	RFLRState = RFLR_STATE_RX_RUNNING;
        	}
        	result = RFLR_STATE_RX_DONE;			
			break;
		
		default:
		  	break;		  		  			  
	}
	
	return  result;
}

void sx1278Lora_SetRFStatus(tRFLRStates st)
{
	RFLRState = st;
}

uint8_t sx1278Lora_GetRFStatus(void)
{
	return RFLRState;
}


void *sx1278Lora_GetRxData(uint8_t *size)
{
	*size = RxPacketSize;
	RxPacketSize = 0;
	
	return (void *)RFBuffer;
}

bool sx1278_GetLoRaOn(void)
{
    return LoRaOn;
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
	 
	if(size > RF_PACKET_LEN || size <= 0)
		return;
	
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