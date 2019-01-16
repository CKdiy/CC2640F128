
#include "crc.h"

/*******************************************************************************
* Function Name   : crc8
* Description     : CRC8计算校验
* Input           : 1、crc初始值;2、待校验缓存;3、待校验缓存长;
* Output          : None
* Return          : CRC8校验值
*******************************************************************************/
uint8_t crc8(uint8_t crc, uint8_t *buf, uint32_t len)
{
    unsigned char i;
    unsigned int j=0;
    for (j = 0; j < len; j++)
    {
        crc ^= *(buf+j);
        for (i = 0; i < 8; i++)
        {
            if ((crc & 0x01) != 0)
            {
                crc >>= 1;
                crc ^= 0x8c;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/*******************************************************************************
* Function Name   : crc16
* Description     : CRC16计算校验
* Input           : 1、crc初始值;2、待校验缓存;3、待校验缓存长;
* Output          : None
* Return          : CRC16校验值
*******************************************************************************/
uint16_t crc16(uint16_t crc, uint8_t *buf, uint32_t len)
{
    unsigned int i,j;
    unsigned short wTemp = 0;           
    
    for(i = 0; i < len; i++)      
    {             
        for(j = 0; j < 8; j++)      
        {      
            wTemp = ((*(buf+i) << j) & 0x80 ) ^ ((crc & 0x8000) >> 8);      
    
            crc <<= 1;      
    
            if(wTemp != 0)       
            {     
                crc ^= 0x1021;      
            }     
        }      
    }      
    
    return crc;
}

/*******************************************************************************
* Function Name   : crc32
* Description     : CRC32计算校验
* Input           : 1、crc初始值;2、待校验缓存;3、待校验缓存长;
* Output          : None
* Return          : CRC32校验值
*******************************************************************************/
uint32_t crc32( uint32_t crc, uint8_t *buf, uint32_t len)
{
    int i;
    
    crc = crc ^ 0xFFFFFFFF;
    
    while(len --)
    {
        crc ^= *buf ++;
        
        for (i = 0; i < 8; i ++)
        {
            if ((crc & 0x00000001) != 0)
            {
                crc >>= 1;
                crc ^= 0xEDB88320L ;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    
    return crc ^ 0xFFFFFFFF;
}

/*******************************************************************************
* Function Name   : checksum_8
* Description     : 8位和校验
* Input           : 1、初始值;2、待校验缓存;3、待校验缓存长;
* Output          : None
* Return          : 8位和校验值
*******************************************************************************/
uint8_t checksum_8(uint8_t cksum, uint8_t *buf, uint32_t len)
{  
    int i;

    for(i = 0;i < len; i ++)
    {
        cksum += *(buf+i);
    }
    return cksum;
}  