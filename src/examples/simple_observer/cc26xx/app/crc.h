
#ifndef __CRC_H__
#define __CRC_H__

#include <xdc/std.h>

/********************************************
* typedef *
********************************************/



/********************************************
* Defines *
********************************************/



/********************************************
* Globals * 
********************************************/ 



/********************************************
* Functions * 
********************************************/
uint8_t crc8(uint8_t crc, uint8_t *buf, uint32_t len);

uint16_t crc16(uint16_t crc, uint8_t *buf, uint32_t len);

uint32_t crc32( uint32_t crc, uint8_t *buf, uint32_t len);

uint8_t checksum_8(uint8_t cksum, uint8_t *buf, uint32_t len);

#endif
