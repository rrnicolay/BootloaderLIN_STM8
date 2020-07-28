#include "crc.h"
#include "flash.h"
#include "definitions.h"

// Updates CRC value
uint8_t CRC8_Update(uint8_t data, uint8_t crc);

// Computes CRC of flashed app
uint8_t CRC8_GetCrc();



// Computes CRC of flashed app
uint8_t CRC8_GetCrc()
{
  uint8_t calcCrc = 0;
  uint8_t appNofBlocks = FLASH_Read((FlashRegCode) NOFBLOCKS_CODE);
  
  // Reads firmware from flash and compute CRC
  for (uint16_t i = APP_ADDR; i < (BLOCK_SIZE*appNofBlocks) + APP_ADDR; i++)
  {
    calcCrc = CRC8_Update(*((PointerAttr uint8_t*) i), calcCrc);
  }
  
  return calcCrc;
}


// Computes CRC of flashed app
uint8_t CRC8_IsFlashedAppCrcOk()
{
  uint8_t appNofBlocks = FLASH_Read((FlashRegCode) NOFBLOCKS_CODE);
  
  // If no app was flashed... return 0
  if(appNofBlocks == 0)
  {
    return 0;
  }
  
  uint8_t calcCrc = CRC8_GetCrc();
  uint8_t appCrc = FLASH_Read((FlashRegCode) CRC_CODE);
    
  // return 1 if they match
  return (calcCrc == appCrc);
}


/* Calculate CRC-8-CCIT.
 * Polynomial: x^8 + x^2 + x + 1 (0x07)
 * @param data input byte
 * @param crc initial CRC
 * @return CRC value */
uint8_t CRC8_Update(uint8_t data, uint8_t crc)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
  {
    crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
  }
  return crc;
}
