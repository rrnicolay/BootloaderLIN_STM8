#ifndef _FLASH_H_
#define _FLASH_H_

#include "definitions.h"

// Code to access EEPROM
typedef enum
{
  ID_CODE,
  FW_VERSION_CODE,
  BOOT_STATUS_CODE,
  BOOT_VERSION_CODE,
  NOFBLOCKS_CODE,
  CRC_CODE,
  REGS_NUM, // Need to be the last one! Insert before
} FlashRegCode;

// Initialize registers
void FLASH_Init();

// Read parameter reg ant return its value
uint8_t FLASH_Read(FlashRegCode code);

// Write 'value' on reg
void FLASH_Write(FlashRegCode code, uint8_t value);

// Enable write to PROG section
void FLASH_UnlockProg();

// Disable write to PROG section
void FLASH_LockProg();

#define IN_RAM(a) __ramfunc a
// Writes an entire block to flash
IN_RAM(void FLASH_ProgBlock(uint16_t StartAddress, uint8_t *Buffer));

#endif
