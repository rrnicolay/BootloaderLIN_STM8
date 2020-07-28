#include "iostm8s003f3.h"
#include "flash.h"
#include "definitions.h"

// Size of flash, in bytes
#define FLASH_SZ 8192
// Bootloader address
#define BOOT_ADDR 0x8000

// Data EEPROM memory: start address
#define FLASH_DATA_START_PHYSICAL_ADDRESS ((uint32_t)0x004000)

// Keys to unlock memory
#define FLASH_PUKR_KEY1 0x56
#define FLASH_PUKR_KEY2 0xAE

__IO_REG8 (FLASH_ID, FLASH_DATA_START_PHYSICAL_ADDRESS + ID_CODE, __READ_WRITE);
__IO_REG8 (FLASH_FW_VERSION, FLASH_DATA_START_PHYSICAL_ADDRESS + FW_VERSION_CODE, __READ_WRITE);
__IO_REG8 (FLASH_BOOT_STATUS, FLASH_DATA_START_PHYSICAL_ADDRESS + BOOT_STATUS_CODE, __READ_WRITE);
__IO_REG8 (FLASH_BOOT_VERSION, FLASH_DATA_START_PHYSICAL_ADDRESS + BOOT_VERSION_CODE, __READ_WRITE);
__IO_REG8 (FLASH_NOFBLOCKS, FLASH_DATA_START_PHYSICAL_ADDRESS + NOFBLOCKS_CODE, __READ_WRITE);
__IO_REG8 (FLASH_CRC, FLASH_DATA_START_PHYSICAL_ADDRESS + CRC_CODE, __READ_WRITE);

// Translates a FlashRegCode to the register address
static uint8_t volatile* map[REGS_NUM] =
{
  &FLASH_ID,
  &FLASH_FW_VERSION,
  &FLASH_BOOT_STATUS,
  &FLASH_BOOT_VERSION,
  &FLASH_NOFBLOCKS,
  &FLASH_CRC,
};

// Enable write to DATA section
static void FLASH_UnlockData();

// Disable write to DATA section
static void FLASH_LockData();



// Initialize registers
void FLASH_Init()
{
  // Writes bootloader version in EEPROM
  FLASH_Write((FlashRegCode)BOOT_VERSION_CODE, BOOTLOADER_VERSION);
}


// Unlock PROG memory
void FLASH_UnlockProg(void)
{
  FLASH_PUKR = FLASH_PUKR_KEY1;
  FLASH_PUKR = FLASH_PUKR_KEY2;
  while(FLASH_IAPSR_PUL == 0x00);
}


// Disable write to PROG section
void FLASH_LockProg()
{
  FLASH_IAPSR_PUL = 0;
}


// Unlock DATA memory
void FLASH_UnlockData(void)
{
	FLASH_DUKR = FLASH_PUKR_KEY2;
  FLASH_DUKR = FLASH_PUKR_KEY1;
}


// Disable write to DATA section
void FLASH_LockData()
{
  FLASH_IAPSR_DUL = 0;
}


// Read parameter reg ant return its value
uint8_t FLASH_Read(FlashRegCode code)
{
  uint8_t value = 0;

  value = *(map[code]);

  return value;
}


// Write 'value' on reg
void FLASH_Write(FlashRegCode code, uint8_t value)
{
  FLASH_UnlockData();
  
  if(FLASH_IAPSR_DUL)
  {
    if(*(map[code]) != value)
    {
      *(map[code]) = value;
    }
    
    FLASH_LockData();
  }
}


// Store a block in flash
IN_RAM(void FLASH_ProgBlock(uint16_t startAddr, uint8_t *buffer))
{
  // Standard programming mode
  FLASH_CR2_PRG = 1;
  // Negated PRG
  FLASH_NCR2_NPRG = 0;
  
  // Copy data bytes from RAM to FLASH memory within the block program,
  // all bytes written in a programming sequence must be in the same block
  for (uint8_t i = 0; i < BLOCK_SIZE; i++)
  {
    *((PointerAttr uint8_t*)startAddr + i) = ((uint8_t)(buffer[i]));
  }
  
  while(FLASH_IAPSR_EOP == 0x00);
}
