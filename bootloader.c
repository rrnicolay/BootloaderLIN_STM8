#include "definitions.h"
#include "bootloader.h"
#include "flash.h"
#include "pins.h"
#include "lin.h"
#include "crc.h"

// Max size of an app, in bytes
#define MAX_APP_SZ 6272 // + MAX_BOOT_SZ need to be equal to FLASH_SZ
// Max size of bootloader, in bytes
#define MAX_BOOT_SZ 1920 // + MAX_APP_SZ need to be equal to FLASH_SZ
// Max number of blocks of a valid app
#define MAX_APP_BLOCKS (MAX_APP_SZ / BLOCK_SIZE)
// Min number of blocks of a valid app
#define MIN_APP_BLOCKS 6


// Stores received UART data
static uint8_t rxBuffer[BLOCK_SIZE];
// Global error code
static uint8_t errCode = (ErrorCode)OK_ERR;



// Send status to master
static void BOOT_SendStatus();

// Send data stored in flash to master
static void BOOT_SendInfo(uint8_t pid);

// Start update process
static void BOOT_StartUpdate(uint8_t nOfBlocks, uint8_t rxCrc);

// Jump to application
static void BOOT_StartApp();


// Wait for commands
void BOOT_Run()
{
  uint8_t nBlocks = 0;
  uint8_t rxCrc = 0;
  uint8_t chksum = 0;
  uint8_t calcChksum = 0;
  
    // If is in app mode
  if(FLASH_Read((FlashRegCode)BOOT_STATUS_CODE) == 0x00)
  {
    // If has an app programmed
    if(FLASH_Read((FlashRegCode)FW_VERSION_CODE) != 0x00)
    {
      BOOT_StartApp();
    }
  }
  
  // Turn on the LED (indicates bootloader mode)
  PINS_SetLedPin(1);

  // enter bootloader
  while(1)
  {
    uint8_t pid = LIN_ReadPid();
    if(pid == 0)
    {
      errCode = (ErrorCode)HEADER_ERR;
      continue;
    }
    
    if((pid & ID_MASK) == BRLIN_BOOT_ID_RX)
    {
      BOOT_SendInfo(pid);
      continue;
    }
    
    // Read opcode
    uint8_t rx = LIN_Read();
    
    switch(rx)
    {
    // Enter update mode
    case 'U':
      rxBuffer[0] = rx;
      
      nBlocks = LIN_Read();
      if(nBlocks > MAX_APP_BLOCKS || nBlocks < MIN_APP_BLOCKS)
      {
        errCode = (ErrorCode)SIZE_ERR;
        break;
      }
      rxBuffer[1] = nBlocks;
      
      rxCrc = LIN_Read();
      rxBuffer[2] = rxCrc;
      
      rx = LIN_Read();
      rxBuffer[3] = rx;
      
      chksum = LIN_Read();
      calcChksum = LIN_CalcChecksum(&rxBuffer[0], 4, pid);
      
      if(chksum == calcChksum)
      {
        // Turn off the LED
        PINS_SetLedPin(0);
        
        // Enable blink mode
        LIN_EnableLedBlink(1);
        
        // Reset error
        errCode = (ErrorCode)OK_ERR;
        
        BOOT_StartUpdate(nBlocks, rxCrc);
      }
      else
      {
        errCode = (ErrorCode)CHECKSUM_ERR;
      }
      break;
    
    // Go to app
    case 'G':
      rxBuffer[0] = rx;
      
      rx = LIN_Read();
      rxBuffer[1] = rx;
      
      chksum = LIN_Read();
      calcChksum = LIN_CalcChecksum(&rxBuffer[0], 2, pid);
      
      if(chksum == calcChksum)
      {
        BOOT_StartApp();
      }
      else
      {
        errCode = (ErrorCode)CHECKSUM_ERR;
      }
      break;
      
    default:
      errCode = (ErrorCode)COMMAND_ERR;
      break;
    }
  }
}


// Receive firmware via UART and saves on flash
void BOOT_StartUpdate(uint8_t nOfBlocks, uint8_t rxCrc)
{
  uint16_t addr = APP_ADDR;

  // Write received CRC and app's number of blocks to flash
  FLASH_Write((FlashRegCode)NOFBLOCKS_CODE, nOfBlocks);
  FLASH_Write((FlashRegCode)CRC_CODE, rxCrc);
  
  // Unlock flash
  FLASH_UnlockProg();

  // get main firmware
  for(uint8_t i = 0; i < nOfBlocks; i++)
  {
    // Wait status request
    BOOT_SendStatus();
    
    // Read a block
    errCode = LIN_ReadBlock(rxBuffer);
    
    if(errCode == (ErrorCode)OK_ERR)
    {
      // Write block to flash
      FLASH_ProgBlock(addr, rxBuffer);
    }
    else
    {
      // Disable blink mode
      LIN_EnableLedBlink(0);
    }
    
    addr += BLOCK_SIZE;
  }
  
  // Lock flash
  FLASH_LockProg();
  
  // Disable blink mode
  LIN_EnableLedBlink(0);
  
  if(errCode != (ErrorCode)OK_ERR || CRC8_IsFlashedAppCrcOk() == 0)
  {
    errCode = (ErrorCode)CRC_ERR;
  }
  else
  {
    // Turn on the LED
    PINS_SetLedPin(1);
  }
  
  BOOT_SendStatus();
}


// Waits and reply for a status request
void BOOT_SendStatus()
{
  uint8_t toSend[3];
  uint8_t idx = 0;
  
  while(1)
  {
    uint8_t pid = LIN_ReadPid();
    
    if(pid == 0 || ((pid & ID_MASK) != BRLIN_BOOT_ID_RX))
    {
      errCode = (ErrorCode)HEADER_ERR;
      continue;
    }
    
    toSend[idx++] = errCode;
    toSend[idx++] = 0x00;
    
    // Put checksum in last byte of array
    toSend[idx] = LIN_CalcChecksum(&toSend[0], idx, pid);
    idx++;
    
    // Send data
    LIN_Write(toSend, idx);
    
    break;
  }
  
  errCode = (ErrorCode)OK_ERR;
}


// Sends back some info to master
void BOOT_SendInfo(uint8_t pid)
{
  uint8_t info[9];
  uint8_t idx = 0;
  
  info[idx++] = errCode;
  info[idx++] = FLASH_Read((FlashRegCode)FW_VERSION_CODE);
  info[idx++] = FLASH_Read((FlashRegCode)BOOT_VERSION_CODE);
  info[idx++] = 0x00; // RFFU

  // Put checksum on last byte
  info[idx] = LIN_CalcChecksum(&info[0], idx, pid);
  idx++;
  
  // Send data
  LIN_Write(info, idx);
    
  // Clear error
  errCode = (ErrorCode)OK_ERR;
}


// Jump to application
void BOOT_StartApp()
{
  // If CRC matches, jump!
  if(CRC8_IsFlashedAppCrcOk())
  {
    // The address need to be the value of APP_ADDR
    asm("JP 0x8780");
  }
}
