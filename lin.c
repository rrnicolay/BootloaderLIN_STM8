#include "iostm8s003f3.h"
#include "lin.h"
#include "definitions.h"
#include "pins.h"

#define LIN_FRAME_DATA_SZ 8 // bytes
#define ID_FIELD_SZ 6 // bits

#define SYNC_VAL 0x55

static uint8_t blinkEn = 0;


// Calculate LIN parity
static uint8_t LIN_CalcParity(uint8_t id);


// Enable blink on frame reception
void LIN_EnableLedBlink(uint8_t en)
{
  blinkEn = en;
}


/* Will block until a break is received.
 * Also, reads sync and parity
 * Returns:
 * - PID on success
 * - 0 on error. Error causes:
 *   - Didnt receive correct SYNC byte
 *   - Wrong ID
 *   - Parity doesnt match */
uint8_t LIN_ReadPid()
{
  // Wait for break sync
  while(UART1_CR4_LBDF == 0) ;
  
  if(blinkEn)
  {
    // Turn on reception LED
    PINS_SetLedPin(1);
  }
  
  // Dummy read just to discard UART1_DR received on a break
  uint8_t mRx = LIN_Read();
  
  // Clear break detected flag
  UART1_CR4_LBDF = 0;
  
  // Read Sync
  mRx = LIN_Read();
  
  if(blinkEn)
  {
    // Turn off reception LED
    PINS_SetLedPin(0);
  }
  
  if(mRx != SYNC_VAL)
  {
    return 0;
  }
  
  // Read PID
  uint8_t pid = LIN_Read();
  // Extract ID
  uint8_t id = pid & ID_MASK;
  
  // Check ID
  if(id != BRLIN_BOOT_ID_TX && id != BRLIN_BOOT_ID_RX)
  {
    return 0;
  }

  uint8_t calcParity = LIN_CalcParity(id);
  uint8_t parity = pid >> ID_FIELD_SZ;
  
  // Check parity
  if(parity != calcParity)
  {
    return 0;
  }
  
  return pid;
}


// Checksum Calc
uint8_t LIN_CalcChecksum(uint8_t* frame, uint8_t sz, uint8_t pid)
{
	uint16_t chkSum = pid;
  
	for(uint8_t i=0; i<sz; i++)
	{
		chkSum += frame[i];
		if(chkSum > 255)
		{
			chkSum -= 255;
		}
	}
	chkSum = ~chkSum & 0xFF;
	return chkSum;
}


// Read BLOCK_SIZE bytes from UART
// @param dest destination buffer
ErrorCode LIN_ReadBlock(uint8_t *dest)
{
  // Index of received byte in block
  uint8_t index = 0;
  // returning error code
  ErrorCode errCode = (ErrorCode)OK_ERR;
  
  // Receive 64 bytes (8 LIN frames). i is the frame index
  for (uint8_t i = 0; i < BLOCK_SIZE/LIN_FRAME_DATA_SZ; i++)
  {
    uint8_t pid = LIN_ReadPid();
    
    if(pid == 0 || ((pid & ID_MASK) != BRLIN_BOOT_ID_TX))
    {
      errCode = (ErrorCode)HEADER_ERR;
      continue;
    }
    
    // Receive data. j is the data index of current frame
    for(uint8_t j = 0; j < LIN_FRAME_DATA_SZ; j++)
    {
      uint8_t rx = LIN_Read();
      dest[index++] = rx;
    }
    
    uint8_t chksum = LIN_Read();
    uint8_t calcChksum = LIN_CalcChecksum(&dest[index-LIN_FRAME_DATA_SZ], LIN_FRAME_DATA_SZ, pid);
    
    if(chksum != calcChksum)
    {
      errCode = (ErrorCode)CHECKSUM_ERR;
    }
  }
  
  return errCode;
}


/* Calc. Parity
 P0 = ID0 ? ID1 ? ID2 ? ID4
 P1 = ! (ID1 ? ID3 ? ID4 ? ID5) */
uint8_t LIN_CalcParity(uint8_t id)
{
	uint8_t p0 = (id & 0x01) ^ ((id >> 1) & 0x01) ^ ((id >> 2) & 0x01);
	p0 ^= ((id >> 4) & 0x01);
	uint8_t p1 = ((id >> 1) & 0x01) ^ ((id >> 3) & 0x01) ^ ((id >> 4) & 0x01);
	p1 ^= ((id >> 5) & 0x01);
	p1 = ~p1;
	return (((p1 & 0x01) << 1) | p0);
}


// Write an array of data
void LIN_Write(uint8_t* data, uint8_t size)
{
  for(uint8_t i=0; i<size; i++)
  {
    // is blocking!
    while (UART1_SR_TXE == 0);
    UART1_DR = data[i];
  }
}


// Read byte from UART
uint8_t LIN_Read()
{
  while(UART1_SR_RXNE == 0);
  uint8_t byte = UART1_DR;
  return byte;
}


// Config UART in LIN mode 19200bps
void LIN_Config()
{
  UART1_CR1_M = 0; // 8 Data bits
  UART1_CR1_PCEN = 0; // Disable parity
  UART1_CR3_STOP = 0; // one stop bit
  UART1_BRR2 = 0x01; // Set the baud rate registers to 19200 baud
  UART1_BRR1 = 0x34; // based upon a 16 MHz system clock

  UART1_CR3_CPOL = 0; // clock polarity
  UART1_CR3_CPHA = 0; // lock phase
  UART1_CR3_LBCL = 0; // last bit clock pulse

  UART1_CR2_TIEN = 0; // Transmitter interrupt disable
  UART1_CR2_TCIEN = 0; // Transmission complete interrupt disable
  UART1_CR2_RIEN = 0; // Receiver interrupt enable
  UART1_CR2_ILIEN = 0; // IDLE Line interrupt disable
  
  UART1_CR3_LINEN = 0; // 1: Enable LIN
  UART1_CR4_LBDIEN = 0; // 1: Enable LIN Break Detection Interruption
  UART1_CR3_CKEN = 0;
  UART1_CR5_SCEN = 0;
  UART1_CR5_HDSEL = 0;
  UART1_CR5_IREN = 0;

  UART1_CR2_TEN = 1; // Turn on the UART transmit
  UART1_CR2_REN = 1; // Turn on the UART receive
}
