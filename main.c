/* An important warning: be aware of the resulting size of bootloader and main
application. The size changes based on optimizations settings. The available 
flash is scarce. Remember to use correct linker files, so IAR can enforce flash 
sections of bootloader and main application. */

#include <intrinsics.h>
#include <iostm8s003f3.h>

/*****************************************************************************/
/* Global definitions */

/* Bootloader version. Should be incremented manually at each released version */
#define BOOTLOADER_VERSION 0x01
/* Number of bytes in a block, 64 bytes in low density STM8S */
#define BLOCK_SIZE 64
/* Size of flash, in bytes */
#define FLASH_SZ 8192
/* Max size of an app, in bytes */
#define MAX_APP_SZ 6272 // + MAX_BOOT_SZ need to be equal to FLASH_SZ
/* Max size of bootloader, in bytes */
#define MAX_BOOT_SZ 1920 // + MAX_APP_SZ need to be equal to FLASH_SZ
/* Bootloader address */
#define BOOT_ADDR 0x8000
/* Address of app */
#define APP_ADDR 0x8780
/* Max number of blocks of a valid app */
#define MAX_APP_BLOCKS (MAX_APP_SZ / BLOCK_SIZE)
/* Min number of blocks of a valid app */
#define MIN_APP_BLOCKS 6

/* Code to access EEPROM */
typedef enum
{
  FW_VERSION_CODE,
  BOOT_STATUS_CODE,
  BOOT_VERSION_CODE,
  NOFBLOCKS_CODE,
  CRC_CODE,
} FlashRegCode;

/* This enum has the status and error code to send to master.
 ACK: 0xAX, NACK: 0xFX */
typedef enum
{
  OK_ERR        = 0xA0,
  HEADER_ERR    = 0xF1,
  SIZE_ERR      = 0xF2,
  CHECKSUM_ERR  = 0xF3,
  CRC_ERR       = 0xF4,
  COMMAND_ERR   = 0xF5,
} ErrorCode;

/* Data EEPROM memory: start address */
#define FLASH_DATA_START_PHYSICAL_ADDRESS ((uint32_t)0x004000)

/* EEPROM regs */
__IO_REG8 (FLASH_FW_VERSION, FLASH_DATA_START_PHYSICAL_ADDRESS + FW_VERSION_CODE, __READ);
__IO_REG8 (FLASH_BOOT_STATUS, FLASH_DATA_START_PHYSICAL_ADDRESS + BOOT_STATUS_CODE, __READ_WRITE);
__IO_REG8 (FLASH_BOOT_VERSION, FLASH_DATA_START_PHYSICAL_ADDRESS + BOOT_VERSION_CODE, __READ_WRITE);
__IO_REG8 (FLASH_NOFBLOCKS, FLASH_DATA_START_PHYSICAL_ADDRESS + NOFBLOCKS_CODE, __READ_WRITE);
__IO_REG8 (FLASH_CRC, FLASH_DATA_START_PHYSICAL_ADDRESS + CRC_CODE, __READ_WRITE);

#define LIN_FRAME_DATA_SZ 8 // bytes
#define ID_FIELD_SZ 6 // bits
#define ID_MASK 0x3F

#define SYNC_VAL 0x55

#define BOOT_ID_RX 0x31
#define BOOT_ID_TX 0x30

#define FLASH_PUKR_KEY1 0x56
#define FLASH_PUKR_KEY2 0xAE

#define IN_RAM(a) __ramfunc a
#define FAR
#define NEAR
#define TINY
#define __CONST const
#define PointerAttr NEAR


/*****************************************************************************/
/* Global vars */

/* Stores received UART data */
static uint8_t rxBuffer[BLOCK_SIZE];
/* Global error code */
static uint8_t errCode = (ErrorCode)OK_ERR;


/*****************************************************************************/
/* Function prototypes */

static void UART_Config();

static void Start_App();

static uint8_t CalcChecksum(uint8_t* frame, uint8_t sz, uint8_t pid);

static uint8_t UART_Read();

static void UART_Write(uint8_t data);

static uint8_t CalcParity(uint8_t id);

static uint8_t CRC8_Update(uint8_t data, uint8_t crc);

static void UART_ReadBlock(uint8_t *dest);

static void sendStatus();

static uint8_t readPID();

IN_RAM(void FLASH_ProgBlock(uint16_t StartAddress, uint8_t *Buffer));

static void unlock_PROG(void);

static void sendInfo(uint8_t pid);

static void startUpdate(uint8_t nOfBlocks, uint8_t rxCrc);

static void unlock_DATA(void);

static void setError(uint8_t err);

static uint8_t ComputeCRC();


/*****************************************************************************/

/* Will block until a break is received.
 * Also, reads sync and parity
 * Returns:
 * - PID on success
 * - 0 on error. Error causes:
 *   - Didnt receive correct SYNC byte
 *   - Wrong ID
 *   - Parity doesnt match
 * */
uint8_t readPID()
{
  /* Wait for break sync */
  while(UART1_CR4_LBDF == 0) ;
  
  /* Turn on reception LED */
  //LEDS_SetRx(1);
  
  /* Dummy read just to discard UART1_DR received on a break */
  uint8_t mRx = UART_Read();
  
  /* Clear break detected flag */
  UART1_CR4_LBDF = 0;
  
  /* Read Sync */
  mRx = UART_Read();
  
  /* Turn off reception LED */
  //LEDS_SetRx(0);
  
  if(mRx != SYNC_VAL)
  {
    return 0;
  }
  
  /* Read PID */
  uint8_t pid = UART_Read();
  /* Mask parity and lower bit */
  uint8_t id = pid & ID_MASK;
  
  /* Check ID */
  if(id != BOOT_ID_TX && id != BOOT_ID_RX)
  {
    return 0;
  }

  uint8_t calcParity = CalcParity(id);
  uint8_t parity = pid >> ID_FIELD_SZ;
  
  /* Check parity */
  if(parity != calcParity)
  {
    return 0;
  }
  
  return pid;
}

/* Wait for commands */
void BOOT_Exec()
{
  uint8_t nBlocks = 0;
  uint8_t rxCrc = 0;
  uint8_t chksum = 0;
  uint8_t calcChksum = 0;
  
  /* Turn off reception LED */
  //LEDS_SetRx(0);

  /* enter bootloader */
  while(1)
  {
    uint8_t pid = readPID();
    if(pid == 0)
    {
      setError((ErrorCode)HEADER_ERR);
      continue;
    }
    
    if((pid & ID_MASK) == BOOT_ID_RX)
    {
      sendInfo(pid);
      continue;
    }
    
    /* Read opcode */
    uint8_t rx = UART_Read();
    
    switch(rx)
    {
    /* Enter update mode */
    case 'U':
      rxBuffer[0] = rx;
      
      nBlocks = UART_Read();
      if(nBlocks > MAX_APP_BLOCKS || nBlocks < MIN_APP_BLOCKS)
      {
        setError((ErrorCode)SIZE_ERR);
        break;
      }
      rxBuffer[1] = nBlocks;
      
      rxCrc = UART_Read();
      rxBuffer[2] = rxCrc;
      
      rx = UART_Read();
      rxBuffer[3] = rx;
      
      chksum = UART_Read();
      calcChksum = CalcChecksum(&rxBuffer[0], 4, pid);
      
      if(chksum == calcChksum)
      {
        setError((ErrorCode)OK_ERR);
        startUpdate(nBlocks, rxCrc);
      }
      else
      {
        setError((ErrorCode)CHECKSUM_ERR);
      }
      break;
    
    /* Go to app */
    case 'G':
      rxBuffer[0] = rx;
      
      rx = UART_Read();
      rxBuffer[1] = rx;
      
      chksum = UART_Read();
      calcChksum = CalcChecksum(&rxBuffer[0], 2, pid);
      
      if(chksum == calcChksum)
      {
        Start_App();
      }
      else
      {
        setError((ErrorCode)CHECKSUM_ERR);
      }
      break;
      
    default:
      setError((ErrorCode)COMMAND_ERR);
      break;
    }
  }
}


/* Set an error code and turn on the error LED */
void setError(uint8_t err)
{
  errCode = err;
}


/* Receive firmware via UART and saves on flash */
void startUpdate(uint8_t nOfBlocks, uint8_t rxCrc)
{
  uint16_t addr = APP_ADDR;
  
  /* Unlock DATA to write CRC and number of blocks */
  unlock_DATA();
  
  /* If data write is allowed... */
  if(FLASH_IAPSR_DUL)
  {
    /* Writes number of blocks */
    FLASH_NOFBLOCKS = nOfBlocks;
    /* Writes received CRC */
    FLASH_CRC = rxCrc;
    /* Lock data */
    FLASH_IAPSR_DUL = 0;
  }
  else
  {
    return;
  }
  
  /* Unlock flash */
  unlock_PROG();

  /* get main firmware */
  for(uint8_t i = 0; i < nOfBlocks; i++)
  {
    /* Wait status request */
    sendStatus();
    
    /* Read a block */
    UART_ReadBlock(rxBuffer);
    
    /* Write block to flash */
    if(errCode == (ErrorCode)OK_ERR)
    {
      FLASH_ProgBlock(addr, rxBuffer);
    }
    
    addr += BLOCK_SIZE;
  }
  
  /* Lock flash */
  FLASH_IAPSR_PUL = 0x00;
  
  // Computes CRC of received firmware
  uint8_t isCrcOk = ComputeCRC();
  
  if(errCode != (ErrorCode)OK_ERR || isCrcOk != 0)
  {
    setError((ErrorCode)CRC_ERR);
    //LEDS_BootFail();
  }
  
  sendStatus();
}


/* Will read stored firmware and return 1 if CRC doesnt match
with the one stored in EEPROM */
uint8_t ComputeCRC()
{
  uint8_t crc = 0;
  
  /* Reads firmware from flash and compute CRC */
  for (uint16_t i = APP_ADDR; i < (BLOCK_SIZE*FLASH_NOFBLOCKS) + APP_ADDR; i++)
  {
    crc = CRC8_Update(*((PointerAttr uint8_t*) i), crc);
  }
  
  return (crc != FLASH_CRC);
}


/* Read BLOCK_SIZE bytes from UART
 * @param dest destination buffer */
void UART_ReadBlock(uint8_t *dest)
{
  /* Index of received byte in block */
  uint8_t index = 0;
  
  /* Receive 64 bytes (8 LIN frames) */
  for (uint8_t i = 0; i < BLOCK_SIZE/LIN_FRAME_DATA_SZ; i++)
  {
    uint8_t pid = readPID();
    if(pid == 0 || ((pid & ID_MASK) != BOOT_ID_TX))
    {
      //LEDS_BootFail();
      setError((ErrorCode)HEADER_ERR);
      continue;
    }
    
    /* Receive data */
    for(uint8_t j = 0; j < LIN_FRAME_DATA_SZ; j++)
    {
      uint8_t rx = UART_Read();
      dest[index++] = rx;
    }
    
    uint8_t chksum = UART_Read();
    uint8_t calcChksum = CalcChecksum(&dest[index-LIN_FRAME_DATA_SZ], LIN_FRAME_DATA_SZ, pid);
    
    if(chksum != calcChksum)
    {
      //LEDS_BootFail();
      setError((ErrorCode)CHECKSUM_ERR);
    }
  }
}


/* Waits and reply for a status request */
void sendStatus()
{
  uint8_t toSend[3];
  uint8_t idx = 0;
  
  while(1)
  {
    uint8_t pid = readPID();
    
    if(pid == 0 || ((pid & ID_MASK) != BOOT_ID_RX))
    {
      setError((ErrorCode)HEADER_ERR);
      continue;
    }
    
    toSend[idx++] = errCode;
    toSend[idx++] = 0x00;
    
    toSend[idx] = CalcChecksum(&toSend[0], idx, pid);
    
    for(uint8_t i=0; i<=idx; i++)
    {
      UART_Write(toSend[i]);
    }
    
    break;
  }
  
  setError((ErrorCode)OK_ERR);
}


/* Sends back some info to master */
void sendInfo(uint8_t pid)
{
  uint8_t info[9];
  uint8_t idx = 0;
  
  info[idx++] = errCode;
  info[idx++] = FLASH_FW_VERSION;
  info[idx++] = FLASH_BOOT_VERSION;
  info[idx++] = 0x00;
 
  info[idx] = CalcChecksum(&info[0], idx, pid);
  
  for(uint8_t i=0; i<=idx; i++)
  {
    UART_Write(info[i]);
  }
    
  /* Clear error */
  setError((ErrorCode)OK_ERR);
}


void main(void)
{
  /* Disable interruptions */
  __disable_interrupt(); // Disable general interrupts
  
  CLK_CKDIVR = 0x00; // Clock divider set to 0 (16MHz)
  
  if(FLASH_BOOT_STATUS == 0x00) // If is in app mode
  {
    if(FLASH_FW_VERSION != 0x00) // If has an app programmed
    {
      Start_App();
    }
  }
  
  /* Writes bootloader version in EEPROM */
  if(FLASH_BOOT_VERSION != BOOTLOADER_VERSION)
  {
    unlock_DATA();
    
    /* If data write is allowed... */
    if(FLASH_IAPSR_DUL)
    {
      /* Write new version */
      FLASH_BOOT_VERSION = BOOTLOADER_VERSION;
      /* Lock data */
      FLASH_IAPSR_DUL = 0;
    }
  }
  
  //GPIO_Config();
  UART_Config();
  
  BOOT_Exec();
}


void Start_App()
{
  // If CRC matches, jump!
  if(ComputeCRC() == 0)
  {
    /* The address need to be the value of APP_ADDR */
    asm("JP 0x8780");
  }
}


/* Config UART in LIN mode 19200bps */
void UART_Config()
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
  
  UART1_CR3_LINEN = 0; // Disable LIN
  UART1_CR4_LBDIEN = 0; // Disable LIN Break Detection Interruption
  UART1_CR3_CKEN = 0;
  UART1_CR5_SCEN = 0;
  UART1_CR5_HDSEL = 0;
  UART1_CR5_IREN = 0;

  UART1_CR2_TEN = 1; // Turn on the UART transmit
  UART1_CR2_REN = 1; // Turn on the UART receive
}


/* Unlock PROG memory */
void unlock_PROG(void)
{
  FLASH_PUKR = 0x56;
  FLASH_PUKR = 0xAE;
  while(FLASH_IAPSR_PUL == 0x00);
}


/* Unlock DATA memory */
void unlock_DATA(void)
{
	FLASH_DUKR = 0xAE; /* Warning: keys are reversed on data memory !!! */
  FLASH_DUKR = 0x56;
}


/* Calc. Parity
 P0 = ID0 ? ID1 ? ID2 ? ID4
 P1 = ! (ID1 ? ID3 ? ID4 ? ID5) */
uint8_t CalcParity(uint8_t id)
{
	uint8_t p0 = (id & 0x01) ^ ((id >> 1) & 0x01) ^ ((id >> 2) & 0x01);
	p0 ^= ((id >> 4) & 0x01);
	uint8_t p1 = ((id >> 1) & 0x01) ^ ((id >> 3) & 0x01) ^ ((id >> 4) & 0x01);
	p1 ^= ((id >> 5) & 0x01);
	p1 = ~p1;
	return (((p1 & 0x01) << 1) | p0);
}


/* Checksum Calc
http://microchip.wdfiles.com/local--files/lin%3Aspecification/LIN-Spec_2.2_Rev_A.PDF
* Called with the reply frame, but the ID is summed in checksum. */
uint8_t CalcChecksum(uint8_t* frame, uint8_t sz, uint8_t pid)
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


/* Write byte into UART */
void UART_Write(uint8_t data)
{
  while (UART1_SR_TXE == 0);
  UART1_DR = data;
}


/* Read byte from UART */
uint8_t UART_Read()
{
  while(UART1_SR_RXNE == 0);
  uint8_t byte = UART1_DR;
  return byte;
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


/* Store a block in flash */
IN_RAM(void FLASH_ProgBlock(uint16_t startAddr, uint8_t *buffer))
{
  /* Standard programming mode */
  FLASH_CR2_PRG = 1;
  /* Negated PRG */
  FLASH_NCR2_NPRG = 0;
  
  /* Copy data bytes from RAM to FLASH memory within the block program,
   * all bytes written in a programming sequence must be in the same block */
  for (uint8_t i = 0; i < BLOCK_SIZE; i++)
  {
    *((PointerAttr uint8_t*)startAddr + i) = ((uint8_t)(buffer[i]));
  }
  
  while(FLASH_IAPSR_EOP == 0x00);
}
