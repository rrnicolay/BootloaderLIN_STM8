#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

/* Bootloader version. Should be incremented manually at each released version */
#define BOOTLOADER_VERSION 0x01

// Number of bytes in a block, 64 bytes in low density STM8S
#define BLOCK_SIZE 64
// Address of app
#define APP_ADDR 0x8780
// 6-bit mask
#define ID_MASK 0x3F
#define BRLIN_BOOT_ID_RX 0x31
#define BRLIN_BOOT_ID_TX 0x30

// This enum has the status and error code to send to master.
// ACK: 0xAX, NACK: 0xFX
typedef enum
{
  OK_ERR        = 0xA0,
  HEADER_ERR    = 0xF1,
  SIZE_ERR      = 0xF2,
  CHECKSUM_ERR  = 0xF3,
  CRC_ERR       = 0xF4,
  COMMAND_ERR   = 0xF5,
} ErrorCode;

// IO definitions: define access restrictions to peripheral registers
#define __I volatile const // defines 'read only' permissions
#define __O volatile       // defines 'write only' permissions
#define __IO volatile      // defines 'read / write' permissions

// Signed integer types
typedef signed char int8_t;
typedef signed short int16_t;
typedef signed long int32_t;

// Unsigned integer types
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;

// Macros needed to write from RAM
#define FAR
#define NEAR
#define TINY
#define __CONST const
#define PointerAttr NEAR

#endif
