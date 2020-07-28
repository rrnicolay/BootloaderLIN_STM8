#ifndef _LIN_H_
#define _LIN_H_

#include "definitions.h"


// Configure piripheral registers
void LIN_Config();

// Calculate LIN checksum
uint8_t LIN_CalcChecksum(uint8_t* frame, uint8_t sz, uint8_t pid);

// Reads a byte
uint8_t LIN_Read();

// Write an array od data
void LIN_Write(uint8_t* data, uint8_t size);

// Read PID of frame
uint8_t LIN_ReadPid();

// Read a block od data
ErrorCode LIN_ReadBlock(uint8_t *dest);

// Enable blink on frame reception
void LIN_EnableLedBlink(uint8_t en);

#endif
