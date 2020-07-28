#include "iostm8s003f3.h"
#include "pins.h"

#define LED_PIN (PA_ODR_ODR3)


// Configure GPIO
void PINS_Config()
{
  // Output led pin
  PA_ODR_ODR3 = 0; // Default off
  PA_DDR_DDR3 = 1; // Output
  PA_CR1_C13 = 1; // PP
  PA_CR2_C23 = 0; // Fast
}


// Turn on or off the LED
void PINS_SetLedPin(uint8_t val)
{
  LED_PIN = val;
}
