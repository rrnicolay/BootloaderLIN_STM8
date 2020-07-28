#include "iostm8s003f3.h"
#include "leds.h"

// Inform if a packet has been received in UART
void LEDS_OnOff(uint8_t isOn)
{
  PINS_SetLedPin(isOn);
}


