#include <intrinsics.h>
#include "iostm8s003f3.h"
#include "lin.h"
#include "flash.h"
#include "bootloader.h"
#include "pins.h"

// Configure clock
static void CLK_Config();


void main(void)
{
  // Disable interruptions
  __disable_interrupt();
  
  // Peripheral init
  CLK_Config();
  PINS_Config();
  LIN_Config();
  FLASH_Init();
  
  // Run!
  BOOT_Run();
}


// Configure clock
void CLK_Config()
{
  // Clock divider set to 0 (16MHz)
  CLK_CKDIVR = 0x00;
}
