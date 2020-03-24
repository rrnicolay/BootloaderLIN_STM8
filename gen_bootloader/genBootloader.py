import os, math, argparse
import sys
from datetime import date

DEBUG = True

APP_START_ADDR = (0x8780)

# Prints a formatted IVT
def printIVT(ivt):
  cnt = 0
  for i in range(0, len(ivt)):
    print('0x%02X, ' %(ivt[i]), end = '')
    cnt = cnt + 1
    # Prints newline after 16 bytes
    if(cnt == 16):
      print()
      cnt = 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Overhead Bootloader Generator')
  parser.add_argument('file', help='firmware in binary format (.bin)')
  args = parser.parse_args()
  FILE = args.file
  data = open(FILE, 'r+b')
  curDate = "%02d%02d%04d" % (date.today().day, date.today().month, date.today().year)
  outputName = "bootloaderOverhead_" + curDate

  with data as f:
    # Read entire IVT
    ivt = bytearray(f.read(128))
    if(DEBUG):
      print("Original IVT:")
      printIVT(ivt)
    
    # Here's where the magic happens! Builds new IVT targeting app IVT
    for i in range (4, 128, 4):
      ivt[i] = 0x82
      ivt[i+1] = 0x00
      ivt[i+2] = (APP_START_ADDR + i) >> 8
      ivt[i+3] = (APP_START_ADDR + i) & 0x00ff

    # Replace original IVT with modified one
    f.seek(0)
    f.write(ivt)

    # Read modified IVT just to compare
    f.seek(0)
    ivt = bytearray(f.read(128))
    data.close()

    if(DEBUG):
      print("\nModified IVT:")
      printIVT(ivt)
    
    print("\nInterrupt vector table modified!")
    print("Converting to Intel Hex (.hex)")

    os.system("python bin2hex.py --offset=0x8000 %s %s.hex" %(FILE, outputName))

    print("Input size: %d bytes" %(os.path.getsize(FILE)))

    size = os.path.getsize("%s.hex" % (outputName))
    print("Final size: %d bytes" %(size))
    print("Finished!")
