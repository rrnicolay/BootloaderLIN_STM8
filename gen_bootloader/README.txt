This script modifies the IVT of the bootloader.

STM8S has an interrupt vector table (IVT) of 128 bytes and it is located at a
fixed address(0x8000 to 0x8080). What we need to do is remap this table to point
to app's vector table. So, in default interruption addresses (0x8000 to 0x8080)
we will pass control to appropriate app interruption. This will cause some extra
delay for an interruption reach the final handler code, and, as we have an
interrupt vector table for the bootloader and another for the app, some flash
space (124 bytes) is wasted. If we are going to protect the bootloader using UBC,
we need to go with two IVTs. An alternative would be to write app IVT over
bootloader IVT. With the chosen approach, what we need to do is redirect
interruptions from bootloader IVT to the app's IVT.

Input file: a binary file (.bin) of the bootloader. To generate this file,
put the project in "Release" mode and enable output convertion of a binary
file under project settings. Remember to configure the correct address space
for flash usage (e.g. 0x8000 to 0x8780) in project's .icf file.

To execute:
    Open cmd.exe;
    Navigate to this project's folder;
    Run "python genBootloader.py input.bin"
    An Intel Hex file will be generated with the modified interrupt vectors;

Pre-requitites:
    Python 3;