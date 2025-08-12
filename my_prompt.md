## Pico-Ice Development Using the Arduino-Pico core and VScode With PlatformIO

I wish to use the Arduino-Pico core, from Earl Philhower with VScode using PlatformIO to program the FPGA CRAM on the pico-ice development board.  The .bin FPGA bitstream files will eventually be created with the OSS CAD Suite, but for now we will just write either the traffic_light_controller.bin or blank_design.bin to the FPGA.  These files are located in our main directory. I included a utility (convert_binaries.py) to convert the binary files into C++ headers. 

## The Task
The pico-ice-sdk supplied with the pico-ice is using the pico-sdk, not the Arduino-Pico core.  We need to port the portions of the pico-ice-sdk that program the CRAM to the Arduino-Pico core.  Read the FPGA-DS.txt datasheet to come up with the steps and timing we need to do to program the CRAM.  Then verify them by reading the pico-ice-sdk, to make sure our code matches the documented sequence. Post-load verification via sysCONFIG is intentionally not performed to avoid resetting the user design.

## Documentation We Need
The pico-ice documentation is at: https://pico-ice.tinyvision.ai/md_getting__started.html  I have included the pico-ice-sdk source code in the project so you can use it as a reference. The pico-ice uses the RP2040 SPI1 to transfer data to and from the FPGA CRAM.  The documentation for using SPI with the Arduino Pico core is at:  https://arduino-pico.readthedocs.io/en/latest/spi.html  It also has other documentation for the Arduino Pico core if you follow the other links on that page.  I am including the ICE40UP5K FPGA datasheet converted from pdf to text (FPGA-dS.txt) for you to refer to.  I have included the pico-ice_spice_netlist so you can look at the connections on the pico-ice PCB.

The RP2040 GPIO I expect to use are:

| GPIO | Name         |
|------|--------------|
|   8  | ICE_SI (MOSI)|
|  11  | ICE_SO (MISO)|
|  10  | ICE_SCK      |
|  9   | ICE_SSN      |     
|  13  | LED_R        |
|  12  | LED_G        |
|  15  | LED_B        |
|  14  | RAM_SS       |
|  27  | \ICE_RESET (\CRESET) |
|  24  | PIN_CLOCK      |

You might think I have MOSI and MISO reversed on the RP2040, but I don't.  It has to be this way, because the FPGA does not switch which pins it is using for MOSI and MISO when it changes from master to slave and vice versa.  This will make it harder for us, because we will have to use software SPI or the RP2040 PIO.  We need to drive RAM_SS low, to avoid a bus contention between the RP2040 and the FLASH SRAM on the pico-ice when programming the CRAM.  The RP2040 will be the SPI master, and the FPGA will be the slave. The FPGA can be configured as master or slave, so we will need to be careful to configure it as the slave.

## Debugging Tools
The pico-ice has three LEDs (LED_R, LED_G, and LED_B) (all active low) which can be turned on by either the FPGA or the RP2040.  The traffic_light_controller.bin makes them blink first green, briefly yellow and then red over and over again, so the first choice LED I recommend for debugging is LED_B.  The serial port also works well for control and debugging.  I have an oscilloscope (with only 20k points of memory) which I can use to check signals to help debug.  

## The Purpose
The purpose of this project is to demonstrate how to program the pico-ice FPGA with Arduino-Pico core, using PlatformIO and VScode so others can follow in our footsteps. I would like to produce a simple example showing how to make doing this really easy and a document telling how to use it.  I also want a summary of the steps and timig used to program the CRAM in this document so the whole process is not mysterious at all.
