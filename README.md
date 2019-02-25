# dadamachines – doppler

![doppler](img/dadamachines-doppler-front.png)

The doppler is a Cortex M4F microcontroller + FPGA development board. It comes in the same tiny form factor similar to a [Teensy](https://www.pjrc.com/store/teensy35.html) and is open source. 

## Features
- 120Mhz ARM Cortex M4F MCU 1MB Flash (Microchip ATSAMD51G19A) with FPU
- FPGA 5000 LUT, 1MBit RAM, 6 DSP Cores,OSC, PLL (Lattice ICE40UP5K)
- Arduino IDE compatible
- Breadboard friendly (DIL48)
- Micro USB
- Power over USB or external via pin headers
- VCC 3.5V …. 5.5V 
- All GPIO Pins are 3.3V
- 1 LED connected to SAMD51
- 4 x 4 LED Matrix (connected to FPGA)
- 2 User buttons (connected to FPGA)
- AREF Solder Jumper
- I2C (need external pullup), SPI, QSPI Pins
- 2 DAC pins, 10 ADC pins
- Full open source toolchain
- SWD programming pin headers
- Double press reset to enter the bootloader
- [UF2 Bootloader](https://github.com/Microsoft/uf2-samdx1) with Firmwareupload via simple USB stick mode (Mass Storage Device)

## Getting started with the doppler
We are very excited about FPGA’s but getting started with them is daunting especially because of the ecosystem of proprietary toolchains and expensive hardware. The doppler takes away most of these pains by providing all the tooling to get you up and running.

There are two chips on the board a SAMD51 ARM Microcontroller and an ICE40 FPGA. The microcontroller is easily programmable with, for example, the beginner-friendly Arduino environment. It also needs to be used to upload the configuration as a bitstream to the FPGA. This guide goes in detail on how to set up both development environments for the SAMD51 and the ICE40.

### Set up programming via the Arduino IDE

### Build the bitstream for the FPGA
1. Install Docker: 
On MacOs you can do this easily by running:
> brew cask install docker
or [download here](https://www.docker.com/products/docker-desktop)

2. Clone or download doppler FPGA firmware: 
[https://github.com/noscene/Doppler_FPGA_Firmware](https://github.com/noscene/Doppler_FPGA_Firmware)
> git clone https://github.com/noscene/Doppler_FPGA_Firmware
> cd Doppler_FPGA_Firmware/doppler_simple_io

 https://github.com/noscene/Doppler_FPGA_Firmware/tree/master/doppler_simple_io


