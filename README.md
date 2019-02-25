# dadamachines – doppler
## Cortex M4F & FPGA Development Board

![doppler](img/dadamachines-doppler-front.png)
- arduino compatible board
- 120Mhz ARM Cortex M4 MCU 1MB Flash (samd51g19a) with FPU
- FPGA 5000 LUT, 1MBit RAM, 6 DSP Cores,OSC, PLL (Lattice ice40up5k)
- breadboard friendly (DIL48)
- micro usb
- power over USB or external via pin headers
- VCC 3.5V …. 5.5V 
- all GPIO Pins are 3.3V
- 1 led connected to samd51
- 4 x 4 LED Matrix (connected on fpga)
- 2 Custom Buttons (connected on fpga)
- ARef Solder Jumper
- I2C (need external pullup), SPI, QSPI Pins
- 2 DAC pins, 10 ADC pins
- full opensource toolchain
- SWD programming pin headers
- double press reset to enter the bootloader
- uf2 bootloader with Firmwareupload via simple USB stick mode
