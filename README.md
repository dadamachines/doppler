# dadamachines – doppler

![doppler](img/dadamachines-doppler-front.png)

The doppler is a Cortex M4F Microcontroller + FPGA development board. It comes in the same tiny form factor similar to a [Teensy](https://www.pjrc.com/store/teensy35.html) and is open source. 

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

There are two chips on the board a SAMD51 ARM Microcontroller and an ICE40 FPGA. The Microcontroller is easily programmable with, for example, the beginner-friendly Arduino environment. It also needs to be used to upload the configuration as a bitstream to the FPGA. This guide goes in detail on how to set up both development environments for the SAMD51 and the ICE40.

### Set up programming via the Arduino IDE
Please find the instructions in our repository with the dadamachines board support packages for **Arduino**.
[https://github.com/dadamachines/arduino-board-index](https://github.com/dadamachines/arduino-board-index)

### Build the bitstream for the FPGA

1.1 Install **Docker** as binary: 
- [download here](https://www.docker.com/products/docker-desktop).

1.2 If you are on **macOS** using **homebrew** you can do this easily by running:  
```
brew cask install docker
```  

2. Open the **Docker** Application. And give it the privileges it's asking for.

3. [Download zip](https://github.com/dadamachines/doppler-FPGA-firmware/archive/master.zip) or git clone the [**doppler-FPGA-firmware**](https://github.com/dadamachines/doppler-FPGA-firmware):  
```
  git clone https://github.com/dadamachines/doppler-FPGA-firmware
```
4. Go into the directory you just downloaded.    
```
cd doppler-FPGA-firmware/
```
5. Build the **icestorm** toolchain with **Docker**:  
This will take a while...  
```
docker build -t icestorm  icestorm/
```
6. Set the Mountpoint for **Docker**  
```
export MOUNTPOINT=`pwd`  
docker run -it -v $MOUNTPOINT:/PRJ icestorm bash
```
7. Now we are in the container and can build our bitstream.   For our **doppler_simple_io** example:  
```
cd PRJ/doppler_simple_io/
make
ls -l
```

### Load the bitstream on the FPGA
```
  #include <ICEClass.h>
  #include "[PATHTO]/top.bin.h"
  ICEClass ice40;
   
  void setup() {
    ice40.upload(top_bin,sizeof(top_bin)); // Upload BitStream Firmware to FPGA -> see variant.h
    ice40.upload(); // Upload BitStream Firmware to FPGA -> see variant.h
    ice40.initSPI();  // start SPI runtime Link to FPGA
  }
  
  void loop() {
     
    static uint16_t x = 0;
    ice40.sendSPI16(x++);   delay(50);
  }
```

### Programming the FPGA
Our language of choice for the FPGA is Verilog

We recommend [this Youtube channel]([https://www.youtube.com/channel/UCsdA-aNqtMA1%5C_2T15aXePWw/videos]) to learn Verilog

[https://www.youtube.com/watch?v=-bIeiMmqaZE](https://www.youtube.com/watch?v=-bIeiMmqaZE)

## Board Layout and PINs

```
    /*          DOPPLER-Board-Layout:
     *                                                                                    ---------------- FPGA Pins ------------------
     *                                                     DAC1      SCK  MOSI DAC0      LedR LedG LedB       CT1            CP0
     * DIL Pin 48   47   46   45   44   43   42   41   40   39   38   37   36   35   34   33   32   31   30   29   28   27   26   25
     *       |--O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O---|
     * name  | VIN  5V  3.3V  A10  A9   A8   A7   A6   A5   A4   A3   A2   A1   A0   GND  R2   R1   R0   F14  F13  F12  F11  F10  F9   |
     * alt   | VIN  5V  3.3V PA11 PA10 PA09 PA08 PA07 PA06 PA05 PA04 PB09 PB08 PA02  GND  41   40   39   38   37   36   35   34   32   |
     *       |                                                                                            ö  ö  ö  ö                   |
     *      |                                                                                             ö  ö  ö  ö         |BTN:S1|  |
     *     | USB                           DOPPLER: SamD51 <- SPI -> icE40        |BTN:RESET|             ö  ö  ö  ö                   |
     *      |                                                                                             ö  ö  ö  ö         |BTN:S2|  |
     *       |                                                                                                                         |
     * alt   | GND PA13 PA12 PB11 PA14 PA15 PB10 PA31 PA30  RES PA19 PA20 PA21 PA22 3.3V  11   12   13   18   19   20   21   23   25   |
     * name  | GND   0    1    2    3    4    5                   6    7    8    9  3.3V  F0   F1   F2   F3   F4   F5   F6   F7   F8   |
     *       L--O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O---|
     * DIL Pin  1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24
     *             SCL  SDA   MISO           SS   SWD  SWC RES                                 CT0                      CP0
     *             -- I2C--                       --- SWD  ---   ----- Shared  -----      ---------------- FPGA Pins ------------------
     */
```

## Examples

### Arduino - Blink on board LED

```
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```

### Arduino - fast write 2 DAC Channels on A0 and A4 (PA02+PA05)

```
void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_DAC0,OUTPUT);
  pinMode(PIN_DAC1,OUTPUT);
  dacInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t left = 0;
  static uint16_t right = 0;
  left+=256;
  right-=256;
  dacWrite(left,right);
}
```

### Arduino - Create second I2C Bus as Wire1 on pins A7+A8 and scan the bus
see https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-wire
how to handle sercoms
```
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function
TwoWire Wire1(&sercom0, A7, A8);  // create new Wire Port

void setup() {
  // put your setup code here, to run once:
  Wire1.begin();  // set pinPeripheral after this line!!!
  pinPeripheral(A7, PIO_SERCOM);    // assign SDA
  pinPeripheral(A8, PIO_SERCOM);    // assign SDC
  
  Serial.begin(9600);
  Serial.println("\nI2C Scanner");
}

void loop() {
  // put your main code here, to run repeatedly:
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )   {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    Wire1.requestFrom(address,2);
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error==4)  {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done, if find on each addr -> check pullups! \n");
  delay(5000);           // wait 5 seconds for next scan
}
```

### Arduino - Create UART RX,TX Serial2 on pins A7+A8 
Learn how to use SERCOMS on SAMD Microcontrollers [https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial](https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial)
```
// Create Serial2 instance
#include "wiring_private.h" // pinPeripheral() function
Uart Serial2 (&sercom0, A7, A8, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_0_Handler() { Serial2.IrqHandler(); }
void SERCOM0_1_Handler() { Serial2.IrqHandler(); }
void SERCOM0_2_Handler() { Serial2.IrqHandler(); }
void SERCOM0_3_Handler() { Serial2.IrqHandler(); }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);
  pinPeripheral(A7, PIO_SERCOM);  // TX
  pinPeripheral(A8, PIO_SERCOM);  // RX
}

uint8_t i=0;
void loop() {
  Serial.print(i);
  Serial2.write(i++);
  if (Serial2.available()) {
    Serial.print(" -> 0x"); Serial.print(Serial2.read(), HEX);
  }
  Serial.println();
  delay(500);
}
```

### FPGA - Demo > set 4x4 LED Matrix
See [https://github.com/dadamachines/doppler-FPGA-firmware](https://github.com/dadamachines/doppler-FPGA-firmware) to learn how to make a bitstream from Verilog.
```
#include <ICEClass.h>

ICEClass ice40;
uint16_t leds = 1;

void setup() {
  // put your setup code here, to run once:
  ice40.upload(); // Upload BitStream Firmware to FPGA -> see variant.h
  delay(100);

  // start SPI runtime Link to FPGA
  ice40.initSPI();
}


void loop() {
    // put your main code here, to run repeatedly:
    ice40.sendSPI16(leds );
    leds = leds << 1;
    if(leds==0){
      leds=1;  
    }
    delay(100);
}
```

### FPGA - Demo > Show hex chars on 4x4 matrix
```
#include <ICEClass.h>
ICEClass ice40;
uint16_t hexmapFont[16] = { 0xF99F,0xF22F,0xF42F,0xF17F,0x1F99,0x7F8F,0xF9F8,0x111F,
                            0x7DBE,0x1F9F,0x9F9F,0xADAC,0xF88F,0xE99E,0xF8EF,0x8E8F };
void setup() { // put your setup code here, to run once:
  ice40.upload(); // Upload BitStream Firmware to FPGA -> see variant.h
  delay(100);
  ice40.initSPI();  // start SPI runtime Link to FPGA
}

void loop() {  // put your main code here, to run repeatedly:
  for(int i = 0 ; i < 16 ; i++){
      ice40.sendSPI16(hexmapFont[i] );  
      delay(800);
  }
}
```

## Contribute
To report a bug, contribute, discuss usage, or simply request support, please [create an issue here](https://github.com/dadamachines/doppler/issues/new).

## Credits
Ideation & Software: [Sven Braun](https://github.com/noscene)  
Design & Hardware: [Johannes Elias Lohbihler](https://github.com/nevvkid)  
Sales & Distribution: [dadamachines](https://dadamachines.com)  
Production: [Watterott electronic](https://github.com/watterott)  


Arduino & Arduino IDE: [arduino.cc](https://www.arduino.cc/)  
SAMD51 Arduino Cores: [Adafruit Industries](https://github.com/adafruit)  
Icestorm: [https://github.com/cliffordwolf/icestorm](https://github.com/cliffordwolf/icestorm)  
Yosys: [https://github.com/YosysHQ/yosys](https://github.com/YosysHQ/yosys)  


