# Rotary Encoder with I²C Interface
Rotary encoders are a great addition to many projects. However, controlling them typically requires several pins, interrupts, and debouncing. Thanks to this device, a rotary encoder can be easily controlled via the I²C interface, with the built-in CH32V003 handling the rest. By assigning different I²C addresses, it is even possible to daisy-chain multiple rotary encoders. The device is powered through the I²C connection and operates within a voltage range of 2.7V to 5.5V.

![I2C_Knob_pic1.jpg](https://raw.githubusercontent.com/wagiminator/CH32V003-I2C-Knob/main/documentation/I2C_Knob_pic1.jpg)

# Hardware
## Schematic
![I2C_Knob_wiring.png](https://raw.githubusercontent.com/wagiminator/CH32V003-I2C-Knob/main/documentation/I2C_Knob_wiring.png)

## The CH32V003 Family of 32-bit RISC-V Microcontrollers
The CH32V003 series is a collection of industrial-grade general-purpose microcontrollers that utilize the QingKe RISC-V2A core design supporting the RV32EC instruction set. These microcontrollers are equipped with various features such as a 48MHz system main frequency, 16KB flash, 2KB SRAM, 2.7V - 5.5V supply voltage support, a single-wire serial debug interface, low power consumption, and an ultra-small package. Additionally, the CH32V003 series includes a built-in set of components including a DMA controller, a 10-bit ADC, op-amp comparators, multiple timers, and standard communication interfaces such as USART, I2C, and SPI.

## Building Instructions
1. Take the Gerber files (the *zip* file inside the *hardware* folder) and upload them to a PCB (printed circuit board) manufacturer of your choice (e.g., [JLCPCB](https://jlcpcb.com/)). They will use these files to create the circuit board for your device and send it to you.
2. Once you have the PCB, you can start soldering the components onto it. Use the BOM (bill of materials) and schematic as a guide to make sure everything is connected correctly. You can find the corresponding files in the *hardware* folder.
3. Upload the firmware by following the instructions in the next section (see below).

# Software
## Programming and Debugging Device
To program the CH32V003 microcontroller, you will need a special programming device which utilizes the proprietary single-wire serial debug interface (SDI). The [WCH-LinkE](http://www.wch-ic.com/products/WCH-Link.html) (pay attention to the "E" in the name) is a suitable device for this purpose and can be purchased commercially for around $4. This debugging tool is not only compatible with the CH32V003 but also with other WCH RISC-V and ARM-based microcontrollers.

![CH32V003_wch-linke.jpg](https://raw.githubusercontent.com/wagiminator/Development-Boards/main/CH32V003F4P6_DevBoard/documentation/CH32V003_wch-linke.jpg)

To use the WCH-LinkE on Linux, you need to grant access permissions beforehand by executing the following commands:
```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="8010", MODE="666"' | sudo tee /etc/udev/rules.d/99-WCH-LinkE.rules
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="8012", MODE="666"' | sudo tee -a /etc/udev/rules.d/99-WCH-LinkE.rules
sudo udevadm control --reload-rules
```

On Windows, if you need to you can install the WinUSB driver over the WCH interface 1 using the [Zadig](https://zadig.akeo.ie/) tool.

To upload the firmware, you should make the following connections to the WCH-LinkE:

```
WCH-LinkE      I2C-Knob
+-------+      +------+
|  SWDIO| <--> |DIO   |
|    GND| ---> |GND   |
|    3V3| ---> |VCC   |
+-------+      +------+
```

If the blue LED on the WCH-LinkE remains illuminated once it is connected to the USB port, it means that the device is currently in ARM mode and must be switched to RISC-V mode initially. There are a few ways to accomplish this:
- You can utilize the Python command-line tool [rvprog](https://pypi.org/project/rvprog/) (with *-v* option).
- Alternatively, you can select "WCH-LinkRV" in the software provided by WCH, such as MounRiver Studio or WCH-LinkUtility.
- Another option is to hold down the ModeS button on the device while plugging it into the USB port.

More information can be found in the [WCH-Link User Manual](http://www.wch-ic.com/downloads/WCH-LinkUserManual_PDF.html).

## Compiling and Uploading Firmware using the Makefile
### Linux
Install the toolchain (GCC compiler, Python3, and rvprog):
```
sudo apt install build-essential libnewlib-dev gcc-riscv64-unknown-elf
sudo apt install python3 python3-pip
pip install rvprog
```

Connect the I2C-Knob via the 3-pin PROG header to the WCH-LinkE programming device. Open a terminal and navigate to the folder with the *makefile*. Run the following command to compile and upload:
```
make flash
```

### Other Operating Systems
Follow the instructions on [CNLohr's ch32v003fun page](https://github.com/cnlohr/ch32v003fun/wiki/Installation) to set up the toolchain on your respective operating system (for Windows, use WSL). Also, install [Python3](https://www.pythontutorial.net/getting-started/install-python/) and [rvprog](https://pypi.org/project/rvprog/). Compile and upload with "make flash". Note that I only have Debian-based Linux and have not tested it on other operating systems.

## Compiling and Uploading Firmware using PlatformIO
- Install [PlatformIO](https://platformio.org) and [platform-ch32v](https://github.com/Community-PIO-CH32V/platform-ch32v). Follow [these instructions](https://pio-ch32v.readthedocs.io/en/latest/installation.html) to do so. Linux/Mac users may also need to install [pyenv](https://realpython.com/intro-to-pyenv).
- Click on "Open Project" and select the firmware folder with the *platformio.ini* file.
- Connect the WCH-LinkE to the board, then click "Upload".

## Uploading pre-compiled Firmware Binary
WCH offers the free but closed-source software [WCH-LinkUtility](https://www.wch.cn/downloads/WCH-LinkUtility_ZIP.html) to upload the precompiled hex-file with Windows. Select the "WCH-LinkRV" mode in the software, open the *.hex* file in the *bin* folder and upload it to the microcontroller.

Alternatively, there is an open-source tool called [minichlink](https://github.com/cnlohr/ch32v003fun/tree/master/minichlink) developed by Charles Lohr (CNLohr). It can be used with Windows, Linux and Mac.

If you have installed [Python3](https://www.pythontutorial.net/getting-started/install-python/) on your system, you can also use the platform-independent open-source command-line tool [rvprog](https://pypi.org/project/rvprog/) for uploading:
```
rvprog -f bin/i2c_knob.bin
```

## Power Cycle Erase
The firmware uses the MCU's sleep mode to save energy. However, this can make it impossible to reprogram the chip using the single-wire debug interface. If that happens, you will need to perform a power cycle erase ("unbrick") with your programming software. This is not necessary when using the Python tool [rvprog](https://pypi.org/project/rvprog/), as it automatically detects the issue and performs a power cycle on its own.

# Operating Instructions
The device has four 16-bit and two 8-bit registers that can be read and written. The 16-bit registers are signed and the least significant byte is always transmitted first. With each access (reading or writing), the registers are always transferred starting with the first in the following order:
1. Encoder wheel value (16-bit)
2. Encoder switch state (8-bit, 0=switch released, 1=switch pressed)
3. Encoder wheel value loop flag (8-bit, 0=do not loop, 1=loop around)
4. Encoder wheel minimum value (16-bit)
5. Encoder wheel maximum value (16-bit)
6. Encoder wheel value change step (16-bit)

An example code for controlling the device is attached. It uses the standard Arduino Wire library, so it should run on almost all supported microcontrollers.

```c
#include <Wire.h>

#define encoder_addr 0x36

int16_t value, lastvalue;
boolean pressed, lastpressed;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  encoder_set(-50, 50, 1, 0, 0);
}

void loop() {
  value = encoder_getValue();
  if(value != lastvalue) {
    Serial.println(value);
    lastvalue = value;
  }
  pressed = encoder_isPressed();
  if(pressed != lastpressed) {
    if(pressed) Serial.println("Switch was pressed");
    lastpressed = pressed;
  }
  delay(20);
}

// Set encoder wheel parameters
void encoder_set(int16_t rmin, int16_t rmax, int16_t rstep, int16_t rval, uint8_t rloop) {
  Wire.beginTransmission(encoder_addr);
  Wire.write((uint8_t)(rval & 0xff)); Wire.write((uint8_t)(rval >> 8));
  Wire.write(0); Wire.write(rloop);
  Wire.write((uint8_t)(rmin & 0xff)); Wire.write((uint8_t)(rmin >> 8));
  Wire.write((uint8_t)(rmax & 0xff)); Wire.write((uint8_t)(rmax >> 8));
  Wire.write((uint8_t)(rstep & 0xff)); Wire.write((uint8_t)(rstep >> 8));
  Wire.endTransmission();
}

// Set encoder wheel value
void encoder_setValue(int16_t rval) {
  Wire.beginTransmission(encoder_addr);
  Wire.write((uint8_t)(rval & 0xff)); Wire.write((uint8_t)(rval >> 8));
  Wire.endTransmission();
}

// Read encoder wheel value
int16_t encoder_getValue() {
  Wire.requestFrom(encoder_addr, 2);
  return((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8));
}

// Read encoder switch state
boolean encoder_isPressed() {
  Wire.requestFrom(encoder_addr, 3);
  Wire.read(); Wire.read();
  return(Wire.read());
}
```

# References, Links and Notes
- [EasyEDA Design Files](https://oshwlab.com/wagiminator)
- [MCU Templates](https://github.com/wagiminator/MCU-Templates)
- [MCU Flash Tools](https://github.com/wagiminator/MCU-Flash-Tools)
- [WCH: CH32V003 Datasheet](http://www.wch-ic.com/products/CH32V003.html)
- [ATtiny412 I2C-Knob](https://github.com/wagiminator/ATtiny412-I2C-Rotary-Encoder)

![I2C_Knob_pic2.jpg](https://raw.githubusercontent.com/wagiminator/CH32V003-I2C-Knob/main/documentation/I2C_Knob_pic2.jpg)
![I2C_Knob_pic3.jpg](https://raw.githubusercontent.com/wagiminator/CH32V003-I2C-Knob/main/documentation/I2C_Knob_pic3.jpg)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
