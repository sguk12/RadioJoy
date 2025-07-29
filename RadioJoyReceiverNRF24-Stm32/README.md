## Button matrix (deprecated, replaced with the two PCF8574P)
PB5 - PA15 are the pins of STM32F103C8T6 MCU
\+ is a button with the diode
```
+-----+-----+-----+-----+------> PB5  \
|     |     |     |     |              | 5 X 2 10 buttons
|     |     |     |     |              | 7 ways header
+-----+-----+-----+-----+------> PB6  /
|     |     |     |     | 
|     |     |     |     | 
+-----+-----+-----+-----+------> PB7     5 encoder buttons see the encoders section below
|     |     |     |     | 
|     |     |     |     | 
+-----+-----+-----+-----+------> PB8     rotary swithch 6 ways header
|     |     |     |     | 
|     |     |     |     | 
+-----+-----+-----+-----+------> PB9    rotary swithch 6 ways header
|     |     |     |     | 
|     |     |     |     | 
V     V     V     V     V
PB3   PB4   PA9   PA10  PA15
```

## GPIO Enhancer two PCF8574P connected via I2C
PB8, PB9, GND, +5V                 4 way header if using a separate breadboard
5 X 5 button matrix will need     10 way header

## Encoders
6 encoders, 5 of them with push buttons.
Wave '~' means external interruption required:
```
PB7~,   PB6      GND                3 ways header
PB5~,   PB4                         2 ways header
PB3~,   PA15                        2 ways header
PA10~,  PA9                         2 ways header
PB11~,  PB10                        2 ways header
PB1~,   PB0                         2 ways header
```

or 10 ways header + 3 ways header

## Radio
nRF24L01(+) 2.4GHz Wireless Transceiver
The radio chip needs too much power for 3.3v power source on the controller, so the radio needs to be powered from the 5v. So we have to use the only 5v tolerable SPI, SPI2.
```
7 ways header:
PB12 -->  CS
PB13 -->  SCK
PB14 -->  MISO
PB15 -->  MOSI
PA8  -->  CE
+5V
GND
````

## Analog axis (if needed)
All of the unused pins that are left for us: (PA11, PA12 are the USB pins, PC13 is LED)
all of the ADCs are 3.3v and 12 bit (0..4095)
```
PA0,    +GND, 3.3V     3 ways header
PA1,    +GND, 3.3V     3 ways header
PA2,    +GND, 3.3V     3 ways header
PA3,    +GND, 3.3V     3 ways header
PA4,    +GND, 3.3V     3 ways header
```

or 7 ways header for 5 axis

## Total
* 10 ways header X 2
* 7 ways header X 2
* 4 ways header
* 3 ways header
