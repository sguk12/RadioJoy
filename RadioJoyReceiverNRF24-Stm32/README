## Button matrix
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

## Encoders
6 encoders, 5 of them with push buttons.
Wave '~' means external interruption required:
```
GND, PB7                            2 ways header
PA14~,  PA15    no pushbutton       2 ways header
PA0~,   PA1      +PB3               3 ways header
PA2~,   PA3      +PB4               3 ways header
PA4~,   PA5      +PA9               3 ways header
PAC14~, PC15     +PA10              3 ways header
PB10,   PB11     +PB15              3 ways header
```

## Radio
nRF24L01(+) 2.4GHz Wireless Transceiver
The radio chip needs too much power for 3.3v power source on the controller, so the radio needs to be powered from the 5v. So we have to use the only 5v tolerable SPI, SPI2.
```
5 ways header:
PB12 -->  CS
PB13 -->  SCK
PB14 -->  MISO
PB15 -->  MOSI
PA8  -->  CE
````

## Analog axis (if needed)
All of the unused pins that are left for us: (PA11, PA12 are the USB pins, PC13 is LED)
all of the ADCs are 3.3v and 12 bit (0..4095)
```
PB0,    +GND, 3.3V     3 ways header
PB1,    +GND, 3.3V     3 ways header
PA6,    +GND, 3.3V     3 ways header
PA7     +GND, 3.3V     3 ways header
```