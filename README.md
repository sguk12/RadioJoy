# RadioJoy
This is a wireless Joystick device. It consists of 3 or 4 modules:
1) Receiver: Arduino Pro Micro 5v 16 MHz is plugged into a USB port. It presents itself as two Joysticks: 6 axis 12 buttons and three axis (for the wireless  EDTracker)
2) Transmitter: Arduino pro mini 3.3v 8MHz reads the ADXL345 tilt sensor values and transmits them to the receiver as the Rudder axis
3) Transmitter2: Arduino pro mini 3.3v 8MHz handles the desktop part of the Joystick - 5 more axes TODO: add more details???
4) EDTracker see here for more details: https://github.com/sguk12/EdTracker-radio

1) Receiver schematics:
![Pro Micro based Receiver Module](/RadioJoyReceiverNRF24/receiver_schem.png?raw=true "Pro Micro based Receiver Module")
