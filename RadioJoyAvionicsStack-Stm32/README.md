# Avionics stack 
two double rotary encoders and a single  rotary encoder on a side.
stm32f103c8t6 is the same as the Dashboard, but no radio this time (yet)

## Rotary Encoders
Noname from Aliexpress, 10 0.1uF capasitors on the main board

## buttons
15 normally closed (push to off) buttons
2 buttons on the double encoders (one button on each)
1 button on the single side encoder
all of this is driven by 2 PCF8574P

## Other things
Claude fixed a bug in the RotaryEncoderWrapper.h that improves the USB button state reporting
