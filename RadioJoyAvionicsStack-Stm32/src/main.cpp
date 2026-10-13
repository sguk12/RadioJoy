/**
 * This sketch is for STM32F103C8T6 Blue Pill
 * It receives the joystick tilt data via the radio channel and
 * presents it to the USB hub.
 * The rudder's transmitter arduino (Pro Mini 8MHz 3.3V) reads the
 * ADXL345 sensor data and sends it in too.
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <Joystick.h>
#include "RotaryEncoderWrapper.h"

// #define DEBUG

#ifdef DEBUG
  HardwareSerial Serial1(PA10, PA9);
#endif

#ifdef DEBUG
  #define DEBUG_BEGIN(x) Serial1.begin(x)
  #define DEBUG_PRINTLN(x)  Serial1.println(x)
  #define DEBUG_PRINT(x)  Serial1.print(x)
  // the below is to disable the every loop printouts and use the targeted printouts with DEBUG2_PRINT()
  // #define DEBUG_PRINTLN(x)
  // #define DEBUG_PRINT(x)
  #define DEBUG2_PRINTLN(x)  Serial1.println(x)
  #define DEBUG2_PRINT(x)  Serial1.print(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG2_PRINT(x)
  #define DEBUG2_PRINTLN(x)
#endif

#define LED_PIN PC13
#define NUMBER_OF_THROTTLE_BUTTONS 16
#define NUMBER_OF_JOYSTICK_BUTTONS 4
#define NUMBER_OF_FIRST_JOYSTICK_BUTTONS NUMBER_OF_JOYSTICK_BUTTONS + NUMBER_OF_THROTTLE_BUTTONS + 4 // 4 calculated OFF buttons for 4 rotary switches
// 20 pushbuttons + 5 encoder buttons * 6 switch positions + 5 encoders * 2 rotation directions * 6 switch positions + 1 unmodifiable encoder * 2 rotation dir 
// + 10 pushbuttons for the 0 switch position
// = 20 + 5 * 6 + 5 * 2 * 6 + 2 + 10
#define NUMBER_OF_BUTTONS 122


// functions declarations
void blink(int);
void scanButtonMatrix(void);
void encoder1ISR(void);
void encoder1ISR(void);
void encoder2ISR(void);
void encoder3ISR(void);
void encoder4ISR(void);
void encoder5ISR(void);
void readDashboard(void);
void mapRawButtonsToDashboardButtonArray(void);
uint8_t debounce(uint8_t x, uint8_t &count);

Joystick_ Dashboard(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, NUMBER_OF_BUTTONS, 0,
  false, false, true, true, true, true,
  false, false, false, false, false);


#define PCF8574_OUT 0X20
#define PCF8574_IN 0X21
TwoWire WIRE(PB9,PB8);

#define ENC1A PB1  //EXTI1
#define ENC1B PB0  //EXTI0
#define ENC2A PB10 //EXTI10
#define ENC2B PB11 //EXTI11
#define ENC3A PB7  //EXTI7
#define ENC3B PB6  //EXTI6
#define ENC4A PB5  //EXTI5
#define ENC4B PB4  //EXTI4
#define ENC5A PB3  //EXTI3
#define ENC5B PA15 //EXTI15


RotaryEncoderWrapper encoderWrapper1(ENC1A, ENC1B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper2(ENC2A, ENC2B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper3(ENC3A, ENC3B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper4(ENC4A, ENC4B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper5(ENC5A, ENC5B, RotaryEncoder::LatchMode::TWO03);
uint8_t pastSingleSwitchPosition = 0;
uint8_t pastFourSwitchPosition = 0;
uint8_t rawButtonMatrix[5][7];

void setup()
{
  DEBUG_BEGIN(115200);
  pinMode(LED_PIN, OUTPUT);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ENC1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3A), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3B), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4A), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4B), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC5A), encoder5ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC5B), encoder5ISR, CHANGE);

  
  Dashboard.begin(false);
  USB_Begin();
  
  WIRE.begin();
}

void loop()
{
  DEBUG_PRINTLN("Loop");   // DEBUG
  digitalWrite(LED_PIN, HIGH); //High means led is off
  
  
  readDashboard();
  Dashboard.sendState();

  delay(5);
}

uint8_t debounce(uint8_t x, uint8_t &count) {
  
  if (x == 1) {
    if (count < 4) {
      count ++;
      return 0;
    }
    return 1;
  } else {
    count = 0;  // Reset on any 0
  }
  return 0;
}


/*******************************************************************************************************
* Blink LED function
********************************************************************************************************/
void blink(int delayInterval)
{
  digitalWrite(LED_PIN, LOW);
  delay(delayInterval);
  digitalWrite(LED_PIN, HIGH); //High means led is off
}

void encoder1ISR(){
  encoderWrapper1.tick();
}
void encoder2ISR(){
  encoderWrapper2.tick();
}
void encoder3ISR(){
  encoderWrapper3.tick();
}
void encoder4ISR(){
  encoderWrapper4.tick();
}
void encoder5ISR(){
  encoderWrapper5.tick();
}

void printEncoderState(uint8_t i, RotaryEncoderWrapper encoder) {
  DEBUG2_PRINT("Encoder"); DEBUG2_PRINT(i); DEBUG2_PRINT(" position: "); DEBUG2_PRINT(encoder.getPosition());
  DEBUG2_PRINT(" reported position: "); DEBUG2_PRINTLN(encoder.getReportedPosition());
  DEBUG2_PRINT("Encoder"); DEBUG2_PRINT(i); DEBUG2_PRINT(" Button state  CW: "); DEBUG2_PRINT(encoder.getButtonCW());
  DEBUG2_PRINT(" CCW: "); DEBUG2_PRINTLN(encoder.getButtonCCW());
}

void readDashboard() {
  int16_t rudderTrim  = constrain(512 + encoderWrapper2.getPosition(), 0, 1023);
  int16_t dashboardRz = constrain(512 + encoderWrapper3.getPosition(), 0, 1023);
  int16_t dashboardRx = constrain(512 + encoderWrapper4.getPosition(), 0, 1023);
  int16_t dashboardRy = constrain(512 + encoderWrapper5.getPosition(), 0, 1023);

  Dashboard.setZAxis(rudderTrim);
  Dashboard.setRxAxis(dashboardRx);
  Dashboard.setRyAxis(dashboardRy);
  Dashboard.setRzAxis(dashboardRz);

  scanButtonMatrix();
  mapRawButtonsToDashboardButtonArray();
}


void processButtonStates(uint8_t column, uint8_t rowStates) {
  for (uint8_t row = 0; row < 8; row++) {
    uint8_t rawButtonMatrixRow = row;
    if (row == 4)
      continue; // P4 of the input PCF7584 is not connected
    if (row > 4)
      rawButtonMatrixRow = row - 1;
    // I'm using 15 normally closed (push to disconnect) pushbuttons
    if (row == 1 || row == 2 || row == 3){
        rawButtonMatrix[column][rawButtonMatrixRow] = rowStates & (1 << row);
    } else {
      rawButtonMatrix[column][rawButtonMatrixRow] = !(rowStates & (1 << row));
    }
    // if (rawButtonMatrix[column][rawButtonMatrixRow]) {
    //   DEBUG_PRINT("Button pressed: "); DEBUG_PRINT(" (Col: "); DEBUG_PRINT(column); DEBUG_PRINT(", Row: "); DEBUG_PRINT(row); DEBUG_PRINTLN(")");
    // }
  }
}

void scanButtonMatrix() {
  for (uint8_t col = 0; col < 5; col++) {
    uint8_t columnMask = ~(1 << col);  // Creates the inverted bit pattern
    
    WIRE.beginTransmission(PCF8574_OUT); // transmit to device PCF8574_OUT
    WIRE.write(columnMask); // sends one byte  
    WIRE.endTransmission(); // stop transmitting

    WIRE.requestFrom(PCF8574_IN, 1); // request from device PCF8574_IN one byte
    uint8_t inputStates = 0xFF;
    if (WIRE.available()){
      inputStates = WIRE.read(); // reads one byte  
      // printButtonStates(columnMask, col, inputStates);
      processButtonStates(col, inputStates);
    }
    else {
      DEBUG_PRINTLN("WIRE not available");
    }
  }
}

uint8_t getSwitchPosition(uint8_t col0, uint8_t col1, uint8_t col2, uint8_t col3, uint8_t col4) {
  if ( col0) return 1;
  if ( col1) return 2;
  if ( col2) return 3;
  if ( col3) return 4;
  if ( col4) return 5;
  return 0;
}

void mapRawButtonsToDashboardButtonArray() {
  for(uint8_t i=0; i < NUMBER_OF_BUTTONS; i++){
    Dashboard.setButton(i, 0);
  }
  uint8_t buttonIndex = 0;
  uint8_t fourSwitchPosition = getSwitchPosition(rawButtonMatrix[0][5], rawButtonMatrix[1][5], rawButtonMatrix[2][5], rawButtonMatrix[3][5], rawButtonMatrix[4][5]);
  // if (fourSwitchPosition == 0) {
  //   // 10 first pushbuttons
  //   for (uint8_t row = 0; row < 2; row++) {
  //     for (uint8_t col = 0; col < 5; col++) {
  //       Dashboard.setButton(buttonIndex, rawButtonMatrix[col][row]);
  //       buttonIndex++;
  //     }
  //   }
  //   // 10 "shifted" pushbuttons
  //   buttonIndex += 10; // skip 10 buttons/button positions 11-20 because the four-switch is in the position 0 and these buttons are shifted to 112-122
  // } else {
  //   // 20 pushbuttons
  //   for (uint8_t row = 0; row < 4; row++) {
  //     for (uint8_t col = 0; col < 5; col++) {
  //       Dashboard.setButton(buttonIndex, rawButtonMatrix[col][row]);
  //       buttonIndex++;
  //     }
  //   }
  // }

  // single modifying switch is row 6
  // four enc modifying switch is row 5
  // encoder buttons are row 4

  encoderWrapper1.update();
  // Set button states - these will alternate between UP/DOWN for each tick
  Dashboard.setButton(buttonIndex++, encoderWrapper1.getButtonCCW());  // CCW button
  Dashboard.setButton(buttonIndex++, encoderWrapper1.getButtonCW());   // CW button
  Dashboard.setButton(buttonIndex++, rawButtonMatrix[0][4]); // the 'single' encoder's pushbutton

  for (uint8_t slot = 0; slot < 6; slot++) {
    if (slot == fourSwitchPosition) {
      encoderWrapper2.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper2.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper2.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[1][4]); // the encoder's pushbutton

      encoderWrapper3.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper3.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper3.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[2][4]); // the encoder's pushbutton

      encoderWrapper4.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper4.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper4.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[3][4]); // the encoder's pushbutton

      // if (encoderWrapper6.update()) {printEncoderState(6, encoderWrapper6);}
      encoderWrapper5.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper5.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper5.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[4][4]); // the encoder's pushbutton
    } else {
      buttonIndex += 12; // skip 12 buttons/button positions because all buttons have already been set to 0 in the first line of this function
    }
  }

  if (fourSwitchPosition == 0) {
    // 10 "shifted" pushbuttons
    for (uint8_t row = 2; row < 4; row++) {
      for (uint8_t col = 0; col < 5; col++) {
        Dashboard.setButton(buttonIndex, rawButtonMatrix[col][row]);
        buttonIndex++;
      }
    }
  }
  
  if (pastFourSwitchPosition != fourSwitchPosition) {
    encoderWrapper2.reset();
    encoderWrapper3.reset();
    encoderWrapper4.reset();
    encoderWrapper5.reset();
    pastFourSwitchPosition = fourSwitchPosition;
  }
}
