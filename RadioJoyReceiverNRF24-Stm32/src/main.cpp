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
#include <SPI.h>
#include <RF24.h>
#include <Joystick.h>
#include "RadioJoy.h"
#include "RotaryEncoderWrapper.h"

// #define DEBUG

#ifdef DEBUG
  HardwareSerial Serial1(PA10, PA9);
#endif

#ifdef DEBUG
  #define DEBUG_BEGIN(x) Serial1.begin(x)
  #define DEBUG_PRINTLN(x)  Serial1.println(x)
  #define DEBUG_PRINT(x)  Serial1.print(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif

#define LED_PIN PC13
#define NUMBER_OF_BUTTONS 112 // 20 pushbuttons + 5 encoder buttons * 6 switch positions + 5 encoders * 2 rotation directions * 6 switch positions + 1 unmodifiable encoder * 2 rotation dir = 20 + 5 * 6 + 5 * 2 * 6 + 2

// functions declarations
void radioBegin(void);
void blink(int);
void sendInvitationTo(const char*, int8_t);
void readSlaveResponseAndUpdateJoystick(void);
void scanButtonMatrix(void);
void encoder1ISR(void);
void encoder2ISR(void);
void encoder3ISR(void);
void encoder4ISR(void);
void encoder5ISR(void);
void encoder6ISR(void);
void readDashboard(void);
void mapRawButtonsToDashboardButtonArray(void);

unsigned long lastRadioReset = 0;


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, 12, 0,
  true, true, false, true, true, false,
  true, true, false, false, false);

Joystick_ Dashboard(0x04,
  JOYSTICK_TYPE_JOYSTICK, NUMBER_OF_BUTTONS, 0,
  false, false, false, false, false, false,
  false, false, false, false, false);


RF24 radio(PA8, PB12); // radio(9, 8) Arduino's pins connected to CE,CS pins on NRF24L01

#define PCF8574_OUT 0X20
#define PCF8574_IN 0X21
TwoWire WIRE(PB9,PB8);

#define ENC1A PA7  //EXTI7 change to EXTI2 ?
#define ENC1B PA6  //EXTI6 change to EXTI8,9,12,13,14 ?
#define ENC2A PB1  //EXTI1
#define ENC2B PB0  //EXTI0
#define ENC3A PB11 //EXTI11
#define ENC3B PB10 //EXTI10
#define ENC4A PB6  //EXTI6
#define ENC4B PB7  //EXTI7
#define ENC5A PB4  //EXTI4
#define ENC5B PB5  //EXTI5
#define ENC6A PA15 //EXTI15
#define ENC6B PB3  //EXTI3


RotaryEncoderWrapper encoderWrapper1(ENC1A, ENC1B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper2(ENC2A, ENC2B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper3(ENC3A, ENC3B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper4(ENC4A, ENC4B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper5(ENC5A, ENC5B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoderWrapper encoderWrapper6(ENC6A, ENC6B, RotaryEncoder::LatchMode::TWO03);
uint8_t pastSingleSwitchPosition = 0;
uint8_t pastFourSwitchPosition = 0;
uint8_t rawButtonMatrix[5][7];

void setup()
{
  DEBUG_BEGIN(115200);
  pinMode(LED_PIN, OUTPUT);

  // register interrupt routine
  // the first encoder is going to be polled becasuse of the EXTI6,7 overlap
  // attachInterrupt(digitalPinToInterrupt(ENC1A), encoder1ISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENC1B), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3A), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3B), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4A), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4B), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC5A), encoder5ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC5B), encoder5ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC6A), encoder6ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC6B), encoder6ISR, CHANGE);

  
  Joystick.setRudderRange(0, 255); //default axis min..max is 0..1023
  Joystick.begin(false);
  Dashboard.begin(false);
  USB_Begin();
  
  // we are using SPI 2 because all pins of this SPI are 5v tolerable
  SPI.setMOSI(PB15);
  SPI.setMISO(PB14);
  SPI.setSCLK(PB13);
  SPI.begin();
  
  radioBegin();
  WIRE.begin();
}

void loop()
{
  DEBUG_PRINTLN("Loop");   // DEBUG
  digitalWrite(LED_PIN, HIGH); //High means led is off
  if(lastRadioReset == 0){
    lastRadioReset = millis();
    #ifdef DEBUG
        radio.printDetails();   // DEBUG
    #endif
  }

  sendInvitationTo((char*)"rudder", fromRudderToReceiver);
  sendInvitationTo((char*)"throttle", fromThrottleToReceiver);

  Joystick.sendState();
  
  readDashboard();
  Dashboard.sendState();

  delay(5);
}

void sendInvitationTo(const char* name, int8_t slave) {
  // radio.stopListening();                                    // First, stop listening so we can talk.
  // if (!radio.write( &slave, sizeof(int8_t), 1 )){ // This will block until complete
  //   DEBUG_PRINT(name);   // DEBUG
  //   DEBUG_PRINTLN(" failed");   // DEBUG
  //   radio.startListening();
  //   if(lastRadioReset + 1000 < millis()){
  //     radioBegin();
  //     lastRadioReset = millis();
  //   }
  // }else{
  //   DEBUG_PRINT(name);   // DEBUG
  //   DEBUG_PRINT(" ");   // DEBUG
  //   readSlaveResponseAndUpdateJoystick();
  // }
  delay(5);
}

void readSlaveResponseAndUpdateJoystick(){
  DEBUG_PRINT(F("readSlaveResponseAndUpdateJoystick "));   // DEBUG

  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = millis();               // Set up a timeout period, get the current microseconds
  bool timeout = false;                                   // Set up a variable to indicate if a response was received or not

  while ( ! radio.available() ){                             // While nothing is received
    if (millis() - started_waiting_at > 20 ){            // If waited longer than 20ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }
  }

  if ( timeout ){
    DEBUG_PRINTLN(F("\t----- timed out."));   // DEBUG
  }else{
    do{
      RadioJoystick buf;
      radio.read( &buf, sizeof(buf) );
      if (buf.fromToByte == fromRudderToReceiver){
        // Message with a good checksum received.
        Joystick.setRudder(buf.axisRudder);
        blink(10);
        DEBUG_PRINTLN(F("Rudder recieved."));   // DEBUG
      } else if (buf.fromToByte == fromThrottleToReceiver) {
        // Message with a good checksum received.
        Joystick.setXAxis(buf.axisX);
        Joystick.setYAxis(buf.axisY);
        Joystick.setThrottle(buf.axisThrottle);
        Joystick.setRxAxis(buf.axisPropellor);
        Joystick.setRyAxis(buf.axisTrim);
        for(uint8_t i=0; i < 12; i++){
          // assign the i-th bit of the buf.buttons
          Joystick.setButton(i, bitRead(buf.buttons, i));
        }
        DEBUG_PRINTLN(F("Throttle recieved."));   // DEBUG
        blink(10);
      } else{
        DEBUG_PRINT(F("Message is not from rudder or not for me: "));   // DEBUG
        DEBUG_PRINTLN(buf.fromToByte);   // DEBUG
      }
    }while(radio.available()); // read all the data from FIFO
  }
}

void radioBegin(){
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(102);
  radio.openWritingPipe(TRANSMITTER_ADDRESS);
  radio.openReadingPipe(1,RECEIVER_ADDRESS);
  radio.enableDynamicAck();

  // Start the radio listening for data
  radio.startListening();
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
void encoder6ISR(){
  encoderWrapper6.tick();
}

void printEncoderState(uint8_t i, RotaryEncoderWrapper encoder) {
  DEBUG_PRINT("Encoder");
  DEBUG_PRINT(i);
  DEBUG_PRINT(" CW Button state: ");
  DEBUG_PRINTLN(encoder.getButtonCW());
  DEBUG_PRINT("Encoder");
  DEBUG_PRINT(i);
  DEBUG_PRINT(" CCW Button state: ");
  DEBUG_PRINTLN(encoder.getButtonCCW());
}

void readDashboard() {
  scanButtonMatrix();
  mapRawButtonsToDashboardButtonArray();
}

// void printButtonStates(uint8_t columnMask, uint8_t col, uint8_t inputStates){
//     DEBUG_PRINT("Column mask: ");
// #ifdef DEBUG
//     Serial1.print(columnMask, BIN);
//     Serial1.print(" ");
// #endif
//     DEBUG_PRINT("Column ");
//     DEBUG_PRINT(col);
//     DEBUG_PRINT(" Button state: ");
// #ifdef DEBUG
//     Serial1.println(inputStates, BIN);
// #endif
// }

/*
among the buttons I have 7 normally closed push buttons :`(
Button pressed: 11 (Col: 1, Row: 3)
Button pressed: 18 (Col: 2, Row: 2)
Button pressed: 19 (Col: 2, Row: 3)
Button pressed: 26 (Col: 3, Row: 2)
Button pressed: 27 (Col: 3, Row: 3)
Button pressed: 34 (Col: 4, Row: 2)
Button pressed: 35 (Col: 4, Row: 3)
*/
void processButtonStates(uint8_t column, uint8_t rowStates) {
  for (uint8_t row = 0; row < 8; row++) {
    uint8_t rawButtonMatrixRow = row;
    if (row == 4)
      continue; // P4 is not connected
    if (row > 4)
      rawButtonMatrixRow = row - 1;
    // workarond for normally closed pushbuttons
    if ((column == 1 && row == 3) ||
       (column == 2 && row == 2) ||
       (column == 2 && row == 3) ||
       (column == 3 && row == 2) ||
       (column == 3 && row == 3) ||
       (column == 4 && row == 2) ||
       (column == 4 && row == 3)
      ){
        rawButtonMatrix[column][rawButtonMatrixRow] = rowStates & (1 << row);
    } else {
      rawButtonMatrix[column][rawButtonMatrixRow] = !(rowStates & (1 << row));
    }
    if (rawButtonMatrix[column][rawButtonMatrixRow]) {
      DEBUG_PRINT("Button pressed: ");
      DEBUG_PRINT(" (Col: ");
      DEBUG_PRINT(column);
      DEBUG_PRINT(", Row: ");
      DEBUG_PRINT(row);
      DEBUG_PRINTLN(")");
    }
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
  uint8_t i = 0;
  // 20 pushbuttons
  for (uint8_t row = 0; row < 4; row++) {
    for (uint8_t col = 0; col < 5; col++) {
      Dashboard.setButton(i, rawButtonMatrix[col][row]);
      i++;
    }
  }

  // the first encoder is not modified with the switches
  encoderWrapper1.tick(); // polling the first encoder, while the other encoders work via interruptions
  encoderWrapper1.update();
  // Set button states - these will alternate between UP/DOWN for each tick
  Dashboard.setButton(i++, encoderWrapper1.getButtonCCW());  // CCW button
  Dashboard.setButton(i++, encoderWrapper1.getButtonCW());   // CW button

  // single modifying switch is row 6
  // four enc modifying switch is row 5
  // encoder buttons are row 4

  uint8_t singleSwitchPosition = getSwitchPosition(rawButtonMatrix[0][6], rawButtonMatrix[1][6], rawButtonMatrix[2][6], rawButtonMatrix[3][6], rawButtonMatrix[4][6]);
  for (uint8_t slot = 0; slot < 6; slot++) {
    if (slot == singleSwitchPosition) {
      encoderWrapper2.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(i++, encoderWrapper2.getButtonCCW());  // CCW button
      Dashboard.setButton(i++, encoderWrapper2.getButtonCW());   // CW button
      Dashboard.setButton(i++, rawButtonMatrix[0][4]); // the 'single' encoder's pushbutton
    } else {
      Dashboard.setButton(i++, 0);  // CCW button
      Dashboard.setButton(i++, 0);   // CW button
      Dashboard.setButton(i++, 0);   // push button
    }
  }
  if (pastSingleSwitchPosition != singleSwitchPosition) {
    encoderWrapper2.reset();
    pastSingleSwitchPosition = singleSwitchPosition;
  }

  uint8_t fourSwitchPosition = getSwitchPosition(rawButtonMatrix[0][5], rawButtonMatrix[1][5], rawButtonMatrix[2][5], rawButtonMatrix[3][5], rawButtonMatrix[4][5]);
  for (uint8_t slot = 0; slot < 6; slot++) {
    if (slot == fourSwitchPosition) {
      encoderWrapper3.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(i++, encoderWrapper3.getButtonCCW());  // CCW button
      Dashboard.setButton(i++, encoderWrapper3.getButtonCW());   // CW button
      Dashboard.setButton(i++, rawButtonMatrix[1][4]); // the encoder's pushbutton

      encoderWrapper4.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(i++, encoderWrapper4.getButtonCCW());  // CCW button
      Dashboard.setButton(i++, encoderWrapper4.getButtonCW());   // CW button
      Dashboard.setButton(i++, rawButtonMatrix[2][4]); // the encoder's pushbutton

      encoderWrapper5.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(i++, encoderWrapper5.getButtonCCW());  // CCW button
      Dashboard.setButton(i++, encoderWrapper5.getButtonCW());   // CW button
      Dashboard.setButton(i++, rawButtonMatrix[3][4]); // the encoder's pushbutton

      encoderWrapper6.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(i++, encoderWrapper6.getButtonCCW());  // CCW button
      Dashboard.setButton(i++, encoderWrapper6.getButtonCW());   // CW button
      Dashboard.setButton(i++, rawButtonMatrix[4][4]); // the encoder's pushbutton
    } else {
      Dashboard.setButton(i++, 0);  // CCW button
      Dashboard.setButton(i++, 0);   // CW button
      Dashboard.setButton(i++, 0);   // push button

      Dashboard.setButton(i++, 0);  // CCW button
      Dashboard.setButton(i++, 0);   // CW button
      Dashboard.setButton(i++, 0);   // push button

      Dashboard.setButton(i++, 0);  // CCW button
      Dashboard.setButton(i++, 0);   // CW button
      Dashboard.setButton(i++, 0);   // push button

      Dashboard.setButton(i++, 0);  // CCW button
      Dashboard.setButton(i++, 0);   // CW button
      Dashboard.setButton(i++, 0);   // push button
    }
  }
  if (pastFourSwitchPosition != fourSwitchPosition) {
    encoderWrapper3.reset();
    encoderWrapper4.reset();
    encoderWrapper5.reset();
    encoderWrapper6.reset();
    pastFourSwitchPosition = fourSwitchPosition;
  }
}
