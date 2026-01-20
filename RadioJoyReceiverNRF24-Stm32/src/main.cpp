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
uint8_t debounce(uint8_t x, uint8_t &count);

unsigned long lastRadioReset = 0;
int16_t previousRudderTrim = 0;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, NUMBER_OF_FIRST_JOYSTICK_BUTTONS, 0,
  true, true, false, true, true, false,
  true, true, false, false, false);

Joystick_ Dashboard(0x04,
  JOYSTICK_TYPE_JOYSTICK, NUMBER_OF_BUTTONS, 0,
  false, false, true, true, true, true,
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
RotaryEncoderWrapper encoderWrapper2(ENC2A, ENC2B, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoderWrapper encoderWrapper3(ENC3A, ENC3B, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoderWrapper encoderWrapper4(ENC4A, ENC4B, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoderWrapper encoderWrapper5(ENC5A, ENC5B, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoderWrapper encoderWrapper6(ENC6A, ENC6B, RotaryEncoder::LatchMode::FOUR3);
uint8_t pastSingleSwitchPosition = 0;
uint8_t pastFourSwitchPosition = 0;
uint8_t rawButtonMatrix[5][7];
int16_t dashboardRz=0;
int16_t dashboardRx=0;
int16_t dashboardRy=0;

void setup()
{
  DEBUG_BEGIN(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PA5, INPUT_ANALOG);

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
  
  encoderWrapper1.tick(); // manual polling of the first encoder

  if(lastRadioReset == 0){
    lastRadioReset = millis();
    #ifdef DEBUG
        radio.printDetails();   // DEBUG
    #endif
  }

  sendInvitationTo((char*)"rudder", fromRudderToReceiver);
  sendInvitationTo((char*)"joystick", fromJoystickToReceiver);
  sendInvitationTo((char*)"throttle", fromThrottleToReceiver);

  Joystick.sendState();
  
  readDashboard();
  Dashboard.sendState();

  delay(5);
}

void sendInvitationTo(const char* name, int8_t slave) {
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &slave, sizeof(int8_t), 1 )){ // This will block until complete
    DEBUG_PRINT(name); DEBUG_PRINTLN(" failed");   // DEBUG
    radio.startListening();
    if(lastRadioReset + 1000 < millis()){
      radioBegin();
      lastRadioReset = millis();
    }
  }else{
    DEBUG_PRINT(name); DEBUG_PRINT(" ");   // DEBUG
    readSlaveResponseAndUpdateJoystick();
  }
  
  encoderWrapper1.tick(); // manual polling of the first encoder
  delay(3);
}

uint8_t fuelLeftOffCount = 0;
uint8_t fuelRightOffCount = 0;
uint8_t magnetoLeftOffCount = 0;
uint8_t magnetoRightOffCount = 0;

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
        blink(5);
        DEBUG_PRINTLN(F("Rudder recieved."));   // DEBUG
      } else if (buf.fromToByte == fromJoystickToReceiver) {
        // Message with a good checksum received.
        Joystick.setXAxis(buf.axisX);
        Joystick.setYAxis(buf.axisY);
        for(uint8_t i=0; i < NUMBER_OF_JOYSTICK_BUTTONS; i++){
          // TODO: This is workaround for the MSFS2O24 mixing up the buttons from the two joysticks with the same name
          // assign the i-th bit of the buf.buttons
          Joystick.setButton(i, bitRead(buf.buttons, i));
        }
        // Joystick.setButton(0, joystickButtons[0]); // there is no button 0 on the Dashboard, so no overlap confusing MSFS2024
        DEBUG_PRINTLN(F("Joystick recieved."));   // DEBUG
        blink(5);
      } else if (buf.fromToByte == fromThrottleToReceiver) {
        // Message with a good checksum received.
        Joystick.setThrottle(buf.axisThrottle);
        Joystick.setRxAxis(buf.axisPropellor);
        Joystick.setRyAxis(buf.axisTrim);
        dashboardRz = buf.axisRudder;
        dashboardRx = buf.axisX;
        dashboardRy = buf.axisY;

        uint8_t throttleButtons[NUMBER_OF_THROTTLE_BUTTONS];
        for(uint8_t i=0; i < NUMBER_OF_THROTTLE_BUTTONS; i++){
          // assign the i-th bit of the buf.buttons
          throttleButtons[i] = bitRead(buf.buttons, i);
          Joystick.setButton(NUMBER_OF_JOYSTICK_BUTTONS + i, throttleButtons[i]);
        }
        uint8_t fuelLeftOff = throttleButtons[1]+throttleButtons[5] == 0 ? 1 : 0;
        uint8_t fuelRightOff = throttleButtons[0]+throttleButtons[4] == 0 ? 1 : 0;
        uint8_t magnetoLeftOff = throttleButtons[2]+throttleButtons[6]+throttleButtons[10]+throttleButtons[14] > 0 ? 0 : 1;
        uint8_t magnetoRightOff = throttleButtons[3]+throttleButtons[7]+throttleButtons[11]+throttleButtons[15] > 0 ? 0 : 1;
        fuelLeftOff = debounce(fuelLeftOff, fuelLeftOffCount);
        fuelRightOff = debounce(fuelRightOff, fuelRightOffCount);
        magnetoLeftOff = debounce(magnetoLeftOff, magnetoLeftOffCount);
        magnetoRightOff = debounce(magnetoRightOff, magnetoRightOffCount);

        uint8_t i = NUMBER_OF_JOYSTICK_BUTTONS + NUMBER_OF_THROTTLE_BUTTONS;
        Joystick.setButton(i++, fuelLeftOff);
        Joystick.setButton(i++, fuelRightOff);
        Joystick.setButton(i++, magnetoLeftOff);
        Joystick.setButton(i++, magnetoRightOff);

        DEBUG_PRINTLN(F("Throttle recieved."));   // DEBUG
        blink(5);
      } else{
        DEBUG_PRINT(F("Message is not from rudder or not for me: ")); DEBUG_PRINTLN(buf.fromToByte);   // DEBUG
      }
    }while(radio.available()); // read all the data from FIFO
  }
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
  DEBUG2_PRINT("Encoder"); DEBUG2_PRINT(i); DEBUG2_PRINT(" position: "); DEBUG2_PRINT(encoder.getPosition());
  DEBUG2_PRINT(" reported position: "); DEBUG2_PRINTLN(encoder.getReportedPosition());
  DEBUG2_PRINT("Encoder"); DEBUG2_PRINT(i); DEBUG2_PRINT(" Button state  CW: "); DEBUG2_PRINT(encoder.getButtonCW());
  DEBUG2_PRINT(" CCW: "); DEBUG2_PRINTLN(encoder.getButtonCCW());
}

void readDashboard() {
  encoderWrapper1.tick(); // manual polling of the first encoder
  int16_t analogIn = analogRead(PA5);
  int16_t rudderTrim = (analogIn + previousRudderTrim) >> 1; // low pass filter (kind of)
  Dashboard.setZAxis(rudderTrim);
  previousRudderTrim = rudderTrim;

  Dashboard.setRxAxis(dashboardRx);
  Dashboard.setRyAxis(dashboardRy);
  Dashboard.setRzAxis(dashboardRz);

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
  if (fourSwitchPosition == 0) {
    // 10 first pushbuttons
    for (uint8_t row = 0; row < 2; row++) {
      for (uint8_t col = 0; col < 5; col++) {
        Dashboard.setButton(buttonIndex, rawButtonMatrix[col][row]);
        buttonIndex++;
      }
    }
    // 10 "shifted" pushbuttons
    buttonIndex += 10; // skip 10 buttons/button positions 11-20 because the four-switch is in the position 0 and these buttons are shifted to 112-122
  } else {
    // 20 pushbuttons
    for (uint8_t row = 0; row < 4; row++) {
      for (uint8_t col = 0; col < 5; col++) {
        Dashboard.setButton(buttonIndex, rawButtonMatrix[col][row]);
        buttonIndex++;
      }
    }
  }

  // the first encoder is not modified with the switches
  encoderWrapper1.tick(); // polling the first encoder, while the other encoders work via interruptions
  // if (encoderWrapper1.update()) {printEncoderState(1, encoderWrapper1);}
  encoderWrapper1.update();
  // Set button states - these will alternate between UP/DOWN for each tick
  Dashboard.setButton(buttonIndex++, encoderWrapper1.getButtonCCW());  // CCW button
  Dashboard.setButton(buttonIndex++, encoderWrapper1.getButtonCW());   // CW button

  // single modifying switch is row 6
  // four enc modifying switch is row 5
  // encoder buttons are row 4

  uint8_t singleSwitchPosition = getSwitchPosition(rawButtonMatrix[0][6], rawButtonMatrix[1][6], rawButtonMatrix[2][6], rawButtonMatrix[3][6], rawButtonMatrix[4][6]);
  for (uint8_t slot = 0; slot < 6; slot++) {
    if (slot == singleSwitchPosition) {
      // if (encoderWrapper2.update()) {printEncoderState(2, encoderWrapper2);}
      encoderWrapper2.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper2.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper2.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[0][4]); // the 'single' encoder's pushbutton
    } else {
      buttonIndex += 3; // skip 3 buttons/button positions because all buttons have already been set to 0 in the first line of this function
    }
  }
  if (pastSingleSwitchPosition != singleSwitchPosition) {
    encoderWrapper2.reset();
    pastSingleSwitchPosition = singleSwitchPosition;
  }

  for (uint8_t slot = 0; slot < 6; slot++) {
    if (slot == fourSwitchPosition) {
      encoderWrapper3.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper3.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper3.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[1][4]); // the encoder's pushbutton

      encoderWrapper4.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper4.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper4.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[2][4]); // the encoder's pushbutton

      encoderWrapper5.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper5.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper5.getButtonCW());   // CW button
      Dashboard.setButton(buttonIndex++, rawButtonMatrix[3][4]); // the encoder's pushbutton

      // if (encoderWrapper6.update()) {printEncoderState(6, encoderWrapper6);}
      encoderWrapper6.update();
      // Set button states - these will alternate between UP/DOWN for each tick
      Dashboard.setButton(buttonIndex++, encoderWrapper6.getButtonCCW());  // CCW button
      Dashboard.setButton(buttonIndex++, encoderWrapper6.getButtonCW());   // CW button
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
    encoderWrapper3.reset();
    encoderWrapper4.reset();
    encoderWrapper5.reset();
    encoderWrapper6.reset();
    pastFourSwitchPosition = fourSwitchPosition;
  }
}
