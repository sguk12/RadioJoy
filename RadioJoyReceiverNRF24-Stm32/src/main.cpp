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

#define DEBUG

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
#define NUMBER_OF_BUTTONS 52

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

#define ENC1A PA0
#define ENC1B PA1


RotaryEncoderWrapper encoderWrapper1(ENC1A, ENC1B, RotaryEncoder::LatchMode::TWO03);

void setup()
{
  DEBUG_BEGIN(115200);
  pinMode(LED_PIN, OUTPUT);

  // Initialize encoder wrapper
  encoderWrapper1.reset();
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ENC1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), encoder1ISR, CHANGE);

  
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
  // TODO: uncomment the below method when switching to STM32 joystik/radio master
  //
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
  // delay(delayInterval);
}

void encoder1ISR(){
  encoderWrapper1.tick();
}

void readDashboard() {
  for(uint8_t i=0; i < NUMBER_OF_BUTTONS; i++){
    Dashboard.setButton(i, 0);
  }

  // Update encoder state - this processes one tick at a time
  encoderWrapper1.update();
  
  // Set button states - these will alternate between UP/DOWN for each tick
  Dashboard.setButton(0, encoderWrapper1.getButtonCCW());  // CCW button
  Dashboard.setButton(1, encoderWrapper1.getButtonCW());   // CW button
  DEBUG_PRINT("CW Button state: ");
  DEBUG_PRINTLN(encoderWrapper1.getButtonCW());
  DEBUG_PRINT("CCW Button state: ");
  DEBUG_PRINTLN(encoderWrapper1.getButtonCCW());

  scanButtonMatrix();
}

void processButtonStates(uint8_t column, uint8_t rowStates) {
  for (uint8_t row = 0; row < 8; row++) {
    if (!(rowStates & (1 << row))) {  // Button pressed (assuming active low)
      uint8_t buttonIndex = column * 8 + row;
      DEBUG_PRINT("Button pressed: ");
      DEBUG_PRINT(buttonIndex);
      DEBUG_PRINT(" (Col: ");
      DEBUG_PRINT(column);
      DEBUG_PRINT(", Row: ");
      DEBUG_PRINT(row);
      DEBUG_PRINTLN(")");
      
      // Handle button press
      //handleButtonPress(buttonIndex);
    }
  }
}

void scanButtonMatrix() {
  for (uint8_t col = 0; col < 8; col++) {
    uint8_t columnMask = ~(1 << col);  // Creates the inverted bit pattern
    
    WIRE.beginTransmission(PCF8574_OUT); // transmit to device PCF8574_OUT
    WIRE.write(columnMask); // sends one byte  
    WIRE.endTransmission(); // stop transmitting

    WIRE.requestFrom(PCF8574_IN, 1); // request from device PCF8574_IN one byte
    uint8_t inputStates = 0xFF;
    if (WIRE.available())
      inputStates = WIRE.read(); // sends one byte  

    DEBUG_PRINT("Column mask: ");
#ifdef DEBUG
    Serial1.print(columnMask, BIN);
    Serial1.print(" ");
#endif
    DEBUG_PRINT("Column ");
    DEBUG_PRINT(col);
    DEBUG_PRINT(" Button state: ");
#ifdef DEBUG
    Serial1.println(inputStates, BIN);
#endif
    
    processButtonStates(col, inputStates);
  }
}
