/**
 * This sketch is for STM32F103C8T6 Blue Pill
 * It receives the joystick tilt data via the radio channel and
 * presents it to the USB hub.
 * The rudder's transmitter arduino (Pro Mini 8MHz 3.3V) reads the
 * ADXL345 sensor data and sends it in too.
 *
 */
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Joystick.h>
#include <RotaryEncoder.h>
#include "RadioJoy.h"

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

// functions declarations
void radioBegin(void);
void blink(int);
void readSlaveResponseAndUpdateJoystick(void);
void encoder1ISR(void);
void encoder2ISR(void);
void encoder3ISR(void);
void encoder4ISR(void);
void encoder5ISR(void);
void encoder6ISR(void);
struct EncoderDirection {
  uint8_t colckwise; // 0 means counter clockwise, 1 means clockwise
  uint8_t buttonValue;
};
EncoderDirection deduceEncoderClick(RotaryEncoder);


unsigned long lastRadioReset = 0;


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, 78, 0,
  true, true, false, true, true, false,
  true, true, false, false, false);


RF24 radio(PA8, PB12); // radio(9, 8) Arduino's pins connected to CE,CS pins on NRF24L01

#define ENC1A PA0
#define ENC1B PA1
int16_t positionEncoder1 = 0;
// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder1 = nullptr;

void setup()
{
  DEBUG_BEGIN(115200);
  pinMode(LED_PIN, OUTPUT);

  // setup the rotary encoder functionality
  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder1 = new RotaryEncoder(ENC1A, ENC1B, RotaryEncoder::LatchMode::TWO03);
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ENC1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), encoder1ISR, CHANGE);

  USB_Begin();

  Joystick.setXAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
  Joystick.setRxAxisRange(0, 1023);
  Joystick.setRyAxisRange(0, 1023);
  Joystick.setThrottleRange(0, 1023);
  Joystick.setRudderRange(0, 255);
  Joystick.begin(false);

  // we are using SPI 2 because all pins of this SPI are 5v tolerable
  SPI.setMOSI(PB15);
  SPI.setMISO(PB14);
  SPI.setSCLK(PB13);
  SPI.begin();

  radioBegin();
}

void loop()
{
  // DEBUG_PRINTLN("Loop");   // DEBUG
    static int pos = 0;

  encoder1->tick(); // just call tick() to check the state.

  long newPos = encoder1->getPosition();
  if (pos != newPos) {
    Serial1.print("pos:");
    Serial1.print(newPos);
    Serial1.print(" dir:");
    Serial1.println((int)(encoder1->getDirection()));
    pos = newPos;
  } // if

  digitalWrite(LED_PIN, HIGH); //High means led is off
  if(lastRadioReset == 0){
    lastRadioReset = millis();
    #ifdef DEBUG
        radio.printDetails();   // DEBUG
    #endif
  }

  // Send an invitation to the edtracker slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromEdTrackerToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    DEBUG_PRINTLN("Failed Ed tracker");   // DEBUG
    radio.startListening();
    if(lastRadioReset + 1000 < millis()){
      radioBegin();
      lastRadioReset = millis();
    }
  }else{
    // DEBUG_PRINT("Ed tracker ");   // DEBUG
    readSlaveResponseAndUpdateJoystick();
  }

  delay(5);

  // Send an invitation to the rudder slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromRudderToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    radio.startListening();
   DEBUG_PRINTLN(F("failed rudder"));   // DEBUG
  }else{
  //  DEBUG_PRINT(F("rudder   "));   // DEBUG
    readSlaveResponseAndUpdateJoystick();
  }
  delay(5);

  // Send an invitation to the throttle slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromThrottleToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    radio.startListening();
    DEBUG_PRINTLN(F("failed throttle"));   // DEBUG
  }else{
    // DEBUG_PRINT(F("throttle "));   // DEBUG
    readSlaveResponseAndUpdateJoystick();
  }
  Joystick.sendState();

  delay(5);
}

void readSlaveResponseAndUpdateJoystick(){
  // DEBUG_PRINT(F("readSlaveResponseAndUpdateJoystick "));   // DEBUG

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
    // DEBUG_PRINTLN(F("\t----- timed out."));   // DEBUG
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
  // EncoderDirection ed = deduceEncoderClick(encoder1);
  // Joystick.setButton(12 + ed.colckwise, ed.buttonValue);
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
  encoder1->tick();
}
EncoderDirection deduceEncoderClick(RotaryEncoder encoder) {
  EncoderDirection ed;
  int16_t position = encoder.getPosition();
  DEBUG_PRINT(F("Encoder position: "));   // DEBUG
  DEBUG_PRINTLN(position);   // DEBUG
  // encoder.setPosition(0);
  if (position == 0) {
    ed.buttonValue = 0;
    ed.colckwise = 0;
  } else if ( position > 0 ) {
    ed.buttonValue = 1;
    ed.colckwise = 1;
  } else {
    ed.buttonValue = 1;
    ed.colckwise = 0;
  }
  return ed;
}