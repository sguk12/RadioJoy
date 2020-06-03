/**
 * This sketch is for Arduino SS Micro Pro 16Mhz 5V
 * It receives the joystick tilt data via the radio channel and
 * presents it to the USB hub.
 * The transmitter arduino (Pro Mini 8MHz 3.3V) reads the
 * ADXL345 sensor data.
 *
 *
 */

#include <printf.h>
#include <RF24.h>
#include <Joystick.h>
#include "RadioJoy.h"

//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define JOYSTICK_REPORT_ID_MODIFICATOR 2 // this is to make EdTracker report first so it can be debugged in Ubuntu
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define JOYSTICK_REPORT_ID_MODIFICATOR 0
#endif

#define LED_PIN 17

boolean blinkState;
// Timer related so track operations between loop iterations (LED flashing, etc)
unsigned long lastMillis = 0;
int numberOfBlinks = 1;
int currentBlink = 1;
int blinkDelay=200;

unsigned long lastRadioReset = 0;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID + JOYSTICK_REPORT_ID_MODIFICATOR,
  JOYSTICK_TYPE_JOYSTICK, 12, 0,
  true, true, false, true, true, false,
//  true, true, true, true, true, true,
  true, true, false, false, false);
Joystick_ EdTracker(JOYSTICK_DEFAULT_REPORT_ID + 1,
  JOYSTICK_TYPE_JOYSTICK, 0, 0,
  true, true, true, false, false, false,
  false, false, false, false, false);


RF24 radio(9, 10);// radio(9, 8) Arduino's pins connected to CE,CS pins on NRF24L01

void setup()
{
  Joystick.setXAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
  Joystick.setRxAxisRange(0, 1023);
  Joystick.setRyAxisRange(0, 1023);
  Joystick.setThrottleRange(0, 1023);
  Joystick.setRudderRange(0, 255);
  Joystick.setZAxisRange(-32767, 32767);
  Joystick.setRzAxisRange(-32767, 32767);
  Joystick.begin(false);

  EdTracker.setXAxisRange(-32767, 32767);
  EdTracker.setYAxisRange(-32767, 32767);
  EdTracker.setZAxisRange(-32767, 32767);
  EdTracker.begin(false);
#ifdef DEBUG
  Serial.begin(115200);   // DEBUG
  printf_begin();   // DEBUG
#endif

  radioBegin();
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

void loop()
{
  if(lastRadioReset == 0){
    lastRadioReset = millis();
    #ifdef DEBUG
        // while (!Serial);   // DEBUG             // Leonardo: wait for serial monitor
        radio.printDetails();   // DEBUG
    #endif
  }

  if(numberOfBlinks == 1){
    blink3(1);
  }

  // Send an invitation to the edtracker slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromEdTrackerToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    radio.printDetails();
    radio.startListening();
    if(lastRadioReset + 1000 < millis()){
      radioBegin();
      lastRadioReset = millis();
      numberOfBlinks = 9;
    }
  }else{
    readSlaveResponseAndUpdateJoystick();
  }

  EdTracker.sendState();
  delay(5);

  // Send an invitation to the rudder slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromRudderToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    radio.startListening();
   DEBUG_PRINTLN(F("failed rudder"));   // DEBUG
  }else{
    readSlaveResponseAndUpdateJoystick();
  }
  delay(5);

  // Send an invitation to the throttle slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromThrottleToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    radio.startListening();
    DEBUG_PRINTLN(F("failed throttle"));   // DEBUG
  }else{
    readSlaveResponseAndUpdateJoystick();
  }
  Joystick.sendState();

  if(numberOfBlinks > 1){
    blink();
  }else{
    blink3(0);
  }
  DEBUG_PRINTLN(F("blink"));   // DEBUG
  delay(5);
}

void readSlaveResponseAndUpdateJoystick(){
  DEBUG_PRINTLN(F("readSlaveResponseAndUpdateJoystick"));   // DEBUG

  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = millis();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not

  while ( ! radio.available() ){                             // While nothing is received
    if (millis() - started_waiting_at > 20 ){            // If waited longer than 20ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }
  }

  if ( timeout ){
    if (numberOfBlinks == 1){
      numberOfBlinks = 3;
    }
    DEBUG_PRINTLN(F("Failed, response timed out."));   // DEBUG
  }else{
    do{
      RadioJoystick buf;
      radio.read( &buf, sizeof(buf) );
      if (buf.fromToByte == fromRudderToReceiver){
        // Message with a good checksum received.
        Joystick.setRudder(buf.axisRudder);
      } else if (buf.fromToByte == fromThrottleToReceiver) {
        // Message with a good checksum received.
        Joystick.setXAxis(buf.axisX);
        Joystick.setYAxis(buf.axisY);
        Joystick.setThrottle(buf.axisThrottle);
        Joystick.setRxAxis(buf.axisPropellor);
        Joystick.setRyAxis(buf.axisTrim);
        for(int16_t i=0; i < 12; i++){
          // assign the i-th bit of the buf.buttons
          Joystick.setButton(i, bitRead(buf.buttons, i));
        }
        if (buf.buttons){
          numberOfBlinks = 15;
        }else{
          numberOfBlinks = 1;
        }
      } else if (buf.fromToByte == fromEdTrackerToReceiver) {
        // Message with a good checksum received.
        EdTracker.setXAxis(buf.axisX);
        EdTracker.setYAxis(buf.axisY);
        EdTracker.setZAxis(buf.axisRudder);
      } else{
        if (numberOfBlinks == 1){
          numberOfBlinks = 6;
        }
        DEBUG_PRINT(F("Message is not from rudder or not for me: "));   // DEBUG
        DEBUG_PRINTLN(buf.fromToByte);   // DEBUG
      }
    }while(radio.available()); // read all the data from FIFO
  }
}

/*******************************************************************************************************
* Blink LED function
********************************************************************************************************/
void blink()
{
  unsigned long nowMillis = millis();
  if (nowMillis > lastMillis ){
    lastMillis = nowMillis + blinkDelay;
    digitalWrite(LED_PIN, blinkState);

    if(blinkState){
      if(numberOfBlinks > 1){
        currentBlink++;
      }else{
        currentBlink=1;
      }
    }
    if(blinkState && currentBlink > numberOfBlinks){
      lastMillis = nowMillis + blinkDelay * numberOfBlinks*2;
      currentBlink = 1;
    }
    blinkState = !blinkState;
  }
}

void blink2()
{
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void blink3(boolean blinkOnOff)
{
  digitalWrite(LED_PIN, blinkOnOff);
}
