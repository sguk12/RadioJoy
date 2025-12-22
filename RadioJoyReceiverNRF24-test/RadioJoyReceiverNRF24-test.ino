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

// #define DEBUG

#ifdef DEBUG
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINT_BIN(x)  Serial.print((uint16_t)x, BIN)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT_BIN(x)
#endif

#define LED_PIN 17

boolean blinkState;

unsigned long lastRadioReset = 0;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, 16, 0,
  true, true, false, true, true, false,
  true, true, false, false, false);


RF24 radio(9, 10);// radio(9, 8) Arduino's pins connected to CE,CS pins on NRF24L01

void setup()
{
  Joystick.setXAxisRange(0, 1023); //default axis min..max is 0..1023
  Joystick.setYAxisRange(0, 1023); //default axis min..max is 0..1023
  Joystick.setRxAxisRange(0, 1023); //default axis min..max is 0..1023
  Joystick.setRyAxisRange(0, 1023); //default axis min..max is 0..1023
  Joystick.setThrottleRange(0, 1023); //default axis min..max is 0..1023
  Joystick.setRudderRange(0, 1023); //default axis min..max is 0..1023
  Joystick.begin(false);

  DEBUG_BEGIN(115200);   // DEBUG

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
  DEBUG_PRINTLN("Loop");   // DEBUG
  digitalWrite(LED_PIN, LOW);

  if(lastRadioReset == 0){
    lastRadioReset = millis();
    #ifdef DEBUG
        radio.printDetails();   // DEBUG
    #endif
  }

  // Send an invitation to the throttle slave
  radio.stopListening();                                    // First, stop listening so we can talk.
  if (!radio.write( &fromThrottleToReceiver, sizeof(int8_t), 1 )){ // This will block until complete
    radio.startListening();
    DEBUG_PRINTLN(F("failed throttle"));   // DEBUG
  }else{
    readSlaveResponseAndUpdateJoystick();
  }
  Joystick.sendState();
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
        Joystick.setRudder(buf.axisRudder);
        DEBUG_PRINT("propellor:\t");
        DEBUG_PRINTLN(buf.axisPropellor);
        DEBUG_PRINT("trim:\t\t");
        DEBUG_PRINTLN(buf.axisTrim);
        DEBUG_PRINT("rudder:\t\t");
        DEBUG_PRINTLN(buf.axisRudder);
        for(int16_t i=0; i < 16; i++){
          // assign the i-th bit of the buf.buttons
          Joystick.setButton(i, bitRead(buf.buttons, i));
        }
        blink(10);
      } else {
        DEBUG_PRINT(F("Message is not from rudder or not for me: "));   // DEBUG
        DEBUG_PRINTLN(buf.fromToByte);   // DEBUG
      }
    }while(radio.available()); // read all the data from FIFO
  }
}

/*******************************************************************************************************
* Blink LED function
********************************************************************************************************/
void blink(int delayInterval)
{
  digitalWrite(LED_PIN, HIGH);
  delay(delayInterval);
  digitalWrite(LED_PIN, LOW); //High means led is off
}
