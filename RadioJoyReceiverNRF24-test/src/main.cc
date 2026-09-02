/**
 * This sketch is for Arduino Micro/Leonardo??
 * It receives the joystick tilt data via the radio channel and
 * presents it to the USB hub.
 */
#include <Arduino.h>
#include <RF24.h>
#include <Joystick.h>
#include "RadioJoy.h"

// #define DEBUG

#ifdef DEBUG
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
  // the below is to disable the every loop printouts and use the targeted printouts with DEBUG2_PRINT()
  // #define DEBUG_PRINTLN(x)
  // #define DEBUG_PRINT(x)
  #define DEBUG2_PRINTLN(x)  Serial.println(x)
  #define DEBUG2_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG2_PRINT(x)
  #define DEBUG2_PRINTLN(x)
#endif

#define LED_PIN 17
#define NUMBER_OF_JOYSTICK_BUTTONS 4

// functions declarations
void radioBegin(void);
void blink(int);
void sendInvitationTo(const char*, int8_t);
void readSlaveResponseAndUpdateJoystick(void);

unsigned long lastRadioReset = 0;
int16_t previousRudderTrim = 0;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK, NUMBER_OF_JOYSTICK_BUTTONS, 0,
  true, true, false, true, true, false,
  true, true, false, false, false);


RF24 radio(9, 10); // radio(9, 10) Arduino's pins connected to CE,CS pins on NRF24L01

void setup()
{
  DEBUG_BEGIN(115200);
  pinMode(LED_PIN, OUTPUT);
  
  Joystick.setRudderRange(0, 255); //default axis min..max is 0..1023
  Joystick.begin(false);
  
  radioBegin();
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
  sendInvitationTo((char*)"joystick", fromJoystickToReceiver);
  sendInvitationTo((char*)"throttle", fromThrottleToReceiver);

  Joystick.sendState();
  
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
