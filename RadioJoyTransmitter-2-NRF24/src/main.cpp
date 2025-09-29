/**
 * This sketch is for Arduino Pro Mini 8MHz 3.3V 
 * It reads the analog voltage input ftom joystick pots and sends
 * the data to the receiver arduino.
 * The receiver arduino (Micro Pro 16Mhz 5V) is connected to the USB
 * 
 */

#include <Arduino.h>
#include <RF24.h>
#include "RadioJoy.h"

// #define DEBUG

#ifdef DEBUG
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif

void checkButton(int pin, int button);
int16_t scanButtons();

RF24 radio(7, 8); // CE and CS pins used for NRF24L01 SPI connection

int16_t previousA0 = 0;
int16_t previousA1 = 0;

struct Button {
  unsigned long startDebounce = 0;
  int state = HIGH;
};

#define NUMBER_BUTTONS 4
Button buttons[NUMBER_BUTTONS];
const int DEBOUNCE = 10;

RadioJoystick joystick;

void setup()
{
  
  DEBUG_BEGIN(115200);

  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(102);
  radio.openWritingPipe(RECEIVER_ADDRESS);
  radio.openReadingPipe(1,TRANSMITTER_ADDRESS);

  // Start the radio listening for data
  radio.startListening();

  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);
  pinMode(A3,INPUT_PULLUP);
}

void loop()
{
  // let's wait for the server's invitation so sen our data
  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = millis();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  
  while ( ! radio.available() ){                             // While nothing is received
    if (millis() - started_waiting_at > 500 ){            // If waited longer than 20ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }      
  }
      
  if ( timeout ){                                             // Describe the results
    DEBUG_PRINTLN(F("Failed, response timed out."));
  }else{
    uint8_t request = 0;
    radio.read( &request, sizeof(uint8_t) );
    if (fromJoystickToReceiver == request) {
      // if the request was for the throttle data
      // read the data from the sensors
      delay(2); //this delay is to allow the receiver to prepare for this transmission

      radio.stopListening();                                    // First, stop listening so we can talk.
      if (!radio.write( &joystick, sizeof(joystick) )){ // This will block until complete
        DEBUG_PRINTLN(F("failed"));
      }

      // the previous value of the joystick record has just been sent, let's calculate a new one
      joystick.fromToByte = fromJoystickToReceiver;
      int16_t a0 = analogRead(A0);
      joystick.axisY = (a0 + previousA0) >> 1;
      previousA0 = a0;
      int16_t a1 = analogRead(A1);
      joystick.axisX = (a1 + previousA1) >> 1;
      previousA1 = a1;
      joystick.buttons = scanButtons();

      // Let's make the magnetic field sensor readings somewhat linear
      // Xcentre=606, k1=0.84=511/606, k2=1.23=511/(1023-606)
      int Xcentre=592; float kx1=0.86/* =511/592 */, kx2=1.19;/* =511/(1023-592) */
      if(joystick.axisX < Xcentre){
        joystick.axisX = (int)(joystick.axisX * kx1);
      }else{
        joystick.axisX = (int)(511 + (joystick.axisX - Xcentre) * kx2 );
      }
      int Ycentre=433; float ky1=1.18/* =511/433 */, ky2=0.87;/* =511/(1023-433) */
      if(joystick.axisY < Ycentre){
        joystick.axisY = (int)(joystick.axisY * ky1);
      }else{
        joystick.axisY = (int)(511 + (joystick.axisY - Ycentre) * ky2 );
      }
      DEBUG_PRINT("X=");
      DEBUG_PRINT(joystick.axisX); 
      DEBUG_PRINT(", Y=");
      DEBUG_PRINT(joystick.axisY);
      DEBUG_PRINT(", \tButtons=");
      DEBUG_PRINT(buttons[0].state);
      DEBUG_PRINT(buttons[1].state);
      DEBUG_PRINT(buttons[2].state);
      DEBUG_PRINTLN(buttons[3].state);
    }
  }
}

int16_t scanButtons()
{
  checkButton(3, 0);
  checkButton(A2, 1);
  checkButton(A3, 2);
  checkButton(4, 3);

  int16_t result = 0;
  int16_t currentBit;
  for(int i = 15; i >= 0; i--){
    if(i < NUMBER_BUTTONS && buttons[i].state == LOW){
      currentBit = 1;
    }else{
      currentBit = 0;
    }
    result = result | currentBit << i;
  }
  return result;
}

void checkButton(int pin, int button){
  int currentState;

  if (buttons[button].startDebounce != 0) {
    if(buttons[button].startDebounce + DEBOUNCE > millis()){
      // debounce time has not lapsed yet... exit
      return;
    }else{
      // debounce time has lapsed... reset the start time and continue
      buttons[button].startDebounce = 0;
    }
  }
  
  currentState = digitalRead(pin);
  
  if (buttons[button].state == currentState) {
    // the button state has not changed... exit
    return;
  }else{
    // the button state has changed... let's capture the new state and start the debounce process
    buttons[button].startDebounce = millis();
    buttons[button].state = currentState;
  }
}

