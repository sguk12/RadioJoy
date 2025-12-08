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
  #define DEBUG_PRINT_BIN(x)  Serial.print(x, BIN)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT_BIN(x)
#endif

#define SIX_AXIS

void checkButton(int pin, int button);
int16_t scanButtons();

int16_t previousAxisThrottle = 0;
int16_t previousAxisPropellor = 0;
int16_t previousAxisTrim = 0;
int16_t previousAxisRudder = 0;
int16_t previousAxisX = 0;
int16_t previousAxisY = 0;


RF24 radio(9, 10); // CE and CS pins used for NRF24L01 SPI connection

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

  pinMode(0, INPUT_PULLUP); // this is RXI, which is used in the Serial
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);

}

void loop()
{
  // read the data from the sensors
  RadioJoystick joystick;
  joystick.fromToByte = fromThrottleToReceiver;
  
#ifdef SIX_AXIS
  int16_t axisThrottle = analogRead(A7);
  joystick.axisThrottle = 1023 - ((axisThrottle + previousAxisThrottle) >> 1); // low pass filter, axis inversion
  previousAxisThrottle = axisThrottle;

  int16_t axisPropellor = analogRead(A3);
  joystick.axisPropellor = 1023 - ((axisPropellor + previousAxisPropellor) >> 1); // low pass filter, axis inversion
  previousAxisPropellor = axisPropellor;

  int16_t axisTrim = analogRead(A1);
  joystick.axisTrim = 1023 - ((axisTrim + previousAxisTrim) >> 1); // low pass filter, axis inversion
  previousAxisTrim = axisTrim;

  int16_t axisRudder = analogRead(A6);
  joystick.axisRudder = 1023 - ((axisRudder + previousAxisRudder) >> 1); // low pass filter, axis inversion
  previousAxisRudder = axisRudder;

  int16_t axisX = analogRead(A2);
  joystick.axisX = 1023 - ((axisX + previousAxisX) >> 1); // low pass filter, axis inversion
  previousAxisX = axisX;

  int16_t axisY = analogRead(A0);
  joystick.axisY = 1023 - ((axisY + previousAxisY) >> 1); // low pass filter, axis inversion
  previousAxisY = axisY;

  joystick.buttons = scanButtons();
#else
  int16_t axisThrottle = analogRead(A0);
  joystick.axisThrottle = (axisThrottle + previousAxisThrottle) >> 1; // low pass filter
  previousAxisThrottle = axisThrottle;

  int16_t axisPropellor = analogRead(A1);
  joystick.axisPropellor = (axisPropellor + previousAxisPropellor) >> 1; // low pass filter
  previousAxisPropellor = axisPropellor;

  int16_t axisTrim = analogRead(A2);
  joystick.axisTrim = (axisTrim + previousAxisTrim) >> 1; // low pass filter
  previousAxisTrim = axisTrim;

  joystick.axisRudder = 0;
  joystick.axisX = 0;
  joystick.axisY = 0;
  joystick.buttons = 0;
#endif

// let's wait for the server's invitation so sen our data
  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = millis();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  
  while ( ! radio.available() ){                             // While nothing is received
    if (millis() - started_waiting_at > 500 ){               // If waited longer than 0.5 sec, indicate timeout and exit while loop
        timeout = true;
        break;
    }      
  }
      
  if ( timeout ){                                             // Describe the results
    DEBUG_PRINTLN(F("Failed, response timed out."));
  }else{
    uint8_t request = 0;
    radio.read( &request, sizeof(uint8_t) );

    DEBUG_PRINT("got request: ");
    DEBUG_PRINTLN(request[0]);

    if (fromThrottleToReceiver == request) {
      // if the request was for the throttle data
      DEBUG_PRINT(", Throttle=");
      DEBUG_PRINT(joystick.axisThrottle); 
      DEBUG_PRINT(", Propellor=");
      DEBUG_PRINT(joystick.axisPropellor); 
      DEBUG_PRINT(", Trim=");
      DEBUG_PRINTLN(joystick.axisTrim); 

      delay(2); //this delay is to allow the receiver to prepare for this transmission

      radio.stopListening();                                    // First, stop listening so we can talk.
      if (!radio.write( &joystick, sizeof(joystick) )){ // This will block until complete
        DEBUG_PRINTLN(F("failed"));
      }
    }else{
      // DEBUG_PRINTLN("request is not recognised");
    }
  }
}

struct Button {
  unsigned long startDebounce = 0;
  int state = LOW; 
};

Button buttons[16];
const int DEBOUNCE = 10;

int16_t scanButtons()
{
  // scan the first row
  digitalWrite(A4, LOW);
  delay(1);
  checkButton(0, 0);
  checkButton(2, 1);
  checkButton(3, 2);
  checkButton(4, 3);
  checkButton(5, 4);
  checkButton(6, 5);
  checkButton(7, 6);
  checkButton(8, 7);
  digitalWrite(A4, HIGH);
  
  // scan the second row
  digitalWrite(A5, LOW);
  delay(1);
  checkButton(0, 8);
  checkButton(2, 9);
  checkButton(3, 10);
  checkButton(4, 11);
  checkButton(5, 12);
  checkButton(6, 13);
  checkButton(7, 14);
  checkButton(8, 15);
  digitalWrite(A5, HIGH);

  int16_t result = 0;
  int16_t currentBit;
  for(int i = 15; i >= 0; i--){
    if (i == 6 || i == 7 || i == 14 || i == 15) { // these are the normally closed buttons
      if(buttons[i].state == HIGH){ // the normally closed buttons (push to break the connection)
        currentBit = 0;
      }else{
        currentBit = 1;
      }
    } else {
      if(buttons[i].state == HIGH){ // the rotary switches and normally open buttons
        currentBit = 1;
      }else{
        currentBit = 0;
      }
    }
    result = result | currentBit << i;

    DEBUG_PRINT("buttons: ");
    DEBUG_PRINT_BIN(result);
    DEBUG_PRINTLN(" ");
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
