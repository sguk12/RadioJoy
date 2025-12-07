/**
 * This sketch is for Arduino Pro Mini 8MHz 3.3V 
 * It reads the analog voltage input ftom joystick pots and sends
 * the data to the receiver arduino.
 * The receiver arduino (Micro Pro 16Mhz 5V) is connected to the USB
 * 
 */

#include <Arduino.h>
#include <RF24.h>
#include <Wire.h>
#include "RadioJoy.h"

#define DEBUG

#ifdef DEBUG
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINT_BITS(x)  Serial.print((uint16_t)x, BIN)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT_BITS(x)
#endif

#define SIX_AXIS

#define PCF8574_IN 0X20

int16_t scanButtonMatrix();

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
  
  Wire.begin(); 
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(102);
  radio.openWritingPipe(RECEIVER_ADDRESS);
  radio.openReadingPipe(1,TRANSMITTER_ADDRESS);

  // Start the radio listening for data
  radio.startListening();

  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);

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

  joystick.buttons = scanButtonMatrix();
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
    DEBUG_PRINTLN(request);

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
//        Serial.println("request is not recognised");
    }
  }
}

int16_t scanButtonMatrix() {
  uint8_t inputStates1 = 0xFF;
  uint8_t inputStates2 = 0xFF;
  uint16_t result = 0xFF;

  digitalWrite(7, LOW);
  Wire.requestFrom(PCF8574_IN, 1); // request from device PCF8574_IN one byte
  if (Wire.available()){
    inputStates1 = Wire.read(); // reads one byte 
    inputStates1 ^= (uint8_t)0b00000011; // invert two bits with XOR with 0b11000000
  }
  else {
    DEBUG_PRINTLN("WIRE not available");
  }
  digitalWrite(7, HIGH);

  digitalWrite(8, LOW);
  Wire.requestFrom(PCF8574_IN, 1); // request from device PCF8574_IN one byte
  if (Wire.available()){
    inputStates2 = Wire.read(); // reads one byte  
    inputStates2 ^= (uint8_t)0b00000011; // invert two bits with XOR with 0b11000000
  }
  else {
    DEBUG_PRINTLN("WIRE not available");
  }
  digitalWrite(8, HIGH);
  // Combine: second read in upper byte, first read in lower byte
  result = ((uint16_t)inputStates2 << 8) | inputStates1;

  DEBUG_PRINT_BITS(result);
  DEBUG_PRINTLN();

  return result;
}
