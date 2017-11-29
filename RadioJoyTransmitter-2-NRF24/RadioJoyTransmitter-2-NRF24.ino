/**
 * This sketch is for Arduino Pro Mini 8MHz 3.3V 
 * It reads the analog voltage input ftom joystick pots and sends
 * the data to the receiver arduino.
 * The receiver arduino (Micro Pro 16Mhz 5V) is connected to the USB
 * 
 */

#include <RF24.h>
#include "RadioJoy.h"

RF24 radio(7, 8); // CE and CS pins used for NRF24L01 SPI connection

void setup()
{
  
//  Serial.begin(9600);   // Debugging only
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(102);
  radio.openWritingPipe(RECEIVER_ADDRESS);
  radio.openReadingPipe(1,TRANSMITTER_ADDRESS);

  // Start the radio listening for data
  radio.startListening();

  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(A4,INPUT_PULLUP);
  pinMode(A5,INPUT_PULLUP);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

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
    //Serial.println(F("Failed, response timed out."));
  }else{
    uint8_t request = 0;
    radio.read( &request, sizeof(uint8_t) );
//Serial.print("got request: ");
//Serial.println(request[0]);
    if (fromThrottleToReceiver == request) {
      // if the request was for the throttle data
      // read the data from the sensors
      RadioJoystick joystick;
      joystick.fromToByte = fromThrottleToReceiver;
      joystick.axisX = analogRead(A7);
      joystick.axisY = analogRead(A6);
      joystick.axisThrottle = analogRead(A2);
      joystick.axisPropellor = analogRead(A0);
      joystick.axisTrim = analogRead(A1);
      joystick.buttons = scanButtons();
//Serial.print("X=");
//Serial.print((int)joystick.axisX);
//Serial.print(", y=");
//Serial.println(joystick.axisY); 
//Serial.print(", Throttle=");
//Serial.print(joystick.axisThrottle); 
//Serial.print(", Propellor=");
//Serial.print(joystick.axisPropellor); 
//Serial.print(", Trim=");
//Serial.println(joystick.axisTrim); 

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
//Serial.print("X out =");
//Serial.println((int)joystick.axisX);
      delay(2); //this delay is to allow the receiver to prepare for this transmission

      radio.stopListening();                                    // First, stop listening so we can talk.
      if (!radio.write( &joystick, sizeof(joystick) )){ // This will block until complete
        //Serial.println(F("failed"));
      }
    }else{
//        Serial.println("request is not recognised");
    }
  }
}

struct Button {
  unsigned long startDebounce = 0;
  int state = HIGH;
};

Button buttons[16];
const int DEBOUNCE = 10;

int16_t scanButtons()
{
  // scan the first row
  digitalWrite(2, LOW);
  delay(1);
  checkButton(A4, 0);
  checkButton(A5, 1);
  checkButton(5, 2);
  checkButton(6, 3);
  digitalWrite(2, HIGH);
  
  // scan the second row
  digitalWrite(3, LOW);
  delay(1);
  checkButton(A4, 4);
  checkButton(A5, 5);
  checkButton(5, 6);
  checkButton(6, 7);
  digitalWrite(3, HIGH);

  // scan the third row
  digitalWrite(4, LOW);
  delay(1);
  checkButton(A4, 8);
  checkButton(A5, 9);
  checkButton(5, 10);
  checkButton(6, 11);
  digitalWrite(4, HIGH);

  int16_t result = 0;
  int16_t currentBit;
  for(int i = 15; i >= 0; i--){
    if(buttons[i].state == LOW){
      currentBit = 1;
    }else{
      currentBit = 0;
    }
    result = result | currentBit << i;
//Serial.print("button[");
//Serial.print(i);
//Serial.print("]=");
//Serial.print(buttons[i].state);
//Serial.print("  ");
  }
//  printBinaryUnsignedInt(result);
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

//void printBinaryUnsignedInt(unsigned int value)
//{
//    for (unsigned int mask = 0x8000; mask; mask >>= 1)
//    {
//        Serial.print((mask & value) ? '1' : '0');
//    }
//    Serial.println();
//}


