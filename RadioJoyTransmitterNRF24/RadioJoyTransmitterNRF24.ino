/**
 * This sketch is for Arduino Pro Mini 8MHz 3.3V 
 * It reads the ADXL345 sensor data (tilt of the joystick) and sends
 * the tilt data to the receiver arduino.
 * The receiver arduino (Micro Pro 16Mhz 5V) is connected to the USB
 * 
 */

#include <RF24.h>
#include <SparkFun_ADXL345.h>
#include "RadioJoy.h"

ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

RF24 radio(7, 8); // CE and CS pins used for NRF24L01 SPI connection

class Axis
{
  public:
    float x;
    // boundaries manually captured values minX = -100, maxX = 64
    // range = 164, half range = 82
    float const half_range = 82.00;
    float minX = -100.00;// manually captured value
    float maxX = 64.00;// manually captured value

    void calibrate(int newX){
      x = (float)newX;
//      if(x < minX) minX = x;
//      if(x > maxX) maxX = x; 
      // the above boundaries calculation was commented out to prevent
      // boundary expansion due to jolts and the below value limiting was added
      if(x < minX){
//Serial.print("minX=");
//Serial.print(x);
//Serial.print(", maxX=");
//Serial.println(maxX);
        x = minX;
      }
      if(x > maxX){
//Serial.print("minX=");
//Serial.print(minX);
//Serial.print(", maxX=");
//Serial.println(x);
        x = maxX; 
      }

    };
    int calculatePosition(void){
      float fraction = ((x - minX)/(maxX - minX));
      int pos = fraction * 255;
      return pos;
    };
    void lowPassFilter(float alpha = 0.5){
      oldX = x * alpha + (oldX * (1.0 - alpha));
      x = oldX;
    };
  
  private:
    // values for the low pass filter
    float oldX = 0;
//    float alpha = 0.5;
};

Axis rudder;


void setup()
{
  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(2);            // Give the range settings
  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' 
                                      // or 3 wire SPI mode when set to 1 Default: Set to 1
  findAxisCentre();
  
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(102);
  radio.openWritingPipe(RECEIVER_ADDRESS);
  radio.openReadingPipe(1,TRANSMITTER_ADDRESS);

  // Start the radio listening for data
  radio.startListening();

//  batteryCheck();
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
    if (fromRudderToReceiver == request) {
      // if the request was for the rudder data
      // read the data from the sensors
      RadioJoystick joystick;
      joystick.fromToByte = fromRudderToReceiver;
      joystick.axisRudder = readAxisData();

//Serial.print(", filteredX=");
//Serial.print((int)rudder.x);
//Serial.print(", buf.rudder=");
//Serial.println(joystick.axisRudder); 

      delay(2); // this delay is to allow the receiver to prepare for our transmission
      radio.stopListening();                                    // First, stop listening so we can talk.
      if (!radio.write( &joystick, sizeof(joystick) )){ // This will block until complete
        //Serial.println(F("failed"));
      }
    }
    else
    {
//      Serial.println("request is not recognised");
    }
  }
}

int16_t readAxisData(){
  // Get the Accelerometer Readings
  int x,y,z;                          // init variables hold results
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  rudder.calibrate(z);

  rudder.lowPassFilter();

  return (int16_t)rudder.calculatePosition();
}


/**
 * assuming the rudder is kept centred for 5 seconds, while led is blinking
 */
void findAxisCentre(){
  unsigned long started_find_centre_at = millis();
  boolean blinkState;
  unsigned long lastMillis = 0;

  unsigned long time_to_centre = 5000; // we'll spend 5 sec to find the centrepoint
  int16_t oldCentre = 0;
  

  while ( millis() < started_find_centre_at + time_to_centre ){
    int LED_PIN = 13;
    pinMode(LED_PIN, OUTPUT);
    
    // Get the Accelerometer Readings
    int x,y,z;                          // init variables hold results
    adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
    rudder.x = (float) z;
    rudder.lowPassFilter(0.1);          // more wight to the old value

    unsigned long nowMillis = millis();
    if (nowMillis > lastMillis ) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  
      lastMillis = nowMillis + 100;
    }
    delay(50);
  }

  float centre = rudder.x;
  rudder.minX = centre - rudder.half_range;
  rudder.maxX = centre + rudder.half_range;
}

//void batteryCheck(){
//  int LED_BATTERY_PIN = 2;
//  int SENSOR_PIN = A0;
//  int CHARGED_BATTERY = 512;
//  
//  int sensorValue = analogRead(SENSOR_PIN);
//
//  if (sensorValue < CHARGED_BATTERY){
//    pinMode(LED_BATTERY_PIN, OUTPUT);
//    for (int i=sensorValue; i<1023; i=i+100){
//      digitalWrite(LED_BATTERY_PIN, HIGH);    // turn the LED on (HIGH is the voltage level)
//      delay(1000);                            // wait for a second
//      digitalWrite(LED_BATTERY_PIN, LOW);     // turn the LED off by making the voltage LOW
//      delay(1000);                            // wait for a second
//    }
//  }
//}



