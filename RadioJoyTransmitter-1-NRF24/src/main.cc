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

void findAxisCentre();
int16_t readAxisData();
int16_t axisData = 0;

// moving average of the last AXIS_SAMPLES raw readings
const uint8_t AXIS_SAMPLES = 30;
int16_t axisSamples[AXIS_SAMPLES] = {0};
uint8_t axisSampleIndex = 0;
int32_t axisSampleSum = 0;

int16_t updateAxisAverage(){
  int16_t reading = readAxisData();
  axisSampleSum -= axisSamples[axisSampleIndex];
  axisSamples[axisSampleIndex] = reading;
  axisSampleSum += reading;
  axisSampleIndex = (axisSampleIndex + 1) % AXIS_SAMPLES;
  return axisSampleSum / AXIS_SAMPLES;
}

ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

RF24 radio(7, 8); // CE and CS pins used for NRF24L01 SPI connection

class Axis
{
  public:
    int x;
    // boundaries manually captured values minX = -100, maxX = 64
    // range = 164, half range = 82
    int const half_range = 82;
    int minX = -100;// manually captured value
    int maxX = 64;// manually captured value

    void calibrate(int newX){
      x = newX;
      // the above boundaries calculation was commented out to prevent
      // boundary expansion due to jolts and the below value limiting was added
      if(x < minX){
        x = minX;
      }
      if(x > maxX){
        x = maxX; 
      }
    };

    int calculatePosition(void){
      float fraction = (float)(x - minX) / (maxX - minX);
      int pos = fraction * 255;
      return pos;
    };
  
};

Axis rudder;


void setup()
{
  // Serial.begin(115200);

  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(2);            // Give the range settings
  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' 
                                      // or 3 wire SPI mode when set to 1 Default: Set to 1
  adxl.set_bw(0x0B);                  // sampling rate to 200Hz.
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
}

void loop()
{
  // in loop(), at the top
  // static uint32_t loopCount = 0;
  // static uint32_t lastReport = 0;
  // loopCount++;
  // uint32_t now = millis();
  // if (now - lastReport >= 1000) {
  //   Serial.print(F("loops/s "));   Serial.print(loopCount);
  //   Serial.print(F("  us/loop ")); Serial.println(1000000UL / loopCount);
  //   loopCount = 0;
  //   lastReport = now;
  // }

  axisData = updateAxisAverage(); // rolling average over the last 10 readings

  // let's wait for the server's invitation so sen our data
  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = millis();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  
  while ( ! radio.available() ){                             // While nothing is received
    if (millis() - started_waiting_at > 20 ){            // If waited longer than 20ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }      
  }
      
  axisData = updateAxisAverage(); // rolling average over the last 10 readings

  if ( timeout ){                                             // Describe the results
    // Serial.println(F("Failed, radio timed out."));
  }else{
    uint8_t request = 0;
    radio.read( &request, sizeof(uint8_t) );
    if (fromRudderToReceiver == request) {
      axisData = updateAxisAverage(); // rolling average over the last 10 readings

      // if the request was for the rudder data
      // read the data from the sensors
      RadioJoystick joystick;
      joystick.fromToByte = fromRudderToReceiver;
      rudder.calibrate(axisData);
      joystick.axisRudder = rudder.calculatePosition();
      // Serial.print("Joystick position\t"); Serial.print(joystick.axisRudder);Serial.print("\t\taxisData ");Serial.println(axisData);

      delay(2); // this delay is to allow the receiver to prepare for our transmission
      radio.stopListening();                                    // First, stop listening so we can talk.
      if (!radio.write( &joystick, sizeof(joystick) )){ // This will block until complete
        // Serial.println(F("Failed to send joystick response."));
      }
    }
    else
    {
      // Serial.print("Request is not recognised ");
      // Serial.println(request);
    }
  }

  axisData = updateAxisAverage(); // rolling average over the last 10 readings

}

int16_t readAxisData(){
  // Get the Accelerometer Readings
  int x,y,z;                          // init variables hold results
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
  return z;
}


/**
 * assuming the rudder is kept centred for 5 seconds, while led is blinking
 */
void findAxisCentre(){
  // Serial.println("Finding Axis Centre...");
  unsigned long started_find_centre_at = millis();
  boolean blinkState;
  unsigned long lastMillis = 0;

  unsigned long time_to_centre = 2500; // we'll spend 2.5 sec to find the centrepoint
  int centre = 0;

  while ( millis() < started_find_centre_at + time_to_centre ){
    int LED_PIN = 13;
    pinMode(LED_PIN, OUTPUT);
    
    // Get the Accelerometer Readings
    int x,y,z;                          // init variables hold results
    adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
    centre = (centre + z) >> 1; // low pass filter (kind of)

    unsigned long nowMillis = millis();
    if (nowMillis > lastMillis ) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  
      lastMillis = nowMillis + 100;
    }
    delay(20);
  }

  
  rudder.minX = centre - rudder.half_range;
  rudder.maxX = centre + rudder.half_range;
  // Serial.print("Found Axis Centre "); Serial.println(centre);
  // Serial.print("Rudder min "); Serial.println(rudder.minX);
  // Serial.print("Rudder max "); Serial.println(rudder.maxX);
}

