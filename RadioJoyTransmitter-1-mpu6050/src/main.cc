// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v6.12)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-10 - Uses the new version of the DMP Firmware V6.12
//                 - Note: I believe the Teapot demo is broken with this versin as
//                 - the fifo buffer structure has changed
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps612.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <RF24.h>
#include "RadioJoy.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

RF24 radio(7, 8); // CE and CS pins used for NRF24L01 SPI connection

// #define DEBUG

#ifdef DEBUG
  #define OUTPUT_READABLE_YAWPITCHROLL
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LED_BLINK_CYCLES 10 // number of loop cycles the LED stays on/off
bool blinkState = false;
uint16_t blinkCycleCount = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {
  digitalWrite(LED_PIN, HIGH);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  DEBUG_BEGIN(115200);
  
  // initialize device
  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  DEBUG_PRINTLN(F("Testing device connections..."));
  DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  // load and configure the DMP
  DEBUG_PRINTLN(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-8);
  mpu.setYGyroOffset(4);
  mpu.setZGyroOffset(16);
  mpu.setXAccelOffset(-5084);
  mpu.setYAccelOffset(3204);
  mpu.setZAccelOffset(740);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    DEBUG_PRINTLN();
  #ifdef DEBUG
    mpu.PrintActiveOffsets();
  #endif
    // turn on the DMP, now that it's ready
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready!"));
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUG_PRINT(F("DMP Initialization failed (code "));
    DEBUG_PRINT(devStatus);
    DEBUG_PRINTLN(F(")"));
  }
  
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(102);
  radio.openWritingPipe(RECEIVER_ADDRESS);
  radio.openReadingPipe(1,TRANSMITTER_ADDRESS);
  radio.startListening();
  
  // configure LED for output
  // pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}



// convert an axis angle in degrees to a joystick range of 0..255,
// saturating at -12.5 degrees (0) and 12.5 degrees (255)
int16_t calibrate(float axisAngle) {
  const float kMaxAngle = 18.0f;
  const int16_t kMinOutput = 0;
  const int16_t kMaxOutput = 255;
  if (axisAngle <= -kMaxAngle) return kMinOutput;
  if (axisAngle >= kMaxAngle) return kMaxOutput;
  return (int16_t)(kMinOutput + (axisAngle + kMaxAngle) / (2 * kMaxAngle) * (kMaxOutput - kMinOutput));
}

// exponential moving average low-pass filter; caller owns filteredValue so
// each axis can keep its own independent filter state across calls.
// lower alpha = smoother but slower to respond.
// filteredValue starts as NAN so the first call seeds it with newValue
// instead of ramping up from 0.
float filteredAxis1Data = NAN;
float lowPassFilter(float newValue, float &filteredValue) {
  const float alpha = 0.15f;
  if (isnan(filteredValue)) {
    filteredValue = newValue;
  } else {
    filteredValue += alpha * (newValue - filteredValue);
  }
  return filteredValue;
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    DEBUG_PRINT("yaw pitch roll\t");
    DEBUG_PRINT(ypr[0] * 180 / M_PI);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(ypr[1] * 180 / M_PI);
    DEBUG_PRINT("\t");
    DEBUG_PRINT(ypr[2] * 180 / M_PI);
    DEBUG_PRINTLN();
#endif
  float axisData = ypr[2] * 180 / M_PI;

  // low-pass filter to smooth out sensor noise
  axisData = lowPassFilter(axisData, filteredAxis1Data);

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
      
  if ( timeout ){                                             // Describe the results
    DEBUG_PRINTLN(F("Failed, radio timed out."));
  }else{
    uint8_t request = 0;
    radio.read( &request, sizeof(uint8_t) );
    if (fromRudderToReceiver == request) {

      // if the request was for the rudder data
      // read the data from the sensors
      RadioJoystick joystick;
      joystick.fromToByte = fromRudderToReceiver;
      
      joystick.axisRudder = calibrate(axisData);
      DEBUG_PRINT("Joystick position\t"); DEBUG_PRINT(joystick.axisRudder); DEBUG_PRINT("\t\taxisData "); DEBUG_PRINTLN(axisData);

      delay(2); // this delay is to allow the receiver to prepare for our transmission
      radio.stopListening();                                    // First, stop listening so we can talk.
      if (!radio.write( &joystick, sizeof(joystick) )){ // This will block until complete
        DEBUG_PRINTLN(F("Failed to send joystick response."));
      }
    }
    else
    {
      DEBUG_PRINT("Request is not recognised ");
      DEBUG_PRINTLN(request);
    }
  }

    // blink LED to indicate activity: stay on for LED_BLINK_CYCLES cycles,
    // then off for LED_BLINK_CYCLES cycles
    blinkCycleCount++;
    if (blinkCycleCount >= LED_BLINK_CYCLES) {
      blinkCycleCount = 0;
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}