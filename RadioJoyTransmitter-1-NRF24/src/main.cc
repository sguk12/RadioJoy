/**
 * This sketch is for Arduino Pro Mini 8MHz 3.3V 
 * It reads the ADXL345 sensor data (tilt of the joystick) and sends
 * the tilt data to the receiver arduino.
 * The receiver arduino (Micro Pro 16Mhz 5V) is connected to the USB
 * 
 */

#include <RF24.h>
#include <Wire.h>
#include "RadioJoy.h"

void findAxisCentre();

RF24 radio(7, 8); // CE and CS pins used for NRF24L01 SPI connection

// moving average of the last AXIS_SAMPLES raw readings
const uint8_t AXIS_SAMPLES = 50;

class Adxl
{
  public:
    // Setup registers
    void initADXL345() {
      Wire.begin();
      Wire.setClock(400000); // this sets the I2C speed to 400KHz, the default is 100KHz
      adxlWrite(0x2D, 0x08); // POWER_CTL: measurement mode
      adxlWrite(0x31, 0x08); // DATA_FORMAT: full resolution, ±2g
      // adxlWrite(0x2C, 0x0B); // BW_RATE: 200 Hz
      adxlWrite(0x2C, 0x0C); // BW_RATE: 400 Hz
      adxlWrite(0x38, 0x9F); // FIFO_CTL: set FIFO Stream mode 10011111
    };

    void populateAxisSamplesFromFifo()
    {
      uint8_t status;
      readFifoStatus(status);
      status &= 0x3F;   // FIFO_STATUS entries = bits [5:0]
      
      fillCount += status;
      if (fillCount > AXIS_SAMPLES) fillCount = AXIS_SAMPLES;

      for(int i=0;i<status;i++)
      {
        int16_t xRaw, yRaw, zRaw;
        if (readRaw(xRaw, yRaw, zRaw))
        {
          axisSampleSum -= axisSamples[axisSampleIndex];
          axisSamples[axisSampleIndex] = zRaw;
          axisSampleSum += zRaw;
          axisSampleIndex = (axisSampleIndex + 1) % AXIS_SAMPLES;
        }
      }
    };

    int16_t getAveragedZValue()
    {
      return axisSampleSum / fillCount;
    }

  private:
    const uint8_t ADXL345_ADDR = 0x53;   // SDO = GND

    uint8_t fillCount = 1;
    int16_t axisSamples[AXIS_SAMPLES] = {0};
    uint8_t axisSampleIndex = 0;
    int32_t axisSampleSum = 0;

    // Write to ADXL345 register
    void adxlWrite(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(reg);
      Wire.write(value);
      Wire.endTransmission();
    };

    // Read 6 bytes starting from DATAX0
    bool readRaw(int16_t &x, int16_t &y, int16_t &z) {
      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(0x32);              // DATAX0
      Wire.endTransmission(false);

      if (Wire.requestFrom(ADXL345_ADDR, (uint8_t)6, (uint8_t)true) < 6) return false;  // bus error → don't read garbage
      x = readWord();
      y = readWord();
      z = readWord();

      return true;
    }

    int16_t readWord()
    {
      uint8_t lo = Wire.read();
      uint8_t hi = Wire.read();
      return (int16_t)(lo | (hi << 8));
    }

    // Read 1 byte starting from FIFO_STATUS
    void readFifoStatus(uint8_t &value) {
      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(0x39);  // FIFO_STATUS
      Wire.endTransmission(false);

      Wire.requestFrom(ADXL345_ADDR, (uint8_t)1, (uint8_t)true);
      value = (uint8_t) Wire.read();
    };

};

Adxl adxl2 = Adxl();             // USE FOR I2C COMMUNICATION

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

  adxl2.initADXL345();
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

  adxl2.populateAxisSamplesFromFifo();

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
    // Serial.println(F("Failed, radio timed out."));
  }else{
    uint8_t request = 0;
    radio.read( &request, sizeof(uint8_t) );
    if (fromRudderToReceiver == request) {
      int16_t axisData = adxl2.getAveragedZValue();

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

    adxl2.populateAxisSamplesFromFifo();
    int16_t z = adxl2.getAveragedZValue();
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

