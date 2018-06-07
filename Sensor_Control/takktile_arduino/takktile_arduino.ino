/**************************************************************************/
/*!
  @file takktile_arduino.ino
  @author Eric Bakan & Yaroslav Tenzer
  @license BSD

  Driver for the TakkTile Strip sensor

  This is a library for the TakkTile Strip sensor
  ----> http://www.takktile.com/product:takkstrip

  @section HISTORY

  v1.0 - First release by Eric Bakan
  v1.1 - Updated for automatic sensor detection
  v1.2 - Updated the code for wrapping related issues
  v1.3 - Updated the code to reduce transmitted data bytes

  @section NOTES

  // Some logic copied from https://github.com/adafruit/Adafruit_MPL115A2/

*/
/**************************************************************************/

// Arduino addressing is using 7bit
// All the addresses were calculated in 8bit
// And were right shifted when actually used

#include <i2c_t3.h>
#include <SPI.h>
#include <SD.h>

File myFile;
const int chipSelect = BUILTIN_SDCARD;
uint32_t logTime;

const uint8_t ANALOG_COUNT = 1; //number of sensors or columns of data collected

int pin_SDA = 18; 
int pin_SCL = 19;
//Definitions for MEMS Barometer----------------------------------------------------------------------------------------------------------------------
#define FILE_BASE_NAME "Data"

#define MAX_STRIPS 8
#define MAX_SENSORS 6

#define NUM_SENSORS MAX_STRIPS*MAX_SENSORS // reserve addresses for 8 strips with 6 sensors on each
#define PRECISION 3

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D

const uint32_t SAMPLE_INTERVAL_MS = 1000; //Logging time interval, in milliseconds. If want to change, take note the timestamp is using in-built micros() function recording time in microseconds


float a0[NUM_SENSORS];
float b1[NUM_SENSORS];
float b2[NUM_SENSORS];
float c12[NUM_SENSORS];

byte addressArray[NUM_SENSORS];
byte addressLength;

float pressureHistory[NUM_SENSORS];
boolean flagHistoryExists = false;

boolean flagShowAddress = false;
boolean flagShowPressure = true;
boolean flagShowTemperature = true;
//--------------------------------------------------------------------------------------------

//specific to barometer
void initialize() {
  // s 0C
  Wire.beginTransmission(SENSOR_ALL_ON >> 1);
  Wire.endTransmission();

  // s C0 12 01
  Wire.beginTransmission(0xC0 >> 1);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();

  // s 0D
  Wire.requestFrom(SENSOR_ALL_ON >> 1, 1);

  delay(5);
}

//specific to barometer
void readCoeffs(byte addressSensor, byte num) {

  // Select sensor
  Wire.beginTransmission(addressSensor >> 1);
  Wire.endTransmission();

  // Request coefficients
  Wire.beginTransmission(FREESCALE_ADDRESS >> 1);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(FREESCALE_ADDRESS >> 1, 8);
  int16_t a0coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b1coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b2coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t c12coeff = (( (uint16_t) (Wire.read() << 8) | Wire.read())) >> 2;

  // Turn sensor off
  Wire.requestFrom(addressSensor >> 1, 1);

  a0[num] = (float)a0coeff / 8;
  b1[num] = (float)b1coeff / 8192;
  b2[num] = (float)b2coeff / 16384;
  c12[num] = (float)c12coeff;
  c12[num] /= 4194304.0;
  //Serial.println(a0[num],15);
  //Serial.println(b1[num],15);
  //Serial.println(b2[num],15);
  //Serial.println(c12[num],15);
}

//Collecting pressure readings from barometer
void readData(byte addressSensor, float* oTemp, float* oPressure)
{
  // Select sensor
  Wire.beginTransmission(addressSensor >> 1);
  Wire.endTransmission();

  // Request P/T data
  Wire.beginTransmission(FREESCALE_ADDRESS >> 1);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  Wire.requestFrom(FREESCALE_ADDRESS >> 1, 4);
  uint16_t pressure = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  uint16_t temp = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;

  // Turn sensor off
  Wire.requestFrom(addressArray[addressSensor] >> 1, 1);

  // ------ Ignore the calibrations for the moment

  float pressureComp = 2118.75 + (-2.457885742187500 + 0.000806570053100 * temp) * pressure + -1.118347167968750 * temp;

  //Calculate temp & pressure
  *oPressure = ((65.0F / 1023.0F) * pressureComp) + 50.05F; // kPa
  *oTemp = ((float) temp - 498.0F) / -5.35F + 25.0F; // C

  // ------

  //*oPressure = pressure;
  //*oTemp = temp;
  Serial.print(*oPressure);
}

//specific to barometer
void checkAddresses()
{
  addressLength = 0;
  int temp_add = 0;
  // check every strip
  for (int strip_n = 0; strip_n < MAX_STRIPS; strip_n++) {
    // check every sensor
    for (int sensor_n = 0; sensor_n < MAX_SENSORS; sensor_n++) {
      temp_add = (strip_n << 4) + sensor_n * 2; // calculate the address

      // check if the Attiny responds with its address
      // this also switches ON the Chip Select line on the desired sensor
      Wire.beginTransmission(temp_add >> 1); // take into account that its 7bit !
      // if response finishes with an ACK then continue
      if (Wire.endTransmission() == 0) {
        // check if there is a sensor on this line
        Wire.beginTransmission(FREESCALE_ADDRESS >> 1);
        // if there is an ACK then there is a sensor
        if (Wire.endTransmission() == 0) {
          addressArray[addressLength] = temp_add;
          addressLength++;
        }
        // Turn off the chip select line
        Wire.requestFrom(temp_add >> 1, 1);
      }
    }
    //Serial.println(']');
  }
}


void setup () {
  Wire.begin();
   Wire.setSDA(pin_SDA);
  Wire.setSCL(pin_SCL);
Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);

  Serial.begin(9600);
  

  checkAddresses(); // check how many sensors are connected

  // for each found sensor, read the coefficients ..(for barometer only)
  for (int i = 0; i < addressLength; i++) {
    readCoeffs(addressArray[i], i);
  }

  // Wait for USB Serial
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization success!");

  // Start on a multiple of the sample interval.
  logTime = micros() / (1000UL * SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL * SAMPLE_INTERVAL_MS;
}


void loop() {


//Change this portion to suit the respective sensor-------------------------------------------- 

  float oTemp = 0;
  float oPressure = 50;
  float p_current = 0;
  float p_history = 0;
  float delta_up = 0;
  float delta_down = 0;

  initialize();

  Serial.print('[');
  for (int i = 0; i < addressLength; i++)
  {
    if (i > 0) {
      Serial.print(',');
    }
    readData(addressArray[i], &oTemp, &oPressure);
    
    if (flagHistoryExists) {
      p_current = oPressure;
      p_history = pressureHistory[i];
      delta_up = p_current - p_history;
      delta_down = p_history - (p_current - 1024);
      if (delta_up < delta_down) {
        oPressure = p_history + delta_up;
      } else {
        oPressure = p_history - delta_down;
      }
    }
    pressureHistory[i] = oPressure;

    // Start output to the serial port

    Serial.print('[');

    Serial.print('[');
    // Print out sensor ID value if the flag was set
    if (flagShowAddress) {
      //Serial.print(addressArray[i],HEX);
    }

    // Print out Pressure values if the flag was set
    if (flagShowPressure) {
      if (flagShowAddress) {
        //  Serial.print(',');
      }
      Serial.print(oPressure, PRECISION);
    }

    // Print out Temperature values if the flag was set
    if (flagShowTemperature) {
      if (flagShowPressure) {

        Serial.print(',');
      }
      Serial.print(oTemp, PRECISION);
    }
    Serial.print(']');
  }
  Serial.println(']');

  // End output to the serial port
  // ------------------------------
  flagHistoryExists = true;

  // Listen to the commands from the serial port
  if (Serial.available()) {
    byte inByte = (byte)
                  Serial.read();
    if (inByte == 'n') {
      flagShowAddress = !flagShowAddress;
    }
    if (inByte == 'p') {
      flagShowPressure = !flagShowPressure;
    }
    if (inByte == 't') {
      flagShowTemperature = !flagShowTemperature;
    }

  }
  
//----------------------------------------------------------------------------------------------------


// Time for next record.
    logTime += 1000UL * SAMPLE_INTERVAL_MS;

    // Wait for log time.
    int32_t diff;
    do {
      diff = micros() - logTime;
    } while (diff < 0);

    // Check for data rate too high.
    if (diff > 10) {
      Serial.println("Missed data record");
    }

    uint16_t data[ANALOG_COUNT];

    // Read all channels to avoid SD write latency between readings.
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
      data[i] = oPressure; //Change oPressure to output value(s)of sensor in use
    }
    // Write data to file.  Start with log time in micros.
    //file.print(logTime);

//    // Write ADC data to CSV record.
//    file.write(',');
//    file.print(data[0]);
//    file.println();
// WRITE TIME,data[0]
    writeFile(logTime/1000000);
    writeFile(",");
    writeFile(data[0]);
    writeFile('\n');
    // the calculations of the wrapping
    // that are used to calculate the minus signs
}

void writeFile(String text){
  myFile = SD.open("Pressure.csv", FILE_WRITE);
  if (myFile) { 
    // read from the file until there's nothing else in it:
    myFile.print(text);
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
