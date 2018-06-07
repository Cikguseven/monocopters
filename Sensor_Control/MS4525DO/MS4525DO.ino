//#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <i2c_t3.h>

File myFile;
unsigned long time;
const int chipSelect = 254;
//int address1 = 72;
int pin_SDA = 17; 
int pin_SCL = 16;
#define sensor_address 0x28
#define K_CONSTANT 0.0001450376808

void setup(){

  Wire.begin();
Wire.beginTransmission((int)sensor_address);
Wire.write(0);  
 Wire.endTransmission();
 Wire.setSDA(pin_SDA);
 Wire.setSCL(pin_SCL);
Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_INT, I2C_RATE_400);
 //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17);
 //Wire.begin();
//>>>>>>> Stashed changes
  Serial.begin(9600);
  SD.begin(chipSelect);
  Serial.println("Starting");
  time = millis();
}

void get_sensor_data(byte *a,byte *b,byte *c,byte *d){
//<<<<<<< Updated upstream
 
  Wire.requestFrom((int)sensor_address, (int) 4);
//=======
  //Wire.beginTransmission(sensor_address);
  //Wire.endTransmission();
  Wire.requestFrom((int)sensor_address,(int)4);
//>>>>>>> Stashed changes
  *a = Wire.read();
  *b = Wire.read();
  *c = Wire.read();
  *d = Wire.read();
}

void compute_sensor_data(){
  byte aa,bb,cc,dd;
  get_sensor_data(&aa,&bb,&cc,&dd);
  
  long unsigned combinedPressure = aa, combinedTemperature = cc;
  
  double Temperature,Pressure;
  
  combinedPressure &= 0x3F;
  combinedPressure = combinedPressure << 8;
  combinedPressure |= bb;
  combinedTemperature = combinedTemperature << 8;
  combinedTemperature |= dd;
  combinedTemperature = combinedTemperature >> 5;
  
  
  Serial.print("byte: ");Serial.print(bb,BIN);
  Serial.print("|");Serial.print(bb,BIN);
  Serial.print("|");Serial.print(cc,BIN);
  Serial.print("|");Serial.println(dd,BIN);
  Serial.print("Comined Pressure: ");Serial.println(combinedPressure,BIN);
  Serial.print("Combined Temperature: ");Serial.println(combinedTemperature,BIN);
  
  compute_temperature(combinedTemperature,&Temperature);
  compute_pressure(combinedPressure,&Pressure);
  compute_air_speed(Temperature, Pressure);
  //Serial.println("--");
  ;
}

void compute_temperature(long unsigned tlong, double *temperature){
  *temperature = (double) tlong * 200.0 / 2047.0 - 50; // Degrees C
  //Serial.print("Temperature (degrees C): ");Serial.println(*temperature);
  *temperature += 273.15; // Degrees K
}

void compute_pressure(long unsigned plong, double *pressure){
  double Pr = ( (double) plong - 0.05*16383.0 ) / (0.9*16383);
  double Pd = abs(1 - 2*Pr);
  
  //Serial.print("Pressure (difference): ");Serial.println(Pd,DEC);
  *pressure = Pd * 6894.76; // Pascalaa
}

void compute_air_speed(double T,double P){ 
  double airDensity = 101352.9 / (287.058 * T);
  double adjustedP = P*K_CONSTANT;
  double air_speed = sqrt(2*(adjustedP) / airDensity);
  //Serial.print("Air Speed (ms-1): ");
  Serial.println(air_speed,DEC);
  //writeSD(air_speed);

}

void loop(){
  Serial.println("Looping...");
  compute_sensor_data();
  
  delay(1000);
}
 
void writeSD(String data){
  myFile = SD.open("Airspeed.csv",FILE_WRITE);
    if (myFile) { 
    // read from the file until there's nothing else in it:
    myFile.print((int)time);
    myFile.print(',');
    myFile.println(data);
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
