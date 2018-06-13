//#include <Wire.h>
#include <IRremote.h>
#include <SPI.h>
#include <SD.h>
#include <i2c_t3.h>
const int ledPin = 13;
int IR_IN_PIN = 11;
IRrecv irrecv(IR_IN_PIN);
decode_results results;

File myFile;
const int chipSelect = 254;
char fileName[50] = "";
int fileint = 0;
//int address1 = 72;
int pin_SDA = 17; 
int pin_SCL = 16;
#define sensor_address 0x28
#define K_CONSTANT 0.0001450376808
bool writeToSD = false;

void setup(){
  
  Wire.begin();
  Wire.beginTransmission((int)sensor_address);
  Wire.write(0);  
  Wire.endTransmission();
  Wire.setSDA(pin_SDA);
  Wire.setSCL(pin_SCL);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_INT, I2C_RATE_400);
  pinMode(ledPin, OUTPUT);
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17);
  //Wire.begin();
  //>>>>>>> Stashed changes
  Serial.begin(9600);
  irrecv.enableIRIn();
  SD.begin(chipSelect);
  Serial.println("Starting");

}

void get_sensor_data(byte *a,byte *b,byte *c,byte *d){
  Wire.requestFrom((int)sensor_address, (int) 4);
  *a = Wire.read();
  *b = Wire.read();
  *c = Wire.read();
  *d = Wire.read();
}

void compute_sensor_data(){
  byte aa,bb,cc,dd;
  get_sensor_data(&aa,&bb,&cc,&dd);
  
  long unsigned combinedPressure = aa, combinedTemperature = cc;
  
  double Temperature,Pressure,Air_Speed;
  
  combinedPressure &= 0x3F;
  combinedPressure = combinedPressure << 8;
  combinedPressure |= bb;
  combinedTemperature = combinedTemperature << 8;
  combinedTemperature |= dd;
  combinedTemperature = combinedTemperature >> 5;
  
  
  //Serial.print("byte: ");Serial.print(bb,BIN);
  //Serial.print("|");Serial.print(bb,BIN);
  //Serial.print("|");Serial.print(cc,BIN);
  //Serial.print("|");Serial.println(dd,BIN);
  //Serial.print("Comined Pressure: ");Serial.println(combinedPressure,BIN);
  //Serial.print("Combined Temperature: ");Serial.println(combinedTemperature,BIN);
  
  compute_temperature(combinedTemperature,&Temperature);
  compute_pressure(combinedPressure,&Pressure);
  compute_air_speed(Temperature, Pressure,&Air_Speed);
  //Serial.println("--");
  if(writeToSD){
    writeSD(millis()/1000);
    writeSD(',');
    writeSD(Air_Speed*1000000);
    writeSD(',');
    writeSD(Pressure*1000000);
    writeSD(',');
    writeSD(Temperature*1000000);
    writeSD('\n');
  }
}

void compute_temperature(long unsigned tlong, double *temperature){
  *temperature = (double) tlong * 200.0 / 2047.0 - 50; // Degrees C
  *temperature += 273.15; // Degrees K
}

void compute_pressure(long unsigned plong, double *pressure){
  double Pr = ( (double) plong - 0.05*16383.0 ) / (0.9*16383);
  double Pd = abs(1 - 2*Pr);
  
  //Serial.print("Pressure (difference): ");Serial.println(Pd,DEC);
  *pressure = Pd * 6894.76; // Pascalaa
}

void compute_air_speed(double T,double P,double *Air_Speed){ 
  double airDensity = 101352.9 / (287.058 * T);
  double adjustedP = P*K_CONSTANT;
  *Air_Speed = sqrt(2*(adjustedP) / airDensity);
  //Serial.print("Air Speed (ms-1): ");
  //Serial.println(air_speed,DEC);

}

void loop(){
  compute_sensor_data();
  ir_loop();
  delay(100);
}
 
void writeSD(String data){
  myFile = SD.open(fileName,FILE_WRITE);
    if (myFile) { 
    // read from the file until there's nothing else in it:

    myFile.print(data);
  
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening SD card");
  }
}

void ir_loop() {
  if (irrecv.decode(&results)) {
    //Serial.println(results.value);
    if(results.value == 2065 || results.value == 3772782313){
      writeToSD = (writeToSD) ? false:true;
      Serial.println(writeToSD);

      if(writeToSD){

        String file_name = "IMU" + String(fileint);
        file_name += ".csv";
        file_name.toCharArray(fileName,50);
        while(SD.exists(fileName)){
          fileint ++;
          file_name = "IMU" + String(fileint);
          file_name += ".csv";
          Serial.println(file_name);
          memset(fileName, 0, sizeof(fileName ));
          file_name.toCharArray(fileName,50);
        }
        digitalWrite(ledPin, HIGH);
      }else{
        digitalWrite(ledPin, LOW);
      }

      delay(1000);
    }
    irrecv.resume();
  }
}
