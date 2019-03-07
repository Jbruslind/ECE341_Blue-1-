//Note: 0x20 register holds current and voltage for ASC71020 current sensor
//first 15 bits hold rms voltage value, and next 15 bits hold rms current value

#include <Wire.h>

#define SENSOR 0x61 //DO NOT CHANGE

int address_alpha = 96;      // slave address for sensor 1 in decimal (hex --> decimal)
int address_beta = 0;       // slave address for sensor 2 in decimal (hex --> decimal)
int VI_reg = 32;            // register 0x20 (32 in dec.) holds voltage and current values for ASC71020 
int sclk = 22;               // pin of clock for ESP32
int sda = 21;               // pin of data for ESP32

long Din = 80;

String alpha = "";
String beta = "";
  
int vol_one = 0;                  // integer representation of voltage in sensor 1          
int cur_one = 0;                  // integer representation of current in sensor 1  
int vol_two = 0;                  // integer representation of voltage in sensor 2  
int cur_two = 0;                  // integer representation of current in sensor 2  

void setup() {
  Wire.begin();                             // no address == join as master
  Serial.begin(115200);                     // initialize serial communication with computer
}

void loop() {

  Wire.beginTransmission(SENSOR);
  Wire.write(0x20);
  Wire.beginTransmission(SENSOR);
  Wire.write(SENSOR);
  Wire.endTransmission();
  Wire.requestFrom(SENSOR, 4);
  if(Wire.available())
  {
    Serial.print("Wire Read");
    Din = Wire.read();
  }

  /*
  //Get voltage/current from sensor 2;
  Wire.beginTransmission(address_beta);    // device address is specified in datasheet
  Wire.write(byte(0x20));                  // register 0x20 holds voltage and current values for ASC71020 
  Wire.requestFrom(address_beta, 4);       // requests 1 byte from address
  beta = Wire.read();                      // reads in 1 byte from sensor
  Wire.endTransmission();                  // stop transmitting
  */


  
  Serial.println(Din);                  // print average current to serial monitor
  /*
  //convert binary values to int
  vol_one = int(vol_alpha);
  cur_one = int(cur_alpha);
  vol_two = int(vol_beta);
  cur_two = int(cur_beta);
  */
  
  delay(100);

}
