#include <Wire.h>                            // utilizes Arduino Wire library

#define SENSOR 0x61                          // DO NOT CHANGE | Slave address of current/voltage sensor

const int numReadings = 175;                 // number of elements in shift register for "relative" average

uint8_t inputs[4];                           // stores input from chip | index 0 and 1 are voltage, 2 and 3 are current
String bits[4];                              // literal binary representation of inputs
String voltage = "0000000000000000";         // holds 16 voltage bits 
String current = "0000000000000000";         // holds 16 current bits
float cur_readings[numReadings];             // shift register that holds last n current readings, where n = numReadings
float vol_readings[numReadings];             // shift register that holds last n voltage readings, where n = numReadings
float final_voltage = 0;                     // float form of binary voltage bits
float final_current = 0;                     // float form of binary current bits
float max_vol = 0;                           // biased and scaled voltage
float max_cur = 0;                           // biased and scaled current
 
void setup() {
  for (int i = 0; i < numReadings; i++) {    // initialize all readings to 0
    vol_readings[i] = 0;
    cur_readings[i] = 0;                        
  }  
  
  Wire.begin();                              // no address == join as master
  Wire.setClock(400000);                     // set clock rate [400kHz is max for ASC71020]
  Serial.begin(115200);                      // initialize serial communication with computer
}

void loop() {
  Wire.beginTransmission(SENSOR);                 // write start signal + slave address + write bit
  Wire.write(0x2A);                               // write desired register (instantaneous voltage)
  Wire.endTransmission();                         // send singal + write new start signal
  Wire.requestFrom(SENSOR, 2);                    // write slave address + read bit (request 2 bytes) | ignoring sign bit
  
  while(Wire.available()){                        // Wire.available() returns number of bytes left to send from slave
    inputs[2 - Wire.available()] = Wire.read();   // read from slave     
  }                                               
 
  Wire.endTransmission(); 
  Wire.beginTransmission(SENSOR);                 // write start signal + slave address + write bit
  Wire.write(0x2B);                               // write desired register (instantaneous current)
  Wire.endTransmission();                         // send singal + write new start signal
  Wire.requestFrom(SENSOR, 2);                    // write slave address + read bit (request 2 bytes) | ignoring sign bit
  
  while(Wire.available()){                        // Wire.available() returns number of bytes left to send from slave
    inputs[4 - Wire.available()] = Wire.read();   // read from slave     
  }  

  Wire.endTransmission(1);                        // stop signal

  for (int i = 0; i < 4; i++){  
    bits[i] = toBinary(inputs[i]);                // convert int to binary
  }
  
  voltage = bits[1] + bits[0];                    // assemble voltage binary from inputs
  current = bits[3] + bits[2];                    // assemble current binary from inputs

  final_current = 0.0;                            // set decimal equivalent of binary to zero before counting
  final_voltage = 0.0;

  for (int k = 0; k < 16; k++){                                                           // convert binary to readable, calibrated float values
    final_current += float(int((current[k] - '0'))) * float(pow(2.0, float(-(k+0))));     // convert char -> int -> float then multiply bit by 2^-(decimal place) to find value to add to decimal
    final_voltage += float(int((voltage[k] - '0'))) * float(pow(2.0, float(-(k+1))));     // look up Q decimal binary format for reference                                                                                         
  }

  for(int i = 1; i < numReadings; i++){                                                   // finds max value and shifts register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
     if(i == 1){
        max_cur = cur_readings[numReadings - i];                                          // sets max value to oldest value on first shift through
        max_vol = vol_readings[numReadings - i];
     }
     if (cur_readings[numReadings - i] > max_cur){                                        // gets new max current if higher value is found
        max_cur = cur_readings[numReadings - i];
     }
     if (vol_readings[numReadings - i] > max_vol){                                        // gets new max voltage if higher value is found
        max_vol = vol_readings[numReadings - i];
     }     
     
     cur_readings[numReadings - i] = cur_readings[numReadings - (i + 1)];                 // shifts values to the right
     vol_readings[numReadings - i] = vol_readings[numReadings - (i + 1)];
  }
  cur_readings[0] = final_current;                                              // store new value in 0th index || Ex: [X | X | Y] --> [W | X | Y]
  vol_readings[0] = final_voltage;   

  max_cur -= 0.0;                                                               // apply zero-current bias
  max_vol -= 0.19825;                                                           // apply zero-voltage bias
  max_cur *= (30.0/float(pow(2.0, 0.5)));                                       // scale current to size and take rms
  max_vol *= (215.5/float(pow(2.0, 0.5)));                                      // scale voltage to size and take rms

  debug();                                                                      // print values to serial plotter
}

void debug(){
  for (int i = 0; i < 4; i++){                                                                    
      Serial.print("byte ["); Serial.print(i); Serial.print("] dec: "); Serial.print(inputs[i]); Serial.print(" | bin: "); Serial.println(bits[i]);   
  }                                                                                                 // print each transmitted byte from sensor
  
  Serial.print("voltage bits: "); Serial.println(voltage);                                          // print equiv. voltage bits
  Serial.print("current bits: "); Serial.println(current);                                          // print equiv. current bits
  Serial.print("calculated voltage: "); Serial.print(max_vol); Serial.println(" volts");            // print calc. voltage value
  Serial.print("calculated current: "); Serial.print(max_cur); Serial.println(" amps");             // print calc. current value
  Serial.println("");
}

String toBinary(const uint8_t dec){             // returns literal binary equivalent string of decimal input
  int counter = 0;                              // tracks position of bitmask
  String temp = "00000000";                     // 8 bit container to hold binary result

  for(uint8_t i = 0x80; i != 0; i >>= 1){       // applies shifting bitmask and stores result to temp string
    temp[counter] = (dec & i) ? '1' : '0';      // if (dec & i) != 0, then temp[counter] = 1, else temp[counter] = 0
    counter++;                                  // increments position of bitmask tracker  
  }
  return temp;                                  // final result is 8-char string of literal binary representation of input decimal Ex: 23 --> 00010111
}
