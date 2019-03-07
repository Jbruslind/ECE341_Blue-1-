// Note: 0x20 register holds current and voltage for ASC71020 current sensor
// first 15 bits hold rms voltage value, and next 15 bits hold rms current value

#include <Wire.h>                            // utilizes Arduino Wire library

#define SENSOR 0x61                          // DO NOT CHANGE | Slave address of current/voltage sensor

uint8_t inputs[4];                           // inputs[0] = data7_0, inputs[1] = data15_8, inputs[2] = data23_16, inputs[3] = data31_24
String bits[4];                              // literal binary representation of inputs
String memory_map = "";                      // concatination of literal binary representation of inputs == [data31_24 + data23_16 + data15_8 + data7_0]                   
String voltage = "000000000000000";          // holds 15 voltage bits
String current = "000000000000000";          // holds 15 current bits
float final_voltage = 0;                     // final calculated and scaled voltage
float final_current = 0;                     // final calculated and scaled current
 
void setup() {
  Wire.begin();                              // no address == join as master
  Wire.setClock(400000);                     // set clock rate [400kHz is max for ASC71020]
  Serial.begin(115200);                      // initialize serial communication with computer
}

void loop() {
  Wire.beginTransmission(SENSOR);            // write start signal + slave address + write bit
  Wire.write(0x20);                          // write desired register
  Wire.endTransmission();                    // send singal + write new start signal
  Wire.requestFrom(SENSOR, 4);               // write slave address + read bit (request 4 bytes)
  
  while(Wire.available()){                        // Wire.available() returns number of bytes left to send from slave
    inputs[4 - Wire.available()] = Wire.read();   // read from slave     
  }                                               // inputs[0] = data7_0, inputs[1] = data15_8, inputs[2] = data23_16, inputs[3] = data31_24

  for (int i = 0; i < 4; i++){  
    bits[i] = toBinary(inputs[i]);                // convert int to binary
  }

  Wire.endTransmission(1);                        // stop signal

  for (int j = 3; j >= 0; j--){                   // concatinate inputs into "memory map"
    memory_map += bits[j];
  }

  for (int k = 0; k < 15; k++){                                                           // convert binary to readable, calibrated float values
    current[k] = memory_map[1 + k];                                                       // pluck current reading from memory map
    voltage[k] = memory_map[17 + k];                                                      // pluck voltage reading from voltage map
    final_current += float(int((current[k] - '0'))) * float(pow(2.0, float(-(k+1))));     // find current decimal value sent by sensor
    final_voltage += float(int((voltage[k] - '0'))) * float(pow(2.0, float(-(k+1))));     // find voltage decimal value sent by sensor
                                                                                          // convert char -> int -> float then multiply bit by 2^-(decimal place) to find value to add to decimal
                                                                                          // look up Q decimal binary format for reference
  }

  final_current *= 30.0;                                                                  // scale current 
  final_voltage *= 215.5;                                                                 // scale voltage

  debug(memory_map, voltage, current, final_voltage, final_current, inputs);              // print values to serial plotter
  reset(memory_map, final_voltage, final_current, voltage, current, inputs, bits);        // reset values for next loop
  
  delay(100);                                                                             // delay for stability
}

void debug(const String mem, const String vol, const String cur, const float f_vol, const float f_cur, const uint8_t inputs[4]){
  for (int i = 0; i < 4; i++){                                                                    
      Serial.print("byte ["); Serial.print(i); Serial.print("] dec: "); Serial.print(inputs[i]); Serial.print(" | bin: "); Serial.println(toBinary(inputs[i]));   
  }                                                                                               // print each transmitted byte from sensor
  
  Serial.print("memory: "); Serial.println(mem);                                                  // print memory map
  Serial.print("voltage bits: "); Serial.println(vol);                                            // print equiv. voltage bits
  Serial.print("current bits: "); Serial.println(cur);                                            // print equiv. current bits
  Serial.print("calculated voltage: "); Serial.print(f_vol); Serial.println(" volts");            // print calc. voltage value
  Serial.print("calculated current: "); Serial.print(f_cur); Serial.println(" amps");             // print calc. current value
  Serial.println("");
}

void reset(String &mem_map, float &f_vol, float &f_cur, String &vol, String &cur, uint8_t inputs[4], String bits[4]){
  memory_map = "";                              // reset memory map
  final_voltage = 0;                            // reset calculated voltage
  final_current = 0;                            // reset calcualted current
  voltage = "000000000000000";                  // reset voltage bits
  current = "000000000000000";                  // reset current bits
  for (int i = 0; i < 4; i++){                  // reset all input containers
    inputs[i] = 0;
    bits[i] = "";
  }
}

String toBinary(const uint8_t dec){
  int counter = 0;                              // tracks position of bitmask
  String temp = "00000000";                     // 8 bit container to hold binary result

  for(uint8_t i = 0x80; i != 0; i >>= 1){       // applies shifting bitmask and stores result to temp string
    temp[counter] = (dec & i) ? '1' : '0';      // if (dec & i) != 0, then temp[counter] = 1, else temp[counter] = 0
    counter++;                                  // increments position of bitmask tracker  
  }
  return temp;                                  // final result is 8-char string of literal binary representation of input decimal Ex: 23 --> 00010111
}
