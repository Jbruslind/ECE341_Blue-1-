#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


#include <WiFi.h>
#include <PubSubClient.h>

#include <Wire.h>
#include "Countimer.h"
/************************************************
 * Bluetooth Stuff
 */

/************************************************
 * Wifi Control Stuff
 */
 
const char* ssid = "billnyethewifi";
const char* password = "Robot123";

const char* mqtt_server = "192.168.8.101"; //Set ip of MQTT broker 

WiFiClient espClient;  //Initalize Wifi object
PubSubClient client(espClient); //Use wifi object to setup MQTT client
long lastMsg = 0; //Store flag about last message recieved
char msg[50]; //array to store message received
String Data; 
long lastReconnectAttempt = 0;

/************************************************
 * General Stuff
 */
Countimer plug_one;  //Countdown timer for plug one
Countimer plug_two;   //Countdown timer for plug two
Countimer plug_three; //Countdown timer for plug three

int csns_2 = 34; //Pin to read current data from plug 2
int csns_3 = 35; //Pin to read current data from plug 1

float cur_1 = 0;
float cur_2 = 0;

int relay_1 = 25; //Pin to set relay 1 state (off = HIGH, on = LOW)
int relay_2 = 26; //Pin to set relay 2 state (see above)
int relay_3 = 27; //Pin to set relay 3 state (see above)

#define SENSOR 0x61                          // DO NOT CHANGE | Slave address of current/voltage sensor (ACS71020)

const int numReadings_1 = 10;                 // number of elements in shift register

float readings[numReadings_1];                // shift register that holds last n current readings, where n = numReadings
float total = 0;                            // the total of currents in shift register
float average = 0;                          // the average of last n current readings, where n = numReadings
float current_1 = 0.0;                        // calculated current from sensor
int inputVal = 0;                           // raw input from sensor
int bias = 0;                               // input from sensor adjusted for zero-current bias

const int numReadings_2 = 10;                 // number of elements in shift register

float readings_2[numReadings_2];                // shift register that holds last n current readings, where n = numReadings
float total_2 = 0;                            // the total of currents in shift register
float average_2 = 0;                          // the average of last n current readings, where n = numReadings
float current_2 = 0.0;                        // calculated current from sensor
int inputVal_2 = 0;                           // raw input from sensor
int bias_2 = 0;                               // input from sensor adjusted for zero-current bias

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
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);                     // set clock rate [400kHz is max for ASC71020]
  pinMode(csns_2, INPUT);
  pinMode(csns_3, INPUT);
  
  Serial.begin(115200);

  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  
  for (int i = 0; i < numReadings; i++) {    // initialize all readings to 0
    vol_readings[i] = 0;
    cur_readings[i] = 0;                        
  }  
  /*
   * Structure for the below class call is as follows: starting Hour, minute, second, 
   * object behavior (count up or down), and what function to call when timer is done 
   * (with any specific arguments)
   */
  plug_three.setCounter(0,0,0,plug_three.COUNT_DOWN,timer_complete_3);
  plug_one.setCounter(0,0,0,plug_one.COUNT_DOWN,timer_complete_1); 
  plug_two.setCounter(0,0,0,plug_two.COUNT_DOWN,timer_complete_2);


  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

}
/*
 * Basically just a monitor function, will check for any changes in the system and
 * call specific functions accordingly. 
 * Inputs: none
 * Outputs: none
 * Behavior: The only things that aren't on inturrupts are the current sensing, voltage
 * sensing, and heartbeat message. This function should contiuously measure the current
 * and voltage, and then send out a heartbeat message to the MQTT broker/Bluetooth device
 * (if connected) with data about the voltage, current for each plug, timer statuses
 * and any issues (flags in the system). 
 */
void loop() {
  plug_one.run();
  plug_two.run();
  plug_three.run();
  check_time();
  get_sensor();
  Data = String("!|") + max_vol + String("|") + max_cur + String("|") + A_current_1() + String("|") + A_current_2() + String("|") 
  + plug_one.getCurrentTime() + String("|") + plug_two.getCurrentTime() + String("|")+ plug_three.getCurrentTime() + String("|?");  //Append all the data to one string
  Serial.println(Data);
   if (!client.connected()) { //Checks if we're connected to an MQTT broker, if not try
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0; //Keep track of how long we've spend trying (will attempt every >=5 sec) 
      }
    }
  } else { 
    // Client connected
      if(client.connected()) //only publish if we're actually connected
      {
        long now = millis();
        if (now - lastReconnectAttempt > 500) 
        {
          lastReconnectAttempt = now;
          client.publish("/Switch/Data", Data.c_str()); //Publish heartbeat of data        
        }
      }
    client.loop();
  }
}
void check_time()
{
  if(plug_one.getCurrentSeconds() == 0 && plug_one.getCurrentMinutes() == 0 && plug_one.getCurrentHours() == 0)
  {
    timer_complete_1();
  }
  if(plug_two.getCurrentSeconds() == 0 && plug_two.getCurrentMinutes() == 0 && plug_two.getCurrentHours() == 0)
  {
    timer_complete_2();
  }
  if(plug_three.getCurrentSeconds() == 0 && plug_three.getCurrentMinutes() == 0 && plug_three.getCurrentHours() == 0)
  {
    timer_complete_3();
  }
}
/*
 * What to do when the timer is done counting down. 
 * Inputs: none
 * Outputs: none
 * Behavior: Will check which timer expired and set that timer's respective plug to OFF. Had to make 3 
 * seperate functions because the timer will reset back to it's max value once complete (which would be unknown). So 
 * by making 3 functions, whichever one gets called, we'll know exactly which timer completed
 */
void timer_complete_1()
{
  digitalWrite(relay_1, HIGH);
  plug_one.setCounter(0,0,0);
  plug_one.stop();
}

void timer_complete_2()
{
  digitalWrite(relay_2, HIGH);
  plug_two.setCounter(0,0,0);  
  plug_two.stop();
}

void timer_complete_3()
{
  digitalWrite(relay_3, HIGH);
  plug_three.setCounter(0,0,0); 
  plug_three.stop(); 
}

/*
 * Funciton to connect to a WIFI signal 
 * Inputs: none (global variables should be setup before: ssid, password
 * Outputs: none
 * Behavior: Will attempt to connect to a WIFI signal using the wifi.h library. Will try for 
 */
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }

  randomSeed(micros());

  //Serial.println("");
  //Serial.println("WiFi connected");
  //Serial.println("IP address: ");
  //Serial.println(WiFi.localIP());
}

/*
 * The action taken when a message is recieved
 * Inputs: The topic that the message is apart of (char array), the payload delievered (byte array), and how long the payload is (int)
 * Outputs: none
 * Behavior: This function will parse a standard message that should be sent from the webclient/broker. It should take the form of !|Timer1|Timer2|Timer3|? where the timers are all
 * formatted as HH:MM:SS (HH, MM, and SS should all be bytes with the numbers as chars) 
 */
void callback(char* topic, byte* payload, unsigned int length) {
  if(length = 30)
  {
    for(int i = 2; i < 28; i++)
    {
       payload[i] = payload[i] - 48;
    }
    if(payload[0] == '!' && payload[29] == '?') //error checking to make sure message is properly formatted
    {
     plug_one.setCounter(payload[2]*10 + payload[3], payload[5]*10 + payload[6], payload[8]*10 + payload[9]); //Selectively take out the values needed 
     plug_two.setCounter(payload[11]*10 + payload[12], payload[14]*10 + payload[15], payload[17]*10 + payload[18]); //Selectively take out the values needed
     plug_three.setCounter(payload[20]*10 + payload[21], payload[23]*10 + payload[24], payload[26]*10 + payload[27]); //Selectively take out the values needed
     start_timers();
    }
  }
}


void start_timers()
{
  if(plug_one.getCurrentSeconds() != 0 || plug_one.getCurrentMinutes() != 0 || plug_one.getCurrentHours() != 0)
  {
    digitalWrite(relay_1, LOW);
    plug_one.start();
  }
    if(plug_two.getCurrentSeconds() != 0 || plug_two.getCurrentMinutes() != 0 || plug_two.getCurrentHours() != 0)
  {
    digitalWrite(relay_2, LOW);
    plug_two.start();
  }
    if(plug_three.getCurrentSeconds() != 0 || plug_three.getCurrentMinutes() != 0 || plug_three.getCurrentHours() != 0)
  { 
    digitalWrite(relay_3, LOW);
    plug_three.start();
  }
}
/*
 * Simple function to reconnect the client to the broker
 * Inputs: none
 * Outputs: Boolean, if connected (TRUE) or not (FALSE)
 * Behavior: Will try to connect to the specified broker. If able to, will publish an opening statement in the /Init/ topic
 */

boolean reconnect() {
  if (client.connect("BTSwitch")) { //What it will be called to the broker
    // Once connected, publish an announcement...
    client.publish("Init","hello world"); //Opening statement on topic /Init/ with message
    // ... and resubscribe
    client.subscribe("Commands"); //Subscribe to this specific topic for messages from the web-client
  }
  return client.connected(); //Return if connected or not
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


void get_sensor()
{
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
}

float A_current_1()
{
    for(int i = 1; i < numReadings_1; i++){     // shift register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
      readings[numReadings_1 - i] = readings[numReadings_1 - (i + 1)];
  }

  inputVal = analogRead(34);          // read raw input from sensor
  bias = inputVal - (1850);                 // adjust reading for zero-current bias 
  current_1 = bias/(165.0);                   // calculate current from sensor using known component sensitivity
  readings[0] = current_1;                    // store new value in 0th index || Ex: [X | X | Y] --> [W | X | Y]
     
  for (int i = 0; i < numReadings_1; i++){    // sum recent currents within shift register
    total += readings[i];
  }
  
  average = total/numReadings_1;              // find average value within shift register
  total = 0;  
  return average;
}

float A_current_2()
{
    for(int i = 1; i < numReadings_2; i++){     // shift register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
      readings_2[numReadings_2 - i] = readings[numReadings_2 - (i + 1)];
  }

  inputVal_2 = analogRead(35);          // read raw input from sensor
  bias_2 = inputVal_2 - (1850);                 // adjust reading for zero-current bias 
  current_2 = bias_2/(165.0);                   // calculate current from sensor using known component sensitivity
  readings_2[0] = current_2;                    // store new value in 0th index || Ex: [X | X | Y] --> [W | X | Y]
     
  for (int i = 0; i < numReadings_2; i++){    // sum recent currents within shift register
    total_2 += readings_2[i];
  }
  
  average_2 = total_2/numReadings_2;              // find average value within shift register
  total_2 = 0;  
  return average_2;
}

