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
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String txValue;
int relay_3 = 18; //Pin to set relay 3 state (see above) 
int relay_1 = 25; //Pin to set relay 1 state (off = HIGH, on = LOW)
int relay_2 = 26; //Pin to set relay 2 state (see above)

/*Bluetooth stuff
 */

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


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
//Countimer plug_one;  //Countdown timer for plug one
//Countimer plug_two;   //Countdown timer for plug two
//Countimer plug_three; //Countdown timer for plug three

int plug_one = 0;
int plug_two = 0;
int plug_three = 0;
int time_one = 0;
int time_two = 0;
int time_three = 0;

int csns_2 = 34; //Pin to read current data from plug 2
int csns_3 = 35; //Pin to read current data from plug 1

float cur_1 = 0;
float cur_2 = 0;


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

#define SENSOR_BIAS_ALPHA 1850                                // zero current bias for 1st current sensor
#define SENSOR_BIAS_BRAVO 1850                                // zero current bias for 2nd current sensor

const int num_readings_too = 25;                           // number of elements in shift register
const int input_pin_alpha = 34;                               // input pin for 1st channel of microcontroller
const int input_pin_bravo = 35;                                // input pin for 2nd channel of microcontroller

float readings_alpha[num_readings_too];                           // shift register that holds last n current readings for sensor 1, where n = num_readings_too
float readings_bravo[num_readings_too];                            // shift register that holds last n current readings for sensor 2, where n = num_readings_too
float current_alpha = 0.0;
float current_bravo = 0.0;                                          // calculated current from sensors
float max_val_1 = 0;
float max_val_2 = 0;
int inputVal_alpha = 0;                                             // raw input from sensors
int inputVal_bravo = 0;
int bias_alpha = 0;                                                 // input from sensors adjusted for zero-current bias
int bias_bravo = 0;

long stop_time = 0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        //Serial.println("*********");
        //Serial.print("Received Value: ");

        /*for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();
        */
        // Do stuff based on the command receivplued from the app
        if (rxValue.find("SYS_ON") != -1) { 
          //Serial.print("Turning ON!");
          plug_one = 3.6*pow(10,6);
          plug_two = 3.6*pow(10,6);
          plug_three = 3.6*pow(10,6);
        }
        else if (rxValue.find("SYS_OFF") != -1) {
          //Serial.print("Turning OFF!");
          plug_one = 0;
          plug_two = 0;
          plug_three = 0;
        }
        
        else if (rxValue.find("A") != -1) { 
          //Serial.print("Turning ON!");
          plug_one = 3.6*pow(10,6);
        }
        else if (rxValue.find("B") != -1) {
          //Serial.print("Turning OFF!");
          plug_one = 0;
        }
        
        if (rxValue.find("C") != -1) { 
          //Serial.print("Turning ON!");
          plug_two = 3.6*pow(10,6);
        }
        else if (rxValue.find("D") != -1) {
          //Serial.print("Turning OFF!");
          plug_two = 0;
        }
        
        if (rxValue.find("E") != -1) { 
          //Serial.print("Turning ON!");
          plug_three = 3.6*pow(10,6);
        }
        else if (rxValue.find("F") != -1) {
          //Serial.print("Turning OFF!");
          plug_three = 0;
        }
        //Serial.println();
        //Serial.println("*********");
      }
      start_timers();
    }
    
};

void setup() {
  Serial.begin(115200);
  
 // Create the BLE Device
  BLEDevice::init("BLUE1"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);                     // set clock rate [400kHz is max for ASC71020]
  pinMode(csns_2, INPUT);
  pinMode(csns_3, INPUT);

  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
    
  for (int i = 0; i < num_readings_too; i++) {                    // initialize all readings to 0
    readings_alpha[i] = 0;                        
    readings_bravo[i] = 0;
  } 
  
  for (int i = 0; i < numReadings; i++) {    // initialize all readings to 0
    vol_readings[i] = 0;
    cur_readings[i] = 0;                        
  }  
  /*
   * Structure for the below class call is as follows: starting Hour, minute, second, 
   * object behavior (count up or down), and what function to call when timer is done 
   * (with any specific arguments)
   */
  //plug_three.setCounter(0,0,0,plug_three.COUNT_DOWN,timer_complete_3);
  //plug_one.setCounter(0,0,0,plug_one.COUNT_DOWN,timer_complete_1); 
  //plug_two.setCounter(0,0,0,plug_two.COUNT_DOWN,timer_complete_2);


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
//Bluetooth stuff


  
  //plug_one.run();
  //plug_two.run();
  //plug_three.run();
  check_time();
  get_sensor();
  Data = String("!|") + max_vol + String("|") + max_cur + String("|") + A_current_1() + String("|") + A_current_2() + String("|") 
  + get_time(time_one, plug_one) + String("|") + get_time(time_two, plug_two) + String("|")+ get_time(time_three, plug_three) + String("|?");  //Append all the data to one string
  Serial.println(Data);
  if(millis() - stop_time > 500)
  {
    stop_time = millis();
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
          //long now = millis();
          //if (now - lastReconnectAttempt > 500) 
          //{
            //lastReconnectAttempt = now;
       client.publish("/Switch/Data", Data.c_str()); //Publish heartbeat of data        
            //}
        }
      //delay(300);
      client.loop();
    }
    ///delay(300);
  }
  
}
void check_time()
{
  if(millis() - time_one > plug_one)
  {
    timer_complete_1();
  }
  if(millis() - time_two > plug_two)
  {
    timer_complete_2();
  }
  if(millis() - time_three > plug_three)
  {
    timer_complete_3();
  }
}

String get_time(int time_to, int plug)
{
  time_to = plug - (millis() - time_to); 
  if(time_to <= 0)
  {
    return "00:00:00";
  }
  String format = "";
  int hours = time_to/(3.6*pow(10,6));
  time_to = time_to - hours*3.6*pow(10,6); 
  if(hours < 10)
  {
    format = "0" + String(hours) + ":"; 
  }
  else
  {
    format = String(hours) + ":";
  }
  int minutes = time_to/60000;
  time_to = time_to - minutes*60000;
  if(minutes < 10)
  {
    format = format + "0" + String(minutes) + ":"; 
  }
  else
  {
    format = format + String(minutes) + ":";
  }
  int seconds = time_to/1000;
  if(seconds < 10)
  {
    format = format + "0" + String(seconds);
  }
  else
  {
    format = format + String(seconds);
  }
  return format;
}
/*
 * What to do when the timer is done counting down. 
 * Inputs: none
 * Outputs: none
 * Behavior: Will check which timer expirelay_2 and set that timer's respective plug to OFF. Had to make 3 
 * seperate functions because the timer will reset back to it's max value once complete (which would be unknown). So 
 * by making 3 functions, whichever one gets called, we'll know exactly which timer completed
 */
void timer_complete_1()
{
  digitalWrite(relay_1, HIGH);
  plug_one = 0;
}

void timer_complete_2()
{
  digitalWrite(relay_2, HIGH);
  plug_two = 0;
}

void timer_complete_3()
{
  digitalWrite(relay_3, HIGH);
  plug_three = 0;
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
  if(length == 30)
  {
    for(int i = 2; i < 28; i++)
    {
       payload[i] = payload[i] - 48;
    }
    if(payload[0] == '!' && payload[29] == '?') //error checking to make sure message is properly formatted
    {
     plug_one = ((payload[2]*10 + payload[3])*3.6*(pow(10,6))) + ((payload[5]*10 + payload[6])*60000) + ((payload[8]*10 + payload[9])*1000); //Selectively take out the values needed 
     plug_two = ((payload[11]*10 + payload[12])*3.6*(pow(10,6)))+ ((payload[14]*10 + payload[15])*60000) + ((payload[17]*10 + payload[18])*1000); //Selectively take out the values needed
     plug_three = ((payload[20]*10 + payload[21])*3.6*(pow(10,6))) + ((payload[23]*10 + payload[24])*60000) + ((payload[26]*10 + payload[27])*1000); //Selectively take out the values needed
     start_timers();
    }
  }
}


void start_timers()
{
  if(plug_one != 0)
  {
    digitalWrite(relay_1, LOW);
    time_one = millis();
  }
    if(plug_two != 0)
  {
    digitalWrite(relay_2, LOW);
    time_two = millis();
  }
    if(plug_three != 0)
  { 
    digitalWrite(relay_3, LOW);
    time_three = millis();
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

  Wire.endTransmission();                        // stop signal

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
  max_vol -=  0.19825;                                                           // apply zero-voltage bias
  if(plug_one == 0) max_cur = 0.00;
  max_cur *= (30.0/float(pow(2.0, 0.5)));                                       // scale current to size and take rms
  max_vol *= (215.5/float(pow(2.0, 0.5)));                                      // scale voltage to size and take rms
}

float A_current_1()
{
  if(plug_two == 0) return 0.00;
  
  inputVal_alpha = abs(analogRead(input_pin_alpha));                     // read raw input from sensor
  
  for(int i = 1; i < num_readings_too; i++){                                                   // finds max value and shifts register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
     if(i == 1){
        max_val_1 = readings_alpha[num_readings_too - i];
     }
     if (readings_alpha[num_readings_too - i] > max_val_1){                                        // gets new max current if higher value is found
        max_val_1 = readings_alpha[num_readings_too - i];
     }   
     
     readings_alpha[num_readings_too - i] = readings_alpha[num_readings_too - (i + 1)];                 // shifts values to the right
  }
  readings_alpha[0] = inputVal_alpha;   
  
  bias_alpha = abs(max_val_1 - (SENSOR_BIAS_ALPHA));                      // adjust reading for zero-current bias 
  current_alpha = float(bias_alpha)/(165.0);                                     // calculate current from sensor using known component sensitivity
  current_alpha /= float(pow(2.0,0.5));
  current_alpha -= 0.09;

  return current_alpha;

  //delay(10);
}

float A_current_2()
{
  if(plug_three == 0) return 0.00;
  
  inputVal_bravo = abs(analogRead(input_pin_bravo));                     // read raw input from sensor
  
  for(int i = 1; i < num_readings_too; i++){                                                   // finds max value and shifts register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
     if(i == 1){
        max_val_2 = readings_bravo[num_readings_too - i];
     }
     if (readings_bravo[num_readings_too - i] > max_val_2){                                        // gets new max current if higher value is found
        max_val_2 = readings_bravo[num_readings_too - i];
     }   
     
     readings_bravo[num_readings_too - i] = readings_bravo[num_readings_too - (i + 1)];                 // shifts values to the right
  }
  readings_bravo[0] = inputVal_bravo;   
  
  bias_bravo = abs(max_val_2 - (SENSOR_BIAS_BRAVO));                      // adjust reading for zero-current bias 
  current_bravo = float(bias_bravo)/(165.0);                                     // calculate current from sensor using known component sensitivity
  current_bravo /= float(pow(2.0,0.5));
  current_bravo -= 0.09;

  return current_bravo;
  //delay(10);                                                  // introduce delay to allow for settle time/stability   
}
