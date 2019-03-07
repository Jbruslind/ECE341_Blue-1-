#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


#include <WiFi.h>
#include <PubSubClient.h>

#include <Wire.h>
#include "Countimer.h"

#define BUILTIN_LED 7

/************************************************
 * Bluetooth Stuff
 */

/************************************************
 * Wifi Control Stuff
 */

const char* ssid = "Bill_nye_the_wifi"; //Setup info about the wifi
const char* password = ")";

const char* mqtt_server = "192.168.0.41"; //Set ip of MQTT broker 

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

#define CSNS_ADDR = 0x61 //Address to talk to I2C slave (ACS71020)

int csns_2 = 34; //Pin to read current data from plug 2
int csns_3 = 35; //Pin to read current data from plug 1

int relay_1 = 25; //Pin to set relay 1 state (off = HIGH, on = LOW)
int relay_2 = 26; //Pin to set relay 2 state (see above)
int relay_3 = 27; //Pin to set relay 3 state (see above)

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(csns_2, INPUT);
  pinMode(csns_3, INPUT);

  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  /*
   * Structure for the below class call is as follows: starting Hour, minute, second, 
   * object behavior (count up or down), and what function to call when timer is done 
   * (with any specific arguments)
   */
  plug_one.setCounter(0,0,0,plug_one.COUNT_DOWN,timer_complete); 
  plug_two.setCounter(0,0,0,plug_one.COUNT_DOWN,timer_complete);
  plug_three.setCounter(0,0,0,plug_one.COUNT_DOWN,timer_complete);

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
      Data = String("!|") + /*Voltage here + */ String("|") /*+ Csns_1 here*/ + analogRead(csns_2) + String("|") + analogRead(csns_3)
      + String("|") + plug_one.getCurrentTime() + String("|") + plug_one.getCurrentTime() + String("|")
      + plug_one.getCurrentTime() + String("|?");  //Append all the data to one string
  
      if(client.connected()) //only publish if we're actually connected
      {
        client.publish("/Switch/Data", Data.c_str()); //Publish heartbeat of data
      }

    client.loop();
  }

}

/*
 * What to do when the timer is done counting down. 
 * Inputs: Timer number that has expired
 * Outputs: none
 * Behavior: Will take timer number and use that to turn off the respective relay, 
 * and set the respective timer to 0. 
 */
void timer_complete()
{
  
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
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * The action taken when a message is recieved
 * Inputs: The topic that the message is apart of (char array), the payload delievered (byte array), and how long the payload is (int)
 * Outputs: none
 * Behavior: This function will parse a standard message that should be sent from the webclient/broker. It should take the form of !|Timer1|Timer2|Timer3|? where the timers are all
 * formatted as HH:MM:SS (HH, MM, and SS should all be ints) 
 */
void callback(char* topic, byte* payload, unsigned int length) {
  if(length > 0)
  {
    if(payload[0] == '!') //error checking to make sure message is properly formatted
    {
      plug_one.setCounter(atoi(payload[2]), atoi(payload[4]), atoi(payload[6])); //Selectively take out the values needed 
    }
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


