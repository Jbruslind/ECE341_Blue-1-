# ECE341 Final Project - AC Bluetooth Switch
Final project for ECE 341 (Junior Design II). Made by Jorian Bruslind, Mack Hall, and Zach Bendt. 
This project uses an ESP32 microcontroller to facilitate all the necessary functions: Relay control, 
sensor measurement, relay control, and bluetooth/wifi connections. 
#Requirements for project

##Base Engineering requirements
The project's base requirements, standard for all bluetooth switch teams

1. The system will turn off and on at least 2 independent household lamps with up to 100W incandescent 
bulbs and report current watts delivered by each channel.
2. 9 out of 10 users will be able to turn the switches off and on from a mobile phone in less 
than 10 seconds without any training or having previously seen the interface.
3. The system will use only US standard plugins for connecting to external devices and will not 
allow any object with a diameter greater than 1mm to enter the enclosure, and will be disable if 
more than 5A is drawn from the wall power.
4. The system must turn off the output after a time of upto 1 hour ± 1 minute when enabled.
5. The system will be able to accept commands from a mobile phone over 20 feet away from the plug-ins.

##Specialized requirements
The project also allowed for teams to specify their own 2 unique requirements

1. The system will have WiFi capabilities 10m or more away from a standard household router and 
will have a web interface that is accessible from a laptop/browser. 
2. The system must be able to accept voice commands through a mobile phone using the Google Now 
API for all functionality. 


 
