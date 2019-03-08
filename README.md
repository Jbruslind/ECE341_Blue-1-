# ECE341 Final Project - AC Bluetooth Switch
Final project for ECE 341 (Junior Design II). Made by Jorian Bruslind, Mack Hall, and Zach Bendt. 
This project uses an ESP32 microcontroller to facilitate all the necessary functions: Relay control, 
sensor measurement, relay control, and bluetooth/wifi connections. 
#Requirements for project

##	Base Engineering requirements
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

##	Specialized requirements
The project also allowed for teams to specify their own 2 unique requirements

1. The system will have WiFi capabilities 10m or more away from a standard household router and 
will have a web interface that is accessible from a laptop/browser. 
2. The system must be able to accept voice commands through a mobile phone using the Google Now 
API for all functionality. 

## System Overview
The overall project was broken up into many different "blocks" that could be individually assigned
and constructed. 

![Alt text](Jbruslind/ECE341_Blue-1-/blob/master/Block Diagram/HighLevelBlockDiagramV3.png)

1. Fuse - Find a fuse (or circuit equivalent) that is able to break only at 5A within ~.5sec. 
This should be resettable and can be inserted easily into a PCB

2. Transformer - This block is supposed to take in 120Vac (US mains power) and convert it
down to 5V at ~2A. 

3. Voltage Sensor - This block will measure 120Vac and output some measurable signal. 
This could be a radiometric voltage (0-5V) or through some serial protocol.

4. Relay - This block should be a electrically controlled switch (a relay), that can 
take in some voltage signal from 0 - 5V and switch a voltage line which will be 120Vac @ 0 - 5A

5. Current Sensor - This block will take in the individual 120Vac lines (3 total)
and output a signal that corresponds to the current flowing through said lines. This
signal can be radiometric or serial.

6. AC plug - This block will output the shared 120Vac line (after it goes through each relay)
and have accessible ports to 120Vac (line), 0V (neutral), and earth ground. 

7. Smartphone app - This will be a piece of software that can be run on a typical
android OS version (4.0+) and can interface with the ESP32 controller. It must be
"user friendly" in that someone with no exposure to the device prior will be able
to navigate it. This must also use the Google Now API for voice activation/control

8. Wifi Control (not pictured) - This must be some piece of software that allows
for control of the ESP32 microcontroller through a "user friendly" interface. There
are little restrictions to how this can be done other than it must use Wifi and be 
easy to navigate. 




 
