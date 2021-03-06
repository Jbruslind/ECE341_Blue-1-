#	Schematic files
This folder contains all the schematic files as pdfs for easy viewing. The project
was designed using the Altium Circuitmaker software. All project files are open 
for anyone to use and can be found in the project search engine within Circuitmaker. 
##	MCU
![MCU](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/MCU.png)

This is the ESP32 general connection page which provides ports to the other 
necessary sheets within the project. The ESP needs a 3.3V VDD and all its pins
are 3.3V tolerant (not 5V). The inputs are for the I2C bus, 2 radiometric current sensors, 
3 relay control lines, UART programming, and for power/gnd.

The BJTs are used for programming the ESP over USB (seen below in the UART->TTL bridge).
The switches on EN and IO0 are used for programming and resetting respectively.

##	UART Bridge
![UART Bridge](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/UART_Bridge.png)
 
 The ESP32 does not have a native way to communicate over UART (USB signals), so a UART-TTL bridge
 was chosen to perform that job. The Silabs CP2012 is a popular choice among other microcontroller
 development boards and has a proven design. This chip will take in the USB signals from some external
 controller and program the ESP32. 
 
 ##	3.3V LDO
 
 ![3.3V LDO](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/3_3_LDO.png)
 
 The input voltage that we are using for this board is expected to be 4.5 - 5.5V. The ESP32 and 
 the other peripheral sensors cannot tolerate this voltage normally. So a Low Dropout Regulator
 was chosen to step this 5V voltage down to 3.3V. This was chosen to be the LM1117 which is, again,
 a popular choice among development boards for 3.3V supplies. The necessary current was calculated
 to be ~.5A for all the board components (.3A alone for the relays). The LM1117 can supply up to .8A
 at max, which was more then enough for our application. 
 
 ##	Input Plugs
 
 ![Input Plugs](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/Input_Plugs.png)
 
 This schematic is just the input USB port (for programming/power of the ESP32 and sensors)
 as well as testing pins for the 120Vac (line, neutral, and Egnd). 
 
 ## Current Sensor ACS722
 
 ![ACS722 Current Sensor](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/Current_Sensor_722.png)
 
 The ACS722 is a very powerful current measuring chip that uses the Allegro Hall effect technology. This 
 chip is able to measure up to +-10A of current and output the result as a radiometric voltage signal
from 1.5V -> 0A to 3.3V -> 10A. This is a linear relationship and has a sensitivity of ~267mV/A. 
Using the ESP32's built in Analog - Digital Converters (ADCs) we are able to measure this signal 
with an accuracy of 12 bits over 3.3V (as in 0000.... = 0V and 1111.... = 3.3V) with a resolution
of 2^12 / 3.3V (or ~.8mV). This translates to a  .8mV/bit * 1/267 A/mV = .003 A/bit or 3mA resolution.

## ACS71020 

![ACS71020 Current/Voltage Sensor](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/ACS710202_Current_Sense.png)

This chip is actually a very powerful current/voltage sensor for AC applications. Not only is it able to tolerate 120Vac up to
30A, it can also preform RMS, power, actual power, and instantaneous power. The package is also isolated
for voltages up to 1047Vrms in basic isolation and 517Vrms in reinforced isolation. The measurement values
are read over the I2C bus, making communication easy and space efficient. 


*note*: we actually had a very difficult time getting this part to work as the chip itself is very new
(released around Nov '18) and no examples for implementation were found online. So we had to make our 
own I2C drivers for this, which proved difficult as we were inexperienced. 

## Relay Node

![Relay Node](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/Relay_node.png)

For this the relays are simply electromechanical with a rated usage of ~100,000 switches. They are 
5V compatible and take ~70.2mA for full activation/saturation. In this circuit the ESP will send a 
3.3V signal to a 2N222 bjt amplifier (with ~100 gain, this equates to ~.7mA from the ESP) which in
turn will supply the necessary current for the relays. Each one has an LED line attached to it so that
there will be a status LED which will light up when active (when the ESP sends a +3.3V signal). As of 
the writing of this, the relays were connected wrong in that the output line was connected to the 
"Normally Closed" (NC) pin of the relay (meaning when ESP sends 3.3V it will be open, and normally closed)
This should be changed to the "Normally Open" pin. 

## Voltage Selection Diodes

![Voltage Select](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/Voltage_Selector_Diode.png)

There will be situations where we would like to only power the ESP32/peripheral sensors (to program, test, etc) and *not* 
anything else (since powering the relays might overcurrent a USB 2.0 port on say a laptop. As such it would be good to have 
a USB voltage input and a normal 5V input. Having 2 separate voltages however can cause issues (if they are trying to 
backdrive each other). A solution was made where we would use 2 Oring diodes that will only let the higher of the 2 
voltages act as the driving voltage, while still allowing the other to be connected. This lets us power only the 
ESP32/periphery using the USB voltage (to program) and still being able to plug in the 5V line later on if we want to 
do testing. We can also power the whole board from the main 5V while using the USB connection 
to do debugging. 

 
