# PCB Production Files

This directory holds all the necessary information/files to manufacture the project
PCB. The manufacturing files directory holds all the gerber files (7 layers: Top Copper Layer, 
Top Silkscreen Overlay, Top Soldermask Layer, Bottom Copper Layer, Bottom Silkscreen 
Overlay, Bottom Soldermask Layer, and the Keepout Layer which serves as the board outline).
This directory also holes the drill files (Slotted holes and round holes)

![Render of board](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/Top_View_PCB.png)


The gerber files directory contains all the above files as well as all the other layers
that are not used in production (usually just used for reference inside CircuitMaker). This
includes some layers which we used to designate spacings between objects and layers used
to create unqiue shapes of copper pours. 

The Bluetooth Switch directory holds all specific information about the project which
include pdf versions of all schematics and the PCB print, as well as other useful files 
such as a Bill of Materials and STEP file output of the whole board. 

## Special Layout Considerations

There weren't very many layout considerations that had to be made for this project. 
Other than trace sizes and component placement, the only special considerations that had
to be made involved the placement of the I2C signal lines, placement of the UART D+ and D-
lines from the UART-TTL bridge and the spacings between the AC voltage lines and the DC 
signal/power lines. The PCB was grouped into various segments to simplify component placement.
For example, the ESP32 segment included the ESP32, USB programmer, UART Bridge, EN/Boot
transistor and peripheral capacitors/resistors. This was grouped this way so all the 
important connections would be close together for easy routing. Other groupings
included the relay nodes (which included the relays, current sensors, flyback diodes, 
status LEDs, output plugs, and peripheral resistors/capacitors) and the power supply
which included the ORing diodes, input filtering capacitors, and Linear Dropout Regulator. 

  
![Highlighted Sections](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/Top_View_PCB_highlights.png)

1. Red indicates the ESP32 section
2. Blue indicates the Power section
3. Yellow indicates the Relay Node section

### Connectors
For this project, a USB B connection was chosen to act as our USB interface as it is 
robust and very durable. Initially the project used a USB micro due to its abundance
in other products, but this was found to be easily broken and torn out of the board. 

![USB B Connector](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/USB_B.jpg)


The power outlets were chosen to be NEMA standard C13 plugs so that way we would only 
need to solder the plugs into the board (no wires). Unfortunatly most of these plugs
were panel mount, meaning it was not easy to plug into a PCB, but we were able to find 
PCB mount plugs (which were adapted panel mount plugs) on Digikey [here](https://www.digikey.com/product-detail/en/qualtek/739W-X2-32-A/Q1211-ND/8681866)

![C13 Plug made by Qualtek](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/NEMAC13Plug.jpg)


The input DC power was chosen to be an XT30 plug which is a 2 pin, yellow, friction fit 
plug that is often used on drones/quadcopters for lithium batteries. DC power only needed
to provide .5A and the XT30 is rated for 30A per plug.  

![XT30 Connectors](https://github.com/Jbruslind/ECE341_Blue-1-/blob/master/Readme_images/XT30-2.jpg)

