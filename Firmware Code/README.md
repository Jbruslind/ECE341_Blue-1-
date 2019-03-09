# Firmware Code

This folder holds all of the code used throughout this project. This code is primarily used for 
controlling the microcontroller used on the custom PCB, the ESP32. The code was done in C++ via 
the Arduino IDE. Within the folder, there also exists the flow diagram for the NodeRed server 
that handles the user web interface.

Individual pieces of code are present in this folder, which was how they were developed. After the
individual pieces of code were able to successfully work on their own, they were adapted into a
single .ino file to be uploaded to the ESP32, called ESP_Firmware.