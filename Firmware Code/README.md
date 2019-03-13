# Firmware Code

This folder holds all of the code used throughout this project. This code is primarily used for 
controlling the microcontroller used on the custom PCB, the ESP32. The code was done in C++ via 
the Arduino IDE. Within the folder, there also exists the flow diagram for the NodeRed server 
that handles the user web interface.

Individual pieces of code are present in this folder, which was how they were developed. After the
individual pieces of code were able to successfully work on their own, they were adapted into a
single .ino file to be uploaded to the ESP32, called ESP_Firmware.

##	Functions

###	void check_time()
Helper function whose main job is to check the timers and current time. When the current time 
minus the time each timer started goes beyond the set time limit, it calls a separate function 
to turn off the plugs and reset all the values. 

This could be improved by consolidating the functions (a previous revision needed 3 seperate functions
this current revision does not) so only one "timer complete" function is called with a specific argument 
for what plug to turn off (Ex. Timer_complete(1) -> turn off plug one). 

Inputs: None

Outputs: none

### String get_time(int time_to, int plug)

Helper function that will convert the time data variables (which are stored as raw millisecond values) to
human-readable strings. In this case it will take in the time at which the plug timer started (time_to) and
how long the plug is supposed to remain on (plug). These values are used along with the current time (millis)
to find exactly how long it's been since the plug timer started. This value is then divided into hours, minutes
and seconds and then casted into a string with the format HH:MM:SS. 

This function is kinda clunky because it uses strings and not char arrays. Since the format is known (HH:MM:SS) 
it would be better to just have a 8 char static array and then populate it with values as we calculate them. 

Inputs: Time that the timer started at (time_to), how long the timer is on for (plug)

Outputs: String formatted as HH:MM:SS (int values)

### void timer_complete_X()

There are 3 of these functions for each plug. All it does is turn "off" the plug (by setting the relay high) and
resets the time the plug is supposed to be on to 0. 

This is kind of a stupid way to do this and it should definitely be improved. Consolidating it into another function
and not having 3 seperate functions to do a very simple task would be a good start. 

Inputs: None

Outputs: None

###	setup_wifi()

This function will attempt to connect to the specified wifi SSID with the given password. IT is a blocking function 
and will hold up all other operations if it cannot connect to the givien SSID. 

This should be fixed so that it is not blocking and will just retry the connection every X seconds when it can't connect.

Inputs: None

Outputs: None

###	 start_timers()

