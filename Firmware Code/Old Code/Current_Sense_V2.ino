#define SENSOR_BIAS_ALPHA 1850                                // zero current bias for 1st current sensor
#define SENSOR_BIAS_BRAVO 1850                                // zero current bias for 2nd current sensor

const int num_readings = 10;                            // number of elements in shift register
const int input_pin_alpha = 35;                               // input pin for 1st channel of microcontroller
const int input_pin_bravo = 34;                                // input pin for 2nd channel of microcontroller

float readings_alpha[num_readings];                           // shift register that holds last n current readings for sensor 1, where n = num_readings
float readings_bravo[num_readings];                            // shift register that holds last n current readings for sensor 2, where n = num_readings
float total = 0;                                              // the total of currents in shift register
float average = 0;                                            // the average of last n current readings_alpha, where n = num_readings
float current = 0.0;                                          // calculated current from sensors
int inputVal = 0;                                             // raw input from sensors
int bias = 0;                                                 // input from sensors adjusted for zero-current bias

void setup() {
  Serial.begin(115200);                                       // initialize serial communication with computer
  
  for (int i = 0; i < num_readings; i++) {                    // initialize all readings to 0
    readings_alpha[i] = 0;                        
    readings_bravo[i] = 0;
  }                 
}

void loop() {
  for(int i = 1; i < num_readings; i++){                      // shift register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
      readings_alpha[num_readings - i] = readings_alpha[num_readings - (i + 1)];
  }

  inputVal = analogRead(input_pin_alpha);                     // read raw input from sensor
  bias = inputVal - (SENSOR_BIAS_ALPHA);                                   // adjust reading for zero-current bias 
  current = bias/(165.0);                                     // calculate current from sensor using known component sensitivity
  readings_alpha[0] = current;                                // store new value in 0th index || Ex: [X | X | Y] --> [W | X | Y]
     
  for (int i = 0; i < num_readings; i++){                     // sum recent currents within shift register
    total += readings_alpha[i];
  }
  
  average = total/num_readings;                               // find average value within shift register
  total = 0;                                                  // zero total for next loop

  Serial.print("Aplha Current: ");
  Serial.println(average);                                    // print average current to serial monitor
  Serial.println("");
  
  for(int i = 1; i < num_readings; i++){                      // shift register to the right || Ex: [X | Y | Z ] --> [X | X | Y]
      readings_bravo[num_readings - i] = readings_bravo[num_readings - (i + 1)];
  }

  inputVal = analogRead(input_pin_bravo);                     // read raw input from sensor
  bias = inputVal - (SENSOR_BIAS_BRAVO);                      // adjust reading for zero-current bias 
  current = bias/(165.0);                                     // calculate current from sensor using known component sensitivity
  readings_bravo[0] = current;                                // store new value in 0th index || Ex: [X | X | Y] --> [W | X | Y]
     
  for (int i = 0; i < num_readings; i++){                     // sum recent currents within shift register
    total += readings_bravo[i];
  }
  
  average = total/num_readings;                               // find average value within shift register
  total = 0;                                                  // zero total for next loop

  Serial.print("Bravo Current: ");
  Serial.println(average);                                    // print average current to serial monitor
  Serial.println("");
  delay(10);                                                  // introduce delay to allow for settle time/stability        
}
