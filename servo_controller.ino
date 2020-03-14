/* Ctrl-S (Control Servos)
 by Clark Teeple
 <>
 This code enables you to set setpoints and interpolate to them over a specified time, all via serial commands

 This code is based on...
  - Arduino Servo Sweep example <https://www.arduino.cc/en/Tutorial/Sweep>
  - Ctrl-P (Control Pressure) <https://github.com/cbteeple/pressure_controller>
    - There is a lot of shared DNA between this firmware and my pressure controller project.
*/

#include <Servo.h>

// Set up these parameters
#define NUM_SERVOS 4
int servo_pins[]={2,3,4,5,6,7,8,9};
int timestep = 10;


/*
 * No configuration needed past here. Everything past here should be taken care of.
 * Feel free to investigate if you want to understand how I wrote this though.
 */

Servo myServos[NUM_SERVOS];

float pos = 0;    // variable to store the servo position
unsigned long transition_time = 0;
float setpoint[NUM_SERVOS];
float prev_setpoint[NUM_SERVOS];
bool new_setpoint = true;

int servo_min = 544;
int servo_max = 2400;


void setup() {
  Serial.begin(112500);

  // Initialize the servos
  for(int i=0; i<NUM_SERVOS; i++){
    myServos[i].attach(servo_pins[i],servo_min, servo_max);
    setpoint[i] = 0;
    prev_setpoint[i] = 0;
  }
}



void loop() {

  if (new_setpoint){
    int num_steps = ceil(float(transition_time)/float(timestep));

    String pos_str = "";
    for (int step = 0; step <= num_steps; step += 1) {

      // Update the setpoints for all of the servos at once
      pos_str = String(millis());
      pos_str += '\t'+"Positions: ";
      for(int i=0; i<NUM_SERVOS; i++){
        pos = lin_interp(float(prev_setpoint[i]), float(setpoint[i]), float(step)/float(num_steps));     
        float pos_micro = mapFloat(pos,0.0,180.0,float(servo_min),float(servo_max)); 
        myServos[i].writeMicroseconds(int(pos_micro));              // tell servo to go to position
        pos_str += '\t'+String(pos,4);
        
      }
      send_string(pos_str);
      delay(timestep);
    }
    
    pos_str = String(millis());
    pos_str += '\t'+"Positions: ";
    for(int i=0; i<NUM_SERVOS; i++){
      float pos_micro = mapFloat(setpoint[i],0.0,180.0,float(servo_min),float(servo_max)); 
      myServos[i].writeMicroseconds(int(pos_micro));
      pos_str+= '\t'+String(setpoint[i],4);
      
      prev_setpoint[i] = setpoint[i];
    }
    send_string(pos_str);
    new_setpoint = false;
  
  }

  // check for new serial messages
  check_serial();

}



// Check for new serial data
String check_serial() {
  // Get new command
  String command="";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // Add new byte to the inputString:
    command += inChar;
    // If the incoming character is a newline, set a flag so we can process it
    if (inChar == '\n') {
      command.toUpperCase();
    }
  }

  parse_command(command);

  return command;
}


// Send a string via serial
void send_string(String bc_string){
  Serial.println(bc_string);
}


// Parse Commands
void parse_command(String command){
  if (command.length()){
    String out_str="_";
    if(command.startsWith("TIME")){
      if (get_string_value(command,';', 1).length()){
        transition_time = get_string_value(command,';', 1).toInt();
        out_str+="New ";
      }
      out_str+="Transition Time: " + String(transition_time);
    }
    
    else if(command.startsWith("SET")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          setpoint[i] = get_string_value(command,';', i+1).toFloat();
          
        }
        new_setpoint = true;
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toFloat();

        for(int i=0; i<NUM_SERVOS; i++){
          setpoint[i] = allset;
          
        }
        new_setpoint = true;
        out_str+="New ";
      }
      out_str+="Setpoint: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(setpoint[i],4);
      }
      
    }
    else{
      out_str = "Unrecognized Command";  
    }
    send_string(out_str);
  }
}

// Get one element of a string command with delimeter
String get_string_value(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}



// Linear interpolation
float lin_interp(float a, float b, float f){
    return a + f * (b - a);
}



// Float mapping (not native to arduino for some reason)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

