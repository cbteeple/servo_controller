/* Ctrl-S (Control Servos)
 by Clark Teeple
 <>
 This code enables you to set setpoints and interpolate to them over a specified time, all via serial commands

 This code is based on...
  - Arduino Servo Sweep example <https://www.arduino.cc/en/Tutorial/Sweep>
  - Ctrl-P (Control Pressure) <https://github.com/cbteeple/pressure_controller>
    - There is a lot of shared DNA between this firmware and my pressure controller project.
*/


/*

anthro
set;50;130

middle
set;100;80

cross
set;150;30




 */

#include <Servo.h>
#include "smooth_pos.h"

// Set up these parameters
#define NUM_SERVOS 2
int servo_pins[]={2,3,4,5,6,7,8,9};
int timestep = 10;

// Minimum and maximum PWM timings for the servos to reach 0 and 180 degrees
int servo_min = 540;  // default from arduino tutorial: 544
int servo_max = 2490; // default from arduino tutorial: 2400


#define NUM_DEF_POSITIONS 10
float def_positions[NUM_DEF_POSITIONS][NUM_SERVOS]=
    {{105,80},{155,30},{55,130},{75,110},{85,100},
    {0,0},{0,0},{0,0},{0,0},{0,0}};

/*
 * No configuration needed past here. Everything past here should be taken care of.
 * Feel free to investigate if you want to understand how I wrote this though.
 */

Servo myServos[NUM_SERVOS];


unsigned long transition_time = 100;
float setpoint[NUM_SERVOS];
float prev_setpoint[NUM_SERVOS];
bool new_setpoint = true;
bool use_degrees = true;
bool echo_global = true;

smoothPos smooth_positions[NUM_SERVOS];


void setup() {
  Serial.begin(112500);

  // Initialize the servos
  for(int i=0; i<NUM_SERVOS; i++){
    myServos[i].attach(servo_pins[i],servo_min, servo_max);
    setpoint[i] = def_positions[0][i];
    prev_setpoint[i] = def_positions[0][i];

    smooth_positions[i].init(def_positions[0][i], transition_time);
    smooth_positions[i].set_mode(0);
  }
}



void loop() {

  if (new_setpoint){
    // Set the new goal for all channels 
    for(int i=0; i<NUM_SERVOS; i++){
      smooth_positions[i].set_time_ms(transition_time);
      smooth_positions[i].set_new_goal(setpoint[i]);
    }

    int num_steps = ceil(float(transition_time)/float(timestep));
    String pos_str = "";
    
    for (int step = 0; step <= num_steps; step += 1) {

      int curr_time_ms = timestep*step;

      // Update the setpoints for all of the servos at once
      pos_str = String(millis());
      pos_str += '\t'+"Positions: ";
      for(int i=0; i<NUM_SERVOS; i++){
        float pos = smooth_positions[i].get_point_ms(curr_time_ms);
        float pos_micro = pos;
        if (use_degrees){
          pos_micro = mapFloat(pos,0.0,180.0,float(servo_min),float(servo_max));
        }
         
        myServos[i].writeMicroseconds(int(pos_micro));              // tell servo to go to position

        pos_str += '\t'+String(pos,4);
        
        
      }
      send_string(pos_str);
      delay(timestep);
    }
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
    bool echo_one_time=false;
    
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
    else if(command.startsWith("MODE")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          smooth_positions[i].set_mode(get_string_value(command,';', i+1).toInt());
        }
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        int allset=get_string_value(command,';', 1).toInt();

        for(int i=0; i<NUM_SERVOS; i++){
          smooth_positions[i].set_mode(allset);
        }
        out_str+="New ";
      }
      out_str+="Mode: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(smooth_positions[i].get_mode());
      }
      
    }

    else if(command.startsWith("POS")){
      if (get_string_value(command,';', 1).length()){
        int pos_cmd=get_string_value(command,';', 1).toInt();

        if (pos_cmd>=0 & pos_cmd<NUM_DEF_POSITIONS){
          for(int i=0; i<NUM_SERVOS; i++){
            setpoint[i] = def_positions[pos_cmd][i];
            
          }
          new_setpoint = true;
          out_str+="New ";
        }
      }
      out_str+="Position Setpoint: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(setpoint[i],4);
      }
      
    }

    else if(command.startsWith("DEFPOS")){
      if (get_string_value(command,';', NUM_SERVOS+1).length()){
        int pos_idx = get_string_value(command,';', 1).toInt();

        if (pos_idx>=0 & pos_idx<NUM_DEF_POSITIONS){
        
          for(int i=0; i<NUM_SERVOS; i++){
            def_positions[pos_idx][i] = get_string_value(command,';', i+2).toFloat();
            
          }
          out_str+="New Position Definitions: ";
          out_str += String(pos_idx,4);
          for(int i=0; i<NUM_SERVOS; i++){
            out_str += '\t'+String(def_positions[pos_idx][i],4);
          }
        }
      }
      else if (get_string_value(command,';', 1).length()){
        int pos_idx  = get_string_value(command,';', 1).toInt();
        float allset = get_string_value(command,';', 2).toFloat();

        if (pos_idx>=0 & pos_idx<NUM_DEF_POSITIONS){
        
          for(int i=0; i<NUM_SERVOS; i++){
            def_positions[pos_idx][i] = allset;
            
          }
          out_str+="New Position Definitions: ";
          out_str += String(pos_idx,4);
          for(int i=0; i<NUM_SERVOS; i++){
              out_str += '\t'+String(def_positions[pos_idx][i],4);
          }
        }
      }

      else{
        out_str+="Position Definitions: ";
        out_str+= '\n';
  
        for (int pos_idx=0; pos_idx <NUM_DEF_POSITIONS; pos_idx++){

          out_str += String(pos_idx);
          for(int i=0; i<NUM_SERVOS; i++){
            out_str += '\t'+String(def_positions[pos_idx][i],4);
          }
          out_str += '\n';
        }
      }
    }

    else if(command.startsWith("DEG")){
      if (get_string_value(command,';', 1).length()){
        use_degrees = bool(get_string_value(command,';', 1).toInt());
        out_str+="New ";
      }
      out_str+="Degrees Usage: " + String(use_degrees);
    }
    else if(command.startsWith("ECHO")){
      if (get_string_value(command,';', 1).length()){
        echo_global = bool(get_string_value(command,';', 1).toInt());
        echo_one_time = true;
        out_str+="New ";
      }
      out_str+="Echo: " + String(echo_global);
    }


    
    else{
      out_str = "Unrecognized Command";  
    }

    if (echo_global or echo_one_time){
      send_string(out_str);
      echo_one_time = false;
    }
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




// Float mapping (not native to arduino for some reason)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







