#include <stdlib.h>
#include <Arduino.h>

#ifndef __smooth_pos_H__
#define __smooth_pos_H__


// Define a nice class to handle smoothing.
class smoothPos
{
  private:
    // Parameters the user will end up setting via public functions
    float x1 = 0;
    float x2 = 0;
    float tf = 1.0;

    // Parameters used for  motion smoothing with linear velocity ramps
    float vm = 0;
    float a  = 0;

    // Parameters used in control logic
    float goal_reached = true;
    int mode = 1;

    // Calculate the parameters involved in motion smoothing with linear velocity ramps
    void calc_params(){
      vm = 2*(x2-x1)/tf;
      a  = 2*vm/tf;
    };

    // Linear interpolation
    float lin_interp(float a, float b, float f){
        return a + f * (b - a);
    };



  
  public:
    // Class definition function
    smoothPos(){};

    // Initialize with position and transition time;
    void init(float curr_pos, float tf_in){
      x1 = curr_pos;
      x2 = curr_pos;
      goal_reached = true;
      tf = tf_in;
    }

    // Set the motion control mode
    //   0 = linear position interp,
    //   1 = smooth position (linear velocity ramp)
    void set_mode(int new_mode){
      if (new_mode >=0 & new_mode <=1){
        mode = new_mode;
      }
    
    }

    // Get the motion control mode
    int get_mode(){
        return mode;  
    }

    // Set the transition time in milliseconds
    void set_time_ms(int time_in){
      tf = float(time_in)/1000.0;
    };

    // Set the transition time in seconds    
    void set_time_sec(float time_in){
      tf = time_in;
    };

    // Set a new goal position and prepare to move there
    void set_new_goal(float goal){
      if (goal_reached){
        x1 = x2;
        x2 = goal;
        goal_reached = false;
        calc_params();
      }
    };

    // Calculate the interpolated setpoint to use at the current time (in miliseconds)
    float get_point_ms(int t_curr){
        return get_point(t_curr/1000.0);
    }

    // Calculate the interpolated setpoint to use at the current time (in seconds).
    float get_point(float t_curr){

      if (t_curr==0){
        return x1;
      }
      if (t_curr==tf){
        goal_reached = true;
        return x2;
      }

      float set = 0;
      switch (mode){
        case 0: { // Linear interpolation
          set = lin_interp(x1, x2, t_curr/tf);   
        } break;
        
        case 1: { // Smooth motion
          if (t_curr <= tf/2.0){
            set = 0.5*a*pow(t_curr,2) + x1;
          }
          else{
            set = x2 - 0.5*a*pow(tf-t_curr,2);
          
          }
        } break;
      }
      
      return set;
    };


    // manually mark that you have reached the desired setpoint.
    float mark_reached(){
      goal_reached = true;
      return x2;
    };

};




#endif
