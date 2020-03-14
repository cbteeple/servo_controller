# servo_controller

Control several servos simultaneously using some serial commands.

## Setup
### Hardware
1. Several servos connected to 5V, Ground, and each to thier own PWM pin
2. 5V power (4 servos work fine on USB power most of the time).

### Software
1. Update _servo_pins_ variable with pin numbers for your board (most arduino/teeney boards have many PWM pins)
2. Update _NUM_SERVOS_ to suit your setup.
3. Mess with the _timestep_ if you want to. 10 ms seems to work for me.
4. Flash the program

## Usage
1. Connect to your MCU via serial, and type commands into the arduino serial monitor (or any other serial terminal)

### Global Settings
1. **Transition Time in ms** (`int, default = 500`) - transition between the current setpoint and new setpoints
    - `time;1500` sets a 1.5 second transition (servo speed is limited to meet this time)
    - `time;0` sets a 0 second transition (servo moves as fast as it can)
2. **Echo** (`bool, default = true`) - recieve echos of each commnd you send.
    - `echo;0` turn echos off
    - `echo;1` turn echos on
3. **Use Degrees** (`bool, default = true`) - use degrees rather than raw pwm microsecond values
    - `deg;0` use raw microsecond values
    - `deg;1` use degrees

### Channel-Specific Settings
_Note: for channel-specific commands, the number of arguments in a command needs to equal the number of channels you have. Otherwise the first value will be applied to all channels._

1. **Control Mode for all chanels** (`int, default = 1`) - Control how the servos move during the transition time
    - `mode;0` do linear interpolation from the current positon the new setpoint
    - `mode;1` use smooth control (linear velocity ramps), assuming speed is 0 at both ends.
2. **Control Mode for each channel** (`int, default = 1,...,1`)
    - `mode;0;1;1;1` set all channels to use smooth control, except channel 1 (which uses linear interpolation)
3. **New Setpoint for all channels** (`float, default = 0`) - Set setpoint angle for all channels (in degrees or microseconds depending on the `deg` setting)
    - `set;90` sets the setpoint to 90 degrees for all servos
4. **New Setpoint for each channel** (`float, default = 0,...,0`) - Set the setpoint for each channel separately
    - `set;45;90;135;180` sets the setpoints for all four channels

## Reference:
### This project is based heavliy on my [Ctrl-P project](https://github.com/cbteeple/pressure_controller).   
- The command structure is identical.
- The onboard live interpolation is similar

### Some differences:  
- This is super simple. No trajectory buffer or other deterministic timing.
- No real-time checking for new serial commands.
- You can't store settings in EEPROM yet.
