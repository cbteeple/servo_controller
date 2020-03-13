# servo_controller

Control several servos simultaneously using some serial commands.

## Hardware Setup
1. Several servos connected to 5V, Ground, and each to thier own PWM pin
2. 5V power (4 servos work fine on USB power most of the time).

## Software Setup
1. Update _servo_pins_ variable with pin numbers for your board (most arduino/teeney boards have many PWM pins)
2. Update _NUM_SERVOS_ to suit your setup.
3. Mess with the _timestep_ if you want to. 10 ms seems to work for me.
4. Flash the program

## Usage
1. Connect to your MCU via serial
2. Set the desired transition time (in ms) between the current setpoint and new setpoints
  - `time;1500` sets a 1.5 second transition (servo speed is limited to meet this time)
  - `time;0` sets a 0 second transition (servo moves as fast as it can)
3. Set setpoint angle for all channels (in degrees)
  - `set;90` sets the setpoint to 90 degrees for all servos
4. Set the setpoint for each channel separately
  - `set;0;90;120;180` sets the setpoints for all four channels
  - _Note: the number of setpoints you send needs to equal the number of channels you have. Otherwise the fiest value will be applied to all channels._

## Reference:
This project is based heavliy on my [Ctrl-P project](https://github.com/cbteeple/pressure_controller).
- The command structure is identical.
- The onboard live interpolation is similar

Some differences:
- This is super simple. No trajectory buffer or other deterministic timing.
- No real-time checking for new serial commands.
- You can't store settings in EEPROM yet.
