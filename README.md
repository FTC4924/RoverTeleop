# Runt Rover Arduino
Code for ServoCity Runt Rovers, powered by Arduino (Pololu Leonardo), with R/C Transmitter and Receiver. Supports Teleop and Autonomous

The aim of this project was to create a starting point for the Runt Rover line of low-cost robot platforms like these: https://www.servocity.com/whippersnapper-runt-rover/
(6 Volt DC motors drive a lightweight chassis).  We wanted to have both autonomous control and teleop control, so this meant using a transmitter/receiver 
but also a single-board-computer.  The receiver output is read by the arduino. Channels 1 & 2 control the low-voltage drive motors, while channels 3 & 4 
are available for servos or PWM actuators, for end-effectors such as a claw, arm or rotating robot-head.
This was a summer project of FIRST Tech Challenge (FTC) team 4924, the Red Beard Pandas. Sponsored by New River Robotics Association.
Red Beard Pandas: https://www.team4924.org/    New River Robotics: https://www.newriverrobotics.org/

Environment configuration:
Setting up Arduino IDE for Runt Rover coding - using the Pololu board https://www.pololu.com/product/4007 and motor controller https://www.pololu.com/product/2512 
with goBILDA 6-channel transmitter & receiver, plus 2 or 4 motor rover platform (such as ServoCity 6 Volt robot chassis)

For Arduino IDE, need to add these drivers & libraries:

Pololu drivers - Need A-Star Windows driver here: https://www.pololu.com/docs/0J61/6.1 
Then  https://www.pololu.com/docs/0J61/6.2 - through step 10. No need to run the blink example.

Servo Input - from the Arduino Tools, Manage Libraries..  search on ServoInput - add the library with dependencies.

Motor shield: the bridge from Arduino to 2 DC motors
Drv8835-motor-shield - search for drv8835 in the arduino library

Servo.h - also from the Arduino Tools, Manage Libraries..  search on Servo - add the library with dependencies.  This library is to drive servo output.  Important modification referenced here:
https://www.pololu.com/docs/0J69/3.9.1  - otherwise, we can’t output to servos since the motor driver board uses the same timer. Short version: just
overwrite ServoTimers.h file with the modified version (using timer3 instead of Timer1 for _AVR_ATmega32U4__) in libraries/Servo/src/avr/ServoTimers.h

Code: RoverTeleop, this github repo

Wiring: 
6V battery: Connect to Vin & GND on motor board
R/C Receiver: 3 wire servo connector from MBUS port to: Black pin to Arduino GND (4th from miniUSB connector on pin row)
Red to REF (2nd pin from power-jack side, 2 open pins away from the motor board)
Brown/Signal to Serial input/RX - not used (tried & failed to read all servos via MBus. Apparently, Arduino doesn’t support the inverted serial signal)
Servo Channels: R/C receiver into Arduino/Leonardo: Put male end of 4 wires into Pins 0,1,2,3 (7 is also an interrupt, but used by motor board). Put female ends of above wires into receiver, Channels 1,2,3,4.  -for example, orange male end into Leonardo pin 0, orange female end into goBILDA element receiver channel 1, signal pin.
Servos: output will come from Leonardo - used pins 4 & 5 for channels 3 & 4.  Channels 1 & 2 go to the motor shield via programming.


Docs on pins available: https://www.partsnotincluded.com/how-to-use-an-rc-controller-with-an-arduino/
Leonardo, other 32u4-based: use digital pins 0, 1, 2, 3, 7 for Interrupts
For the best results the servo channel’s signal pin should be connected to a pin on the Arduino that is capable of external interrupts. This allows the Arduino to read the servo’s position in the background without disturbing your program
