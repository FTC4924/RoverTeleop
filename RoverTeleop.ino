/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  with modifications by Henry Bass, August 2023
 *  @link       github.com/dmadison/ServoInput
 *  @license    LGPLv3 - Copyright (c) 2020 David Madison
 *
 *  This code is modified from examples in the Arduino Servo Input Library.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Example:      RoverTeleop
 *  Description:  Reads channels 1, 2, 3 and 4 from an RC receiver and prints 
 *                them over Serial: steering as a percent of bias to factor
 *                and throttle as an integer percentage for both forwards and reverse.
 *                Makes use of deadzone for joystick sensitivity.
 *                Outputs to 2 analog motors on an 8835 daugther board
 *                and output to 2 PWM servos plugged into pins defined below. 
 *
 *  Wiring:       Servo signals in a Pololu Leonardo to pins 0,1,2,3  (interrupts)
 * More Info:     See tutorial at https://www.partsnotincluded.com/how-to-use-an-rc-controller-with-an-arduino/
 */

#include <ServoInput.h>
#include <Servo.h>
#include <DRV8835MotorShield.h>  //Pololu DRV8835 Dual Motor Driver Shield Library

/* Signal pins for ServoInput MUST be interrupt-capable pins!
 *   **Micro, Leonardo (32U4): 0, 1, 2, 3, 7 <- USE THIS, but pin 7 is occupied by our Pololu motor board.
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */

// Steering Setup - Right joystick left-right, Channel 1
const int SteeringSignalPin = 0;  // MUST be interrupt-capable!, Left-Right on Right joystick
const int SteeringPulseMin = 980;  // microseconds (us)
const int SteeringPulseMax = 2060;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle Setup - Right Joystick up down, Channel 2
const int ThrottleSignalPin = 1;  // MUST be interrupt-capable!, Up-down on Right joystick
const int ThrottlePulseMin = 980;  // microseconds (us)
const int ThrottlePulseMax = 2060;  // Ideal values for your servo can be found with the "Calibration" example
const float Deadzone = 0.10; // Deadzone of 10% 

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

// Left Joystick up-down Channel 3 - is like Throttle, 0-100%
const int Chan3SignalPin = 2;  // MUST be interrupt-capable!, Up-down on Left joystick
const int Chan3PulseMin = 980;  // microseconds (us)
const int Chan3PulseMax = 2060;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<Chan3SignalPin> Chan3(Chan3PulseMin, Chan3PulseMax);

// Left Joystick left-right Channel 4 - is like steering, but read as an angle for this example
const int Chan4SignalPin = 3;  // MUST be interrupt-capable!, Left-Right on Right joystick
const int Chan4PulseMin = 980;  // microseconds (us)
const int Chan4PulseMax = 2060;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<Chan4SignalPin> Chan4(Chan4PulseMin, Chan4PulseMax);

//Motor Setup
#define MaxMotorSpeed 400  //Maximum speed value to send to motors
int MaxDesiredSpeed = 300;
#define MinSpeed 15
int leftSpeed = 0;
int rightSpeed = 0;
DRV8835MotorShield motors; //Create DRV8853 Motor Shield object

//servo setup (output servos)
Servo Ch3Servo;
int Ch3Val; //value to command the servo to position, usually 0-180 degrees
Servo Ch4Servo;
int Ch4Val; //value to command the servo to position, usually 0-180 degrees

void setup() {
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);  
  motors.flipM2(true);  //1 motor needed to be flipped to make both move forward for positive speed values 

  //Servos for output. Issue: PWM uses timers from pins 9 & 10, which is our motor controller. Can't have both!
  //https://forum.arduino.cc/t/dc-motors-doesnt-work-when-attaching-servo/462978
  // fix is to move the Servo.h timer. See this article: https://www.pololu.com/docs/0J69/3.9.1 (mod ServoTimers.h)
  Ch3Servo.attach(4, 980, 2060); //servo for Channel 3, Up-down on left joystick. Pin # for signal wire of servo
  Ch4Servo.attach(5, 980, 2060); //servo for Channel 4, left-right on left joystick. Pin # for signal wire of servo

	Serial.begin(115200);

	while (!ServoInput.available()) {  // wait for all signals to be ready
		Serial.println("Waiting for servo signals...");
		delay(500);
	}
}

void loop() {
	Serial.print("RC - ");

	float steeringPercent = steering.mapDeadzone(-200, 200, Deadzone);  // returns steering bias %
	Serial.print("Steer: ");
	Serial.print(steeringPercent);
	Serial.print("% ");

	Serial.print(" | ");  // separator

	int throttlePercent = throttle.mapDeadzone(-100, 100, Deadzone);  // remap to a percentage both forward and reverse
	Serial.print("Thro: ");
	Serial.print(throttlePercent);
	Serial.print("% ");

	if (throttlePercent >= 0) {
		Serial.print("(Fwd)");
	}
	else {
		Serial.print("(Rev)");
	}
	Serial.print(" | ");  // separator

  int Chan3Percent = Chan3.mapDeadzone(-100, 100, Deadzone);  // remap to a percentage both forward and reverse
	Serial.print("Chan3: ");
	Serial.print(Chan3Percent);
	Serial.print("% ");

	if (Chan3Percent >= 0) {
		Serial.print("(Fwd)");
	}
	else {
		Serial.print("(Rev)");
	}
  Serial.print(" | ");  // separator

  float Chan4Angle = Chan4.getAngle();  // returns 0 - 180
	Serial.print("Chan4: ");
	Serial.print(Chan4Angle);
	Serial.print("deg ");

  //Now apply Channels 1 & 2  to create steerable motor output.
  //Throttle both motors according to channel 2:
  rightSpeed = MaxDesiredSpeed * throttlePercent/100;
  leftSpeed = MaxDesiredSpeed * throttlePercent/100;
  //Steering input affects motor bias
  rightSpeed = rightSpeed *(1 - steeringPercent/100);
  leftSpeed = leftSpeed *(1 + steeringPercent/100);
  //limit Speeds to max:
  if (rightSpeed > MaxMotorSpeed) {
		rightSpeed = MaxMotorSpeed;
	}
    if (rightSpeed < -MaxMotorSpeed) {
		rightSpeed = -MaxMotorSpeed;
	}
   if (leftSpeed > MaxMotorSpeed) {
		leftSpeed = MaxMotorSpeed;
	}
    if (leftSpeed < -MaxMotorSpeed) {
		leftSpeed = -MaxMotorSpeed;
	}


  Serial.print(" | ");  // separator
	Serial.print("M1:M2 ");
	Serial.print(leftSpeed);
	Serial.print(":");
  Serial.print(rightSpeed);

  motors.setM1Speed(leftSpeed);
  motors.setM2Speed(rightSpeed);

  Ch3Val = map(Chan3Percent, -100, 100, 0, 180); // remap the throttle percent to 0-180 degrees.  Coulda done with angle but this might be useful elsewhere.
  Ch3Servo.write(Ch3Val);
  Ch4Val = Chan4Angle; //that was easy
  Ch4Servo.write(Ch4Val);

  Serial.print(" | ");  // separator
	Serial.print("V3:V4 ");
	Serial.print(Ch3Val);
	Serial.print(":");
  Serial.print(Ch4Val);

	Serial.println();
  //delay(500);  //0.5 sec delay to make serial monitor more readable.  Remove for production!
}
