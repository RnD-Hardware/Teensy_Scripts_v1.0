/*********************************************************************
    ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Software License Agreement -(BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

       Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
       Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM 150

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

//#ifdef USE_BASE

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE 30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
//#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int indexROS = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// ZL Motor
int32_t enc_count_left, enc_count_right;
uint16_t temperature;
int err;
#define main_SW_FB     34
#define Estop_FB       33
#define Main_SW_FB digitalRead(main_SW_FB)
#define estop_FB digitalRead(Estop_FB)

// smartSwitch()
bool flag_shutdown, Estop_FB_flag = false;
unsigned long shut_down_count;

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  indexROS = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      //      Serial7.print("BAUDRATE=");
      //      Serial7.println(BAUDRATE);
      break;
    case ANALOG_READ:

      //      Serial7.print("ANALOG_READ=");
      //      Serial7.println(ANALOG_READ);
      //      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:

      //      Serial7.print("DIGITAL_READ=");
      //      Serial7.println(DIGITAL_READ);
      //      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      //      analogWrite(arg1, arg2);
      //      Serial.println("ANALOG_WRITE_OK");

      //      Serial7.print("ANALOG_WRITE=");
      //      Serial7.println(ANALOG_WRITE);
      break;
    case DIGITAL_WRITE:
      //      if (arg2 == 0) digitalWrite(arg1, LOW);
      //      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      //      Serial.println("DIGITAL_WRITE_OK");

      //      Serial7.print("DIGITAL_WRITE=");
      //      Serial7.println(DIGITAL_WRITE);
      break;
    case PIN_MODE:
      //      if (arg2 == 0) pinMode(arg1, INPUT);
      //      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      //      Serial.println("PIN_MODE_OK");

      //      Serial7.print("PIN_MODE=");
      //      Serial7.println(PIN_MODE);
      break;
    //    case PING:
    //      Serial.println(Ping(arg1));
    //      break;

    case READ_ENCODERS:
      Serial.print(-encoder_read_left()); //readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(encoder_read_right());

      Serial7.print("en=");
      Serial7.print(-encoder_read_left()); //readEncoder(LEFT));
      Serial7.print(" ");
      Serial7.println(encoder_read_right());

      //      Serial7.print("READ_ENCODERS=");
      //      Serial7.println(READ_ENCODERS);
      break;
    case RESET_ENCODERS:
      clear_encoder();  // resetEncoders();
      resetPID();
      Serial.println("RESET_ENCODERS_OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        drive_vel_motor_straight(0, 0);// setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("MOTOR_SPEEDS_OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // Sneaky way to temporarily disable the PID
      drive_vel_motor_straight(arg1, arg2); // setMotorSpeeds(arg1, arg2);
      Serial.println("MOTOR_RAW_PWM_OK");
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial7.print("PID=");
      Serial7.print(Kp);
      Serial7.print("=");
      Serial7.print(Kd);
      Serial7.print("=");
      Serial7.print(Ki);
      Serial7.print("=");
      Serial7.print(Ko);
      Serial7.println("=");

      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  Serial7.begin(9600);
  pinMode(Estop_FB, INPUT);
  // Initialize the motor controller if used
  init_can();
  clear_error();  clear_encoder();
  //  en_motor();
  stop_motor();
  en_motor();
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/

void loop() {
  //  drive_vel_motor_straight(5, 5);
  //  Serial.print(encoder_read_left()); //readEncoder(LEFT));
  //  Serial.print(" ");
  //  Serial.println(encoder_read_right());
  //  clear_encoder();
  eStop_freeMotor();
  while (Serial.available() > 0) {
    eStop_freeMotor();
    // Read the next character
    chr = Serial.read();
    //    Serial7.print("chr=");
    Serial7.println(chr);
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[indexROS] = NULL;
      else if (arg == 2) argv2[indexROS] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[indexROS] = NULL;
        arg = 2;
        indexROS = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
        //        Serial7.print("cmd=");
        //        Serial7.println(cmd);
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[indexROS] = chr;
        indexROS++;
      }
      else if (arg == 2) {
        argv2[indexROS] = chr;
        indexROS++;
      }
    }
  }

  // If we are using base control, run a PID calculation at the appropriate intervals
  //#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    ;
    // setMotorSpeeds(0, 0);
    drive_vel_motor_straight(0, 0);
    moving = 0;
  }
  //#endif
}
