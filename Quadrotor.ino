/*
 * Quadrotor - Version 1
 * To be uploaded to the Flight Controller
 * To be used with tx_test.ino
 * 
 * 
 * Controller Scheme:
 *  - PID controllers for Yaw, Pitch, Roll, and Thrust
 *  - Yaw, Pitch and Roll come from MPU6050 measurements
 *  - Thrust comes from BMP180 measurements
 *  
 *  Radio Scheme:
 *  - "Quad node" receives YPRT data through 4 different pipes from the "Control Node"
 *  - Data is fed into a data structure "rxData" and passed to the controller inputs
 *  
 *  Servo Scheme:
 *  - One servo sends PWM signals to one of 4 ESC's, which control prop speeds
 *  - Servos are initialized to send PWM values of 0
 *  - Each ESC has a minimum PWM value that must be met before its prop starts spinning.
 *    Likewise each ESC has a maximum PWM value that corresponds to max prop speeds.
 *    The controllers must be aware of these min/max values
 *  
 *  All libraries and setup functions are included and defined in the Quadrotor.h header file.
 */

/////////////////////
// INCLUDE LIBRARIES
//////////////////////
// Controller class organizes PID functions
#include "Controller.h"

// Quadrotor class organizes Quad functions
#include" Quadrotor.h"




//////////
// SETUP
/////////
void setup() {
  // Start Serial monitor for debugging
  Serial.begin(57600);
  printf_begin();

  // Instantiate Quadrotor
  Quad Quad1;

  // Instantiate Controllers
  // Instantiate controllers
  Controller yaw_controller;
  Controller pitch_controller;
  Controller roll_controller;
  Controller thrust_controller;
  
  // Run setup functions
  setup_radio();              // Set up Radio and start listening
  setup_servos();             // Set up servo connections to ESC's
  setup_controller(yaw_controller, *controllerInputs, *controllerOutputs.propSpeed1, *controllerSetpoints);   // Prop 1 setup
  setup_controller(prop_2, *controllerInputs, *controllerOutputs.propSpeed2, *controllerSetpoints);   // Prop 2 setup
  setup_controller(prop_3, *controllerInputs, *controllerOutputs.propSpeed3, *controllerSetpoints);   // Prop 3 setup
  setup_controller(prop_4, *controllerInputs, *controllerOutputs.propSpeed4, *controllerSetpoints);   // Prop 4 setup
  setup_MPU();                // Set up Accelerometer/Gyroscope
  setup_BMP180();             // Set up Barometer

  // Turn debug mode on or off
  // When on: Serial control
  // When off: Radio control
  bool DEBUG_MODE = false;
}


////////
// LOOP
////////
void loop() {
  // Get Setpoint Commands (desired)
  read_commands();
  
  // Get Sensor Values (actual)
  read_MPU6050();
  read_BMP180();
  
  // Compute prop speeds to reduce error

}




