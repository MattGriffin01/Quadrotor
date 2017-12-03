/*
* This file creates a class called Controller.
* The Controller class is used to instantiate a separate PID controller for Thrust, Roll Moment, Pitch Moment, and Yaw Moment
* The instantiations are in Quadrotor.ino main file
*/


#include "PID_v1_vector.h"


// Class definition
class Controller
{
public:
	
	// Public variables
	double Setpoint, Input, Output;
	double Kp, Kd, Ki;

	// Public functions
	void setMode(String MODE);                        // Set mode to Automatic or Direct (manual)
	void setSampleTime(int Ts);                       // Set sample time
	void setOutputLimits(int low, int high);          // Set limits on Output value
  void setSetpoint(double SETPOINT);                // set desired setpoint for input
  void setConstants(double KP, double KI, double KD);  // set tuning to get input error = 0

	void compute();                                   // Compute output based on input arguments



private:

	PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, AUTOMATIC);

};

// Function to set the PID mode
// AUTOMATIC: Controller will automatically adjust the output based on PID and input
// MANUAL: User can force the output of the controller to a desired number.
//          Controller will discontinue operation until back in AUTOMATIC
void Controller::setMode(String MODE) {
	if (MODE == "AUTOMATIC") {
		myPID.SetMode(AUTOMATIC);
	}
	else if (MODE == "MANUAL") {
		myPID.SetMode(MANUAL);
	}
}

// Function to set feedback sample time.
// Controller will only update the output once a sample time has passed, regardless of how often Compute() is called.
// This simplifies calculations
void Controller::setSampleTime(int Ts) {
	myPID.SetSampleTime(Ts);
}

// Function to restrict PID output to certain range.  Since Arduino PWM range is from 0-255, the controller
// should know not to try and output something out of range.
// ESC's have their own min/max values to operate the motors - these should be taken into account too
void Controller::setOutputLimits(int low, int high) {
	myPID.SetOutputLimits(low, high);
}

// Function to update controller's output
void Controller::compute() {
	myPID.Compute();
}

// Function to set the controller setpoint
void Controller::setSetpoint(double SETPOINT){
	Setpoint = SETPOINT;
}

// Function to set the controller PID constants
void Controller::setConstants(double KP, double KI, double KD) {
	Kp = KP;
	Ki = KI;
	Kd = KD;
}
