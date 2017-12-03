/*
 * Quadrotor.h - Version 1
 * Includes all libraries and functions necessary for:
 * - Radio operation
 * - Servo operation
 * - MPU6050 Accelerometer/Gyroscope data collection
 * - BMP180 Barometer data collection
 * 
 * Includes setup functions for Radio, Servo, and Accel/Gyro and Barometer
 */

 // Radio include
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Servo include
#include <Servo.h>

// MPU6050 include
#include <MPU6050_6Axis_MotionApps20.h>

// BMP180 include
#include <SFE_BMP180.h>
#include "I2Cdev.h"
#include "Wire.h"


////////////////////////
// INITIALIZE RADIO
//////////////////////
RF24 radio(7,8);       // Set up nRF24L01 radio on SPI bus plus pins 7 & 8

// Data structure to organize info received at the Quad node from the Control node
struct dataStructRx{
  float yaw_command;
  float pitch_command;
  float roll_command;
  float height_command;
};

dataStructRx rxData;

// Define which pipe carries what information
#define YAW_PIPE 1
#define PITCH_PIPE 2
#define ROLL_PIPE 3
#define HEIGHT_PIPE 4

// Setup Functions
void setup_radio();     // Set up radio
bool read_commands();      // Read incomming data from radio into structure "rxData", return true when data on all 4 channels has been read


/////////////////////
// INITIALIZE SERVOS
/////////////////////

// Instantiate servo connections to Electronic Speed Controllers (ESC's)
Servo esc_1;                 // Front left ESC
Servo esc_2;                 // Front right ESC
Servo esc_3;                 // Back right ESC
Servo esc_4;                 // Back left ESC

// Array to hold PWM signals that will be written to the ECS's
unsigned int prop_speeds[4];

// Setup Function
void setup_servos();



////////////////////////////
// INITIALIZE CONTROLLERS
////////////////////////////

// Setpoint Variable Pointers
float* yaw_setpoint;
float* pitch_setpoint;
float* roll_setpoint;
float* thrust_setpoint;

// Input Variables
float* yaw_input;
float* pitch_input;
float* roll_input;
float* thrust_input;

// Output Variables
float* yaw_output;
float* pitch_output;
float* roll_output;
float* thrust_output;



/* Setup Function
 *  MyController: Which controller to set up
 *  *_input: Pointer to the variable that will change on each Loop
 *  *_output: Pointer to the variable that will change on each controller computation
 */
void setup_controller(Controller MyController, float* _input, float* _output, float* _setpoint);


//////////////////////
// INITIALIZE MPU6050
/////////////////////

// Instantiate class
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation variables
Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat gravity;    // [x, y, z]            gravity vector

// Interrupt routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady();

// Set up MPU6050
uint8_t setup_MPU();

/////////////////////
// INITIALIZE BMP180
/////////////////////

SFE_BMP180 pressure;  // Create SFE_BMP180 object on I2C bus with address 0x77 (defined in SFE_BMP180.h header file)
double baseline;      // baseline pressure

// Setup Functions
void setup_BMP180();  // Set up BMP180
double getPressure(); // Function to take pressure measurements



///////////////////
// SETUP THE RADIO
//////////////////////////////////////////////////////////////////////////////
void setup_radio()
{
  Serial.println("--\nSetting up the radio...\n");
  Serial.println("Role: Receiver");
  
  // Radio pipe addresses for the 2 nodes to communicate.
  // Each pipe is used to send one of either Yaw, Pitch, Roll, or Height setpoint commands
  const uint64_t pipes[4] = {0xF0F0F0F0A1LL, 0xF0F0F0F0B2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0D4LL};
  radio.begin();

  // Can reduce payload size if it's OK to send something smaller than a float (current)
  // Payload size defaults to 32-bit (float) but here it is reduced to 16-bits (int)
  //radio.setPayloadSize(16);

  // Enable auto-ack for payloads on all pipes
  radio.enableDynamicPayloads();

  // Open all 4 pipes for reading
  radio.openReadingPipe(YAW_PIPE,pipes[0]);       // Open reading pipe for Yaw setpoint commands
  radio.openReadingPipe(PITCH_PIPE,pipes[1]);     // Open reading pipe for Pitch setpoint commands
  radio.openReadingPipe(ROLL_PIPE,pipes[2]);      // Open reading pipe for Roll setpoint commands
  radio.openReadingPipe(HEIGHT_PIPE,pipes[3]);    // Open reading pipe for Height setpoint commands

  // Start listening
  radio.startListening();
  Serial.println("\nRadio Setup complete.  Now listening.\n");
}

////////////////////
// SET UP THE SERVOS
//////////////////////////////////////////////////////////////////////////////////////////
// Attach the servo objects to the ESC's that they will write PWM signal to
void setup_servos(){
  esc_1.attach(9);
  esc_2.attach(6);
  esc_3.attach(5);
  esc_4.attach(3);
}


///////////////////////////
// SET UP THE CONTROLLERS
///////////////////////////////////////////////////////////////////////////////////////////////
void setup_controller(Controller MyController, *_input, *_output, *_setpoint);{
  // Set Input
  MyController.Input = _input;
  
  // Set Output
  MyController.Output = _output;
  
  // Set Setpoint
  MyController.Setpoint = _setpoint;
  
  // Set Mode
  MyController.setMode("AUTOMATIC");
  
  // Set Sample Time to 1ms
  MyController.setSampleTime(1);
  
  // Set Output Limits
  MyController.setOutputLimits(0, 255);       /* Should be w_min and w_max*/
  
  // Set Default Setpoint
  MyController.setSetpoint(0);
  
  // Set Default Constants
  MyController.setConstants(0, 0, 0);
}


/////////////////
// SETUP THE MPU
///////////////////////////////////////////////////////////////////////////////////////
// Set up Interrupt routine
void dmpDataReady() {
    mpuInterrupt = true;
}


uint8_t setup_MPU()
{
  Serial.println("--\nSetting up the MPU...\n");
  Wire.begin();

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing MPU6050 connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  Serial.println("\nMPU Setup complete.\n");
  return devStatus;
}

//////////////////
// SETUP BMP180
//////////////////////////////////////////////////////////////////////////////////////
void setup_BMP180(){
  // Initialize BMP180
  Serial.println(pressure.begin() ? F("BMP180 connection successful") : F("BMP180 connection failed (disconnected?)"));

  // Get baseline pressure
  baseline = getPressure();
  Serial.print("Baseline Pressure: "); Serial.print(baseline); Serial.println(" mb");
}

// Function to read pressure
double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}



////////////////////////////
// READ RADIO INFORMATION
/////////////////////////////////////////////////////////////////////////////////////////////
// Return true if all 4 channels successfully read data
bool read_commands(){
  bool done[4] = {false, false, false, false};
  
  // Check if there is data ready on each pipe
  byte pipeNum = 1;
  for (pipeNum = 1; pipeNum <=4; pipeNum++){
    if (radio.available(&pipeNum)){
      switch(pipeNum){
         // 1: Yaw pipe
         case 1 : radio.read(&rxData.yaw_command, sizeof(float));
         
         // 2: Pitch pipe
         case 2 : radio.read(&rxData.pitch_command, sizeof(float));
         
         // 3: Roll pipe
         case 3 : radio.read(&rxData.roll_command, sizeof(float));
         
         // 4: Height pipe
         case 4 : radio.read(&rxData.height_command, sizeof(float));
      }
    }
  }
  return done[0] && done[1] && done[2] && done[3];
}




/////////////////////
// QUADROTOR CLASS
///////////////////////////////////////////////////////////////////////////
class Quadrotor
 {
   public:
   // Constructor
   Quadrotor();
   // Destructor
   ~Quadrotor();
   
   // Declare updater functions
   void thrust_update(int thrustUpdate);
   void yaw_update(int yawUpdate);
   void pitch_update(int pitchUpdate);
   void roll_update(int rollUpdate);
   void update_propSpeeds(int PROPSPEEDS[4]);
   
   // Declare get functions
   int get_thrust();
   int get_ypr();
   int get_propSpeeds();
   
   // Variables
   Servo escFL;
   Servo escFR;
   Servo escBL;
   Servo escBR;
   
   int propSpeeds[4];

 };

// Contructor
Quadrotor::Quadrotor()
{
  // Write 0 to all servos
  update_propSpeeds([0,0,0,0]);
  
  // Read all servo speeds
  propSpeeds = get_propSpeeds();
}


void Quadrotor::update_propSpeeds(int PROPSPEEDS[4])
{
  escFL.write(PROPSPEEDS[0]);
  escFR.write(PROPSPEEDS[1]);
  escBL.write(PROPSPEEDS[2]);
  escBR.write(PROPSPEEDS[3]);
}


void Quadrotor::thrust_update(int thrustUpdate)
{
  // thrust = up/down
  // thrust_update applies the same additional thrust to each ESC
  propSpeeds = get_propSpeeds() + thrustUpdate;
  update_propSpeeds(propSpeeds);
}

void Quadrotor::yaw_update(int yawUpdate)
{
  // yaw = twist CW/CCW
  // not sure how to do this yet
}

void Quadrotor::pitch_update(int pitchUpdate)
{
  // pitch = angle up/down on y axis
  // pitch_update speeds up the front ESCs and slows down the rear ESCs by a factor.
  propSpeeds = get_propSpeeds() + [pitchUpdate, pitchUpdate, -pitchUpdate, -pitchUpdate];
  update_propSpeeds(propSpeeds);
}

void Quadrotor::roll_update(int rollUpdate)
{
  // roll = angle up/down on x axis
  // roll_update speeds up the right ESCs and slows down the left ESCs by a factor.
  propSpeeds = get_propSpeeds() + [-rollUpdate, rollUpdate, -rollUpdate, rollUpdate];
  update_propSpeeds(propSpeeds);
}

int Quadrotor::get_thrust()
{
  // thrust = mass*gravity + z_acceleration
  // z_acceleration comes from gyro
}

int Quadrotor::get_ypr()
{
  // ypr data comes from DMP on the gyro
}


int Quadrotor::get_propSpeeds()
{
  propSpeedsRead = [escFL.read(), escFR.read(), escBL.read(), escBR.read()];
  return propSpeedsRead;
}


void Quadrotor::StabilizeAircraft()
{
  // PID stuff here
}


/////WRONG PLACE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}





//
// Header Functions
//


