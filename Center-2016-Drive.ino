/* Standard_Base_2015_Drive
 This is downloaded to Linemen, WR, or RB Drive Arduino (by the speed controlelr).
 Make sure the Xbee Shield switch is set to USB or DLINE when uploading,
 but then switch it back to XBEE or UART when you are done.
 
 Author : Kat Landers
 Date   : April 2015
 */

 
/***************************************************
 * Select Center Robot
 ***************************************************/
/*--------Uncomment the Robot that you are programming----------*/

//#define FIVEHOURS
#define KURT

/***************************************************
 * Include Statements
 ***************************************************/
#include <ND/Xbox/RXDATA.h>
#include <ND/Actuation/Motors.h>
#include <ND/Xbee/ND_XBEE.h>
#include <ND/Sensors/Sensor.h>
#include <Servo.h>

using namespace ND;

/***************************************************
 * Addressing and Other Constants
 ***************************************************/
// Baud Rate
#define BAUD_RATE 38400

// Addresses
#ifdef KURT
  #define CONTROLLER_ADDRESS 0x9011
#endif

#ifdef FIVEHOURS
  #define CONTROLLER_ADDRESS 0x9012
#endif

// Gain Settings
#define THROTTLE_GAIN 140 // for drive wheels
#define THROTTLE_GAIN_LOW 75 // slow speed for drive wheels
#define STEERING_GAIN 50 // for drive wheels
#define STEERING_GAIN_LOW 40 // slow speed for drive wheels

// Offsets
#define ZERO_OFFSET 0

// Controller Mode constants
#define MANUAL_MODE 0
#define AUTOMATIC_MODE 1
#define CALIBRATION_MODE 2
#define TUTORIAL_MODE 3

#define SPIN_HARD 80 // this value is used to determine the spin in place for spinning with RB and LB ... 0-255
#define BOUNCEDELAY 20

//Center Specific Values

#ifdef KURT
  #define RELEASE_SERVO_UPPER 85 //Starting position
  #define RELEASE_SERVO_LOWER 50
  #define ARM_SERVO_UPPER 78 //Starting position: All the way back
  #define ARM_SERVO_LOWER 35
  #define ALIGNMENT_SERVO_UPPER 41 //Starting positon: UPPER position is a lower value in degrees
  //#define ALIGNMENT_SERVO_LOWER 60
#endif

#ifdef FIVEHOURS
  #define RELEASE_SERVO_UPPER 90
  #define ARM_SERVO_UPPER 90
  #define ALIGNMENT_SERVO_UPPER 90 //Up position is a lower value in degrees
  #define RELEASE_SERVO_LOWER 0
  #define ARM_SERVO_LOWER 0
  //#define ALIGNMENT_LOWER 0
#endif

/***************************************************
 * Pin Numbers
 ***************************************************/
// Pins 0 and 1 are RX and TX, used by the Xbees
#define TACKLE_SENSOR_PIN 8 // first interrupt ... this is digital pin 2
#define LEFT_MOTOR_PIN 10 // interior wire
#define RIGHT_MOTOR_PIN 11 // interior wire
//Center Specific
#define RELEASE_SERVO_PIN 4 // two position that pushes the ball out of the claw
#define ARM_SERVO_PIN 5 // (two position) the one that spins the arm to push the arm out
#define ALIGNMENT_SERVO_PIN 3 // the arrow that goes down (two-position)
#define INTERRUPT_PIN 2 //


/***************************************************
 * Function Prototypes and Namespaces
 ***************************************************/
void updateDriveF(boolean);
void control(signed char, signed char, signed char);
void ISR_Alignment();

using namespace ND;

/***************************************************
 * Debugging Variables
 ***************************************************/
//#define DEBUG_TIME
#ifdef DEBUG_TIME
long timedbg;
#endif

//#define DEBUG
#ifdef DEBUG // variables used in debugging
int * vals1, *vals2;
motor_offsets mo;
uint8_t val;
#endif

/***************************************************
 * Global Variables
 ***************************************************/

// START DRIVE VARS
// Motor Class
Servo lm; 
Servo rm;
Motors Wheels(&lm, &rm, THROTTLE_GAIN, STEERING_GAIN, ZERO_OFFSET); // used to control a pair of steering motors
boolean blnSlowGains = false;

// Data Classes
ND_XBEE ndbee; // used for RF communication
Rx16Response rx16 = Rx16Response();
RXDATA rxdata; // used to handle the incoming data from the xbox controller... button presses, releases, etc.

// Sensor Class
bool tackled = false; // determines if you are tackled or not
Tackle_Sensor tackle_sensor(TACKLE_SENSOR_PIN);
uint8_t message[1] = {1};
// END DRIVE VARS


// Center Servo VARS
Servo release_servo; 
Servo arm_servo; 
Servo alignment_servo;
volatile boolean alignment_toggle = false;
boolean arm_engaged = false;
boolean release_engaged = false;
volatile boolean down = false;
double alignment_servo_value = double(ALIGNMENT_SERVO_UPPER);

int count;

/***************************************************
 * Setup
 ***************************************************/
void setup() {
  
  // Setup/initialize servo handles
  release_servo.attach(RELEASE_SERVO_PIN);
  arm_servo.attach(ARM_SERVO_PIN);
  alignment_servo.attach(ALIGNMENT_SERVO_PIN);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(0,ISR_Alignment,FALLING);
  alignment_servo.write(ALIGNMENT_SERVO_UPPER);
  arm_servo.write(ARM_SERVO_UPPER);
  release_servo.write(RELEASE_SERVO_UPPER);
  
  // Setup/initialize motors 
  lm.attach(LEFT_MOTOR_PIN); 
  rm.attach(RIGHT_MOTOR_PIN);
  Wheels.steering_gain = STEERING_GAIN;
  Wheels.throttle_gain = THROTTLE_GAIN;
  Wheels.Stop();
  Serial.begin(BAUD_RATE);
  // setup the RF communication
  ndbee = ND_XBEE();
  ndbee.xbee.setSerial(Serial);
  rxdata = RXDATA();
  Serial.println("STARTING...");
  delay(2000); // delay 2 seconds
}

/***************************************************
 * Interrupt Routines
 ***************************************************/
// ??????
void ISR_Alignment(){
  down = true;
  alignment_toggle = false;
}


/***************************************************
 * Loop
 ***************************************************/
void loop() {

  static long timeout_timer;

  /************************************************************
   * PART 1. Read the Tackle Sensor
   ************************************************************/
  tackle_sensor.Read();

  /************************************************************
   * PART 2. Read the Packet
   ************************************************************/
  if (ndbee.Read()) {
    // got something -- Check the Address
    if (ndbee.Address16() == CONTROLLER_ADDRESS) {
      rxdata.set(ndbee.getData());
#ifdef DEBUG
      Serial.print("Succes");
      rxdata.debugprint(&Serial);
#endif
    }
#ifdef DEBUG
    Serial.print(ndbee.Address16());
    Serial.print(" ADDR\t");
    Serial.println("gotsomething");
#endif
    timeout_timer = millis();
  } 
  else {
    rxdata.NoPacket = true;
    if (timeout_timer + 250 <= millis()) rxdata.ZERO(); // after so many cycles of no packets send default signals to actuators...
    //if (rxdata.TimeOut()) rxdata.ZERO(); // after so many cycles of no packets cut the power...
  }

  /************************************************************
   * PART 3. Select Your Mode
   ************************************************************/
  switch (rxdata.get(MODE, 2))
  {

  case MANUAL_MODE:

    //Servo States
    if(rxdata.ButtonPress(A) && alignment_toggle == false){
      alignment_toggle = true;
    }
    else if(rxdata.ButtonPress(A) && alignment_toggle == true){
      alignment_toggle = false;
    }

    if(rxdata.ButtonPress(X) && arm_engaged == false){
      arm_engaged = true;
    } 
    else if(rxdata.ButtonPress(X) && arm_engaged == true){
      arm_engaged = false;
    }

    if(rxdata.ButtonPress(Y) && !release_engaged && arm_engaged){
      release_engaged = true;
    } 
    else if(rxdata.ButtonPress(Y) && release_engaged && arm_engaged){
      release_engaged = false;
    }


    //Servo Outputs
    if(alignment_toggle && !down){
      alignment_servo_value += 0.05;
      alignment_servo.write(int(alignment_servo_value));
    } else if(alignment_toggle && down){
      alignment_servo_value = double(ALIGNMENT_SERVO_UPPER);
      alignment_servo.write(alignment_servo_value);
      alignment_toggle = false;
      down = false;
    }

    if(arm_engaged){
      arm_servo.write(ARM_SERVO_LOWER);
    } 
    else if(!arm_engaged){
      arm_servo.write(ARM_SERVO_UPPER);
    }

    if(release_engaged){
      release_servo.write(RELEASE_SERVO_LOWER);
    } 
    else if(!release_engaged){
      release_servo.write(RELEASE_SERVO_UPPER);
    }


    updateDriveF(false); // Always update the drive control
    break;

  case AUTOMATIC_MODE:
    updateDriveF(false); // Always update the drive control
    break;

  case CALIBRATION_MODE:

    /* CALIBRATION MODE (Mode == 3) for setting the motor offsets
     Here is how you calibrate ...
     There are forward and reverse motor offsets for when you are going forward or backward and they are on 1 OF THE 2 MOTORS!!
     First, let's calibrate forward.  Go forward and see which motor is faster. If it goes straight, no calibration necessary :) ... move on to calibrate reverse.
     However, if it fades to the right (on a smooth surface) then choose the forward motor as the left motor because the left motor is faster and we need to knock it down.
     The opposite is true - if it fades to the left, select the right motor as the offset motor for forward.
     Then just keep increasing the offset until you get reasonable performance at high speeds
     Do the same thing for reverse...
     Afterwards, save these values by pressing START ... try not to save too often as the arduino only has limited writes to the EEPROM before it's memory is done :) (~100,000 to be exact)
     
     Now here are the controls
     
     Forward: (DO NOT have Right Trigger pressed!!)
     Picking a side: 2 choices
     1) X button : chooses left motor as offset motor (faster motor)
     2) B button : chooses right motor as offset motor (faster motor)
     Incrementing and Decrementing offset:
     1) Y Button : Increases offset (offset can go from 0 to 255) ... you should not be up to 255
     2) A Button : Decreases offset (offset can go from 0 to 255) ... you should not be up to 255
     Reverse: (Same as forward but just press the Right Trigger down
     */

    // Just enable the analog driving feature in this mode
    Wheels.Controller(rxdata.getJoystick(LY), rxdata.getTrigger(L2));
    if (rxdata.getTrigger(R2)) { // calibrate the reverse offset if R2 is pressed
      Wheels.cdata.fwd_rev = CALIBRATE_REVERSE;
      Wheels.cdata.left_right = CALIBRATE_MOTOR_REVERSE;
    } 
    else { // calibrate the forward offset if R2 is not pressed
      Wheels.cdata.fwd_rev = CALIBRATE_FORWARD;
      Wheels.cdata.left_right = CALIBRATE_MOTOR_FORWARD;
    }

    // Deal with the Buttons
    if (rxdata.ButtonPress(A)) Wheels.Calibrate(  -1  ,  Wheels.cdata.fwd_rev);
    if (rxdata.ButtonPress(Y)) Wheels.Calibrate(   1  ,  Wheels.cdata.fwd_rev);
    if (rxdata.ButtonPress(X)) Wheels.Calibrate(  MOTOR_LEFT   ,  Wheels.cdata.left_right);
    if (rxdata.ButtonPress(B)) Wheels.Calibrate(  MOTOR_RIGHT  ,  Wheels.cdata.left_right);
    if (rxdata.ButtonPress(START)) Wheels.Calibrate(0, CALIBRATE_SAVE_OFFSETS);
#ifdef DEBUG
    mo = Wheels.offs();
    Serial.println(); 
    Serial.print("fwd: "); 
    Serial.print(mo.forward()); 
    Serial.print(" \trev: ");
    Serial.print(mo.reverse()); 
    Serial.print(" \tdir: "); 
    Serial.println(mo.direct());
#endif
    break;

  case TUTORIAL_MODE: // TUTORIAL MODE:

    updateDriveF(true); // Always update the drive control
    break;

  default: 
    break;
  }

#ifdef DEBUG_TIME
  Serial.print("One loop = "); 
  Serial.print(micros() - timedbg); 
  Serial.println(" microseconds");
  timedbg = micros();
#endif
}

/***************************************************
 * Drive Control Function
 ***************************************************/
void control(signed char throttle, signed char steering, signed char side2side) {
  if (!tackle_sensor.tackled()) {
    //if (!tackled)
    Wheels.Controller(throttle, steering);
#ifdef DEBUG
    vals1 = Wheels.vals(1);
    vals2 = Wheels.vals(2);
    Serial.print("Left: "); 
    Serial.print(vals1[0]); 
    Serial.print(" \t");
    Serial.print("Right: "); 
    Serial.println(vals2[0]);
#endif
  }
  else { // stop the motors if tackled
    Wheels.Stop();
#ifdef DEBUG
    Serial.println("TACKLED");
#endif
  }
}

/***************************************************
 * Drive Function
 ***************************************************/
void updateDriveF(boolean blnTutorialMode) {
  if (tackle_sensor.tackled()) Wheels.Stop(); // brake
  else {
    if (rxdata.getTrigger(R2) && rxdata.getTrigger(L2)) { // L2 + R2 is brake
      if (Wheels.moving) Wheels.Stop(); // brake
    } 
    else if (blnTutorialMode) {
      Wheels.steering_gain = STEERING_GAIN_LOW;
      Wheels.throttle_gain = THROTTLE_GAIN_LOW;
    } 
    else if (rxdata.ButtonPress(R3)) { // Press R3 to toggle gains
      if (blnSlowGains) {
        blnSlowGains = false;
        Wheels.steering_gain = STEERING_GAIN;
        Wheels.throttle_gain = THROTTLE_GAIN;
      } 
      else {
        blnSlowGains = true;
        Wheels.steering_gain = STEERING_GAIN_LOW;
        Wheels.throttle_gain = THROTTLE_GAIN_LOW;
      }
    }
    if (rxdata.getTrigger(R2))
      Wheels.TurnRight(rxdata.getJoystick(LY), rxdata.getTrigger(R2)); // high speed turning right
    else if (rxdata.getTrigger(L2))
      Wheels.TurnLeft(rxdata.getJoystick(LY), rxdata.getTrigger(L2)); // high speed turning left
    else if (rxdata.get(R1))
      Wheels.Move(SPIN_HARD, -SPIN_HARD); // spin fast right (CW looking at ground)
    else if (rxdata.get(L1))
      Wheels.Move(-SPIN_HARD, SPIN_HARD); // spin fast left (CW looking at ground)
    else
      control(rxdata.getJoystick(LY), rxdata.getJoystick(RX), rxdata.getJoystick(LX)); // control
  }
}

