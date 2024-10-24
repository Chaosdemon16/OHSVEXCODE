#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Leftmotorfront = motor(PORT1, ratio36_1, false);
motor Leftmotorback = motor(PORT11, ratio36_1, false);
motor_group Leftdrivetrain = motor_group(Leftmotorback, Leftmotorfront);
motor Rightmotorfront = motor(PORT10, ratio36_1, true);
motor Rightmotorback = motor(PORT20, ratio36_1, true);
motor_group Rightdrivetrain = motor_group(Rightmotorback, Rightmotorfront);
controller Controller1 = controller(primary);
drivetrain Drivetrain = drivetrain(Leftdrivetrain, Rightdrivetrain, 319.19, 295, 40, mm, 1);
motor intake = motor(PORT15, ratio18_1, false);
motor upittyBit = motor(PORT16, ratio36_1, false);
motor topupittybit = motor(PORT17, ratio36_1, true);
motor arm = motor(PORT14, ratio36_1, false);
motor claw = motor(PORT17, ratio18_1, false);
digital_out goalGrab = digital_out(Brain.ThreeWirePort.H);
digital_out hoodLift = digital_out(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool isSuckingUpRings = false;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          Leftdrivetrain.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          Rightdrivetrain.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        Leftdrivetrain.setVelocity(drivetrainLeftSideSpeed, percent);
        Leftdrivetrain.spin(reverse);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        Rightdrivetrain.setVelocity(drivetrainRightSideSpeed, percent);
        Rightdrivetrain.spin(reverse);
      }
      if (Controller1.ButtonDown.pressing()){
        goalGrab.set(false);
      } else if (Controller1.ButtonUp.pressing()) {
        goalGrab.set(true);
      }
      if (Controller1.ButtonA.pressing()) {
        isSuckingUpRings = true;
      } else if (Controller1.ButtonB.pressing()){
        isSuckingUpRings = false;
      }
      
      if (isSuckingUpRings == true) {
        intake.spin(fwd, 90, percentUnits::pct);
        upittyBit.spin(fwd, 90, percentUnits::pct);
      } else if (isSuckingUpRings == false){
        intake.stop();
        upittyBit.stop();
      }
      if (Controller1.ButtonR1.pressing()) {
        intake.spin(fwd, 90, percentUnits::pct);
        upittyBit.spin(fwd, 90, percentUnits::pct);
        topupittybit.spin(fwd, 90, percentUnits::pct);
      } else if (Controller1.ButtonR2.pressing()){
        intake.spin(reverse, 90, percentUnits::pct);
        upittyBit.spin(reverse, 90, percentUnits::pct);
        topupittybit.spin(reverse, 90, percentUnits::pct);
      } else {
        
        intake.stop();
        upittyBit.stop();
        topupittybit.stop();
      }
      if (Controller1.ButtonL1.pressing()) {
        goalGrab.set(false);
      } else if (Controller1.ButtonL2.pressing()) {
        goalGrab.set(true);
      }
      
      
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}