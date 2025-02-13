#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT10, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT11, ratio18_1, true);
motor rightMotorB = motor(PORT20, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor Conveyer = motor(PORT5, ratio6_1, false);
motor Intake = motor(PORT14, ratio18_1, false);
motor_group Conveyerbelt = motor_group(Conveyer, Intake);
motor arm = motor(PORT17, ratio36_1, false);
motor ladyBrown = motor(PORT13, ratio36_1, false);
digital_out claw = digital_out(Brain.ThreeWirePort.H);
digital_out goalGrab = digital_out(Brain.ThreeWirePort.F);
line Color = line(Brain.ThreeWirePort.D);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
int speed = 40;
int fishSpeed = 40;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool goalExtended = true;
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 7 && drivetrainLeftSideSpeed > -7) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 7 && drivetrainRightSideSpeed > -7) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control Conveyer
      if (Controller1.ButtonR1.pressing()) {
        Conveyer.spin(fwd, speed, percentUnits::pct);
        Intake.spin(fwd, 90, percentUnits::pct);
        Controller1.Screen.print("Stand Clear of Conveyer!");
      } else if (Controller1.ButtonR2.pressing()) {
        Conveyer.spin(reverse, 70, percentUnits::pct);
        Intake.spin(reverse, 90, percentUnits::pct);
      } else if (Controller1.ButtonX.pressing()){
        if (speed == 40) {
          speed = 90;
          
          }
      } else if (Controller1.ButtonY.pressing()) { 
        if (speed == 90) {
          speed = 40;
        }
      } else {
        Conveyer.stop(hold);
        Intake.stop(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      
      if (Controller1.ButtonDown.PRESSED) {
        if (goalExtended == false) {
          goalGrab.set(true);
          claw.set(true); 
          goalExtended = true;
        } else if (goalExtended == true) {
          goalGrab.set(false);
          claw.set(false);
          goalExtended = false;
        }
      }
      if (Controller1.ButtonL1.pressing()) {
        ladyBrown.spin(fwd, speed, percentUnits::pct);
      } else if (Controller1.ButtonL2.pressing()) {
        ladyBrown.spin(reverse, 30, percentUnits::pct);
      } else {
        ladyBrown.stop(hold);
      }
      if (Controller1.ButtonA.PRESSED) {
        ladyBrown.spinToPosition(35, degrees, true);
      } else if (Controller1.ButtonB.PRESSED) {
        ladyBrown.spinToPosition(145, degrees, false);
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