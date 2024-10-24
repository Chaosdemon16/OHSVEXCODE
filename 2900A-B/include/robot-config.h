using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern controller Controller1;
extern motor intake;
extern motor upittyBit;
extern motor topupittybit;
extern motor arm;
extern motor claw;
extern digital_out goalGrab;
extern digital_out hoodLift;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 **/
void  vexcodeInit( void );