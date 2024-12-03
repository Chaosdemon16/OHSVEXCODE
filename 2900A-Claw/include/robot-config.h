using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern controller Controller1;
extern motor ARM_LEFT;
extern motor ARM_RIGHT;
extern digital_out goalGrab;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 **/
void  vexcodeInit( void );