using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern drivetrain Drivetrain;
extern motor Conveyer;
extern motor Intake;
extern motor ladyBrown;
extern motor_group Conveyerbelt;
extern digital_out goalGrab;
extern digital_out claw;
extern bool alliance;
extern char colora;
extern int speed;
extern int fishSpeed;
extern int fishPosition;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );