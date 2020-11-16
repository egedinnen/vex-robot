using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern motor RightMotor;
extern motor LeftMotor;
extern controller Controller1;
extern motor OrangeMotor;
extern motor IntakeLeftMotor;
extern motor IntakeRightMotor;
extern motor IntakeMiddleLMotor;
extern motor IntakeMiddleRMotor;
extern signature Vision__BLUE_BALL;
extern signature Vision__RED_BALL;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__NAH;
extern vision Vision;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );