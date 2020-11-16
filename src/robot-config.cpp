#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightMotor = motor(PORT6, ratio18_1, true);
motor LeftMotor = motor(PORT7, ratio18_1, false);
controller Controller1 = controller(primary);
motor OrangeMotor = motor(PORT10, ratio18_1, false);
motor IntakeLeftMotor = motor(PORT2, ratio18_1, false);
motor IntakeRightMotor = motor(PORT3, ratio18_1, false);
motor IntakeMiddleLMotor = motor(PORT1, ratio18_1, false);
motor IntakeMiddleRMotor = motor(PORT4, ratio18_1, false);
/*vex-vision-config:begin*/
signature Vision__BLUE_BALL = signature (1, -729, 643, -43, 2621, 6473, 4547, 1.1, 0);
signature Vision__RED_BALL = signature (2, 8113, 10513, 9314, -2121, -985, -1554, 1.1, 0);
signature Vision__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__NAH = signature (7, 2777, 4961, 3869, -2643, -1413, -2028, 0.7, 0);
vision Vision = vision (PORT8, 111, Vision__BLUE_BALL, Vision__RED_BALL, Vision__SIG_3, Vision__SIG_4, Vision__SIG_5, Vision__SIG_6, Vision__NAH);
/*vex-vision-config:end*/

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}