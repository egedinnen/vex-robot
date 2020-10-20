/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightMotor           motor         10              
// LeftMotor            motor         1               
// Controller1          controller                    
// ArmMotor             motor         3               
// ClawMotor            motor         7               
// AnkleMotor           motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Timer.reset();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Ayarlar
double distkP = 2;//Oranın ayarlama katsayısı
double distkI = 0.001;//integral ayarlama katsayısı
double distkD = 5;//türev ayarlama katsayısı

double diffkP = 0.1;//Oranın ayarlama katsayısı
double diffkI = 0.005;//integral ayarlama katsayısı
double diffkD = 0.05;//türev ayarlama katsayısı

double driveTarget;
double distSpeed;
double diffSpeed;

double distError; // Sensörden alınan değer - istenilen değer = hata
double diffError;

double prevDistError; // 20 ms önceki hata
double prevDiffError;

double diffIntegral;
double distIntegral;

double distDerivative; // hata - önceki hata -> hızı bulmuş olur. Buna göre hatayı 0'a indirene kadar robotun hızıyla oynar.
double diffDerivative;



double rightMotorPower;
double leftMotorPower;

bool enableDrivePID = true; // aktif edilebilirlik koyuyoruz çünkü usercontrol sürecinde PID sürüş sisteminin kullanılmasını istemiyoruz. Sadece otonom zamanında kullanılacak.

void resetEncoders(){
  RightMotor.setPosition(0,degrees);
  LeftMotor.setPosition(0,degrees);
}

void driveStraight(int distance) // cm
{
  resetEncoders();
  while (1)
  {
    double leftEncoder = (LeftMotor.position(degrees) / 360) * 31.92; // degrees to cm
    double rightEncoder = (RightMotor.position(degrees) / 360) * 31.92;

    distError = distance - ((leftEncoder + rightEncoder)/2); //Calculate distance error
    diffError = leftEncoder - rightEncoder; //Calculate difference error

    // Find the integral ONLY if within controllable range AND if the distance error is not equal to zero
    if( std::abs(distError) < 60 && distError != 0)
    {
      distIntegral = distIntegral + distError;
    }
    else
    {
      distIntegral = 0; //Otherwise, reset the integral
    }

    // Find the integral ONLY if within controllable range AND if the difference error is not equal to zero
    if( std::abs(diffError) < 60 && diffError != 0)
    {
      diffIntegral = diffIntegral + diffError;
    }
    else
    {
      diffIntegral = 0; //Otherwise, reset the integral
    }

    distDerivative = distError - prevDistError; //Calculate distance derivative
    distDerivative = diffError - prevDiffError; //Calculate difference derivative

    prevDistError = distError; //Update previous distance error
    prevDiffError = diffError; //Update previous difference error

    distSpeed = (distError * distkP) + (distIntegral * distkI) + (distDerivative * distkD); //Calculate distance speed
    diffSpeed = (diffError * diffkP) + (diffIntegral * diffkI) + (diffDerivative* diffkD); //Calculate difference (turn) speed

    LeftMotor.spin(forward, distSpeed - diffSpeed, voltageUnits::volt); //Set motor values
    RightMotor.spin(forward, distSpeed + diffSpeed, voltageUnits::volt); //Set motor values 

    rightMotorPower = distSpeed + diffSpeed;
    leftMotorPower = distSpeed - diffSpeed;

    Brain.Screen.print(LeftMotor.position(degrees));
    Brain.Screen.newLine();
    
    if (distError == 0){
      break;
    }
  }
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void autonomous(void) {
  driveStraight(100);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  enableDrivePID = false;

  //Drivetrain
  double turnImportance = 0.5;
  while (1) {
    double forwardVal = Controller1.Axis3.position(percent);
    double turnVal = Controller1.Axis1.position(percent);

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1-(std::abs(turnVolts)/12.0) * turnImportance);
    
    LeftMotor.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RightMotor.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    //Arm Control
  bool topLeftButton = Controller1.ButtonL1.pressing();
  bool bottomLeftButton = Controller1.ButtonL2.pressing();

  if (bottomLeftButton){
    ArmMotor.spin(forward, 6.0, voltageUnits::volt);
  }
  else if (topLeftButton){
    ArmMotor.spin(forward, -6.0, voltageUnits::volt);
  }else {
    ArmMotor.spin(forward, 0.0, voltageUnits::volt);
  }

  //Claw Control
  bool xButton = Controller1.ButtonX.pressing();
  bool bButton = Controller1.ButtonB.pressing();
  

  if (xButton){
    ClawMotor.spin(forward, 4.0, voltageUnits::volt);
  }
  else if (bButton){
    ClawMotor.spin(forward, -4.0, voltageUnits::volt);
  }else {
    ClawMotor.spin(forward, 0.0, voltageUnits::volt);
  }

  //Ankle Control
  bool topRightButton = Controller1.ButtonR1.pressing();
  bool bottomRightButton = Controller1.ButtonR2.pressing();

  if (bottomRightButton){
    AnkleMotor.spin(forward, 6.0, voltageUnits::volt);
  }
  else if (topRightButton){
    AnkleMotor.spin(forward, -6.0, voltageUnits::volt);
  }else {
    AnkleMotor.spin(forward, 0.0, voltageUnits::volt);
  }
}
  }

  
  
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
