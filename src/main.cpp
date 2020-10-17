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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Ayarlar
double kP = 0.5;//Oranın ayarlama katsayısı
double kI = 0.5;//integral ayarlama katsayısı
double kD = 0.5;//türev ayarlama katsayısı
//Dönüş Ayarları
double turnkP = 0.5;//Oranın ayarlama katsayısı
double turnkI = 0.5;//integral ayarlama katsayısı
double turnkD = 0.5;//türev ayarlama katsayısı
//Otonom Ayarları
double desiredValue = 0;//istenilen değer
double desiredTurnValue = 0;//istenilen dönme değeri


double error; // Sensörden alınan değer - istenilen değer = hata
int prevError = 0; // 20 ms önceki hata
int derivative; // hata - önceki hata -> hızı bulmuş olur. Buna göre hatayı 0'a indirene kadar robotun hızıyla oynar.
double totalError; // toplam hata + hata = toplam hata
double lastTimeStamp = vex::timer::system();
double dt = vex::timer::system() - lastTimeStamp;//Süre


double turnError; // Sensörden alınan değer - istenilen değer = hata
int turnPrevError = 0; // 20 ms önceki hata
int turnDerivative; // hata - önceki hata -> hızı bulmuş olur. Buna göre hatayı 0'a indirene kadar robotun hızıyla oynar.
double turnTotalError; // toplam hata + hata = toplam hata

bool resetDriveSensors = false;//ne zaman sürüş sensörlerini iptal etmek istesek bu bool değişkenini true yaparak sensör verilerini sıfırlayabiliriz.

bool enableDrivePID = true; // aktif edilebilirlik koyuyoruz çünkü usercontrol sürecinde PID sürüş sisteminin kullanılmasını istemiyoruz. Sadece otonom zamanında kullanılacak.

int drivePID(){
  while(enableDrivePID){
    //Sensörleri sıfırlayan method
    if (resetDriveSensors){
      resetDriveSensors = false;
      LeftMotor.setPosition(0,degrees);
      RightMotor.setPosition(0,degrees);
    }

    //İki motorunda konumunu hesapla
    int leftMotorPosition = LeftMotor.position(degrees);
    int rightMotorPosition = RightMotor.position(degrees);


    /////////////////////////////////////////////////////////
    // İleri Geri Hareket PID'si
    /////////////////////////////////////////////////////////
    int averagePosition = (leftMotorPosition + rightMotorPosition)/2; // ortalamasını alarak direkt toplam gitmesi gereken yolu almasını söyleyeceğiz. Çünkü her iki tarafta çalışıyor olacak. Birine ayrı değer birine ayrı değer vermek durumunda kalmayacağız.
    
    //Oransal
    error = desiredValue - averagePosition;

    //İntegral
    totalError += error * dt;

    //Türevsel
    derivative = (error - prevError) / dt;

    //kod
    double pidMotorPower = (error * kP + totalError * kI + derivative * kD);



    /////////////////////////////////////////////////////////
    // Dönüş PID'si
    /////////////////////////////////////////////////////////
    int turnDifference = leftMotorPosition - rightMotorPosition; // Motorların birbirine olan farkını alarak farklılık değerini pid döngüsüne yolluyoruz.
    
    //Oransal
    turnError = turnDifference - desiredTurnValue;
    //İntegral
    turnTotalError += turnError;
    //Türevsel
    turnDerivative = turnError - turnPrevError;
    //kod
    double pidTurnPower = (turnError * turnkP + /*turnTotalError * turnkI*/ + turnDerivative * turnkP);
    
    
    
    /////////////////////////////////////////////////////////
    //Motorların hareketi
    /////////////////////////////////////////////////////////
    LeftMotor.spin(forward, pidMotorPower, voltageUnits::volt);
    RightMotor.spin(forward, pidMotorPower, voltageUnits::volt);
    
    //Hata döngüsünü tekrarlama
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }
  return 1;
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
  resetDriveSensors = true;//Otonom sürecine başlamadan önce sensörleri sıfırlıyoruz.
  desiredValue = 400;//PID döngüsüyle ulaşmak istediğimiz mesafe
  vex::task driveWithPID(drivePID);//İstediğimiz mesafeyi gitmesi için sürüş pid döngümüzü çalıştırıyoruz
  vex::task::sleep(10000);
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
  }

  //Arm Control
  bool topLeftButton = Controller1.ButtonL1.pressing();
  bool bottomLeftButton = Controller1.ButtonL2.pressing();

  if (topLeftButton){
    ArmMotor.spin(forward, 12.0, voltageUnits::volt);
  }
  else if (bottomLeftButton){
    ArmMotor.spin(forward, -12.0, voltageUnits::volt);
  }else {
    ArmMotor.spin(forward, 0.0, voltageUnits::volt);
  }

  //Claw Control
  bool topRightButton = Controller1.ButtonL1.pressing();
  bool bottomRightButton = Controller1.ButtonL2.pressing();

  if (topRightButton){
    ClawMotor.spin(forward, 12.0, voltageUnits::volt);
  }
  else if (bottomRightButton){
    ClawMotor.spin(forward, -12.0, voltageUnits::volt);
  }else {
    ClawMotor.spin(forward, 0.0, voltageUnits::volt);
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
