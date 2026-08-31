/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// new
// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vex_log.h"
#include "remote-stuff.h"
#include "basic-func.h"
#include "auton-func.h"
#include "parameters.h"
#include "autonomous.h"
#include "timer-stuff.h"
#include "iostream"
#include "odom.h"
using namespace vex;
using std::cout;
using std::endl;

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

void pre_auton(void)
{
  setPistonW(false);
  setPistonD(true);
  // vex_log(V_Trace, "Starting main::pre_auton...");
  //  Initializing Robot Configuration. DO NOT REMOVE!
  // resetHeading();
  vexcodeInit();
  // MyOdom.resetPosOdom();
  wait(1000, msec);
  IMU.calibrate();
  while (IMU.isCalibrating())
  {
  }
  wait(1000, msec);
  fullResetForwardPos();
  resetForwardPos();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  // vex_log(V_Trace, "Completed main::pre_auton");
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
int auton_choose = 1;

void autonomous(void)
{
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  vex_log(V_Trace, "Starting main::autonomous...");
  // skills();
    
  runAuton(auton_choose);
  vex_log(V_Trace, "Completed main::autonomous");

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
// R1 intake
// R2 outtake
// L1 clamp
// L2 lift
// up auto
// down redirect
void usercontrol(void)
{
  bool lowstate = false;
  bool globalLowstate = false;
  // float print_i = 0;
  bool lmidwing = false;
  bool rmidwing = false;
  bool fwingdown = false;
  bool gwingdown = false;
  bool mlstate = false;
  bool matchloaddown = false;
  int autocollide = 0;
  bool RightPressed = 0;
  bool LeftPressed = 0;
  bool UpPressed = 0;
  bool BPressed = 0;
  bool YPressed = 0;
  bool APressed = 0;
  bool XPressed = 0;
  bool isJammed = 0;
  int liftCounter = 0;
  bool DownPressed = 0;
  bool L1Pressed = 0;
  bool L2Pressed = 0;
  int print_i = 0;
  bool pistonLState = false;
  bool pistonWState = false;
  bool pistonIState = true;
  bool pistonDState = false;
  bool pistonOState = true;
  int pressCount = 0;
  bool downState = false;
  bool BState = false;
  bool L2State = false;
  bool L1State = false;

  setPistonF(false);
  setPistonG(false);  
  vex_log(V_Trace, "Starting main::usercontrol...");

  // User control code here, inside the loop
  while (true)
  {

    int Ch1 = abbs(C1) < Joystick_LowerDeadzone ? 0 : C1;
    int Ch2 = abbs(C2) < Joystick_LowerDeadzone ? 0 : C2;
    int Ch3 = abbs(C3) < Joystick_LowerDeadzone ? 0 : C3;

    // if((abbs(Ch3) < 30 || sign(Ch3) == sign(Ch2)) && abbs(Ch2) > 80 && abbs(Ch2) > abbs(Ch1) + 40) autocollide = sign(Ch2);
    // else if((abbs(Ch3) >= 50 && !(Ch3 > 0 && autocollide == 1)) || (abbs(Ch2) < abbs(Ch1) && abbs(Ch1) > 40)) autocollide = 0;
    if (autocollide == 0)
    {

      // tank
      //moveLeft(0.6 *Ch3);
      //moveRight(0.6* Ch2);
      // arcade
      moveLeft(Ch3 + 0.6 * Ch1); // 0.8 * Ch3
      moveRight(Ch3 - 0.5 * Ch1);
    }
    else
    {
      moveLeft(autocollide * 15);
      moveRight(autocollide * 15);
    }
    // Defaults

// --- Priority handling ---
if(L1 && R1) {
  setIntakeSpeed(0);
  setPistonO(true);
  setPistonD(true);
  liftSetMode(false);
}
else {
  setPistonO(false);
    if(R1 && !L1) {
      liftSetMode(true);
        setPistonD(true);
        setIntakeSpeed(100);
        globalLowstate = false;

    }
    else if(R2) {
      liftSetMode(false);
        globalLowstate = true;
        setPistonD(true);
        setIntakeSpeed(-100);
    }
    else if(L1 && !R1) {
      liftSetMode(false);
        setPistonD(false);
        setIntakeSpeed(100);
        globalLowstate = false;
    }
    else if(L2) {
      liftSetMode(false);
      setPistonD(true);
      setIntakeSpeed(100);
      globalLowstate = false; 
    }
    else{
      setIntakeSpeed(0);
      setPistonD(true);
      liftSetMode(false);
      
    }
}
if(DOWN ){
  //downState = !downState;
  setPistonW(false);
}
else if(!DOWN && getAutonFinished()){

  setPistonW(true);

}
else{
  setPistonW(false);
}
DownPressed = DOWN;

if(BB && !BPressed){
  mlstate = !mlstate;
  setPistonI(mlstate);
  this_thread::sleep_for(2);
}
BPressed = BB;

if (BX && !XPressed){
  rmidwing = !rmidwing;
  setPistonG(rmidwing);


  this_thread::sleep_for(2);
}

XPressed = BX;

if (RIGHT && !RightPressed){
  lmidwing = !lmidwing;
  setPistonF(lmidwing);

  this_thread::sleep_for(2);
}
if(lmidwing || rmidwing){
  globalLowstate = true;
  //setPistonH(true);
}
else if(!R2 && !lowstate){
  globalLowstate = false;
  //setPistonH(false);
}

if (BA && !APressed){
  lowstate = !lowstate;
  globalLowstate = lowstate;
  //setPistonH(lowstate);
  this_thread::sleep_for(2);
}
APressed = BA;
if (UP && !UpPressed){
  //runAuton(1);
 setAutonFinished(true);
}
UpPressed = UP;
setPistonH(globalLowstate);



RightPressed = RIGHT;

YPressed = BY;



  }

  vex_log(V_Trace, "Completed main::usercontrol");
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
 

  // vex_log(V_Trace, "Starting main::main...");

  // here imu

  thread Intake(intake);
  // thread Cata(KickerTurn);
  thread OdomPosition(MyOdomPos);
  thread Doinker(doinker);
  thread Lift(lift);
  
  thread distanceSense(Distance);


  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // vex_log(V_Info, "Info message");
  //  Run the pre-autonomous function.
     
  pre_auton();
  // vex_log(V_Critical, "IMPORTANT message");

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    // cout << getHeading() << endl;
    wait(5, msec);
  }

  // vex_log(V_Error, "Exiting main::main");
}
