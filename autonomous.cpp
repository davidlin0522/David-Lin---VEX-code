#include "autonomous.h"
#include "parameters.h"
#include "timer-stuff.h"
#include "robot-config.h"
#include "odom.h"
#include "iostream"

#include "vex_log.h"
#include "skills.h"
#include "string.h"
using std::cout;
using std::endl;
using namespace std;

void autonInit(void)
{
  vex_log(V_Trace, "Starting autonomous::autonInit...");

  motorResetFull();
  resetHeading();
  resetForwardPos();
  // reset IMU readings and odom

  vex_log(V_Trace, "Completed autonomous::autonInit");
}
void test()
{
  //setPistonW(false);
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);

  resetForwardPos();
  resetHeading();
  MyOdom.setHeadingOdom(0);
  // start here
  //rotateToHeading(90);
  //swingToHeading(-90, 50, true);
  //PIDPosForwardAbs(600);
}


void leftfourball(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);

  resetForwardPos();

  // heading mirrored
  MyOdom.setHeadingOdom(-33);

  setDoink(false);
  setDoinkR(false);
  setMiddleState(false);
  liftSetMode(0);
  setIntakeSpeed(100);

  // same
  softStartTimerForward(100, 100, 380);

  setDoinkR(true);

  // swing flipped (1 → -1)
  softStartTimerSwing(-1, 100, 100, 390);

  // same
  softStartTimerForward(100, 20, 985);

  // rotate flipped
  softStartTimerRotate(100, 100, 75);

  setIntakeSpeed(0);
  setDoink(true);

  // same
  softStartTimerForward(-100, -40, 450);

  setIntakeSpeed(100);

  softStartTimerForward(-40, 0, 1100);

  setDoinkR(false);
  setIntakeSpeed(0);
  setDoink(false);

  // headings flipped
  rotateToHeading(-180);
  rotateToHeading(-110);

  softStartTimerForward(100, 0, 65);

  // flipped
  rotateToHeading(175);

  PIDPosForwardAbs(-600);
}


void rightfourball(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos(); // add this
  this_thread::sleep_for(100);

  resetForwardPos();
  MyOdom.setHeadingOdom(33);
  setDoink(false); // storage. setDoink(true) is not storing
  setDoinkR(false); // matchload. setDoinkR(true) is matchload bar down
  setMiddleState(false); // mid goal intake state
  liftSetMode(0); // mid goal up, liftSetMode(1); midgoal down
  setIntakeSpeed(100);


  softStartTimerForward(100, 100, 380);
  setDoinkR(true);                        
  softStartTimerSwing(1, 100,100, 380);

  softStartTimerForward(100, 20, 850);
  softStartTimerRotate(-100,-100,95);
  setIntakeSpeed(0);
  setDoink(true);


  softStartTimerForward(-100, -40, 450);
  setIntakeSpeed(100);
  softStartTimerForward(-40, 0, 1100);
  setDoinkR(false);
  setIntakeSpeed(0);
  setDoink(false);

  rotateToHeading(180);
  rotateToHeading(-110);
  softStartTimerRotate(100,100,40);
  softStartTimerForward(100,0,58);
  rotateToHeading(185);
  PIDPosForwardAbs(-600); 
}
bool autonFinished = false;
void setAutonFinished( bool _input){
  autonFinished = _input;
}
bool getAutonFinished(){
  return autonFinished;
}
void longlongmidsawpright(){
  // longlongmid sawp
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos(); // add this
  this_thread::sleep_for(100);


  // setup
 
  MyOdom.setHeadingOdom(-90);
  setPistonD(true);
  setPistonI(false);
  setPistonL(false);
  setPistonW(true);
  autonFinished = true;
  setIntakeSpeed(100);
   resetForwardPos();

  softStartTimerForward(0,-95, 200);
  posForwardAbsWithHeading(90, -620, -90);
  PIDPosForwardAbs(-840);
  setPistonI(true);
  rotateToHeading(178);
    posForwardAbsWithHeading(50, 150, 178);
    moveForwardTimer(35,370,1);
    moveForwardTimer(10,10,1);
    this_thread::sleep_for(300);
    resetForwardPos();
    softStartTimerForward(0,-80, 160);
    posForwardAbsWithHeading(80, -520, 183);
    moveForwardTimer(40,400,-1);
    lockBase();
    setPistonD(false);
    this_thread::sleep_for(500);
    unlockBase();
    setPistonI(false);
    rotateToHeading(-87, 60);

    softStartTimerForward(0,95, 160);
    setPistonD(true);
    
    posForwardAbsWithHeading(95, 1180, -87);
    posForwardAbsWithHeading(60, 1360, -87);

    setPistonI(true);
  
    posForwardAbsWithHeading(60, 1420, -87);
    
    PIDPosForwardAbs(1670,65);
    setPistonI(false);
    
    rotateToHeading(-127);
      softStartTimerForward(0,70, 140);
    posForwardAbsWithHeading(70, 400, -127);
    PIDPosForwardAbs(650, 70);
    setPistonI(true);
    rotateToHeading(180);
    
    softStartTimerForward(0,-50, 100);
    setPistonD(false);
    setIntakeSpeed(0);
    posForwardAbsWithHeading(50, -250, 180);
    moveForwardTimer(30, 300, -1);
    lockBase();
    setIntakeSpeed(100);
    
    this_thread::sleep_for(400);
    unlockBase();
    resetForwardPos();
    softStartTimerForward(0,60, 120);
    setPistonD(true);
    posForwardAbsWithHeading(60, 410, 180);
    moveForwardTimer(33,400,1);
    moveForwardTimer(10,10,1);

    //lockBase();
    this_thread::sleep_for(500);
    lockBase();
    this_thread::sleep_for(50);
    resetForwardPos();
    unlockBase();
    
    //unlockBase();
    //setPistonI(false);
    softStartTimerForward(0,-50, 80);
    posForwardAbsWithHeading(50, -150, 180);
    PIDPosForwardAbs(-230);

    rotateToHeading(-135);
    softStartTimerForward(0,-95,160);
        setIntakeSpeed(-100);
    liftSetMode(true);
    posForwardAbsWithHeading(95,-50,-135);
    setIntakeSpeed(-10);
    posForwardAbsWithHeading(95,-970,-135);

    moveForwardTimer(40,250,-1);
    setIntakeSpeed(60);
    
    //setPistonL(true);
    
    lockBase();
    this_thread::sleep_for(600);
    unlockBase();
    
    
    //swingToHeading(-90,-50, false);
    //lockBase();

  }




void nineSplitRight(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);
  setAutonFinished(true);

  resetForwardPos();
  MyOdom.setHeadingOdom(40); 
  setPistonD(true);
  setPistonI(false);
  setPistonL(true);
  setPistonF(true);
  setPistonG(false);
  setPistonW(true);
  setIntakeSpeed(100);


  resetForwardPos();
  softStartTimerForward(0, 70, 180); 
  posForwardAbsWithHeading(70, 290, 40); 
  setPistonI(true);
  softStartTimerForward(70, 50, 60); 
  posForwardAbsWithHeading(50, 560, 40); 
  
  softStartTimerForward(60, 40, 60); 
  setPistonI(false);
  posForwardAbsWithHeading(40, 670, 40); 
  timerCurve(300, 30, 1.95);
  posForwardAbsWithHeading(30, 1120, 68); 
  //moveForwardTimer(30, 400, 1);

  setPistonI(true);
  moveForwardTimer(25, 300, 1);
  //PIDPosForwardAbs(1150);
  
  timerCurve(700, -18, 2.3);
  resetForwardPos();
  
  softStartTimerForward(0,30,100);
  moveForwardTimer(30, 210, 1);
  softStartTimerForward(30,-40,100);
  posForwardAbsWithHeading(40, -560, 0); 
  PIDPosForwardAbs(-710);
    setPistonF(false);
  setPistonG(true);
  
  rotateToHeading(90); //here 
  softStartTimerForward(0, 90, 160);
  
  posForwardAbsWithHeading(80, 340, 90); 
  softStartTimerForward(90,25,100);
  //setIntakeSpeed(0);
  moveForwardTimer(25, 300, 1);
  softStartTimerForward(30,0,60);
  lockBase();
  this_thread::sleep_for(50);
  resetForwardPos();
  unlockBase();
    softStartTimerForward(0, -60, 160);
  
  posForwardAbsWithHeading(60, -200, 90); 

  PIDPosForwardAbs(-280);

  rotateToHeading(183);
  
  softStartTimerForward(0,-40,80);
  posForwardAbsWithHeading(60,-200,183);
  setPistonD(false);
  
  moveForwardTimer(30, 100, -1);
  setPistonD(false);
  moveForwardTimer(30, 200, -1);
  lockBase();
  
  this_thread::sleep_for(500);
  unlockBase();
  setPistonD(true);
  resetForwardPos();

  softStartTimerForward(0,70,140);
  posForwardAbsWithHeading(70,430,180);
  moveForwardTimer(36, 330, 1);
  moveForwardTimer(10,10,1);
  this_thread::sleep_for(500);
  
  lockBase();
  this_thread::sleep_for(50);
  resetForwardPos();
  unlockBase();
  softStartTimerForward(0,-50, 30);
  posForwardAbsWithHeading(50, -140, 180);
  PIDPosForwardAbs(-190);
  rotateToHeading(-45);
  setPistonI(false);

  resetForwardPos();
  softStartTimerForward(0,80,160);
  posForwardAbsWithHeading(80,650,-45);
  //PIDPosForwardAbs(1150);
   posForwardAbsWithHeading(80,980,-45);
   softStartTimerForward(80,30,100);

  setPistonH(true);

  moveForwardTimer(30, 300, 1);
  resetForwardPos();
  //softStartTimerForward(30,-30, 120);
  //posForwardAbsWithHeading(30, -20, -45);
  PIDPosForwardAbs(-20);

  setIntakeSpeed(-40);
  lockBase();
  this_thread::sleep_for(1300);
  unlockBase();
}

// HERE

// NASA: practice match, our first qual (q2?), q59, q125, q200, q251
// Google: q30, q186
// USNavy: q73, q106, q136, q169, q220

void nineSplitLeftNasa(){
}

void nineSplitLeftGoogle(){
}

void nineSplitLeftUSNavy(){
}

void nineSplitRightNasa(){
}

void nineSplitRightGoogle(){
}

void nineSplitRighttUSNavy(){
}

void sawpNasa(){
}

void sawpGoogle(){
}

void sawpUSnavy(){
}

void fastfourthree(){
  // longlongmid sawp
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos(); // add this
  this_thread::sleep_for(100);


  // setup
 
  MyOdom.setHeadingOdom(-90);
  setPistonD(true);
  setPistonI(false);
  setPistonL(false);
  setPistonW(true);
  autonFinished = true;
  setIntakeSpeed(100);
   resetForwardPos();

  softStartTimerForward(0,-95, 200);
  posForwardAbsWithHeading(90, -620, -90);
  PIDPosForwardAbs(-855);
  setPistonI(true);
  rotateToHeading(178);
    posForwardAbsWithHeading(50, 150, 178);
    moveForwardTimer(35,370,1);
    moveForwardTimer(10,10,1);
    this_thread::sleep_for(300);
    resetForwardPos();
    softStartTimerForward(0,-80, 160);
    posForwardAbsWithHeading(80, -520, 183);
    moveForwardTimer(40,400,-1);
    lockBase();
    setPistonD(false);
    this_thread::sleep_for(500);
    unlockBase();
    setPistonI(false);
    rotateToHeading(-87, 60);

    softStartTimerForward(0,95, 160);
    setPistonD(true);
    
    posForwardAbsWithHeading(55, 230, -87);
    //posForwardAbsWithHeading(60, 1360, -87);
    setPistonI(true);

    PIDPosForwardAbs(310,47);
    setPistonI(false);


    rotateToHeading(-45);

  softStartTimerForward(0,30,160);
  posForwardAbsWithHeading(30,180,-45);
  setIntakeSpeed(-100);
    moveForwardTimer(30,300,1);
  setIntakeSpeed(-30);
  moveForwardTimer(30,300,1);
  //PIDPosForwardAbs(600);

  setPistonH(true);
   
  moveForwardTimer(30,100,1);
  lockBase();
 
  setPistonG(true);

  this_thread::sleep_for(3000);
  unlockBase();

    
}

void lowrush(){
MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);
  setAutonFinished(true);

  resetForwardPos();
  MyOdom.setHeadingOdom(40); 
  setPistonD(true);
  setPistonI(false);
  setPistonL(true);
  setPistonF(true);
  setPistonG(true);
  setPistonW(true);
  setIntakeSpeed(100);


  resetForwardPos();
  softStartTimerForward(0, 70, 180); 
  posForwardAbsWithHeading(70, 340, 40); 
  setPistonI(true);
  softStartTimerForward(70, 50, 60); 
  posForwardAbsWithHeading(50, 600, 40); 
  
  softStartTimerForward(60, 40, 60); 
  setPistonI(false);
  posForwardAbsWithHeading(40, 710, 40); 
  timerCurve(300, 30, 1.95);
  posForwardAbsWithHeading(30, 1175, 68); 
  //moveForwardTimer(30, 400, 1);

  setPistonI(true);
  moveForwardTimer(25, 300, 1);
  //PIDPosForwardAbs(1150);
  
  timerCurve(700, -18, 2.3);
  resetForwardPos();
  
  softStartTimerForward(0,30,100);
  setPistonI(false);
  moveForwardTimer(30, 210, 1);
  softStartTimerForward(30,-40,100);
  posForwardAbsWithHeading(40, -240, 40); 
  PIDPosForwardAbs(-400);


  rotateToHeading(-42);
  setPistonF(false);
  setPistonG(false);
  setPistonI(false);
  softStartTimerForward(0,30,160);
  posForwardAbsWithHeading(30,370,-42);
  setIntakeSpeed(-100);
    moveForwardTimer(30,100,1);
  setIntakeSpeed(-30);
  moveForwardTimer(30,300,1);
  setPistonG(true);
  //PIDPosForwardAbs(600);

  setPistonH(true);
   
  moveForwardTimer(20,150,-1);
  lockBase();
 
  

  this_thread::sleep_for(1500);
  unlockBase();
  moveForwardTimer(20,200,1);
    lockBase();
  this_thread::sleep_for(2000);
  unlockBase();
  

}


void fourthreesplit_right(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);

  resetForwardPos();
  MyOdom.setHeadingOdom(0); 
  setPistonD(true);
  setPistonI(false);
  setPistonL(false);
  setPistonF(false);
  setPistonG(false);
  setIntakeSpeed(100);
  softStartTimerForward(0,30, 70);
  posForwardAbsWithHeading(30, 70, 0);
  //PIDPosForwardAbs(50);
  rotateToHeading(23);
  resetForwardPos();
  softStartTimerForward(0, 35, 140);
  posForwardAbsWithHeading(35, 370, 23);
  setPistonI(true);
    PIDPosForwardAbs(580,45);
    setPistonI(false);
    this_thread::sleep_for(300);
        moveForwardTimer(20, 200, 1);
    //setPistonI(true);

        angleRotateAbs(30, 30);
    //this_thread::sleep_for(300);

    //setPistonI(false);
    //PIDPosForwardAbs(620,45);

  //posForwardAbsWithHeading(35, 330, 23);



  
  rotateToHeading(127);
  setPistonI(true);
      
  softStartTimerForward(0, 65, 127);
  posForwardAbsWithHeading(65, 570, 127);

  PIDPosForwardAbs(850,60);

  rotateToHeading(180);
  setIntakeSpeed(0);
  
  softStartTimerForward(0,-40,80);
  posForwardAbsWithHeading(60,-300,180);
  setPistonD(false);
  
  moveForwardTimer(30, 100, -1);
  setPistonD(false);
  moveForwardTimer(30, 200, -1);
  lockBase();
  setIntakeSpeed(100);
    setPistonI(true);
  this_thread::sleep_for(1100);
  unlockBase();
  setPistonD(true);
  resetForwardPos();

  softStartTimerForward(0,70,140);
  posForwardAbsWithHeading(70,420,180);
  moveForwardTimer(36, 330, 1);
  moveForwardTimer(10,10,1);
  this_thread::sleep_for(400);
  
  lockBase();
  this_thread::sleep_for(50);
  resetForwardPos();
  unlockBase();
  softStartTimerForward(0,-50, 30);
  posForwardAbsWithHeading(50, -140, 180);
  PIDPosForwardAbs(-170);
  rotateToHeading(-48.5);
  setPistonI(false);

  resetForwardPos();
  softStartTimerForward(0,80,160);
  posForwardAbsWithHeading(80,650,-48.5);
  //PIDPosForwardAbs(1150);
   posForwardAbsWithHeading(60,930,-48.5);
   softStartTimerForward(60,30,100);

  setPistonH(true);

  moveForwardTimer(30, 300, 1);
  resetForwardPos();
  //softStartTimerForward(30,-30, 120);
  //posForwardAbsWithHeading(30, -20, -45);
  PIDPosForwardAbs(-20);
  

  setIntakeSpeed(-35);
  lockBase();
  this_thread::sleep_for(2100);
  unlockBase();



    



  //PIDPosForwardAbs(500);

  
}

void nineSplitLeft(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);

  resetForwardPos();
  MyOdom.setHeadingOdom(-40); 
  setPistonD(true);
  setPistonI(false);
  setPistonL(true);
  setPistonF(true);
  setPistonG(false);
  setIntakeSpeed(100);


  resetForwardPos();
  softStartTimerForward(0, 70, 180); 
  posForwardAbsWithHeading(70, 390, -40); 
  setPistonI(true);
  softStartTimerForward(70, 50, 60); 
  posForwardAbsWithHeading(50, 620, -40); 
  
  softStartTimerForward(60, 40, 60); 
  setPistonI(false);
  posForwardAbsWithHeading(40, 820, -40); 
  timerCurve(300, 30, 0.495);
  posForwardAbsWithHeading(30, 1090, -64); 
  //moveForwardTimer(30, 400, 1);

  setPistonI(true);
  moveForwardTimer(25, 460, 1);
  softStartTimerForward(25, -18, 90);
  //PIDPosForwardAbs(1150);
  
  timerCurve(700, -18, 0.435);
  resetForwardPos();
  
  softStartTimerForward(0,30,100);
  moveForwardTimer(30, 210, 1);
  softStartTimerForward(30,-40,100);
  posForwardAbsWithHeading(40, -560, 0); 
  PIDPosForwardAbs(-710);
  setPistonF(false);
  setPistonG(true);
  
  rotateToHeading(-90); //here 
  softStartTimerForward(0, 90, 160);
  
  posForwardAbsWithHeading(80, 370, -90); 
  softStartTimerForward(90,25,100);
  //setIntakeSpeed(0);
  moveForwardTimer(23, 400, 1);
  softStartTimerForward(23,0,60);
  lockBase();
  this_thread::sleep_for(100);

  unlockBase();
    resetForwardPos();
    softStartTimerForward(0, -60, 160);
  
  posForwardAbsWithHeading(60, -180, -90); 

  PIDPosForwardAbs(-230);

  rotateToHeading(-180);
  
  softStartTimerForward(0,-40,80);
  posForwardAbsWithHeading(60,-200,-180);
  
  moveForwardTimer(30, 150, -1);
  setPistonD(false);
  moveForwardTimer(30, 150, -1);
  
  lockBase();
  
  this_thread::sleep_for(500);
  unlockBase();
  setPistonD(true);
  resetForwardPos();

  softStartTimerForward(0,70,140);
  posForwardAbsWithHeading(70,410,180);
  moveForwardTimer(33, 330, 1);
  moveForwardTimer(10,10,1);
  this_thread::sleep_for(400);
  
  lockBase();
  this_thread::sleep_for(50);
  resetForwardPos();
  unlockBase();
  softStartTimerForward(0,-50, 30);
  posForwardAbsWithHeading(50, -160, 180);
  PIDPosForwardAbs(-270);
  rotateToHeading(-135);
  setPistonI(false);

  resetForwardPos();
  softStartTimerForward(0,-80,160);
  posForwardAbsWithHeading(80,-650,-135);
  //PIDPosForwardAbs(1150);
   posForwardAbsWithHeading(80,-940,-135);
   setIntakeSpeed(-100);
   softStartTimerForward(-80,-30,100);
   setIntakeSpeed(-8);

  //setPistonH(true);
  liftSetMode(true);

  moveForwardTimer(-30, 150, 1);
  resetForwardPos();
  lockBase();


  setIntakeSpeed(25);
  //lockBase();
  this_thread::sleep_for(1700);
  unlockBase();
 
}

/*
void nineDumpRight(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);

  resetForwardPos();
  MyOdom.setHeadingOdom(30); // was -30
  setDoink(false);
  setDoinkR(false);
  setMiddleState(false);
  liftSetMode(0);
  setIntakeSpeed(100);

  resetForwardPos();
  softStartTimerForward(0, 90, 180); // was -90
  posForwardAbsWithHeading(90, 600, 30); // was -30

  setPistonI(true);
  PIDPosForwardAbs(800);
  setPistonI(false);

  posForwardAbsWithHeading(40, 500, 30); // was -30
  rotateToHeading(-30); // was 30

  moveForwardTimer(60, 600, -1);

  swingToHeading(180, 70, true); // flipped false → true
  setPistonI(true);
  
  posForwardAbsWithHeading(40, 500, 180);

  moveForwardTimer(30, 400, 1);
  moveForwardTimer(10, 100, 1);

  this_thread::sleep_for(800);

  softStartTimerForward(0, -70, 100); // was 

  //PIDPosForwardAbs(-200);

  //rotateToHeading(-225); // (would be mirrored if used)
  //softStartTimerForward(0, 90, 180);

  posForwardAbsWithHeading(70, -500, 180);

  moveForwardTimer(20, 100, -1);
  setPistonD(true);
  //moveForwardTimer(30, 400, -1);
}


void nineDumpLeft(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos(); // add this
  this_thread::sleep_for(100);

  resetForwardPos();
  MyOdom.setHeadingOdom(-30);
  setDoink(false); // storage. setDoink(true) is not storing
  setDoinkR(false); // matchload. setDoink(true) is matchload bar down
  setMiddleState(false); // mid goal intake state
  liftSetMode(0); // mid goal up, liftSetMode(1); midgoal down
  setIntakeSpeed(100);
  resetForwardPos();
  softStartTimerForward(0,-90,180);
  posForwardAbsWithHeading(90, 600, -30);
  setPistonI(true);
  PIDPosForwardAbs(-800);
  setPistonI(false);

  posForwardAbsWithHeading(40, 500, -30);
  rotateToHeading(30);
  moveForwardTimer(60, 600, -1);
  swingToHeading(180, 70, false);
  setPistonI(true);
  
    posForwardAbsWithHeading(40, 500, 180);


  moveForwardTimer(30, 400, 1);

  moveForwardTimer(10, 100, 1);
  this_thread::sleep_for(800);
  softStartTimerForward(0,-70, 100);
  //PIDPosForwardAbs(-200);

  //rotateToHeading(225);
  //softStartTimerForward(0, -90, 180);
  posForwardAbsWithHeading(70, -500, 180);
  moveForwardTimer(20, 100, -1);
  //moveForwardTimer(30, 400, -1);
}
*/
void standard_AWP_right()
{
  // standard awp
  // make faster: matchlaod bar down before so no delay, no wait at matchload just timer move in for longer and tune
  // too slow right now, long mid long sawp
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos(); // add this
  this_thread::sleep_for(100);

  resetForwardPos();
  MyOdom.setHeadingOdom(-90);
  MyOdom.setPosOdom(304.8, 609.6);
  setDoink(false); // storage. setDoink(true) is not storing
  setDoinkR(false); // matchload. setDoink(true) is matchload bar down
  setMiddleState(false); // mid goal intake state
  liftSetMode(0); // mid goal up, liftSetMode(1); midgoal down
  setIntakeSpeed(100);
  
  // Mathchload and score long
  // softStartTimerForward();
  // posForwardAbsWithHeading(100);
  PIDPosForwardAbs(-850, 100);
  rotateToHeading(180,100);
  setDoinkR(true); // matchload bar
  this_thread::sleep_for(140);
  PIDPosForwardAbs(300, 100);
  setIntakeSpeed(0);
  this_thread::sleep_for(5);
  setDoink(true);
  softStartTimerForward(-100,0,1000);
  setIntakeSpeed(100);
  this_thread::sleep_for(900);
  setDoinkR(false); 
  rotateToHeading(90,100);
  setDoink(false);
  // Grab stacks
  PIDPosForwardAbs_WithEvent(1490, 100, 1200);

  // Score mid
  setIntakeSpeed(0);
  rotateToHeading(135,100);
  softStartTimerForward(-100,0,550);
  setMiddleState(true);
  liftSetMode(1);
  setIntakeSpeed(100);
  this_thread::sleep_for(1800);
  rotateToHeading(135,100);


  // Score second long goal
  PIDPosForwardAbs(1270,100);
  rotateToHeading(180,100);
  liftSetMode(0);
  setMiddleState(false);
  setDoinkR(true);
  this_thread::sleep_for(80);
  PIDPosForwardAbs(400,100);
  setIntakeSpeed(0);
  this_thread::sleep_for(5);
  setDoink(true);
  softStartTimerForward(-100,0,1100);
  setIntakeSpeed(100);
  this_thread::sleep_for(2000);
}

void fourlowrush(){
MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);
  setAutonFinished(true);

  resetForwardPos();
  MyOdom.setHeadingOdom(40); 
  setPistonD(true);
  setPistonI(false);
  setPistonL(true);
  setPistonF(false);
  setPistonG(false);
  setPistonW(true);
  setIntakeSpeed(100);

resetForwardPos();
  softStartTimerForward(0, 70, 180); 
  posForwardAbsWithHeading(70, 370, 40); 
  setPistonI(true);

  PIDPosForwardAbs(600, 50);
  setPistonI(false);

  
  rotateToHeading(-45);

  setPistonH(true);
  softStartTimerForward(0,30,160);
  posForwardAbsWithHeading(30,435,-42);
  setIntakeSpeed(-100);
  moveForwardTimer(30,100,1);
  setIntakeSpeed(-30);
  moveForwardTimer(30,300,1);
  setPistonG(true);
  //PIDPosForwardAbs(600);
  moveForwardTimer(20,150,-1);
  lockBase();
 
  this_thread::sleep_for(1500);
  unlockBase();
  moveForwardTimer(20,200,1);
  lockBase();
  this_thread::sleep_for(2000);
  unlockBase();
  setIntakeSpeed(-60);
  this_thread::sleep_for(2000);

  //rotateToHeading(22);

}

void sixlowrushright(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos(); // add this
  this_thread::sleep_for(100);


  // setup
  resetForwardPos();
  MyOdom.setHeadingOdom(30);
  setPistonD(false);
  setPistonI(false);
  setPistonL(false);
  setIntakeSpeed(100);

  resetForwardPos();
  softStartTimerForward(0, 90, 180); // was -90
  posForwardAbsWithHeading(90, 600, 30); // was -30

  setPistonI(true);
  PIDPosForwardAbs(800);
  setPistonI(false);

  posForwardAbsWithHeading(40, 500, 30);

  rotateToHeading(45);
  resetForwardPos();
  softStartTimerForward(0,-80,160);
  posForwardAbsWithHeading(80,-295,-135);
  PIDPosForwardAbs(125);

  resetForwardPos();
  rotateToHeading(-45);
  setPistonH(true);

  PIDPosForwardAbs(300);
  setIntakeSpeed(-100);
  this_thread::sleep_for(2000);
}

void doublemidleft(){
  MyTimer autotimer;
  fullresetsideWaysPos();
  fullresetForwardEncoderPos();
  this_thread::sleep_for(100);

  // setup
  resetForwardPos();
  MyOdom.setHeadingOdom(-30); // was 30
  setPistonD(false);
  setPistonI(false);
  setPistonL(false);
  setIntakeSpeed(100);

  resetForwardPos();
  softStartTimerForward(0, 90, 180); 
  posForwardAbsWithHeading(90, 600, -30); 

  setPistonI(true);
  PIDPosForwardAbs(800);
  setPistonI(false);

  posForwardAbsWithHeading(40, 500, -30); // was 30

  rotateToHeading(-45); // was -135

  resetForwardPos();
  softStartTimerForward(0, -80, 160);
  posForwardAbsWithHeading(80, -295, 135); 
  PIDPosForwardAbs(-125);

  resetForwardPos();
  rotateToHeading(-135);
  PIDPosForwardAbs(300);
  setPistonL(true);
  this_thread::sleep_for(300);
  setPistonL(false);

  setPistonH(true);
  PIDPosForwardAbs(300);
  rotateToHeading(-90);

  resetForwardPos();
  softStartTimerForward(0,-80,160);
  posForwardAbsWithHeading(80,-630,-90);
  PIDPosForwardAbs(-270);

  rotateToHeading(-45);
  PIDPosForwardAbs(300);
  setIntakeSpeed(-100);
  this_thread::sleep_for(3000);



}

void runAuton(int auton_number)
{
  setAutonFinished(true);
  vex_log(V_Trace, "Starting autonomous::runAuton...");

  setAutonMode();
  autonInit();
  if (auton_number == 1)
  {
    lowrush();
    //fastfourthree();
    //fourlowrush();
    //longlongmidsawpright();
    //nineSplitLeft();
    //nineSplitRight();
  }

  else if (auton_number == 2)

    vex_log(V_Trace, "Completed autonomous::runAuton");
}


