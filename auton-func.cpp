#include "vex.h"
#include "basic-func.h"
#include "auton-func.h"
#include "timer-stuff.h"
#include "PID.h"
#include "parameters.h"
#include "iostream"

using namespace vex;
using std::cout;
using std::endl;

void voltageForward(float _power)
{
  moveLeft(_power);
  moveRight(_power);
  this_thread::sleep_for(5);
}
void timerForward(float _power, int _duration)
{
  moveForward(_power);
  this_thread::sleep_for(_duration);
  unlockBase();
}

void timerForwardWithHeading(float _power, int _duration, float _targetHeading)
{
  auto myTimer = MyTimer();
  float headingError = _targetHeading - getHeading();
  float TurnP = headingError * 2.0; // 2.0 is

  while (myTimer.getTime() < _duration)
  {
    headingError = _targetHeading - getHeading();
    TurnP = headingError * 2.0; // 2.0 is kp
    if (fabs(TurnP) > 20)
    {
      TurnP = sign(TurnP) * 20; // 20 is max P
    }
    moveLeft(_power + TurnP);
    moveRight(_power + TurnP);
  }
  unlockBase();
}
void PIDPosForwardAbs_WithEvent(float _target, float _maxPower, float triggerPoint)
{
  float output = 0;
  float prevOutput = 0;
  bool doinkTriggered = false;

  auto pid = PID();
  pid.setCoefficient(0.36, 0.01, 6.7);
  pid.setTarget(_target);
  pid.setIMax(20);
  pid.setIRange(7);
  pid.setErrorTolerance(5);
  pid.setDTolerance(40);
  pid.setJumpTime(25);

  while (!pid.targetArrived())
  {
    float pos = getForwardPos();
    if (!doinkTriggered && pos >= triggerPoint)
    {
      setDoinkR(true);
      doinkTriggered = true;
    }

    pid.update(pos);
    output = pid.getOutput();

    output = slew(prevOutput, output, 5, 6);

    moveForward(output > _maxPower ? sign(output) * _maxPower : output);

    prevOutput = output;
    this_thread::sleep_for(5);
  }

  unlockBase();
}

void posForwardRel(float _power, float _target)
{
  float startPos = getForwardPos();
  moveForward(_power);
  while (fabs(getForwardPos() - startPos) < _target)
  {
    this_thread::sleep_for(5);
  } // test sleep original no sleep
  unlockBase();
}

void posForwardAbs(float _power, float _target)
{
  float targetRel = _target - getForwardPos();
  float power = sign(targetRel) * fabs(_power);
  posForwardRel(power, targetRel);
}

void posForwardRelWithHeading(float _power, float _targetPos, float _targetHeading)
{
  float startPos = getForwardPos();

  MyTimer convergeTimer;
  convergeTimer.reset();
  float headingError = shortDeg(_targetHeading - getHeading());
  float PTurn = headingError * 2.3; // 2.0 is
  // cout<<startPos <<endl;
  while (fabs(getForwardPos() - startPos) < fabs(_targetPos))
  {

    headingError = shortDeg(_targetHeading - getHeading());
    // cout << fabs(getForwardPos() - startPos) << endl;
    //  cout << getForwardPos() << endl;
    PTurn = headingError * 2.3; // 2.0 is kp
    if (fabs(PTurn) > 20)
    {
      PTurn = sign(PTurn) * 20;
    } // 20 is max P
    moveLeft(_power + PTurn);
    moveRight(_power - PTurn);
    this_thread::sleep_for(5);
    if (convergeTimer.getTime() >= 1200 && fabs(_targetPos - getForwardPos() - startPos) < 100)
    {
      break;
    }
    else if (convergeTimer.getTime() >= 3000)
    {
      break;
    }
  }
  unlockBase();
}

void posForwardAbsWithHeading(float _power, float _targetPos, float _targetHeading)
{
  float targetPosRel = _targetPos - getForwardPos();
  float power = sign(targetPosRel) * fabs(_power);
  posForwardRelWithHeading(power, targetPosRel, _targetHeading);
}

void PIDPosForwardAbs(float _target, float _maxPower)
{
  float output = 0;
  float prevOutput = 0;
  bool firstIter = true;
  auto pid = PID();
  pid.setCoefficient(0.38, 0.001, 6.6); // Og 0.35, 0.04, 5 //0.29, 0.03, 4.1
  // HERE
  pid.setTarget(_target);
  pid.setIMax(20);  // og 35
  pid.setIRange(7); // og 10
  pid.setErrorTolerance(10);
  pid.setDTolerance(40); // og 10
  pid.setJumpTime(25);   // og 50
  while (!pid.targetArrived())
  {

    pid.update(getForwardPos());
    output = pid.getOutput();

    // cout << output << endl;

    output = slew(prevOutput, output, 5, 6);
    cout << output << endl;

    moveForward(output > _maxPower ? sign(output) * _maxPower : output);
    // moveForward(pid.getOutput() > _maxPower ? sign(pid.getOutput()) * _maxPower : pid.getOutput());
    prevOutput = output;
    cout << getForwardPos() << endl;
    //  cout << prevOutput << endl;
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Forward Position: %.1f            ", getForwardPos());
    // cout << vexSystemHighResTimeGet() << "," << getForwardPos() << "," << Rotation2.position(deg) * (2 * 25.4 * 3.1416) / 360 << endl;

    this_thread::sleep_for(5);
  }
  unlockBase();
}

void PIDPosForwardRel(float _target)
{
  PIDPosForwardAbs(_target + getForwardPos());
}

void PIDPosCurveRel(float left_target, float right_target, float tolerance, float _maxPower)
{
  float _target = (left_target + right_target) / 2;
  float ratio = left_target / right_target;
  auto pid = PID();
  float k = 1;
  pid.setCoefficient(1, 0.05, 3); // og 1.05, 0.05, 1.5
  pid.setTarget(_target);
  pid.setIMax(30);
  pid.setIRange(20);
  pid.setErrorTolerance(tolerance);
  pid.setDTolerance(5);
  pid.setJumpTime(40);
  float leftPos_err = 0, rightPos_err = 0;
  float PIDOutput = 0;
  while (!pid.targetArrived())
  {
    leftPos_err = (getForwardPos() / _target) * left_target - getLeftPos();
    rightPos_err = (getForwardPos() / _target) * right_target - getRightPos();
    pid.update(getForwardPos());
    PIDOutput = pid.getOutput();
    if (fabs(PIDOutput) > _maxPower)
    {
      PIDOutput = sign(PIDOutput) * _maxPower;
    }
    if (ratio > 1)
    {
      moveLeft(PIDOutput + k * leftPos_err);
      moveRight(PIDOutput / ratio + k * rightPos_err);
    }
    else
    {
      moveLeft(PIDOutput * ratio + k * leftPos_err);
      moveRight(PIDOutput + k * rightPos_err);
    }
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Forward Position: %.1f.        ", getForwardPos());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}

void PIDPosCurveAbs(float left_target, float right_target, float tolerance, float _maxPower)
{
  PIDPosCurveRel(getLeftPos() + left_target, getRightPos() + right_target, tolerance, _maxPower);
}
void PIDSwingToDistance(float _target, float _maxPower, float _side)
{
  auto pid = PID();

  pid.setCoefficient(1, 0.05, 1.4); // og 1.05, 0.05, 1.5
  pid.setTarget(_target);
  pid.setIMax(30);
  pid.setIRange(20);
  pid.setErrorTolerance(5); // 5mm tolerance
  pid.setDTolerance(5);
  pid.setJumpTime(40);
  float pidInput = 0;
  float pidOutput = 0;

  if (_side == -1)
  {
    // left side drives
    lockRight();
  }
  else
  {
    lockLeft();
    // right
  }
  while (!pid.targetArrived())
  {
    if (_side == -1)
    {
      pidInput = getLeftPos();
      pid.update(pidInput);
      pidOutput = pid.getOutput();
      pidOutput = (pidOutput > _maxPower) ? _maxPower : pidOutput;
      moveLeft(pidOutput);
    }
    else
    {
      pidInput = getRightPos();
      pid.update(pidInput);
      pidOutput = pid.getOutput();
      pidOutput = (pidOutput > _maxPower) ? _maxPower : pidOutput;
      moveRight(pidOutput);
    }
  }
  unlockBase();
}
void PIDSwingToHeading(float _target, float _maxPower, float _side)
{
  auto pid = PID();

  pid.setCoefficient(1, 0.05, 1.4); // og 1.05, 0.05, 1.5
  pid.setTarget(_target);
  pid.setIMax(30);
  pid.setIRange(20);
  pid.setErrorTolerance(0.4); // 0.4 deg tolerance
  pid.setDTolerance(5);
  pid.setJumpTime(40);
  pid.setType("turn");
  float pidInput = 0;
  float pidOutput = 0;

  if (_side == -1)
  {
    // left side drives
    lockRight();
  }
  else
  {
    lockLeft();
    // right
  }
  while (!pid.targetArrived())
  {
    if (_side == -1)
    {
      pidInput = getHeading();
      pid.update(pidInput);
      pidOutput = pid.getOutput();
      pidOutput = (pidOutput > _maxPower) ? _maxPower : pidOutput;
      moveLeft(pidOutput);
    }
    else
    {
      pidInput = getHeading();
      pid.update(pidInput);
      pidOutput = pid.getOutput();
      pidOutput = (pidOutput > _maxPower) ? _maxPower : pidOutput;
      moveRight(pidOutput);
    }
  }
}

void swingToHeading(float targetHeading, float power, bool swingLeft) {
  // normalize target
  if (targetHeading < 0) targetHeading += 360;
  if (targetHeading >= 360) targetHeading -= 360;

  double current = getHeading();
  float headingError = shortDeg(targetHeading - current);

  int direction = (headingError > 0) ? 1 : -1;

  if (swingLeft) {
    lockLeft();
  } else {
    lockRight();
  }

  while (true) {
    current = getHeading();

    // ✅ recompute error properly
    double newError = shortDeg(targetHeading - current);

    // Exit when we pass target
    if ((headingError > 0 && newError <= 0) || 
        (headingError < 0 && newError >= 0)) {
      break;
    }

    if (swingLeft) {
      moveRight( power);
    } else {
      moveLeft(- power);
    }

    headingError = newError;
    vex::task::sleep(10);
  }

  moveLeft(0);
  moveRight(0);
  resetForwardPos();
}

void softStartTimerForward(float _powerInit, float _powerFinal, int _duration)
{
  auto myTimer = MyTimer();
  float timeSlope = (_powerFinal - _powerInit) / _duration;
  while (myTimer.getTime() < _duration)
  {
    moveForward(_powerInit + myTimer.getTime() * timeSlope);
    this_thread::sleep_for(5);
  }
  unlockBase();
}

void timerRotate(float _power, int _duration)
{
  turnClockwise(_power);
  this_thread::sleep_for(_duration);
  resetForwardPos();
  unlockBase();
}
void angleRotateRel(float _power, float _target)
{
  float startAngle = getHeading();

  // direction based on target
  float power = sign(_target) * fabs(_power);

  turnClockwise(power);

  while (true)
  {
    float current = getHeading();

    // how far we've rotated from start
    float rotated = shortDeg(current - startAngle);

    if (_target > 0)
    {
      if (rotated >= _target)
        break;
    }
    else
    {
      if (rotated <= _target)
        break;
    }

    this_thread::sleep_for(5);
  }

  resetForwardPos();
  unlockBase();
}

void angleRotateAbs(float _power, float _target)
{
  float error = shortDeg(_target - getHeading());
  angleRotateRel(_power, error);
}
void PIDAngleRotateAbs(float _target, float kp, float ki, float kd, float _maxPower, float tolerance)
{
  float output = 0;
  float prevOutput = 0;
  auto pid = PID();

  pid.setCoefficient(kp, ki, kd);
  pid.setTarget(_target);
  pid.setIMax(10);
  pid.setIRange(5);
  pid.setErrorTolerance(tolerance);
  pid.setDTolerance(23); // 5
  pid.setJumpTime(15);   // og 50
  pid.setType("turn");
  while (!pid.targetArrived())
  {
    pid.update(getHeading());
    output = pid.getOutput();

    // cout << output << endl;

    output = slew(prevOutput, output, 6, 6);
    if (fabs(output) > _maxPower)
    {
      turnClockwise(_maxPower * sign(output));
    }
    else
    {
      turnClockwise(output);
    }
    turnClockwise(output);
    prevOutput = output;
    cout << getHeading() << endl;
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Heading: %.2f.       ", getHeading());
    this_thread::sleep_for(5); // og 5
  }
  cout << "done" << endl;
  resetForwardPos();
  unlockBase();
}
void PIDAngleRotateRel(float _target)
{
  PIDAngleRotateAbs(getHeading() + _target);
}

void softStartTimerRotate(float _powerInit, float _powerFinal, int _duration)
{
  auto myTimer = MyTimer();
  float timeSlope = (_powerFinal - _powerInit) / _duration;
  while (myTimer.getTime() < _duration)
  {
    turnClockwise(_powerInit + timeSlope * myTimer.getTime());
  }
  resetForwardPos();
  unlockBase();
}
void rotateToHeading(float _target, float _maxpower)
{
  float error = shortDeg(_target - getHeading());
  float dir = sign(error);

  // Stage 1: soft start
  softStartTimerRotate(0, _maxpower * dir, fabs(_maxpower));

  // Stage 2: constant power until near target
  if (fabs(error) > 45)
  {
    float preTarget = _target - dir * 40; // always 30° before final
    angleRotateAbs(_maxpower, preTarget);
    // softStartTimerRotate(_maxpower * dir, _maxpower * dir * 0.5, fabs(_maxpower));
  }

  // Stage 3: PID finish
  PIDAngleRotateAbs(_target, 8.7, 0.006, 69, _maxpower, 0.6); // 8.5 0.006 71
}
void timerWait(float _waitTime)
{
  this_thread::sleep_for(_waitTime);
}
void posForwardWithHeadingFromDistance(
    vex::distance &distanceSense,
    float power,
    float targetDist,
    float targetHeading,
    float offset)
{
  MyTimer timer;
  timer.reset();
  float current = getDistanceNormal(distanceSense, getHeading() + offset);
  MyTimer FinishTimer;
  FinishTimer.reset();

  while (true)
  {
    current = getDistanceNormal(distanceSense, getHeading() + offset);
    /*cout << distanceSense.objectDistance(vex::distanceUnits::mm) << "NIIII" << endl;
    cout << getDistance(distanceSense) << "herererere" << endl;
    cout << current << "here" << endl;*/
    // heading hold
    float headingError = shortDeg(targetHeading - getHeading());
    float PTurn = headingError * 2.3;

    if (fabs(PTurn) > 20)
      PTurn = sign(PTurn) * 20;

    moveLeft(power + PTurn);
    moveRight(power - PTurn);

    // ===== EXIT LOGIC =====

    // moving toward wall
    if (current <= targetDist)
    {
      if (FinishTimer.getTime() > 100)
      {
        break;
      }
    }
    else
    {
      FinishTimer.reset();
    }

    // ===== FAILSAFE =====
    if (timer.getTime() > 2500)
      break;

    this_thread::sleep_for(5);
  }
}

void PIDPosForwardFromDistance(vex::distance distanceSense, float _target)
{
  auto pid = PID();
  pid.setCoefficient(0.597, 0.06, 7.5); // Og 0.35, 0.04, 5 //0.29, 0.03, 4.1
  pid.setTarget(_target);
  pid.setIMax(40);   // og 35
  pid.setIRange(10); // og 10
  pid.setErrorTolerance(5);
  pid.setDTolerance(10); // og 10
  pid.setJumpTime(30);   // og 50
  while (!pid.targetArrived())
  {
    pid.update(getDistance(distanceSense));
    moveForward(pid.getOutput());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Forward Position: %.1f            ", getDistance(distanceSense));
    // cout << vexSystemHighResTimeGet() << "," << getForwardPos() << "," << Rotation2.position(deg) * (2 * 25.4 * 3.1416) / 360 << endl;

    this_thread::sleep_for(5);
  }
  unlockBase();
}
void curveToDist(vex::distance distanceSense, float _target, float direct, float side, float _power, float _offset)
{
  float error = _target - getDistanceNormal(distanceSense, getHeading() + _offset);

  float P = 0;
  float kp = 0.1;
  float outputLeft = 0;
  float outputRight = 0;
  while (fabs(getDistance(distanceSense) - _target) > 5)
  {
    error = _target - getDistanceNormal(distanceSense, getHeading() + _offset);

    P = kp * error;
    if (side == 1) // curve to the right
    {
      outputLeft = -1 * P * direct;
      outputRight = 1 * P * direct;
    }
    else
    {
      outputLeft = P * direct;
      // cout << outputLeft << endl;
      outputRight = -1.0 * P * direct;
    }
    moveLeft(_power * direct);
    cout << _power * direct << endl;
    moveRight(_power * direct);
    this_thread::sleep_for(5);
  }
}
void curveToHeading(float target, float input, float ratio)
{
  float prevError = shortDeg(target - getHeading());

  while (true)
  {
    // keep curving
    moveLeft(input * ratio);
    moveRight(input / ratio);

    float currentError = shortDeg(target - getHeading());

    // if error sign flipped → we passed target
    if ((prevError > 0 && currentError <= 0) ||
        (prevError < 0 && currentError >= 0))
    {
      break;
    }
    MyTimer jamTimer;
    if (fabs(currentError) > fabs(prevError))
    {
      if (jamTimer.getTime() > 300)
      {
        break;
      }
    }

    prevError = currentError;
  }
}

static bool autonMode = false;
void clearAutonMode(void) { autonMode = false; }
void setAutonMode(void) { autonMode = true; }