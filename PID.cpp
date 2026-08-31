#include "PID.h"
#include "basic-func.h"

PID::PID() : firstIter(true), arrived(false), IMax(20), IRange(50), jumpTime(50), type("normal")
{
  myTimer.reset();
  PIDTimer.reset();
}

void PID::setCoefficient(float _kp, float _ki, float _kd)
{
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

void PID::setTarget(float _target) { target = _target; }
void PID::setIMax(float _IMax) { IMax = _IMax; }
void PID::setIRange(float _IRange) { IRange = _IRange; }
void PID::setErrorTolerance(float _errorTol) { errorTol = _errorTol; }
void PID::clearErrorInt() { errorInt = 0; }
void PID::setDTolerance(float _DTol) { DTol = _DTol; }
void PID::setJumpTime(float _jumpTime) { jumpTime = _jumpTime; }
bool PID::targetArrived() { return arrived; }
float PID::getI() { return I; }
float PID::getOutput() { return output; }
void PID::setType(std::string _type) { type = _type; }
void PID::setSmallBigErrorTolerance(double new_small_error_tolerance, double new_big_error_tolerance)
{
  small_error_tolerance = new_small_error_tolerance;
  big_error_tolerance = new_big_error_tolerance;
}
void PID::setSmallBigErrorDuration(double new_small_error_duration, double new_big_error_duration)
{
  small_error_duration = new_small_error_duration;
  big_error_duration = new_big_error_duration;
}
void PID::update(float input)
{

  errorCurr = target - input;
  if (type == "turn")
  {
    errorCurr = shortDeg(errorCurr);
  }
  P = kp * errorCurr;
  if (firstIter)
  {
    firstIter = false;
    errorPrev = errorCurr;
    errorInt = 0;
    small_check_time = Brain.timer(msec);
    big_check_time = Brain.timer(msec);
  }
  errorDer = errorCurr - errorPrev;
  errorPrev = errorCurr;
  D = errorDer * kd;
  if (fabs(P) >= IRange)
  {
    errorInt = 0;
  }
  else
  {
    errorInt += errorCurr;
    if (fabs(errorInt) * ki > IMax)
    {
      errorInt = sign(errorInt) * IMax / ki;
    }
  }
  if (sign(errorInt) != sign(errorCurr) || (fabs(errorCurr) <= errorTol))
  {
    errorInt = 0;
  }
  I = ki * errorInt;
  if (PIDTimer.getTime() >= 2000)
  {
    arrived = true;
  }

  if (fabs(errorCurr) <= errorTol && fabs(D) <= DTol)
  {
    if (myTimer.getTime() >= jumpTime)
    {
      arrived = true;
    }
  }

  else
  {
    myTimer.reset();
  }

  output = P + I + D;
}
