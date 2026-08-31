#include "vex.h"
#include "robot-config.h"
#include "basic-func.h"
#include "parameters.h"
#include "PID.h"
#include "timer-stuff.h"
#include "iostream"
#include <string>
#include "vex_log.h"
#include "intakeParameters.h"
// using namespace std;
template <typename T>
T clampMath(T value, T minVal, T maxVal)
{
  if (minVal > maxVal)
    std::swap(minVal, maxVal);
  return (value < minVal) ? minVal : (value > maxVal) ? maxVal
                                                      : value;
}
float abbs(float x) { return x >= 0 ? x : -x; }

float deg2rad(float deg) { return deg / 180.0 * PI; }

float rad2deg(float rad) { return rad / PI * 180.0; }

float sqre(float x) { return x * x; }

int sign(float _input)
{
  if (_input > 0)
    return 1;
  else if (_input < 0)
    return -1;
  else
    return 0;
}
float shortDeg(float _degree)
{
  int _cir = int(_degree / 360);
  _degree -= _cir * 360;
  if (_degree > 180.0)
    _degree -= 360;
  if (_degree < -180.0)
    _degree += 360;
  return _degree;
}

float slew(float target, float current,

           float accelStep, float decelStep)
{

  double step = (abs(target) > abs(current)) ? accelStep : decelStep;

  if (target > current + step)
    return current + step;

  if (target < current - step)
    return current - step;

  return target;
}
bool PTOState = false;
void setPTOState(bool _input)
{
  PTOState = _input;
}
void moveLeft(float _input)
{
  if (fabs(_input) > 100)
    _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLBT.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLFT.spin(directionType::fwd, (int)_input, velocityUnits::pct);


  if (PTOState == true)
  {
    Motor_Intake2.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  }
}

void moveLeftVel(float _input)
{
  if (fabs(_input) > 100)
    _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLBT.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLFT.spin(directionType::fwd, (int)_input, velocityUnits::pct);

}

void lockLeft(void)
{
  Motor_BaseLF.stop(vex::brakeType::hold);
  Motor_BaseLM.stop(vex::brakeType::hold);
  Motor_BaseLB.stop(vex::brakeType::hold);
  Motor_BaseLFT.stop(vex::brakeType::hold);
  Motor_BaseLBT.stop(vex::brakeType::hold);
}

void unlockLeft(void)
{
  Motor_BaseLF.stop(vex::brakeType::coast);
  Motor_BaseLM.stop(vex::brakeType::coast);
  Motor_BaseLB.stop(vex::brakeType::coast);
  Motor_BaseLFT.stop(vex::brakeType::coast);
  Motor_BaseLBT.stop(vex::brakeType::coast);
}

void moveRight(float _input)
{
  if (fabs(_input) > 100)
    _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRBT.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRFT.spin(directionType::fwd, (int)_input, velocityUnits::pct);

  if (PTOState == true)
  {
    Motor_Intake1.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  }
}

void moveRightVel(float _input)
{
  if (fabs(_input) > 100)
    _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRBT.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRFT.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}

void lockRight(void)
{
  Motor_BaseRF.stop(vex::brakeType::hold);
  Motor_BaseRM.stop(vex::brakeType::hold);
  Motor_BaseRB.stop(vex::brakeType::hold);
  Motor_BaseRBT.stop(vex::brakeType::hold);
  Motor_BaseRFT.stop(vex::brakeType::hold);
}

void unlockRight(void)
{
  Motor_BaseRF.stop(vex::brakeType::coast);
  Motor_BaseRM.stop(vex::brakeType::coast);
  Motor_BaseRB.stop(vex::brakeType::coast);
  Motor_BaseRBT.stop(vex::brakeType::coast);
  Motor_BaseRFT.stop(vex::brakeType::coast);
}

void moveForward(float _input)
{
  moveLeft(_input);
  moveRight(_input);
}

void moveForwardVel(float _input)
{
  moveLeftVel(_input);
  moveRightVel(_input);
}

void moveForwardTimer(float _input, float _time, float _direct)
{
  float error = 0;
  float initHeading = 0;
  MyTimer movetimer;
  movetimer.reset();
  initHeading = getHeading();
  while (movetimer.getTime() < _time)
  {
    error = initHeading - getHeading();
    moveLeft(_input * _direct + 2.0 * error);
    moveRight(_input * _direct - 2.0 * error);
  }
}

void turnClockwise(float _input)
{
  /*if(fabs(_input) >=60 ){
    _input = sign(_input) * 60;
  }*/
  moveLeft(_input);
  moveRight(-_input);
}

void turnClockwiseTimer(float _input, float _time)
{
  /*if(fabs(_input) >=60 ){
    _input = sign(_input) * 60;
  }*/
  MyTimer turnTimer;
  while (turnTimer.getTime() <= _time)
  {
    moveLeft(_input);
    moveRight(-_input);
  }
}

void timerCurve(float _time, float _input, float _ratio)
{
  MyTimer curveTimer;
  curveTimer.reset();
  while (curveTimer.getTime() < _time)
  {
    moveLeft(_input * _ratio);
    moveRight(_input / _ratio);
  }
}
void lockBase(void)
{
  lockLeft();
  lockRight();
}

void unlockBase(void)
{
  unlockLeft();
  unlockRight();
}

float getCurVel()
{
  return (Motor_BaseLM.velocity(pct) + Motor_BaseRM.velocity(pct)) / 2;
}

static volatile float _leftPosLast = 0, _rightPosLast = 0;

float getLeftPos()
{
  // cout << _leftPosLast << endl;
  // cout << (Motor_BaseLF.position(deg) + Motor_BaseLB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2.0 * 360) - _leftPosLast << endl;
  return (Motor_BaseLF.position(deg) + Motor_BaseLB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2.0 * 360) - _leftPosLast; // return mm
  // return (Motor_BaseLB.position(deg) + Motor_BaseLM.position(deg))/(2*360) * (WHEEL_DIAM * 25.4 * 3.1416) * CHASSIS_GEAR_RATIO - _leftPosLast;
  // return (Motor_BaseLF.position(deg) + Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416)/ (2*360)- _leftPosLast;
}

float getRightPos()
{
  return (Motor_BaseRF.position(deg) + Motor_BaseRB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2.0 * 360) - _rightPosLast; // return mm
  // return (Motor_BaseRB.position(deg) + Motor_BaseRM.position(deg))/(2*360) * (WHEEL_DIAM * 25.4 * 3.1416) * CHASSIS_GEAR_RATIO - _rightPosLast;
  // return (Motor_BaseRF.position(deg) + Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416)/ (2*360)- _rightPostLast;
}

float getForwardPos()
{
  /*cout<<getLeftPos();
  cout<<"||";
  cout<<getRightPos()<<endl;*/
  return (getLeftPos() + getRightPos()) / 2;
}

void resetLeftPos()
{
  /*cout<< "left pos:" <<  _leftPosLast << endl;
  cout<< "BaseLF:" <<  Motor_BaseLF.position(deg) << endl;
  cout<< "BaseLM:" <<  Motor_BaseLM.position(deg) << endl;     */

  _leftPosLast = (Motor_BaseLF.position(deg) + Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2 * 360);
  // cout<< "left pos:" <<  _leftPosLast << endl;
}
void setleftPos(float _input)
{
  _leftPosLast = ((Motor_BaseLF.position(deg) + Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2 * 360)) + _input;
}

void fullResetLeftPos()
{
  // Motor_BaseLF.resetPosition();
  // Motor_BaseLM.resetPosition();
  _leftPosLast = 0;
}

void fullResetRightPos()
{
  // Motor_BaseRF.resetPosition();
  // Motor_BaseRM.resetPosition();
  _rightPosLast = 0;
}
void fullResetForwardPos()
{
  fullResetLeftPos();
  fullResetRightPos();
}
void resetRightPos()
{
  // cout<< "BaseRF:" <<  Motor_BaseRF.position(deg) << endl;
  // cout<< "BaseRM:" <<  Motor_BaseRM.position(deg) << endl;
  _rightPosLast = (Motor_BaseRF.position(deg) + Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2 * 360);
}
void setRightPos(float _input)
{
  _rightPosLast = ((Motor_BaseRF.position(deg) + Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2 * 360)) + _input;
}
void fullresetsideWaysPos()
{
  Rotation1.resetPosition();
  // Rotation1.setReversed(true); // change if needed
}
void fullresetForwardEncoderPos()
{
  Rotation2.resetPosition();
  // Rotation2.setReversed(false); // change if needed
}

void resetForwardPos()
{

  //  cout<< "Heading: %.2f.       ", to_string(_leftPosLast) <<endl;
  // cout<< "left pos:" <<  _leftPosLast << endl;
  // cout<< "right pos:" <<  _rightPosLast << endl;

  resetLeftPos();
  resetRightPos();
  // cout<< " curr left pos:" << _leftPosLast  << endl;
  // cout<< " curr right pos:" <<  _rightPosLast << endl;
}
void setForwardPos(float _input)
{
  setleftPos(_input);
  setRightPos(_input);
}
void motorResetLeft()
{
  Motor_BaseLF.resetPosition();
  Motor_BaseLM.resetPosition();
}
void motorResetRight()
{
  Motor_BaseRF.resetPosition();
  Motor_BaseRM.resetPosition();
}
void motorResetFull()
{
  motorResetLeft();
  motorResetRight();
}

static float headingOffset = 0;
void softStartTimerArc(
    float startVel,
    float endVel,
    int dir,          // 1 = clockwise, -1 = counter-clockwise
    float radius,     // mm (or same unit as trackWidth)
    int duration      // ms
)
{
  auto myTimer = MyTimer();

  float trackWidth = 400; // same as your arcToHeading
  float timeSlope = (endVel - startVel) / duration;

  while (myTimer.getTime() < duration)
  {
    float curVel = startVel + timeSlope * myTimer.getTime();

    // calculate inner/outer ratio
    float innerRatio = (radius - trackWidth / 2.0) / (radius + trackWidth / 2.0);

    float outerVel = curVel;
    float innerVel = curVel * innerRatio;

    if (dir == 1) // clockwise
    {
      moveLeft(innerVel);
      moveRight(outerVel);
    }
    else // counter-clockwise
    {
      moveLeft(outerVel);
      moveRight(innerVel);
    }

    this_thread::sleep_for(5);
  }

  unlockBase();
}
float getHeading()
{
  return IMU.rotation() * kGyro + headingOffset;
}
void softStartTimerSwing(float _side, float _powerInit, float _powerFinal, int _duration)
{
  auto myTimer = MyTimer();

  float timeSlope = (_powerFinal - _powerInit) / _duration;

  // light resistance instead of hold
  if (_side == -1)
  {
    Motor_BaseRF.stop(vex::brakeType::brake);
    Motor_BaseRM.stop(vex::brakeType::brake);
    Motor_BaseRB.stop(vex::brakeType::brake);
  }
  else
  {
    Motor_BaseLF.stop(vex::brakeType::brake);
    Motor_BaseLM.stop(vex::brakeType::brake);
    Motor_BaseLB.stop(vex::brakeType::brake);
  }

  while (myTimer.getTime() < _duration)
  {
    float curPower = _powerInit + myTimer.getTime() * timeSlope;

    if (_side == -1)
    {
      moveLeft(curPower);
    }
    else
    {
      moveRight(curPower);
    }

    this_thread::sleep_for(5);
  }

  unlockBase();
}
void resetHeading()
{
  IMU.resetRotation();
  // IMU.resetHeading();
  headingOffset = 0;
}

void resetHeading(float _offset)
{
  IMU.resetRotation();
  headingOffset = _offset;
}

float getPitch()
{
  return IMU.pitch();
}
double normalizeTarget(double angle)
{
  // Adjust angle to be within +/-180 degrees of the inertial sensor's rotation
  if (angle - getHeading() > 180)
  {
    while (angle - getHeading() > 180)
      angle -= 360;
  }
  else if (angle - getHeading() < -180)
  {
    while (angle - getHeading() < -180)
      angle += 360;
  }
  return angle;
}
void scaleToMin(double &left_output, double &right_output, double min_output)
{
  // Scale outputs to ensure minimum voltage is met for both sides
  if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0)
  {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  }
  else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0)
  {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  }
  else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0)
  {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  }
  else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0)
  {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}
void scaleToMax(double &left_output, double &right_output, double max_output)
{
  // Scale outputs to ensure maximum voltage is not exceeded for both sides
  if (fabs(left_output) >= fabs(right_output) && left_output > max_output)
  {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  }
  else if (fabs(right_output) > fabs(left_output) && right_output > max_output)
  {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  }
  else if (fabs(left_output) > fabs(right_output) && left_output < -max_output)
  {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  }
  else if (fabs(right_output) > fabs(left_output) && right_output < -max_output)
  {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}
double getRadius(double x, double y, double x1, double y1, double angle)
{
  double delta_x = x1 - x;
  double delta_y = y1 - y;
  if ((2 * delta_y * sin(deg2rad(90 - angle))) == 0)
  {
    return 999;
  }
  return (delta_x * delta_x + delta_y * delta_y) / (2 * delta_y * sin(deg2rad(90 - angle)));
}
void setPistonL(bool _input)
{
  if (_input)
    PistonL.off();
  else
    PistonL.on();
}
void setPistonF(bool _input)
{
  if (_input)
    PistonF.off();
  else
    PistonF.on();
}
void setPistonG(bool _input)
{
  if (_input)
    PistonG.off();
  else
    PistonG.on();
}
void setPistonH(bool _input)
{
  if (_input)
    PistonH.off();
  else
    PistonH.on();
}
void setPistonD(bool _input)
{
  if (_input)
    PistonD.off();
  else
    PistonD.on();
}
void setPistonW(bool _input)
{
  if (_input)
    PistonW.off();
  else
    PistonW.on();
}

void setPistonI(bool _input)
{
  if (_input)
    PistonI.off();
  else
    PistonI.on();
}
void setPistonO(bool _input)
{
  if (_input)
    PistonO.off();
  else
    PistonO.on();
}
struct DistanceUnit
{
  vex::distance *sensor;

  float values[3] = {0, 0, 0};
  int index = 0;

  float filtered = 0;
  float lastValid = 0;

  bool initialized = false;
  DistanceUnit(vex::distance *s)
  {
    sensor = s;

    values[0] = values[1] = values[2] = 0;
    index = 0;

    filtered = 0;
    lastValid = 0;

    initialized = false;
  }
};
DistanceUnit frontUnit{&FrontDistance};
DistanceUnit backUnit{&BackDistance};
DistanceUnit leftUnit{&LeftDistance};
DistanceUnit rightUnit{&RightDistance};
void updateSensor(DistanceUnit &unit)
{
  float raw = unit.sensor->objectDistance(vex::distanceUnits::mm);

  if (raw <= 1 || raw >= 2000)
    return;

  if (!unit.initialized)
  {
    unit.values[0] = raw;
    unit.values[1] = raw;
    unit.values[2] = raw;
    unit.filtered = raw;
    unit.lastValid = raw;
    unit.initialized = true;
    return;
  }

  unit.values[unit.index] = raw;
  unit.index = (unit.index + 1) % 3;

  unit.filtered =
      (unit.values[0] + unit.values[1] + unit.values[2]) / 3.0;

  unit.lastValid = unit.filtered;
}
int Distance()
{
  // wait(200, msec);

  while (true)
  {
    updateSensor(frontUnit);
    updateSensor(backUnit);
    updateSensor(leftUnit);
    updateSensor(rightUnit);
    this_thread::sleep_for(20);

    // wait(20, msec);
  }
  return 0;
}
float getDistance(vex::distance &sensor)
{
  DistanceUnit *unit = nullptr;

  if (&sensor == frontUnit.sensor)
    unit = &frontUnit;
  else if (&sensor == backUnit.sensor)
  {
    unit = &backUnit;
  }
  else if (&sensor == leftUnit.sensor)
    unit = &leftUnit;
  else if (&sensor == rightUnit.sensor)
    unit = &rightUnit;

  if (unit == nullptr)
    return 0;

  // If not initialized yet, return raw reading temporarily
  if (!unit->initialized)
  {
    float raw = sensor.objectDistance(vex::distanceUnits::mm);

    return raw;
  }

  return unit->filtered;
}
float getDistanceNormal(vex::distance &distanceSense, float _heading, float distanceOffset)
{
  return (getDistance(distanceSense) + distanceOffset) * cos(deg2rad(_heading));
}
float hues[3] = {0, 0, 0};
float currHue;
float getOpticalHue()
{
  currHue = Optical1.hue();
  hues[0] = hues[1];
  hues[1] = hues[2];
  hues[2] = currHue;
  // cout << (hues[0] + hues[1] + hues[2]) / 3 << endl;
  return (hues[0] + hues[1] + hues[2]) / 3;
}
bool redirectState = false;
void setRedirectState(bool _input)
{
  Optical1.setLight(ledState::on);
  Optical1.setLightPower(100, vex::pct);
  // cout << redirectState;
  redirectState = _input;
}
int output = 0;
int getColorState(float _input)
{

  if ((getOpticalHue() >= 0) && (getOpticalHue() <= REDHUE))
  {
    output = 1;
  }
  else if ((getOpticalHue() >= 200) && (getOpticalHue() <= (BLUEHUE)))
  {
    output = -1;
  }
  else
  {
    output = 0;
  }
  if (ALLIANCECOLOR == 1)
  {
    output *= -1;
  }
  // cout << output << endl;
  return output;
}
float intake_speed = 0;
float intake_speedOne = 0;
float intake_speedTwo = 0;
float kicker_speed = 0;
float distanceSensed = 200;
bool neutralState = false;
bool pastNeutralState = false;
bool colorSort = false;
bool armReached = 0;
bool BumperPressed = 0;
bool kickerFire = 0;
bool kickSet = 0;
bool jammed = 0;
bool intakeIsJammed = 0;
bool matchLoadState;
bool IntakePistonState = 0;
bool middleGoalState = false;
bool prevMiddleGoalState = false;
bool prevLowGoalState = false;
bool lowGoalState = false;
void setIntakeSpeed(float _input, bool _isroll)
{
  intake_speed = _input;
}
void setIntakeSpeed(float _input)
{
  intake_speed = _input;
}

void setNeutralState(bool _input)
{
  neutralState = _input;
}
void setMiddleState(bool _input)
{
  middleGoalState = _input;
}
void setIntakePistonState(bool _input)
{
  IntakePistonState = _input;
}
int ready = 1;
int curcolor = 1;
float curDistance;
int ringCount = 0;
bool ringInNeutral = false;
bool intakeInit = false;
bool bucketState = false;

bool pastBucketState = false;
bool getIntakeInit()
{
  return intakeInit;
}
MyTimer redirectTimer;
MyTimer intakeJammedTimer;
void setBucket(bool _input)
{
  bucketState = _input;
}
void bucket()
{
  
}
void setLowGoalState(bool _input)
{
  lowGoalState = _input;
}
PID intakePID;

void initIntakePID()
{
  intakePID.setCoefficient(0.002, 0.0000, 0.0005); // P, I, D
  intakePID.setIMax(1.0);                          // limit I contribution to ±1 volt
  intakePID.setIRange(0.5);                        // allow integration within ±200 RPM
  intakePID.setErrorTolerance(5);
}
float softStartTimerIntake(float _powerInit, float _powerFinal, int _duration, int _time)
{

  float timeSlope = (_powerFinal - _powerInit) / _duration;
  if (_duration <= 0)
    return _powerFinal;
  if (_time >= _duration)
  {
    return _powerFinal;
  }

  return _powerInit + _time * timeSlope;

  // moveForward(_powerInit + myTimer.getTime() * timeSlope);
}
void intake()
{
  auto middleGoalTimer = MyTimer();
  auto lowGoalTimer = MyTimer();
  static float filteredRPM = 0.0f;
  static float prevFilteredRPM = 0.0f;
  static float filteredD = 0.0f;
  static float prevError = 0.0f;
  static float prevVoltage = 0.0f;
  initIntakePID();

  while (true)
  {
    // cout << intake_speed << endl;

    intake_speedOne = intake_speed;
    // cout << intake_speedOne << endl;

    if (middleGoalState != prevMiddleGoalState)
    {

      middleGoalTimer.reset();
    }
    if (lowGoalState != prevLowGoalState)
    {
      lowGoalTimer.reset();
    }
    if (middleGoalState == true)
    {
      if (intake_speed > 0)
      {
        // cout << "hi" << endl;
        intake_speedOne = softStartTimerIntake(85, 30, 1800, middleGoalTimer.getTime());
      }
    }
    if (lowGoalState == true)
    {
      if (intake_speed > 0)
      {
        // cout << "hi" << endl;

        intake_speedOne = softStartTimerIntake(-28, -22, 1800, lowGoalTimer.getTime());
      }
    }

    prevMiddleGoalState = middleGoalState;
    prevLowGoalState = lowGoalState;
    intakeInit = true;

    float targetRPM = intake_speedOne * MAX_RPM / 100.0;

    // cout << targetRPM << endl;

    if (PTOState == false)
    {
      if (intake_speedOne == 0)
      {
        filteredRPM = 0;
        prevFilteredRPM = 0;
        filteredD = 0;
        prevError = 0;
        prevVoltage = 0;
        Motor_Intake1.spin(directionType::fwd, (int)0, voltageUnits::mV);
        Motor_Intake2.spin(directionType::fwd, (int)0, voltageUnits::mV);
      }
      else if (fabs(intake_speedOne) == 100)
      {
        Motor_Intake1.spin(directionType::fwd, (int)100 * 130 * sign(intake_speedOne), voltageUnits::mV);
        Motor_Intake2.spin(directionType::fwd, (int)100 * 130 * sign(intake_speedOne), voltageUnits::mV);
      }
      else
      {

        // --- Read and Filter RPM ---
        float rawRPM = Motor_Intake1.velocity(vex::rpm) * INTAKEGEARRATIO;
        static bool first = true;

        filteredRPM = rpmAlpha * rawRPM + (1.0f - rpmAlpha) * filteredRPM;

        // --- Error ---
        float error = targetRPM - filteredRPM;

        // --- Feedforward ---
        float Vff = kV * targetRPM;

        // --- Continuous Nonlinear Gain Scheduling ---
        float absError = fabs(error);
        float kP = kP0 + kP1 * (absError / (absError + E0));

        // --- Acceleration (for rebound detection) ---
        float dOmega = (filteredRPM - prevFilteredRPM) / Intakedt;

        // Only suppress gain when RPM rising (rebound)
        float reboundScale = 1.0f / (1.0f + accelScale * std::max(0.0f, dOmega));
        reboundScale = clampMath(reboundScale, 0.4f, 1.0f);

        // Effective proportional gain
        float kP_eff = kP * reboundScale;

        // --- Proportional Contribution ---
        float Pout = kP_eff * error;

        // --- Derivative on Error ---
        float rawD = -(filteredRPM - prevFilteredRPM) / Intakedt;
        if (first)
        {
          filteredD = rawD;
          filteredRPM = rawRPM;
          prevFilteredRPM = rawRPM;
          first = false;
        }

        // Filter D
        filteredD = dAlpha * rawD + (1.0f - dAlpha) * filteredD;

        // Clamp D
        filteredD = clampMath(filteredD, -Dmax, Dmax);

        float Dout = kD * filteredD;

        // --- Total Feedback ---
        float Vfb = Pout + Dout;

        // --- Combine ---
        float voltage = Vff + Vfb;

        // --- Asymmetric Slew Limiting ---
        float delta = voltage - prevVoltage;

        if (delta > slewUp)
          voltage = prevVoltage + slewUp;
        else if (delta < -slewDown)
          voltage = prevVoltage - slewDown;

        // --- Clamp Voltage ---
        voltage = clampMath(voltage, -12.0f, 12.0f);

        // --- Apply ---
        Motor_Intake1.spin(directionType::fwd, voltage * 1000.0f, voltageUnits::mV);
        Motor_Intake2.spin(directionType::fwd, voltage * 1000.0f, voltageUnits::mV);

        // --- Save State ---
        prevFilteredRPM = filteredRPM;
        prevError = error;
        prevVoltage = voltage;

        // --- Debug ---
        // cout << Brain.timer(msec) << "," << rawRPM << "," << voltage << endl;
      }
    }
    // Motor_Intake3.spin(directionType::fwd, (int)130 * intake_speedTwo * ready, voltageUnits::mV);

    this_thread::sleep_for(3);
  }
}

void setKickerSpeed(float _input)
{
  kicker_speed = _input;
}
void setKick()
{
  kickSet = true;
  // cout<<"set";
}
void setKickerFire(bool _input)
{
  // cout<<"test";
  kickerFire = _input;
}
int kickerGetRotation()
{
  return Motor_Cata.position(deg);
}
void setMatchLoadState(bool _input)
{
  matchLoadState = _input;
}
void setJammed(bool _input)
{
  jammed = _input;
}

void matchLoadCycle()
{
}
void KickerTurn()
{
  while (true)
  {
    Motor_Cata.setBrake(vex::brakeType::hold);
    if (matchLoadState)
    {
      kicker_speed = 100;
    }
    else
    {

      kicker_speed = 0;
    }
    Motor_Cata.spin(directionType::fwd, (int)130 * kicker_speed, voltageUnits::mV);
    this_thread::sleep_for(5);
  }
}
// pct of max height(0-100)

float liftTarget = 0;
float liftOffset = 0;
float liftTimeStamp = 5; // ms
float prevPos = 0;
float liftVel = 0;
bool liftInit = true;

std::string mode = "";
void liftResetPos()
{
  liftOffset = (Motor_Lift.position(deg)) * LIFTRATIO;
}
void liftSetPos(float _input)
{
  liftOffset = _input;
}

float liftGetPos()
{
  // cout<<(Motor_Lift.position(deg) + Motor_Lift2.position(deg))/2 * 1/6 <<endl;
  // cout<<((Motor_Lift.position(deg) + Motor_Lift2.position(deg)) * LIFTRATIO / 2 - liftOffset) / LIFTRANGE * 100<<endl;
  return ((Motor_Lift.position(deg)) * LIFTRATIO - liftOffset) / LIFTRANGE * 100; // pct
}
void liftSetVel()
{
  liftVel = (liftGetPos() - prevPos) / liftTimeStamp; // pct/s
  prevPos = liftGetPos();
}
float liftGetVel()
{

  return liftVel;
}
void liftTurn(float _input)
{
  Motor_Lift.spin(vex::directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_Lift2.spin(vex::directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void lockLift()
{
  Motor_Lift.setBrake(vex::brakeType::hold);
  Motor_Lift2.setBrake(vex::brakeType::hold);
}
void liftSetMode(int _liftCounter)
{
  if (PISTONLIFT == true)
  {
    switch (_liftCounter % 2)
    {
    case 0:
      mode = "rest";
      break;
    case 1:
      mode = "max";
      break;
    }
  }
  else
  {
    if (_liftCounter == -1)
    {
      mode = "manual";
    }
    else
    {
      switch (_liftCounter % 4)
      {
      /*case 0:
        mode = "rest";
        break;
      case 1:
        mode = "return";
        break;
      case 2:
        mode = "max";
        break;
      case 3:
        mode = "return";
        break;
      case 4:
        mode = "rest";
        break;*/
      case 0:
        mode = "rest";
        break;
      case 1:
        mode = "2Ring";
        break;
      /*case 2:
        mode = "3Ring";
        break;*/
      case 2:
        mode = "return";
        break;
      case 3:
        mode = "max";
        break;
        /*case 5:
          mode = "return";
          break;
        case 6:
          mode = "3Ring";
          break;
        case 7:
          mode = "2Ring";
          break;
        case 8:
          mode = "rest";
          break;*/
      }
    }
  }
}
void lift()
{
  float output;
  PID liftpid;
  stringstream s;
  liftpid.setCoefficient(11, 0.12, 20);
  liftpid.setDTolerance(30);
  liftpid.setErrorTolerance(0.01);
  liftpid.setIMax(40);
  liftpid.setIRange(25);
  // MyTimer resetTimer;

  if (liftInit)
  {

    liftResetPos();
    s << liftGetPos();

    liftInit = false;
    mode = "rest";
    vex_log(V_Critical, s.str());
  }
  while (true)
  {
    lockLift();
    liftSetVel();
    // std::cout << liftpid.getOutput() << endl;
    if (mode == "rest")
    {
      if (PISTONLIFT == true)
      {
        setPistonL(false);
      }
      else
      {
        liftTarget = 0;
        liftpid.setTarget(0);
      }
    }

    else if (mode == "return")
    {
      liftTarget = 85;
      liftpid.setTarget(33);
    }
    else if (mode == "2Ring")
    {
      liftTarget = 1;
      liftpid.setTarget(9.5);
    }
    else if (mode == "3Ring")
    {
      liftTarget = 25;
      liftpid.setTarget(35);
    }
    else if (mode == "max")
    {
      if (PISTONLIFT == true)
      {
        setPistonL(true);
      }
      else
      {
        liftTarget = 100;
        liftpid.setTarget(100);
      }
    }
    liftpid.update(liftGetPos());
    if (mode == "manual")
    {
    }
    else
    {
      if (liftpid.getI() >= 40)
      {
        liftTurn(100 * sign(liftTarget - liftGetPos()));
      }
      /*else if(liftTarget == 0){
        liftTurn(0);

      }*/
      else
      {
        liftTurn(liftpid.getOutput());
      }
    }
    this_thread::sleep_for(5);
  }
}
bool prevdoinkState = false;
bool prevdoinkRState = false;
bool doinkRState = false;
bool doinkState = false;
void setDoink(bool doinkCommand)
{
  prevdoinkState = doinkState;
  doinkState = doinkCommand;
}
void setDoinkR(bool doinkCommand)
{
  prevdoinkRState = doinkRState;
  doinkRState = doinkCommand;
}
void doinker()
{
  while (true)
  {
    if (prevdoinkState == false && doinkState == true)
    {
      setPistonD(true);
    }
    else if (prevdoinkState == true && doinkState == false)
    {
      setPistonD(false);
    }
    if (prevdoinkRState == false && doinkRState == true)
    {
      setPistonI(true);
    }
    else if (prevdoinkRState == true && doinkRState == false)
    {
      setPistonI(false);
    }
    this_thread::sleep_for(5);
  }
}
void ellipseFunc(float x, float y, float a, float b)
{
  float x2;
  float y2;
  float y3;
  float yint;

  x2 = x - a;

  while (x2 <= x + a)
  {
    y2 = sqrt(sqre(b) - sqre(x2 - x) * sqre(b) / sqre(a)) + y;
    y3 = -sqrt(sqre(b) - sqre(x2 - x) * sqre(b) / sqre(a)) + y;
    yint = y3;
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawPixel(x2, y2);
    Brain.Screen.drawPixel(x2, y3);
    while (yint <= y2)
    {
      Brain.Screen.setPenColor(ClrBrown);
      Brain.Screen.drawPixel(x2, yint);
      yint += 1;
    }
    x2 += 1;
  }
}
void parabolaFunc(float x, float y, float a, float xstop)
{
  float y2;
  float x2;
  float x3;
  float xint;
  x2 = x;
  while (x2 <= xstop)
  {
    y2 = a * sqre(x2 - x) + y;
    x3 = 2 * x - x2;
    Brain.Screen.drawPixel(x2, y2);
    Brain.Screen.drawPixel(x3, y2);
    xint = x3;
    while (xint >= x2)
    {
      Brain.Screen.drawPixel(xint, y2);
      xint = xint - 1;
    }
    x2 += a;
  }
}
void emote()
{
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(red);

  Brain.Screen.setFont(vex::fontType::mono30);
  Brain.Screen.printAt(205, 136, "GG EZ");
  Brain.Screen.setPenColor(black);
  ellipseFunc(240, 50, 50, 15);
  parabolaFunc(240, 136, -0.01, 245);
  Brain.Screen.drawRectangle(230, 150, 20, 30, yellow);
  ellipseFunc(240, 180, 20, 5);
  Brain.Screen.drawRectangle(215, 180, 50, 25, black);
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.drawCircle(180, 90, 20);
  Brain.Screen.drawCircle(300, 90, 20);
}
bool prevClampState = false;
bool clampState = false;
void setClamp(bool clampCommand)
{
  prevClampState = clampState;
  clampState = clampCommand;
}
void clamp()
{
  
}