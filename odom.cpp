#include "vex.h"
#include "basic-func.h"
#include "auton-func.h"
#include "timer-stuff.h"
#include "PID.h"
#include "parameters.h"
#include "odom.h"
#include "string.h"
#include "iostream"
using namespace vex;
using std::cout;
using std::endl;
Odom MyOdom;
Odom::Odom()
{
    odomPosX = 0.0;
    odomPosY = 0.0;
    odomHeading = 0.0;
    lastForwardPos = 0;
    lastSidewaysPos = 0;
    lastHeading = 0;
    odomInitialized = false;
}

float Odom::getLeftPosOdom()
{
    return (Motor_BaseLF.position(deg) + Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2.0 * 360);
}

float Odom::getRightPosOdom()
{
    return (Motor_BaseRF.position(deg) + Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAM * 25.4 * 3.1416) / (2.0 * 360);
}

float Odom::getForwardPosOdom()
{
    return (Rotation2.position(deg) * (2 * 25.4 * 3.1416) / 360);

    // return (getRightPosOdom() + getLeftPosOdom()) / 2; // no encoder
}

float Odom::getSidewaysPosOdom()
{

    // cout << Rotation1.position(deg) << endl;
    // return 0;
    return (Rotation1.position(deg) * (2.72 * 25.4 * 3.1416) / (HTRACKERRATIO * 360));
}

void Odom::resetLeftPosOdom()
{
    Motor_BaseLF.resetPosition();
    Motor_BaseLM.resetPosition();
}

void Odom::resetRightPosOdom()
{
    Motor_BaseRF.resetPosition();
    Motor_BaseRM.resetPosition();
}

void Odom::resetPosOdom()
{
    odomPosX = 0.0;
    odomPosY = 0.0;
    lastForwardPos = getForwardPosOdom();
    lastSidewaysPos = getSidewaysPosOdom();
    lastHeading = getOdomHeading();
}

void Odom::setPosOdom(float _x, float _y)
{
    odomPosX = _x;
    odomPosY = _y;
    lastForwardPos = getForwardPosOdom();
    lastSidewaysPos = getSidewaysPosOdom();
    lastHeading = getOdomHeading();
}
void Odom::setHeadingOdom(float _heading)
{

    resetHeading(_heading);
}

float Odom::calTheta(float _dx, float _dy, int _direct)
{
    float theta = rad2deg(atan(_dx / _dy));
    if (_direct * _dy < 0)
    {
        if (_dy * _dx > 0)
            theta -= 180;
        else
            theta += 180;
    }
    return theta;
}
float Odom::getOdomPosX()
{
    return odomPosX;
}
float Odom::getOdomPosY()
{
    return odomPosY;
}
float Odom::getOdomHeading()
{
    return getHeading();
}
void Odom::updateOdomPos()
{

    static float currForwardPos, currHeading;
    static float currSidewaysPos;
    if (!odomInitialized)
    {
        lastForwardPos = getForwardPosOdom();
        lastSidewaysPos = getSidewaysPosOdom();
        lastHeading = getOdomHeading();
        odomInitialized = true;
        return;
    }

    currForwardPos = getForwardPosOdom();
    currSidewaysPos = getSidewaysPosOdom();
    currHeading = getOdomHeading();

    odomPosX += (currForwardPos - lastForwardPos) * sin(deg2rad(0.5 * (currHeading + lastHeading))) - (currSidewaysPos - lastSidewaysPos) * cos(deg2rad(0.5 * (currHeading + lastHeading)));
    odomPosY += (currForwardPos - lastForwardPos) * cos(deg2rad(0.5 * (currHeading + lastHeading))) + (currSidewaysPos - lastSidewaysPos) * sin(deg2rad(0.5 * (currHeading + lastHeading)));

    lastSidewaysPos = currSidewaysPos;
    lastForwardPos = currForwardPos;
    lastHeading = currHeading;
    // cout << odomPosX << endl;
    // cout << odomPosY << endl;

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("GPS: x:  %.1f  ", currHeading);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("GPS: x:  %.1f  y:  %.1f", odomPosX, odomPosY);
    // cout << odomPosX << endl;
}
void Odom::updateCrtPara()
{
    xDiff = xTarget - odomPosX;
    yDiff = yTarget - odomPosY;
    dLength = sqrt(sqre(xDiff) + sqre(yDiff));
    dTheta = abbs(headingTarget - odomHeading);
}
void Odom::odomPIDMove(float _xTarget, float _yTarget, int _direct, int _maxPower, int mode, float angleTol)
{
    int direct;
    // direct = 1 forward -1 back
    // continue = 1, PID = 0, heading 2
    direct = sign(_direct) * 1;
    xTarget = _xTarget;
    yTarget = _yTarget;
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("GPS: direct:  %.1f ", direct);
    updateCrtPara();
    float _targetHeading = calTheta(xDiff, yDiff, direct);

    if (fabs(getOdomHeading() - _targetHeading) > angleTol || mode != 1)
    {
        rotateToHeading(_targetHeading, 95);
    }
    softStartTimerForward(getCurVel(), _maxPower * direct, _maxPower);
    // had no brackets check out why if needed

    // PIDAngleRotateAbs(_targetHeading, 3.8, 0.125, 31.7, 0.6);

    // if heading move straight to target
    if (mode == 2)
    {

        posForwardAbsWithHeading(_maxPower, dLength * direct, _targetHeading);
    }
    else if (mode == 3)
    {
    }
    else
    {
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("GPS: pos:  %.1f ", getForwardPos());
        posForwardAbsWithHeading(_maxPower, 0.75 * dLength * direct, _targetHeading); // 0.75 * dLength * direct
        PIDPosForwardAbs(dLength * direct, _maxPower);
    }
}

void Odom::posForwardRelWithHeadingOdom(float _power, float _targetPos, float _xGoal, float _yGoal)
{
    float startPos = getForwardPos();
    while (fabs(getForwardPos() - startPos) < fabs(_targetPos))
    {
        float _targetHeading = calTheta(_xGoal - odomPosX, _yGoal - odomPosY);
        float headingError = shortDeg(_targetHeading - getHeading());
        float powerTurn = 2.3 * headingError; // 2.0 is kp
        if (fabs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20;
        moveLeft(_power + powerTurn);
        moveRight(_power - powerTurn);
        this_thread::sleep_for(5);
    }
    unlockBase();
}
void Odom::posForwardAbsWithHeadingOdom(float _power, float _targetPos, float _xGoal, float _yGoal)
{
    float targetPosRel = _targetPos - getForwardPos();
    float power = sign(targetPosRel) * fabs(_power);
    float _targetHeading = calTheta(_xGoal - odomPosX, _yGoal - odomPosY);
    posForwardRelWithHeading(power, targetPosRel, _targetHeading);
}

bool is_turning = false;
bool dir_change_start = false;
bool dir_change_end = false;
float correct_heading = 0;
void Odom::moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn)
{
    unlockBase();      // Stop chassis before moving
    is_turning = true; // Set turning state
    double threshold = 0.5;
    int add = dir > 0 ? 0 : 180;
    double max_slew_fwd = dir > 0 ? MAXSLEWFWD : MAXSLEWREV;
    double max_slew_rev = dir > 0 ? MAXSLEWFWD : MAXSLEWREV;
    bool min_speed = false;
    if (!exit)
    {
        // Adjust slew rates and min speed for chaining
        if (!dir_change_start && dir_change_end)
        {
            max_slew_fwd = dir > 0 ? 24 : MAXSLEWFWD;
            max_slew_rev = dir > 0 ? MAXSLEWREV : 24;
        }
        if (dir_change_start && !dir_change_end)
        {
            max_slew_fwd = dir > 0 ? MAXSLEWFWD : 24;
            max_slew_rev = dir > 0 ? 24 : MAXSLEWREV;
            min_speed = true;
        }
        if (!dir_change_start && !dir_change_end)
        {
            max_slew_fwd = 24;
            max_slew_rev = 24;
            min_speed = true;
        }
    }

    PID pid_distance = PID();
    pid_distance.setErrorTolerance(10);
    pid_distance.setCoefficient(0.5, 0.06, 8);
    PID pid_heading = PID();
    pid_heading.setCoefficient(4, 0.02, 5);
    pid_heading.setErrorTolerance(10);

    // Set PID targets for distance and heading
    pid_distance.setTarget(hypot(x - odomPosX, y - odomPosY));
    pid_distance.setIMax(0);
    pid_distance.setIRange(3);
    pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
    pid_distance.setSmallBigErrorDuration(50, 50);
    pid_distance.setDTolerance(5);

    pid_heading.setTarget(normalizeTarget(rad2deg(atan2(x - odomPosX, y - odomPosY)) + add));
    pid_heading.setIMax(0);
    pid_heading.setIRange(1);
    pid_heading.setSmallBigErrorTolerance(0, 0);
    pid_heading.setSmallBigErrorDuration(0, 0);
    pid_heading.setDTolerance(0);

    // Reset the chassis
    double start_time = Brain.timer(msec);
    double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
    double exittolerance = 1;
    bool perpendicular_line = false, prev_perpendicular_line = true;

    double current_angle = 0, overturn_value = 0;
    bool ch = true;

    // Main PID loop for moving to point
    while (Brain.timer(msec) - start_time <= time_limit_msec)
    {
        // Continuously update targets as robot moves
        pid_heading.setTarget(normalizeTarget(rad2deg(atan2(x - odomPosX, y - odomPosY)) + add));
        pid_distance.setTarget(hypot(x - odomPosX, y - odomPosY));
        current_angle = getOdomHeading();
        pid_distance.update(0);
        // Calculate drive output based on heading and distance
        left_output = pid_distance.getOutput() * cos(deg2rad(atan2(x - odomPosX, y - odomPosY) * 180 / M_PI + add - current_angle)) * dir;
        right_output = left_output;
        // Check if robot has crossed the perpendicular line to the target
        perpendicular_line = ((odomPosY - y) * -cos(deg2rad(normalizeTarget(current_angle + add))) <= (odomPosX - x) * sin(deg2rad(normalizeTarget(current_angle + add))) + exittolerance);
        if (perpendicular_line && !prev_perpendicular_line)
        {
            break;
        }
        prev_perpendicular_line = perpendicular_line;

        // Only apply heading correction if far from target
        if (hypot(x - odomPosX, y - odomPosY) > 100 && ch == true)
        {
            pid_heading.update(current_angle);
            correction_output = pid_heading.getOutput();
        }
        else
        {
            correction_output = 0;
            ch = false;
        }

        // Minimum Output Check
        if (min_speed)
        {
            scaleToMin(left_output, right_output, 5);
        }

        // Overturn logic for sharp turns
        overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
        if (overturn_value > 0 && overturn)
        {
            if (left_output > 0)
            {
                left_output -= overturn_value;
            }
            else
            {
                left_output += overturn_value;
            }
        }
        right_output = left_output;
        left_output = left_output + correction_output;
        right_output = right_output - correction_output;

        // Max Output Check
        scaleToMax(left_output, right_output, max_output);

        // Max Acceleration/Deceleration Check
        if (prev_left_output - left_output > max_slew_rev)
        {
            left_output = prev_left_output - max_slew_rev;
        }
        if (prev_right_output - right_output > max_slew_rev)
        {
            right_output = prev_right_output - max_slew_rev;
        }
        if (left_output - prev_left_output > max_slew_fwd)
        {
            left_output = prev_left_output + max_slew_fwd;
        }
        if (right_output - prev_right_output > max_slew_fwd)
        {
            right_output = prev_right_output + max_slew_fwd;
        }
        prev_left_output = left_output;
        prev_right_output = right_output;
        moveLeft(left_output * 100 / 12);
        moveRight(right_output * 100 / 12); // Apply output to chassis
        wait(10, msec);
    }
    if (exit == true)
    {
        prev_left_output = 0;
        prev_right_output = 0;
        lockBase(); // Stop at end if required
    }
    correct_heading = getOdomHeading(); // Update global heading
    is_turning = false;                 // Reset turning state
}
void Odom::boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn)
{
    unlockBase();      // Stop chassis before moving
    is_turning = true; // Set turning state
    double threshold = 0.5;
    int add = dir > 0 ? 0 : 180;
    double max_slew_fwd = dir > 0 ? MAXSLEWFWD : MAXSLEWREV;
    double max_slew_rev = dir > 0 ? MAXSLEWFWD : MAXSLEWREV;
    bool min_speed = false;
    if (!exit)
    {
        // Adjust slew rates and min speed for chaining
        if (!dir_change_start && dir_change_end)
        {
            max_slew_fwd = dir > 0 ? 24 : MAXSLEWFWD;
            max_slew_rev = dir > 0 ? MAXSLEWFWD : 24;
        }
        if (dir_change_start && !dir_change_end)
        {
            max_slew_fwd = dir > 0 ? MAXSLEWFWD : 24;
            max_slew_rev = dir > 0 ? 24 : MAXSLEWREV;
            min_speed = true;
        }
        if (!dir_change_start && !dir_change_end)
        {
            max_slew_fwd = 24;
            max_slew_rev = 24;
            min_speed = true;
        }
    }
    PID pid_distance = PID();
    pid_distance.setCoefficient(0.6, 0.06, 7.5);
    pid_distance.setErrorTolerance(10);
    PID pid_heading = PID();
    pid_heading.setCoefficient(2.7, 0.12, 26);
    pid_heading.setErrorTolerance(10);

    pid_distance.setTarget(0); // Target is dynamically updated
    pid_distance.setIMax(3);
    pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
    pid_distance.setSmallBigErrorDuration(50, 250);
    pid_distance.setDTolerance(5);

    pid_heading.setTarget(normalizeTarget(rad2deg(atan2(x - odomPosX, y - odomPosY))));
    pid_heading.setIMax(0);
    pid_heading.setIRange(1);
    pid_heading.setSmallBigErrorTolerance(0, 0);
    pid_heading.setSmallBigErrorDuration(0, 0);
    pid_heading.setDTolerance(0);

    double start_time = Brain.timer(msec);
    double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
    double exit_tolerance = 3;
    float chase_power = max_output / 9.8;
    bool perpendicular_line = false, prev_perpendicular_line = true;
    double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

    // Main PID loop for boomerang path
    while ((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec)
    {
        hypotenuse = hypot(odomPosX - x, odomPosY - y); // Distance to target
        // Calculate carrot point for path leading
        carrot_x = x - hypotenuse * sin(deg2rad(a + add)) * dlead;
        carrot_y = y - hypotenuse * cos(deg2rad(a + add)) * dlead;
        pid_distance.setTarget(hypot(carrot_x - odomPosX, carrot_y - odomPosY) * dir);
        current_angle = getOdomHeading();
        // Calculate drive output based on carrot point
        pid_distance.update(0);
        left_output = pid_distance.getOutput() * cos(deg2rad(atan2(carrot_x - odomPosX, carrot_y - odomPosY) * 180 / M_PI + add - current_angle));
        right_output = left_output;
        // Check if robot has crossed the perpendicular line to the target
        perpendicular_line = ((odomPosY - y) * -cos(deg2rad(normalizeTarget(a))) <= (odomPosX - x) * sin(deg2rad(normalizeTarget(a))) + exit_tolerance);
        if (perpendicular_line && !prev_perpendicular_line)
        {
            break;
        }
        prev_perpendicular_line = perpendicular_line;

        // Minimum Output Check
        if (min_speed)
        {
            scaleToMin(left_output, right_output, 2);
        }

        // Heading correction logic based on distance to carrot/target
        if (hypot(carrot_x - odomPosX, carrot_y - odomPosY) > 100)
        {
            pid_heading.setTarget(normalizeTarget(rad2deg(atan2(carrot_x - odomPosX, carrot_y - odomPosY)) + add));
            pid_heading.update(current_angle);
            correction_output = pid_heading.getOutput();
        }
        else if (hypot(x - odomPosX, y - odomPosY) > 80)
        {
            pid_heading.setTarget(normalizeTarget(rad2deg(atan2(x - odomPosX, y - odomPosY)) + add));
            correction_output = pid_heading.getOutput();
        }
        else
        {
            pid_heading.setTarget(normalizeTarget(a));
            correction_output = pid_heading.getOutput();
            if (exit && hypot(x - odomPosX, y - odomPosY) < 60)
            {
                break;
            }
        }

        // Limit slip speed for smoother curves
        slip_speed = sqrt(chase_power * getRadius(odomPosX, odomPosY, carrot_x, carrot_y, current_angle) * 9.8);
        if (left_output > slip_speed)
        {
            left_output = slip_speed;
        }
        else if (left_output < -slip_speed)
        {
            left_output = -slip_speed;
        }

        // Overturn logic for sharp turns
        overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
        if (overturn_value > 0 && overturn)
        {
            if (left_output > 0)
            {
                left_output -= overturn_value;
            }
            else
            {
                left_output += overturn_value;
            }
        }
        right_output = left_output;
        left_output = left_output + correction_output;
        right_output = right_output - correction_output;

        // Max Output Check
        scaleToMax(left_output, right_output, max_output);

        // Max Acceleration/Deceleration Check
        if (prev_left_output - left_output > max_slew_rev)
        {
            left_output = prev_left_output - max_slew_rev;
        }
        if (prev_right_output - right_output > max_slew_rev)
        {
            right_output = prev_right_output - max_slew_rev;
        }
        if (left_output - prev_left_output > max_slew_fwd)
        {
            left_output = prev_left_output + max_slew_fwd;
        }
        if (right_output - prev_right_output > max_slew_fwd)
        {
            right_output = prev_right_output + max_slew_fwd;
        }
        prev_left_output = left_output;
        prev_right_output = right_output;
        moveLeft(left_output * 100 / 12);
        moveRight(right_output * 100 / 12);
        // Apply output to chassis
        wait(10, msec);
    }
    if (exit)
    {
        prev_left_output = 0;
        prev_right_output = 0;
        lockBase(); // Stop at end if required
    }
    correct_heading = a; // Update global heading
    is_turning = false;  // Reset turning state
}
/*bool odomInit = false;
bool getOdomInit()
{
    return odomInit;
}*/

void MyOdomPos()
{
    while (1)
    {

        MyOdom.updateOdomPos();
        // odomInit = true;
        this_thread::sleep_for(5);
    }
}