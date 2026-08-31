#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Vex devices
controller Controller1 = controller(primary);

#ifdef ROBOT1
motor Motor1 = motor(PORT3, ratio6_1,true);
motor Motor2 = motor(PORT3, ratio6_1,true);


motor Motor_BaseLF  = motor(PORT18, ratio6_1, true);
motor Motor_BaseLM  = motor(PORT9, ratio6_1, false);
motor Motor_BaseLB  = motor(PORT10,  ratio6_1, true);
motor Motor_BaseLBT = motor(PORT7,  ratio6_1, true);
motor Motor_BaseLFT = motor(PORT19, ratio6_1, false);

motor Motor_BaseRF  = motor(PORT21, ratio6_1, false);
motor Motor_BaseRM  = motor(PORT17, ratio6_1, false);
motor Motor_BaseRB  = motor(PORT2, ratio6_1, false);
motor Motor_BaseRBT = motor(PORT1,  ratio6_1, true);
motor Motor_BaseRFT = motor(PORT4,  ratio6_1, true);


motor Motor_Lift    = motor(PORT3, ratio6_1, false);
motor Motor_Lift2   = motor(PORT3, ratio6_1, false);
motor Motor_Intake1  = motor(PORT3, ratio6_1, false);
motor Motor_Intake2  = motor(PORT3, ratio6_1, false);
motor Motor_Intake3  = motor(PORT3, ratio6_1, false);


// sensors
limit Bumper1 = limit(Brain.ThreeWirePort.F);
// distance
distance IntakeDistance = distance(PORT3);
distance RightDistance = distance(PORT3);
distance LeftDistance = distance(PORT3);
distance FrontDistance = distance(PORT3);
distance BackDistance = distance(PORT3);

optical Optical1 = optical(PORT3);
// temp for prototyping
inertial IMU = inertial(PORT3);
rotation Rotation1 = rotation(PORT3, false);
rotation Rotation2 = rotation(PORT3, false);

// pistons
led PistonL = led(Brain.ThreeWirePort.E); // midgoal score
led PistonI = led(Brain.ThreeWirePort.G); // matchload
led PistonW = led(Brain.ThreeWirePort.D); // long wing
led PistonD = led(Brain.ThreeWirePort.C); // lock
led PistonO = led(Brain.ThreeWirePort.A); // mid dc
led PistonH = led(Brain.ThreeWirePort.F); // low goal
led PistonF = led(Brain.ThreeWirePort.B); // left midgoal wing
led PistonG = led(Brain.ThreeWirePort.H); // right midgoal wing
#endif
bool RemoteEnabled = true;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
  // Nothing to initialize
}