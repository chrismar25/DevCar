Servo myservo; //myservo is an object of class Servo.
//NewPing sA(5,5,MaxDistAE); // sA is an object of class NewPing with trig_pin = echo_pin = 5
VL53L1X sB; // sB is a LIDAR sensor of the type VL53L1X.
//NewPing sB(7,7,MaxDist); // sB is an object of class NewPing
//NewPing sC(8,8,MaxDist); // sC is an object of class NewPing
NewPing sC(5,5,MaxDist); // sC is an object of class NewPing
VL53L1X sD; // sD is a LIDAR sensor of the type VL53L1X.
//NewPing sD(10,10,MaxDist); // sD is an object of class NewPing
//NewPing sE(11,11,MaxDistAE); // sE is an object of class NewPing. It has happened that sE reported e = 1149 cm > MaxDistAE.
//It should not happen.
int StartSig = 1; //Used with the start module for remote start. The car should not start until StartSig is set to 1 by 
//a remote control.
int KillSig = 1; // Startsig = 0 combined with KillSig = 1 is considered as the power on state but not the start state.
int i;
int imax = 7; //
int dt_Pings2 = 30; //50; //25;  // 2*dt_Pings2 >= 50
int dt_Pings3 = 30; //34; //17;   // 3*dt_Pings3 >= 50. Use at least 50 ms between pings of the same sensor.
int T = 20; //The inter-measurement period that should be at least as long as the TimingBudget for LIDAR sensors.
int v_start = 100;
int v_straight = 120; //150; //Speed straight forward in 3aS and 3bS.
int v_corr = 100; //120; //Speed at small corrections of the direction in steps 3aL, 3aR, 3bL, 3bR.
int v_turn = 90; //100; //Speed in sharp curves. 80 gives too low torque. The motor needs other gear wheels.


int bold; //
int usold = 51; //
int dold; //
//const int incMax = 10; //Max acceptable increase in b or d between measurements without turning sharply in step 4.
//const int cdecmax = 10*v_straight/120*(2*T+dt_Pings3)/70; //If c decreases more than this between 2 pings
//the cause can be the sudden appearance of another car or turning towards a wall or an erroneous reading. 
//const int amin = 8; // DevCar can get stuck on a wall unless there is a minimum allowed distance.
const int usmin = 50; //Drive straight ahead if us > usmin. DevCar may turn the wrong way in scenario 5a, 5b if cmin is
//larger than the width of the track.
//Dev car has a turn radius of about 40 cm to the outer wheel.
const int srightmin = 11; //Turn left if b < bmin, c < cmin and d > dmin. Tried 40, 20, 30
const int sleftmin = 11; //Turn right if sleft < sleftmin, c < cmin and b > bmin. Tried 40, 20, 30
int srightmax = 0; //The max value is likely to become about sqrt(2) times the distance to the wall to the right
int sleftmax = 0; //The max value is likely to become about sqrt(2) times the distance to the wall to the left
//const int akeepmax = 45; //The width of the racing track must have room for emin + car_width + bkeep.
int srightkeep;
const int srightkeepmin = 20; //Larger than or equal to bmin + tol
const int srightkeepmax = 30;
const int srightlarge = 80;
int srightturn; //b directly after turning right.
int sleftkeep;
const int sleftkeepmin = srightkeepmin; //
const int sleftkeepmax = srightkeepmax;
const int sleftlarge = srightlarge;
int sleftturn; //d directly after turning left.
int lwallflag; //lwallflag = 1 if DevCar should stay at a constant d = dkeep, i.e. follow the left wall
const int tol2 = 12; //Tolerance for large direction corrections needed in sharp curves and at some obstacles
const int tol = 6; //Tolerance for small direction corrections 
