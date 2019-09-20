

//Preliminary code for folkrace at RobotSM 2019

#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <VL53L1X.h>

#define MaxDistAE 400 // Max distance to detect for sensors A and E. 
#define MaxDist 60 // Max distance chosen for sensors B, C and D. The sonar function ping_cm() returns zero
//if there is no ping echo within set distance limit MaxDist.

/////////////////////////
// variable_assign
/////////////////////////


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



/////////////////
// functions
/////////////////
int read_sright(void){ //Read the distance b from a LIDAR sensor.
  int s1;
      sB.startContinuous(T); 
      s1 = sB.read()/10;//Read b. 
      sB.stopContinuous(); 
      if (sB.timeoutOccurred()) { Serial.print("TIMEOUT for sB: Distance out of range for timing budget"); }
      Serial.print("sright:");
      Serial.print(s1);
      Serial.println("cm");
      return s1;
}
int read_us(void){ //Read distance c from the ultrasonic sensor.
  int s2;
      s2 = sC.ping_cm();//Ping and measure c. 
      delay(dt_Pings3);
      if (s2 == 0 || s2 >= MaxDist) //The distance is either above the upper distance limit or the supplied power is unstable.
        s2 = MaxDist; //An erroneous sensor reading should not cause an exit from the loop.
      Serial.print("us:");
      Serial.print(s2);
      Serial.println("cm");
      return s2;
}
int read_sleft(void){ //Read the distance d from a LIDAR sensor.
   int s1;
      sD.startContinuous(T); 
      s1 = sD.read()/10;//Read d. 
      sD.stopContinuous(); 
      if (sD.timeoutOccurred()) { Serial.print("TIMEOUT for sB: Distance out of range for timing budget"); }
      Serial.print("sleft:");
      Serial.print(s1);
      Serial.println("cm");
      return s1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          VOID SETUP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  //Setup commands for LIDAR sensors:
  Wire.begin(); //Initiate the Wire library and join the I2C bus as a master unless the adress is specified.
  Wire.setClock(400000); // Set the clock frequency for I2C communication to 400 kHz (fast mode).
  pinMode(7, OUTPUT); // XSHUT port on sB set as OUTPUT for subsequent digitalWrite.
  pinMode(10, OUTPUT); // XSHUT port on sD set as OUTPUT for subsequent digitalWrite.
  digitalWrite(7, LOW);
  digitalWrite(10, LOW);
  pinMode(7, INPUT); //Return to INPUT mode (high impedance). Don't use digitalWrite(7, HIGH) since the XSHUT port
  //is not 5 V tolerant.
  sB.init(); //Reset the sensor. It will also reset the address. Therefore, run init before setting the address.
  sB.setAddress(1); //Arbitrary 7-bit address
  pinMode(10, INPUT);//Return to INPUT mode (high impedance). Don't use digitalWrite(7, HIGH) since the XSHUT port
  //is not 5 V tolerant.
  sD.init();
  sD.setAddress(2);
  sB.setDistanceMode(VL53L1X::Short); //In short distance mode, distance up to 1.3 m can be measured almost independently
  //of ambient light
  sD.setDistanceMode(VL53L1X::Short);
  sB.setMeasurementTimingBudget(20000); // Minimum time for each measurement. The minimum value is 20000 us, only allowed
  //in Short distance mode.
  sD.setMeasurementTimingBudget(20000);
  //sB.startContinuous(T); // It may be energy saving to measure when needed instead of continuously.
  //sD.startContinuous(T);// Start continuous readings at a rate of one measurement every 20 ms (the
  // inter-measurement period). This period should be at least as long as the timing budget.
  //End of setup command block for LIDAR sensors.
  pinMode(12, OUTPUT); //Initiates Motor shield Channel A pin for rotation direction of the Main Motor. Channel B is not used.
  pinMode(9, OUTPUT); //Initiates Motor shield Channel A pin for brake control of the Main Motor
  pinMode(2, OUTPUT); // Start signal digital pin connected to the starting module. A referee can set the value remotely
  //if the pin mode is INPUT.
  pinMode(4, OUTPUT); // Kill signal digital pin connected to the starting module. A referee will set the value remotely
  //if the pin mode is INPUT.
  digitalWrite(2, StartSig); //Set a value to the start signal pin. digitalWrite can only have
  //effect if both pins 2 and 4 are OUTPUT.
  digitalWrite(4, KillSig); //Set a value to the kill signal pin. digitalWrite can only have 
  //effect if both pins 2 and 4 are OUTPUT.




//////////////////////////////////////////////////////////////////////////////////
//   Step 1: Start
//   Start DevCar when the start signal is set to 1 remotely via the start module.
/////////////////////////////////////////////////////////////////////////////////
  myservo.attach(6); // Attach the signal pin of servo to digital pin 6 of arduino
  StartSig = digitalRead(2); //Read Start Signal Pin
  KillSig = digitalRead(4);  //Read kill Switch Pin
  Serial.print("StartSig = ");
  Serial.println(StartSig);
  //Serial.print("KillSig = ");
  //Serial.println(KillSig);
  if (StartSig == 1 && KillSig == 1) {
    //if(StartSig == 1){

    

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   Step 2: Drive straight forward briefly while checking distances to the walls and determining which wall to follow.
//
//   Drive slowly straight forward initially and measure the distances to the left and right wall and choose to follow
//   to nearest wall at an approximately constant distance. Set a flag, a “wall_flag”, equal to 1 if DevCar should follow 
//   the left wall. Set the flag to 0 if DevCar should follow the right wall. 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    myservo.write(110); //Set front wheel angular direction to be straight forward which means angle 110°-111° for the modified
    //servo mechanism!
    digitalWrite(12, HIGH); //Establishes forward direction of the motor connected to shield Channel A
    digitalWrite(9, LOW);   //Disengage the brake for the motor connected to Channel A
    analogWrite(3, v_start); //Sets the motor on Channel A at a speed <= 255. We have to adapt the speed setting to the speed
    //obtained in tests.
    for (i = 1; i <= imax; i++) {
      sright = read_sright();
      sleft = read_sleft();
      if (sright > srightmax)  //It may be better to rely on bmax than on an average value
        srightmax = sright;
      if (sleft > sleftmax)
        sleftmax = sleft;
    }
    if (sleftmax <= srightmax) {
      lwallflag = 1;
      sleftkeep = min(sleftkeepmax, max(sleft, sleftmin));
    }
    else {
      lwallflag = 0;
      srightkeep = min(srightkeepmax, max(sright, srightmin)); //Try to keep b equal to bkeep
    }
    Serial.print("srightmax =");
    Serial.print(srightmax);
    Serial.println("cm");
    Serial.print("sleftmax =");
    Serial.print(sleftmax);
    Serial.println("cm");
    Serial.print("srightkeep =");
    Serial.print(srightkeep);
    Serial.println("cm");
    Serial.print("sleftkeep =");
    Serial.print(sleftkeep);
    Serial.println("cm");
    us = read_us();
    sright = read_sright();
    sleft = read_sleft();
  }
}
}

void loop() {
  // put your main code here, to run repeatedly:
  //In folkrace the kill switch will not be used.




//////////////////////////////////////////////////////////////////////////////////
//  Step 3: Set speed to medium high
//
//  Check if the referee has set the start signal to 0. Continue if start signal 
//  is still 1. Check wall_flag. Set a medium high speed. 
//////////////////////////////////////////////////////////////////////////////////
  Serial.print("lwallflag:");
  Serial.println(lwallflag);
  analogWrite(3, v_corr); //Set a medium high speed.


///////////////////////////////////////////////////////////////////////////////////////////////
//  Step 4: Small or large direction corrections and straight forward:
//
//  if wall_flag == 1, repeatedly, aim at approaching a constant distance to the left wall.
//  Increase the speed to a high speed if DevCar is within a tolerance from a constant distance
//  from the chosen constant distance.
////////////////////////////////////////////////////////////////////////////////////////////////

  if (lwallflag == 1) {//Scenario 4a. Keep d constant

    //TURN SHARPLY RIGHT IF-FOR myservo.write()
    if (((sleftkeep - sleft) > tol2) && (us > usmin) && (sright > srightmin)) {
      for (i = 105; i >= 85; i = i - 5) {
        myservo.write(i); //Turn the wheels sharply to the right.
        delay(1);
      }
      Serial.println("Scenario 4aRR. Turn sharply right!");
      while (((sleftkeep - sleft) > tol2) && (us > usmin) && (sright > srightmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("sleftkeep after loop:");
      Serial.print(sleftkeep);
      Serial.println("cm");
    }
    if (((sleftkeep - sleft) > tol) && (us > usmin) && (sright > srightmin)) {
      myservo.write(98); //Turn the wheels to the right.
      Serial.println("Scenario 4aR. Turn right!");
      while (((sleftkeep - sleft) > tol) && (us > usmin) && (sright > srightmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("sleftkeep after loop:");
      Serial.print(sleftkeep);
      Serial.println("cm");
    }
    
    //TURN SHARPLY LEFT IF-FOR myservo.write()
    if (((sleft - sleftkeep) > tol2) && (us > usmin) && (sright > srightmin)) {
      for (i = 115; i <= 135; i = i + 5) {
        myservo.write(i); //Turn the wheels to the left.
        delay(1);
      }
      Serial.println("Scenario 4aLL. Turn sharply left.");
      while (((sleft - sleftkeep) > tol2) && (us > usmin) && (sright > srightmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;} //Not good
      }
      Serial.print("sleftkeep after loop:");
      Serial.print(sleftkeep);
      Serial.println("cm");
    }
    if (((sleft - sleftkeep) > tol) && (us > usmin) && (sright > srightmin)) {
      myservo.write(120); //Turn the wheels to the left.
      Serial.println("Scenario 4aL. Turn left.");
      while (((sleft - sleftkeep) > tol) && (us > usmin) && (sright > srightmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;} //Not good
      }
      Serial.print("sleftkeep after loop:");
      Serial.print(sleftkeep);
      Serial.println("cm");
    }

    
    //STRAIGHT FORWARD 
    if (((sleft - sleftkeep) <= tol) && ((sleftkeep - sleft) <= tol) && (us > usmin) && (sright > srightmin)) {
      myservo.write(110); // Drive straight forward
      analogWrite(3, v_straight); //Set a high speed
      Serial.println("Scenario 4aS. Drive straight forward.");
      while (((sleft - sleftkeep) <= tol) && ((sleftkeep - sleft) <= tol) && (us > usmin) && (sright > srightmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("sleftkeep after loop:");
      Serial.print(sleftkeep);
      Serial.println("cm");
    }
  }
  else {//Scenario 4b. Keep a constant a
    if (((srightkeep - sright) > tol2) && (us > usmin) && (sleft > sleftmin)) {
      for (i = 115; i <= 135; i = i + 5) {
        myservo.write(i); //Turn the wheels to the left.
        delay(1);
      }
      Serial.println("Scenario 4LL. Turn sharply left.");
      while (((srightkeep - sright) > tol2) && (us > usmin) && (sleft > sleftmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("srightkeep after loop:");
      Serial.print(srightkeep);
      Serial.println("cm");
    }
    if (((srightkeep - sright) > tol) && (us > usmin) && (sleft > sleftmin)) {
      myservo.write(120); //Turn the wheels to the left.
      Serial.println("Scenario 4bL. Turn left.");
      while (((srightkeep - sright) > tol) && (us > usmin) && (sleft > sleftmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("srightkeep after loop:");
      Serial.print(srightkeep);
      Serial.println("cm");
    }
    if (((sright - srightkeep) > tol2) && (us > usmin) && (sleft > sleftmin)) {
      for (i = 105; i >= 85; i = i - 5) {
        myservo.write(i); //Turn the wheels to the right.
        delay(1);
      }
      Serial.println("Scenario 4bRR. Turn sharply right.");
      while (((sright - srightkeep) > tol2) && (us > usmin) && (sleft > sleftmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("srightkeep after loop:");
      Serial.print(srightkeep);
      Serial.println("cm");
    }
    if (((sright - srightkeep) > tol) && (us > usmin) && (sleft > sleftmin)) {
      myservo.write(98); //Turn the wheels to the right.
      Serial.println("Scenario 4bR. Turn right.");
      while (((sright - srightkeep) > tol) && (us > usmin) && (sleft > sleftmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("srightkeep after loop:");
      Serial.print(srightkeep);
      Serial.println("cm");
    }
    if (((sright - srightkeep) <= tol) && ((srightkeep - sright) <= tol) && (us > usmin) && (sleft > sleftmin)) {
      myservo.write(110); // Drive straight forward
      analogWrite(3, v_straight); //Set a high speed
      Serial.println("Scenario 4bS. Drive straight forward.");
      while (((sright - srightkeep) <= tol) && ((srightkeep - sright) <= tol) && (us > usmin) && (sleft > sleftmin)) {
        usold = us;
        sright = read_sright();
        us = read_us();
        sleft = read_sleft();
        //if (usold-us > cdecmax){us = usold;}
      }
      Serial.print("srightkeep after loop:");
      Serial.print(srightkeep);
      Serial.println("cm");
    }
  }
  //Exit from step 4 above should only take place if us < usmin or b < bmin or d < dmin.



///////////////////////////////////////////////////////////////////////////////////////////////////
//  Step 5: Slow down or stop temporarily to get time to measure distances.
//
//  Slow down or stop temporarily in front of an obstacle. 
//////////////////////////////////////////////////////////////////////////////////////////////////
   analogWrite(3, v_turn); //Slow down in front of an obstacle.




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Step 6: Avoid obstacles and turn in curves not already handled by step 4.
//
//  Avoid obstacles and turn in curves not already handled by step 4. Step 6 is subdivided into a number of scenarios. 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (sleft >= sleftmin && us <= usmin && sleft >= sright) { //Scenario 6a
   for(i = 115; i <= 135; i = i + 5){
    myservo.write(i); //myservo.write(135); // Turn sharply left
    delay(1);
   }  
  Serial.println("Scenario 6a. Turn sharply left.");
  while (us <= usmin){
    usold = us;
    us = read_us();
    //if (usold-us > cdecmax){us = usold;}
  }
  for(i = 130; i >= 110; i = i - 5){
      myservo.write(i); //Set wheels straight forward.
      delay(1);
  }
  sright = read_sright();
  if (sright > srightlarge){ //An obstacle was passed and was not a curve.
    Serial.println("The car has passed the obstacle which was not a curve.");
    sleft = read_sleft();
    sleftturn = sleft;
    //The obstacle was likely to be another robot.
    for(i = 105; i >= 85; i = i - 5){ 
      myservo.write(i); //myservo.write(85); // Turn sharply right to get DevCar parallel to a wall again.
      delay(1);
    }  
    while (sright > srightkeepmin && us > usmin && sleft < sleftturn*2){
      usold = us;
      sright = read_sright();
      us = read_us();
      sleft = read_sleft();
      //if (usold-us > cdecmax){us = usold;}  
    }
    for(i = 90; i <= 110; i = i + 5){
      myservo.write(i); //Set wheels straight forward again.
      delay(1);
    }
    lwallflag = 1;
    sleftkeep = min(sleftkeepmax,max(sleft,sleftkeepmin));
    Serial.print("sleftkeep:");
    Serial.print(sleftkeep);
    Serial.println("cm");
  }
  else{//DevCar passed through a curve to the left. Follow the right wall. 
    lwallflag = 0;
    srightkeep = min(srightkeepmax,max(sright,srightkeepmin));
    Serial.print("srightkeep:");
    Serial.print(srightkeep);
    Serial.println("cm");
    Serial.println("DevCar passed through a curve to the left.");
  }
  }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if (sright >= srightmin && us <= usmin && sright > sleft) { //Scenario 6b.
   for(i = 105; i >= 85; i = i - 5){
    myservo.write(i); //myservo.write(85); // Turn sharply right
    delay(1);
  }
  Serial.println("Scenario 6b. Turn sharply right.");
   while (us <= usmin){
    usold = us;
    us = read_us();
    //if (usold-us > cdecmax){us = usold;}
  }  
   for(i = 90; i <= 110; i = i + 5){
    myservo.write(i); //Set wheels straight forward.
    delay(1);
  }
  sleft = read_sleft();
  if (sleft > sleftlarge){//The car has passed the obstacle which was not a curve.
    Serial.println("The car has passed the obstacle which was not a curve.");
    sright = read_sright();
    srightturn = sright;
    for(i = 115; i <= 135; i = i + 5){
      myservo.write(i); //myservo.write(135); // Turn sharply left to get DevCar parallel to a wall again.
      delay(1);
    }  
    while (sleft > sleftkeepmin && us > usmin && sright < srightturn*2){
      usold = us;
      sright = read_sright();
      us = read_us();
      sleft = read_sleft();
      //if (usold-us > cdecmax){us = usold;}
    }
    for(i = 130; i >= 110; i = i - 5){
      myservo.write(i); //Set wheels straight forward again.
      delay(1);
    }
    lwallflag = 0;
    srightkeep = min(srightkeepmax,max(sright,srightkeepmin));
  }
  else {//DevCar has passed through a curve to the right.
    lwallflag = 1;
    sleftkeep = min(sleftkeepmax,max(sleft,dkeepmin)); //Follow the left wall.
    Serial.println("DevCar passed through a curve to the right.");
  }
}  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if (sright <= srightmin && us >= usmin && sleft > sleftmin){ //There is an obstacle to the right. Turn left somewhat.
   Serial.println("Scenario 6c. Turn somewhat left.");
   for(i = 115; i <= 125; i = i + 5){
      myservo.write(i); //Turn the wheels to the left.
      delay(1);
  }
   while(sright <= srightmin && us >= usmin && sleft >= sleftmin){
    usold = us;
    sright = read_sright();
    us = read_us();
    sleft = read_sleft();
    //if (usold-us > cdecmax){us = usold;}
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if (sleft <= sleftmin && us >= usmin && sright > srightmin){ //There is an obstacle to the left. Turn right somewhat.
   Serial.println("Scenario 6d. Turn somewhat right.");
   for(i = 105; i >= 95; i = i - 5){
      myservo.write(i); //Turn the wheels to the right.
      delay(1);
  }
   while(sleft <= sleftmin && us >= usmin && sright >= srightmin){
    usold = us;
    sright = read_sright();
    us = read_us();
    sleft = read_sleft();
    //if (usold-us > cdecmax){us = usold;}
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if (sright <= srightmin && sleft <= sleftmin) { //Scenario 6e is unlikely
   digitalWrite(9, HIGH);   //Stop the car. Engage the brake for the motor connected to Channel A
   Serial.println("Scenario 6e. Stop and wait.");
   while (sright <= srightmin && sleft <= sleftmin) {
    sright = read_sright();
    sleft = read_sleft();      
  }
  digitalWrite(9, LOW);   //Start the car.
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Step 7: Read sensor values outside all other steps to handle cases not covered by the previous steps
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//b = read_b();
//c = read_c();
//d = read_d();
//else if(StartSig == 0 && KillSig == 0){
//  digitalWrite(9, HIGH);   //Stop the car. Engage the brake for the motor connected to Channel A
//  Serial.print("StartSig again = ");
//  Serial.println(StartSig);
//  Serial.print("KillSig again = ");
//  Serial.println(KillSig);
//}  
}
