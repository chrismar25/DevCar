

//Preliminary code for folkrace at RobotSM 2019

#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <VL53L1X.h>

#define MaxDistAE 400 // Max distance to detect for sensors A and E. 
#define MaxDist 60 // Max distance chosen for sensors B, C and D. The sonar function ping_cm() returns zero
//if there is no ping echo within set distance limit MaxDist.


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


  ////////////////////////////////
  //Step 1: Start:
  ////////////////////////////////
  myservo.attach(6); // Attach the signal pin of servo to digital pin 6 of arduino
  StartSig = digitalRead(2); //Read Start Signal Pin
  KillSig = digitalRead(4);  //Read kill Switch Pin
  Serial.print("StartSig = ");
  Serial.println(StartSig);
  //Serial.print("KillSig = ");
  //Serial.println(KillSig);
  if (StartSig == 1 && KillSig == 1) {
    //if(StartSig == 1){

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 2: Drive straight forward briefly while checking distances to the walls and determining which wall to follow:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    myservo.write(110); //Set front wheel angular direction to be straight forward which means angle 110°-111° for the modified
    //servo mechanism!
    digitalWrite(12, HIGH); //Establishes forward direction of the motor connected to shield Channel A
    digitalWrite(9, LOW);   //Disengage the brake for the motor connected to Channel A
    analogWrite(3, v_start); //Sets the motor on Channel A at a speed <= 255. We have to adapt the speed setting to the speed
    //obtained in tests.
    for (i = 1; i <= imax; i++) {
      b = read_b();
      d = read_d();
      if (b > bmax)  //It may be better to rely on bmax than on an average value
        bmax = b;
      if (d > dmax)
        dmax = d;
    }
    if (dmax <= bmax) {
      dflag = 1;
      dkeep = min(dkeepmax, max(d, dmin));
    }
    else {
      dflag = 0;
      bkeep = min(bkeepmax, max(b, bmin)); //Try to keep b equal to bkeep
    }
    Serial.print("bmax =");
    Serial.print(bmax);
    Serial.println("cm");
    Serial.print("dmax =");
    Serial.print(dmax);
    Serial.println("cm");
    Serial.print("bkeep =");
    Serial.print(bkeep);
    Serial.println("cm");
    Serial.print("dkeep =");
    Serial.print(dkeep);
    Serial.println("cm");
    c = read_c();
    b = read_b();
    d = read_d();
  }
}
}

void loop() {
  // put your main code here, to run repeatedly:
  //In folkrace the kill switch will not be used.

  ///////////////////////////////////////////////////////////
  //Step 3: Set speed to medium high:
  //////////////////////////////////////////////////////////
  Serial.print("dflag:");
  Serial.println(dflag);
  analogWrite(3, v_corr); //Set a medium high speed.


  /////////////////////////////////////////////////////////////////////
  //Step 4: Small or large direction corrections and straight forward:
  /////////////////////////////////////////////////////////////////////

  if (dflag == 1) {//Scenario 4a. Keep d constant

    //TURN SHARPLY RIGHT IF-FOR myservo.write()
    if (((dkeep - d) > tol2) && (c > cmin) && (b > bmin)) {
      for (i = 105; i >= 85; i = i - 5) {
        myservo.write(i); //Turn the wheels sharply to the right.
        delay(1);
      }
      Serial.println("Scenario 4aRR. Turn sharply right!");
      while (((dkeep - d) > tol2) && (c > cmin) && (b > bmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("dkeep after loop:");
      Serial.print(dkeep);
      Serial.println("cm");
    }
    if (((dkeep - d) > tol) && (c > cmin) && (b > bmin)) {
      myservo.write(98); //Turn the wheels to the right.
      Serial.println("Scenario 4aR. Turn right!");
      while (((dkeep - d) > tol) && (c > cmin) && (b > bmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("dkeep after loop:");
      Serial.print(dkeep);
      Serial.println("cm");
    }
    
    //TURN SHARPLY LEFT IF-FOR myservo.write()
    if (((d - dkeep) > tol2) && (c > cmin) && (b > bmin)) {
      for (i = 115; i <= 135; i = i + 5) {
        myservo.write(i); //Turn the wheels to the left.
        delay(1);
      }
      Serial.println("Scenario 4aLL. Turn sharply left.");
      while (((d - dkeep) > tol2) && (c > cmin) && (b > bmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;} //Not good
      }
      Serial.print("dkeep after loop:");
      Serial.print(dkeep);
      Serial.println("cm");
    }
    if (((d - dkeep) > tol) && (c > cmin) && (b > bmin)) {
      myservo.write(120); //Turn the wheels to the left.
      Serial.println("Scenario 4aL. Turn left.");
      while (((d - dkeep) > tol) && (c > cmin) && (b > bmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;} //Not good
      }
      Serial.print("dkeep after loop:");
      Serial.print(dkeep);
      Serial.println("cm");
    }

    
    //STRAIGHT FORWARD 
    if (((d - dkeep) <= tol) && ((dkeep - d) <= tol) && (c > cmin) && (b > bmin)) {
      myservo.write(110); // Drive straight forward
      analogWrite(3, v_straight); //Set a high speed
      Serial.println("Scenario 4aS. Drive straight forward.");
      while (((d - dkeep) <= tol) && ((dkeep - d) <= tol) && (c > cmin) && (b > bmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("dkeep after loop:");
      Serial.print(dkeep);
      Serial.println("cm");
    }
  }
  else {//Scenario 4b. Keep a constant a
    if (((bkeep - b) > tol2) && (c > cmin) && (d > dmin)) {
      for (i = 115; i <= 135; i = i + 5) {
        myservo.write(i); //Turn the wheels to the left.
        delay(1);
      }
      Serial.println("Scenario 4bLL. Turn sharply left.");
      while (((bkeep - b) > tol2) && (c > cmin) && (d > dmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("bkeep after loop:");
      Serial.print(bkeep);
      Serial.println("cm");
    }
    if (((bkeep - b) > tol) && (c > cmin) && (d > dmin)) {
      myservo.write(120); //Turn the wheels to the left.
      Serial.println("Scenario 4bL. Turn left.");
      while (((bkeep - b) > tol) && (c > cmin) && (d > dmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("bkeep after loop:");
      Serial.print(bkeep);
      Serial.println("cm");
    }
    if (((b - bkeep) > tol2) && (c > cmin) && (d > dmin)) {
      for (i = 105; i >= 85; i = i - 5) {
        myservo.write(i); //Turn the wheels to the right.
        delay(1);
      }
      Serial.println("Scenario 4bRR. Turn sharply right.");
      while (((b - bkeep) > tol2) && (c > cmin) && (d > dmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("bkeep after loop:");
      Serial.print(bkeep);
      Serial.println("cm");
    }
    if (((b - bkeep) > tol) && (c > cmin) && (d > dmin)) {
      myservo.write(98); //Turn the wheels to the right.
      Serial.println("Scenario 4bR. Turn right.");
      while (((b - bkeep) > tol) && (c > cmin) && (d > dmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("bkeep after loop:");
      Serial.print(bkeep);
      Serial.println("cm");
    }
    if (((b - bkeep) <= tol) && ((bkeep - b) <= tol) && (c > cmin) && (d > dmin)) {
      myservo.write(110); // Drive straight forward
      analogWrite(3, v_straight); //Set a high speed
      Serial.println("Scenario 4bS. Drive straight forward.");
      while (((b - bkeep) <= tol) && ((bkeep - b) <= tol) && (c > cmin) && (d > dmin)) {
        cold = c;
        b = read_b();
        c = read_c();
        d = read_d();
        //if (cold-c > cdecmax){c = cold;}
      }
      Serial.print("bkeep after loop:");
      Serial.print(bkeep);
      Serial.println("cm");
    }
  }
  //Exit from step 4 above should only take place if c < cmin or b < bmin or d < dmin.

///////////////////////////////////////////////////////////////////////////////////////////////////
//Step 5: Slow down or stop temporarily to get time to measure distances:
//////////////////////////////////////////////////////////////////////////////////////////////////
analogWrite(3, v_turn); //Slow down in front of an obstacle.

///////////////////////////////////////////////////////////////////////////////////////////////////////
//Step 6: Avoid obstacles and turn in curves not already handled by step 4:
///////////////////////////////////////////////////////////////////////////////////////////////////////
if (d >= dmin && c <= cmin && d >= b) { //Scenario 6a
  for(i = 115; i <= 135; i = i + 5){
    myservo.write(i); //myservo.write(135); // Turn sharply left
    delay(1);
  }  
  Serial.println("Scenario 6a. Turn sharply left.");
  while (c <= cmin){
    cold = c;
    c = read_c();
    //if (cold-c > cdecmax){c = cold;}
  }
  for(i = 130; i >= 110; i = i - 5){
      myservo.write(i); //Set wheels straight forward.
      delay(1);
  }
  b = read_b();
  if (b > blarge){ //An obstacle was passed and was not a curve.
    Serial.println("The car has passed the obstacle which was not a curve.");
    d = read_d();
    dturn = d;
    //The obstacle was likely to be another robot.
    for(i = 105; i >= 85; i = i - 5){ 
      myservo.write(i); //myservo.write(85); // Turn sharply right to get DevCar parallel to a wall again.
      delay(1);
    }  
    while (b > bkeepmin && c > cmin && d < dturn*2){
      cold = c;
      b = read_b();
      c = read_c();
      d = read_d();
      //if (cold-c > cdecmax){c = cold;}  
    }
    for(i = 90; i <= 110; i = i + 5){
      myservo.write(i); //Set wheels straight forward again.
      delay(1);
    }
    dflag = 1;
    dkeep = min(dkeepmax,max(d,dkeepmin));
    Serial.print("dkeep:");
    Serial.print(dkeep);
    Serial.println("cm");
  }
  else{//DevCar passed through a curve to the left. Follow the right wall. 
    dflag = 0;
    bkeep = min(bkeepmax,max(b,bkeepmin));
    Serial.print("bkeep:");
    Serial.print(bkeep);
    Serial.println("cm");
    Serial.println("DevCar passed through a curve to the left.");
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
else if (b >= bmin && c <= cmin && b > d) { //Scenario 6b.
  for(i = 105; i >= 85; i = i - 5){
    myservo.write(i); //myservo.write(85); // Turn sharply right
    delay(1);
  }
  Serial.println("Scenario 6b. Turn sharply right.");
  while (c <= cmin){
    cold = c;
    c = read_c();
    //if (cold-c > cdecmax){c = cold;}
  }  
  for(i = 90; i <= 110; i = i + 5){
    myservo.write(i); //Set wheels straight forward.
    delay(1);
  }
  d = read_d();
  if (d > dlarge){//The car has passed the obstacle which was not a curve.
    Serial.println("The car has passed the obstacle which was not a curve.");
    b = read_b();
    bturn = b;
    for(i = 115; i <= 135; i = i + 5){
      myservo.write(i); //myservo.write(135); // Turn sharply left to get DevCar parallel to a wall again.
      delay(1);
    }  
    while (d > dkeepmin && c > cmin && b < bturn*2){
      cold = c;
      b = read_b();
      c = read_c();
      d = read_d();
      //if (cold-c > cdecmax){c = cold;}
    }
    for(i = 130; i >= 110; i = i - 5){
      myservo.write(i); //Set wheels straight forward again.
      delay(1);
    }
    dflag = 0;
    bkeep = min(bkeepmax,max(b,bkeepmin));
  }
  else {//DevCar has passed through a curve to the right.
    dflag = 1;
    dkeep = min(dkeepmax,max(d,dkeepmin)); //Follow the left wall.
    Serial.println("DevCar passed through a curve to the right.");
  }
}  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
else if (b <= bmin && c >= cmin && d > dmin){ //There is an obstacle to the right. Turn left somewhat.
  Serial.println("Scenario 6c. Turn somewhat left.");
  for(i = 115; i <= 125; i = i + 5){
      myservo.write(i); //Turn the wheels to the left.
      delay(1);
  }
  while(b <= bmin && c >= cmin && d >= dmin){
    cold = c;
    b = read_b();
    c = read_c();
    d = read_d();
    //if (cold-c > cdecmax){c = cold;}
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
else if (d <= dmin && c >= cmin && b > bmin){ //There is an obstacle to the left. Turn right somewhat.
  Serial.println("Scenario 6d. Turn somewhat right.");
  for(i = 105; i >= 95; i = i - 5){
      myservo.write(i); //Turn the wheels to the right.
      delay(1);
  }
  while(d <= dmin && c >= cmin && b >= bmin){
    cold = c;
    b = read_b();
    c = read_c();
    d = read_d();
    //if (cold-c > cdecmax){c = cold;}
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
else if (b <= bmin && d <= dmin) { //Scenario 6e is unlikely
  digitalWrite(9, HIGH);   //Stop the car. Engage the brake for the motor connected to Channel A
  Serial.println("Scenario 6e. Stop and wait.");
  while (b <= bmin && d <= dmin) {
    b = read_b();
    d = read_d();      
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
