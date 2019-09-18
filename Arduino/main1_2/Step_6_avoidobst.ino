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
