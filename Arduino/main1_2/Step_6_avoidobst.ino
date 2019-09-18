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
