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
