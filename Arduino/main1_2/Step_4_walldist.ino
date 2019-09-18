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
