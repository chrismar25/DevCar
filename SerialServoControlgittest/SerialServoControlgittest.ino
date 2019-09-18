#include <Servo.h>
Servo myservo;  // create servo object to control a servo
String angleRequest;
int servoPos = 90;
bool newInput = false;
int mode = 0;
int limit = 0;


// Changeing something in the code

// further changes

// more changes

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial communication established!!!");
  myservo.attach(9);
  myservo.write(90);
  Serial.println("Do you wish to constrain input within predetermined values? 0: No, 1: Yes");
  waitForInput();
  if(Serial.readString().toInt() == 1){
    limit = 1;
    Serial.println("Outputs will be constrained within a predetermined range!");
  }  
}

void loop() {
  Serial.println("Mode of operation, 1 --> Set angle, 2 --> Sweep between angles");
  waitForInput();
  mode = Serial.readString().toInt();
  Serial.print("Selected mode is: ");
  Serial.println(mode);
  if (mode == 1) {
    setAngle();
  }
  else if (mode == 2) {
    sweep();
  }
}

void setAngle() {
  Serial.println("Enter requested angle in degrees, 0 <= X <= 180");
  waitForInput();
  angleRequest = Serial.readString();
  newInput = true;
  
  if (newInput) { //New input
    Serial.println(angleRequest.toInt());
    if (0 <= angleRequest.toInt() && angleRequest.toInt() <= 180) { //If new requested angle is within allowed range
      servoPos = angleRequest.toInt();    //Enterpret requested angle as int
      Serial.print("New requested angle is: ");
      Serial.println(servoPos);
    }
    else {
      Serial.println("Requested angle is outside range, 0 <= X <= 180");
    }
    setServo(servoPos);
    angleRequest = "";
    newInput = false;
  }

}

void sweep() {
  int lowerLimit, upperLimit, delayTime, pos = 0;
  
  Serial.print("Input the lower angle limit: ");
  waitForInput();
  lowerLimit = Serial.readString().toInt();
  Serial.println(lowerLimit);
  
  Serial.print("Input the upper angle limit: ");
  waitForInput();
  upperLimit = Serial.readString().toInt();
  Serial.println(upperLimit);

  Serial.print("Input the delay for each angle update: ");
  waitForInput();
  delayTime = Serial.readString().toInt();
  Serial.println(delayTime);

  while(!Serial.available()){ // continue sweeping until next serial input
    for (pos = lowerLimit; pos <= upperLimit; pos += 1) {
      // in steps of 1 degree
      setServo(pos);
      delay(delayTime);                
    }
    for (pos = upperLimit; pos >= lowerLimit; pos -= 1) { 
      setServo(pos);              
      delay(delayTime);       
    }
  }
}

void waitForInput(){
  while (Serial.available() == 0) {
//    Serial.println("waiting");
    delay(2);
  } // Wait for serial input
}

void setServo(int angle){
  if(limit == 1){
    //Serial.print("Mapped value: "); Serial.print(angle);
    //Serial.print(" Output: "); Serial.println(map(angle, 0, 180, 60, 107));
    myservo.write(map(angle, 0, 180, 57, 119));
  }
  else{
    Serial.print("Angle is: "); Serial.println(angle);
    myservo.write(angle);
  }
}
