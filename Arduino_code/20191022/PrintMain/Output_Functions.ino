void setSteeringServo (Servo myservo, int angle){
<<<<<<< HEAD
  int steer = map(angle, 0, 180, 50, 120);
  Serial.print("Steeringangle: ");
  Serial.println(steer);
=======
  int steer = map(angle, 0, 180, 20, 160);
  //Serial.print("Steeringangle: ");
  //Serial.println(steer);
>>>>>>> Development
  myservo.write(steer);
}

void setIsolatedVoltageLevel(int spd){
  analogWrite(isolatedVoltageLevel, spd);
}
