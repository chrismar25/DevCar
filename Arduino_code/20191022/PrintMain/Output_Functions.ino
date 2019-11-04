void setSteeringServo (Servo myservo, int angle){
  int steer = map(angle, 0, 180, 50, 120);
  Serial.print("Steeringangle: ");
  Serial.println(steer);
  myservo.write(steer);
}

void setIsolatedVoltageLevel(int spd){
  analogWrite(isolatedVoltageLevel, spd);
}
