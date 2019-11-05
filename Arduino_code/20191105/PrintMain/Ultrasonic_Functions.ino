void printFromUltrasonicSensor(int trigPin, int echoPin){
  // Prints the distance on the Serial Monitor
  Serial.print("Ultrasonic distance: ");
  Serial.println(getUltrasonicDistance(trigPin, echoPin));
}

void plotFromUltrasonicSensor(int trigPin, int echoPin){
  // Prints the distance on the Serial Monitor
  Serial.print(constrain((getUltrasonicDistance(trigPin, echoPin)),2,400));
  Serial.print(",");
}

int getUltrasonicDistance(int trigPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = (int)duration*0.035/2;
  return distance;
}
