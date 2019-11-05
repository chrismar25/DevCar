int LidarSteeringTesting(int lidarLeft, int lidarRight){
if (lidarLeft > lidarRight){
    return 0;
}
else return 180;
    
}

int steerIrLid (VL53L1X LidarLeft, VL53L1X LidarRight, int smoothingLid, int sensorAPin, int sensorBPin, int smoothingIr){

  // Sensor A is the Left Lidar Sensor
  // Sensor B is the right Lidar Sensor
  // When the car is on the middle, posIr=50. When the car is closer to the left wall is around 30 and
  // when is close to the right wall is around 60.
  
  //int angle = steeringWithLidarSensors()
  //int LidarLeft = sensorA.read();
  //int LidarRight = sensorB.read();
  int posIr = posWithIr(sensorAPin, sensorBPin, smoothingIr);
  int posLid = posWithLidar(LidarLeft,LidarRight,smoothingLid);
  //delay(100);
  Serial.println();

  /*
  Serial.print("IR pos: ");
  Serial.print(posIr);
  Serial.print(" Lidar pos: ");
  Serial.println(posLid);
  */
  
  return 1;
}

int setIsolatedVoltageFromSerial(){
  if(Serial.available() != 0){
    Serial.println("Give a value 0 < X < 255 to set isolatedVoltagelevel");
    String speedRequest = Serial.readString();
    while (Serial.available() == 0) {
      Serial.println("Waiting");
      delay(2000);
    }
    isolatedVoltage = Serial.readString().toInt();
  }
  return isolatedVoltage;
}
