void printFromLidarSensors(VL53L1X sensorA, VL53L1X sensorB){
  //saves sensor value for sensor A and B
  int A = sensorA.read();
  int B = sensorB.read();

  //prints sensor value for sensor A and B
  Serial.print("Sensor A:  ");
  Serial.print(A); 
  Serial.print(",   Sensor B:  ");
  Serial.print(B);
  Serial.println();
  
  if (sensorA.timeoutOccurred() or sensorB.timeoutOccurred())
  { 
    Serial.print("TIMEOUT ");
    Serial.println();
  }

  
  delay(100); //How often we want to read and print sensor meassurements in ms
}

void plotFromLidarSensors(VL53L1X sensorA, VL53L1X sensorB){
  //saves sensor value for sensor A and B
  int A = sensorA.read();
  int B = sensorB.read();

  //prints sensor value for sensor A and B
  Serial.print(A); 
  Serial.print(",");
  Serial.print(B);
  Serial.print(",");
}

VL53L1X setupSensor(VL53L1X sensor, int address, int pin){

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  
  pinMode(pin, INPUT);
  
  sensor.setTimeout(500);

  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  //Sets and prints a new address for the sensor
  sensor.setAddress(address);
  Serial.print("Sensor:  ");
  Serial.println(sensor.getAddress());

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);

  return sensor;
}
