void printFromIrSensors(int IrSensorA, int IrSensorB){
  Serial.print("IR sensor A: ");
  Serial.print(readShortRangeIrSensor(IrSensorA));
  Serial.print("  IR sensor B: ");
  Serial.print(readShortRangeIrSensor(IrSensorB));
  Serial.println();
  //Serial.println("Relative position:");
  //Serial.println(positionOnTrack(IrSensorA, IrSensorB, filterIr));
  delay(500);
}

void plotFromIrSensors(int IrSensorA, int IrSensorB){
  Serial.print(readShortRangeIrSensor(IrSensorA));
  Serial.print(",");
  Serial.print(readShortRangeIrSensor(IrSensorB));
  Serial.print(",");
}

int positionOnTrack(int sensorAPin, int sensorBPin, int smoothing){
  int ValueSensorA = readSensorVoltage(sensorAPin);
  avgIrA = (avgIrA * (smoothing - 1) + ValueSensorA) / smoothing;
  //Serial.print("Value A: ");
  //Serial.print(avgIrA);
  int ValueSensorB = readSensorVoltage(sensorBPin);
  avgIrB = (avgIrB * (smoothing - 1) + ValueSensorB) / smoothing;
  //Serial.print(" Values B: ");
  //Serial.println(avgIrB);
  float exactPosOnTrack = ValueSensorA / (ValueSensorA + ValueSensorB);
  int posOnTrack = (int) exactPosOnTrack;
  return posOnTrack;
}

void plotPositionOnTrack(int sensorAPin, int sensorBPin, int smoothing){
  int posIr = 0;
  int ValueSensorA = readSensorVoltage(sensorAPin);
  avgIrA = (avgIrA * (smoothing - 1) + ValueSensorA) / smoothing;
  //Serial.print(avgIrA);
  //Serial.print(",");
  
  int ValueSensorB = readSensorVoltage(sensorBPin);
  avgIrB = (avgIrB * (smoothing - 1) + ValueSensorB) / smoothing;
  //Serial.print(avgIrB);
  //Serial.print(",");

  float sum = avgIrA + avgIrB;
  float exactPosOnTrack = (float)(avgIrA /sum )*100;
  posIr = exactPosOnTrack;
  int posOnTrack = (int) exactPosOnTrack;
  Serial.print(posIr);
  //Serial.print(",");
}

int steeringWithIr(int sensorAPin, int sensorBPin, int smoothing){
  int posIr = 0;
  int ValueSensorA = readSensorVoltage(sensorAPin);
  avgIrA = (avgIrA * (smoothing - 1) + ValueSensorA) / smoothing;
  
  int ValueSensorB = readSensorVoltage(sensorBPin);
  avgIrB = (avgIrB * (smoothing - 1) + ValueSensorB) / smoothing;

  float sum = avgIrA + avgIrB;
  float exactPosOnTrack = (float)(avgIrA /sum )*100;
  posIr = exactPosOnTrack;
  //Serial.print("posIr: ");
  //Serial.print(posIr);
  
  Serial.println(posIr);
  Serial.print(",");
  int strAngle = map(posIr, 20, 80, 180, 0);
  return strAngle;
  
}

int posWithIr(int sensorAPin, int sensorBPin, int smoothing){
  int ValueSensorA = readSensorVoltage(sensorAPin);
  int posIr = 0;
  avgIrA = (avgIrA * (smoothing - 1) + ValueSensorA) / smoothing;
  
  int ValueSensorB = readSensorVoltage(sensorBPin);
  avgIrB = (avgIrB * (smoothing - 1) + ValueSensorB) / smoothing;

  float sum = avgIrA + avgIrB;
  float exactPosOnTrack = (float)(avgIrA /sum )*100;
  posIr = exactPosOnTrack;
  //Serial.print("posIr: ");
  //Serial.print(posIr);
   return posIr;
}

int readShortRangeIrSensor(int sensorPin){
  float volts = analogRead(sensorPin)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  return distance;
}

int readSensorVoltage(int sensorPin){
  float volts = analogRead(sensorPin);
  int powerSupply = analogRead(voltageWatch);
  /*
  Serial.print("IR volt: ");
  Serial.print(volts);
  Serial.print(" Input volt: ");
  Serial.println(powerSupply);
  */
  int value = (int) volts;
  //value = map(value, 0, powerSupply, 0, 1023);

  //Serial.println("");
  //Serial.print("IR volt: ");
  //Serial.print(value);
  //Serial.print(",");
  //Serial.print(" Input volt: ");
  //Serial.println(powerSupply);
  
  return value;
}
