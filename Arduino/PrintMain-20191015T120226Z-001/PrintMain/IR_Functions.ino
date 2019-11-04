void printFromIrSensors(int IrSensorA, int IrSensorB){
  Serial.print("IR sensor A: ");
  Serial.print(readShortRangeIrSensor(IrSensorA));
  Serial.print("  IR sensor B: ");
//  Serial.print(readLongRangeIrSensor(IrSensorB));
  Serial.println();
  delay(500);
}

void plotFromIrSensors(int IrSensorA, int IrSensorB){
  Serial.print(readShortRangeIrSensor(IrSensorA));
  Serial.print(",");
}

int positionOnTrack(int sensorAPin, int sensorBPin, int smoothing){
  int ValueSensorA = readSensorVoltage(sensorAPin);
  avgIrA = (avgIrA * (smoothing - 1) + ValueSensorA) / smoothing;
  Serial.print("Value A: ");
  Serial.print(avgIrA);
  int ValueSensorB = readSensorVoltage(sensorBPin);
  avgIrB = (avgIrB * (smoothing - 1) + ValueSensorB) / smoothing;
  Serial.print(" Values B: ");
  Serial.println(avgIrB);
  float exactPosOnTrack = ValueSensorA / (ValueSensorA + ValueSensorB);
  int posOnTrack = (int) exactPosOnTrack;
  return posOnTrack;
}

void plotPositionOnTrack(int sensorAPin, int sensorBPin, int smoothing){
  int ValueSensorA = readSensorVoltage(sensorAPin);
  avgIrA = (avgIrA * (smoothing - 1) + ValueSensorA) / smoothing;
//  Serial.print("Value A: ");
//  Serial.print(avgIrA);
  int ValueSensorB = readSensorVoltage(sensorBPin);
  avgIrB = (avgIrB * (smoothing - 1) + ValueSensorB) / smoothing;
//  Serial.print(",");
//  Serial.print(avgIrB);
//  float sum = ValueSensorA + ValueSensorB;
//  float exactPosOnTrack = (float)(ValueSensorA /sum )*100;
  float sum = avgIrA + avgIrB;
  float exactPosOnTrack = (float)(avgIrA /sum )*100;
  pos = (pos * (smoothing - 1) + exactPosOnTrack) / smoothing;
  int posOnTrack = (int) exactPosOnTrack;
  Serial.print(pos);
  Serial.print(",");
}

int readShortRangeIrSensor(int sensorPin){
  float volts = analogRead(sensorPin)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  return distance;
}

int readSensorVoltage(int sensorPin){
  float volts = analogRead(sensorPin);
  int value = (int) volts;
  return value;
}
