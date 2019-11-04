#include <Wire.h>
#include <VL53L1X.h>


//Define pins and variables for the two LIDAR sensors
VL53L1X lidarSensorA;
VL53L1X lidarSensorB;

int activateLidarA = 2;
int activateLidarB = 3;

const unsigned int numReadings = 100;
unsigned int analogVals[numReadings];
unsigned int i = 0;

int AddressLidarSensorA = 10;
int AddressLidarSensorB = 11;

//Define pins and variables for the two IR sensors
#define IrSensorA A0
#define IrSensorB A1

int avgIrA = 0;
int avgIrB = 0;
float pos = 0;
int filter = 3;

//Define pins and variables for the Ultrasonic sensor
#define UsTrigPin 4
#define UsEchoPin 5

int UsDistance = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  //Setup Lidar sensors
  lidarSensorA = setupSensor(lidarSensorA, AddressLidarSensorA, activateLidarA);
  lidarSensorB = setupSensor(lidarSensorB, AddressLidarSensorB, activateLidarB);

  //Setup pins for Ir sensors
  pinMode(IrSensorA, INPUT);
  pinMode(IrSensorB, INPUT);

  //Setup pins for Ultrasonic sensor
  pinMode(UsTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(UsEchoPin, INPUT); // Sets the echoPin as an Input
}

void loop() {
  //Lidar
  //printFromLidarSensors(lidarSensorA, lidarSensorB);
  plotFromLidarSensors(lidarSensorA, lidarSensorB);
  
  //Infra-red
  //printFromIrSensors(IrSensorA, IrSensorB);
  //plotFromIrSensors(IrSensorA, IrSensorB);
  plotPositionOnTrack(IrSensorA, IrSensorB, filter);
  
  //Ultrasonic
  //printFromUltrasonicSensor(UsTrigPin, UsEchoPin);
  plotFromUltrasonicSensor(UsTrigPin, UsEchoPin);

  Serial.println();
  delay(100);
}
