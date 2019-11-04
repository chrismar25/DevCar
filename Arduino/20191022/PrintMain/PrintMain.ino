

#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

//Define pins and controlling variables for controlling isolated powersupply
const int isolatedVoltageLevel = 11;
const int isolatedVoltagePolarity = 13;
const int isolatedVoltageActivation = 8;
const int isolatedVoltageCurrent = A1;
int isolatedVoltage = 0;

//Define pins and Variables for the servo steering motor
Servo myservo;  // create servo object to control a servo
int servoAng = 90;
#define servoPin 6

///////////////////////////////////////////////////////////////////
//Define pins and variables for the two LIDAR sensors
VL53L1X lidarSensorA;
VL53L1X lidarSensorB;

int activateLidarA = 7;
int activateLidarB = 10;

float avgLidarA = 0;
float avgLidarB = 0;
float posLid = 0;
int filterLid = 1;
const unsigned int numReadings = 100;
unsigned int analogVals[numReadings];
unsigned int i = 0;

int AddressLidarSensorA = 10;
int AddressLidarSensorB = 11;

/////////////////////////////////////////////////////////////////
//Define pins and variables for the two IR sensors
#define IrSensorA A2
#define IrSensorB A3
#define voltageWatch A1

int avgIrA = 0;
int avgIrB = 0;
//int posIr = 0;
int filterIr = 2;

////////////////////////////////////////////////////////////////
//Define pins and variables for the Ultrasonic sensor
#define UsTrigPin 4
#define UsEchoPin 5

int UsDistance = 0;

////////////////////////////////////////////////////////////////
//                      Void Setup
////////////////////////////////////////////////////////////////  
void setup() {
  Serial.begin(4800);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  //Setup Lidar sensors
  lidarSensorA = setupSensor(lidarSensorA, AddressLidarSensorA, activateLidarA);
  lidarSensorB = setupSensor(lidarSensorB, AddressLidarSensorB, activateLidarB);

  //Setup pins for Ir sensors
  pinMode(IrSensorA, INPUT);
  pinMode(IrSensorB, INPUT);
  pinMode(voltageWatch, INPUT);


  //Setup pins for Ultrasonic sensor
  pinMode(UsTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(UsEchoPin, INPUT); // Sets the echoPin as an Input

  //Setup pins for Servo motor
  myservo.attach(servoPin);
  myservo.write(90);

  //Setup pins for the isolated voltage/powersupply
  pinMode(isolatedVoltageLevel, OUTPUT);
  pinMode(isolatedVoltagePolarity, OUTPUT);
  pinMode(isolatedVoltageActivation, OUTPUT);
  pinMode(isolatedVoltageCurrent, INPUT);
  
  digitalWrite(isolatedVoltagePolarity, HIGH);
  while (!Serial);
  Serial.println("PinModes set & serial communication established!!!");
 
}
///////////////////////////////////////////////////////////////////
//                      Void Loop
///////////////////////////////////////////////////////////////////


void loop() {
  //Isolated voltage testing
 /* int levelIV = setIsolatedVoltageFromSerial();
  setIsolatedVoltageLevel(levelIV);
  Serial.print("Isolated voltage variable is currently: ");
  Serial.println(isolatedVoltage);
  
  
  //Steering with a combination of Ir and Lidar testing
  //int something = steerIrLid (lidarSensorA, lidarSensorB, filterLid, IrSensorA, IrSensorB, filterIr);

*/  
  
  //Steering with lidar testing
  int steerAngle = steeringWithLidarSensors(lidarSensorA, lidarSensorB, filterLid);
  int posIR =  positionOnTrack(IrSensorA, IrSensorB, filterIr);

  //Steering with Ir sensor testing
    int steerIR = steeringWithIr(IrSensorA, IrSensorB, filterIr);
// setSteeringServo(myservo, 90);
  
  //Lidar
  //printFromLidarSensors(lidarSensorA, lidarSensorB);
  //plotFromLidarSensors(lidarSensorA, lidarSensorB);
  
  //Infra-Red
  //printFromIrSensors(IrSensorA, IrSensorB);
  //plotFromIrSensors(IrSensorA, IrSensorB);
  //plotPositionOnTrack(IrSensorA, IrSensorB, filterIr);
  
  //Ultrasonic
  //printFromUltrasonicSensor(UsTrigPin, UsEchoPin);
  //plotFromUltrasonicSensor(UsTrigPin, UsEchoPin);

  Serial.println("");
  //From Lidar to Steering Servo
 int distLeft = lidarSensorA.read();
  int distRight = lidarSensorB.read();
  int servoAng = LidarSteeringTesting(distLeft, distRight);
  Serial.println(servoAng);
  setSteeringServo(myservo, servoAng);
  
}
