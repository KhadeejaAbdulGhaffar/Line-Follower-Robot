//libraries
#include <QTRSensors.h>

//motor driver variables
int aphase = 9;
int aenlb = 6;
int bphase = 5;
int benlb = 3;
int mode = 8;

//PID controllers
float Kp = 0.07;
float Ki = 0.0006;
float Kd = 0.7;

//calibration
int button_calibration = A3;
int button_start = 2;

//QTRsensor
QTRSensors qtr;
const uint8_t SensorCount = 8; //8 is the number we have on our sensor array
uint16_t sensorValues[SensorCount]; //an array to store values of sensor each time we call a read function

//PID control
int lastError = 0;

void setup() {
  //setting up pins
  //qtr sensor array
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, A0, A1, A2, A4, A5}, SensorCount);
  qtr.setEmitterPin(7);

  //motor drivers
  pinMode(aphase, OUTPUT);
  pinMode(aenlb, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benlb, OUTPUT);
  pinMode(mode, OUTPUT);

  //calibration button
  pinMode(button_calibration, INPUT);
  pinMode(button_start, INPUT);

  //mode will always be high
  digitalWrite(mode, HIGH);

  //user press calibration button and the robot calibrates
  while(digitalRead(button_calibration == LOW){}

  //sensor array to have reliable and accurate reading of the sensor array
  //10 seconds
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  //progress tuck to wait for user to restart
  while(digitalRead(button_start == LOW){}
}

//when we press start button of robot this executes:
void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
}

void PID_control(){
  //if robot goes left or right of the track, these controllers interact with the motors to put it on the right track
  uint16_t positionLine = qtr.readLineBlack(sensorValues); //change array with latest readings of each sensor
  //calculation for position of line
  int error = 3500 - positionLine;
  
  int P = error;
  int I = error + I;
  int D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kp;

  //connection between pid controls and the motors
  //speed robot wants to take when it goes straight forward 
  //when there is an error motorSpeedA increases and motorSpeedB decreases so that robot can take a curve
  int motorSpeedA = 75 + motorSpeedChange; 
  int motorSpeedB = 75 - motorSpeedChange; 

  Serial.print(motorSpeedChange); //print values

  if(motorSpeedA > 125){
    motorSpeedA = 125;
  }
  if(motorSpeedB > 125){
    motorSpeedB = 125;
  }
  if(motorSpeedA < -75){
    motorSpeedA = -75;
  }
  if(motorSpeedB < -75){
    motorSpeedB = -75;
  }

  forward_movement(motorSpeedA, motorSpeedB)
}

void forward_movement(int speedA, int speedB){
  if (speedA < 0){
    digitalWrite(aphase, HIGH);
    speedA = 0 - speedA;
  }
  else{
    digitalWrite(aphase, LOW);
  }
  if (speedB < 0){
    digitalWrite(bphase, HIGH);
    speedB = 0 - speedB;
  }
  else{
    digitalWrite(bphase, LOW);
  }
  analogWrite(aenlb, SpeedA);
  analogWrite(benlb, SpeedB);
}
