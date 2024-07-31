#include <Wire.h>
#include "Arduino.h"

//by Soumyadeep Dutta
// MPU6050 Registers
const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float angleX, angleY, angleZ;
unsigned long prevTime = 0;
float elapsedTime, timeStep;
float angle = 0.0;
float accelAngleY, gyroAngleY;

// PID constants to be adjusted
float kp = 2.0;
float ki = 0.5;
float kd = 1.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;



// Motor Pins
const int motorA1 = 5;   //IN1
const int motorA2 = 18;  //IN2
const int motorB1 = 19;  //IN3
const int motorB2 = 23;  //IN4

//for enable pin
const int enable1 = 26;  //wheel 1
const int enable2 = 25;  //wheel 2 

const int freq = 30000;
const int resolution = 8;

float c = 100;

// PID function
float PID(float targetAngle, float currentAngle)
{
  float error = targetAngle - currentAngle;
  Serial.print("Error: ");
  Serial.print(error);
  integral += error * timeStep;
  derivative = (error - prevError) / timeStep;
  float output = (kp * error) + (ki * integral) + (kd * derivative);
  prevError = error;
  return output;
}

// Function to set motor speeds
void setMotorSpeeds(float motorSpeed) {
  //int motorSpeed = constrain(speed, 0, 255);
  if (motorSpeed > 0) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    /*analogWrite(motorA1, motorSpeed);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, motorSpeed);
    analogWrite(motorB2, 0);*/

    //adjust speed using ENABLE PINS
    //ledcWrite(enable1, motorSpeed);
    //ledcWrite(enable2, motorSpeed);
    analogWrite(enable1,motorSpeed);
    analogWrite(enable2,motorSpeed);

    //control direction : FRONT
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    
  } 
  else if(motorSpeed == 0)
  { //motor stops
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
  else 
  {
    digitalWrite(LED_BUILTIN, LOW);
    /*analogWrite(motorA1, 0);
    analogWrite(motorA2, -motorSpeed);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, -motorSpeed);*/

    //adjust speed using ENABLE PINS
    //ledcWrite(enable1, (motorSpeed*(-1)));
    //ledcWrite(enable2, (motorSpeed*(-1)));
    analogWrite(enable1,(motorSpeed*(-1)));
    analogWrite(enable2,(motorSpeed*(-1)));

    //control direction : BACK
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // MPU6050 initialization
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //setup enable pins
  pinMode(enable1,OUTPUT);
  pinMode(enable2,OUTPUT);
  //ledcAttachPin(enable1, freq, resolution, enable1);
  //ledcAttachPin(enable2, freq, resolution, enable2);
 // pinMode(enable1,OUTPUT);
 // pinMode(enable2,OUTPUT);
}

void loop() {
  //  MPU6050 data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  gyroX  = Wire.read() << 8 | Wire.read();
  gyroY  = Wire.read() << 8 | Wire.read();
  gyroZ  = Wire.read() << 8 | Wire.read();

  // Calculate angles
  accelAngleY = atan2(accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
  gyroAngleY = gyroY / 131.0;
  
  // Complementary filter
  unsigned long currTime = millis();
  elapsedTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;
  
  angleY = 0.98 * (angleY + gyroAngleY * elapsedTime) + 0.02 * accelAngleY;

  // PID control
  float pidOutput = PID(0, angleY);
  //c = (c>0)?(c*-1):(c*-1);
  setMotorSpeeds(pidOutput);
  digitalWrite(LED_BUILTIN, LOW);

  //setMotorSpeeds(c);
  //delay(5000);
  // Debugging
  Serial.print("Angle: ");
  Serial.print(angleY);
  Serial.print(" PID Output: ");
  Serial.println(pidOutput);

  delay(10);
  
}

