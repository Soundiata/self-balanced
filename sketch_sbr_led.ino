/*
 *  
 *  Manage the self-balanced Robot
 *   - Define PIN layout
 *   - Attach & Control 6-DOFIMU via I2C
 *   - Drive motors accordingly to PID control
 *  
 *  TODO:
 *   - Register manipulation for PWM (increase reactivity of motor drive)
 *   - NeoPixel management
 *   - State machine implementation
 *   - PID tuning
 *   - trace events in serial
 *   - inbound communication 
 *  
 *  Created on 8 August 2018
 *  By Mamadi KEITA
 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include <PID_v1.h>


//Rotation direction of the motor
#define CW 0
#define CCW 1
//Movements of MPU6050
#define YAW 0
#define PITCH 1
#define ROLL 2

//Motor Definition
#define MOTOR_A 0
#define MOTOR_B 1
 
 //PIN Setting
 const byte BUTTON_PIN = 4;
 const byte NEOPIXEL_LED_PIN = 6;
 const byte MOTORA_SPEED_PIN = 3;
 const byte MOTORB_SPPED_PIN = 11;
 const byte MOTORA_DIR_PIN = 12;
 const byte MOTORB_DIR_PIN = 13;

//Instantiate MPU6050 and variables
MPU6050 mpu;
// --
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
// --
Quaternion q;         // [w, x, y, z]       quaternion container
VectorFloat gravity;  // [x, y, z]          gravity vector
float ypr[3];         // [Yaw, Pitch, Roll] yaw/pitch/roll container & gravity vector

volatile bool mpuInterrupt = false; 
// -- MPU Calibration
const int OFFSET_ACCEL_X = 0;
const int OFFSET_ACCEL_Y = 0;
const int OFFSET_ACCEL_Z = 0;
const int OFFSET_GYRO_X = 0;
const int OFFSET_GYRO_Y = 0;
const int OFFSET_GYRO_Z = 0;
//--------------------------------

//Instanciate Neopixel
Adafruit_NeoPixel strip = Adafruit_NeoPixel (3, NEOPIXEL_LED_PIN, NEO_GRB + NEO_KHZ800);
//--------------------------------

//Instanciate PID
const double Kp = 2;
const double Ki = 5;
const double Kd = 1;
double setPointPID, inputPID, outputPID;

PID motorPID(&inputPID, &outputPID, &setPointPID, Kp, Ki, Kd, DIRECT);
//--------------------------------


// Interrup detection routine
void dmpDataReady() {
  mpuInterrupt = true;
}



void setup() {
  //Open Serial Connection for logging
  Wire.begin();
  Serial.begin(57600);
  
  //Setup components
  setupMotor();
  setupNeoPixel();
  setupIMU();
  
  inputPID = 0 ;
  setPointPID = 100;
  motorPID.SetMode(AUTOMATIC);
  
}

void loop() {
  processIMU();
  inputPID = ypr[PITCH];
  motorPID.Compute();
  regulatedDrive();
  
}

void setupMotor() {
  Serial.print("Initialize Motors ...");
  pinMode(MOTORA_SPEED_PIN, OUTPUT);
  pinMode(MOTORB_SPPED_PIN, OUTPUT);
  pinMode(MOTORA_DIR_PIN, OUTPUT);
  pinMode(MOTORB_DIR_PIN, OUTPUT);

  digitalWrite(MOTORA_SPEED_PIN, LOW);
  digitalWrite(MOTORB_SPPED_PIN, LOW);
  digitalWrite(MOTORA_DIR_PIN, LOW);
  digitalWrite(MOTORB_DIR_PIN, LOW);
  
  Serial.print("Motors initialized.");
}

void setupNeoPixel() {
  Serial.print("Initialize NeoPixel Strip ...");
  strip.begin();
  strip.show();

  Serial.print("NeoPixel initialized.");
}

void setupIMU() {
  TWBR = 24; //400kHz I2C clock for Arduino Uno
  
  Serial.print("Initialize MPU ...");
  mpu.initialize();

  Serial.print("Initialize DMP ...");
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(OFFSET_ACCEL_X);
  mpu.setYAccelOffset(OFFSET_ACCEL_Y);
  mpu.setZAccelOffset(OFFSET_ACCEL_Z);
  mpu.setXGyroOffset(OFFSET_GYRO_X);
  mpu.setYGyroOffset(OFFSET_GYRO_Y);
  mpu.setZGyroOffset(OFFSET_GYRO_Z);

  if (devStatus == 0) {
    Serial.print("DMP initialization success !");
    //Turn on the DMP
    mpu.setDMPEnabled(true);

    //Enable interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    //Set DMP Ready flag
    dmpReady = true;

    //Get expected packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  } else {
    //Initialization failed
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.print(")");
  }
}


void processIMU() {
  if (!dmpReady) {
    return;
  }

  while (!mpuInterrupt && fifoCount < packetSize) {
    //Wait for MPU interrupt or extra packet available
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow");
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -=packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void activateMotor(byte motr, byte dir, byte spd) {
  byte motorDirection = 0;
  byte motorSpeed = 0;

  if (motr == MOTOR_A)
  {
    motorDirection = MOTORA_DIR_PIN;
    motorSpeed = MOTORA_SPEED_PIN;
  }
  else
  {
    motorDirection = MOTORB_DIR_PIN;
    motorSpeed = MOTORB_DIR_PIN;
  }

  digitalWrite(motorDirection, dir);
  analogWrite(motorSpeed, spd);
}

void regulatedDrive() {
  activateMotor(MOTOR_A, CW, outputPID);
  activateMotor(MOTOR_B, CCW, outputPID);
}

