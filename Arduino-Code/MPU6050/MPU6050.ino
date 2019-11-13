
/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

  This software may be distributed and modified under the terms of the GNU
  General Public License version 2 (GPL2) as published by the Free Software
  Foundation and appearing in the file GPL2.TXT included in the packaging of
  this file. Please note that GPL2 Section 2[b] requires that all works based
  on this software must also be made publicly available under the terms of
  the GPL2 ("Copyleft").

  Contact information
  -------------------

  Kristian Lauszus, TKJ Electronics
  Web      :  http://www.tkjelectronics.com
  e-mail   :  kristianl@tkjelectronics.com
*/

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kinematics.h"
//#include "NR.h"
#include <MatrixMath.h>
#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 20, 4);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define pi 3.14
Kalman kalmanX1, kalmanX2, kalmanX3, kalmanX4, kalmanX5; // Create the Kalman instances
Kalman kalmanY1, kalmanY2, kalmanY3, kalmanY4, kalmanY5;
Kinematics k1,k2,k3;

/* IMU Data */
double ROLL[6], PITCH[6], R[3][3], Z[3], Z1[3], loc_resultant1[3][1], loc_resultant2[3][1], loc_resultant3[3][1], loc_resultant4[3][1];
//double KALMANX[2],KALMANY[2];
double accX, accY, accZ;
double RotIMU[3][3];
double accXarr[6], accYarr[6], accZarr[6], gyroXarr[6], gyroYarr[6], gyroZarr[6];
double dt,dt2;

double GLOB_resultant1[3][1], GLOB_resultant2[3][1], GLOB_resultant3[3][1], GLOB_resultant4[3][1], Final_Glob1[3][1], Final_Glob2[3][1], Final_Glob3[3][1], Final_Glob4[3][1], R_BP[3][3];
double gyroX, gyroY, gyroZ;
double gyroXrate1, gyroXrate2, gyroXrate3, gyroXrate4, gyroXrate5;
double gyroYrate1, gyroYrate2, gyroYrate3, gyroYrate4, gyroYrate5;
int16_t tempRaw;
double roll1, pitch1, roll2, pitch2, roll3, pitch3, roll4, pitch4, roll5, pitch5 ;
double gyroXangle1, gyroYangle1, gyroXangle2, gyroYangle2, gyroXangle3, gyroYangle3, gyroXangle4, gyroYangle4, gyroXangle5, gyroYangle5; // Angle calculate using the gyro only
double compAngleX1, compAngleY1, compAngleX2, compAngleY2, compAngleX3, compAngleY3, compAngleX4, compAngleY4, compAngleX5, compAngleY5; // Calculated angle using a complementary filter
double kalAngleX1, kalAngleY1, kalAngleX2, kalAngleY2, kalAngleX3, kalAngleY3, kalAngleX4, kalAngleY4, kalAngleX5, kalAngleY5; // Calculated angle using a Kalman filter
double RR[3][3];
uint32_t timer,timer1;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
int addressA = 33; //S0
int addressB = 35; //S1
int addressC = 37; //S2
int addressD = 39; //S3
int inByte = 0;
const int MPU = 0x68;

int Pin1 = 13;      // Analog Input Pin 0
int Pin2 = 12;      // Analog Input Pin 1
int Pin3 = 14;      // Analog Input Pin 2

//Switches
const int switch_pin1 = 49;
const int switch_pin2 = 51;
const int switch_pin3 = 53;
int switch_1 = 0;
int switch_2 = 0;
int switch_3 = 0;
int scenario = 0;

// Koordinaten
double x_Old = 0;
double y_Old = 0;
double dx;
double dy;
double Length_Actual_1;
double Length_Actual_2;
double Length_Actual_3;
double Delta_Length_1;
double Delta_Length_2;
double Delta_Length_3;
double delta1 = 0;
double delta2 = 0;
double delta3 = 0;
double sum1 = 0;
double sum2 = 0, sum3 = 0;
double u_1 = 127;
double u_2 = 127;
double u_3 = 127;
double K1 = 255 / 29.7043;
//double K1=255/28;
double K2 = 0.66 * 0.1071;
//double K3=0.167*0.1071;
double K3 = 0.33 * 0.1071;
int pause = 0;
double Switch_Volt = 0;
int Szenario = 0;
int sign = 1;
int Step = 0;
double xTar;
double zTar;
double phiTar;
double deadLength_1 = 168;
double deadLength_2 = 168;
double deadLength_3 = 168;
int timer_wait = 0;
 int count = 0;
 int dyn_pose = 0;
// DC motor on M1 an M2
AF_DCMotor motor1(4);
AF_DCMotor motor2(1);
AF_DCMotor motor3(3);


void setup() {

  Wire.begin(); // wake up I2C bus
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)+
  Wire.endTransmission(true);
  Serial.begin(115200);
  pinMode(addressA, OUTPUT);  //S0
  pinMode(addressB, OUTPUT);  //S1
  pinMode(addressC, OUTPUT);  //S2
  pinMode(addressD, OUTPUT);  //S4

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  dt2 = (double)(micros() - timer1) / 1000000;

  delay(100); // Wait for sensor to stabilize

  digitalWrite(addressA, LOW);
  digitalWrite(addressB, LOW);
  digitalWrite(addressC, LOW);
  digitalWrite(addressD, LOW);
  read_imus(1);
  //Serial.print("\t");

  //       Second IMU: Check the table for channel 1 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, HIGH);
  digitalWrite(addressC, LOW);
  digitalWrite(addressD, LOW);
  read_imus(2);
  //Serial.print("\t");
  
  //        Third IMU: Check the table for channel 2 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, LOW);
  digitalWrite(addressC, HIGH);
  digitalWrite(addressD, LOW);
  read_imus(3);
  //Serial.print("\t");

  // Forth IMU: Check the table for channel 3 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, HIGH);
  digitalWrite(addressC, HIGH);
  digitalWrite(addressD, LOW);
  read_imus(4);
  //Serial.print("\t");

  //IMU1
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll1  = atan2(accYarr[1], accZarr[1]) * RAD_TO_DEG + 4.322;
  pitch1 = atan(-accXarr[1] / sqrt(accYarr[1] * accYarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll1  = atan(accYarr[1] / sqrt(accXarr[1] * accXarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG + 4.322;
  pitch1 = atan2(-accXarr[1], accZarr[1]) * RAD_TO_DEG;
#endif

  kalmanX1.setAngle(roll1); // Set starting angle
  kalmanY1.setAngle(pitch1);
  gyroXangle1 = roll1;
  gyroYangle1 = pitch1;
  compAngleX1 = roll1;
  compAngleY1 = pitch1;

  //IMU2
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll2  = atan2(accYarr[2], accZarr[2]) * RAD_TO_DEG + 4.2753;
  pitch2 = atan(-accXarr[2] / sqrt(accYarr[2] * accYarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll2  = atan(accYarr[2] / sqrt(accXarr[2] * accXarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG + 4.2753;
  pitch2 = atan2(-accXarr[2], accZarr[2]) * RAD_TO_DEG;
#endif

  kalmanX2.setAngle(roll2); // Set starting angle
  kalmanY2.setAngle(pitch2);
  gyroXangle2 = roll2;
  gyroYangle2 = pitch2;
  compAngleX2 = roll2;
  compAngleY2 = pitch2;

  //IMU3
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll3  = atan2(accYarr[3], accZarr[3]) * RAD_TO_DEG + 2.4853;
  pitch3 = atan(-accXarr[3] / sqrt(accYarr[3] * accYarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll3  = atan(accYarr[3] / sqrt(accXarr[3] * accXarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG + 2.4853;
  pitch3 = atan2(-accXarr[3], accZarr[3]) * RAD_TO_DEG;
#endif

  kalmanX3.setAngle(roll3); // Set starting angle
  kalmanY3.setAngle(pitch3);
  gyroXangle3 = roll3;
  gyroYangle3 = pitch3;
  compAngleX3 = roll3;
  compAngleY3 = pitch3;

  //IMU4
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll4  = atan2(accYarr[4], accZarr[4]) * RAD_TO_DEG;
  pitch4 = atan(-accXarr[4] / sqrt(accYarr[4] * accYarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll4  = atan(accYarr[4] / sqrt(accXarr[4] * accXarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
  pitch4 = atan2(-accXarr[4], accZarr[4]) * RAD_TO_DEG;
#endif

  kalmanX4.setAngle(roll4); // Set starting angle
  kalmanY4.setAngle(pitch4);
  gyroXangle4 = roll4;
  gyroYangle4 = pitch4;
  compAngleX4 = roll4;
  compAngleY4 = pitch4;


  //IMU5
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll5  = atan2(accYarr[5], accZarr[5]) * RAD_TO_DEG;
  pitch5 = atan(-accXarr[5] / sqrt(accYarr[5] * accYarr[5] + accZarr[5] * accZarr[5])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll5  = atan(accYarr[5] / sqrt(accXarr[5] * accXarr[5] + accZarr[5] * accZarr[5])) * RAD_TO_DEG;
  pitch5 = atan2(-accXarr[5], accZarr[5]) * RAD_TO_DEG;
#endif

  kalmanX5.setAngle(roll5); // Set starting angle
  kalmanY5.setAngle(pitch5);
  gyroXangle5 = roll5;
  gyroYangle5 = pitch5;
  compAngleX5 = roll5;
  compAngleY5 = pitch5;

  timer = micros();
  timer1=micros();
  randomSeed(analogRead(0));
  MeasureLength();
  CalculateTargetPose();
  // turn on motor #1
  motor1.setSpeed(200);
  motor1.run(RELEASE);

  // turn on motor #2
  motor2.setSpeed(200);
  motor2.run(RELEASE);

  // turn on motor #3
  motor3.setSpeed(200);
  motor3.run(RELEASE);

  pinMode(switch_pin1, OUTPUT);
  pinMode(switch_pin2, OUTPUT);
  pinMode(switch_pin3, OUTPUT);

  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.backlight(); // finish with backlight on
  lcd.setCursor(0, 0);
  lcd.print("Current Pose");
//  lcd.setCursor(12, 0);
//  lcd.print("Target");

  //Serial.print("X");Serial.print("\t");Serial.print("Z");Serial.print("\t");Serial.print("PHI");  Serial.print("\t");Serial.print("L1");Serial.print("\t");Serial.print("\t");Serial.print("L2");Serial.print("\t");Serial.print("\t");Serial.println("L3");

}

void loop() {

dt = (double)(micros() - timer) / 1000000;
timer=micros();
//dt2 = (double)(micros() - timer1) / 1000000;
//timer1=micros();
  // First IMU: Check the table for channel 0 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, LOW);
  digitalWrite(addressC, LOW);
  digitalWrite(addressD, LOW);
  read_imus(1);
  //Serial.print("\t");
  //       Second IMU: Check the table for channel 1 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, HIGH);
  digitalWrite(addressC, LOW);
  digitalWrite(addressD, LOW);
  read_imus(2);
  //Serial.print("\t");
  //        Third IMU: Check the table for channel 2 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, LOW);
  digitalWrite(addressC, HIGH);
  digitalWrite(addressD, LOW);
  read_imus(3);
  //Serial.print("\t");
  // Forth IMU: Check the table for channel 3 (1=HIGH, 0=LOW)
  digitalWrite(addressA, LOW);
  digitalWrite(addressB, HIGH);
  digitalWrite(addressC, HIGH);
  digitalWrite(addressD, LOW);
  read_imus(4);
  //Serial.print("\t");

  
//  // Fifth IMU: Check the table for channel 3 (1=HIGH, 0=LOW)
//  digitalWrite(addressA, LOW);
//  digitalWrite(addressB, LOW);
//  digitalWrite(addressC, LOW);
//  digitalWrite(addressD, HIGH);
//  read_imus(5);


//   //Serial.print(accXarr[1]); Serial.print("\t");
//   Serial.print(accYarr[1]); Serial.print("\t");
//   Serial.print(accZarr[1]); Serial.print("\t");
//   //Serial.print(gyroXarr[1]); Serial.print("\t");
//   Serial.print(gyroYarr[1]); Serial.print("\t");
//   //Serial.print(gyroZarr[1]); Serial.print("\t");
//
//   //Serial.print(accXarr[2]); Serial.print("\t");
//   Serial.print(accYarr[2]); Serial.print("\t");
//   Serial.print(accZarr[2]); Serial.print("\t");
//   //Serial.print(gyroXarr[2]); Serial.print("\t");
//   Serial.print(gyroYarr[2]); Serial.print("\t");
//   //Serial.print(gyroZarr[2]); Serial.print("\t");
//
//   //Serial.print(accXarr[3]); Serial.print("\t");
//   Serial.print(accYarr[3]); Serial.print("\t");
//   Serial.print(accZarr[3]); Serial.print("\t");
//   //Serial.print(gyroXarr[3]); Serial.print("\t");
//   Serial.print(gyroYarr[3]); Serial.print("\t");
//   //Serial.print(gyroZarr[3]); Serial.print("\t");
//
//   //Serial.print(accXarr[4]); Serial.print("\t");
//   Serial.print(accYarr[4]); Serial.print("\t");
//   Serial.print(accZarr[4]); Serial.print("\t");
//   //Serial.print(gyroXarr[4]); Serial.print("\t");
//   Serial.print(gyroYarr[4]); Serial.print("\t");
//   //Serial.print(gyroZarr[4]); Serial.print("\t");

  //calculateIMUAngles_raw();
  calculateIMUAngles_comp();
  //calculateIMUAngles_Kalm();
      
  //Serial.print("\t");


//  calculateIMUAngles();
//  readSwitches();
//
//  if (scenario == 1){
//  //Accelerometer Values
//  direc_vec(1, roll1 + 180, 0.0, 0.0);
//  direc_vec(2, roll2, 0.0, 0.0);
//  direc_vec(3, roll3 - 180, 0.0, 0.0);
//  direc_vec(4, roll4, 0.0, 0.0);
//
//k1.calculateKinematics((double*)GLOB_resultant1, (double*)GLOB_resultant2, (double*)GLOB_resultant3, roll4, 0.0);
//  
// //Kalman Filter
//   direc_vec(1, kalAngleX1+180, 0.0, 0.0);
//   direc_vec(2, kalAngleX2, 0.0, 0.0);
//   direc_vec(3, kalAngleX3-180, 0.0, 0.0);
//   direc_vec(4, kalAngleX4, 0.0, 0.0);
//
  k2.calculateKinematics((double*)GLOB_resultant1, (double*)GLOB_resultant2, (double*)GLOB_resultant3, kalAngleX4, 0.0);
// Complimentary Values
   direc_vec(1, compAngleX1+180, 0.0, 0.0);
   direc_vec(2, compAngleX2, 0.0, 0.0);
   direc_vec(3, compAngleX3-180, 0.0, 0.0);
   direc_vec(4, compAngleX4, 0.0, 0.0);
  
k3.calculateKinematics((double*)GLOB_resultant1, (double*)GLOB_resultant2, (double*)GLOB_resultant3, compAngleX4, 0.0);

//Serial.print(k1.p[0][0]);Serial.print("\t");Serial.print(k1.p[2][0]);Serial.print("\t");Serial.print(roll4-90);Serial.print("\t");Serial.print("\t");
//Serial.print(k2.p[0][0]);Serial.print("\t");Serial.print(k2.p[2][0]);Serial.print("\t");Serial.print(kalAngleX4-90);Serial.print("\t");Serial.print("\t");//Serial.print(Length_Actual_1,3); Serial.print("\t");Serial.print(Length_Actual_2,3);Serial.print("\t");Serial.println(Length_Actual_3,3);
//Serial.print(k3.p[0][0]);Serial.print("\t");Serial.print(k3.p[2][0]);Serial.print("\t");Serial.print(compAngleX4-90);Serial.print("\t");Serial.print("\t");
//Serial.print(Length_Actual_1,3); Serial.print("\t");Serial.print(Length_Actual_2,3);Serial.print("\t");Serial.print(Length_Actual_3,3);Serial.print("\t");Serial.println(dt*1000);//Serial.print("\t");Serial.println(dt*1000);
//  
//    }
//    
//  else if (scenario == 2){
//   //Kalman Filter
//   direc_vec(1, kalAngleX1+180, 0.0, 0.0);
//   direc_vec(2, kalAngleX2, 0.0, 0.0);
//   direc_vec(3, kalAngleX3-180, 0.0, 0.0);
//   direc_vec(4, kalAngleX4, 0.0, 0.0);
//
//  }
//  
//  else if (scenario == 3){
//   //Complimentary Values
//   direc_vec(1, compAngleX1+180, 0.0, 0.0);
//   direc_vec(2, compAngleX2, 0.0, 0.0);
//   direc_vec(3, compAngleX3-180, 0.0, 0.0);
//   direc_vec(4, compAngleX4, 0.0, 0.0);
//  }

//  k1.calculateKinematics((double*)GLOB_resultant1, (double*)GLOB_resultant2, (double*)GLOB_resultant3, roll4, 0.0);
  
  
  MeasureLength();

//   Serial.print(Length_Actual_1); Serial.print("\t");
//   Serial.print(Length_Actual_2); Serial.print("\t");
//   Serial.print(Length_Actual_3); Serial.print("\t");
//   Serial.println(dt,5);
  
  Delta_Length_1 = k1.TarLen_1 -  (Length_Actual_1);
  Delta_Length_2 = k1.TarLen_2 -  (Length_Actual_2);
  Delta_Length_3 = k1.TarLen_3 -  (Length_Actual_3);
  
//  Serial.print("Target Lengths"); Serial.print("\t"); Serial.print("\t"); Serial.print(k1.TarLen_1); Serial.print("\t");
//  Serial.print(k1.TarLen_2); Serial.print("\t");
//  Serial.println(k1.TarLen_3);
//  //
//  Serial.print("Target Pose"); Serial.print("\t"); Serial.print("\t"); Serial.print(xTar); Serial.print("\t");
//  Serial.print(zTar); Serial.print("\t");
//  Serial.println(phiTar);
  //        // Current Pose LCD
  lcd.setCursor(0, 1);
  lcd.print("x=");
  lcd.setCursor(2, 1);
  lcd.print(k3.p[0][0], 1);
  lcd.setCursor(0, 2);
  lcd.print("z=");
  lcd.setCursor(2, 2);
  lcd.print(k3.p[2][0], 1);
  lcd.setCursor(0, 3);
  lcd.print("\340");
  lcd.setCursor(1, 3);
  lcd.print("=");
  lcd.setCursor(2, 3);
  lcd.print(compAngleX4-90, 1);
  lcd.setCursor(7, 3);
  lcd.print("\337");

//  //        //Target Pose LCD
//  lcd.setCursor(10, 1);
//  lcd.print("x=");
//  lcd.setCursor(12, 1);
//  lcd.print(xTar, 1);
//  lcd.setCursor(10, 2);
//  lcd.print("z=");
//  lcd.setCursor(12, 2);
//  lcd.print(zTar, 1);
//  lcd.setCursor(10, 3);
//  lcd.print("\340");
//  lcd.setCursor(11, 3);
//  lcd.print("=");
//  lcd.setCursor(12, 3);
//  lcd.print(phiTar, 1);
//  lcd.setCursor(17, 3);
//  lcd.print("\337");
//
  if ((abs(Delta_Length_1) < 2) && (abs(Delta_Length_2) < 2) && (abs(Delta_Length_3) < 2)) {
    //Serial.print("New Target Pose");
    if (pause > 50) {
      pause = 0;
      CalculateTargetPose();
    }
    else {
      pause = pause + 1;
    }
  }
  PoseControl();  
          timer_wait = timer_wait+1;
          if (timer_wait>800){
            CalculateTargetPose();
            timer_wait = 0;
            Serial.println();
          }
//Serial.print(k1.p[0][0]);Serial.print("\t");Serial.print(k1.p[2][0]);Serial.print("\t");Serial.print(roll4-90);Serial.print("\t");
//Serial.print(k2.p[0][0]);Serial.print("\t");Serial.print(k2.p[2][0]);Serial.print("\t");Serial.print(roll4-90);Serial.print("\t");
//Serial.print(k3.p[0][0]);Serial.print("\t");Serial.print(k3.p[2][0]);Serial.print("\t");Serial.print(roll4-90);Serial.print("\t");
//Serial.println(dt2);

}



void read_imus(int b)
{
  int a = b;
//  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
//  timer = micros();
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  accXarr[a] = (double)accX;
  accYarr[a] = (double)accY;
  accZarr[a] = (double)accZ;

  gyroXarr[a] = (double)gyroX;
  gyroYarr[a] = (double)gyroY;
  gyroZarr[a] = (double)gyroZ;
  //  Serial.println(accX);
  //  Serial.println(accY);
  //  Serial.println(accZ);
  //delay(15);
}


void direc_vec(int num, double r, double p, double y) //, double base_roll, double base_pitch)
{

  int number = num;
  double Roll = r * pi / 180;
  double Pitch = p * pi / 180;
  double Yaw = y * pi / 180;

  R[0][0] = cos(Pitch) * cos(Yaw);
  R[0][1] = -cos(Pitch) * sin(Yaw);
  R[0][2] = sin(Pitch);
  R[1][0] = cos(Roll) * sin(Yaw) + sin(Roll) * sin(Pitch) * cos(Yaw);
  R[1][1] = cos(Roll) * cos(Yaw) - sin(Roll) * sin(Pitch) * sin(Yaw);
  R[1][2] = -sin(Roll) * cos(Pitch);
  R[2][0] = sin(Roll) * sin(Yaw) - cos(Roll) * sin(Pitch) * cos(Yaw);
  R[2][1] = sin(Roll) * cos(Yaw) + cos(Roll) * sin(Pitch) * sin(Yaw) ;
  R[2][2] = cos(Roll) * cos(Pitch) ;

  double Rol = 0;
  double Pit = pi / 180 * 90;
  double Ya = -pi / 180 * 90;

  RotIMU[0][0] = cos(Pit) * cos(Ya);
  RotIMU[0][1] = -cos(Pit) * sin(Ya);
  RotIMU[0][2] = sin(Pit);
  RotIMU[1][0] = cos(Pit) * sin(Ya);
  RotIMU[1][1] = cos(Rol) * cos(Ya) - sin(Rol) * sin(Pit) * sin(Ya);
  RotIMU[1][2] = -sin(Rol) * cos(Pit);
  RotIMU[2][0] = sin(Rol) * sin(Ya) - cos(Rol) * sin(Pit) * cos(Ya);
  RotIMU[2][1] = sin(Rol) * cos(Ya) + cos(Rol) * sin(Pit) * sin(Ya) ;
  RotIMU[2][2] = cos(Rol) * cos(Pit) ;

  Z[0] = 0;
  Z[1] = 0;
  Z[2] = 1;

  if (number == 1) {
    Matrix.Multiply((double*)R, (double*)Z, 3, 3, 1, (double*)loc_resultant1);
    Matrix.Multiply((double*)RotIMU, (double*)loc_resultant1, 3, 3, 1, (double*)GLOB_resultant1);
    // Matrix.Print((double*)GLOB_resultant1, 3, 1, "GLOB_resultant1");

  }
  else if (number == 2) {
    Matrix.Multiply((double*)R, (double*)Z, 3, 3, 1, (double*)loc_resultant2);
    Matrix.Multiply((double*)RotIMU, (double*)loc_resultant2, 3, 3, 1, (double*)GLOB_resultant2);
    //Matrix.Print((double*)GLOB_resultant2, 3, 1, "GLOB_resultant2");
  }
  else if (number == 3) {
    Matrix.Multiply((double*)R, (double*)Z, 3, 3, 1, (double*)loc_resultant3);
    Matrix.Multiply((double*)RotIMU, (double*)loc_resultant3, 3, 3, 1, (double*)GLOB_resultant3);
    //Matrix.Print((double*)GLOB_resultant3, 3, 1, "GLOB_resultant3");
  }
  else if (number == 4) {
    Matrix.Multiply((double*)R, (double*)Z, 3, 3, 1, (double*)loc_resultant4);
    Matrix.Multiply((double*)RotIMU, (double*)loc_resultant4, 3, 3, 1, (double*)GLOB_resultant4);
    //Matrix.Print((double*)GLOB_resultant4, 3, 1, "GLOB_resultant4");
    //k1.calculateKinematics((double*) GLOB_resultant1,(double*) GLOB_resultant2,(double*) GLOB_resultant3, Roll , Pitch);

  }
  // else if(number == 5){
  //
  // }
  //Serial.println();
}

void CalculateTargetPose() {
  
  xTar = 1000;
  zTar = 1000;
  phiTar = 85;
  k1.calTargetPose(xTar, zTar, phiTar);
  while ((k1.TarLen_1 < 170) || (k1.TarLen_1 > 265) || (k1.TarLen_2 < 170) || (k1.TarLen_2 > 265) || (k1.TarLen_3 < 170) || (k1.TarLen_3 > 265) ){//|| (k1.cond_num > 10000)) {
    if (dyn_pose == 0){
    xTar =   170;  //random(7000, 30000) / 100;
    zTar =   170;   // random(7000, 30000) / 100;
    phiTar = -23;      //random(-4500, 4500) / 100;
    k1.calTargetPose(xTar, zTar, phiTar);
    //  k1.TarLen_1=200;
    //  k1.TarLen_2=240;
    //  k1.TarLen_3=260;
    dyn_pose = 1;
//    lcd.clear();
//    Serial.println();
//    lcd.print("Pose 1");
    break;
    
    }
    else if (dyn_pose == 1){
      xTar = 230;    //random(7000, 30000) / 100;
    zTar =  180;    // random(7000, 30000) / 100;
    phiTar =   0;    //random(-4500, 4500) / 100;
   dyn_pose = 0;
//   Serial.println();
//   lcd.clear();
//   lcd.print("Pose 2");
   k1.calTargetPose(xTar, zTar, phiTar);
  break;
  
    }
    
  }

  //
  //


  //Serial.print(xTar); Serial.print("\t");
  //Serial.print(zTar); Serial.print("\t");
  //Serial.print(phiTar); Serial.println();
}

void PoseControl() {
  //if  ((Szenario==1)||(Szenario==3)) {
  //if  (Szenario==1){
  K1 = 25;
  K3 = 0.01;
  delta1 = sqrt((delta1 - Delta_Length_1) * (delta1 - Delta_Length_1));
  sum1 = sum1 + Delta_Length_1;
  sum1 = constrain(sum1, -500, 500);
  //u_1 = K1*(Delta_Length_1 +sum1/5 - K3*delta1);
  u_1 = K1 * (Delta_Length_1 - K3 * delta1);
  u_1 = constrain(u_1, -255, 255);
  //Serial.print(sum1);Serial.print("\t");Serial.println(u_1);

  delta2 = sqrt((delta2 - Delta_Length_2) * (delta2 - Delta_Length_2));
  sum2 = sum2 + Delta_Length_2;
  sum2 = constrain(sum2, -500, 500);
  //u_2 = K1*(Delta_Length_2 +sum2/5- K3*delta2);
  u_2 = K1 * (Delta_Length_2 - K3 * delta2);
  u_2 = constrain(u_2, -255, 255);
  // Serial.print(sum2);Serial.print("\t");Serial.println(u_2);

  delta3 = sqrt((delta3 - Delta_Length_3) * (delta3 - Delta_Length_3));
  sum3 = sum3 + Delta_Length_3;
  sum3 = constrain(sum3, -500, 500);
  //u_1 = K1*(Delta_Length_1 +sum1/5 - K3*delta1);
  u_3 = K1 * (Delta_Length_3 - K3 * delta3);
  u_3 = constrain(u_3, -255, 255);
  //}
  //else if  ((Szenario==2)||(Szenario==3)){
  //  K1=50;
  //--  delta1=sqrt((delta1-Delta_Length_1)*(delta1-Delta_Length_1));
  //  u_1 = K1*(Delta_Length_1);
  //  u_1=constrain(u_1, -255, 255);
  // // Serial.print(sum1);Serial.print("\t");Serial.println(u_1);
  //
  //  delta2=sqrt((delta2-Delta_Length_2)*(delta2-Delta_Length_2));
  //  u_2 = K1*(Delta_Length_2);
  //  u_2 = constrain(u_2, -255, 255);
  //  //Serial.print(sum2);Serial.print("\t");Serial.println(u_2);
  //}
  //else {
  // u_1=0;
  // u_2=0;
  //}
  //Serial.print("Controller Output"); Serial.print("\t"); Serial.print(u_1); Serial.print("\t");
  //Serial.print(u_2); Serial.print("\t");
  //Serial.println(u_3);
  //Serial.println();

  if (u_1 > 0) {
    motor1.run(FORWARD);
    motor1.setSpeed(u_1);
  }
  else if (u_1 < 0) {
    u_1 = -u_1;
    motor1.run(BACKWARD);
    motor1.setSpeed(u_1);
  }

  if (u_2 > 0) {
    motor2.run(FORWARD);
    motor2.setSpeed(u_2);
  }
  else if (u_2 < 0) {
    u_2 = -u_2;
    motor2.run(BACKWARD);
    motor2.setSpeed(u_2);
  }

  if (u_3 > 0) {
    motor3.run(FORWARD);
    motor3.setSpeed(u_3);
  }
  else if (u_3 < 0) {
    u_3 = -u_3;
    motor3.run(BACKWARD);
    motor3.setSpeed(u_3);
  }


  delta1 = Delta_Length_1;
  delta2 = Delta_Length_2;
  delta3 = Delta_Length_3;
  //Serial.println(delta1);
  //Serial.println(delta2);
  //Serial.println(delta3);
}

void MeasureLength() {

  Length_Actual_1 = 168.4 + (0.101 * analogRead(Pin1) + 1.191);
  //Serial.print("Current Lengths"); Serial.print("\t"); Serial.print("\t"); Serial.print(Length_Actual_1, 3); Serial.print("\t");


  Length_Actual_2 = 168.4 + (0.101 * analogRead(Pin2) +1.029);
  //Serial.print(Length_Actual_2, 3); Serial.print("\t");
  //
  Length_Actual_3 = 168.4 + (0.101 * analogRead(Pin3) - 0.4);
  //Serial.println(Length_Actual_3, 3);

 // Serial.print("Current Lengths");Serial.print("\t");Serial.print("\t");Serial.print(Length_Actual_1,3); Serial.print("\t");Serial.print(Length_Actual_2,3);Serial.print("\t");Serial.println(Length_Actual_3,3);
}

void readSwitches(){
  switch_1 = digitalRead(switch_pin1);
  switch_2 = digitalRead(switch_pin2);
  switch_3 = digitalRead(switch_pin3);
  if (switch_1 == 1 && switch_2 == 0 && switch_3 == 0){
    //Pass raw values
    scenario = 1;
  }
  else if (switch_2 == 1 && switch_1 == 0 && switch_3 == 0){
    //Pass Kalman values
    scenario = 2;
  }
  else if (switch_3 == 1 && switch_1 == 0 && switch_2== 0){
    //Pass Complimentary values
    scenario = 3;
  }
//  else if (switch_1 == LOW && switch_2 == LOW){
//    
//  }
//  else if (switch_2 == LOW && switch_3 == LOW){
//    
//  }
//  else if (switch_1 == LOW && switch_3 == LOW){
//    
//  }
}
//  Switch_Volt=analogRead(Pin0);
//  //Serial.println(Switch_Volt);
//  if (Switch_Volt == 0){
//    //Serial.println("All Switches are off --> Szenario=1");
//    Szenario=1;
//  }
//  else if (Switch_Volt > 1020){
//    //Serial.println("S1 is on --> Szenario=2");
//    Szenario=2;
//  }
//  else if ((Switch_Volt < 800) && ((Switch_Volt > 700))){
//    //Serial.println("S2 is on --> Szenario=3");
//    Szenario=3;
//  }
//  else if ((Switch_Volt < 500) && ((Switch_Volt > 400))){
//   //Serial.println("S3 is on");
//   Szenario=4;
//  }
//  else if ((Switch_Volt < 700) && ((Switch_Volt > 600))){
//   //Serial.println("S2 and S3 are on");
//   Szenario=5;
//  }
//}


void calculateIMUAngles_raw()
{
//IMU1
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll1  = atan2(accYarr[1], accZarr[1]) * RAD_TO_DEG + 4.322;
  pitch1 = atan(-accXarr[1] / sqrt(accYarr[1] * accYarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll1  = atan(accYarr[1] / sqrt(accXarr[1] * accXarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG + 4.322;
  pitch1 = atan2(-accXarr[1], accZarr[1]) * RAD_TO_DEG;
#endif

//IMU2
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll2  = atan2(accYarr[2], accZarr[2]) * RAD_TO_DEG + 4.2753;
  pitch2 = atan(-accXarr[2] / sqrt(accYarr[2] * accYarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll2  = atan(accYarr[2] / sqrt(accXarr[2] * accXarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG + 4.2753;
  pitch2 = atan2(-accXarr[2], accZarr[2]) * RAD_TO_DEG;
#endif

//IMU3
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll3  = atan2(accYarr[3], accZarr[3]) * RAD_TO_DEG + 2.4853;
  pitch3 = atan(-accXarr[3] / sqrt(accYarr[3] * accYarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG + 2.4853;
#else // Eq. 28 and 29
  roll3  = atan(accYarr[3] / sqrt(accXarr[3] * accXarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG;
  pitch3 = atan2(-accXarr[3], accZarr[3]) * RAD_TO_DEG;
#endif

//IMU4
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll4  = atan2(accYarr[4], accZarr[4]) * RAD_TO_DEG;
  pitch4 = atan(-accXarr[4] / sqrt(accYarr[4] * accYarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll4  = atan(accYarr[4] / sqrt(accXarr[4] * accXarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
  pitch4 = atan2(-accXarr[4], accZarr[4]) * RAD_TO_DEG;
#endif

//   Serial.print(roll1); Serial.print("\t");
//   Serial.print(roll2); Serial.print("\t");
//   Serial.print(roll3); Serial.print("\t");
//   Serial.print(roll4); Serial.print("\t");

}

void calculateIMUAngles_comp()
{
//IMU1
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll1  = atan2(accYarr[1], accZarr[1]) * RAD_TO_DEG + (4.322 + 0.6508);
  pitch1 = atan(-accXarr[1] / sqrt(accYarr[1] * accYarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll1  = atan(accYarr[1] / sqrt(accXarr[1] * accXarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG +(4.322 + 0.6508);
  pitch1 = atan2(-accXarr[1], accZarr[1]) * RAD_TO_DEG;
#endif
gyroXrate1 = gyroXarr[1] / 131.0; // Convert to deg/s
gyroYrate1 = gyroYarr[1] / 131.0 -2.64-.42; // Convert to deg/s
compAngleX1 = 0.93* (compAngleX1 - gyroYrate1 * dt) + 0.07 * roll1; // Calculate the angle using a Complimentary filter
compAngleY1 = 0.93 * (compAngleY1 - gyroXrate1 * dt) + 0.07 * pitch1;

//IMU2
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll2  = atan2(accYarr[2], accZarr[2]) * RAD_TO_DEG + (4.2753 );
  pitch2 = atan(-accXarr[2] / sqrt(accYarr[2] * accYarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll2  = atan(accYarr[2] / sqrt(accXarr[2] * accXarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG + (4.2753 );
  pitch2 = atan2(-accXarr[2], accZarr[2]) * RAD_TO_DEG;
#endif
gyroXrate2 = gyroXarr[2] / 131.0; // Convert to deg/s
gyroYrate2 = gyroYarr[2] / 131.0-0.7412-3.25; // Convert to deg/s
compAngleX2 = 0.93 * (compAngleX2 - gyroYrate2 * dt) + 0.07* roll2; // Calculate the angle using a Complimentary filter
compAngleY2 = 0.93 * (compAngleY2 - gyroXrate2 * dt) + 0.07 * pitch2;

//IMU3
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll3  = atan2(accYarr[3], accZarr[3]) * RAD_TO_DEG + (2.5);
  pitch3 = atan(-accXarr[3] / sqrt(accYarr[3] * accYarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll3  = atan(accYarr[3] / sqrt(accXarr[3] * accXarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG +(2.5);
  pitch3 = atan2(-accXarr[3], accZarr[3]) * RAD_TO_DEG;
#endif
gyroXrate3 = gyroXarr[3] / 131.0; // Convert to deg/s
gyroYrate3 = gyroYarr[3] / 131.0 -1.35; // Convert to deg/s
compAngleX3 = 0.93* (compAngleX3 - gyroYrate3 * dt) + 0.07* roll3; // Calculate the angle using a Complimentary filter
compAngleY3 = 0.93 * (compAngleY3 - gyroXrate3 * dt) + 0.07* pitch3;

//IMU4
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll4  = atan2(accYarr[4], accZarr[4]) * RAD_TO_DEG - 0.5837;
  pitch4 = atan(-accXarr[4] / sqrt(accYarr[4] * accYarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll4  = atan(accYarr[4] / sqrt(accXarr[4] * accXarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG - 0.5837;
  pitch4 = atan2(-accXarr[4], accZarr[4]) * RAD_TO_DEG;
#endif
gyroXrate4 = gyroXarr[4] / 131.0; // Convert to deg/s
gyroYrate4 = gyroYarr[4] / 131.0 -.15-.19; // Convert to deg/s
compAngleX4 = 0.93* (compAngleX4 - gyroYrate4 * dt) + 0.07 * roll4; // Calculate the angle using a Complimentary filter
compAngleY4 = 0.93 * (compAngleY4 - gyroXrate4 * dt) + 0.07 * pitch4;

//   Serial.print(roll1); Serial.print("\t");
//   Serial.print(roll2); Serial.print("\t");
//   Serial.print(roll3); Serial.print("\t");
//   Serial.print(roll4); Serial.print("\t");
//
   Serial.print(compAngleX1); Serial.print("\t");
   Serial.print(compAngleX2); Serial.print("\t");
   Serial.print(compAngleX3); Serial.print("\t");
   Serial.print(compAngleX4); Serial.println();
}

void calculateIMUAngles_Kalm()
{
  //IMU1
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll1  = atan2(accYarr[1], accZarr[1]) * RAD_TO_DEG + 4.322;
  pitch1 = atan(-accXarr[1] / sqrt(accYarr[1] * accYarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll1  = atan(accYarr[1] / sqrt(accXarr[1] * accXarr[1] + accZarr[1] * accZarr[1])) * RAD_TO_DEG + 4.322;
  pitch1 = atan2(-accXarr[1], accZarr[1]) * RAD_TO_DEG;
#endif

  gyroXrate1 = gyroXarr[1] / 131.0; // Convert to deg/s
  gyroYrate1 = gyroYarr[1] / 131.0 -2.64; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll1 < -90 && kalAngleX1 > 90) || (roll1 > 90 && kalAngleX1 < -90)) {
    kalmanX1.setAngle(roll1);
    compAngleX1 = roll1;
    kalAngleX1 = roll1;
    gyroXangle1 = roll1;
  } else

    kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX1) > 90)
    gyroYrate1 = -gyroYrate1; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees

  if ((pitch1 < -90 && kalAngleY1 > 90) || (pitch1 > 90 && kalAngleY1 < -90)) {
    kalmanY1.setAngle(pitch1);
    compAngleY1 = pitch1;
    kalAngleY1 = pitch1;
    gyroYangle1 = pitch1;
  }

  else

    kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY1) > 90)
    gyroXrate1 = -gyroXrate1; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt); // Calculate the angle using a Kalman filter
#endif

  //IMU2
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll2  = atan2(accYarr[2], accZarr[2]) * RAD_TO_DEG + 4.2753;
  pitch2 = atan(-accXarr[2] / sqrt(accYarr[2] * accYarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll2  = atan(accYarr[2] / sqrt(accXarr[2] * accXarr[2] + accZarr[2] * accZarr[2])) * RAD_TO_DEG + 4.2753;
  pitch2 = atan2(-accXarr[2], accZarr[2]) * RAD_TO_DEG;
#endif

  gyroXrate2 = gyroXarr[2] / 131.0; // Convert to deg/s
  gyroYrate2 = gyroYarr[2] / 131.0-0.7412; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else

    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees

  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  }

  else

    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt); // Calculate the angle using a Kalman filter
#endif

  //IMU3
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll3  = atan2(accYarr[3], accZarr[3]) * RAD_TO_DEG + 2.4853;
  pitch3 = atan(-accXarr[3] / sqrt(accYarr[3] * accYarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG + 2.4853;
#else // Eq. 28 and 29
  roll3  = atan(accYarr[3] / sqrt(accXarr[3] * accXarr[3] + accZarr[3] * accZarr[3])) * RAD_TO_DEG;
  pitch3 = atan2(-accXarr[3], accZarr[3]) * RAD_TO_DEG;
#endif

  gyroXrate3 = gyroXarr[3] / 131.0; // Convert to deg/s
  gyroYrate3 = gyroYarr[3] / 131.0 -1.6130; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll3 < -90 && kalAngleX3 > 90) || (roll3 > 90 && kalAngleX3 < -90)) {
    kalmanX3.setAngle(roll3);
    compAngleX3 = roll3;
    kalAngleX3 = roll3;
    gyroXangle3 = roll3;
  } else

    kalAngleX3 = kalmanX3.getAngle(roll3, gyroXrate3, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX3) > 90)
    gyroYrate3 = -gyroYrate3; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleY3 = kalmanY3.getAngle(pitch3, gyroYrate3, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees

  if ((pitch3 < -90 && kalAngleY3 > 90) || (pitch3 > 90 && kalAngleY3 < -90)) {
    kalmanY3.setAngle(pitch3);
    compAngleY3 = pitch3;
    kalAngleY3 = pitch3;
    gyroYangle3 = pitch3;
  }

  else

    kalAngleY3 = kalmanY3.getAngle(pitch3, gyroYrate3, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY3) > 90)
    gyroXrate3 = -gyroXrate3; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleX3 = kalmanX3.getAngle(roll3, gyroXrate3, dt); // Calculate the angle using a Kalman filter
#endif

  //IMU4
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll4  = atan2(accYarr[4], accZarr[4]) * RAD_TO_DEG;
  pitch4 = atan(-accXarr[4] / sqrt(accYarr[4] * accYarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll4  = atan(accYarr[4] / sqrt(accXarr[4] * accXarr[4] + accZarr[4] * accZarr[4])) * RAD_TO_DEG;
  pitch4 = atan2(-accXarr[4], accZarr[4]) * RAD_TO_DEG;
#endif

  gyroXrate4 = gyroXarr[4] / 131.0; // Convert to deg/s
  gyroYrate4 = gyroYarr[4] / 131.0 -0.3003; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll4 < -90 && kalAngleX4 > 90) || (roll4 > 90 && kalAngleX4 < -90)) {
    kalmanX4.setAngle(roll4);
    compAngleX4 = roll4;
    kalAngleX4 = roll4;
    gyroXangle4 = roll4;
  } else

    kalAngleX4 = kalmanX4.getAngle(roll4, gyroXrate4, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX4) > 90)
    gyroYrate4 = -gyroYrate4; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleY4 = kalmanY4.getAngle(pitch4, gyroYrate4, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees

  if ((pitch4 < -90 && kalAngleY4 > 90) || (pitch4 > 90 && kalAngleY4 < -90)) {
    kalmanY4.setAngle(pitch4);
    compAngleY4 = pitch4;
    kalAngleY4 = pitch4;
    gyroYangle4 = pitch4;
  }

  else

    kalAngleY4 = kalmanY4.getAngle(pitch4, gyroYrate4, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY4) > 90)
    gyroXrate4 = -gyroXrate4; // Invert rate, so it fits the restriced accelerometer reading

  kalAngleX4 = kalmanX4.getAngle(roll4, gyroXrate4, dt); // Calculate the angle using a Kalman filter
#endif

//   Serial.print(roll1); Serial.print("\t");
//   Serial.print(roll2); Serial.print("\t");
//   Serial.print(roll3); Serial.print("\t");
//   Serial.print(roll4); Serial.print("\t");
//
//   Serial.print(kalAngleX1); Serial.print("\t");
//   Serial.print(kalAngleX2); Serial.print("\t");
//   Serial.print(kalAngleX3); Serial.print("\t");
//   Serial.print(kalAngleX4); Serial.print("\t");
}
