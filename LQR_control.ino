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

//LQ
#include <LMotorController.h>

#define MIN_ABS_SPEED 30

#define fullturnTicks 2240
#define lengthOfWheel 0.2826

#define ToPhiRad(x) ((x)*0.00140249672)//0.00140249672

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi


//gyro_scale
//MOTOR CONTROLLER
double motorSpeedFactorLeft = 0.752;
double motorSpeedFactorRight = 0.948;

int ENA = 11;
int IN1 = 8;//left back
int IN2 = 9;//left forward
int IN3 = 6;//right back
int IN4 = 7;//right forward
int ENB = 5;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);








#define coordX 0
#define coordY 1
#define coordZ 2

float angle=-3;//in degree // in radian angle =.05
float balanceAt=-3;

int encoder0PinA = 18;
int encoder0PinB = 19;


long spd=255;

volatile int encoder0Pos = 0;


long timer_=0;   //general purpuse timer
long timer_old;

//LQ



#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  //LQ
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);

  //LQ




  
  Serial.begin(115200);
  Wire.begin();


  //LQ
  attachInterrupt(2, doEncoder, CHANGE);  // encoder pin on interrupt 1 - pin 3
  timer_=millis();







  //LQ

  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}
//LQ
 float K1=-1 ,K2=-2.189 ,K3=34.8086 ,K4= 7.4225 ;
 long getLQRSpeed(float phi,float dphi,float angle,float dangle){
  return constrain((phi*K1+dphi*K2+K3*angle+dangle*K4)*285,-255,255);
 }
 float lastPhi=0,dPhi=0;
 float G_Dt=0.02;


 
float getPhiAdding(float dif){

  if(dif<200 && dif>-200){return 0.f;}
  float ret = dif*0.08;

  return ret;
}

float getFactorAdding(float dif){
  if(dif<200 && dif>-200){return 0.f;}
  float ret = dif/500*20;
  return ret;
}


float phiDif=0.f;
float factorDif=0.f;

 





//LQ

void loop() {


  
  /*   if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    if (Serial.available()){
    BluetoothData=Serial.read();
     if(BluetoothData=='w'){   
      phiDif=200;//constrain(phiDif+10,-200,200);
     } else if(BluetoothData=='s'){ 
      phiDif=-200;//constrain(phiDif-10,-200,200);      
     } else if(BluetoothData=='a'){   
      factorDif=200;//constrain(factorDif+10,-200,200);
     } else if(BluetoothData=='d'){   
      factorDif=-200;//constrain(factorDif-10,-200,200);      
     } else if(BluetoothData=='c'){   
      factorDif=0;//constrain(factorDif-10,-200,200);      
      phiDif=0;
     }
    }
   * /
   */
   //LQ
   timer_old=timer_;
   timer_=millis();
   if (timer>timer_old){
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    }
   else{
      G_Dt = 0;
    }








   
   //LQ
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  //LQ
  if(abs(pitch)>1 && abs(pitch)<5){

    dPhi=(ToPhiRad(encoder0Pos)-lastPhi)/G_Dt;
     
    encoder0Pos+=getPhiAdding(phiDif);
    
    lastPhi=ToPhiRad(encoder0Pos);

    spd=getLQRSpeed(ToPhiRad(encoder0Pos),dPhi,balanceAt-compAngleY, gyroYrate);
     float factorL=getFactorAdding(factorDif);
     motorController.move(-abs(spd-factorL),abs(spd+factorL), MIN_ABS_SPEED);
     //Serial.println("speed:");
     //Serial.print(factorL);
  }
  else{
      digitalWrite(8,HIGH);
      digitalWrite(9,LOW);
      analogWrite(11,0);
      digitalWrite(6,HIGH);
      digitalWrite(7,LOW);
      analogWrite(5,0);
  
      Serial.println("stop");}

   if(millis()%50==0){
      Serial.println(compAngleY);
    }
    
    
    
    
    
    
    
    






  //LQ








  /* Print Data */
#if 0 // Set to 1 to activate
  //Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  //Serial.print(accZ); Serial.print("\t");

  //Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  //Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  //Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");
  delay(2);
}


void doEncoder() {
  if (digitalRead(encoder0PinA) == HIGH) {
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  if(encoder0Pos>fullturnTicks*3 || encoder0Pos<-fullturnTicks*3){
    encoder0Pos=0;
  }
}
