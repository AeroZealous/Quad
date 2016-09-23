/*
  AeroQuad v3.1 - January 2013
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include<Wire.h>
#include<AQMath.h>
#include<Device_I2C.h>
#include<Gyroscope_ITG3200.h>
//#include <Accelerometer_BMA180.h>
#include <GlobalDefined.h>
#include "acc_filter.h"
#include <Receiver_328p.h> // for AeroQuad shield v1.x with Arduino Due/Uno and mini shield v1.0 using a standard PWM receiver
#include <Motors_PWM_Timer.h>
//Choose how many channels you have available (6, 8 or 10)

#define LASTCHANNEL 6

// -------------  End of configuration ----------------- //

#define FRONT_LEFT  MOTOR1
#define FRONT_RIGHT MOTOR2
#define REAR_RIGHT  MOTOR3
#define REAR_LEFT   MOTOR4

unsigned long timer;

//The channel values read from RECEIVER
int throttle, yaw, pitch, roll;

//The values read from the GYRO
int gyroll, gypitch, gyyaw;

//The values read from the ACCELEROMETER
float acx , acy, acz;
//The offset of gyro at ground level
float ac0x, ac0y, ac0z;

//Compensation values calculated for various axes
int pitchF, pitchR, rollL, rollR, yawCC, yawCCW;

//D variables
int error_p_prev, error_r_prev;
//I variable
int error_p_total, error_r_total, error_y_total;

//the maximum change in throttle the pich and roll sticks can make
#define PITCH_MAX   200
#define ROLL_MAX    200

#define constrainPPM(x) (constrain((x), 1000,2000))
float x0, y0, z0;

/*************** for accelerometer*****/
#define window_size 3
float target_roll = 0, target_pitch = 0, roll_angle = 0, pitch_angle = 0, yaw_angle, target_yaw = 0, roll_angle_prev, pitch_angle_prev;

/******************/


void setup() {

  Serial.begin(115200);

  initializeReceiver(LASTCHANNEL);
  initializeMotors(FOUR_Motors);
  receiverXmitFactor = 1.0;

  Wire.begin();
  initializeGyro();
  initializeAccel();
  calibrateGyro();
  computeAccelBias();
  accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = -0.047;
  measureAccel();
 
}

void loop() {

  if ((millis() - timer) > 20) // 50Hz
  {
    timer = millis();

    readReceiver();
    measureGyro();
    measureAccel();
    throttle = receiverCommand[THROTTLE];

    yaw = receiverCommand[ZAXIS];
    roll = receiverCommand[XAXIS];
    pitch = receiverCommand[YAXIS];

    gyroll = degrees(gyroRate[XAXIS]);
    gypitch = degrees(gyroRate[YAXIS]);
    gyyaw = degrees(gyroRate[ZAXIS]);


/**************** accelerometer **************************/

   
    if(!check) {
      initiate(meterPerSecSec[ZAXIS], med_bufferz);
      initiate(meterPerSecSec[XAXIS], med_bufferx);
      initiate(meterPerSecSec[YAXIS], med_buffery);

                
      accel_net = sqrt(meterPerSecSec[ZAXIS]*meterPerSecSec[ZAXIS] + meterPerSecSec[XAXIS]*meterPerSecSec[XAXIS] + meterPerSecSec[YAXIS]*meterPerSecSec[YAXIS]);    //accel_net acceleration
      initiate(accel_net, med_buffer_net);
      check = 2;
    } else if(check == 2) {
      pushupdate(med_bufferz, meterPerSecSec[ZAXIS]);
      pushupdate(med_bufferx, meterPerSecSec[XAXIS]);
      pushupdate(med_buffery, meterPerSecSec[YAXIS]);

  
      accel_z = sort_med(med_bufferz);    //for accel_net
      accel_x = sort_med(med_bufferx); 
      accel_y = sort_med(med_buffery);
      accel_net = sqrt(accel_z*accel_z + accel_x*accel_x + accel_y*accel_y);
      pushupdate(med_buffer_net, accel_net);
      accel_net = sort_med(med_buffer_net);
    }
    /*if(gypitch < 20){
      pitch_angle = degrees(atan2(accel_x , sqrt(pow(accel_y, 2) + pow(accel_z , 2))));
      pitch_angle = pitch_angle - target_pitch;
      
    }else{Serial.print("\nPitch:");}
    if(gyroll < 20){
       roll_angle = degrees(atan2(accel_y , sqrt(pow(accel_z, 2) + pow(accel_x , 2))));
        roll_angle = roll_angle - target_roll;
        
    }else{Serial.print("\nPitch:");}
    yaw_angle = degrees(atan2(accel_z , sqrt(pow(accel_x, 2) + pow(accel_y , 2))));
    yaw_angle = yaw_angle - target_yaw; */


      pitch_angle = degrees(atan2(accel_x , sqrt(pow(accel_y, 2) + pow(accel_z , 2))));
      pitch_angle = pitch_angle - target_pitch;
      if(pitch_angle - pitch_angle_prev > 10){
        pitch_angle = 0;
        Serial.print("\nPitch:");
      }
    
       roll_angle = degrees(atan2(accel_y , sqrt(pow(accel_z, 2) + pow(accel_x , 2))));
       roll_angle = roll_angle - target_roll;
       if(roll_angle - roll_angle_prev > 10){
        roll_angle = 0;
        Serial.print("\nPitch:");
       }
       pitch_angle_prev = pitch_angle;
       roll_angle_prev = roll_angle; 

//    Serial.print("\nROll:");
//    Serial.print(roll_angle);
//    Serial.print("\tPitch:");
//    Serial.print(pitch_angle);
//Serial.print("\nYaw:");
//    Serial.print(roll_angle);
    
   
/******************************************/



    
    if (receiverCommand[MODE] < 1500) { //Safety Check
      //MOTORS
      motorCommand[FRONT_LEFT]  = 1000;
      motorCommand[FRONT_RIGHT] = 1000 ;
      motorCommand[REAR_LEFT]   = 1000;
      motorCommand[REAR_RIGHT]  = 1000 ;
      //      Serial.print(" OFF");

      //Also zero the integral error
      error_p_total = error_r_total = 0;error_y_total = 0;
    }
    else {
      pitchF = 0;
      pitchR = 0;
      rollL = 0;
      rollR = 0;
      yawCC = 0;
      yawCCW = 0;
      //calculate PITCH compensation for front and rear motors
      if (pitch > 1509) {
        //lift rear motors up
        // pitch 1500-2000 mapped to pitchR 0-PITCH_MAX
        pitchR = map(pitch, 1500, 2000, 0, PITCH_MAX);
        pitchF = -pitchR;
      } else if (pitch < 1491) {
        //lift front motors up
        //pitch 1500-1000 mapped to pitchF 0-PITCH_MAX
        pitchF = map(pitch, 1500, 1000, 0, PITCH_MAX);
        pitchR = -pitchF;
      }

      //calculate ROLL compensation for left and right motors
      if (roll > 1509) {
        //lift left motors up
        //roll 1500-2000 mapped to rollL 0-ROLL_MAX
        rollL = map(roll, 1500, 2000, 0, ROLL_MAX);
        rollR = -rollL;
      } else if(roll < 1491){
        //lift right motors up
        //roll 1500-1000 mapped to rollR 0-ROLL_MAX
        rollR = map(roll, 1500, 1000, 0, ROLL_MAX);
        rollL = -rollR;
      }

      if (yaw > 1509) {
        //lift left motors up
        //roll 1500-2000 mapped to rollL 0-ROLL_MAX
        yawCCW = map(roll, 1500, 2000, 0, ROLL_MAX);
      } else if(yaw < 1491){
        //lift right motors up
        //roll 1500-1000 mapped to rollR 0-ROLL_MAX
        yawCC = map(roll, 1500, 1000, 0, ROLL_MAX);
      }

      /* Proportional
       * Short summary: The larger the error, the harder it tries to compensate.
       * Thus the compensation is *proportional* to the erro
       */

      //calculate roll pitch compensation
#define P 1.5

      pitchF -= gypitch * P ;
      pitchR += gypitch * P ;  //+ve gypitch indicates rear dive, so increase rear thrust

      rollL -= gyroll * P;     
      rollR += gyroll * P;     //+ve gyroll indicates right side dive, so compensate by increaseing right thrust

#define Py 2
      yawCC  += gyyaw * Py;   //+ve gyyaw indicates clockwise drift of the body, so compensate by increasing clockwise motors speed 
      yawCCW -= gyyaw * Py;   // thus inducing anti-clockwise reaction om the body


      /* Derivative
       * Short summary: It tries to reduce the rate of change of error, for better or worse
       * It acts as friction to reduce overcompensation by P code
       */
#define D 2

      int delta_pitch = gypitch - error_p_prev;
      int delta_roll = gyroll - error_r_prev;

      pitchF -= delta_pitch * D;
      pitchR += delta_pitch * D;
      rollL -= delta_roll * D;
      rollR += delta_roll * D;

      error_p_prev = gypitch;
      error_r_prev = gyroll;
      
      /* Integral
       * Short summary: It remembers the bias due to error and tries to equal the +ve and -ve errors.
       * It is possible that at a slanted orientation the gyro readings are zero and hece both P and D become inactive.
       * But I remembers that there is error from original position which has not been compensated and acts in that direction.
       */
#define I 1
#define I_MAX 40
      error_p_total += gypitch * 0.005;
      pitchF -= constrain(I * error_p_total, -I_MAX, I_MAX );
      pitchR += constrain(I * error_p_total, -I_MAX, I_MAX );

      error_r_total += gyroll * 0.005;
      rollL -= constrain(I * error_r_total, -I_MAX, I_MAX);
      rollR += constrain(I * error_r_total, -I_MAX, I_MAX);


      error_y_total += gyyaw * 0.005;
      yawCC += constrain(I * error_y_total, -I_MAX, I_MAX);
      yawCCW -= constrain(I * error_y_total, -I_MAX, I_MAX);
//      Serial.print(yawCC);




/******************* Accelerometer PID *************************/

// constrain angles.
//    roll_angle = constrain();


/*#define P_ACC 1
      rollL += roll_angle * P_ACC;
      rollR -= roll_angle * P_ACC;

      pitchR += pitch_angle * P_ACC;
      pitchF -= pitch_angle * P_ACC; */



/***************************************************************/


   /* writing into the motors : last step */
      motorCommand[FRONT_LEFT]  = constrainPPM( throttle + pitchF + rollL + yawCC);
      motorCommand[FRONT_RIGHT] = constrainPPM( throttle + pitchF + rollR + yawCCW);
      motorCommand[REAR_LEFT]   = constrainPPM( throttle + pitchR + rollL + yawCCW);
      motorCommand[REAR_RIGHT]  = constrainPPM( throttle + pitchR + rollR + yawCC);
    }

    writeMotors();/*
    Serial.println();
    Serial.print("pf:");
    Serial.print(pitchF);
    Serial.print("\tpR:");
    Serial.print(pitchR);
    Serial.print("\trL:");
    Serial.print(rollL);
    Serial.print("\trR:");
    Serial.print(rollR);*/
    Serial.println();
  }
}
