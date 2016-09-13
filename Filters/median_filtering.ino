/*
  AeroQuad v3.0 - March 2011
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

#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include <AQMath.h>
#include <Accelerometer_BMA180.h>

unsigned long timer;

float avg = 0;
void setup() {
  
  Serial.begin(115200);
//  /Serial.println("Accelerometer library test (BMA180)");

  Wire.begin();
  
  initializeAccel();
  computeAccelBias();
  // changing the scale factor is necessary to get non-0 values from this test sketch
  // don't worry about how accurate the data is at this point, just make sure it changes as you move the board
  accelScaleFactor[XAXIS]=accelScaleFactor[YAXIS]=accelScaleFactor[ZAXIS]=-0.038;

}


//--------------to remove noise, not impulse!----------------------------------//

#define window_size 3

float temp, med_bufferz[window_size], med_bufferx[window_size], med_buffery[window_size], med_buffer_net[window_size];
float sort_med(float *temp_buffer) {
  float buffer[window_size];
  for(int i = 0; i < window_size; i++) {
    buffer[i] = temp_buffer[i];
  }
  for(int i = 0; i < window_size; i++) {
    for(int j = i + 1; j < window_size; j++) {
      if(buffer[i] > buffer[j]) {
        temp = buffer[j];
        buffer[j] = buffer[i];
        buffer[i] = temp;
      }
    }
  }
  return buffer[(window_size / 2)];
}

//replace input with some seed value if you want
void initiate(float input, float *temp_buffer) {
  for(int i = 0; i < window_size; i++) {
	temp_buffer[i] = input;
  }
}
void pushupdate(float *temp_buffer, float input) {
  for(int i = 0; i < (window_size - 1); i++) {
	temp_buffer[i] = temp_buffer[i+1];
  }
  temp_buffer[(window_size - 1)] = input;
}

int check = 0;
float accel_z, accel_x, accel_y, accel_net;
void loop() {
  
  if((millis() - timer) > 20) // 100Hz
  {
    timer = millis();
    measureAccel();
    if(!check) {
      initiate(meterPerSecSec[ZAXIS], med_bufferz);
      initiate(meterPerSecSec[XAXIS], med_bufferx);
      initiate(meterPerSecSec[YAXIS], med_buffery);

								
      accel_net = sqrt(meterPerSecSec[ZAXIS]*meterPerSecSec[ZAXIS] + meterPerSecSec[XAXIS]*meterPerSecSec[XAXIS] + meterPerSecSec[YAXIS]*meterPerSecSec[YAXIS]);		//accel_net acceleration
      initiate(accel_net, med_buffer_net);
      check = 2;
    } else if(check == 2) {
      pushupdate(med_bufferz, meterPerSecSec[ZAXIS]);
      pushupdate(med_bufferx, meterPerSecSec[XAXIS]);
      pushupdate(med_buffery, meterPerSecSec[YAXIS]);

	
      accel_z = sort_med(med_bufferz);		//for accel_net
      accel_x = sort_med(med_bufferx);
      accel_y = sort_med(med_buffery);
      accel_net = sqrt(accel_z*accel_z + accel_x*accel_x + accel_y*accel_y);
      pushupdate(med_buffer_net, accel_net);
      accel_net = sort_med(med_buffer_net);
    }
    
   Serial.print("Roll: ");
   Serial.print(accel_x);
   Serial.print("\tPitch: ");
   Serial.print(accel_y);
   Serial.print("\tYaw: ");
   Serial.print(accel_z);
   //Serial.print();
   //Serial.println();
  }
}
